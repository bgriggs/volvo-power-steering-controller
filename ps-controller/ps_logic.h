/**
 * @file ps_logic.h
 * @brief Pure controller logic, free of Arduino/ESP-IDF dependencies.
 *
 * Everything here is deterministic and side-effect free so it can be compiled
 * and unit tested on a host machine. The sketch (ps-controller.ino) owns all
 * the I/O: CAN transmit/receive, Serial, GPIO and millis().
 *
 * Timestamps are uint32_t rather than `unsigned long` on purpose. On the ESP32
 * `unsigned long` is 32 bits and that is what millis() returns, but on a 64-bit
 * host it is 64 bits. Pinning the width keeps rollover arithmetic identical
 * between the target and the tests.
 */
#ifndef PS_LOGIC_H
#define PS_LOGIC_H

#include <stdint.h>
#include <string.h>

/** Controller status codes reported in byte 0 of the status frame. */
enum PsStatus {
  PS_STATUS_UNKNOWN = 0,
  PS_STATUS_ONLINE = 1,
  PS_STATUS_PUMP_AND_ECU_OFFLINE = 2,
  PS_STATUS_PUMP_OFFLINE = 3,
  PS_STATUS_ECU_OFFLINE = 4
};

/** A peer is considered offline once this long has passed without a frame. */
static const uint32_t PS_ONLINE_TIMEOUT_MS = 1000;

/** CAN identifiers. */
static const uint32_t PS_CAN_ID_PUMP_HEARTBEAT = 0x1B200002; // CAN 1, from pump
static const uint32_t PS_CAN_ID_PUMP_KEEP_ALIVE = 0x1AE0092C; // CAN 1, to pump
static const uint32_t PS_CAN_ID_PUMP_SPEED = 0x02104136;     // CAN 1, to pump
static const uint32_t PS_CAN_ID_HALTECH_DPO1 = 0x2D0;        // CAN 2, from Haltech IO Box A
static const uint32_t PS_CAN_ID_STATUS = 0x100D0001;         // CAN 2, to dash

/** Frame sizes. */
static const uint8_t PS_STATUS_FRAME_LEN = 5;
static const uint8_t PS_PUMP_FRAME_LEN = 8;

/** Pump speed command range. 1 is full speed, larger values are slower. */
static const uint16_t PS_PUMP_SPEED_MIN = 1;
static const uint16_t PS_PUMP_SPEED_MAX = 6000;

/** Base keep alive frame. Byte 0 is replaced with the rotating counter value. */
static const uint8_t PS_KEEP_ALIVE_FRAME[PS_PUMP_FRAME_LEN] = {
  0x00, 0x00, 0x22, 0xE0, 0x41, 0x90, 0x00, 0x00
};

/** The keep alive counter cycles through these four values. */
static const uint8_t PS_KEEP_ALIVE_COUNTER_VALUES[4] = { 0x00, 0x40, 0x80, 0xC0 };
static const uint8_t PS_KEEP_ALIVE_COUNTER_COUNT = 4;

/** Base speed frame. Bytes 6 and 7 are replaced with the speed, big endian. */
static const uint8_t PS_SPEED_FRAME[PS_PUMP_FRAME_LEN] = {
  0xBB, 0x00, 0x3F, 0xFF, 0x06, 0xE0, 0x00, 0x00
};

/**
 * @brief  Whether a peer counts as online given when it was last heard from.
 *
 * A lastRxTs of 0 means "never heard from". The unsigned subtraction is
 * deliberate: it stays correct across the ~49.7 day millis() rollover.
 */
inline bool psIsOnline(uint32_t lastRxTs, uint32_t nowTs) {
  if (lastRxTs == 0) return false;
  return (uint32_t)(nowTs - lastRxTs) < PS_ONLINE_TIMEOUT_MS;
}

/**
 * @brief  Decodes the duty cycle percentage from byte 0 of Haltech IO Box A DPO 1.
 */
inline double psDecodeHaltechDutyCycle(uint8_t raw) {
  return ((double)raw) / 2.5;
}

/**
 * @brief  Gets the pump value from duty cycle percentage where 1 is full on.
 */
inline uint16_t psConvertDutyCycle(double dutyCycle) {
  if (dutyCycle > 100.0) dutyCycle = 100.0; // clamp to max
  if (dutyCycle < 0.0) dutyCycle = 0.0;     // clamp to min

  if (dutyCycle == 0.0)
    return 0;

  // Map 0-100% -> 1-6000, inverted
  uint16_t value = 1 + (uint16_t)((6000 - 1) * (1.0 - dutyCycle / 100.0));
  return value;
}

/**
 * @brief  Maps peer online state onto the status code reported to the dash.
 */
inline uint8_t psStatusCode(bool isPumpOnline, bool isHaltechOnline) {
  if (isPumpOnline && isHaltechOnline)
    return PS_STATUS_ONLINE;
  else if (!isPumpOnline && !isHaltechOnline)
    return PS_STATUS_PUMP_AND_ECU_OFFLINE;
  else if (!isPumpOnline)
    return PS_STATUS_PUMP_OFFLINE;
  else if (!isHaltechOnline)
    return PS_STATUS_ECU_OFFLINE;
  else
    return PS_STATUS_UNKNOWN;
}

/**
 * @brief  Packs the 5 byte controller status frame, big endian.
 *
 * @param  out  Receives PS_STATUS_FRAME_LEN bytes.
 */
inline void psBuildStatusFrame(uint8_t out[PS_STATUS_FRAME_LEN],
                               bool isPumpOnline,
                               bool isHaltechOnline,
                               double dutyCycle,
                               uint16_t pumpSpeed) {
  out[0] = psStatusCode(isPumpOnline, isHaltechOnline);

  // Duty Cycle
  uint16_t value = (uint16_t)(dutyCycle * 10);
  out[1] = (uint8_t)((value & 0xFF00) >> 8);
  out[2] = (uint8_t)((value & 0x00FF));

  // Pump value
  out[3] = (uint8_t)((pumpSpeed & 0xFF00) >> 8);
  out[4] = (uint8_t)((pumpSpeed & 0x00FF));
}

/**
 * @brief  Advances the rotating keep alive counter index, wrapping at 4.
 */
inline uint8_t psNextKeepAliveIndex(uint8_t index) {
  index++;
  if (index > PS_KEEP_ALIVE_COUNTER_COUNT - 1)
    index = 0;
  return index;
}

/**
 * @brief  Packs the 8 byte pump keep alive frame for the given counter index.
 *
 * @param  out  Receives PS_PUMP_FRAME_LEN bytes.
 */
inline void psBuildKeepAliveFrame(uint8_t out[PS_PUMP_FRAME_LEN], uint8_t counterIndex) {
  memcpy(out, PS_KEEP_ALIVE_FRAME, PS_PUMP_FRAME_LEN);
  out[0] = PS_KEEP_ALIVE_COUNTER_VALUES[counterIndex];
}

/**
 * @brief  Packs the 8 byte pump speed frame, speed big endian in bytes 6 and 7.
 *
 * @param  out  Receives PS_PUMP_FRAME_LEN bytes.
 */
inline void psBuildSpeedFrame(uint8_t out[PS_PUMP_FRAME_LEN], uint16_t speed) {
  memcpy(out, PS_SPEED_FRAME, PS_PUMP_FRAME_LEN);
  out[6] = (uint8_t)((speed & 0xFF00) >> 8);
  out[7] = (uint8_t)((speed & 0x00FF));
}

#endif // PS_LOGIC_H
