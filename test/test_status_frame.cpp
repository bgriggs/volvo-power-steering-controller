/**
 * @file test_status_frame.cpp
 * @brief Tests for the controller status frame sent to the dash on CAN 2.
 *
 * Layout documented in README.md, big endian:
 *   byte 0    controller status
 *   bytes 1-2 duty cycle percent, factor 10
 *   bytes 3-4 pump speed value
 */
#include "ps_logic.h"
#include "test_framework.h"

// ---------------------------------------------------------------------------
// psStatusCode
// ---------------------------------------------------------------------------

TEST(status_code_is_online_when_both_peers_are_up) {
  EXPECT_EQ(PS_STATUS_ONLINE, psStatusCode(true, true));
}

TEST(status_code_reports_the_ecu_as_the_root_cause) {
  // With the ECU offline the keep alive is stopped deliberately, so the pump
  // falling silent afterwards is a consequence rather than a second fault.
  // Both peers down therefore reports as ECU offline instead of escalating.
  EXPECT_EQ(PS_STATUS_ECU_OFFLINE, psStatusCode(false, false));
  EXPECT_EQ(PS_STATUS_ECU_OFFLINE, psStatusCode(true, false));
}

TEST(status_code_never_reports_pump_and_ecu_offline) {
  // Kept in the enum because the protocol documents it, but no longer emitted.
  EXPECT_TRUE(psStatusCode(false, false) != PS_STATUS_PUMP_AND_ECU_OFFLINE);
  EXPECT_TRUE(psStatusCode(false, true) != PS_STATUS_PUMP_AND_ECU_OFFLINE);
  EXPECT_TRUE(psStatusCode(true, false) != PS_STATUS_PUMP_AND_ECU_OFFLINE);
  EXPECT_TRUE(psStatusCode(true, true) != PS_STATUS_PUMP_AND_ECU_OFFLINE);
}

TEST(status_code_reports_pump_offline) {
  EXPECT_EQ(PS_STATUS_PUMP_OFFLINE, psStatusCode(false, true));
}

TEST(status_code_reports_ecu_offline) {
  EXPECT_EQ(PS_STATUS_ECU_OFFLINE, psStatusCode(true, false));
}

TEST(status_code_covers_every_combination) {
  // The four cases above are exhaustive, so the 0 "unknown" code is never
  // reported. This pins that down.
  EXPECT_TRUE(psStatusCode(false, false) != PS_STATUS_UNKNOWN);
  EXPECT_TRUE(psStatusCode(false, true) != PS_STATUS_UNKNOWN);
  EXPECT_TRUE(psStatusCode(true, false) != PS_STATUS_UNKNOWN);
  EXPECT_TRUE(psStatusCode(true, true) != PS_STATUS_UNKNOWN);
}

// ---------------------------------------------------------------------------
// psBuildStatusFrame
// ---------------------------------------------------------------------------

TEST(status_frame_packs_status_duty_and_speed) {
  uint8_t frame[PS_STATUS_FRAME_LEN];
  psBuildStatusFrame(frame, true, true, 80.0, 1200);

  // 80.0% * 10 = 800 = 0x0320, speed 1200 = 0x04B0
  EXPECT_BYTES_EQ(frame, { 0x01, 0x03, 0x20, 0x04, 0xB0 });
}

TEST(status_frame_encodes_duty_cycle_big_endian_with_factor_10) {
  uint8_t frame[PS_STATUS_FRAME_LEN];

  psBuildStatusFrame(frame, true, true, 0.0, 0);
  EXPECT_EQ(0x00, frame[1]);
  EXPECT_EQ(0x00, frame[2]);

  psBuildStatusFrame(frame, true, true, 40.0, 0);
  EXPECT_EQ(0x01, frame[1]); // 400 = 0x0190
  EXPECT_EQ(0x90, frame[2]);

  psBuildStatusFrame(frame, true, true, 100.0, 0);
  EXPECT_EQ(0x03, frame[1]); // 1000 = 0x03E8
  EXPECT_EQ(0xE8, frame[2]);
}

TEST(status_frame_clamps_the_reported_duty_cycle) {
  // The frame reports what the pump was actually commanded, so a Haltech
  // byte of 0xFF (102.0%) is reported as 100.0%. This keeps the reported
  // percent and the reported pump value from disagreeing.
  uint8_t frame[PS_STATUS_FRAME_LEN];

  psBuildStatusFrame(frame, true, true, 102.0, 1);
  EXPECT_EQ(0x03, frame[1]); // 1000 = 0x03E8, not 1020
  EXPECT_EQ(0xE8, frame[2]);

  psBuildStatusFrame(frame, true, true, -5.0, 0);
  EXPECT_EQ(0x00, frame[1]);
  EXPECT_EQ(0x00, frame[2]);
}

TEST(status_frame_truncates_fractional_duty_cycle_tenths) {
  uint8_t frame[PS_STATUS_FRAME_LEN];
  psBuildStatusFrame(frame, true, true, 12.34, 0);
  EXPECT_EQ(0x00, frame[1]); // 123.4 truncates to 123 = 0x007B
  EXPECT_EQ(0x7B, frame[2]);
}

TEST(status_frame_encodes_pump_speed_big_endian) {
  uint8_t frame[PS_STATUS_FRAME_LEN];

  psBuildStatusFrame(frame, true, true, 0.0, 0);
  EXPECT_EQ(0x00, frame[3]);
  EXPECT_EQ(0x00, frame[4]);

  psBuildStatusFrame(frame, true, true, 0.0, 1);
  EXPECT_EQ(0x00, frame[3]);
  EXPECT_EQ(0x01, frame[4]);

  psBuildStatusFrame(frame, true, true, 0.0, 6000);
  EXPECT_EQ(0x17, frame[3]); // 6000 = 0x1770
  EXPECT_EQ(0x70, frame[4]);

  psBuildStatusFrame(frame, true, true, 0.0, 0xFFFF);
  EXPECT_EQ(0xFF, frame[3]);
  EXPECT_EQ(0xFF, frame[4]);
}

TEST(status_frame_carries_the_offline_status_byte) {
  uint8_t frame[PS_STATUS_FRAME_LEN];

  psBuildStatusFrame(frame, false, false, 0.0, 0);
  EXPECT_EQ(PS_STATUS_ECU_OFFLINE, frame[0]);

  psBuildStatusFrame(frame, false, true, 0.0, 0);
  EXPECT_EQ(PS_STATUS_PUMP_OFFLINE, frame[0]);

  psBuildStatusFrame(frame, true, false, 0.0, 0);
  EXPECT_EQ(PS_STATUS_ECU_OFFLINE, frame[0]);
}

TEST(status_frame_duty_field_matches_every_haltech_byte) {
  // Sweeps the whole reachable input domain: the encoded tenths must equal
  // min(raw * 4, 1000) for every byte the Haltech can send, with no float
  // truncation drift anywhere in the decode -> clamp -> x10 chain.
  uint8_t frame[PS_STATUS_FRAME_LEN];
  for (int raw = 0; raw <= 255; raw++) {
    psBuildStatusFrame(frame, true, true,
                       psDecodeHaltechDutyCycle((uint8_t)raw), 0);
    uint16_t encoded = (uint16_t)((frame[1] << 8) | frame[2]);
    uint16_t expected = (raw * 4 < 1000) ? (uint16_t)(raw * 4) : (uint16_t)1000;
    EXPECT_EQ(expected, encoded);
  }
}

TEST(status_frame_reads_zero_while_not_commanding_the_pump) {
  // What the dash sees during a fault. The sketch passes 0 for the duty cycle
  // and zeroes the pump speed before the send, so neither field shows a stale
  // live value alongside the fault code.
  uint8_t frame[PS_STATUS_FRAME_LEN];
  psBuildStatusFrame(frame, true, false, 0.0, 0);
  EXPECT_BYTES_EQ(frame, { PS_STATUS_ECU_OFFLINE, 0x00, 0x00, 0x00, 0x00 });
}

TEST(status_frame_writes_every_byte_it_owns) {
  // Guards against a partially filled frame leaking stale bytes onto the bus.
  uint8_t frame[PS_STATUS_FRAME_LEN];
  memset(frame, 0xAA, sizeof(frame));
  psBuildStatusFrame(frame, true, true, 50.0, 3000);
  EXPECT_BYTES_EQ(frame, { 0x01, 0x01, 0xF4, 0x0B, 0xB8 });
}
