/**
 * @file test_pump_frames.cpp
 * @brief Tests for the two frames sent to the power steering pump on CAN 1:
 *        the rotating keep alive and the speed command.
 */
#include "ps_logic.h"
#include "test_framework.h"

// ---------------------------------------------------------------------------
// psNextKeepAliveIndex
// ---------------------------------------------------------------------------

TEST(keep_alive_index_advances_then_wraps_at_four) {
  EXPECT_EQ(1, psNextKeepAliveIndex(0));
  EXPECT_EQ(2, psNextKeepAliveIndex(1));
  EXPECT_EQ(3, psNextKeepAliveIndex(2));
  EXPECT_EQ(0, psNextKeepAliveIndex(3));
}

TEST(keep_alive_index_stays_in_range_over_many_cycles) {
  uint8_t index = 0;
  for (int i = 0; i < 1000; i++) {
    index = psNextKeepAliveIndex(index);
    EXPECT_TRUE(index < PS_KEEP_ALIVE_COUNTER_COUNT);
  }
}

TEST(keep_alive_index_completes_a_full_cycle_every_four_steps) {
  uint8_t index = 0;
  for (int i = 0; i < 4; i++) {
    index = psNextKeepAliveIndex(index);
  }
  EXPECT_EQ(0, index);
}

// ---------------------------------------------------------------------------
// psBuildKeepAliveFrame
// ---------------------------------------------------------------------------

TEST(keep_alive_frame_carries_the_counter_in_byte_zero) {
  uint8_t frame[PS_PUMP_FRAME_LEN];

  psBuildKeepAliveFrame(frame, 0);
  EXPECT_BYTES_EQ(frame, { 0x00, 0x00, 0x22, 0xE0, 0x41, 0x90, 0x00, 0x00 });

  psBuildKeepAliveFrame(frame, 1);
  EXPECT_BYTES_EQ(frame, { 0x40, 0x00, 0x22, 0xE0, 0x41, 0x90, 0x00, 0x00 });

  psBuildKeepAliveFrame(frame, 2);
  EXPECT_BYTES_EQ(frame, { 0x80, 0x00, 0x22, 0xE0, 0x41, 0x90, 0x00, 0x00 });

  psBuildKeepAliveFrame(frame, 3);
  EXPECT_BYTES_EQ(frame, { 0xC0, 0x00, 0x22, 0xE0, 0x41, 0x90, 0x00, 0x00 });
}

TEST(keep_alive_frame_leaves_bytes_one_through_seven_constant) {
  uint8_t frame[PS_PUMP_FRAME_LEN];
  for (uint8_t index = 0; index < PS_KEEP_ALIVE_COUNTER_COUNT; index++) {
    memset(frame, 0xAA, sizeof(frame));
    psBuildKeepAliveFrame(frame, index);
    for (size_t byte = 1; byte < PS_PUMP_FRAME_LEN; byte++) {
      EXPECT_EQ(PS_KEEP_ALIVE_FRAME[byte], frame[byte]);
    }
  }
}

TEST(keep_alive_sequence_repeats_00_40_80_C0) {
  // What the pump actually sees over eight consecutive keep alives, driven the
  // same way the sketch drives it: build with the current index, then advance.
  const uint8_t expected[8] = { 0x00, 0x40, 0x80, 0xC0, 0x00, 0x40, 0x80, 0xC0 };
  uint8_t frame[PS_PUMP_FRAME_LEN];
  uint8_t index = 0;

  for (int i = 0; i < 8; i++) {
    psBuildKeepAliveFrame(frame, index);
    EXPECT_EQ(expected[i], frame[0]);
    index = psNextKeepAliveIndex(index);
  }
}

// ---------------------------------------------------------------------------
// psBuildSpeedFrame
// ---------------------------------------------------------------------------

TEST(speed_frame_packs_speed_big_endian_into_bytes_six_and_seven) {
  uint8_t frame[PS_PUMP_FRAME_LEN];

  psBuildSpeedFrame(frame, 1200); // 0x04B0
  EXPECT_BYTES_EQ(frame, { 0xBB, 0x00, 0x3F, 0xFF, 0x06, 0xE0, 0x04, 0xB0 });
}

TEST(speed_frame_handles_the_ends_of_the_range) {
  uint8_t frame[PS_PUMP_FRAME_LEN];

  psBuildSpeedFrame(frame, 0);
  EXPECT_EQ(0x00, frame[6]);
  EXPECT_EQ(0x00, frame[7]);

  psBuildSpeedFrame(frame, PS_PUMP_SPEED_MIN); // 1, full speed
  EXPECT_EQ(0x00, frame[6]);
  EXPECT_EQ(0x01, frame[7]);

  psBuildSpeedFrame(frame, PS_PUMP_SPEED_MAX); // 6000 = 0x1770
  EXPECT_EQ(0x17, frame[6]);
  EXPECT_EQ(0x70, frame[7]);

  psBuildSpeedFrame(frame, 0xFFFF);
  EXPECT_EQ(0xFF, frame[6]);
  EXPECT_EQ(0xFF, frame[7]);
}

TEST(speed_frame_leaves_bytes_zero_through_five_constant) {
  uint8_t frame[PS_PUMP_FRAME_LEN];
  const uint16_t speeds[] = { 0, 1, 255, 256, 1200, 3000, 6000, 0xFFFF };

  for (uint16_t speed : speeds) {
    memset(frame, 0xAA, sizeof(frame));
    psBuildSpeedFrame(frame, speed);
    for (size_t byte = 0; byte < 6; byte++) {
      EXPECT_EQ(PS_SPEED_FRAME[byte], frame[byte]);
    }
  }
}

TEST(speed_frame_round_trips_every_speed_in_the_pump_range) {
  uint8_t frame[PS_PUMP_FRAME_LEN];
  for (uint16_t speed = 0; speed <= PS_PUMP_SPEED_MAX; speed++) {
    psBuildSpeedFrame(frame, speed);
    uint16_t decoded = (uint16_t)((frame[6] << 8) | frame[7]);
    EXPECT_EQ(speed, decoded);
  }
}

// ---------------------------------------------------------------------------
// The full duty cycle -> frame path the sketch runs every 72 ms.
// ---------------------------------------------------------------------------

TEST(a_haltech_byte_becomes_the_expected_speed_frame) {
  uint8_t frame[PS_PUMP_FRAME_LEN];

  // 0xC8 = 200 counts = 80% duty = speed 1200 = 0x04B0
  psBuildSpeedFrame(frame, psConvertDutyCycle(psDecodeHaltechDutyCycle(0xC8)));
  EXPECT_BYTES_EQ(frame, { 0xBB, 0x00, 0x3F, 0xFF, 0x06, 0xE0, 0x04, 0xB0 });

  // 0xFA = 250 counts = 100% duty = speed 1, full speed
  psBuildSpeedFrame(frame, psConvertDutyCycle(psDecodeHaltechDutyCycle(0xFA)));
  EXPECT_BYTES_EQ(frame, { 0xBB, 0x00, 0x3F, 0xFF, 0x06, 0xE0, 0x00, 0x01 });

  // 0x00 = 0 counts = 0% duty = speed 0
  psBuildSpeedFrame(frame, psConvertDutyCycle(psDecodeHaltechDutyCycle(0x00)));
  EXPECT_BYTES_EQ(frame, { 0xBB, 0x00, 0x3F, 0xFF, 0x06, 0xE0, 0x00, 0x00 });
}
