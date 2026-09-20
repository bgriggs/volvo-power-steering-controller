/**
 * @file test_duty_cycle.cpp
 * @brief Tests for decoding the Haltech duty cycle and mapping it to a pump speed.
 */
#include "ps_logic.h"
#include "test_framework.h"

// ---------------------------------------------------------------------------
// psDecodeHaltechDutyCycle: raw byte from IO Box A DPO 1 -> percent
// ---------------------------------------------------------------------------

TEST(haltech_decode_maps_zero_to_zero_percent) {
  EXPECT_NEAR(0.0, psDecodeHaltechDutyCycle(0x00), 1e-9);
}

TEST(haltech_decode_uses_point_four_percent_per_count) {
  EXPECT_NEAR(0.4, psDecodeHaltechDutyCycle(1), 1e-9);
  EXPECT_NEAR(10.0, psDecodeHaltechDutyCycle(25), 1e-9);
  EXPECT_NEAR(50.0, psDecodeHaltechDutyCycle(125), 1e-9);
  EXPECT_NEAR(80.0, psDecodeHaltechDutyCycle(200), 1e-9);
}

TEST(haltech_decode_reaches_full_scale_at_250) {
  EXPECT_NEAR(100.0, psDecodeHaltechDutyCycle(250), 1e-9);
}

TEST(haltech_decode_can_exceed_100_percent) {
  // Counts 251-255 decode above 100%. Current behavior: the decode does not
  // clamp, so the raw percentage is what reaches the status frame, while
  // psConvertDutyCycle clamps before commanding the pump.
  EXPECT_NEAR(100.4, psDecodeHaltechDutyCycle(251), 1e-9);
  EXPECT_NEAR(102.0, psDecodeHaltechDutyCycle(255), 1e-9);
}

// ---------------------------------------------------------------------------
// psConvertDutyCycle: percent -> pump speed value (inverted, 1 is full speed)
// ---------------------------------------------------------------------------

TEST(convert_maps_100_percent_to_full_speed) {
  EXPECT_EQ(1, psConvertDutyCycle(100.0));
}

TEST(convert_is_inverted_and_linear) {
  EXPECT_EQ(5940, psConvertDutyCycle(1.0));
  EXPECT_EQ(5400, psConvertDutyCycle(10.0));
  EXPECT_EQ(4800, psConvertDutyCycle(20.0));
  EXPECT_EQ(4500, psConvertDutyCycle(25.0));
  EXPECT_EQ(3600, psConvertDutyCycle(40.0));
  EXPECT_EQ(3000, psConvertDutyCycle(50.0));
  EXPECT_EQ(2400, psConvertDutyCycle(60.0));
  EXPECT_EQ(1500, psConvertDutyCycle(75.0));
  EXPECT_EQ(1200, psConvertDutyCycle(80.0));
  EXPECT_EQ(600, psConvertDutyCycle(90.0));
  EXPECT_EQ(60, psConvertDutyCycle(99.0));
}

TEST(convert_clamps_above_100_percent) {
  EXPECT_EQ(1, psConvertDutyCycle(100.0001));
  EXPECT_EQ(1, psConvertDutyCycle(102.0)); // the highest a Haltech byte can decode to
  EXPECT_EQ(1, psConvertDutyCycle(1000.0));
}

TEST(convert_clamps_below_zero_percent) {
  EXPECT_EQ(0, psConvertDutyCycle(-0.0001));
  EXPECT_EQ(0, psConvertDutyCycle(-10.0));
}

TEST(convert_maps_exactly_zero_percent_to_zero) {
  EXPECT_EQ(0, psConvertDutyCycle(0.0));
}

TEST(convert_jumps_from_zero_to_5999_just_above_zero_percent) {
  // Current behavior, worth knowing about: the mapping is discontinuous at 0.
  // Exactly 0% is special cased to 0, but the smallest duty cycle above 0%
  // jumps straight to 5999, the slowest speed the mapping can produce. From
  // there it is continuous again. So a duty cycle creeping off zero swings the
  // pump command across the whole range in one step.
  EXPECT_EQ(5999, psConvertDutyCycle(0.0000001));
  EXPECT_EQ(5999, psConvertDutyCycle(0.01));
  EXPECT_EQ(5994, psConvertDutyCycle(0.1));
}

TEST(convert_never_returns_the_documented_maximum_of_6000) {
  // Current behavior: the slowest speed the mapping can produce is 5999, so
  // the documented 6000 end of the range is unreachable.
  for (int tenths = 0; tenths <= 1000; tenths++) {
    uint16_t speed = psConvertDutyCycle(tenths / 10.0);
    EXPECT_TRUE(speed != PS_PUMP_SPEED_MAX);
  }
}

TEST(convert_output_stays_within_the_pump_range) {
  for (int tenths = 0; tenths <= 1020; tenths++) {
    uint16_t speed = psConvertDutyCycle(tenths / 10.0);
    EXPECT_TRUE(speed <= PS_PUMP_SPEED_MAX);
  }
}

TEST(convert_is_monotonically_non_increasing_above_zero) {
  // More duty cycle must never mean a slower pump.
  uint16_t previous = psConvertDutyCycle(0.1);
  for (int tenths = 2; tenths <= 1000; tenths++) {
    uint16_t speed = psConvertDutyCycle(tenths / 10.0);
    EXPECT_TRUE(speed <= previous);
    previous = speed;
  }
}

// ---------------------------------------------------------------------------
// Decode and convert together, as the sketch chains them.
// ---------------------------------------------------------------------------

TEST(haltech_byte_converts_end_to_end_to_a_pump_speed) {
  EXPECT_EQ(0, psConvertDutyCycle(psDecodeHaltechDutyCycle(0)));
  EXPECT_EQ(5400, psConvertDutyCycle(psDecodeHaltechDutyCycle(25)));  // 10%
  EXPECT_EQ(3600, psConvertDutyCycle(psDecodeHaltechDutyCycle(100))); // 40%
  EXPECT_EQ(1200, psConvertDutyCycle(psDecodeHaltechDutyCycle(200))); // 80%
  EXPECT_EQ(1, psConvertDutyCycle(psDecodeHaltechDutyCycle(250)));    // 100%
  EXPECT_EQ(1, psConvertDutyCycle(psDecodeHaltechDutyCycle(255)));    // 102%, clamped
}
