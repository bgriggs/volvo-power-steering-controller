/**
 * @file test_online.cpp
 * @brief Tests for the pump / ECU online timeout logic.
 */
#include "ps_logic.h"
#include "test_framework.h"

TEST(offline_before_any_frame_is_received) {
  // A last-seen timestamp of 0 is the "never heard from" sentinel.
  EXPECT_FALSE(psIsOnline(0, 0));
  EXPECT_FALSE(psIsOnline(0, 500));
  EXPECT_FALSE(psIsOnline(0, 4000000000u));
}

TEST(online_immediately_after_a_frame) {
  EXPECT_TRUE(psIsOnline(5000, 5000));
}

TEST(online_up_to_but_not_including_the_timeout) {
  EXPECT_TRUE(psIsOnline(5000, 5000 + 999));
  EXPECT_FALSE(psIsOnline(5000, 5000 + PS_ONLINE_TIMEOUT_MS));
  EXPECT_FALSE(psIsOnline(5000, 5000 + 1001));
}

TEST(offline_long_after_the_last_frame) {
  EXPECT_FALSE(psIsOnline(5000, 60000));
}

TEST(online_survives_millis_rollover) {
  // millis() wraps every ~49.7 days. The unsigned subtraction must still give
  // the true elapsed time across the wrap.
  const uint32_t justBeforeWrap = 0xFFFFFF00u; // 256 ms before rollover
  EXPECT_TRUE(psIsOnline(justBeforeWrap, 0x00000000u)); // 256 ms elapsed
  EXPECT_TRUE(psIsOnline(justBeforeWrap, 0x000002E7u)); // 999 ms elapsed
  EXPECT_FALSE(psIsOnline(justBeforeWrap, 0x000002E8u)); // 1000 ms elapsed
}

TEST(a_stale_timestamp_from_the_previous_epoch_reads_as_offline) {
  // Heard from at the very start of the previous millis() epoch, now just past
  // the wrap: that is ~49.7 days, which must read as offline rather than as a
  // small negative interval.
  EXPECT_FALSE(psIsOnline(1, 0x00000000u));
}
