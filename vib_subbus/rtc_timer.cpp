/* rtc_timer.c */
#include <Arduino.h>
#include "rtc_timer.h"

uint32_t rtc_current_count;
#ifdef RTC_USE_MAX_DURATION_REFERENCE
uint16_t rtc_max_state_duration_ref_value;
#endif
static bool rtc_current_count_set;

static subbus_cache_word_t rtc_cache[RTC_HIGH_ADDR-RTC_BASE_ADDR+1] = {
  { 0, 0, true, false, false, false, false },
  { 0, 0, true, false, false, false, false },
  { 0, 0, true, false, false, false, false },
  { 0, 0, true, false, false, false, true }
  #ifdef RTC_USE_MAX_DURATION_REFERENCE
  , { 0, 0, true, false, false, false, false }
  #endif
};

static void rtc_reset() {
  rtc_current_count = micros();
}

/**
 * Reads the RTC counter and saves it to offset 0 and 1 and also saves
 * it in rtc_current_count;
 * Calculates the elapsed time since the previous poll and stores the
 * difference in offset 2.
 * Checks to see if this is a new maximum, and if so, stores it in offset 3.
 */
static void rtc_poll() {
  uint32_t cur_time = micros();
  sb_cache_update32(rtc_cache,RTC_ELAPSED_OFFSET,&cur_time);
  if (rtc_current_count_set) {
    uint16_t dt = cur_time - rtc_current_count;
    sb_cache_update(rtc_cache,RTC_CUR_STATE_DURATION_OFFSET,dt);
    if (dt > rtc_cache[RTC_MAX_STATE_DURATION_OFFSET].cache) {
      sb_cache_update(rtc_cache,RTC_MAX_STATE_DURATION_OFFSET,dt);
      #ifdef RTC_USE_MAX_DURATION_REFERENCE
      sb_cache_update(rtc_cache,RTC_MAX_DURATION_REF_OFFSET,rtc_max_state_duration_ref_value);
      #endif
    }
  } else {
    rtc_current_count_set = true;
  }
  rtc_current_count = cur_time;
}

static void rtc_action(uint16_t offset) {
  if (offset == RTC_MAX_STATE_DURATION_OFFSET)
    sb_cache_update(rtc_cache,RTC_MAX_STATE_DURATION_OFFSET,0);
}

subbus_driver_t sb_rtc = {
  RTC_BASE_ADDR, RTC_HIGH_ADDR, // address range
  rtc_cache,
  rtc_reset,
  rtc_poll,
  rtc_action,
  false
};
