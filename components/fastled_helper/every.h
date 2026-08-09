#pragma once

#include "esphome/core/defines.h"

namespace esphome::fastled_helper {

/**
 * @class CEveryNMillis
 * @brief Original timekeeping class from the FastLED library (lib8tion).
 * It tracks time intervals using Arduino's native millis() function.
 */
class CEveryNMillis {
 public:
  uint32_t mPrevTrigger;  // Timestamp of the last successful trigger
  uint32_t mPeriod;       // Timing interval in milliseconds

  /**
   * @brief Constructor to initialize the timer period.
   * @param period Timing interval in milliseconds.
   */
  CEveryNMillis(uint32_t period) {
    mPeriod = period;
    reset();
  }

  /**
   * @brief Dynamically changes the timer period on the fly.
   * @param period New timing interval in milliseconds.
   */
  void setPeriod(uint32_t period) { mPeriod = period; }

  /**
   * @brief Checks if the required time period has elapsed.
   * @return true if the interval has passed, false otherwise.
   */
  bool ready() {
    uint32_t now = ::millis();
    if (now - mPrevTrigger >= mPeriod) {
      reset();  // Auto-reset the trigger point for the next cycle
      return true;
    }
    return false;
  }

  /**
   * @brief Resets the previous trigger timestamp to the current time.
   */
  void reset() { mPrevTrigger = ::millis(); }

  /**
   * @brief Prepares the timer to trigger instantly on its very next evaluation.
   */
  void trigger() { mPrevTrigger = ::millis() - mPeriod; }

  /**
   * @brief Overloads the bool operator to allow clean 'if(timer)' syntax.
   */
  operator bool() { return ready(); }
};

/**
 * @brief Token-pasting macros from FastLED used to safely concatenate text.
 * Double-wrapping ensures that macro arguments (like __LINE__) are evaluated
 * as integers before being stitched into variable names.
 */
#define CONCAT_HELPER(x, y) x##y
#define CONCAT_MACRO(x, y) CONCAT_HELPER(x, y)

// ============================================================================
// MILLISECOND MACROS
// ============================================================================

/**
 * @brief Indexed millisecond macro. Allows manual timer naming and period updates.
 * @param NAME Explicit name for the static CEveryNMillis instance.
 * @param N Timing period (can be a variable or slider value from ESPHome).
 */
#define EVERY_N_MILLIS_I(NAME, N) \
  static CEveryNMillis NAME(N); \
  if (NAME)

/**
 * @brief Static millisecond macro. Automatically generates a unique name based on the code line.
 * @param N Static timing period in milliseconds.
 */
#define EVERY_N_MILLIS(N) EVERY_N_MILLIS_I(CONCAT_MACRO(ev_m_, __LINE__), N)

// ============================================================================
// SECOND MACROS
// ============================================================================

/**
 * @brief Indexed second macro. Allows manual timer naming and period updates in seconds.
 * @param NAME Explicit name for the static CEveryNMillis instance.
 * @param N Timing period in seconds (multiplied by 1000UL at compile time).
 */
#define EVERY_N_SECONDS_I(NAME, N) \
  static CEveryNMillis NAME((N) * 1000UL); \
  if (NAME)

/**
 * @brief Static second macro. Automatically generates a unique name based on the code line.
 * @param N Static timing period in seconds.
 */
#define EVERY_N_SECONDS(N) EVERY_N_SECONDS_I(CONCAT_MACRO(ev_s_, __LINE__), N)

}  // namespace esphome::fastled_helper
