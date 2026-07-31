#pragma once

#include <cstdint>

namespace esphome::music_leds::debug {

// Master timeline registry visible across all translation units linking debug.h
inline uint32_t last_task_log = 0;

/**
 * @brief Checks if the uniform 2000ms logging interval has expired.
 * @return True exactly once every 2000ms during the active frame checkpoint.
 */
inline bool should_log() { return (millis() - last_task_log >= 2000); }

/**
 * @brief Commits the master time token forward once all logs for the current frame are written.
 */
inline void commit_timestamp() { last_task_log = millis(); }

}  // namespace esphome::music_leds::debug
