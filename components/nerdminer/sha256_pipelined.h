#pragma once

#include <stdbool.h>
#include <stdint.h>

#if defined(CONFIG_IDF_TARGET_ESP32)

namespace esphome::nerdminer {

/**
 * Initialize hardware SHA peripheral clock and clear reset state.
 */
void sha256_pipelined_init();

/**
 * Run pipelined Xtensa SHA-256 double-hash loop on classic ESP32.
 *
 * Overlaps loading block 2 into SHA_TEXT while the hardware engine processes
 * block 1. Exits early when the upper 16 bits of H0 are zero (candidate share),
 * when max_iterations is reached, or when mining_flag becomes false.
 *
 * Note: When returning true, the candidate nonce is (*nonce_swapped - 1).
 *
 * @param header_swapped Pre-byteswapped 80-byte block header (20 x uint32_t).
 * @param nonce_swapped  In/out big-endian nonce value.
 * @param max_iterations Maximum hashes to compute before returning to caller.
 * @param hashes_done    Output count of double-hashes completed in this call.
 * @param mining_flag    Pointer to active flag; loop aborts if false.
 * @return true if a 16-bit early-reject candidate was found, false otherwise.
 */
bool sha256_pipelined_mine(const uint32_t *header_swapped, uint32_t *nonce_swapped, uint32_t max_iterations,
                           uint32_t *hashes_done, const volatile bool *mining_flag);

}  // namespace esphome::nerdminer

#endif  // CONFIG_IDF_TARGET_ESP32
