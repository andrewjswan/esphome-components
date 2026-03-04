#pragma once

/*
 * SparkMiner - Pipelined SHA-256 Assembly Implementation
 *
 * Ported from BitsyMiner by Justin Williams (GPL v3)
 *
 * Uses inline Xtensa assembly to pipeline SHA operations:
 * - While hardware processes block N, CPU loads block N+1
 * - Achieves ~600+ KH/s vs ~300 KH/s sequential
 *
 * Only for standard ESP32 (Xtensa LX6) - CYD boards
 */

#include <stdint.h>
#include <stdbool.h>

// Only available on standard ESP32 (not S3/C3)
#if defined(CONFIG_IDF_TARGET_ESP32)

namespace esphome {
namespace nerdminer {

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize hardware SHA peripheral for pipelined mining.
 * Enables SHA clock and clears reset.
 */
void sha256_pipelined_init(void);

/**
 * Pipelined mining loop using inline Xtensa assembly.
 *
 * This is the hot path for Core 1 - runs until:
 * - mining_flag becomes false, OR
 * - 16-bit early reject passes (potential share found)
 *
 * The assembly pipelines SHA operations:
 * 1. Load block 1 (64 bytes) -> Start SHA
 * 2. While SHA processes, load block 2 (16 bytes + nonce + padding)
 * 3. Continue SHA with block 2
 * 4. While SHA processes, prepare double-hash padding
 * 5. Load result -> Start second SHA
 * 6. Check 16-bit early reject
 * 7. If fails, increment nonce and loop
 *
 * @param sha_base       SHA_TEXT_BASE register address (0x3FF03000)
 * @param header_swapped Pre-byteswapped 80-byte header (20 x uint32_t)
 * @param nonce_ptr      Pointer to nonce value (updated in-place)
 * @param hash_count_ptr Pointer to 64-bit hash counter (incremented per hash)
 * @param mining_flag    Pointer to mining active flag (exits when false)
 *
 * @return true if potential share found (16-bit early reject passed)
 *         false if stopped due to mining_flag becoming false
 */
bool IRAM_ATTR sha256_pipelined_mine(volatile uint32_t *sha_base, const uint32_t *header_swapped, uint32_t *nonce_ptr,
                                     volatile uint64_t *hash_count_ptr, volatile bool *mining_flag);

/**
 * Optimized pipelined mining v2 with unrolled zeros.
 *
 * Optimizations over v1:
 * - Unrolled zero loop (saves ~20 cycles per iteration)
 * - Persistent zero register
 * - Same functionality, slightly faster
 *
 * Note: ESP32 doesn't support midstate caching (no writable state registers)
 * so block 1 must still be reloaded every iteration.
 */
bool IRAM_ATTR sha256_pipelined_mine_v2(volatile uint32_t *sha_base, const uint32_t *header_swapped,
                                        uint32_t *nonce_ptr, volatile uint64_t *hash_count_ptr,
                                        volatile bool *mining_flag);

/**
 * Ultra-optimized mining loop v3 with batched copies.
 * Pipelined mining v3 - equivalent to v2.
 * Register caching was attempted but not achievable due to Xtensa constraints.
 * The SHA hardware is the bottleneck (~715 KH/s on CYD), not CPU.
 */
bool IRAM_ATTR sha256_pipelined_mine_s3_v3(const uint32_t *midstate, const uint32_t *block2_words, uint32_t *nonce_ptr,
                                           volatile uint64_t *hash_count_ptr, volatile bool *mining_flag);

#ifdef __cplusplus
}
#endif

}  // namespace nerdminer
}  // namespace esphome

#endif  // CONFIG_IDF_TARGET_ESP32
