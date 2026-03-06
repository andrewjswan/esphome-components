#pragma once

/*
 * SparkMiner - Pipelined SHA-256 Assembly for ESP32-S3
 */

#include <stdint.h>
#include <stdbool.h>

#if defined(CONFIG_IDF_TARGET_ESP32S3)

namespace esphome {
namespace nerdminer {

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize S3 pipelined SHA hardware
 */
void sha256_pipelined_s3_init(void);

/**
 * Pipelined mining loop using optimized Xtensa LX7 assembly for ESP32-S3.
 * (v1 - recomputes block 1 each iteration)
 *
 * @param header_swapped Pre-byteswapped 80-byte header (20 x uint32_t)
 * @param nonce_ptr      Pointer to nonce in BE format (updated on return)
 * @param hash_count_ptr Pointer to hash counter
 * @param mining_flag    Pointer to mining active flag
 *
 * @return true if potential share found (16-bit early reject passed)
 */
bool sha256_pipelined_mine_s3(const uint32_t *header_swapped, uint32_t *nonce_ptr,
                              volatile uint64_t *hash_count_ptr, volatile bool *mining_flag);

/**
 * Compute midstate from block 1 (first 64 bytes of header).
 * Call this ONCE per job, then use sha256_pipelined_mine_s3_v2.
 *
 * @param block1_swapped First 16 words of header (byte-swapped)
 * @param midstate_out   Output buffer for 8-word midstate
 */
void sha256_s3_compute_midstate(const uint32_t *block1_swapped, uint32_t *midstate_out);

/**
 * Optimized mining loop with midstate caching (v2).
 * ~40-60% faster than v1 by eliminating block 1 recomputation.
 *
 * @param midstate       Pre-computed midstate (8 words from sha256_s3_compute_midstate)
 * @param block2_words   Block 2 template: words 0-2 (timestamp, nbits, merkle_tail) swapped
 * @param nonce_ptr      Pointer to nonce in BE format (updated on return)
 * @param hash_count_ptr Pointer to hash counter
 * @param mining_flag    Pointer to mining active flag
 *
 * @return true if potential share found (16-bit early reject passed)
 */
bool sha256_pipelined_mine_s3_v2(const uint32_t *midstate, const uint32_t *block2_words, uint32_t *nonce_ptr,
                                 volatile uint64_t *hash_count_ptr, volatile bool *mining_flag);

/**
 * Initialize SHA_TEXT zeros for v3.
 * Call once per job to set persistent zeros in block 2 padding area.
 */
void sha256_s3_init_zeros(void);

/**
 * Ultra-optimized mining loop v3 with batched copies.
 * Optimizations:
 * - Batched register loads/stores for SHA_H copy (memory pipeline)
 * - Persistent zeros (skip writing 10 zeros per iteration)
 * - Pre-computed length constants
 *
 * IMPORTANT: Call sha256_s3_init_zeros() once per job before using this.
 *
 * @param midstate       Pre-computed midstate (8 words)
 * @param block2_words   Block 2 template: words 0-2 swapped
 * @param nonce_ptr      Pointer to nonce in BE format
 * @param hash_count_ptr Pointer to hash counter
 * @param mining_flag    Pointer to mining active flag
 *
 * @return true if potential share found
 */
bool sha256_pipelined_mine_s3_v3(const uint32_t *midstate, const uint32_t *block2_words, uint32_t *nonce_ptr,
                                 volatile uint64_t *hash_count_ptr, volatile bool *mining_flag);

#ifdef __cplusplus
}
#endif

}  // namespace nerdminer
}  // namespace esphome

#endif  // CONFIG_IDF_TARGET_ESP32S3
