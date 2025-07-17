#pragma once

/*
 * written by: Bitmaker
 * based on: Blockstream Jade shaLib
 * thanks to @LarryBitcoin
 *
 * Description:
 *
 * NerdSha256plus is a custom C implementation of sha256d based on Blockstream Jade
 * code https://github.com/Blockstream/Jade
 *
 * The folowing file can be used on any ESP32 implementation using both cores
 *
 */

#include "esphome/core/hal.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

namespace esphome {
namespace nerdminer {

struct nerdSHA256_context {
  uint8_t buffer[64];
  uint32_t digest[8];
};

/* Calculate midstate */

void nerd_mids(uint32_t *digest, const uint8_t *dataIn);
bool nerd_sha256d(nerdSHA256_context *midstate, uint8_t *dataIn, uint8_t *doubleHash);

void nerd_sha256_bake(const uint32_t *digest, const uint8_t *dataIn, uint32_t *bake);  // 15 words
bool nerd_sha256d_baked(const uint32_t *digest, const uint8_t *dataIn, const uint32_t *bake, uint8_t *doubleHash);

void ByteReverseWords(uint32_t *out, const uint32_t *in, uint32_t byteCount);

}  // namespace nerdminer
}  // namespace esphome
