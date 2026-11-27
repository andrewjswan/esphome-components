#include "sha256_pipelined.h"

#if defined(CONFIG_IDF_TARGET_ESP32)

#include <esp_attr.h>
#include <soc/dport_reg.h>
#include <soc/hwcrypto_reg.h>

namespace esphome::nerdminer {

void sha256_pipelined_init() {
  DPORT_REG_SET_BIT(DPORT_PERI_CLK_EN_REG, DPORT_PERI_EN_SHA);
  DPORT_REG_CLR_BIT(DPORT_PERI_RST_EN_REG, DPORT_PERI_EN_SHA | DPORT_PERI_EN_SECUREBOOT);
}

// clang-format off
bool IRAM_ATTR sha256_pipelined_mine(const uint32_t *header_swapped, uint32_t *nonce_swapped, uint32_t max_iterations,
                                     uint32_t *hashes_done, const volatile bool *mining_flag) {
  if (max_iterations == 0 || !*mining_flag) {
    *hashes_done = 0;
    return false;
  }

  // Ensure SHA peripheral clock is active
  if ((DPORT_REG_READ(DPORT_PERI_CLK_EN_REG) & DPORT_PERI_EN_SHA) == 0) {
    sha256_pipelined_init();
  }

  volatile uint32_t *sha_base = reinterpret_cast<volatile uint32_t *>(SHA_TEXT_BASE);
  const uint32_t sha_pad = 0x80000000U;
  const uint32_t first_sha_bit_len = 0x00000280U;   // 640 bits (80-byte header)
  const uint32_t second_sha_bit_len = 0x00000100U;  // 256 bits (32-byte intermediate digest)

  // loop_state[0] = remaining iterations (in/out)
  // loop_state[1] = candidate found flag (out: 1 if early-reject passed, 0 otherwise)
  uint32_t loop_state[2] = {max_iterations, 0U};

  // Register allocation inside Xtensa inline assembly (7 inputs + 6 clobbers = 13 regs, fits Xtensa window):
  //   a2 = current big-endian nonce
  //   a3, a4 = scratch registers
  //   a5 = SHA control register base (SHA_TEXT_BASE + 0x90: +0 START, +4 CONTINUE, +8 LOAD, +12 BUSY)
  //   a8 = constant 0 for unrolled padding stores
  //   a9 = remaining iteration counter
  __asm__ __volatile__(
      "l32i.n   a2,  %[nonce], 0     \n"
      "addi     a5,  %[sb], 0x90     \n"
      "movi.n   a8,  0               \n"
      "l32i.n   a9,  %[state], 0     \n"

      "pipe_start:                   \n"
      // Load block 1 (first 64 bytes of header) into SHA_TEXT[0..15]
      "l32i.n   a3,  %[in],  0       \n"
      "s32i.n   a3,  %[sb],  0       \n"
      "l32i.n   a3,  %[in],  4       \n"
      "s32i.n   a3,  %[sb],  4       \n"
      "l32i.n   a3,  %[in],  8       \n"
      "s32i.n   a3,  %[sb],  8       \n"
      "l32i.n   a3,  %[in], 12       \n"
      "s32i.n   a3,  %[sb], 12       \n"
      "l32i.n   a3,  %[in], 16       \n"
      "s32i.n   a3,  %[sb], 16       \n"
      "l32i.n   a3,  %[in], 20       \n"
      "s32i.n   a3,  %[sb], 20       \n"
      "l32i.n   a3,  %[in], 24       \n"
      "s32i.n   a3,  %[sb], 24       \n"
      "l32i.n   a3,  %[in], 28       \n"
      "s32i.n   a3,  %[sb], 28       \n"
      "l32i.n   a3,  %[in], 32       \n"
      "s32i.n   a3,  %[sb], 32       \n"
      "l32i.n   a3,  %[in], 36       \n"
      "s32i.n   a3,  %[sb], 36       \n"
      "l32i.n   a3,  %[in], 40       \n"
      "s32i.n   a3,  %[sb], 40       \n"
      "l32i.n   a3,  %[in], 44       \n"
      "s32i.n   a3,  %[sb], 44       \n"
      "l32i.n   a3,  %[in], 48       \n"
      "s32i.n   a3,  %[sb], 48       \n"
      "l32i.n   a3,  %[in], 52       \n"
      "s32i.n   a3,  %[sb], 52       \n"
      "l32i.n   a3,  %[in], 56       \n"
      "s32i.n   a3,  %[sb], 56       \n"
      "l32i.n   a3,  %[in], 60       \n"
      "s32i.n   a3,  %[sb], 60       \n"

      // Trigger SHA_START on block 1
      "movi.n   a3, 1                \n"
      "s32i.n   a3, a5, 0            \n"
      "memw                          \n"

      // Pipeline: load block 2 (tail + nonce + SHA-256 padding) while hardware hashes block 1
      "l32i     a3,  %[in], 64       \n"
      "s32i.n   a3,  %[sb],  0       \n"
      "l32i     a3,  %[in], 68       \n"
      "s32i.n   a3,  %[sb],  4       \n"
      "l32i     a3,  %[in], 72       \n"
      "s32i.n   a3,  %[sb],  8       \n"
      "s32i.n   a2,  %[sb], 12       \n"
      "s32i.n   %[pad], %[sb], 16    \n"
      "s32i.n   a8,  %[sb], 20       \n"
      "s32i.n   a8,  %[sb], 24       \n"
      "s32i.n   a8,  %[sb], 28       \n"
      "s32i.n   a8,  %[sb], 32       \n"
      "s32i.n   a8,  %[sb], 36       \n"
      "s32i.n   a8,  %[sb], 40       \n"
      "s32i.n   a8,  %[sb], 44       \n"
      "s32i.n   a8,  %[sb], 48       \n"
      "s32i.n   a8,  %[sb], 52       \n"
      "s32i.n   a8,  %[sb], 56       \n"
      "s32i.n   %[len1], %[sb], 60   \n"

      // Wait for block 1 completion
      "pipe_wait1:                   \n"
      "l32i.n   a3, a5, 12           \n"
      "bnez.n   a3, pipe_wait1       \n"

      // Trigger SHA_CONTINUE on block 2
      "movi.n   a3, 1                \n"
      "s32i.n   a3, a5, 4            \n"
      "memw                          \n"

      // Wait for block 2 completion
      "pipe_wait2:                   \n"
      "l32i.n   a4, a5, 12           \n"
      "bnez.n   a4, pipe_wait2       \n"

      // Trigger SHA_LOAD to copy intermediate digest into SHA_TEXT[0..7]
      "movi.n   a4, 1                \n"
      "s32i.n   a4, a5, 8            \n"
      "memw                          \n"

      // Increment nonce while waiting for SHA_LOAD
      "addi.n   a2, a2, 1            \n"

      // Wait for intermediate load
      "pipe_wait3:                   \n"
      "l32i.n   a4, a5, 12           \n"
      "bnez.n   a4, pipe_wait3       \n"

      // Append double-SHA padding (words 9..14 remain zero from block 2)
      "s32i.n   %[pad], %[sb], 32    \n"
      "s32i.n   %[len2], %[sb], 60   \n"

      // Trigger SHA_START for second SHA-256
      "movi.n   a4, 1                \n"
      "s32i.n   a4, a5, 0            \n"
      "memw                          \n"

      // Decrement batch counter while second SHA runs
      "addi.n   a9, a9, -1           \n"

      // Wait for second SHA completion
      "pipe_wait4:                   \n"
      "l32i.n   a4, a5, 12           \n"
      "bnez.n   a4, pipe_wait4       \n"

      // Trigger SHA_LOAD to read final digest into SHA_TEXT[0..7]
      "movi.n   a3, 1                \n"
      "s32i.n   a3, a5, 8            \n"
      "memw                          \n"

      // Wait for final load
      "pipe_wait5:                   \n"
      "l32i.n   a4, a5, 12           \n"
      "bnez.n   a4, pipe_wait5       \n"

      // Early 16-bit reject: read 32-bit word from DPORT SHA_TEXT (offset 28) and check low 16 bits
      "l32i.n   a3, %[sb], 28        \n"
      "extui    a3, a3, 0, 16        \n"
      "beqz.n   a3, pipe_found       \n"

      // Stop if batch limit reached or mining_flag cleared
      "beqz.n   a9, pipe_end         \n"
      "l8ui     a3, %[flag], 0       \n"
      "bnez.n   a3, pipe_start       \n"
      "j        pipe_end             \n"

      "pipe_found:                   \n"
      "movi.n   a3, 1                \n"
      "s32i.n   a3, %[state], 4      \n"

      "pipe_end:                     \n"
      "s32i.n   a2, %[nonce], 0      \n"
      "s32i.n   a9, %[state], 0      \n"
      :
      : [sb] "r"(sha_base), [in] "r"(header_swapped), [nonce] "r"(nonce_swapped),
        [state] "r"(loop_state), [flag] "r"(mining_flag), [pad] "r"(sha_pad),
        [len1] "r"(first_sha_bit_len), [len2] "r"(second_sha_bit_len)
      : "a2", "a3", "a4", "a5", "a8", "a9", "memory");

  *hashes_done = max_iterations - loop_state;
  if (loop_state[1] != 0U) {
    // Read final 32-byte digest directly from SHA_TEXT[0..7] in reverse word order with byte swap
    // (identical to working nerd_sha_ll_read_digest_swap_if in main)
    auto *out_words = reinterpret_cast<uint32_t *>(hash_out);
    out_words = __builtin_bswap32(sha_base);
    out_words = __builtin_bswap32(sha_base);
    out_words = __builtin_bswap32(sha_base);
    out_words = __builtin_bswap32(sha_base);
    out_words = __builtin_bswap32(sha_base);
    out_words = __builtin_bswap32(sha_base);
    out_words = __builtin_bswap32(sha_base);
    out_words = __builtin_bswap32(sha_base);
    return true;
  }
  return false;

}
// clang-format on

}  // namespace esphome::nerdminer

#endif  // CONFIG_IDF_TARGET_ESP32
