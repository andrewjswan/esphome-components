#include "sha256_pipelined.h"

#if defined(CONFIG_IDF_TARGET_ESP32)

#include <esp_attr.h>
#include <soc/dport_access.h>
#include <soc/dport_reg.h>
#include <soc/hwcrypto_reg.h>

namespace esphome::nerdminer {

void sha256_pipelined_init() {
  DPORT_REG_SET_BIT(DPORT_PERI_CLK_EN_REG, DPORT_PERI_EN_SHA);
  DPORT_REG_CLR_BIT(DPORT_PERI_RST_EN_REG, DPORT_PERI_EN_SHA | DPORT_PERI_EN_SECUREBOOT);
}

bool IRAM_ATTR sha256_pipelined_mine(const uint32_t *header_swapped, uint32_t *nonce_swapped, uint32_t max_iterations,
                                     uint32_t *hashes_done, uint8_t *hash_out, const volatile bool *mining_flag) {
  if (max_iterations == 0 || !*mining_flag) {
    *hashes_done = 0;
    return false;
  }

  if ((DPORT_REG_READ(DPORT_PERI_CLK_EN_REG) & DPORT_PERI_EN_SHA) == 0) {
    sha256_pipelined_init();
  }

  volatile uint32_t *sha_text = reinterpret_cast<volatile uint32_t *>(SHA_TEXT_BASE);
  uint32_t n_swapped = *nonce_swapped;
  uint32_t count = 0;

  while (count < max_iterations && *mining_flag) {
    // 1. Load block 1 (first 64 bytes of pre-swapped header)
    for (int i = 0; i < 16; ++i) {
      DPORT_REG_WRITE(SHA_TEXT_BASE + i * 4, header_swapped[i]);
    }
    DPORT_REG_WRITE(SHA_256_START_REG, 1);
    while (DPORT_REG_READ(SHA_256_BUSY_REG) != 0) {
    }

    // 2. Load block 2 (remaining 12 bytes + swapped nonce + SHA-256 padding)
    DPORT_REG_WRITE(SHA_TEXT_BASE + 0 * 4, header_swapped[16]);
    DPORT_REG_WRITE(SHA_TEXT_BASE + 1 * 4, header_swapped[17]);
    DPORT_REG_WRITE(SHA_TEXT_BASE + 2 * 4, header_swapped[18]);
    DPORT_REG_WRITE(SHA_TEXT_BASE + 3 * 4, n_swapped);
    DPORT_REG_WRITE(SHA_TEXT_BASE + 4 * 4, 0x80000000U);
    for (int i = 5; i < 15; ++i) {
      DPORT_REG_WRITE(SHA_TEXT_BASE + i * 4, 0U);
    }
    DPORT_REG_WRITE(SHA_TEXT_BASE + 15 * 4, 0x00000280U);

    DPORT_REG_WRITE(SHA_256_CONTINUE_REG, 1);
    while (DPORT_REG_READ(SHA_256_BUSY_REG) != 0) {
    }

    DPORT_REG_WRITE(SHA_256_LOAD_REG, 1);
    while (DPORT_REG_READ(SHA_256_BUSY_REG) != 0) {
    }

    // 3. Second SHA-256 on 32-byte intermediate digest already loaded in SHA_TEXT[0..7]
    DPORT_REG_WRITE(SHA_TEXT_BASE + 8 * 4, 0x80000000U);
    for (int i = 9; i < 15; ++i) {
      DPORT_REG_WRITE(SHA_TEXT_BASE + i * 4, 0U);
    }
    DPORT_REG_WRITE(SHA_TEXT_BASE + 15 * 4, 0x00000100U);

    DPORT_REG_WRITE(SHA_256_START_REG, 1);
    while (DPORT_REG_READ(SHA_256_BUSY_REG) != 0) {
    }

    DPORT_REG_WRITE(SHA_256_LOAD_REG, 1);
    while (DPORT_REG_READ(SHA_256_BUSY_REG) != 0) {
    }

    // Increment big-endian nonce (keep contract: candidate is *nonce_swapped - 1)
    n_swapped++;
    count++;

    // 4. Read H0 safely via DPORT_SEQUENCE_REG_READ and check upper 16 bits
    DPORT_INTERRUPT_DISABLE();
    uint32_t fin = DPORT_SEQUENCE_REG_READ(SHA_TEXT_BASE + 7 * 4);
    if ((fin & 0xFFFFU) == 0U) {
      auto *out_words = reinterpret_cast<uint32_t *>(hash_out);
      out_words[7] = __builtin_bswap32(fin);
      out_words[0] = __builtin_bswap32(DPORT_SEQUENCE_REG_READ(SHA_TEXT_BASE + 0 * 4));
      out_words[1] = __builtin_bswap32(DPORT_SEQUENCE_REG_READ(SHA_TEXT_BASE + 1 * 4));
      out_words[2] = __builtin_bswap32(DPORT_SEQUENCE_REG_READ(SHA_TEXT_BASE + 2 * 4));
      out_words[3] = __builtin_bswap32(DPORT_SEQUENCE_REG_READ(SHA_TEXT_BASE + 3 * 4));
      out_words[4] = __builtin_bswap32(DPORT_SEQUENCE_REG_READ(SHA_TEXT_BASE + 4 * 4));
      out_words[5] = __builtin_bswap32(DPORT_SEQUENCE_REG_READ(SHA_TEXT_BASE + 5 * 4));
      out_words[6] = __builtin_bswap32(DPORT_SEQUENCE_REG_READ(SHA_TEXT_BASE + 6 * 4));
      DPORT_INTERRUPT_RESTORE();

      *nonce_swapped = n_swapped;
      *hashes_done = count;
      return true;
    }
    DPORT_INTERRUPT_RESTORE();
  }

  *nonce_swapped = n_swapped;
  *hashes_done = count;
  return false;
}

}  // namespace esphome::nerdminer

#endif  // CONFIG_IDF_TARGET_ESP32
