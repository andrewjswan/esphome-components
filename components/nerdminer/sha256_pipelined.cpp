/*
 * SparkMiner - Pipelined SHA-256 Assembly Implementation
 *
 * Ported from BitsyMiner by Justin Williams (GPL v3)
 *
 * Uses inline Xtensa assembly to pipeline SHA operations for ~2x speedup.
 */

#include "nerdminer.h"
#include "sha256_pipelined.h"

#if defined(CONFIG_IDF_TARGET_ESP32)

#include <soc/dport_reg.h>
#include <soc/hwcrypto_reg.h>
#include <hal/sha_ll.h>

// SHA peripheral register offsets from SHA_TEXT_BASE
// SHA_TEXT_BASE = 0x3FF03000
// Control registers start at offset 0x90
#define SHA_START_OFFSET 0x90     // Write 1 to start new hash
#define SHA_CONTINUE_OFFSET 0x94  // Write 1 to continue hash
#define SHA_LOAD_OFFSET 0x98      // Write 1 to load result to buffer
#define SHA_BUSY_OFFSET 0x9C      // Read: 0 = idle, non-zero = busy

namespace esphome {
namespace nerdminer {

void sha256_pipelined_init(void) {
  // Enable SHA peripheral clock and clear reset
  DPORT_REG_SET_BIT(DPORT_PERI_CLK_EN_REG, DPORT_PERI_EN_SHA);
  DPORT_REG_CLR_BIT(DPORT_PERI_RST_EN_REG, DPORT_PERI_EN_SHA | DPORT_PERI_EN_SECUREBOOT);
}

// clang-format off

bool sha256_pipelined_mine(
    volatile uint32_t *sha_base,
    const uint32_t *header_swapped,
    uint32_t *nonce_ptr,
    volatile uint64_t *hash_count_ptr,
    volatile bool *mining_flag
) {
    // Constants for SHA padding
    const uint32_t shaPad = 0x80000000u;        // Padding byte after data
    const uint32_t firstShaBitLen = 0x00000280u;  // 640 bits (80 bytes) in big-endian
    const uint32_t secondShaBitLen = 0x00000100u; // 256 bits (32 bytes) in big-endian

    // Re-initialize SHA hardware (in case power management disabled it)
    DPORT_REG_SET_BIT(DPORT_PERI_CLK_EN_REG, DPORT_PERI_EN_SHA);
    DPORT_REG_CLR_BIT(DPORT_PERI_RST_EN_REG, DPORT_PERI_EN_SHA | DPORT_PERI_EN_SECUREBOOT);

    /*
     * Pipelined Mining Loop in Xtensa Assembly
     *
     * Register allocation:
     *   a2 = nonce (persists across loop iterations)
     *   a3, a4 = scratch registers for data movement
     *   a5 = SHA control base (sha_base + 0x90)
     *   a8 = loop counter for zeroing
     *
     * Control register offsets from a5:
     *   +0x00 (0x90) = SHA_START - write 1 to start new hash
     *   +0x04 (0x94) = SHA_CONTINUE - write 1 to continue hash
     *   +0x08 (0x98) = SHA_LOAD - write 1 to load result to buffer
     *   +0x0C (0x9C) = SHA_BUSY - read busy flag
     *
     * Pipeline strategy:
     *   While SHA hardware processes one operation, CPU prepares next data
     */
    __asm__ __volatile__(

        // Load nonce into register a2, compute control base in a5
        "l32i.n   a2,  %[nonce], 0 \n"    // a2 = *nonce_ptr
        "addi     a5,  %[sb], 0x90 \n"    // a5 = sha_base + 0x90 (control registers)

    "proc_start: \n"

        // ===== BLOCK 1: Load first 64 bytes of header =====
        // Load 16 words (64 bytes) from header_swapped to SHA_TEXT_BASE
        "l32i.n    a3,  %[IN],  0 \n"
        "s32i.n    a3,  %[sb],  0 \n"
        "l32i.n    a3,  %[IN],  4 \n"
        "s32i.n    a3,  %[sb],  4 \n"

        "l32i.n    a3,  %[IN],  8 \n"
        "s32i.n    a3,  %[sb],  8 \n"
        "l32i.n    a3,  %[IN],  12 \n"
        "s32i.n    a3,  %[sb],  12 \n"

        "l32i.n    a3,  %[IN],  16 \n"
        "s32i.n    a3,  %[sb],  16 \n"
        "l32i.n    a3,  %[IN],  20 \n"
        "s32i.n    a3,  %[sb],  20 \n"

        "l32i.n    a3,  %[IN],  24 \n"
        "s32i.n    a3,  %[sb],  24 \n"
        "l32i.n    a3,  %[IN],  28 \n"
        "s32i.n    a3,  %[sb],  28 \n"

        "l32i.n    a3,  %[IN],  32 \n"
        "s32i.n    a3,  %[sb],  32 \n"
        "l32i.n    a3,  %[IN],  36 \n"
        "s32i.n    a3,  %[sb],  36 \n"

        "l32i.n    a3,  %[IN],  40 \n"
        "s32i.n    a3,  %[sb],  40 \n"
        "l32i.n    a3,  %[IN],  44 \n"
        "s32i.n    a3,  %[sb],  44 \n"

        "l32i.n    a3,  %[IN],  48 \n"
        "s32i.n    a3,  %[sb],  48 \n"
        "l32i.n    a3,  %[IN],  52 \n"
        "s32i.n    a3,  %[sb],  52 \n"

        "l32i.n    a3,  %[IN],  56 \n"
        "s32i.n    a3,  %[sb],  56 \n"
        "l32i.n    a3,  %[IN],  60 \n"
        "s32i.n    a3,  %[sb],  60 \n"

        // ===== START SHA on block 1 =====
        "movi.n  a3, 1\n"
        "s32i.n  a3, a5, 0\n"             // Write 1 to SHA_START (offset 0x90)
        "memw   \n"                        // Memory barrier

        // ===== PIPELINE: Prepare block 2 while SHA processes block 1 =====
        // Block 2 = header bytes 64-79 (timestamp, nbits, nonce) + padding

        // Load words 16-18 from header (timestamp, nbits, partial nonce)
        "l32i    a3,  %[IN],   64 \n"
        "s32i.n  a3,  %[sb],    0 \n"
        "l32i    a3,  %[IN],   68 \n"
        "s32i.n  a3,  %[sb],    4 \n"
        "l32i    a3,  %[IN],   72 \n"
        "s32i.n  a3,  %[sb],    8 \n"

        // Store nonce (from register a2)
        "s32i.n    a2, %[sb], 12 \n"

        // Store padding: 0x80000000 at word 4, length at word 15
        "s32i.n    %[pad2], %[sb], 16 \n"
        "s32i.n    %[len1], %[sb], 60 \n"

        // Zero words 5-14 (offsets 20-56)
        "movi.n  a4,  0            \n"
        "addi    a8, %[sb], 20  \n"       // ptr = &sb[5]
        "movi.n  a3, 10         \n"       // 10 words to zero

        "loop    a3, 1f         \n"
        "s32i.n  a4, a8, 0      \n"
        "addi.n  a8, a8, 4      \n"

    "1:\n"
        // ===== WAIT for block 1 to complete =====
        "l32i.n  a3, a5, 12 \n"           // Read SHA_BUSY (offset 0x9C)
        "bnez.n  a3, 1b\n"                // Loop while busy

        // ===== CONTINUE SHA with block 2 =====
        "movi.n  a3, 1\n"
        "s32i.n  a3, a5, 4\n"             // Write 1 to SHA_CONTINUE (offset 0x94)
        "memw \n"

    "2:\n"
        // ===== WAIT for block 2 to complete =====
        "l32i.n  a4, a5, 12 \n"           // Read SHA_BUSY
        "bnez.n  a4, 2b\n"

        // ===== LOAD result to buffer =====
        "movi.n  a4, 1\n"
        "s32i.n  a4, a5, 8\n"             // Write 1 to SHA_LOAD (offset 0x98)
        "memw \n"

        // Increment nonce now (while waiting for load)
        "addi.n     a2, a2, 1\n"

    "3:\n"
        // ===== WAIT for load to complete =====
        "l32i.n  a4, a5, 12\n"
        "bnez.n  a4, 3b\n"

        // ===== PREPARE double-hash block =====
        // First 8 words already contain hash result
        // Add padding at word 8, length at word 15
        "s32i.n   %[pad2], %[sb], 32 \n"  // 0x80000000 at offset 32
        "s32i.n   %[len2], %[sb], 60 \n"  // 256 bits at offset 60

        // ===== START second SHA (double hash) =====
        "movi.n  a4, 1\n"
        "s32i.n  a4, a5, 0\n"             // SHA_START
        "memw\n"

        // ===== INCREMENT 64-bit hash counter (while SHA processes) =====
        "l32i.n  a3, %[ih], 0\n"          // Load low 32 bits
        "addi.n  a3, a3, 1  \n"           // Increment
        "s32i.n  a3, %[ih], 0\n"          // Store low 32 bits

        "bnez.n  a3, 4f\n"                // If low != 0, skip high increment
        "l32i.n  a4, %[ih], 4\n"          // Load high 32 bits
        "addi.n  a4, a4, 1\n"             // Increment high
        "s32i.n  a4, %[ih], 4\n"          // Store high 32 bits

    "4:\n"
        // ===== WAIT for second SHA to complete =====
        "l32i.n  a4, a5, 12\n"
        "bnez.n  a4, 4b\n"

        // ===== LOAD final result =====
        "movi.n  a3, 1\n"
        "s32i.n  a3, a5, 8\n"             // SHA_LOAD
        "memw \n"

    "5:\n"
        // ===== WAIT for load =====
        "l32i.n  a4, a5, 12\n"
        "bnez.n  a4, 5b\n"

        // ===== CHECK mining flag =====
        "l8ui   a3, %[flag], 0     \n"    // Load mining flag byte
        "beqz.n a3, proc_end     \n"      // Exit if mining stopped

        // ===== EARLY REJECT: Check top 16 bits of hash =====
        // Word 7 (offset 28) contains H0 - most significant bits
        // For valid hash, upper 16 bits must be zero
        "l16ui  a3, %[sb], 28         \n" // Load upper 16 bits of H0
        "beqz.n a3, proc_end          \n" // Exit if potential share found!
        "j proc_start                 \n" // Otherwise, try next nonce

    "proc_end:\n"
        // Store final nonce back to memory
        "s32i.n    a2, %[nonce], 0\n"

        :
        : [sb] "r"(sha_base),
          [IN] "r"(header_swapped),
          [ih] "r" (hash_count_ptr),
          [nonce] "r"(nonce_ptr),
          [flag] "r"(mining_flag),
          [pad2]  "r" (shaPad),
          [len2]  "r" (secondShaBitLen),
          [len1] "r" (firstShaBitLen)
        : "a2", "a3", "a4", "a5", "a8", "memory"
    );

    // Check if we exited due to potential share (16-bit check passed)
    // or because mining was stopped
    if (*mining_flag) {
        // Check if SHA hardware is still enabled
        bool shaEnabled = DPORT_REG_READ(DPORT_PERI_CLK_EN_REG) & DPORT_PERI_EN_SHA;
        if (!shaEnabled) {
            ESP_LOGD(TAG, "[SHA-PIPE] WARNING: SHA module disabled, reinitializing");
            sha256_pipelined_init();
            return false;
        }
        // 16-bit check passed - potential share found
        return true;
    }

    // Mining was stopped
    return false;
}

// clang-format on

}  // namespace nerdminer
}  // namespace esphome

#endif  // CONFIG_IDF_TARGET_ESP32

#include "sha256_pipelined_v2.cpp"
#include "sha256_pipelined_v3.cpp"
