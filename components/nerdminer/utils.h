#pragma once

#include "stratum.h"
#include "mining.h"

#include <stddef.h>
#include <stdint.h>

#include <string>

#include "esphome/core/component.h"
#include "esphome/components/socket/socket.h"

// General byte order swapping functions.

#define bswap16(x) __bswap16(x)
#define bswap32(x) __bswap32(x)
#define bswap64(x) __bswap64(x)

namespace esphome {
namespace nerdminer {

uint8_t hex(char ch);

int to_byte_array(const char *in, size_t in_size, uint8_t *out);
double le256todouble(const void *target);
double diff_from_target(void *target);
bool isSha256Valid(const void *sha256);
miner_data calculateMiningData(mining_subscribe &mWorker, mining_job mJob);
bool checkValid(unsigned char *hash, unsigned char *target);
void suffix_string(double val, char *buf, size_t bufsiz, int sigdigits);

bool pool_connected(esphome::socket::Socket *sock);
bool pool_available(esphome::socket::Socket *sock);
std::string pool_read_until(esphome::socket::Socket *sock, char terminator);
ssize_t pool_send(esphome::socket::Socket *sock, const std::string &data);
void pool_close(std::unique_ptr<esphome::socket::Socket> &sock);

uint32_t crc32_reset();
uint32_t crc32_add(uint32_t crc32, const void *data, size_t size);
uint32_t crc32_finish(uint32_t crc32);

}  // namespace nerdminer
}  // namespace esphome
