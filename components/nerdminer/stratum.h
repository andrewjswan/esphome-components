#pragma once

#include "cJSON.h"

#include "esphome/components/socket/socket.h"
#include "esphome/components/wifi/wifi_component.h"

#include <stdint.h>
#include <ArduinoJson.h>
#include <WiFi.h>

#define MAX_MERKLE_BRANCHES 32
#define HASH_SIZE 32
#define COINBASE_SIZE 100
#define COINBASE2_SIZE 128

#define BUFFER 1024

namespace esphome {
namespace nerdminer {

typedef struct {
  std::string sub_details;
  std::string extranonce1;
  std::string extranonce2;
  int extranonce2_size;
  char wName[80];
  char wPass[20];
} mining_subscribe;

typedef struct {
  std::string job_id;
  std::string prev_block_hash;
  std::string coinb1;
  std::string coinb2;
  std::string nbits;
  JsonArray merkle_branch;
  std::string version;
  uint32_t target;
  std::string ntime;
  bool clean_jobs;
} mining_job;

typedef enum {
  STRATUM_SUCCESS,
  STRATUM_UNKNOWN,
  STRATUM_PARSE_ERROR,
  MINING_NOTIFY,
  MINING_SET_DIFFICULTY
} stratum_method;

unsigned long getNextId(unsigned long id);
bool verifyPayload(std::string &line);
bool checkError(const JsonDocument doc);

// Method Mining.Subscribe
mining_subscribe init_mining_subscribe(void);
bool tx_mining_subscribe(esphome::socket::Socket *client, mining_subscribe &mSubscribe);
bool parse_mining_subscribe(std::string line, mining_subscribe &mSubscribe);

// Method Mining.Authorise
bool tx_mining_auth(esphome::socket::Socket *client, const char *user, const char *pass);
stratum_method parse_mining_method(std::string line);
bool parse_mining_notify(std::string line, mining_job &mJob);

// Method Mining.Submit
bool tx_mining_submit(esphome::socket::Socket *client, mining_subscribe mWorker, mining_job mJob, unsigned long nonce,
                      unsigned long &submit_id);

// Difficulty Methods
bool tx_suggest_difficulty(esphome::socket::Socket *client, double difficulty);
bool parse_mining_set_difficulty(std::string line, double &difficulty);

// ID Methods
unsigned long parse_extract_id(const std::string &line);

}  // namespace nerdminer
}  // namespace esphome
