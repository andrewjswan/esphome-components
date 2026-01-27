#include "cJSON.h"
#include "stratum.h"
#include "utils.h"
#include "nerdminer.h"

#include "lwip/sockets.h"
#include "esphome/core/log.h"

#include <ArduinoJson.h>
#include <string.h>
#include <stdio.h>

namespace esphome {
namespace nerdminer {

JsonDocument doc;
unsigned long id = 1;

// Get next JSON RPC Id
unsigned long getNextId(unsigned long id) {
  if (id == ULONG_MAX) {
    id = 1;
    return id;
  }
  return ++id;
}

// Verify Payload doesn't has zero lenght
bool verifyPayload(std::string &line) {
  const char *whitespace = " \t\n\r\f\v";

  line.erase(0, line.find_first_not_of(whitespace));
  size_t last = line.find_last_not_of(whitespace);
  if (last != std::string::npos) {
    line.erase(last + 1);
  } else {
    line.clear();
  }

  return !line.empty();
}

bool checkError(const JsonDocument doc) {
  if (!doc["error"].is<JsonVariant>())
    return false;
  if (doc["error"].size() == 0)
    return false;

  ESP_LOGE(TAG, "ERROR: %d | reason: %s \n", (const int) doc["error"][0], (const char *) doc["error"][1]);

  return true;
}

// STEP 1: Pool server connection (SUBSCRIBE)
// Docs:
// - https://cs.braiins.com/stratum-v1/docs
// - https://github.com/aeternity/protocol/blob/master/STRATUM.md#mining-subscribe
bool tx_mining_subscribe(esphome::socket::Socket *client, mining_subscribe &mSubscribe) {
  char payload[BUFFER] = {0};

  if (client == nullptr)
    return false;

  // Subscribe
  id = 1;  // Initialize id messages
  sprintf(payload, "{\"id\": %u, \"method\": \"mining.subscribe\", \"params\": [\"NerdMinerV2/%s\"]}\n", id,
          NERDMINER_VERSION);

  ESP_LOGD(TAG, "[STRATUM] ==> Mining subscribe");
  ESP_LOGD(TAG, "  Sending: %s", payload);
  pool_send(pool_socket_.get(), std::string(payload));

  std::string line = pool_read_until(client, '\n');
  if (line.empty())
    return false;
  if (!parse_mining_subscribe(line.c_str(), mSubscribe)) {
    return false;
  }

  ESP_LOGD(TAG, "    sub_details: %s", mSubscribe.sub_details.c_str());
  ESP_LOGD(TAG, "    extranonce1: %s", mSubscribe.extranonce1.c_str());
  ESP_LOGD(TAG, "    extranonce2_size: %d", mSubscribe.extranonce2_size);

  if ((mSubscribe.extranonce1.length() == 0)) {
    ESP_LOGD(TAG, "[STRATUM] >>>>>>>>> Work aborted.");
    ESP_LOGD(TAG, "    extranonce1 length: %u", mSubscribe.extranonce1.length());
    doc.clear();
    return false;
  }
  return true;
}

bool parse_mining_subscribe(std::string line, mining_subscribe &mSubscribe) {
  if (!verifyPayload(line))
    return false;
  ESP_LOGD(TAG, "  Receiving: %s", line.c_str());

  DeserializationError error = deserializeJson(doc, line);

  if (error || checkError(doc))
    return false;
  if (!doc["result"].is<JsonVariant>())
    return false;

  mSubscribe.sub_details = doc["result"][0][0][1].as<std::string>();
  mSubscribe.extranonce1 = doc["result"][1].as<std::string>();
  mSubscribe.extranonce2_size = doc["result"][2];

  return true;
}

mining_subscribe init_mining_subscribe(void) {
  mining_subscribe new_mSub;

  new_mSub.extranonce1 = "";
  new_mSub.extranonce2 = "";
  new_mSub.extranonce2_size = 0;
  new_mSub.sub_details = "";

  return new_mSub;
}

// STEP 2: Pool server auth (authorize)
bool tx_mining_auth(esphome::socket::Socket *client, const char *user, const char *pass) {
  char payload[BUFFER] = {0};

  if (client == nullptr) return false;

  // Authorize
  id = getNextId(id);
  sprintf(payload, "{\"params\": [\"%s\", \"%s\"], \"id\": %u, \"method\": \"mining.authorize\"}\n", user, pass, id);

  ESP_LOGD(TAG, "[STRATUM] ==> Autorize work");
  ESP_LOGD(TAG, "  Sending: %s", payload);
  pool_send(pool_socket_.get(), std::string(payload));

  vTaskDelay(200 / portTICK_PERIOD_MS);  // Small delay

  // Don't parse here any answer
  // Miner started to receive mining notifications so better parse all at main thread
  return true;
}

stratum_method parse_mining_method(std::string line) {
  if (!verifyPayload(line))
    return STRATUM_PARSE_ERROR;
  ESP_LOGD(TAG, "  Receiving: %s", line.c_str());

  DeserializationError error = deserializeJson(doc, line);
  if (error || checkError(doc))
    return STRATUM_PARSE_ERROR;

  if (!doc["method"].is<JsonVariant>()) {
    // "error":null means success
    if (doc["error"].isNull())
      return STRATUM_SUCCESS;
    else
      return STRATUM_UNKNOWN;
  }
  stratum_method result = STRATUM_UNKNOWN;

  if (strcmp("mining.notify", (const char *) doc["method"]) == 0) {
    result = MINING_NOTIFY;
  } else if (strcmp("mining.set_difficulty", (const char *) doc["method"]) == 0) {
    result = MINING_SET_DIFFICULTY;
  }

  return result;
}

bool parse_mining_notify(std::string line, mining_job &mJob) {
  ESP_LOGD(TAG, "    Parsing Method [MINING NOTIFY]");
  if (!verifyPayload(line))
    return false;

  DeserializationError error = deserializeJson(doc, line);
  if (error)
    return false;
  if (!doc["params"].is<JsonVariant>())
    return false;

  mJob.job_id = doc["params"][0].as<std::string>();
  mJob.prev_block_hash = doc["params"][1].as<std::string>();
  mJob.coinb1 = doc["params"][2].as<std::string>();
  mJob.coinb2 = doc["params"][3].as<std::string>();
  mJob.merkle_branch = doc["params"][4];
  mJob.version = doc["params"][5].as<std::string>();
  mJob.nbits = doc["params"][6].as<std::string>();
  mJob.ntime = doc["params"][7].as<std::string>();
  mJob.clean_jobs = doc["params"][8];  // bool

#ifdef DEBUG_MINING
  ESP_LOGD(TAG, "    job_id: %s", mJob.job_id.c_str());
  ESP_LOGD(TAG, "    prevhash: %s", mJob.prev_block_hash.c_str());
  ESP_LOGD(TAG, "    coinb1: %s", mJob.coinb1.c_str());
  ESP_LOGD(TAG, "    coinb2: %s", mJob.coinb2.c_str());
  ESP_LOGD(TAG, "    merkle_branch size: %d", mJob.merkle_branch.size());
  ESP_LOGD(TAG, "    version: %s", mJob.version.c_str());
  ESP_LOGD(TAG, "    nbits: %s", mJob.nbits.c_str());
  ESP_LOGD(TAG, "    ntime: %s", mJob.ntime.c_str());
  ESP_LOGD(TAG, "    clean_jobs: %s", YESNO(mJob.clean_jobs));
#endif

  // Check if parameters where correctly received
  if (checkError(doc)) {
    ESP_LOGD(TAG, "[STRATUM] >>>>>>>>> Work aborted.");
    return false;
  }

  return true;
}

bool tx_mining_submit(esphome::socket::Socket *client, mining_subscribe mWorker, mining_job mJob, unsigned long nonce,
                      unsigned long &submit_id) {
  char payload[BUFFER] = {0};

  if (client == nullptr) return false;

  // Submit
  id = getNextId(id);
  submit_id = id;
  snprintf(payload, sizeof(payload),
         "{\"id\": %u, \"method\": \"mining.submit\", \"params\": [\"%s\",\"%s\",\"%s\",\"%s\",\"%08x\"]}\n",
         id,
         mWorker.wName,
         mJob.job_id.c_str(),
         mWorker.extranonce2.c_str(),
         mJob.ntime.c_str(),
         nonce);
  ESP_LOGD(TAG, "  Sending: %s", payload);
  pool_send(pool_socket_.get(), std::string(payload));

  return true;
}

bool parse_mining_set_difficulty(std::string line, double &difficulty) {
  ESP_LOGD(TAG, "    Parsing Method [SET DIFFICULTY]");
  if (!verifyPayload(line))
    return false;

  DeserializationError error = deserializeJson(doc, line);
  if (error)
    return false;
  if (!doc["params"].is<JsonVariant>())
    return false;

  difficulty = (double) doc["params"][0];
  ESP_LOGD(TAG, "    difficulty: %f", difficulty);

  return true;
}

bool tx_suggest_difficulty(esphome::socket::Socket *client, double difficulty) {
  char payload[BUFFER] = {0};

  if (client == nullptr) return false;

  id = getNextId(id);
  sprintf(payload, "{\"id\": %d, \"method\": \"mining.suggest_difficulty\", \"params\": [%.10g]}\n", id, difficulty);

  ESP_LOGD(TAG, "  Sending: %s", payload);
  return pool_send(pool_socket_.get(), std::string(payload)) > 0;
}

unsigned long parse_extract_id(const std::string &line) {
  DeserializationError error = deserializeJson(doc, line);
  if (error) {
    return 0;
  }

  if (doc["id"].isNull()) {
    return 0;
  }
  if (!doc["id"].is<unsigned long>()) {
    return 0;
  }

  return doc["id"].as<unsigned long>();
}

}  // namespace nerdminer
}  // namespace esphome
