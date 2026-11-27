#include <cstdio>
#include <cstring>

#include "stratum.h"
#include "nerdminer.h"
#include "utils.h"

#include "esphome/components/json/json_util.h"
#include "esphome/core/log.h"

namespace esphome::nerdminer {

unsigned long id = 1;

// Get next JSON RPC Id
unsigned long getNextId(unsigned long id) {
  if (id == ULONG_MAX) {
    id = 1;
    return id;
  }
  return ++id;
}

// Verify Payload doesn't have zero length
bool verifyPayload(const std::string &line) {
  const char *whitespace = " \t\n\r\f\v";
  return line.find_first_not_of(whitespace) != std::string::npos;
}

bool checkError(JsonObject doc) {
  if (!doc["error"].is<JsonVariant>() || doc["error"].isNull()) {
    return false;
  }
  if (doc["error"].size() == 0) {
    return false;
  }

  ESP_LOGE(TAG, "ERROR: %d | reason: %s", (int) doc["error"][0], doc["error"][1].as<const char *>());
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
  snprintf(payload, sizeof(payload), "{\"id\": %lu, \"method\": \"mining.subscribe\", \"params\": [\"%s/%s\"]}\n", id,
           NERDMINER_CLIENT, NERDMINER_VERSION);

  ESP_LOGD(TAG, "[STRATUM] ==> Mining subscribe");
  ESP_LOGD(TAG, " Sending: %s", payload);
  pool_send(client, std::string(payload));

  std::string line = pool_read_until(client, '\n');
  if (line.empty())
    return false;
  if (!parse_mining_subscribe(line.c_str(), mSubscribe)) {
    return false;
  }

  ESP_LOGD(TAG, "  sub_details: %s", mSubscribe.sub_details.c_str());
  ESP_LOGD(TAG, "  extranonce1: %s", mSubscribe.extranonce1.c_str());
  ESP_LOGD(TAG, "  extranonce2_size: %d", mSubscribe.extranonce2_size);

  if ((mSubscribe.extranonce1.length() == 0)) {
    ESP_LOGD(TAG, "[STRATUM] >>>>>>>>> Work aborted.");
    ESP_LOGD(TAG, "  extranonce1 length: %u", mSubscribe.extranonce1.length());
    return false;
  }
  return true;
}

bool parse_mining_subscribe(const std::string &line, mining_subscribe &mSubscribe) {
  if (!verifyPayload(line))
    return false;
  ESP_LOGD(TAG, " Receiving: %s", line.c_str());

bool parse_mining_subscribe(const std::string &line, mining_subscribe &mSubscribe) {
  if (!verifyPayload(line)) {
    return false;
  }
  ESP_LOGD(TAG, " Receiving: %s", line.c_str());

  return esphome::json::parse_json(line, [&](JsonObject root) -> bool {
    if (checkError(root) || !root["result"].is<JsonVariant>()) {
      return false;
    }

    mSubscribe.sub_details = root["result"][0][0][1].as<std::string>();
    mSubscribe.extranonce1 = root["result"][1].as<std::string>();
    mSubscribe.extranonce2_size = root["result"][2];
    return true;
  });
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

  if (client == nullptr)
    return false;

  // Authorize
  id = getNextId(id);
  snprintf(payload, sizeof(payload), "{\"params\": [\"%s\", \"%s\"], \"id\": %lu, \"method\": \"mining.authorize\"}\n",
           user, pass, id);

  ESP_LOGD(TAG, "[STRATUM] ==> Autorize work");
  ESP_LOGD(TAG, " Sending: %s", payload);
  pool_send(client, std::string(payload));

  vTaskDelay(200 / portTICK_PERIOD_MS);  // Small delay

  // Don't parse here any answer
  // Miner started to receive mining notifications so better parse all at main thread
  return true;
}

stratum_method parse_mining_method(const std::string &line) {
  if (!verifyPayload(line))
    return STRATUM_PARSE_ERROR;
  ESP_LOGD(TAG, " Receiving: %s", line.c_str());

  stratum_method result = STRATUM_PARSE_ERROR;
  esphome::json::parse_json(line, [&](JsonObject root) -> bool {
    if (checkError(root)) {
      result = STRATUM_PARSE_ERROR;
      return false;
    }

    if (!root["method"].is<JsonVariant>()) {
      result = root["error"].isNull() ? STRATUM_SUCCESS : STRATUM_UNKNOWN;
      return true;
    }

    const char *method = root["method"].as<const char *>();
    if (method != nullptr && strcmp("mining.notify", method) == 0) {
      result = MINING_NOTIFY;
    } else if (method != nullptr && strcmp("mining.set_difficulty", method) == 0) {
      result = MINING_SET_DIFFICULTY;
    } else {
      result = STRATUM_UNKNOWN;
    }
    return true;
  });

  return result;
}

bool parse_mining_notify(const std::string &line, mining_job &mJob) {
  ESP_LOGD(TAG, "  Parsing Method [MINING NOTIFY]");
  if (!verifyPayload(line))
    return false;

  return esphome::json::parse_json(line, [&](JsonObject root) -> bool {
    if (checkError(root) || !root["params"].is<JsonVariant>()) {
      return false;
    }

    mJob.job_id = root["params"][0].as<std::string>();
    mJob.prev_block_hash = root["params"][1].as<std::string>();
    mJob.coinb1 = root["params"][2].as<std::string>();
    mJob.coinb2 = root["params"][3].as<std::string>();

    mJob.merkle_branch.clear();
    if (root["params"][4].is<JsonArray>()) {
      JsonArray branches = root["params"][4].as<JsonArray>();
      for (JsonVariant v : branches) {
        if (!v.isNull()) {
          mJob.merkle_branch.push_back(v.as<std::string>());
        }
      }
    }
    ESP_LOGD(TAG, "  merkle_branch size: %zu", mJob.merkle_branch.size());

    mJob.version = root["params"][5].as<std::string>();
    mJob.nbits = root["params"][6].as<std::string>();
    mJob.ntime = root["params"][7].as<std::string>();
    mJob.clean_jobs = root["params"][8];
    return true;
  });
}

bool tx_mining_submit(esphome::socket::Socket *client, const mining_subscribe &mWorker, const mining_job &mJob,
                      unsigned long nonce, unsigned long &submit_id) {
  char payload[BUFFER] = {0};

  if (client == nullptr)
    return false;

  // Submit
  id = getNextId(id);
  submit_id = id;
  snprintf(payload, sizeof(payload),
           "{\"id\": %lu, \"method\": \"mining.submit\", \"params\": [\"%s\",\"%s\",\"%s\",\"%s\",\"%08lx\"]}\n", id,
           mWorker.wName, mJob.job_id.c_str(), mWorker.extranonce2.c_str(), mJob.ntime.c_str(), nonce);
  ESP_LOGD(TAG, " Sending: %s", payload);
  pool_send(client, std::string(payload));

  return true;
}

bool parse_mining_set_difficulty(const std::string &line, double &difficulty) {
  ESP_LOGD(TAG, "  Parsing Method [SET DIFFICULTY]");
  if (!verifyPayload(line))
    return false;

  return esphome::json::parse_json(line, [&](JsonObject root) -> bool {
    if (checkError(root) || !root["params"].is<JsonVariant>()) {
      return false;
    }

    difficulty = root["params"][0].as<double>();
    ESP_LOGD(TAG, "  difficulty: %f", difficulty);
    return true;
  });
}

bool tx_suggest_difficulty(esphome::socket::Socket *client, double difficulty) {
  char payload[BUFFER] = {0};

  if (client == nullptr)
    return false;

  id = getNextId(id);
  snprintf(payload, sizeof(payload), "{\"id\": %lu, \"method\": \"mining.suggest_difficulty\", \"params\": [%.10g]}\n",
           id, difficulty);

  ESP_LOGD(TAG, " Sending: %s", payload);
  return pool_send(client, std::string(payload)) > 0;
}

unsigned long parse_extract_id(const std::string &line) {
  unsigned long extracted_id = 0;
  esphome::json::parse_json(line, [&](JsonObject root) -> bool {
    if (!root["id"].isNull() && root["id"].is<unsigned long>()) {
      extracted_id = root["id"].as<unsigned long>();
    }
    return true;
  });
  return extracted_id;
}

}  // namespace esphome::nerdminer
