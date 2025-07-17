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
bool verifyPayload(String *line) {
  if (line->length() == 0)
    return false;
  line->trim();
  if (line->isEmpty())
    return false;
  return true;
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
bool tx_mining_subscribe(WiFiClient &client, mining_subscribe &mSubscribe) {
  char payload[BUFFER] = {0};

  // Subscribe
  id = 1;  // Initialize id messages
  sprintf(payload, "{\"id\": %u, \"method\": \"mining.subscribe\", \"params\": [\"NerdMinerV2/%s\"]}\n", id,
          NERDMINER_VERSION);

  ESP_LOGD(TAG, "[STRATUM] ==> Mining subscribe");
  ESP_LOGD(TAG, "  Sending: %s", payload);
  client.print(payload);

  vTaskDelay(200 / portTICK_PERIOD_MS);  // Small delay

  String line = client.readStringUntil('\n');
  if (!parse_mining_subscribe(line, mSubscribe))
    return false;

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

bool parse_mining_subscribe(String line, mining_subscribe &mSubscribe) {
  if (!verifyPayload(&line))
    return false;
  ESP_LOGD(TAG, "  Receiving: %s", line.c_str());

  DeserializationError error = deserializeJson(doc, line);

  if (error || checkError(doc))
    return false;
  if (!doc["result"].is<JsonVariant>())
    return false;

  mSubscribe.sub_details = String((const char *) doc["result"][0][0][1]);
  mSubscribe.extranonce1 = String((const char *) doc["result"][1]);
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
bool tx_mining_auth(WiFiClient &client, const char *user, const char *pass) {
  char payload[BUFFER] = {0};

  // Authorize
  id = getNextId(id);
  sprintf(payload, "{\"params\": [\"%s\", \"%s\"], \"id\": %u, \"method\": \"mining.authorize\"}\n", user, pass, id);

  ESP_LOGD(TAG, "[STRATUM] ==> Autorize work");
  ESP_LOGD(TAG, "  Sending: %s", payload);
  client.print(payload);

  vTaskDelay(200 / portTICK_PERIOD_MS);  // Small delay

  // Don't parse here any answer
  // Miner started to receive mining notifications so better parse all at main thread
  return true;
}

stratum_method parse_mining_method(String line) {
  if (!verifyPayload(&line))
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

bool parse_mining_notify(String line, mining_job &mJob) {
  ESP_LOGD(TAG, "    Parsing Method [MINING NOTIFY]");
  if (!verifyPayload(&line))
    return false;

  DeserializationError error = deserializeJson(doc, line);
  if (error)
    return false;
  if (!doc["params"].is<JsonVariant>())
    return false;

  mJob.job_id = String((const char *) doc["params"][0]);
  mJob.prev_block_hash = String((const char *) doc["params"][1]);
  mJob.coinb1 = String((const char *) doc["params"][2]);
  mJob.coinb2 = String((const char *) doc["params"][3]);
  mJob.merkle_branch = doc["params"][4];
  mJob.version = String((const char *) doc["params"][5]);
  mJob.nbits = String((const char *) doc["params"][6]);
  mJob.ntime = String((const char *) doc["params"][7]);
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

bool tx_mining_submit(WiFiClient &client, mining_subscribe mWorker, mining_job mJob, unsigned long nonce,
                      unsigned long &submit_id) {
  char payload[BUFFER] = {0};

  // Submit
  id = getNextId(id);
  submit_id = id;
  sprintf(payload, "{\"id\": %u, \"method\": \"mining.submit\", \"params\": [\"%s\",\"%s\",\"%s\",\"%s\",\"%s\"]}\n",
          id,
          mWorker.wName,  //"bc1qvv469gmw4zz6qa4u4dsezvrlmqcqszwyfzhgwj", //mWorker.name,
          mJob.job_id.c_str(), mWorker.extranonce2.c_str(), mJob.ntime.c_str(), String(nonce, HEX).c_str());
  ESP_LOGD(TAG, "  Sending: %s", payload);
  client.print(payload);

  return true;
}

bool parse_mining_set_difficulty(String line, double &difficulty) {
  ESP_LOGD(TAG, "    Parsing Method [SET DIFFICULTY]");
  if (!verifyPayload(&line))
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

bool tx_suggest_difficulty(WiFiClient &client, double difficulty) {
  char payload[BUFFER] = {0};

  id = getNextId(id);
  sprintf(payload, "{\"id\": %d, \"method\": \"mining.suggest_difficulty\", \"params\": [%.10g]}\n", id, difficulty);

  ESP_LOGD(TAG, "  Sending: %s", payload);
  return client.print(payload);
}

unsigned long parse_extract_id(const String &line) {
  DeserializationError error = deserializeJson(doc, line);
  if (error) {
    return 0;
  }

  if (!doc["id"].is<JsonVariant>()) {
    return 0;
  }

  unsigned long id = doc["id"];
  return id;
}

}  // namespace nerdminer
}  // namespace esphome
