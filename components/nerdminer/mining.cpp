#include "esphome.h"
#include <ArduinoJson.h>
#include <esp_task_wdt.h>
#include "nerdSHA256plus.h"
#include "stratum.h"
#include "mining.h"
#include "utils.h"
#include "monitor.h"
#include "timeconst.h"

namespace esphome {
namespace nerdminer {

uint32_t templates = 0;
uint32_t hashes = 0;
uint32_t Mhashes = 0;
uint32_t totalKHashes = 0;
uint32_t elapsedKHs = 0;
uint64_t upTime = 0;

uint32_t shares;  // increase if blockhash has 32 bits of zeroes
uint32_t valids;  // increased if blockhash <= target

// Track best diff
double best_diff = 0.0;

unsigned long mElapsed;

IPAddress serverIP(1, 1, 1, 1);  // Temporally save poolIPaddres

// Global work data
static WiFiClient client;
static miner_data mMiner;  // Global miner data (Create a miner class TODO)
mining_subscribe mWorker;
mining_job mJob;
monitor_data mMonitor;
bool isMinerSuscribed = false;
unsigned long mLastTXtoPool = millis();

bool checkPoolConnection(void) {
  if (client.connected()) {
    return true;
  }

  isMinerSuscribed = false;
  ESP_LOGD(TAG, "Client not connected, trying to connect...");

  // Resolve first time pool DNS and save IP
  if (serverIP == IPAddress(1, 1, 1, 1)) {
    WiFi.hostByName(NERDMINER_POOL, serverIP);
    ESP_LOGD(TAG, "Resolved DNS and save ip (first time) got: %s", serverIP.toString().c_str());
  }

  // Try connecting pool IP
  if (!client.connect(serverIP, NERDMINER_POOL_PORT)) {
    ESP_LOGW(TAG, "Imposible to connect to: %s", NERDMINER_POOL);
    WiFi.hostByName(NERDMINER_POOL, serverIP);
    ESP_LOGD(TAG, "Resolved DNS got: %s", serverIP.toString().c_str());
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    return false;
  }

  return true;
}

// Implements a socketKeepAlive function and
// checks if pool is not sending any data to reconnect again.
// Even connection could be alive, pool could stop sending new job NOTIFY
unsigned long mStart0Hashrate = 0;
bool checkPoolInactivity(unsigned int keepAliveTime, unsigned long inactivityTime) {
  unsigned long currentKHashes = (Mhashes * 1000) + hashes / 1000;
  unsigned long elapsedKHs = currentKHashes - totalKHashes;

  // If no shares sent to pool
  // send something to pool to hold socket oppened
  if (millis() - mLastTXtoPool > keepAliveTime) {
    mLastTXtoPool = millis();
    ESP_LOGD(TAG, "Sending: KeepAlive - suggest_difficulty");
    tx_suggest_difficulty(client, DEFAULT_DIFFICULTY);
  }

  if (elapsedKHs == 0) {
    // Check if hashrate is 0 during inactivityTIme
    if (mStart0Hashrate == 0)
      mStart0Hashrate = millis();
    if ((millis() - mStart0Hashrate) > inactivityTime) {
      mStart0Hashrate = 0;
      return true;
    }
    return false;
  }

  mStart0Hashrate = 0;
  return false;
}

void runStratumWorker(void *name) {
  // TEST: https://bitcoin.stackexchange.com/questions/22929/full-example-data-for-scrypt-stratum-client
  ESP_LOGCONFIG(TAG, "[STRATUM] Started. Running %s on core %d", (char *) name, xPortGetCoreID());

#ifdef DEBUG_MEMORY
  ESP_LOGD(TAG, "### [Total Heap / Free heap / Min free heap]: %d / %d / %d", ESP.getHeapSize(), ESP.getFreeHeap(),
           ESP.getMinFreeHeap());
#endif

  // Connect to pool
  double currentPoolDifficulty = DEFAULT_DIFFICULTY;

  while (true) {
    if (WiFi.status() != WL_CONNECTED) {
      // WiFi is disconnected, so reconnect now
      mMonitor.NerdStatus = NM_Connecting;
      vTaskDelay(5000 / portTICK_PERIOD_MS);
      continue;
    }

    if (!checkPoolConnection()) {
      // If server is not reachable add random delay for connection retries
      srand(millis());
      // Generate value between 1 and 120 secs
      vTaskDelay(((1 + rand() % 120) * 1000) / portTICK_PERIOD_MS);
    }

    if (!isMinerSuscribed) {
      // Stop miner current jobs
      mMiner.inRun = false;
      mWorker = init_mining_subscribe();

      // STEP 1: Pool server connection (SUBSCRIBE)
      if (!tx_mining_subscribe(client, mWorker)) {
        client.stop();
        continue;
      }

      // STEP 2: Pool authorize work (Block Info)
      strcpy(mWorker.wName, NERDMINER_POOL_WORKER);
      strcpy(mWorker.wPass, NERDMINER_POOL_PASS);
      tx_mining_auth(client, mWorker.wName, mWorker.wPass);  // Don't verifies authoritzation, TODO
      // tx_mining_auth2(client, mWorker.wName, mWorker.wPass); //Don't verifies authoritzation, TODO

      // STEP 3: Suggest pool difficulty
      tx_suggest_difficulty(client, DEFAULT_DIFFICULTY);

      isMinerSuscribed = true;
      mLastTXtoPool = millis();
    }

    // Check if pool is down for almost 5minutes and then restart connection with pool (1min=600000ms)
    if (checkPoolInactivity(KEEPALIVE_TIME_ms, POOLINACTIVITY_TIME_ms)) {
      // Restart connection
      ESP_LOGI(TAG, "Detected more than 2 min without data form stratum server. Closing socket and reopening...");
      client.stop();
      isMinerSuscribed = false;
      continue;
    }

    // Read pending messages from pool
    while (client.connected() && client.available()) {
      ESP_LOGD(TAG, "Received message from pool...");
      String line = client.readStringUntil('\n');
      stratum_method result = parse_mining_method(line);
      switch (result) {
        case STRATUM_PARSE_ERROR:
          ESP_LOGD(TAG, "  Parsed JSON: error on JSON");
          break;
        case MINING_NOTIFY:
          if (parse_mining_notify(line, mJob)) {
            // Increse templates readed
            templates++;
            // Stop miner current jobs
            mMiner.inRun = false;
            // Prepare data for new jobs
            mMiner = calculateMiningData(mWorker, mJob);
            mMiner.poolDifficulty = currentPoolDifficulty;
            mMiner.newJob = true;
            mMiner.newJob2 = true;
            // Give new job to miner
          }
          break;
        case MINING_SET_DIFFICULTY:
          parse_mining_set_difficulty(line, currentPoolDifficulty);
          mMiner.poolDifficulty = currentPoolDifficulty;
          break;
        case STRATUM_SUCCESS:
          ESP_LOGD(TAG, "  Parsed JSON: Success");
          break;
        default:
          ESP_LOGD(TAG, "  Parsed JSON: Unknown");
          break;
      }
    }

    vTaskDelay(500 / portTICK_PERIOD_MS);  // Small delay
  }
}

////////////////// THREAD CALLS ///////////////////
// This works only with one thread, TODO -> Class or miner_data for each thread
void runMiner(void *task_id) {
  unsigned int miner_id = (uint32_t) task_id;

  ESP_LOGCONFIG(TAG, "[MINER] %d Started...", miner_id);

  while (true) {
    // Wait new job
    while (true) {
      if (mMiner.newJob == true || mMiner.newJob2 == true)
        break;
      vTaskDelay(100 / portTICK_PERIOD_MS);  // Small delay
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);  // Small delay to join both mining threads

    if (mMiner.newJob)
      mMiner.newJob = false;  // Clear newJob flag
    else if (mMiner.newJob2)
      mMiner.newJob2 = false;  // Clear newJob flag
    mMiner.inRun = true;       // Set inRun flag
    mMonitor.NerdStatus = NM_Hashing;

    // Prepare Premining data
    nerdSHA256_context nerdMidstate;  // NerdShaPlus
    uint8_t hash[32];

    // Calcular midstate
    nerd_mids(&nerdMidstate, mMiner.bytearray_blockheader);  // NerdShaPlus

    // Search a valid nonce
    unsigned long nonce = TARGET_NONCE - MAX_NONCE;
    // Split up odd/even nonces between miner tasks
    nonce += miner_id;
    uint32_t startT = micros();
    unsigned char *header64;
    // Each miner thread needs to track its own blockheader template
    uint8_t temp;

    memcpy(mMiner.bytearray_blockheader2, &mMiner.bytearray_blockheader, 80);
    if (miner_id == 0)
      header64 = mMiner.bytearray_blockheader + 64;
    else
      header64 = mMiner.bytearray_blockheader2 + 64;

    bool is16BitShare = true;
    ESP_LOGD(TAG, "[MINER] >>> STARTING TO HASH NONCES");
    while (true) {
      if (miner_id == 0)
        memcpy(mMiner.bytearray_blockheader + 76, &nonce, 4);
      else
        memcpy(mMiner.bytearray_blockheader2 + 76, &nonce, 4);

      // nerd_double_sha2(&nerdMidstate, header64, hash);
      is16BitShare = nerd_sha256d(&nerdMidstate, header64, hash);  // Boosted 80Khs SHA

      /*
      ESP_LOGD(TAG, "hash1: %s", format_hex_pretty(hash, 32).c_str());
      ESP_LOGD(TAG, "hash2: %s", format_hex_pretty(hash2, 32).c_str());
      */

      hashes++;
      if (nonce > TARGET_NONCE)
        break;  // exit
      if (!mMiner.inRun) {
        ESP_LOGD(TAG, "[MINER] WORK ABORTED >> Waiting new job...");
        break;
      }

      // check if 16bit share
      if (hash[31] != 0 || hash[30] != 0) {
        // if(!is16BitShare) {
        // increment nonce
        nonce += 2;
        continue;
      }

      // Check target to submit
      // Difficulty of 1 > 0x00000000FFFF0000000000000000000000000000000000000000000000000000
      // NM2 pool diff 1e-9 > Target = diff_1 / diff_pool > 0x00003B9ACA00....00
      // Swapping diff bytes little endian >>>>>>>>>>>>>>>> 0x0000DC59D300....00
      // if((hash[29] <= 0xDC) && (hash[28] <= 0x59))     //0x00003B9ACA00  > diff value for 1e-9
      double diff_hash = diff_from_target(hash);

      // update best diff
      if (diff_hash > best_diff)
        best_diff = diff_hash;

      if (diff_hash > mMiner.poolDifficulty) {  // (hash[29] <= 0x3B) // (diff_hash > 1e-9)
        tx_mining_submit(client, mWorker, mJob, nonce);
        ESP_LOGD(TAG, "  - Current diff share: %f", diff_hash);
        ESP_LOGD(TAG, "  - Current pool diff: %f", mMiner.poolDifficulty);
        ESP_LOGD(TAG, "  - TX SHARE: %s", esphome::format_hex_pretty(hash, 32).c_str());
#ifdef DEBUG_MINING
        ESP_LOGD(TAG, "  - Current nonce: %d", nonce);
        ESP_LOGD(TAG, "  - Current block header: %s",
                 esphome::format_hex_pretty(mMiner.bytearray_blockheader, 80).c_str());
#endif
        mLastTXtoPool = millis();
      }

      // Check if 32bit share
      if (hash[29] != 0 || hash[28] != 0) {
        // Increment nonce
        nonce += 2;
        continue;
      }
      shares++;

      // Check if valid header
      if (checkValid(hash, mMiner.bytearray_target)) {
        ESP_LOGI(TAG, "[MINER] %d CONGRATULATIONS! Valid block found with nonce: %d | 0x%x", miner_id, nonce, nonce);
        valids++;
        ESP_LOGI(TAG, "[MINER] %d Submitted work valid!", miner_id);
        // Wait for new job
        break;
      }

      // Increment nonce
      nonce += 2;
    }  // exit if found a valid result or nonce > MAX_NONCE

    // wc_Sha256Free(&sha256);
    // wc_Sha256Free(midstate);

    mMiner.inRun = false;
    ESP_LOGD(TAG, "[MINER] >>> Finished job waiting new data from pool");

    if (hashes >= MAX_NONCE_STEP) {
      Mhashes = Mhashes + MAX_NONCE_STEP / 1000000;
      hashes = hashes - MAX_NONCE_STEP;
    }

    uint32_t duration = micros() - startT;
    if (esp_task_wdt_reset() == ESP_OK)
      ESP_LOGD(TAG, "[MINER] >>> Resetting watchdog timer");
  }
}

#define DELAY 100
#define REDRAW_EVERY 10

void resetStat() {
  ESP_LOGD(TAG, "[MONITOR] Resetting stats");
  templates = hashes = Mhashes = totalKHashes = elapsedKHs = upTime = shares = valids = 0;
  best_diff = 0.0;
}

void runMonitor(void *name) {
  mMonitor.Status = false;

  ESP_LOGCONFIG(TAG, "[MONITOR] Started...");

  unsigned long mLastCheck = 0;
  unsigned long frame = 0;

  totalKHashes = (Mhashes * 1000) + hashes / 1000;
  while (1) {
    if ((frame % REDRAW_EVERY) == 0) {
      mMonitor.Status = client.connected() && isMinerSuscribed;

      mElapsed = millis() - mLastCheck;
      mLastCheck = millis();
      unsigned long currentKHashes = (Mhashes * 1000) + hashes / 1000;
      elapsedKHs = currentKHashes - totalKHashes;
      totalKHashes = currentKHashes;

      // Monitor state when hashrate is 0.0
      if (elapsedKHs == 0) {
        ESP_LOGD(TAG,
                 ">>> [MONITOR] Miner: newJob>%s / inRun>%s) - Client: connected>%s / subscribed>%s / wificonnected>%s",
                 YESNO(mMiner.newJob), YESNO(mMiner.inRun), YESNO(client.connected()), YESNO(isMinerSuscribed),
                 YESNO(WiFi.status() == WL_CONNECTED));
      }

#ifdef DEBUG_MEMORY
      ESP_LOGD(TAG, "### [Total Heap / Free heap / Min free heap]: %d / %d / %d", ESP.getHeapSize(), ESP.getFreeHeap(),
               ESP.getMinFreeHeap());
      ESP_LOGD(TAG, "### Max stack usage: %d", uxTaskGetStackHighWaterMark(NULL));
#endif
    }

    vTaskDelay(DELAY / portTICK_PERIOD_MS);
    frame++;
  }
}

}  // namespace nerdminer
}  // namespace esphome
