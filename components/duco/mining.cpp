#include "counter.h"
#include "duco.h"
#include "mining.h"

#include "esphome/components/socket/socket.h" 
#include "esphome/core/defines.h"
#include "esphome/core/log.h"

#include <algorithm>

#if defined(ESP32)
#include <esp_task_wdt.h>
#endif

namespace esphome::duco {

// https://github.com/esp8266/Arduino/blob/master/cores/esp8266/TypeConversion.cpp
constexpr char base36Chars[36] = {
    '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 
    'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P', 'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z'
};

constexpr uint8_t base36CharValues[75] = {
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 0, 0, 0, 0, 0, 0,                                                                        // 0 to 9
    10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 0, 0, 0, 0, 0, 0, // Upper case letters
    10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35                    // Lower case letters
};

#define SPC_TOKEN ' '
#define END_TOKEN '\n'
#define SEP_TOKEN ','
#define IOT_TOKEN '@'

MiningJob::MiningJob(uint8_t core, MiningConfig *config, Duco *parent) {
    this->core = core;
    this->config = config;
    this->parent = parent;
    this->client_buffer = "";
    dsha1 = new DSHA1();
    dsha1->warmup();
}

void MiningJob::mine() {
    if (!this->config->is_ready) return;

    connectToNode();
    if (!this->is_connected) return;

    askForJob();
    if (this->client_buffer.empty() || this->client_buffer == "???\n") return;
      
    dsha1->reset().write((const unsigned char *)this->last_block_hash.c_str(), this->last_block_hash.length());

    int start_time = micros();
    max_micros_elapsed(start_time, 0);

    for (Counter<10> counter; counter < this->difficulty; ++counter) {
        DSHA1 ctx = *dsha1;
        ctx.write((const unsigned char *)counter.c_str(), counter.strlen()).finalize(this->hashArray);
        
        #if defined(ESP32)
            #define SYSTEM_TIMEOUT 10000 // 10ms for ESP32 to prevent watchdog panics
        #else
            #define SYSTEM_TIMEOUT 50000 // 50ms fallback
        #endif
        if (max_micros_elapsed(micros(), SYSTEM_TIMEOUT)) {
            handleSystemEvents();
        } 

        if (memcmp(this->expected_hash, this->hashArray, 20) == 0) {
            unsigned long elapsed_micros  = micros() - start_time;
            float elapsed_time_s = elapsed_micros * 0.000001f;
            this->share_count++;

            float current_hashrate = 0.0f;
            if (elapsed_time_s > 0.0f) {
                current_hashrate = counter / elapsed_time_s;
            } else {
                // Fallback for instant shares to prevent divide-by-zero or fake crazy values
                current_hashrate = counter; 
            }
            this->hashrate = current_hashrate;
            submit(counter, current_hashrate, elapsed_time_s);

            if (this->parent != nullptr) {
                this->parent->on_share_found_callback();
            }

            break; // Share found, exit loop to request the next job
        }
    }
}

bool MiningJob::max_micros_elapsed(unsigned long current, unsigned long max_elapsed) {
    if ((current - this->last_micros_checkpoint) > max_elapsed) {
        this->last_micros_checkpoint = current;
        return true;
    }
    return false;
}

void MiningJob::handleSystemEvents(void) {
    // Reset the FreeRTOS Task Watchdog directly via ESP-IDF API if available
    #if defined(ESP32) && defined(CONFIG_ESP_TASK_WDT_EN)
      esp_task_wdt_reset();
    #endif
    
    #if defined(ESP32)
      // Critical: Suspend the current thread for 10ms to let ESPHome run its automation loops.
      // pdMS_TO_TICKS(10) ensures that the tick-rate conversion is accurate across different ESP32 variants.
      vTaskDelay(pdMS_TO_TICKS(10)); 
    #else
      // ESP8266
      delay(10);
    #endif
}

uint8_t *MiningJob::hexStringToUint8Array(const std::string &hexString, uint8_t *uint8Array, const uint32_t arrayLength) {
    if (hexString.length() < arrayLength * 2) {
        return uint8Array;
    }

    const char *hexChars = hexString.c_str();
    for (uint32_t i = 0; i < arrayLength; ++i) {
        uint8_t high = base36CharValues[hexChars[i * 2] - '0'];
        uint8_t low  = base36CharValues[hexChars[i * 2 + 1] - '0'];
        uint8Array[i] = (high << 4) + low;
    }
    return uint8Array;
}


void MiningJob::connectToNode() {
    if (!this->config->is_ready) return;
    if (this->is_connected && this->client_sock) return;

    uint32_t stopWatch = millis();
    ESP_LOGD(TAG, "Core [%d] - Connecting to a Duino-Coin node...", this->core);

    while (true) {
        // Create a native blockable ESPHome socket
        this->client_sock = esphome::socket::socket_ip(SOCK_STREAM, 0);
        this->client_sock->setblocking(true); 

        struct sockaddr_in server_addr;
        socklen_t len = esphome::socket::set_sockaddr(
            (struct sockaddr*)&server_addr, sizeof(server_addr), this->config->host.c_str(), this->config->port
        );

        // Try to connect using ESPHome socket API
        if (this->client_sock->connect((struct sockaddr*)&server_addr, len) == 0) {
            // Connection successful!
            this->is_connected = true;
            break; 
        }

        // If connection failed, log it, free the socket pointer and sleep for 2 seconds
        ESP_LOGW(TAG, "Core [%d] - Connection failed, retrying in 2 seconds...", this->core);
        this->client_sock.reset();
        vTaskDelay(pdMS_TO_TICKS(2000));

        // Global watchdog protection: if node is totally down for 100 seconds, reseting configuration
        if (millis() - stopWatch > 100000) {
            ESP_LOGE(TAG, "Core [%d] - Critical connection timeout. Resetting configuration...", this->core);
            this->config->is_ready = false; // Invalidate config so loop() can fetch a new one if it recovers
            vTaskDelay(pdMS_TO_TICKS(100)); // Give time for logs to print
            return;
        }
    }

    
    waitForClientData();
    if (!this->is_connected) {
        this->config->is_ready = false;
        return;
    }

    ESP_LOGI(TAG, "Core [%d] - Connected successfully. Node reported version: %s", this->core, this->client_buffer.c_str());

    /* 
    std::string motd_req = "MOTD" + END_TOKEN;
    this->client_sock->send(motd_req.c_str(), motd_req.length(), 0);
    waitForClientData();
    ESP_LOGD(TAG, "Core [%d] - MOTD: %s", this->core, this->client_buffer.c_str());
    */
}

void MiningJob::waitForClientData() {
    this->client_buffer = "";
    char c;
    uint32_t stopWatch = millis();
    
    if (!this->config->is_ready) return;
    if (!this->is_connected || !this->client_sock) return;

    while (this->is_connected && this->client_sock) {
        int res = this->client_sock->read(&c, 1); 
        
        if (res > 0) {
            if (c == END_TOKEN) { 
                if (this->client_buffer.length() == 0) {
                    this->client_buffer = "???\n"; // NOTE: Should never happen
                }
                break;
            }
            this->client_buffer += c;
        } else if (res == 0) {
            ESP_LOGW(TAG, "Core [%d] - Socket disconnected by pool while waiting for data.", this->core);
            this->is_connected = false;
            this->client_sock.reset();
            this->config->is_ready = false;
            break;
        } else {
            if (errno != EAGAIN && errno != EWOULDBLOCK) {
                ESP_LOGE(TAG, "Core [%d] - Socket read error (errno: %d).", this->core, errno);
                this->is_connected = false;
                this->client_sock.reset();
                this->config->is_ready = false;
                break;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1));

        if (millis() - stopWatch > 120000) {
            ESP_LOGE(TAG, "Core [%d] - Timeout after 120s. Invalidating connection...", this->core);
            this->is_connected = false;
            this->client_sock.reset();
            this->config->is_ready = false;
            break;
        }
    }
}

void MiningJob::submit(unsigned long counter, float hashrate, float elapsed_time_s) {
    if (!this->config->is_ready) return;
    if (!this->is_connected || !this->client_sock) return;

    std::string reply = std::to_string(counter);
    reply += SEP_TOKEN;
    reply += std::to_string(hashrate);
    reply += SEP_TOKEN;
    reply += this->config->MINER_BANNER;
    reply += SPC_TOKEN;
    reply += this->config->MINER_VER;
    reply += SEP_TOKEN;
    reply += this->config->RIG_IDENTIFIER;
    reply += SEP_TOKEN;
    reply += "DUCOID" + this->config->chip_id;
    reply += SEP_TOKEN;
    reply += std::to_string(this->config->WALLET_ID);
    reply += END_TOKEN;

    int sent_bytes = this->client_sock->send(reply.c_str(), reply.length(), 0);
    
    if (sent_bytes < 0) {
        ESP_LOGE(TAG, "Core [%d] - Failed to send share to pool.", this->core);
        this->is_connected = false;
        this->client_sock.reset();
        this->config->is_ready = false;
        return;
    }

    unsigned long ping_start = millis();
    waitForClientData();
    this->ping = millis() - ping_start;

    bool is_good = (this->client_buffer.find("GOOD") != std::string::npos);
    if (is_good) {
        this->accepted_share_count++;
    }

    ESP_LOGD(TAG, "Core [%d] - %s share #%d (%lu) hashrate: %.2f kH/s (%.2fs) Ping: %lums (%s)",
             this->core,
             this->client_buffer.c_str(),
             this->share_count.load(),
             counter,
             hashrate / 1000.0f,
             elapsed_time_s,
             this->ping.load(),
             this->config->node_id.c_str());
    this->client_buffer.clear(); 
}

bool MiningJob::parse() {
    if (this->client_buffer.empty() || this->client_buffer == "???\n") {
        ESP_LOGE(TAG, "Core [%d] - Cannot parse empty or invalid client buffer", this->core);
        return false;
    }

    this->client_buffer.erase(std::remove(this->client_buffer.begin(), this->client_buffer.end(), '\r'), this->client_buffer.end());
    this->client_buffer.erase(std::remove(this->client_buffer.begin(), this->client_buffer.end(), '\n'), this->client_buffer.end());

    std::vector<std::string> tokens;
    size_t start = 0;
    size_t end = this->client_buffer.find(SEP_TOKEN);

    while (end != std::string::npos) {
        tokens.push_back(this->client_buffer.substr(start, end - start));
        start = end + 1;
        end = this->client_buffer.find(SEP_TOKEN, start);
    }
    tokens.push_back(this->client_buffer.substr(start));

    // 3 token: last_hash, expected_hash, difficulty
    if (tokens.size() < 3) {
        ESP_LOGE(TAG, "Core [%d] - Parsing failed. Expected 3 tokens, got %d. Buffer: %s", 
                 this->core, tokens.size(), this->client_buffer.c_str());
        return false;
    }

    this->last_block_hash = tokens[0] + ","; 
    this->expected_hash_str = tokens[1];

    hexStringToUint8Array(this->expected_hash_str, this->expected_hash, 20);

    char* endptr;
    unsigned long parsed_diff = std::strtoul(tokens[2].c_str(), &endptr, 10);
    if (endptr == tokens[2].c_str()) {
        ESP_LOGE(TAG, "Core [%d] - Difficulty token '%s' is not a valid number", this->core, tokens[2].c_str());
        return false;
    }
    this->difficulty = (static_cast<uint32_t>(parsed_diff) * 100) + 1;

    return true;
}

void MiningJob::askForJob() {
    if (!this->is_connected || !this->client_sock) return;

    ESP_LOGI(TAG, "Core [%d] - Asking for a new job for user: %s", this->core, this->config->DUCO_USER.c_str());

    // Базовая часть запроса, одинаковая для всех режимов
    std::string job_req = "JOB";
    job_req += SEP_TOKEN;
    job_req += this->config->DUCO_USER;
    job_req += SEP_TOKEN;
    job_req += this->config->START_DIFF;
    job_req += SEP_TOKEN;
    job_req += this->config->MINER_KEY;

    // Блок условной компиляции для датчиков (добавляем телеметрию, если она включена)
    #if defined(USE_DS18B20)
        sensors.requestTemperatures(); 
        float temp = sensors.getTempCByIndex(0);
        ESP_LOGD(TAG, "DS18B20 reading: %.2f°C", temp);
        
        job_req += SEP_TOKEN;
        job_req += "Temp:" + std::to_string(temp) + "*C";

    #elif defined(USE_DHT)
        float temp = dht.readTemperature();
        float hum = dht.readHumidity();
        ESP_LOGD(TAG, "DHT reading: %.2f°C, Humidity: %.2f%%", temp, hum);

        job_req += SEP_TOKEN;
        job_req += "Temp:" + std::to_string(temp) + "*C";
        job_req += IOT_TOKEN;
        job_req += "Hum:" + std::to_string(hum) + "%";

    #elif defined(USE_HSU07M)
        float temp = read_hsu07m();
        ESP_LOGD(TAG, "HSU reading: %.2f°C", temp);

        job_req += SEP_TOKEN;
        job_req += "Temp:" + std::to_string(temp) + "*C";

    #elif defined(USE_INTERNAL_SENSOR)
        float temp = 0;
        temp_sensor_read_celsius(&temp);
        ESP_LOGD(TAG, "Internal temp sensor reading: %.2f°C", temp);

        job_req += SEP_TOKEN;
        job_req += "CPU Temp:" + std::to_string(temp) + "*C";
    #endif

    job_req += END_TOKEN;

    int sent_bytes = this->client_sock->send(job_req.c_str(), job_req.length(), 0);

    if (sent_bytes < 0) {
        ESP_LOGE(TAG, "Core [%d] - Failed to send job request to pool.", this->core);
        this->is_connected = false;
        this->client_sock.reset();
        this->config->is_ready = false;
        return;
    }

    waitForClientData();

    ESP_LOGD(TAG, "Core [%d] - Received job with size of %d bytes: %s", 
             this->core, this->client_buffer.length(), this->client_buffer.c_str());

    if (parse()) {
        ESP_LOGD(TAG, "Core [%d] - Parsed job: %s %s %lu", 
                 this->core, 
                 this->last_block_hash.c_str(), 
                 this->expected_hash_str.c_str(), 
                 this->difficulty.load());
    } else {
        ESP_LOGE(TAG, "Core [%d] - Job parsing failed!", this->core);
    }
}

}  // namespace esphome::duco
