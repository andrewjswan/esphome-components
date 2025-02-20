#include "cm1106.h"
#include "esphome/core/log.h"

#include <cinttypes>

namespace esphome {
namespace cm1106 {

uint8_t cm1106_checksum(uint8_t *response, size_t len) {
  uint8_t crc = 0;
  // Last byte of response is checksum, don't calculate it
  for (int i = 0; i < len - 1; i++) {
    crc -= response[i];
  }
  return crc;
}

void CM1106Component::update() {
  uint8_t response[8] = {0};  // response: 0x16, 0x05, 0x01, DF1, DF2, DF3, DF4, CRC. PPM: DF1*256+DF2
  if (!this->cm1106_write_command_(CM1106_CMD_GET_CO2, sizeof(CM1106_CMD_GET_CO2), response, sizeof(response))) {
    ESP_LOGW(TAG, "Reading data from CM1106 failed!");
    this->status_set_warning();
    return;
  }

  if (!(response[0] == 0x16 && response[1] == 0x05 && response[2] == 0x01)) {
    ESP_LOGW(TAG, "Got wrong UART response from CM1106: %02X %02X %02X %02X...", response[0], response[1], response[2],
             response[3]);
    this->status_set_warning();
    return;
  }

  uint8_t checksum = cm1106_checksum(response, sizeof(response));
  if (response[7] != checksum) {
    ESP_LOGW(TAG, "CM1106 Checksum doesn't match: 0x%02X!=0x%02X", response[7], checksum);
    this->status_set_warning();
    return;
  }

  this->status_clear_warning();

  int16_t ppm = response[3] << 8 | response[4];
  ESP_LOGD(TAG, "CM1106 Received CO₂=%uppm DF3=%02X DF4=%02X", ppm, response[5], response[6]);
  if (this->co2_sensor_ != nullptr)
    this->co2_sensor_->publish_state(ppm);
}

void CM1106Component::calibrate_zero(uint16_t ppm) {
  uint8_t cmd[6];
  memcpy(cmd, CM1106_CMD_SET_CO2_CALIB, sizeof(cmd));
  cmd[3] = ppm >> 8;
  cmd[4] = ppm & 0xFF;
  uint8_t response[4] = {0};

  if (!this->cm1106_write_command_(cmd, sizeof(cmd), response, sizeof(response))) {
    ESP_LOGW(TAG, "Reading data from CM1106 failed!");
    this->status_set_warning();
    return;
  }

  // check if correct response received
  if (memcmp(response, CM1106_CMD_SET_CO2_CALIB_RESPONSE, sizeof(response)) != 0) {
    ESP_LOGW(TAG, "Got wrong UART response from CM1106: %02X %02X %02X %02X", response[0], response[1], response[2],
             response[3]);
    this->status_set_warning();
    return;
  }

  this->status_clear_warning();
  ESP_LOGD(TAG, "CM1106 Successfully calibrated sensor to %uppm", ppm);
}

bool CM1106Component::cm1106_write_command_(uint8_t *command, size_t commandLen, uint8_t *response,
                                            size_t responseLen) {
  // Empty RX Buffer
  while (this->available())
    this->read();
  command[commandLen - 1] = cm1106_checksum(command, commandLen);
  this->write_array(command, commandLen);
  this->flush();

  if (response == nullptr)
    return true;

  return this->read_array(response, responseLen);
}

void CM1106Component::dump_config() {
  ESP_LOGCONFIG(TAG, "CM1106:");
  LOG_SENSOR("  ", "CO2", this->co2_sensor_);
  this->check_uart_settings(9600);
}

}  // namespace cm1106
}  // namespace esphome
