#include "esphome/core/log.h"
#include "pm2005.h"

namespace esphome {
namespace pm2005 {

static const char *const TAG = "pm2005";

#ifdef TYPE_2005
static const uint8_t SITUATION_VALUE_INDEX = 3;
static const uint8_t PM_1_0_VALUE_INDEX = 4;
static const uint8_t PM_2_5_VALUE_INDEX = 6;
static const uint8_t PM_10_0_VALUE_INDEX = 8;
static const uint8_t MEASURING_VALUE_INDEX = 10;
#else
static const uint8_t SITUATION_VALUE_INDEX = 2;
static const uint8_t PM_1_0_VALUE_INDEX = 3;
static const uint8_t PM_2_5_VALUE_INDEX = 5;
static const uint8_t PM_10_0_VALUE_INDEX = 7;
static const uint8_t MEASURING_VALUE_INDEX = 9;
#endif

void PM2005Component::update() {
  if (this->read(data_buffer_, 12) != i2c::ERROR_OK) {
    ESP_LOGW(TAG, "Read result failed");
    this->status_set_warning();
    return;
  }

  if (sensor_situation_ != data_buffer_[SITUATION_VALUE_INDEX]) {
    sensor_situation_ = data_buffer_[SITUATION_VALUE_INDEX];
    if (sensor_situation_ == 1)
      ESP_LOGD(TAG, "Sensor situation: Close.");
    else if (sensor_situation_ == 2) {
      ESP_LOGD(TAG, "Sensor situation: Malfunction.");
      this->status_set_warning();
    } else if (sensor_situation_ == 3)
      ESP_LOGD(TAG, "Sensor situation: Under detecting.");
    else if (sensor_situation_ == 0x80) {
      ESP_LOGD(TAG, "Sensor situation: Detecting completed.");

      if (this->pm_1_0_sensor_ != nullptr) {
        int16_t pm1 = get_sensor_value_(data_buffer_, PM_1_0_VALUE_INDEX);
        ESP_LOGD(TAG, "PM1.0: %d", pm1);
        this->pm_1_0_sensor_->publish_state(pm1);
      }

      if (this->pm_2_5_sensor_ != nullptr) {
        int16_t pm25 = get_sensor_value_(data_buffer_, PM_2_5_VALUE_INDEX);
        ESP_LOGD(TAG, "PM2.5: %d", pm25);
        this->pm_2_5_sensor_->publish_state(pm25);
      }

      if (this->pm_10_0_sensor_ != nullptr) {
        int16_t pm10 = get_sensor_value_(data_buffer_, PM_10_0_VALUE_INDEX);
        ESP_LOGD(TAG, "PM10: %d", pm10);
        this->pm_10_0_sensor_->publish_state(pm10);
      }

      uint16_t sensor_measuring_mode = get_sensor_value_(data_buffer_, MEASURING_VALUE_INDEX);
      ;
      if (sensor_measuring_mode == 2)
        ESP_LOGD(TAG, "The measuring mode of sensor: Single measuring mode.");
      else if (sensor_measuring_mode == 3)
        ESP_LOGD(TAG, "The measuring mode of sensor: Continuous measuring mode.");
      else if (sensor_measuring_mode == 5)
        ESP_LOGD(TAG, "The measuring mode of sensor: Dynamic measuring mode.");

      this->status_clear_warning();
    }
  }
}

uint16_t PM2005Component::get_sensor_value_(const uint8_t *data, uint8_t i) {
  return data_buffer_[i] * 0x100 + data_buffer_[i + 1];
}

void PM2005Component::dump_config() {
  ESP_LOGCONFIG(TAG, "PM2005:");

#ifdef TYPE_2005
  ESP_LOGCONFIG(TAG, "Type: PM2005");
#else
  ESP_LOGCONFIG(TAG, "Type: PM2105");
#endif

  LOG_I2C_DEVICE(this);
  LOG_SENSOR("  ", "PM1.0", this->pm_1_0_sensor_);
  LOG_SENSOR("  ", "PM2.5", this->pm_2_5_sensor_);
  LOG_SENSOR("  ", "PM10 ", this->pm_10_0_sensor_);
}

}  // namespace pm2005
}  // namespace esphome
