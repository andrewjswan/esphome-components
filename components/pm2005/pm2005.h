#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/i2c/i2c.h"

namespace esphome {
namespace pm2005 {

class PM2005Component : public PollingComponent, public i2c::I2CDevice {
 public:
  float get_setup_priority() const override { return esphome::setup_priority::DATA; }

  PM2005Component() = default;

  void set_pm_1_0_sensor(sensor::Sensor *pm_1_0_sensor) { pm_1_0_sensor_ = pm_1_0_sensor; }
  void set_pm_2_5_sensor(sensor::Sensor *pm_2_5_sensor) { pm_2_5_sensor_ = pm_2_5_sensor; }
  void set_pm_10_0_sensor(sensor::Sensor *pm_10_0_sensor) { pm_10_0_sensor_ = pm_10_0_sensor; }

  void dump_config() override;
  void update() override;

 protected:
  uint8_t sensor_situation_ = 0;
  uint8_t data_buffer_[12];

  sensor::Sensor *pm_1_0_sensor_{nullptr};
  sensor::Sensor *pm_2_5_sensor_{nullptr};
  sensor::Sensor *pm_10_0_sensor_{nullptr};

  uint16_t get_sensor_value_(const uint8_t *data, uint8_t i);
};

}  // namespace pm2005
}  // namespace esphome
