#pragma once

#include "esphome/core/component.h"
#include "esphome/core/automation.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/uart/uart.h"

namespace esphome {
namespace cm1106 {

static const char *const TAG = "cm1106";

class CM1106Component : public PollingComponent, public uart::UARTDevice {
 public:
  float get_setup_priority() const override { return esphome::setup_priority::DATA; }

  void update() override;
  void dump_config() override;

  void calibrate_zero(uint16_t ppm);

  void set_co2_sensor(sensor::Sensor *co2_sensor) { co2_sensor_ = co2_sensor; }

 private:
  uint8_t CM1106_CMD_GET_CO2[4] = {0x11, 0x01, 0x01, 0xED};  // head, len, cmd, [data], crc
  uint8_t CM1106_CMD_SET_CO2_CALIB[6] = {0x11, 0x03, 0x03, 0x00, 0x00, 0x00};
  uint8_t CM1106_CMD_SET_CO2_CALIB_RESPONSE[4] = {0x16, 0x01, 0x03, 0xE6};

 protected:
  bool cm1106_write_command_(uint8_t *command, size_t commandLen, uint8_t *response, size_t responseLen);

  sensor::Sensor *co2_sensor_{nullptr};
};

template<typename... Ts> class CM1106CalibrateZeroAction : public Action<Ts...> {
 public:
  CM1106CalibrateZeroAction(CM1106Component *cm1106) : cm1106_(cm1106) {}

  void play(Ts... x) override { this->cm1106_->calibrate_zero(400); }

 protected:
  CM1106Component *cm1106_;
};

}  // namespace cm1106
}  // namespace esphome
