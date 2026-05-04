#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"

namespace esphome {
namespace seplos_v3_p4 {

class SeplosComponent : public Component, public uart::UARTDevice {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;

  void set_voltage_sensor(sensor::Sensor *s) { voltage_sensor_ = s; }

 protected:
  sensor::Sensor *voltage_sensor_{nullptr};
  std::vector<uint8_t> rx_buffer_;
};

}  // namespace seplos_v3_p4
}  // namespace esphome
