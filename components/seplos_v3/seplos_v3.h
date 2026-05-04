#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"
#include <vector>

namespace esphome {
namespace seplos_v3 {

struct SeplosSensor {
  uint8_t address;
  std::string type;
  sensor::Sensor *sensor_obj;
};

class SeplosComponent : public Component, public uart::UARTDevice {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  void register_sensor(uint8_t address, const std::string &type, sensor::Sensor *s);

 protected:
  void process_byte_(uint8_t byte);
  std::vector<uint8_t> rx_buffer_;
  std::vector<SeplosSensor> sensors_;
};

}  // namespace seplos_v3
}  // namespace esphome
