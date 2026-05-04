#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/uart/uart.h"

namespace esphome {
namespace seplos_v3 {

struct SeplosSensor {
  uint8_t address;
  std::string type;
  sensor::Sensor *sensor;
};

class SeplosComponent : public uart::UARTDevice, public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  void register_sensor(uint8_t address, std::string type, sensor::Sensor *s);

 protected:
  void process_byte_(uint8_t byte);
  void decode_pia_(uint8_t address);
  std::vector<uint8_t> rx_buffer_;
  std::vector<SeplosSensor> sensors_;
};

}  // namespace seplos_v3
}  // namespace esphome
