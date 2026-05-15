#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/uart/uart_device.h"
#include <map>

namespace esphome {
namespace seplos_v3 {

class SeplosV3 : public PollingComponent, public uart::UARTDevice {
 public:
  void setup() override;
  void update() override;
  void loop() override;
  void dump_config() override;

  void register_sensor(uint8_t address, std::string type, sensor::Sensor *obj);

 protected:
  struct BmsData {
    std::map<std::string, sensor::Sensor *> sensors;
  };
  std::map<uint8_t, BmsData> bms_list_;
};

}  // namespace seplos_v3
}  // namespace esphome
