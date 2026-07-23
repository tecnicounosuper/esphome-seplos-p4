#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"
#include <vector>
#include <deque>
#include <string>

namespace esphome {
namespace seplos_parser {

class SeplosParser : public Component, public uart::UARTDevice {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;

  void set_bms_count(int bms_count);
  void set_update_interval(int update_interval);

  // Registrazione sensore ad 1 argomento, compatibile con sensor.py
  void register_sensor(sensor::Sensor *s) { sensors_.push_back(s); }

 protected:
  int bms_count_{1};
  uint32_t update_interval_{10000};  
  std::deque<uint8_t> buffer;
  std::vector<uint32_t> last_updates_;
  std::vector<sensor::Sensor *> sensors_;

  // Sensori Mappati internamente per ciascun BMS
  std::vector<sensor::Sensor *> pack_voltage_;
  std::vector<sensor::Sensor *> current_;
  std::vector<sensor::Sensor *> soc_;
  std::vector<sensor::Sensor *> remaining_capacity_;

  size_t get_expected_length();
  bool validate_crc();
  void process_packet();
  bool should_update(int bms_index);
};

}  // namespace seplos_parser
}  // namespace esphome
