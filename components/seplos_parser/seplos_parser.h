#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
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

  // Registrazione diretta da Python con bms_index e type
  void register_sensor(int bms_index, const std::string &type, sensor::Sensor *s);

 protected:
  int bms_count_{1};
  uint32_t update_interval_{10000};  
  std::deque<uint8_t> buffer;
  std::vector<uint32_t> last_updates_;

  // Sensori Mappati per BMS
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
