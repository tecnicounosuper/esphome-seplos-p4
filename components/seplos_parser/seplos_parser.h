#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"
#include <vector>
#include <map>
#include <string>

namespace esphome {
namespace seplos_parser {

struct BmsSensors {
  sensor::Sensor *pack_voltage{nullptr};
  sensor::Sensor *current{nullptr};
  sensor::Sensor *soc{nullptr};
  sensor::Sensor *remaining_capacity{nullptr};
};

class SeplosParserHub : public PollingComponent, public uart::UARTDevice {
 public:
  SeplosParserHub() = default;

  void set_bms_count(uint8_t count) { bms_count_ = count; }

  void set_pack_voltage_sensor(uint8_t bms_idx, sensor::Sensor *s) { bms_sensors_[bms_idx].pack_voltage = s; }
  void set_current_sensor(uint8_t bms_idx, sensor::Sensor *s) { bms_sensors_[bms_idx].current = s; }
  void set_soc_sensor(uint8_t bms_idx, sensor::Sensor *s) { bms_sensors_[bms_idx].soc = s; }
  void set_remaining_capacity_sensor(uint8_t bms_idx, sensor::Sensor *s) { bms_sensors_[bms_idx].remaining_capacity = s; }

  void setup() override;
  void loop() override;
  void update() override;
  void dump_config() override;

 protected:
  void parse_rx_buffer_();
  void parse_seplos_ascii_frame_(const std::vector<uint8_t> &frame);
  void parse_seplos_modbus_frame_(const uint8_t *data, size_t len);
  void publish_val_(uint8_t bms_idx, const std::string &type_str, float value);

  uint8_t bms_count_{2};
  std::map<uint8_t, BmsSensors> bms_sensors_;
  std::vector<uint8_t> rx_buffer_;
  uint32_t last_rx_time_{0};
};

}  // namespace seplos_parser
}  // namespace esphome
