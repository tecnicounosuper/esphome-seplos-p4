#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"
#include <vector>
#include <string>

namespace esphome {
namespace seplos_parser {

enum SeplosSensorType {
  TYPE_PACK_VOLTAGE,
  TYPE_CURRENT,
  TYPE_SOC,
  TYPE_REMAINING_CAPACITY,
  TYPE_UNKNOWN
};

class SeplosSensor : public sensor::Sensor {
 public:
  SeplosSensor() = default;
  void set_bms_index(uint8_t idx) { bms_index_ = idx; }
  uint8_t get_bms_index() const { return bms_index_; }

  void set_sensor_type(const std::string &type_str) {
    if (type_str == "pack_voltage") type_ = TYPE_PACK_VOLTAGE;
    else if (type_str == "current") type_ = TYPE_CURRENT;
    else if (type_str == "soc") type_ = TYPE_SOC;
    else if (type_str == "remaining_capacity") type_ = TYPE_REMAINING_CAPACITY;
    else type_ = TYPE_UNKNOWN;
  }
  SeplosSensorType get_sensor_type() const { return type_; }

 protected:
  uint8_t bms_index_{0};
  SeplosSensorType type_{TYPE_UNKNOWN};
};

class SeplosParserHub : public PollingComponent, public uart::UARTDevice {
 public:
  SeplosParserHub() = default;

  void set_bms_count(uint8_t count) { bms_count_ = count; }
  void register_sensor(SeplosSensor *sensor) { sensors_.push_back(sensor); }

  void setup() override;
  void loop() override;
  void update() override;
  void dump_config() override;

 protected:
  void parse_rx_buffer_();
  void parse_seplos_ascii_frame_(const std::vector<uint8_t> &frame);
  void parse_seplos_modbus_frame_(const uint8_t *data, size_t len);
  void publish_val_(uint8_t bms_idx, SeplosSensorType type, float value);

  uint8_t bms_count_{2};
  std::vector<SeplosSensor *> sensors_;
  std::vector<uint8_t> rx_buffer_;
  uint32_t last_rx_time_{0};
};

}  // namespace seplos_parser
}  // namespace esphome
