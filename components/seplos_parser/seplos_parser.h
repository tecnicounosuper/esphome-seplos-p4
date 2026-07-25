#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"
#include <vector>
#include <map>
#include <string>
#include <array>

namespace esphome {
namespace seplos_parser {

struct BmsSensors {
  sensor::Sensor *pack_voltage{nullptr};
  sensor::Sensor *current{nullptr};
  sensor::Sensor *soc{nullptr};
  sensor::Sensor *soh{nullptr};
  sensor::Sensor *remaining_capacity{nullptr};
  sensor::Sensor *cycles{nullptr};
  sensor::Sensor *min_cell_voltage{nullptr};
  sensor::Sensor *max_cell_voltage{nullptr};
  sensor::Sensor *cell_delta_voltage{nullptr};
  sensor::Sensor *temp1{nullptr};
  sensor::Sensor *temp2{nullptr};
  sensor::Sensor *temp3{nullptr};
  sensor::Sensor *temp4{nullptr};
  sensor::Sensor *mos_temp{nullptr};
  std::array<sensor::Sensor*, 16> cells{nullptr};
};

class SeplosParserHub : public PollingComponent, public uart::UARTDevice {
 public:
  SeplosParserHub() = default;

  void set_bms_count(uint8_t count) { bms_count_ = count; }

  void set_pack_voltage_sensor(uint8_t bms_idx, sensor::Sensor *s) { bms_sensors_[bms_idx].pack_voltage = s; }
  void set_current_sensor(uint8_t bms_idx, sensor::Sensor *s) { bms_sensors_[bms_idx].current = s; }
  void set_soc_sensor(uint8_t bms_idx, sensor::Sensor *s) { bms_sensors_[bms_idx].soc = s; }
  void set_soh_sensor(uint8_t bms_idx, sensor::Sensor *s) { bms_sensors_[bms_idx].soh = s; }
  void set_remaining_capacity_sensor(uint8_t bms_idx, sensor::Sensor *s) { bms_sensors_[bms_idx].remaining_capacity = s; }
  void set_cycles_sensor(uint8_t bms_idx, sensor::Sensor *s) { bms_sensors_[bms_idx].cycles = s; }
  void set_min_cell_voltage_sensor(uint8_t bms_idx, sensor::Sensor *s) { bms_sensors_[bms_idx].min_cell_voltage = s; }
  void set_max_cell_voltage_sensor(uint8_t bms_idx, sensor::Sensor *s) { bms_sensors_[bms_idx].max_cell_voltage = s; }
  void set_cell_delta_voltage_sensor(uint8_t bms_idx, sensor::Sensor *s) { bms_sensors_[bms_idx].cell_delta_voltage = s; }
  void set_temp1_sensor(uint8_t bms_idx, sensor::Sensor *s) { bms_sensors_[bms_idx].temp1 = s; }
  void set_temp2_sensor(uint8_t bms_idx, sensor::Sensor *s) { bms_sensors_[bms_idx].temp2 = s; }
  void set_temp3_sensor(uint8_t bms_idx, sensor::Sensor *s) { bms_sensors_[bms_idx].temp3 = s; }
  void set_temp4_sensor(uint8_t bms_idx, sensor::Sensor *s) { bms_sensors_[bms_idx].temp4 = s; }
  void set_mos_temp_sensor(uint8_t bms_idx, sensor::Sensor *s) { bms_sensors_[bms_idx].mos_temp = s; }

  // Setter per le 16 celle
  void set_cell_1_sensor(uint8_t bms_idx, sensor::Sensor *s) { bms_sensors_[bms_idx].cells[0] = s; }
  void set_cell_2_sensor(uint8_t bms_idx, sensor::Sensor *s) { bms_sensors_[bms_idx].cells[1] = s; }
  void set_cell_3_sensor(uint8_t bms_idx, sensor::Sensor *s) { bms_sensors_[bms_idx].cells[2] = s; }
  void set_cell_4_sensor(uint8_t bms_idx, sensor::Sensor *s) { bms_sensors_[bms_idx].cells[3] = s; }
  void set_cell_5_sensor(uint8_t bms_idx, sensor::Sensor *s) { bms_sensors_[bms_idx].cells[4] = s; }
  void set_cell_6_sensor(uint8_t bms_idx, sensor::Sensor *s) { bms_sensors_[bms_idx].cells[5] = s; }
  void set_cell_7_sensor(uint8_t bms_idx, sensor::Sensor *s) { bms_sensors_[bms_idx].cells[6] = s; }
  void set_cell_8_sensor(uint8_t bms_idx, sensor::Sensor *s) { bms_sensors_[bms_idx].cells[7] = s; }
  void set_cell_9_sensor(uint8_t bms_idx, sensor::Sensor *s) { bms_sensors_[bms_idx].cells[8] = s; }
  void set_cell_10_sensor(uint8_t bms_idx, sensor::Sensor *s) { bms_sensors_[bms_idx].cells[9] = s; }
  void set_cell_11_sensor(uint8_t bms_idx, sensor::Sensor *s) { bms_sensors_[bms_idx].cells[10] = s; }
  void set_cell_12_sensor(uint8_t bms_idx, sensor::Sensor *s) { bms_sensors_[bms_idx].cells[11] = s; }
  void set_cell_13_sensor(uint8_t bms_idx, sensor::Sensor *s) { bms_sensors_[bms_idx].cells[12] = s; }
  void set_cell_14_sensor(uint8_t bms_idx, sensor::Sensor *s) { bms_sensors_[bms_idx].cells[13] = s; }
  void set_cell_15_sensor(uint8_t bms_idx, sensor::Sensor *s) { bms_sensors_[bms_idx].cells[14] = s; }
  void set_cell_16_sensor(uint8_t bms_idx, sensor::Sensor *s) { bms_sensors_[bms_idx].cells[15] = s; }

  void setup() override;
  void loop() override;
  void update() override;
  void dump_config() override;

 protected:
  void parse_rx_buffer_();
  void parse_seplos_ascii_frame_(const std::vector<uint8_t> &frame);
  void parse_seplos_modbus_frame_(const uint8_t *data, size_t len);
  void publish_val_(uint8_t bms_idx, const std::string &type_str, float value);
  void publish_cell_val_(uint8_t bms_idx, uint8_t cell_idx, float value);

  uint8_t bms_count_{2};
  std::map<uint8_t, BmsSensors> bms_sensors_;
  std::vector<uint8_t> rx_buffer_;
  uint32_t last_rx_time_{0};
};

}  // namespace seplos_parser
}  // namespace esphome
