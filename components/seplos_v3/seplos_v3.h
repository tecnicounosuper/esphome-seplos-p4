#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include <vector>

namespace esphome {
namespace seplos_v3 {

class _SeplosV3 : public Component, public uart::UARTDevice {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;

  void set_address(uint8_t address) { address_ = address; } // Setter per l'indirizzo

  void set_pack_voltage_sensor(sensor::Sensor *s) { pack_voltage_sensor_ = s; }
  void set_current_sensor(sensor::Sensor *s) { current_sensor_ = s; }
  void set_soc_sensor(sensor::Sensor *s) { soc_sensor_ = s; }
  
  void set_cell_sensor(size_t index, sensor::Sensor *s) {
    if (index < 16) cell_sensors_[index] = s;
  }
  void set_cell_temp_sensor(size_t index, sensor::Sensor *s) {
    if (index < 4) cell_temp_sensors_[index] = s;
  }

  void set_system_status_text_sensor(text_sensor::TextSensor *s) { system_status_text_sensor_ = s; }
  void set_fet_status_text_sensor(text_sensor::TextSensor *s) { fet_status_text_sensor_ = s; }

 protected:
  void parse_buffer_();
  
  uint8_t address_{0x01}; // Indirizzo di default
  sensor::Sensor *pack_voltage_sensor_{nullptr};
  sensor::Sensor *current_sensor_{nullptr};
  sensor::Sensor *soc_sensor_{nullptr};
  sensor::Sensor *cell_sensors_[16]{nullptr};
  sensor::Sensor *cell_temp_sensors_[4]{nullptr};

  text_sensor::TextSensor *system_status_text_sensor_{nullptr};
  text_sensor::TextSensor *fet_status_text_sensor_{nullptr};

  std::vector<uint8_t> rx_buffer_;
};

}  // namespace seplos_v3
}  // namespace esphome
