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

  // Registrazione dei sensori da ESPHome
  void register_sensor(sensor::Sensor *s) { sensors_.push_back(s); }
  void register_text_sensor(text_sensor::TextSensor *ts) { text_sensors_.push_back(ts); }
  void register_binary_sensor(binary_sensor::BinarySensor *bs) { binary_sensors_.push_back(bs); }

 protected:
  int bms_count_{1};
  uint32_t update_interval_{10000}; // Default 10 secondi in millisecondi
  std::deque<uint8_t> buffer;
  std::vector<uint32_t> last_updates_;

  // Liste grezze collezionate dal costruttore ESPHome
  std::vector<sensor::Sensor *> sensors_;
  std::vector<text_sensor::TextSensor *> text_sensors_;
  std::vector<binary_sensor::BinarySensor *> binary_sensors_;

  // Sensori Analogici Mappati per BMS (vettori indicizzati per bms_index)
  std::vector<sensor::Sensor *> pack_voltage_;
  std::vector<sensor::Sensor *> current_;
  std::vector<sensor::Sensor *> remaining_capacity_;
  std::vector<sensor::Sensor *> total_capacity_;
  std::vector<sensor::Sensor *> total_discharge_capacity_;
  std::vector<sensor::Sensor *> soc_;
  std::vector<sensor::Sensor *> soh_;
  std::vector<sensor::Sensor *> cycle_count_;
  std::vector<sensor::Sensor *> average_cell_voltage_;
  std::vector<sensor::Sensor *> average_cell_temp_;
  std::vector<sensor::Sensor *> max_cell_voltage_;
  std::vector<sensor::Sensor *> min_cell_voltage_;
  std::vector<sensor::Sensor *> delta_cell_voltage_;
  std::vector<sensor::Sensor *> max_cell_temp_;
  std::vector<sensor::Sensor *> min_cell_temp_;
  std::vector<sensor::Sensor *> maxdiscurt_;
  std::vector<sensor::Sensor *> maxchgcurt_;
  std::vector<sensor::Sensor *> case_temp_;
  std::vector<sensor::Sensor *> power_temp_;

  // Matrici per Celle e Temperature [Numero_Elemento][Indice_BMS]
  std::vector<std::vector<sensor::Sensor *>> cells_;
  std::vector<std::vector<sensor::Sensor *>> temps_;

  // Sensori di Testo Mappati per BMS
  std::vector<text_sensor::TextSensor *> system_status_;
  std::vector<text_sensor::TextSensor *> active_alarm_;

  // NUOVO: Sensori Binari MOSFET Mappati per BMS
  std::vector<binary_sensor::BinarySensor *> chg_mos_status_;
  std::vector<binary_sensor::BinarySensor *> dischg_mos_status_;

  // NUOVO: Matrice Sensori Binari Bilanciamento [Cella_0_fino_15][Indice_BMS]
  std::vector<std::vector<binary_sensor::BinarySensor *>> balancing_status_;

  // Funzioni interne di parsing e instradamento dei pacchetti
  size_t get_expected_length();
  bool validate_crc();
  void process_packet();
  bool should_update(int bms_index);
  void map_sensor_vector(std::vector<sensor::Sensor *> &vec, const std::string &name);
};

}  // namespace seplos_parser
}  // namespace esphome
