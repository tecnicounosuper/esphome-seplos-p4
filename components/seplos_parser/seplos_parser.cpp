#include "seplos_parser.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include <algorithm>
#include <string>

namespace esphome {
namespace seplos_parser {

static const char *TAG = "seplos_parser.component";

uint16_t chk_crc16(const uint8_t *data, uint16_t len) {
  uint16_t crc = 0xFFFF;
  for (uint16_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x0001) {
        crc = (crc >> 1) ^ 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

void SeplosParser::setup() {
  ESP_LOGI(TAG, "Inizializzazione Sniffer Seplos Avanzato per %d BMS...", this->bms_count_);

  pack_voltage_.resize(bms_count_, nullptr);
  current_.resize(bms_count_, nullptr);
  remaining_capacity_.resize(bms_count_, nullptr);
  total_capacity_.resize(bms_count_, nullptr);
  total_discharge_capacity_.resize(bms_count_, nullptr);
  soc_.resize(bms_count_, nullptr);
  soh_.resize(bms_count_, nullptr);
  cycle_count_.resize(bms_count_, nullptr);
  average_cell_voltage_.resize(bms_count_, nullptr);
  average_cell_temp_.resize(bms_count_, nullptr);
  max_cell_voltage_.resize(bms_count_, nullptr);
  min_cell_voltage_.resize(bms_count_, nullptr);
  delta_cell_voltage_.resize(bms_count_, nullptr);
  max_cell_temp_.resize(bms_count_, nullptr);
  min_cell_temp_.resize(bms_count_, nullptr);
  maxdiscurt_.resize(bms_count_, nullptr);
  maxchgcurt_.resize(bms_count_, nullptr);
  case_temp_.resize(bms_count_, nullptr);
  power_temp_.resize(bms_count_, nullptr);
  system_status_.resize(bms_count_, nullptr);
  active_alarm_.resize(bms_count_, nullptr);

  chg_mos_status_.resize(bms_count_, nullptr);
  dischg_mos_status_.resize(bms_count_, nullptr);

  cells_.resize(16, std::vector<sensor::Sensor *>(bms_count_, nullptr));
  temps_.resize(4, std::vector<sensor::Sensor *>(bms_count_, nullptr));
  balancing_status_.resize(16, std::vector<binary_sensor::BinarySensor *>(bms_count_, nullptr));

  map_sensor_vector(pack_voltage_, "pack_voltage");
  map_sensor_vector(current_, "current");
  map_sensor_vector(remaining_capacity_, "remaining_capacity");
  map_sensor_vector(total_capacity_, "total_capacity");
  map_sensor_vector(total_discharge_capacity_, "total_discharge_capacity");
  map_sensor_vector(soc_, "soc");
  map_sensor_vector(soh_, "soh");
  map_sensor_vector(cycle_count_, "cycle_count");
  map_sensor_vector(average_cell_voltage_, "average_cell_voltage");
  map_sensor_vector(average_cell_temp_, "average_cell_temp");
  map_sensor_vector(max_cell_voltage_, "max_cell_voltage");
  map_sensor_vector(min_cell_voltage_, "min_cell_voltage");
  map_sensor_vector(delta_cell_voltage_, "delta_cell_voltage");
  map_sensor_vector(max_cell_temp_, "max_cell_temp");
  map_sensor_vector(min_cell_temp_, "min_cell_temp");
  map_sensor_vector(maxdiscurt_, "maxdiscurt");
  map_sensor_vector(maxchgcurt_, "maxchgcurt");
  map_sensor_vector(case_temp_, "case_temp");
  map_sensor_vector(power_temp_, "power_temp");

  for (int c = 0; c < 16; c++) {
    map_sensor_vector(cells_[c], "cell_" + std::to_string(c + 1));
  }
  for (int t = 0; t < 4; t++) {
    map_sensor_vector(temps_[t], "cell_temp_" + std::to_string(t + 1));
  }

  for (int i = 0; i < bms_count_; i++) {
    std::string expected_status = "bms" + std::to_string(i) + " system_status";
    std::string expected_alarm = "bms" + std::to_string(i) + " active_alarms";
    for (auto *ts : this->text_sensors_) {
      if (ts->get_name() == expected_status) system_status_[i] = ts;
      if (ts->get_name() == expected_alarm) active_alarm_[i] = ts;
    }

    std::string expected_chg = "bms" + std::to_string(i) + " mos_carica";
    std::string expected_dis = "bms" + std::to_string(i) + " mos_scarica";
    for (auto *bs : this->binary_sensors_) {
      if (bs->get_name() == expected_chg) chg_mos_status_[i] = bs;
      if (bs->get_name() == expected_dis) dischg_mos_status_[i] = bs;
    }

    for (int c = 0; c < 16; c++) {
      std::string expected_bal = "bms" + std::to_string(i) + " cella_" + std::to_string(c + 1) + "_bilanciamento";
      for (auto *bs : this->binary_sensors_) {
        if (bs->get_name() == expected_bal) {
          balancing_status_[c][i] = bs;
        }
      }
    }
  }
}

void SeplosParser::map_sensor_vector(std::vector<sensor::Sensor *> &vec, const std::string &name) {
  for (int i = 0; i < bms_count_; i++) {
    std::string expected_name = "bms" + std::to_string(i) + " " + name;
    for (auto *sensor : this->sensors_) {
      if (sensor->get_name() == expected_name) {
        vec[i] = sensor;
      }
    }
  }
}

void SeplosParser::loop() {
  while (this->available() > 0) {
    uint8_t byte;
    this->read_byte(&byte);
    buffer.push_back(byte);

    if (buffer.size() >= 3) {
      if (buffer[0] < 0x01 || buffer[0] > 0x10) {
        buffer.pop_front();
        continue;
      }

      size_t expected_len = get_expected_length();
      if (expected_len == 0) {
        buffer.pop_front();
        continue;
      }

      if (buffer.size() >= expected_len) {
        if (validate_crc()) {
          process_packet();
          buffer.erase(buffer.begin(), buffer.begin() + expected_len);
        } else {
          buffer.pop_front();
        }
      }
    }
  }
}

size_t SeplosParser::get_expected_length() {
  uint8_t func = buffer[1];
  uint8_t byte_count = buffer[2];
  
  if (func == 0x04 || func == 0x03) {
    if (byte_count == 0x24) return 41; 
    if (byte_count == 0x34) return 57; 
    if (byte_count == 0x11) return 22; 
    if (byte_count == 0x10) return 21; 
    // Aggiunto supporto flessibile per altre varianti di byte_count dei limiti
    if (byte_count == 0x0E) return 19;
    if (byte_count == 0x06) return 11;
  }
  if (func == 0x01) {
    if (byte_count == 0x12) return 23; 
  }
  return 0;
}

bool SeplosParser::validate_crc() {
  size_t len = get_expected_length();
  if (len == 0 || buffer.size() < len) return false;
  std::vector<uint8_t> local_data(buffer.begin(), buffer.begin() + len);
  uint16_t computed_crc = chk_crc16(local_data.data(), len - 2);
  uint16_t received_crc = (local_data[len - 1] << 8) | local_data[len - 2];
  return computed_crc == received_crc;
}

bool SeplosParser::should_update(int bms_index) {
  if (bms_index < 0 || bms_index >= bms_count_) return false;
  uint32_t now = millis();
  if (now - last_updates_[bms_index] >= update_interval_ || last_updates_[bms_index] == 0) {
    last_updates_[bms_index] = now;
    return true;
  }
  return false;
}

void SeplosParser::process_packet() {
  int bms_index = buffer[0] - 0x01;
  if (bms_index < 0 || bms_index >= bms_count_) return;

  uint8_t function_code = buffer[1];
  uint8_t byte_count = buffer[2];

  if ((function_code == 0x03 || function_code == 0x04) && byte_count == 0x24) {
    if (!should_update(bms_index)) return;

    float volt = ((buffer[3] << 8) | buffer[4]) * 0.01f;
    float curr = ((int16_t)((buffer[5] << 8) | buffer[6])) * 0.01f;
    float rem_cap = ((buffer[7] << 8) | buffer[8]) * 0.01f;
    float tot_cap = ((buffer[9] << 8) | buffer[10]) * 0.01f;
    float discharge_cap = ((buffer[11] << 8) | buffer[12]) * 0.01f;
    float soc_val = ((buffer[13] << 8) | buffer[14]) * 0.1f;
    float soh_val = ((buffer[15] << 8) | buffer[16]) * 0.1f;
    uint16_t cycles = (buffer[17] << 8) | buffer[18];
    float avg_cell_v = ((buffer[19] << 8) | buffer[20]) * 0.001f;
    float avg_cell_t = (((buffer[21] << 8) | buffer[22]) - 2731) * 0.1f;

    if (pack_voltage_[bms_index]) pack_voltage_[bms_index]->publish_state(volt);
    if (current_[bms_index]) current_[bms_index]->publish_state(curr);
    if (remaining_capacity_[bms_index]) remaining_capacity_[bms_index]->publish_state(rem_cap);
    if (total_capacity_[bms_index]) total_capacity_[bms_index]->publish_state(tot_cap);
    if (total_discharge_capacity_[bms_index]) total_discharge_capacity_[bms_index]->publish_state(discharge_cap);
    if (soc_[bms_index]) soc_[bms_index]->publish_state(soc_val);
    if (soh_[bms_index]) soh_[bms_index]->publish_state(soh_val);
    if (cycle_count_[bms_index]) cycle_count_[bms_index]->publish_state(cycles);
    if (average_cell_voltage_[bms_index]) average_cell_voltage_[bms_index]->publish_state(avg_cell_v);
    if (average_cell_temp_[bms_index]) average_cell_temp_[bms_index]->publish_state(avg_cell_t);
  }

  if ((function_code == 0x03 || function_code == 0x04) && byte_count == 0x34) {
    int idx = 3;
    float min_c_v = 5.0f;
    float max_c_v = 0.0f;
    
    for (int c = 0; c < 16; c++) {
      float cell_v = ((buffer[idx] << 8) | buffer[idx + 1]) * 0.001f;
      if (cells_[c][bms_index]) cells_[c][bms_index]->publish_state(cell_v);
      if (cell_v < min_c_v && cell_v > 0.5f) min_c_v = cell_v;
      if (cell_v > max_c_v) max_c_v = cell_v;
      idx += 2;
    }

    float min_t_v = 200.0f;
    float max_t_v = -50.0f;
    for (int t = 0; t < 4; t++) {
      float temp_v = (((buffer[idx] << 8) | buffer[idx + 1]) - 2731) * 0.1f;
      if (temps_[t][bms_index]) temps_[t][bms_index]->publish_state(temp_v);
      if (temp_v < min_t_v) min_t_v = temp_v;
      if (temp_v > max_t_v) max_t_v = temp_v;
      idx += 2;
    }

    float delta_v = max_c_v - min_c_v;
    if (delta_cell_voltage_[bms_index]) delta_cell_voltage_[bms_index]->publish_state(delta_v);
    if (max_cell_voltage_[bms_index]) max_cell_voltage_[bms_index]->publish_state(max_c_v);
    if (min_cell_voltage_[bms_index]) min_cell_voltage_[bms_index]->publish_state(min_c_v);
    if (max_cell_temp_[bms_index]) max_cell_temp_[bms_index]->publish_state(max_t_v);
    if (min_cell_temp_[bms_index]) min_cell_temp_[bms_index]->publish_state(min_t_v);
    if (case_temp_[bms_index]) case_temp_[bms_index]->publish_state(min_t_v);
    if (power_temp_[bms_index]) power_temp_[bms_index]->publish_state(max_t_v);
  }

  // MIGLIORIA: Riconoscimento ed estrazione ottimizzata per limiti di corrente dinamici
  if ((function_code == 0x03 || function_code == 0x04) && 
      (byte_count == 0x11 || byte_count == 0x10 || byte_count == 0x0E || byte_count == 0x06)) {
    
    float max_chg = 0.0f;
    float max_dis = 0.0f;

    if (byte_count == 0x11) {
      max_chg = ((buffer[10] << 8) | buffer[11]) * 0.1f;
      max_dis = ((buffer[12] << 8) | buffer[13]) * 0.1f;
    } else if (byte_count == 0x10) {
      max_chg = ((buffer[9] << 8) | buffer[10]) * 0.1f;
      max_dis = ((buffer[11] << 8) | buffer[12]) * 0.1f;
    } else if (byte_count == 0x0E) {
      max_chg = ((buffer[7] << 8) | buffer[8]) * 0.1f;
      max_dis = ((buffer[9] << 8) | buffer[10]) * 0.1f;
    } else if (byte_count == 0x06) {
      max_chg = ((buffer[3] << 8) | buffer[4]) * 0.1f;
      max_dis = ((buffer[5] << 8) | buffer[6]) * 0.1f;
    }

    if (maxchgcurt_[bms_index] && max_chg > 0.0f) maxchgcurt_[bms_index]->publish_state(max_chg);
    if (maxdiscurt_[bms_index] && max_dis > 0.0f) maxdiscurt_[bms_index]->publish_state(max_dis);
  }

  if (function_code == 0x01 && byte_count == 0x12) {
    bool has_critical_alarm = false;
    for (int i = 3; i < 11; i++) {
      if (buffer[i] != 0x00) {
        has_critical_alarm = true;
        break;
      }
    }

    if (system_status_[bms_index]) system_status_[bms_index]->publish_state(has_critical_alarm ? "Allarme / Protezione" : "Normale");
    if (active_alarm_[bms_index]) active_alarm_[bms_index]->publish_state(has_critical_alarm ? "Attivo (Vedi BMS)" : "Non Attivo");

    uint8_t mos_byte = buffer[15]; 
    bool chg_mos_on = (mos_byte & 0x01);     
    bool dischg_mos_on = (mos_byte & 0x02);  

    if (chg_mos_status_[bms_index]) chg_mos_status_[bms_index]->publish_state(chg_mos_on);
    if (dischg_mos_status_[bms_index]) dischg_mos_status_[bms_index]->publish_state(dischg_mos_on);

    uint16_t balancing_bits = (buffer[13] << 8) | buffer[14];
    for (int c = 0; c < 16; c++) {
      bool is_balancing = (balancing_bits & (1 << c));
      if (balancing_status_[c][bms_index]) {
        balancing_status_[c][bms_index]->publish_state(is_balancing);
      }
    }
  }
}

void SeplosParser::dump_config() {
  ESP_LOGCONFIG(TAG, "Sniffer Seplos Parser:");
  ESP_LOGCONFIG(TAG, "  BMS Count: %d", this->bms_count_);
  ESP_LOGCONFIG(TAG, "  Update Interval: %u ms", this->update_interval_);
}

void SeplosParser::set_bms_count(int bms_count) {
  this->bms_count_ = bms_count;
  last_updates_.resize(bms_count, 0);
}

void SeplosParser::set_update_interval(int update_interval) {
  this->update_interval_ = update_interval * 1000;
}

}  // namespace seplos_parser
}  // namespace esphome
