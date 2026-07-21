#include "seplos_parser.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

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
  ESP_LOGI(TAG, "Inizializzazione Sniffer Seplos per %d BMS...", this->bms_count_);
  last_updates_.resize(bms_count_, 0);
}

void SeplosParser::register_sensor(int bms_index, const std::string &type, sensor::Sensor *s) {
  if (bms_index < 0 || bms_index >= 16) return;

  if (pack_voltage_.size() <= (size_t)bms_index) pack_voltage_.resize(bms_index + 1, nullptr);
  if (current_.size() <= (size_t)bms_index) current_.resize(bms_index + 1, nullptr);
  if (soc_.size() <= (size_t)bms_index) soc_.resize(bms_index + 1, nullptr);
  if (remaining_capacity_.size() <= (size_t)bms_index) remaining_capacity_.resize(bms_index + 1, nullptr);

  if (type == "voltage") pack_voltage_[bms_index] = s;
  else if (type == "current") current_[bms_index] = s;
  else if (type == "soc") soc_[bms_index] = s;
  else if (type == "capacity") remaining_capacity_[bms_index] = s;
}

void SeplosParser::loop() {
  while (this->available() > 0) {
    uint8_t byte;
    this->read_byte(&byte);
    buffer.push_back(byte);

    if (buffer.size() >= 3) {
      if (buffer[0] >= (uint8_t) bms_count_) {
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

    while (buffer.size() > 256) {
      buffer.pop_front();
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
  if (bms_index < 0 || bms_index >= bms_count_ || last_updates_.size() <= (size_t) bms_index) return false;
  uint32_t now = millis();
  if (now - last_updates_[bms_index] >= update_interval_ || last_updates_[bms_index] == 0) {
    last_updates_[bms_index] = now;
    return true;
  }
  return false;
}

void SeplosParser::process_packet() {
  int bms_index = buffer[0];
  if (bms_index < 0 || bms_index >= bms_count_) return;

  uint8_t function_code = buffer[1];
  uint8_t byte_count = buffer[2];

  if ((function_code == 0x03 || function_code == 0x04) && byte_count == 0x24) {
    if (!should_update(bms_index)) return;

    float volt = ((buffer[3] << 8) | buffer[4]) * 0.01f;
    float curr = ((int16_t) ((buffer[5] << 8) | buffer[6])) * 0.01f;
    float rem_cap = ((buffer[7] << 8) | buffer[8]) * 0.01f;
    float soc_val = ((buffer[13] << 8) | buffer[14]) * 0.1f;

    if (bms_index < (int)pack_voltage_.size() && pack_voltage_[bms_index]) pack_voltage_[bms_index]->publish_state(volt);
    if (bms_index < (int)current_.size() && current_[bms_index]) current_[bms_index]->publish_state(curr);
    if (bms_index < (int)remaining_capacity_.size() && remaining_capacity_[bms_index]) remaining_capacity_[bms_index]->publish_state(rem_cap);
    if (bms_index < (int)soc_.size() && soc_[bms_index]) soc_[bms_index]->publish_state(soc_val);
  }
}

void SeplosParser::dump_config() {
  ESP_LOGCONFIG(TAG, "Sniffer Seplos Parser:");
  ESP_LOGCONFIG(TAG, "  BMS Count: %d", this->bms_count_);
  ESP_LOGCONFIG(TAG, "  Update Interval: %lu ms", (unsigned long) this->update_interval_);
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
