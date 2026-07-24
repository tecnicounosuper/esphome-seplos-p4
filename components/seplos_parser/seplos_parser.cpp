#include "seplos_parser.h"
#include "esphome/core/log.h"
#include <cstdio>
#include <cstring>
#include <cstdlib>

namespace esphome {
namespace seplos_parser {

static const char *const TAG = "seplos_parser";

void SeplosParserHub::setup() {
  ESP_LOGI(TAG, "Inizializzazione Seplos V3 Sniffer su UART ESP32-P4...");
  rx_buffer_.reserve(512);
}

void SeplosParserHub::loop() {
  while (available()) {
    uint8_t c;
    read_byte(&c);
    rx_buffer_.push_back(c);
    last_rx_time_ = millis();

    // Rilevamento fine frame ASCII (CR '\r' = 0x0D) o limite buffer
    if (c == 0x0D || rx_buffer_.size() >= 256) {
      parse_rx_buffer_();
      rx_buffer_.clear();
    }
  }

  // Timeout frame (inattivo per più di 50ms)
  if (!rx_buffer_.empty() && (millis() - last_rx_time_ > 50)) {
    parse_rx_buffer_();
    rx_buffer_.clear();
  }
}

void SeplosParserHub::update() {
  ESP_LOGD(TAG, "Heartbeat Sniffer: in ascolto sulla linea RS485...");
}

void SeplosParserHub::parse_rx_buffer_() {
  if (rx_buffer_.size() < 10) return;

  if (rx_buffer_[0] == 0x20 || rx_buffer_[0] == 0x7E) {
    parse_seplos_ascii_frame_(rx_buffer_);
  } else if (rx_buffer_.size() >= 7) {
    parse_seplos_modbus_frame_(rx_buffer_.data(), rx_buffer_.size());
  }
}

void SeplosParserHub::parse_seplos_ascii_frame_(const std::vector<uint8_t> &frame) {
  std::string ascii_str(frame.begin(), frame.end());
  
  if (ascii_str.length() < 20) return;

  char adr_buf[3] = {ascii_str[3], ascii_str[4], '\0'};
  uint8_t bms_idx = (uint8_t) strtol(adr_buf, nullptr, 16);

  ESP_LOGV(TAG, "Frame ASCII intercettato per BMS Index %d", bms_idx);

  if (ascii_str.length() >= 38) {
    std::string v_hex = ascii_str.substr(18, 4);
    uint16_t raw_v = (uint16_t) strtol(v_hex.c_str(), nullptr, 16);
    float voltage = raw_v * 0.01f;
    if (voltage > 30.0f && voltage < 70.0f) {
      publish_val_(bms_idx, "pack_voltage", voltage);
    }

    if (ascii_str.length() >= 26) {
      std::string i_hex = ascii_str.substr(22, 4);
      int16_t raw_i = (int16_t) strtol(i_hex.c_str(), nullptr, 16);
      float current = raw_i * 0.01f;
      publish_val_(bms_idx, "current", current);
    }

    if (ascii_str.length() >= 30) {
      std::string cap_hex = ascii_str.substr(26, 4);
      uint16_t raw_cap = (uint16_t) strtol(cap_hex.c_str(), nullptr, 16);
      float remaining_cap = raw_cap * 0.01f;
      publish_val_(bms_idx, "remaining_capacity", remaining_cap);
    }

    if (ascii_str.length() >= 38) {
      std::string soc_hex = ascii_str.substr(34, 4);
      uint16_t raw_soc = (uint16_t) strtol(soc_hex.c_str(), nullptr, 16);
      float soc = raw_soc * 0.1f;
      if (soc <= 100.0f) {
        publish_val_(bms_idx, "soc", soc);
      }
    }
  }
}

void SeplosParserHub::parse_seplos_modbus_frame_(const uint8_t *data, size_t len) {
  uint8_t bms_idx = data[0];
  uint8_t func = data[1];

  if (bms_idx > 15 || func != 0x03 || len < 15) return;

  uint8_t byte_count = data[2];
  if (byte_count + 5 > len) return;

  uint16_t raw_v = (data[3] << 8) | data[4];
  int16_t raw_i = (data[5] << 8) | data[6];
  uint16_t raw_soc = (data[7] << 8) | data[8];
  uint16_t raw_cap = (data[9] << 8) | data[10];

  float voltage = raw_v * 0.01f;
  float current = raw_i * 0.1f;
  float soc = raw_soc * 0.1f;
  float cap = raw_cap * 0.01f;

  if (voltage > 30.0f && voltage < 70.0f) publish_val_(bms_idx, "pack_voltage", voltage);
  publish_val_(bms_idx, "current", current);
  if (soc <= 100.0f) publish_val_(bms_idx, "soc", soc);
  publish_val_(bms_idx, "remaining_capacity", cap);
}

void SeplosParserHub::publish_val_(uint8_t bms_idx, const std::string &type_str, float value) {
  auto it = bms_sensors_.find(bms_idx);
  if (it == bms_sensors_.end()) return;

  sensor::Sensor *s = nullptr;
  if (type_str == "pack_voltage") s = it->second.pack_voltage;
  else if (type_str == "current") s = it->second.current;
  else if (type_str == "soc") s = it->second.soc;
  else if (type_str == "remaining_capacity") s = it->second.remaining_capacity;

  if (s != nullptr) {
    s->publish_state(value);
  }
}

void SeplosParserHub::dump_config() {
  ESP_LOGCONFIG(TAG, "Seplos Parser Hub (ESP32-P4 Sniffer):");
  ESP_LOGCONFIG(TAG, "  BMS Count: %d", bms_count_);
  ESP_LOGCONFIG(TAG, "  BMS Mappati: %zu", bms_sensors_.size());
}

}  // namespace seplos_parser
}  // namespace esphome
