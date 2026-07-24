#include "seplos_parser.h"
#include "esphome/core/log.h"
#include <cstdio>
#include <cstring>

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
  // Lo sniffer è passivo e ascolta la UART a 19200 baud.
  // Invia uno heartbeat nei log.
  ESP_LOGD(TAG, "Heartbeat Sniffer: in ascolto sulla linea RS485...");
}

void SeplosParserHub::parse_rx_buffer_() {
  if (rx_buffer_.size() < 10) return;

  // Controllo protocollo ASCII Seplos V3 (inizia con 0x20 "~" o 0x20 ' ')
  if (rx_buffer_[0] == 0x20 || rx_buffer_[0] == 0x7E) {
    parse_seplos_ascii_frame_(rx_buffer_);
  } else if (rx_buffer_.size() >= 7) {
    // Prova Modbus RTU (Address 0..15, Function 0x03)
    parse_seplos_modbus_frame_(rx_buffer_.data(), rx_buffer_.size());
  }
}

void SeplosParserHub::parse_seplos_ascii_frame_(const std::vector<uint8_t> &frame) {
  // Converti in stringa per facilitare il parsing ASCII
  std::string ascii_str(frame.begin(), frame.end());
  
  // Esempio frame Seplos V3 ASCII: "~20004642..." o " 20004642..."
  // Offset 1..2: SOI/VER, Offset 3..4: ADR (BMS address in Hex, e.g. "00", "01")
  if (ascii_str.length() < 20) return;

  char adr_buf[3] = {ascii_str[3], ascii_str[4], '\0'};
  uint8_t bms_idx = (uint8_t) strtol(adr_buf, nullptr, 16);

  ESP_LOGV(TAG, "Frame ASCII intercettato per BMS Index %d", bms_idx);

  // Seplos V3 ASCII payload parsing (Valori standard in risposta 0x42 telemetry)
  // Esempio posizioni per Telemetry Data standard Seplos V3:
  // Pack Voltage, Current, Remaining Cap, Total Cap, SOC
  // Estrazione di sicurezza dimostrativa standard:
  if (ascii_str.length() >= 60) {
    // In un frame di risposta standard Seplos V3 Telemetry:
    // Voltage in mV (es. 0xCF20 = 53024 mV -> 53.02 V)
    // Current in 10mA (es. signed short)
    // SOC in 0.1% (es. 0x03E8 = 1000 -> 100.0%)
    
    // Per dimostrazione, la decodifica applica i parser hex a blocchi:
    try {
      // Estrai tensione pacco (4 caratteri hex)
      std::string v_hex = ascii_str.substr(18, 4);
      uint16_t raw_v = (uint16_t) strtol(v_hex.c_str(), nullptr, 16);
      float voltage = raw_v * 0.01f; // 0.01V unità
      if (voltage > 30.0f && voltage < 70.0f) {
        publish_val_(bms_idx, TYPE_PACK_VOLTAGE, voltage);
      }

      // Estrai corrente (4 caratteri hex signed)
      std::string i_hex = ascii_str.substr(22, 4);
      int16_t raw_i = (int16_t) strtol(i_hex.c_str(), nullptr, 16);
      float current = raw_i * 0.01f;
      publish_val_(bms_idx, TYPE_CURRENT, current);

      // Estrai capacità residua (4 caratteri hex)
      std::string cap_hex = ascii_str.substr(26, 4);
      uint16_t raw_cap = (uint16_t) strtol(cap_hex.c_str(), nullptr, 16);
      float remaining_cap = raw_cap * 0.01f;
      publish_val_(bms_idx, TYPE_REMAINING_CAPACITY, remaining_cap);

      // Estrai SOC (4 caratteri hex)
      std::string soc_hex = ascii_str.substr(34, 4);
      uint16_t raw_soc = (uint16_t) strtol(soc_hex.c_str(), nullptr, 16);
      float soc = raw_soc * 0.1f;
      if (soc <= 100.0f) {
        publish_val_(bms_idx, TYPE_SOC, soc);
      }
    } catch (...) {
      ESP_LOGW(TAG, "Errore parsing frame ASCII Seplos V3");
    }
  }
}

void SeplosParserHub::parse_seplos_modbus_frame_(const uint8_t *data, size_t len) {
  uint8_t bms_idx = data[0];
  uint8_t func = data[1];

  if (bms_idx > 15 || func != 0x03 || len < 15) return;

  uint8_t byte_count = data[2];
  if (byte_count + 5 > len) return;

  // Lettura registri Modbus Seplos V3
  // Reg 0: Voltage (0.1V o 0.01V)
  uint16_t raw_v = (data[3] << 8) | data[4];
  int16_t raw_i = (data[5] << 8) | data[6];
  uint16_t raw_soc = (data[7] << 8) | data[8];
  uint16_t raw_cap = (data[9] << 8) | data[10];

  float voltage = raw_v * 0.01f;
  float current = raw_i * 0.1f;
  float soc = raw_soc * 0.1f;
  float cap = raw_cap * 0.01f;

  if (voltage > 30.0f && voltage < 70.0f) publish_val_(bms_idx, TYPE_PACK_VOLTAGE, voltage);
  publish_val_(bms_idx, TYPE_CURRENT, current);
  if (soc <= 100.0f) publish_val_(bms_idx, TYPE_SOC, soc);
  publish_val_(bms_idx, TYPE_REMAINING_CAPACITY, cap);
}

void SeplosParserHub::publish_val_(uint8_t bms_idx, SeplosSensorType type, float value) {
  for (auto *s : sensors_) {
    if (s->get_bms_index() == bms_idx && s->get_sensor_type() == type) {
      s->publish_state(value);
    }
  }
}

void SeplosParserHub::dump_config() {
  ESP_LOGCONFIG(TAG, "Seplos Parser Hub (ESP32-P4 Sniffer):");
  ESP_LOGCONFIG(TAG, "  BMS Count: %d", bms_count_);
  ESP_LOGCONFIG(TAG, "  Sensori Registrati: %zu", sensors_.size());
}

}  // namespace seplos_parser
}  // namespace esphome
