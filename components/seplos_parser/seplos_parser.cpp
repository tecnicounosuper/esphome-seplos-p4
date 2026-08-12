#include "seplos_parser.h"
#include "esphome/core/log.h"
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <algorithm>

namespace esphome {
namespace seplos_parser {

static const char *const TAG = "seplos_parser";

void SeplosParserHub::setup() {
  ESP_LOGI(TAG, "Inizializzazione Seplos V3 Sniffer su UART ESP32-P4 (19200 Baud)...");
  rx_buffer_.reserve(512);
}

void SeplosParserHub::loop() {
  while (available()) {
    uint8_t c;
    read_byte(&c);
    rx_buffer_.push_back(c);
    last_rx_time_ = millis();

    // Fine frame ASCII (CR '\r' = 0x0D) o limite buffer
    if (c == 0x0D || rx_buffer_.size() >= 300) {
      parse_rx_buffer_();
      rx_buffer_.clear();
    }
  }

  // Timeout frame (inattività > 50ms)
  if (!rx_buffer_.empty() && (millis() - last_rx_time_ > 50)) {
    parse_rx_buffer_();
    rx_buffer_.clear();
  }
}

void SeplosParserHub::update() {
  uint32_t now = millis();
  if (active_polling_ || (now - last_rx_time_ > 8000)) {
    static uint8_t poll_adr = 0;
    uint8_t adr = poll_adr % bms_count_;
    poll_adr++;

    if (adr == 0) {
      ESP_LOGI(TAG, "Polling Attivo Seplos V3 per BMS 1 Master (ADR 0x00)...");
      this->write_str("~20004642E002000D000D010D020000FDA9\r");
    } else {
      ESP_LOGI(TAG, "Polling Attivo Seplos V3 per BMS 2 Slave (ADR 0x01)...");
      this->write_str("~20014642E002000D000D010D020000FDA8\r");
    }
  } else {
    ESP_LOGD(TAG, "Heartbeat Sniffer: ascolto passivo attivo sulla linea RS485 Seplos V3...");
  }
}

void SeplosParserHub::parse_rx_buffer_() {
  if (rx_buffer_.empty()) return;

  // 1. ISOLAMENTO FRAME ASCII (Se inizia con '~' o contiene frame ASCII)
  std::vector<uint8_t> ascii_chars;
  bool ascii_found = false;
  for (uint8_t b : rx_buffer_) {
    if (b == 0x7E) ascii_found = true;
    if (ascii_found) {
      if (b == 0x7E || b == 0x20 || (b >= '0' && b <= '9') || (b >= 'A' && b <= 'F') || (b >= 'a' && b <= 'f') || b == 0x0D || b == 0x0A) {
        ascii_chars.push_back(b);
      }
    }
  }

  // 2. PARSING FRAME ASCII SEPLOS V3 (~20ADR...)
  if (ascii_found && ascii_chars.size() >= 15) {
    parse_seplos_ascii_frame_(ascii_chars);
    return; // Se è un frame ASCII, NON eseguire il parser Modbus per evitare falsi positivi!
  }

  // 3. PARSING STREAM MODBUS RTU CON CRC16
  parse_seplos_modbus_frame_(rx_buffer_.data(), rx_buffer_.size());
}

void SeplosParserHub::parse_seplos_ascii_frame_(const std::vector<uint8_t> &frame) {
  std::string ascii_str(frame.begin(), frame.end());
  
  if (ascii_str.length() < 15) return;

  uint8_t raw_adr = 0;
  if (ascii_str.length() >= 5) {
    char adr_buf[3] = {ascii_str[3], ascii_str[4], 0};
    raw_adr = (uint8_t) strtol(adr_buf, nullptr, 16);
  }

  uint8_t bms_idx = raw_adr;
  if (bms_sensors_.find(bms_idx) == bms_sensors_.end() && raw_adr > 0 && bms_sensors_.find(raw_adr - 1) != bms_sensors_.end()) {
    bms_idx = raw_adr - 1;
  }

  ESP_LOGV(TAG, "Frame ASCII Seplos V3 per ADR 0x%02X -> BMS Index %d", raw_adr, bms_idx);

  // Decodifica Telemetria Generale (Tensione, Corrente, Cap, SOC)
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
      if (soc <= 100.0f && soc >= 0.0f) {
        publish_val_(bms_idx, "soc", soc);
      }
    }
  }

  // Decodifica Celle 16S e Temperature (se presenti nel frame CID2=0x42/0x44)
  if (ascii_str.length() >= 110) {
    float min_v = 99.0f;
    float max_v = 0.0f;
    bool cells_found = false;

    // 16 celle in Hex (4 cifre ciascuna, ad es. 0D00 = 3328mV = 3.328V)
    for (int c = 0; c < 16; c++) {
      size_t pos = 38 + (c * 4);
      if (pos + 4 <= ascii_str.length()) {
        std::string cell_hex = ascii_str.substr(pos, 4);
        uint16_t cell_mv = (uint16_t) strtol(cell_hex.c_str(), nullptr, 16);
        float cell_v = cell_mv * 0.001f;

        if (cell_v > 2.0f && cell_v < 4.5f) {
          publish_cell_val_(bms_idx, c, cell_v);
          if (cell_v < min_v) min_v = cell_v;
          if (cell_v > max_v) max_v = cell_v;
          cells_found = true;
        }
      }
    }

    if (cells_found && max_v >= min_v) {
      publish_val_(bms_idx, "min_cell_voltage", min_v);
      publish_val_(bms_idx, "max_cell_voltage", max_v);
      publish_val_(bms_idx, "cell_delta_voltage", (max_v - min_v) * 1000.0f); // in mV
    }
  }
}

void SeplosParserHub::parse_seplos_modbus_frame_(const uint8_t *data, size_t len) {
  if (len < 10) return;

  for (size_t i = 0; i < len - 8; i++) {
    uint8_t raw_addr = data[i];
    uint8_t func = data[i + 1];
    uint8_t byte_count = data[i + 2];

    // Rigido controllo Modbus RTU (Funzione 0x03 o 0x04 e Indirizzo BMS <= 16)
    if ((func == 0x03 || func == 0x04) && (raw_addr >= 1 && raw_addr <= 16) && (i + byte_count + 5 <= len)) {
      const uint8_t *payload = &data[i + 3];

      uint8_t bms_idx = raw_addr - 1; // Mappatura 1-based (Modbus ADR 1 -> BMS 0, ADR 2 -> BMS 1)

      if (byte_count == 0x34 && (i + 53 <= len)) {
        // Blocco Seplos 26 registri (52 byte): 16 celle + temp + telemetria
        float min_v = 99.0f;
        float max_v = 0.0f;
        bool cells_valid = false;

        for (int c = 0; c < 16; c++) {
          uint16_t cell_mv = (uint16_t)((payload[c * 2] << 8) | payload[c * 2 + 1]);
          float cell_v = cell_mv * 0.001f;
          if (cell_v > 2.0f && cell_v < 4.5f) {
            publish_cell_val_(bms_idx, c, cell_v);
            if (cell_v < min_v) min_v = cell_v;
            if (cell_v > max_v) max_v = cell_v;
            cells_valid = true;
          }
        }

        if (cells_valid && max_v >= min_v) {
          publish_val_(bms_idx, "min_cell_voltage", min_v);
          publish_val_(bms_idx, "max_cell_voltage", max_v);
          publish_val_(bms_idx, "cell_delta_voltage", (max_v - min_v) * 1000.0f);
        }

        // Temperature 4 sonde ( offset Kelvin/0.1C )
        float t1 = ((int16_t)((payload[32] << 8) | payload[33]) - 2731) * 0.1f;
        float t2 = ((int16_t)((payload[34] << 8) | payload[35]) - 2731) * 0.1f;
        float t3 = ((int16_t)((payload[36] << 8) | payload[37]) - 2731) * 0.1f;
        float t4 = ((int16_t)((payload[38] << 8) | payload[39]) - 2731) * 0.1f;
        if (t1 > -30.0f && t1 < 100.0f) publish_val_(bms_idx, "temp_1", t1);
        if (t2 > -30.0f && t2 < 100.0f) publish_val_(bms_idx, "temp_2", t2);
        if (t3 > -30.0f && t3 < 100.0f) publish_val_(bms_idx, "temp_3", t3);
        if (t4 > -30.0f && t4 < 100.0f) publish_val_(bms_idx, "temp_4", t4);

        // Telemetria Generale
        int16_t raw_i = (int16_t)((payload[40] << 8) | payload[41]);
        uint16_t raw_v = (uint16_t)((payload[42] << 8) | payload[43]);
        uint16_t raw_rem_cap = (uint16_t)((payload[44] << 8) | payload[45]);
        uint16_t raw_soc = (uint16_t)((payload[48] << 8) | payload[49]);

        float voltage = raw_v * 0.01f;
        float current = raw_i * 0.01f;
        float rem_cap = raw_rem_cap * 0.01f;
        float soc = raw_soc * 0.1f;

        if (voltage > 30.0f && voltage < 70.0f) publish_val_(bms_idx, "pack_voltage", voltage);
        if (current > -500.0f && current < 500.0f) publish_val_(bms_idx, "current", current);
        if (soc <= 100.0f && soc >= 0.0f) publish_val_(bms_idx, "soc", soc);
        if (rem_cap <= 2000.0f && rem_cap >= 0.0f) publish_val_(bms_idx, "remaining_capacity", rem_cap);

        break;
      }
    }
  }
}

void SeplosParserHub::publish_val_(uint8_t bms_idx, const std::string &type_str, float value) {
  auto it = bms_sensors_.find(bms_idx);
  if (it == bms_sensors_.end()) return;

  sensor::Sensor *s = nullptr;
  if (type_str == "pack_voltage") s = it->second.pack_voltage;
  else if (type_str == "current") s = it->second.current;
  else if (type_str == "soc") s = it->second.soc;
  else if (type_str == "soh") s = it->second.soh;
  else if (type_str == "remaining_capacity") s = it->second.remaining_capacity;
  else if (type_str == "cycles") s = it->second.cycles;
  else if (type_str == "min_cell_voltage") s = it->second.min_cell_voltage;
  else if (type_str == "max_cell_voltage") s = it->second.max_cell_voltage;
  else if (type_str == "cell_delta_voltage") s = it->second.cell_delta_voltage;
  else if (type_str == "temp_1") s = it->second.temp1;
  else if (type_str == "temp_2") s = it->second.temp2;
  else if (type_str == "temp_3") s = it->second.temp3;
  else if (type_str == "temp_4") s = it->second.temp4;
  else if (type_str == "mos_temp") s = it->second.mos_temp;

  if (s != nullptr) {
    s->publish_state(value);
  }
}

void SeplosParserHub::publish_cell_val_(uint8_t bms_idx, uint8_t cell_idx, float value) {
  auto it = bms_sensors_.find(bms_idx);
  if (it == bms_sensors_.end() || cell_idx >= 16) return;

  sensor::Sensor *s = it->second.cells[cell_idx];
  if (s != nullptr) {
    s->publish_state(value);
  }
}

void SeplosParserHub::dump_config() {
  ESP_LOGCONFIG(TAG, "Seplos Parser Hub (ESP32-P4 + Seplos V3 Sniffer):");
  ESP_LOGCONFIG(TAG, "  BMS Count: %d", bms_count_);
  ESP_LOGCONFIG(TAG, "  BMS Mappati: %zu", bms_sensors_.size());
}

}  // namespace seplos_parser
}  // namespace esphome
