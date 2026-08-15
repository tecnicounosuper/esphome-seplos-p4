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
    static uint8_t poll_step = 0;
    poll_step++;

    uint8_t target = (poll_step / 2) % (bms_count_ > 0 ? bms_count_ : 1);
    uint8_t query_type = poll_step % 2; // 0 = Telemetria 0x1000, 1 = Celle 0x1100

    if (target == 0) {
      if (query_type == 0) {
        ESP_LOGI(TAG, "Polling Modbus 0x1000 (Telemetria) per BMS 0 Master (Modbus Addr 0x01)...");
        static const uint8_t modbus_q1[8] = {0x01, 0x04, 0x10, 0x00, 0x00, 0x0D, 0x0F, 0x75};
        this->write_array(modbus_q1, 8);
        this->write_str("~20004642E002000D000D010D020000FDA9\r");
      } else {
        ESP_LOGI(TAG, "Polling Modbus 0x1100 (Celle 16S) per BMS 0 Master (Modbus Addr 0x01)...");
        static const uint8_t modbus_q1_cells[8] = {0x01, 0x04, 0x11, 0x00, 0x00, 0x14, 0x31, 0xF5};
        this->write_array(modbus_q1_cells, 8);
      }
    } else {
      if (query_type == 0) {
        ESP_LOGI(TAG, "Polling Modbus 0x1000 (Telemetria) per BMS 1 Slave (Modbus Addr 0x02)...");
        static const uint8_t modbus_q2[8] = {0x02, 0x04, 0x10, 0x00, 0x00, 0x0D, 0x3C, 0x75};
        this->write_array(modbus_q2, 8);
        this->write_str("~20014642E002000D000D010D020000FDA8\r");
      } else {
        ESP_LOGI(TAG, "Polling Modbus 0x1100 (Celle 16S) per BMS 1 Slave (Modbus Addr 0x02)...");
        static const uint8_t modbus_q2_cells[8] = {0x02, 0x04, 0x11, 0x00, 0x00, 0x14, 0x02, 0xF5};
        this->write_array(modbus_q2_cells, 8);
      }
    }
  } else {
    ESP_LOGD(TAG, "Heartbeat Sniffer: ascolto passivo attivo sulla linea RS485 Seplos V3...");
  }
}

void SeplosParserHub::parse_rx_buffer_() {
  if (rx_buffer_.empty()) return;

  // 1. TENTATIVO DI DECODIFICA STREAM MODBUS RTU CON VALIDAZIONE RIGIDA CRC16
  // Priorità a Modbus RTU per evitare che i byte binari vengano scambiati per ASCII
  parse_seplos_modbus_frame_(rx_buffer_.data(), rx_buffer_.size());

  // 2. TENTATIVO DI DECODIFICA FRAME ASCII PURO
  // Un vero frame ASCII Seplos deve iniziare con '~' e contenere SOLTANTO caratteri esadecimali stampabili!
  size_t start_tilde = std::string::npos;
  for (size_t i = 0; i < rx_buffer_.size(); i++) {
    if (rx_buffer_[i] == 0x7E) { // '~'
      start_tilde = i;
      break;
    }
  }

  if (start_tilde != std::string::npos) {
    std::vector<uint8_t> pure_ascii;
    bool valid_ascii_sequence = true;

    for (size_t i = start_tilde; i < rx_buffer_.size(); i++) {
      uint8_t b = rx_buffer_[i];
      if (b == 0x0D || b == 0x0A) break; // Fine riga ASCII
      if (b == 0x7E || (b >= '0' && b <= '9') || (b >= 'A' && b <= 'F') || (b >= 'a' && b <= 'f')) {
        pure_ascii.push_back(b);
      } else {
        valid_ascii_sequence = false; // Trovato byte binario non ASCII! Annulla parsing ASCII.
        break;
      }
    }

    if (valid_ascii_sequence && pure_ascii.size() >= 17) {
      parse_seplos_ascii_frame_(pure_ascii);
    }
  }
}

void SeplosParserHub::parse_seplos_ascii_frame_(const std::vector<uint8_t> &frame) {
  std::string ascii_str(frame.begin(), frame.end());
  
  if (ascii_str.length() < 17) return;
  if (ascii_str[0] != '~') return;

  // Verifica CID1 (46 = Telemetria) e RTN (00 = Risposta OK dal BMS)
  if (ascii_str.length() < 9) return;
  std::string cid1_str = ascii_str.substr(5, 2);
  std::string rtn_str = ascii_str.substr(7, 2);

  // Soltanto se è una risposta OK (RTN = 00) e CID1 = 46!
  if (cid1_str != "46" || rtn_str != "00") {
    return; // Ignora query, echi di trasmissione TX o codici di errore!
  }

  uint8_t raw_adr = 0;
  if (ascii_str.length() >= 5) {
    char adr_buf[3] = {ascii_str[3], ascii_str[4], 0};
    raw_adr = (uint8_t) strtol(adr_buf, nullptr, 16);
  }

  uint8_t bms_idx = raw_adr;
  if (bms_sensors_.find(bms_idx) == bms_sensors_.end() && raw_adr > 0 && bms_sensors_.find(raw_adr - 1) != bms_sensors_.end()) {
    bms_idx = raw_adr - 1;
  }

  // Inizio payload dati alla posizione 13 (dopo header ~20004600E002)
  size_t pos = 13;
  if (ascii_str.length() <= pos + 2) return;

  // 1. Numero di Celle (2 caratteri Hex, es. "10" = 16 celle)
  std::string num_cells_hex = ascii_str.substr(pos, 2);
  pos += 2;
  uint8_t num_cells = (uint8_t) strtol(num_cells_hex.c_str(), nullptr, 16);

  if (num_cells < 4 || num_cells > 24) return; // Controllo di sicurezza: tra 4 e 24 celle!

  float min_v = 99.0f;
  float max_v = 0.0f;
  bool cells_found = false;

  // Lettura dinamica delle celle (4 caratteri Hex ciascuna in mV)
  for (int c = 0; c < num_cells; c++) {
    if (pos + 4 > ascii_str.length()) return;
    std::string cell_hex = ascii_str.substr(pos, 4);
    pos += 4;
    uint16_t cell_mv = (uint16_t) strtol(cell_hex.c_str(), nullptr, 16);
    float cell_v = cell_mv * 0.001f;

    if (cell_v > 1.8f && cell_v < 4.5f) {
      if (c < 16) {
        publish_cell_val_(bms_idx, c, cell_v);
      }
      if (cell_v < min_v) min_v = cell_v;
      if (cell_v > max_v) max_v = cell_v;
      cells_found = true;
    }
  }

  if (cells_found && max_v >= min_v) {
    publish_val_(bms_idx, "min_cell_voltage", min_v);
    publish_val_(bms_idx, "max_cell_voltage", max_v);
    publish_val_(bms_idx, "cell_delta_voltage", (max_v - min_v) * 1000.0f); // in mV
  }

  // 2. Numero di Sonde di Temperatura (2 caratteri Hex, es. "04" = 4 sonde)
  if (pos + 2 > ascii_str.length()) return;
  std::string num_temps_hex = ascii_str.substr(pos, 2);
  pos += 2;
  uint8_t num_temps = (uint8_t) strtol(num_temps_hex.c_str(), nullptr, 16);

  if (num_temps < 1 || num_temps > 8) return; // Controllo validità sonde!

  for (int t = 0; t < num_temps; t++) {
    if (pos + 4 > ascii_str.length()) return;
    std::string temp_hex = ascii_str.substr(pos, 4);
    pos += 4;
    uint16_t temp_raw = (uint16_t) strtol(temp_hex.c_str(), nullptr, 16);
    // Formula Seplos V3: (Kelvin*10 - 2731) * 0.1°C
    float temp_c = (temp_raw - 2731) * 0.1f;
    if (temp_c > -30.0f && temp_c < 100.0f) {
      if (t == 0) publish_val_(bms_idx, "temp_1", temp_c);
      else if (t == 1) publish_val_(bms_idx, "temp_2", temp_c);
      else if (t == 2) publish_val_(bms_idx, "temp_3", temp_c);
      else if (t == 3) publish_val_(bms_idx, "temp_4", temp_c);
      else if (t == 4) publish_val_(bms_idx, "mos_temp", temp_c);
    }
  }

  // 3. Corrente del Pacco (4 caratteri Hex, int16 signed in 0.01A)
  if (pos + 4 <= ascii_str.length()) {
    std::string i_hex = ascii_str.substr(pos, 4);
    pos += 4;
    int16_t raw_i = (int16_t) strtol(i_hex.c_str(), nullptr, 16);
    float current = raw_i * 0.01f;
    if (current > -500.0f && current < 500.0f) {
      publish_val_(bms_idx, "current", current);
    }
  }

  // 4. Tensione Totale del Pacco (4 caratteri Hex, uint16 in 0.01V)
  if (pos + 4 <= ascii_str.length()) {
    std::string v_hex = ascii_str.substr(pos, 4);
    pos += 4;
    uint16_t raw_v = (uint16_t) strtol(v_hex.c_str(), nullptr, 16);
    float voltage = raw_v * 0.01f;
    if (voltage > 30.0f && voltage < 70.0f) {
      publish_val_(bms_idx, "pack_voltage", voltage);
    }
  }

  // 5. Capacità Residua (4 caratteri Hex, uint16 in 0.01Ah)
  if (pos + 4 <= ascii_str.length()) {
    std::string cap_hex = ascii_str.substr(pos, 4);
    pos += 4;
    uint16_t raw_cap = (uint16_t) strtol(cap_hex.c_str(), nullptr, 16);
    float remaining_cap = raw_cap * 0.01f;
    if (remaining_cap >= 0.0f && remaining_cap <= 2000.0f) {
      publish_val_(bms_idx, "remaining_capacity", remaining_cap);
    }
  }

  // 6. Custom / Battery Number (2 caratteri Hex)
  if (pos + 2 <= ascii_str.length()) {
    pos += 2;
  }

  // 7. Capacità Nominale / Totale (4 caratteri Hex, in 0.01Ah)
  if (pos + 4 <= ascii_str.length()) {
    pos += 4;
  }

  // 8. Stato di Carica (SOC) (4 caratteri Hex, uint16 in 0.1%)
  if (pos + 4 <= ascii_str.length()) {
    std::string soc_hex = ascii_str.substr(pos, 4);
    pos += 4;
    uint16_t raw_soc = (uint16_t) strtol(soc_hex.c_str(), nullptr, 16);
    float soc = raw_soc * 0.1f;
    if (soc <= 100.0f && soc >= 0.0f) {
      publish_val_(bms_idx, "soc", soc);
    }
  }

  // 9. Stato di Salute (SOH) (4 caratteri Hex, uint16 in 0.1%)
  if (pos + 4 <= ascii_str.length()) {
    std::string soh_hex = ascii_str.substr(pos, 4);
    pos += 4;
    uint16_t raw_soh = (uint16_t) strtol(soh_hex.c_str(), nullptr, 16);
    float soh = raw_soh * 0.1f;
    if (soh <= 100.0f && soh >= 0.0f) {
      publish_val_(bms_idx, "soh", soh);
    }
  }

  // 10. Numero di Cicli (4 caratteri Hex, uint16)
  if (pos + 4 <= ascii_str.length()) {
    std::string cycles_hex = ascii_str.substr(pos, 4);
    pos += 4;
    uint16_t cycles = (uint16_t) strtol(cycles_hex.c_str(), nullptr, 16);
    if (cycles < 65000) {
      publish_val_(bms_idx, "cycles", (float)cycles);
    }
  }
}

static uint16_t calculate_modbus_crc16(const uint8_t *data, uint16_t len) {
  uint16_t crc = 0xFFFF;
  for (uint16_t pos = 0; pos < len; pos++) {
    crc ^= (uint16_t)data[pos];
    for (int i = 8; i != 0; i--) {
      if ((crc & 0x0001) != 0) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

void SeplosParserHub::parse_seplos_modbus_frame_(const uint8_t *data, size_t len) {
  if (len < 8) return;

  for (size_t i = 0; i < len - 5; i++) {
    uint8_t raw_addr = data[i];
    uint8_t func = data[i + 1];
    uint8_t byte_count = data[i + 2];

    // Rigido controllo Modbus RTU (Funzione 0x03 o 0x04 e Indirizzo BMS <= 16)
    if ((func == 0x03 || func == 0x04) && (raw_addr >= 1 && raw_addr <= 16) && (i + byte_count + 5 <= len)) {
      uint16_t msg_len = byte_count + 5;
      uint16_t calculated_crc = calculate_modbus_crc16(&data[i], msg_len - 2);
      uint16_t frame_crc = data[i + msg_len - 2] | (data[i + msg_len - 1] << 8);

      if (calculated_crc != frame_crc) {
        continue; // CRC errato, passa al prossimo byte
      }

      const uint8_t *payload = &data[i + 3];
      uint8_t bms_idx = raw_addr - 1; // Modbus ADR 1 -> BMS 0 (Master), ADR 2 -> BMS 1 (Slave)

      uint16_t reg0 = (uint16_t)((payload[0] << 8) | payload[1]);
      uint16_t reg1 = (uint16_t)((payload[2] << 8) | payload[3]);
      uint16_t reg2 = (uint16_t)((payload[4] << 8) | payload[5]);

      // Controllo se il payload rappresenta il blocco celle 16S (0x1100):
      // In un blocco celle, le prime 3 word sono le tensioni delle celle 1, 2, 3 in mV (2000 mV..4500 mV)
      bool is_cell_block = (reg0 >= 2000 && reg0 <= 4500) &&
                           (reg1 >= 2000 && reg1 <= 4500) &&
                           (reg2 >= 2000 && reg2 <= 4500);

      if (is_cell_block && byte_count >= 32) {
        // 1. CASO 0x1100 (Pack Info B - Tensioni Celle 16S + Temperature)
        float min_v = 99.0f;
        float max_v = 0.0f;
        bool cells_valid = false;

        for (int c = 0; c < 16; c++) {
          if (c * 2 + 1 >= byte_count) break;
          uint16_t cell_mv = (uint16_t)((payload[c * 2] << 8) | payload[c * 2 + 1]);
          float cell_v = cell_mv * 0.001f;
          if (cell_v > 1.8f && cell_v < 4.5f) {
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

        // Se presenti nel payload 0x1100, leggi anche le 4 temperature
        if (byte_count >= 40) {
          int16_t raw_t1 = (int16_t)((payload[32] << 8) | payload[33]);
          int16_t raw_t2 = (int16_t)((payload[34] << 8) | payload[35]);
          int16_t raw_t3 = (int16_t)((payload[36] << 8) | payload[37]);
          int16_t raw_t4 = (int16_t)((payload[38] << 8) | payload[39]);

          float t1 = raw_t1 > 1000 ? (raw_t1 - 2731) * 0.1f : raw_t1 * 0.1f;
          float t2 = raw_t2 > 1000 ? (raw_t2 - 2731) * 0.1f : raw_t2 * 0.1f;
          float t3 = raw_t3 > 1000 ? (raw_t3 - 2731) * 0.1f : raw_t3 * 0.1f;
          float t4 = raw_t4 > 1000 ? (raw_t4 - 2731) * 0.1f : raw_t4 * 0.1f;

          if (t1 > -30.0f && t1 < 100.0f) publish_val_(bms_idx, "temp_1", t1);
          if (t2 > -30.0f && t2 < 100.0f) publish_val_(bms_idx, "temp_2", t2);
          if (t3 > -30.0f && t3 < 100.0f) publish_val_(bms_idx, "temp_3", t3);
          if (t4 > -30.0f && t4 < 100.0f) publish_val_(bms_idx, "temp_4", t4);
        }

        i += msg_len - 1;
        continue;
      }

      // 2. CASO 0x1000 (Pack Info A - Telemetria Generale)
      // Prima word = Pack Voltage in 0.01V (3000..7000 = 30.00V .. 70.00V)
      if (reg0 >= 3000 && reg0 <= 7000 && byte_count >= 14) {
        float voltage = reg0 * 0.01f;
        int16_t raw_i = (int16_t)reg1;
        float current = raw_i * 0.01f;
        uint16_t raw_rem_cap = reg2;
        float rem_cap = raw_rem_cap * 0.01f;

        if (voltage > 30.0f && voltage < 70.0f) publish_val_(bms_idx, "pack_voltage", voltage);
        if (current > -500.0f && current < 500.0f) publish_val_(bms_idx, "current", current);
        if (rem_cap >= 0.0f && rem_cap <= 2000.0f) publish_val_(bms_idx, "remaining_capacity", rem_cap);

        if (byte_count >= 12) {
          uint16_t raw_soc = (uint16_t)((payload[10] << 8) | payload[11]);
          float soc = raw_soc > 100 ? raw_soc * 0.1f : (float)raw_soc;
          if (soc >= 0.0f && soc <= 100.0f) publish_val_(bms_idx, "soc", soc);
        }

        if (byte_count >= 14) {
          uint16_t raw_soh = (uint16_t)((payload[12] << 8) | payload[13]);
          float soh = raw_soh > 100 ? raw_soh * 0.1f : (float)raw_soh;
          if (soh >= 0.0f && soh <= 100.0f) publish_val_(bms_idx, "soh", soh);
        }

        if (byte_count >= 16) {
          uint16_t cycles = (uint16_t)((payload[14] << 8) | payload[15]);
          if (cycles < 65000) publish_val_(bms_idx, "cycles", (float)cycles);
        }

        if (byte_count >= 26) {
          int16_t raw_t1 = (int16_t)((payload[16] << 8) | payload[17]);
          int16_t raw_t2 = (int16_t)((payload[18] << 8) | payload[19]);
          int16_t raw_t3 = (int16_t)((payload[20] << 8) | payload[21]);
          int16_t raw_t4 = (int16_t)((payload[22] << 8) | payload[23]);
          int16_t raw_tmos = (int16_t)((payload[24] << 8) | payload[25]);

          float t1 = raw_t1 > 1000 ? (raw_t1 - 2731) * 0.1f : raw_t1 * 0.1f;
          float t2 = raw_t2 > 1000 ? (raw_t2 - 2731) * 0.1f : raw_t2 * 0.1f;
          float t3 = raw_t3 > 1000 ? (raw_t3 - 2731) * 0.1f : raw_t3 * 0.1f;
          float t4 = raw_t4 > 1000 ? (raw_t4 - 2731) * 0.1f : raw_t4 * 0.1f;
          float tmos = raw_tmos > 1000 ? (raw_tmos - 2731) * 0.1f : raw_tmos * 0.1f;

          if (t1 > -30.0f && t1 < 100.0f) publish_val_(bms_idx, "temp_1", t1);
          if (t2 > -30.0f && t2 < 100.0f) publish_val_(bms_idx, "temp_2", t2);
          if (t3 > -30.0f && t3 < 100.0f) publish_val_(bms_idx, "temp_3", t3);
          if (t4 > -30.0f && t4 < 100.0f) publish_val_(bms_idx, "temp_4", t4);
          if (tmos > -30.0f && tmos < 100.0f) publish_val_(bms_idx, "mos_temp", tmos);
        }

        i += msg_len - 1;
        continue;
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
