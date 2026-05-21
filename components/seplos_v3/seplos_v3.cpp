#include "seplos_v3.h"
#include "esphome/core/log.h"
#include <cstdint>

namespace esphome {
namespace seplos_v3 {

static const char *const TAG = "seplos_v3";

void _SeplosV3::setup() {
  // Setup vuoto
}

void _SeplosV3::loop() {
  while (available()) {
    uint8_t byte;
    read_byte(&byte);
    rx_buffer_.push_back(byte);

    // Evitiamo che il buffer cresca all'infinito in caso di disallineamento
    if (rx_buffer_.size() > 512) {
      rx_buffer_.erase(rx_buffer_.begin());
    }

    parse_buffer_();
  }
}

void _SeplosV3::parse_buffer_() {
  if (rx_buffer_.size() < 5) return;

  for (size_t i = 0; i <= rx_buffer_.size() - 5; i++) {
    // Controlla se il byte corrisponde all'indirizzo di questa istanza (0x01 o 0x02) e alla funzione Modbus 0x04
    if (rx_buffer_[i] == this->address_ && rx_buffer_[i+1] == 0x04) {
      
      uint8_t byte_count = rx_buffer_[i+2];
      
      // Controllo di validità sulla dimensione del pacchetto dati
      if (byte_count < 10 || byte_count > 100) {
        continue; 
      }

      size_t frame_len = 3 + byte_count + 2; // Header (3) + Dati (byte_count) + CRC (2)
      
      // Se il frame completo non è ancora arrivato interamente nel buffer, aspettiamo il prossimo ciclo
      if (rx_buffer_.size() < i + frame_len) {
        return; 
      }

      // Puntatore all'inizio del blocco dati utile (subito dopo l'header)
      const uint8_t *data = &rx_buffer_[i + 3];

      // 1. Decodifica Tensioni Celle (16 celle = 32 byte)
      for (size_t c = 0; c < 16; c++) {
        if (this->cell_sensors_[c] != nullptr && (c * 2 + 1) < byte_count) {
          uint16_t cell_mv = (data[c * 2] << 8) | data[c * 2 + 1];
          this->cell_sensors_[c]->publish_state(cell_mv / 1000.0f);
        }
      }

      // 2. Decodifica Temperature Sonde Celle (4 sonde = 8 byte, parte da offset 32)
      size_t temp_offset = 32; 
      for (size_t t = 0; t < 4; t++) {
        if (this->cell_temp_sensors_[t] != nullptr && (temp_offset + t * 2 + 1) < byte_count) {
          uint16_t temp_k = (data[temp_offset + t * 2] << 8) | data[temp_offset + t * 2 + 1];
          float temp_c = (temp_k - 2731) / 10.0f; 
          this->cell_temp_sensors_[t]->publish_state(temp_c);
        }
      }

      // 3. Corrente Batteria (offset 44)
      size_t current_offset = 44; 
      if (this->current_sensor_ != nullptr && (current_offset + 1) < byte_count) {
        int16_t raw_current = (data[current_offset] << 8) | data[current_offset + 1];
        this->current_sensor_->publish_state(raw_current / 100.0f);
      }

      // 4. Tensione Totale del Pacco (offset 46)
      size_t voltage_offset = 46;
      if (this->pack_voltage_sensor_ != nullptr && (voltage_offset + 1) < byte_count) {
        uint16_t raw_voltage = (data[voltage_offset] << 8) | data[voltage_offset + 1];
        this->pack_voltage_sensor_->publish_state(raw_voltage / 100.0f);
      }

      // 5. State of Charge / SOC (offset 48)
      size_t soc_offset = 48;
      if (this->soc_sensor_ != nullptr && (soc_offset + 1) < byte_count) {
        uint16_t raw_soc = (data[soc_offset] << 8) | data[soc_offset + 1];
        this->soc_sensor_->publish_state(raw_soc / 10.0f);
      }

      // Rimuoviamo dal buffer solo i dati relativi al frame appena processato con successo
      rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + i + frame_len);
      return;
    }
  }

  // Se il buffer si riempie di spazzatura o risposte non nostre, puliamo per evitare blocchi
  if (rx_buffer_.size() > 128) {
    rx_buffer_.clear();
  }
}

void _SeplosV3::dump_config() {
  ESP_LOGCONFIG(TAG, "Seplos V3 Custom Component - Indirizzo: 0x%02X", this->address_);
}

}  // namespace seplos_v3
}  // namespace esphome
