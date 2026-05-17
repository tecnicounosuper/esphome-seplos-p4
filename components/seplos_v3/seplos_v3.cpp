#include "seplos_v3.h"
#include "esphome/core/log.h"

namespace esphome {
namespace seplos_v3 {

static const char *const TAG = "seplos_v3";

void _SeplosV3::setup() {
  // Inizializzazione vuota
}

void _SeplosV3::loop() {
  while (available()) {
    uint8_t byte;
    read_byte(&byte);
    rx_buffer_.push_back(byte);

    if (rx_buffer_.size() > 128) {
      rx_buffer_.erase(rx_buffer_.begin());
    }

    parse_buffer_();
  }
}

void _SeplosV3::parse_buffer_() {
  if (rx_buffer_.size() < 5) return;

  for (size_t i = 0; i <= rx_buffer_.size() - 5; i++) {
    // Intercetta lo Slave Address 0x01 o 0x02, Funzione Modbus 0x04 e Lunghezza Dati 52 byte (0x34)
    if ((rx_buffer_[i] == 0x01 || rx_buffer_[i] == 0x02) && rx_buffer_[i+1] == 0x04 && rx_buffer_[i+2] == 0x34) {
      size_t frame_len = 57; // 3 Intestazione + 52 Dati + 2 CRC
      
      if (rx_buffer_.size() < i + frame_len) {
        return; 
      }

      const uint8_t *data = &rx_buffer_[i + 3];

      // 1. Tensioni Celle (16 Celle -> 32 byte)
      for (size_t c = 0; c < 16; c++) {
        if (this->cell_sensors_[c] != nullptr) {
          uint16_t cell_mv = (data[c * 2] << 8) | data[c * 2 + 1];
          this->cell_sensors_[c]->publish_state(cell_mv / 1000.0f);
        }
      }

      // 2. Temperature (4 canali -> 8 byte, partendo dal byte 32 del payload)
      size_t temp_offset = 32; 
      for (size_t t = 0; t < 4; t++) {
        if (this->cell_temp_sensors_[t] != nullptr) {
          uint16_t temp_k = (data[temp_offset + t * 2] << 8) | data[temp_offset + t * 2 + 1];
          float temp_c = (temp_k - 2731) / 10.0f; 
          this->cell_temp_sensors_[t]->publish_state(temp_c);
        }
      }

      // 3. Corrente (byte 40-41)
      size_t current_offset = 40; 
      int16_t raw_current = (data[current_offset] << 8) | data[current_offset + 1];
      if (this->current_sensor_ != nullptr) {
        this->current_sensor_->publish_state(raw_current / 100.0f);
      }

      // 4. Tensione Totale Pacco (byte 42-43)
      size_t voltage_offset = 42;
      uint16_t raw_voltage = (data[voltage_offset] << 8) | data[voltage_offset + 1];
      if (this->pack_voltage_sensor_ != nullptr) {
        this->pack_voltage_sensor_->publish_state(raw_voltage / 100.0f);
      }

      // 5. State of Charge (SOC - byte 44-45)
      size_t soc_offset = 44;
      uint16_t raw_soc = (data[soc_offset] << 8) | data[soc_offset + 1];
      if (this->soc_sensor_ != nullptr) {
        this->soc_sensor_->publish_state(raw_soc / 10.0f);
      }

      rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + i + frame_len);
      return;
    }
  }
}

void _SeplosV3::dump_config() {
  ESP_LOGCONFIG(TAG, "Seplos V3 Custom Component attivo.");
}

}  // namespace seplos_v3
}  // namespace esphome
