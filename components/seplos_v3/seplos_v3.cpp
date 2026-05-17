#include "seplos_v3.h"
#include "esphome/core/log.h"

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

    // Manteniamo il buffer abbastanza capiente per elaborare trame concorrenti
    if (rx_buffer_.size() > 512) {
      rx_buffer_.erase(rx_buffer_.begin());
    }

    parse_buffer_();
  }
}

void _SeplosV3::parse_buffer_() {
  if (rx_buffer_.size() < 5) return;

  for (size_t i = 0; i <= rx_buffer_.size() - 5; i++) {
    // Verifica l'indirizzo del BMS corrente (0x01 o 0x02) e la funzione di lettura (0x04)
    if (rx_buffer_[i] == this->address_ && rx_buffer_[i+1] == 0x04) {
      
      // Il Byte 2 contiene il numero di byte di dati restituiti dal BMS
      uint8_t byte_count = rx_buffer_[i+2];
      
      // La lunghezza totale del frame è: Header (3 byte) + Dati (byte_count) + CRC (2 byte)
      size_t frame_len = 3 + byte_count + 2;
      
      // Se il pacchetto nel buffer non è ancora completo, aspettiamo i prossimi cicli del loop
      if (rx_buffer_.size() < i + frame_len) {
        return; 
      }

      // Puntatore all'inizio dei dati (subito dopo l'header di 3 byte)
      const uint8_t *data = &rx_buffer_[i + 3];

      // 1. Tensioni Celle (16 celle = 32 byte)
      for (size_t c = 0; c < 16; c++) {
        if (this->cell_sensors_[c] != nullptr && (c * 2 + 1) < byte_count) {
          uint16_t cell_mv = (data[c * 2] << 8) | data[c * 2 + 1];
          this->cell_sensors_[c]->publish_state(cell_mv / 1000.0f);
        }
      }

      // 2. Temperature (4 sonde = 8 byte, posizionate subito dopo le celle -> offset 32)
      size_t temp_offset = 32; 
      for (size_t t = 0; t < 4; t++) {
        if (this->cell_temp_sensors_[t] != nullptr && (temp_offset + t * 2 + 1) < byte_count) {
          uint16_t temp_k = (data[temp_offset + t * 2] << 8) | data[temp_offset + t * 2 + 1];
          float temp_c = (temp_k - 2731) / 10.0f; 
          this->cell_temp_sensors_[t]->publish_state(temp_c);
        }
      }

      // MAPPATURA CORRETTA DEI REGISTRI GENERALI DEL SEPLOS V3:
      // 3. Corrente (2 byte -> offset 40 nella sezione dati)
      size_t current_offset = 40; 
      if (this->current_sensor_ != nullptr && (current_offset + 1) < byte_count) {
        int16_t raw_current = (data[current_offset] << 8) | data[current_offset + 1];
        // La corrente nel Seplos V3 ha una risoluzione di 0.01A o 0.1A a seconda del modello.
        // Se noti valori troppo alti/bassi, cambieremo il divisore. Proviamo con 100.0f.
        this->current_sensor_->publish_state(raw_current / 100.0f);
      }

      // 4. Tensione Pacco (2 byte -> offset 42 nella sezione dati)
      size_t voltage_offset = 42;
      if (this->pack_voltage_sensor_ != nullptr && (voltage_offset + 1) < byte_count) {
        uint16_t raw_voltage = (data[voltage_offset] << 8) | data[voltage_offset + 1];
        this->pack_voltage_sensor_->publish_state(raw_voltage / 100.0f);
      }

      // 5. Stato di Carica (SOC - 2 byte -> offset 44 nella sezione dati)
      size_t soc_offset = 44;
      if (this->soc_sensor_ != nullptr && (soc_offset + 1) < byte_count) {
        uint16_t raw_soc = (data[soc_offset] << 8) | data[soc_offset + 1];
        // Il SOC viene restituito in percentuale diretta o decimi di percentuale.
        // Se ad esempio vedi 1000 invece di 100%, useremo / 10.0f.
        this->soc_sensor_->publish_state(raw_soc / 10.0f);
      }

      // Rimuove in modo pulito l'intero pacchetto elaborato dal buffer
      rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + i + frame_len);
      return;
    }
  }
}

void _SeplosV3::dump_config() {
  ESP_LOGCONFIG(TAG, "Seplos V3 Custom Component - Indirizzo: 0x%02X", this->address_);
}

}  // namespace seplos_v3
}  // namespace esphome
