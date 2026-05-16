#include "seplos_parser.h"
#include "esphome/core/log.h"

namespace esphome {
namespace seplos_parser {

static const char *const TAG = "seplos_parser";

void SeplosParser::setup() {
  // Inizializzazione se necessaria
}

void SeplosParser::loop() {
  while (available()) {
    uint8_t byte;
    read_byte(&byte);
    rx_buffer_.push_back(byte);

    // Evita l'overflow del buffer se arrivano dati casuali o incompleti
    if (rx_buffer_.size() > 128) {
      rx_buffer_.erase(rx_buffer_.begin());
    }

    parse_buffer_();
  }
}

void SeplosParser::parse_buffer_() {
  if (rx_buffer_.size() < 5) return;

  // Analisi del buffer alla ricerca dell'inizio del frame Modbus RTU (Func: 0x04, Byte Count: 0x34)
  for (size_t i = 0; i <= rx_buffer_.size() - 5; i++) {
    if ((rx_buffer_[i] == 0x01 || rx_buffer_[i] == 0x02) && rx_buffer_[i+1] == 0x04 && rx_buffer_[i+2] == 0x34) {
      // Lunghezza totale frame = 3 byte intestazione + 52 byte dati + 2 byte CRC = 57 byte
      size_t frame_len = 57; 
      
      if (rx_buffer_.size() < i + frame_len) {
        return; // Aspetta che arrivi il frame completo
      }

      // Puntatore di partenza della sezione dati del payload (salta indirizzo, funzione e conteggio byte)
      const uint8_t *data = &rx_buffer_[i + 3];

      // 1. Lettura Tensioni Celle (16 celle, 2 byte per cella, espresse in mV)
      for (size_t c = 0; c < 16; c++) {
        if (this->cell_sensors_[c] != nullptr) {
          uint16_t cell_mv = (data[c * 2] << 8) | data[c * 2 + 1];
          this->cell_sensors_[c]->publish_state(cell_mv / 1000.0f);
        }
      }

      // 2. Lettura Temperature Celle (4 canali, 2 byte ciascuno, espressi in decimi di Kelvin)
      size_t temp_offset = 32; // Calcolato dopo i 32 byte delle celle
      for (size_t t = 0; t < 4; t++) {
        if (this->cell_temp_sensors_[t] != nullptr) {
          uint16_t temp_k = (data[temp_offset + t * 2] << 8) | data[temp_offset + t * 2 + 1];
          float temp_c = (temp_k - 2731) / 10.0f; // Conversione da Kelvin a Celsius
          this->cell_temp_sensors_[t]->publish_state(temp_c);
        }
      }

      // 3. Lettura Corrente (offset variabile a seconda della mappa esatta, indicativamente byte 40)
      size_t current_offset = 40; 
      int16_t raw_current = (data[current_offset] << 8) | data[current_offset + 1];
      if (this->current_sensor_ != nullptr) {
        this->current_sensor_->publish_state(raw_current / 100.0f);
      }

      // 4. Lettura Tensione Totale del Pacco (indicativamente byte 42)
      size_t voltage_offset = 42;
      uint16_t raw_voltage = (data[voltage_offset] << 8) | data[voltage_offset + 1];
      if (this->pack_voltage_sensor_ != nullptr) {
        this->pack_voltage_sensor_->publish_state(raw_voltage / 100.0f);
      }

      // 5. State of Charge (SOC - indicativamente byte 44)
      size_t soc_offset = 44;
      uint16_t raw_soc = (data[soc_offset] << 8) | data[soc_offset + 1];
      if (this->soc_sensor_ != nullptr) {
        this->soc_sensor_->publish_state(raw_soc / 10.0f);
      }

      // Svuota il buffer fino alla fine del pacchetto elaborato
      rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + i + frame_len);
      return;
    }
  }
}

void SeplosParser::dump_config() {
  ESP_LOGCONFIG(TAG, "Seplos Parser Custom Component configurato.");
}

}  // namespace seplos_parser
}  // namespace esphome
