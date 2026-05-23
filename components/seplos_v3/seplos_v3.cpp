#include "seplos_v3.h"
#include "esphome/core/log.h"
#include <cstdint>
#include <vector>

namespace esphome {
namespace seplos_v3 {

static const char *const TAG = "seplos_v3";

// Buffer statico globale condiviso tra tutte le istanze per evitare conflitti sulla seriale
static std::vector<uint8_t> global_rx_buffer;

void _SeplosV3::setup() {
  // Setup vuoto
}

void _SeplosV3::loop() {
  // Qualsiasi istanza giri per prima, svuota la seriale accumulando nel buffer globale condiviso
  while (available()) {
    uint8_t byte;
    read_byte(&byte);
    global_rx_buffer.push_back(byte);
  }

  // Evitiamo che il buffer cresca all'infinito in caso di disallineamenti gravi
  if (global_rx_buffer.size() > 1024) {
    global_rx_buffer.erase(global_rx_buffer.begin(), global_rx_buffer.begin() + 256);
  }

  parse_buffer_();
}

void _SeplosV3::parse_buffer_() {
  if (global_rx_buffer.size() < 5) return;

  for (size_t i = 0; i <= global_rx_buffer.size() - 5; i++) {
    // Controlla l'indirizzo dell'istanza corrente (0x01 o 0x02) e la funzione Modbus 0x04
    if (global_rx_buffer[i] == this->address_ && global_rx_buffer[i+1] == 0x04) {
      
      uint8_t byte_count = global_rx_buffer[i+2];
      
      // Accettiamo solo i due formati validi del Seplos V3 (36 byte o 52 byte di dati)
      if (byte_count != 0x34 && byte_count != 0x24) {
        continue; 
      }

      size_t frame_len = 3 + byte_count + 2; // Header (3) + Dati (byte_count) + CRC (2)
      
      // Se il frame non è completo nel buffer, aspettiamo che finisca di arrivare
      if (global_rx_buffer.size() < i + frame_len) {
        return; 
      }

      // Puntatore all'inizio dei dati utili del pacchetto corrente
      const uint8_t *data = &global_rx_buffer[i + 3];

      if (byte_count == 0x34) {
        // -------------------------------------------------------------
        // FRAME DA 52 BYTE: Lettura di Tensioni Celle e Temperature
        // -------------------------------------------------------------
        
        // 1. Decodifica Tensioni Celle (16 celle = 32 byte)
        for (size_t c = 0; c < 16; c++) {
          if (this->cell_sensors_[c] != nullptr) {
            uint16_t cell_mv = (data[c * 2] << 8) | data[c * 2 + 1];
            this->cell_sensors_[c]->publish_state(cell_mv / 1000.0f);
          }
        }

        // 2. Decodifica Temperature Sonde Celle (4 sonde = 8 byte, parte da offset 32)
        size_t temp_offset = 32; 
        for (size_t t = 0; t < 4; t++) {
          if (this->cell_temp_sensors_[t] != nullptr) {
            uint16_t temp_k = (data[temp_offset + t * 2] << 8) | data[temp_offset + t * 2 + 1];
            float temp_c = (temp_k - 2731) / 10.0f; 
            this->cell_temp_sensors_[t]->publish_state(temp_c);
          }
        }
      } 
      else if (byte_count == 0x24) {
        // -------------------------------------------------------------
        // FRAME DA 36 BYTE: Lettura Dati Generali del Pacco Batteria
        // -------------------------------------------------------------
        
        // 1. Tensione Totale del Pacco (offset 0)
        if (this->pack_voltage_sensor_ != nullptr) {
          uint16_t raw_voltage = (data[0] << 8) | data[1];
          this->pack_voltage_sensor_->publish_state(raw_voltage / 100.0f);
        }

        // 2. Corrente Batteria (offset 2, con segno per carica/scarica)
        if (this->current_sensor_ != nullptr) {
          int16_t raw_current = (data[2] << 8) | data[3];
          this->current_sensor_->publish_state(raw_current / 100.0f);
        }

        // 3. State of Charge / SOC (offset 34)
        size_t soc_offset = 34;
        if (this->soc_sensor_ != nullptr) {
          uint16_t raw_soc = (data[soc_offset] << 8) | data[soc_offset + 1];
          this->soc_sensor_->publish_state(raw_soc / 10.0f);
        }
      }

      // Rimuoviamo il frame elaborato e l'eventuale sporcizia precedente dal buffer globale
      global_rx_buffer.erase(global_rx_buffer.begin(), global_rx_buffer.begin() + i + frame_len);
      return;
    }
  }
}

void _SeplosV3::dump_config() {
  ESP_LOGCONFIG(TAG, "Seplos V3 Custom Component - Indirizzo: 0x%02X", this->address_);
}

}  // namespace seplos_v3
}  // namespace esphome
