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
  // Accumula tutti i byte in arrivo dalla seriale nel buffer globale condiviso
  while (available()) {
    uint8_t byte;
    read_byte(&byte);
    global_rx_buffer.push_back(byte);
  }

  // Protezione per evitare che il buffer cresca all'infinito in caso di disallineamenti gravi
  if (global_rx_buffer.size() > 1024) {
    global_rx_buffer.erase(global_rx_buffer.begin(), global_rx_buffer.begin() + 256);
  }

  parse_buffer_();
}

void _SeplosV3::parse_buffer_() {
  // Abbiamo bisogno di almeno 5 byte per leggere un header Modbus coerente
  while (global_rx_buffer.size() >= 5) {
    uint8_t addr = global_rx_buffer[0];
    uint8_t func = global_rx_buffer[1];
    uint8_t byte_count = global_rx_buffer[2];
    
    // Un frame di risposta Seplos V3 valido DEVE iniziare con Indirizzo (0x01 o 0x02), 
    // Funzione Modbus (0x04) e un numero di byte dati pari a 0x34 (52 byte) o 0x24 (36 byte)
    if ((addr != 0x01 && addr != 0x02) || func != 0x04 || (byte_count != 0x34 && byte_count != 0x24)) {
      // Se non è l'inizio esatto di una risposta valida, scartiamo il primo byte 
      // e continuiamo a scorrere la coda (elimina richieste ed eco della trasmittente)
      global_rx_buffer.erase(global_rx_buffer.begin());
      continue;
    }
    
    // Calcoliamo la lunghezza totale attesa del pacchetto: Header (3 byte) + Dati (byte_count) + CRC (2 byte)
    size_t frame_len = 3 + byte_count + 2;
    
    // Se il frame individuato non è ancora arrivato del tutto, usciamo dal ciclo 
    // e aspettiamo che la seriale riceva i byte mancanti nei prossimi millisecondi
    if (global_rx_buffer.size() < frame_len) {
      return; 
    }
    
    // Il frame in testa alla coda è completo. Verifichiamo se appartiene a QUESTA istanza
    if (addr == this->address_) {
      const uint8_t *data = &global_rx_buffer[3];
      
      if (byte_count == 0x34) {
        // -------------------------------------------------------------
        // PACCHETTO DA 52 BYTE: Tensioni Celle e Temperature
        // -------------------------------------------------------------
        
        // 1. Decodifica Tensioni Celle (16 celle = 32 byte)
        for (size_t c = 0; c < 16; c++) {
          if (this->cell_sensors_[c] != nullptr) {
            uint16_t cell_mv = (data[c * 2] << 8) | data[c * 2 + 1];
            this->cell_sensors_[c]->publish_state(cell_mv / 1000.0f);
          }
        }

        // 2. Decodifica Temperature (4 sonde = 8 byte, parte da offset 32)
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
        // PACCHETTO DA 36 BYTE: Stato Generale del Pacco Batteria
        // -------------------------------------------------------------
        
        // 1. Tensione Totale del Pacco (offset 0)
        if (this->pack_voltage_sensor_ != nullptr) {
          uint16_t raw_voltage = (data[0] << 8) | data[1];
          this->pack_voltage_sensor_->publish_state(raw_voltage / 100.0f);
        }

        // 2. Corrente Batteria (offset 2, con segno +/-)
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

      // Rimuoviamo dalla coda globale solo ed esclusivamente il pacchetto appena elaborato
      global_rx_buffer.erase(global_rx_buffer.begin(), global_rx_buffer.begin() + frame_len);
      
    } else {
      // Il pacchetto in cima alla coda è valido ma appartiene all'ALTRO BMS.
      // Usciamo immediatamente da questo ciclo per permettere all'altra istanza 
      // di intercettarlo ed elaborarlo senza rischiare di cancellarlo.
      return;
    }
  }
}

void _SeplosV3::dump_config() {
  ESP_LOGCONFIG(TAG, "Seplos V3 Custom Component - Indirizzo: 0x%02X", this->address_);
}

}  // namespace seplos_v3
}  // namespace esphome
