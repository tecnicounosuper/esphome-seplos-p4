#include "seplos_v3.h"
#include "esphome/core/log.h"

namespace esphome {
namespace seplos_v3 {

static const char *const TAG = "seplos_v3";

void SeplosComponent::setup() {
  ESP_LOGI(TAG, "Seplos V3 Sniffer - Modalità Parallelo (Master/Slave) avviata");
}

void SeplosComponent::loop() {
  while (this->available()) {
    uint8_t byte;
    this->read_byte(&byte);
    this->process_byte_(byte);
  }
}

void SeplosComponent::process_byte_(uint8_t byte) {
  this->rx_buffer_.push_back(byte);

  // Se il buffer è troppo piccolo, aspetta
  if (this->rx_buffer_.size() < 4) return;

  // Cerca l'inizio di un potenziale pacchetto Modbus (Indirizzo 1 o 2)
  // Il Seplos V3 in parallelo risponde tipicamente agli indirizzi 1, 2, ecc.
  uint8_t addr = this->rx_buffer_[0];
  uint8_t func = this->rx_buffer_[1];

  // Se i primi byte non sembrano Modbus standard (0x03 o 0x04), scarta il primo byte e riprova
  if (addr > 16 || (func != 0x03 && func != 0x04)) {
    this->rx_buffer_.erase(this->rx_buffer_.begin());
    return;
  }

  // Se abbiamo un pacchetto potenzialmente completo (es. lunghezza fissa o variabile)
  // Nota: I pacchetti Seplos V3 possono essere lunghi circa 70-100 byte per i dati completi
  if (this->rx_buffer_.size() >= 70) { 
    ESP_LOGD(TAG, "Pacchetto intercettato per Batteria %d - Lunghezza: %d", addr, this->rx_buffer_.size());
    
    // DEBUG: Stampa i primi 10 byte per analisi
    std::string hex_data = "";
    for(int i=0; i<10; i++) {
        char buf[5];
        sprintf(buf, "%02X ", this->rx_buffer_[i]);
        hex_data += buf;
    }
    ESP_LOGD(TAG, "Header: %s", hex_data.c_str());

    this->decode_packet_(addr);
    this->rx_buffer_.clear();
  }
}

void SeplosComponent::decode_packet_(uint8_t address) {
  // Qui inseriremo la logica di estrazione basata sull'indirizzo
  // Se address == 1 -> Aggiorna sensori Batteria Master
  // Se address == 2 -> Aggiorna sensori Batteria Slave
  
  // Esempio fittizio per testare la ricezione nei log
  for (auto &si : this->sensors_) {
    if (si.address == address) {
       // logica di estrazione float val = ...
       // si.sensor->publish_state(val);
    }
  }
}

void SeplosComponent::register_sensor(uint8_t address, std::string type, sensor::Sensor *s) {
  this->sensors_.push_back({address, type, s});
}

void SeplosComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "Seplos V3 Sniffer Multi-Pacco:");
  for (auto &si : this->sensors_) {
    ESP_LOGCONFIG(TAG, "  Batteria %d -> %s", si.address, si.type.c_str());
  }
}

}  // namespace seplos_v3
}  // namespace esphome
