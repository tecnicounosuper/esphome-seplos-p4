#include "seplos_v3.h"
#include "esphome/core/log.h"

namespace esphome {
namespace seplos_v3 {

static const char *const TAG = "seplos_v3";

void SeplosComponent::setup() {
  ESP_LOGI(TAG, "Seplos V3 Sniffer inizializzato correttamente!");
}

void SeplosComponent::loop() {
  // Legge i dati dalla UART se disponibili
  while (this->available()) {
    uint8_t byte;
    this->read_byte(&byte);
    this->process_byte_(byte);
  }
}

void SeplosComponent::process_byte_(uint8_t byte) {
  // Aggiunge il byte al buffer
  this->rx_buffer_.push_back(byte);

  // LOG DI DEBUG: Stampa ogni byte ricevuto in formato HEX
  // Questo ti serve per vedere se il BMS sta effettivamente inviando dati
  ESP_LOGD(TAG, "Byte ricevuto: %02X", byte);

  // Logica di protezione: se il buffer diventa troppo grande senza essere processato, pulisci
  if (this->rx_buffer_.size() > 128) {
    ESP_LOGV(TAG, "Buffer pieno, svuotamento in corso...");
    this->rx_buffer_.clear();
  }

  // ESEMPIO DI PARSING DI BASE (Protocollo Seplos V3 / Modbus)
  // Se ricevi una sequenza che sembra un pacchetto (es. inizia con l'indirizzo 0x01)
  // e ha una lunghezza minima, qui andremo a inserire la logica di calcolo.
  if (this->rx_buffer_.size() >= 7) { 
      // Qui aggiungeremo il calcolo del checksum e l'estrazione dei valori
      // Per ora limitiamoci a monitorare i byte nel log.
  }
}

void SeplosComponent::register_sensor(uint8_t address, std::string type, sensor::Sensor *s) {
  this->sensors_.push_back({address, type, s});
}

void SeplosComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "Seplos V3 Sniffer Status:");
  ESP_LOGCONFIG(TAG, "  UART attivo sui pin configurati.");
  for (auto &si : this->sensors_) {
    ESP_LOGCONFIG(TAG, "  Sensore registrato: %s (BMS Addr: %d)", si.type.c_str(), si.address);
  }
}

}  // namespace seplos_v3
}  // namespace esphome
