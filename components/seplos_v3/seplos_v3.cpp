#include "seplos_v3.h"
#include "esphome/core/log.h"

namespace esphome {
namespace seplos_v3 {

static const char *const TAG = "seplos_v3";

void SeplosComponent::setup() {
  ESP_LOGI(TAG, "Inizializzazione Seplos V3 Sniffer...");
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
  // Logica di base: accumula i byte in un buffer
  if (this->rx_buffer_.size() > 256) {
    this->rx_buffer_.clear();
  }
  this->rx_buffer_.push_back(byte);

  // Qui andrebbe la logica di analisi del protocollo Seplos (Modbus o Serial)
  // Per ora, stampiamo i dati nel log se vediamo una fine riga o buffer pieno
  if (byte == 0x0A || this->rx_buffer_.size() == 13) { 
    ESP_LOGD(TAG, "Pacchetto ricevuto, lunghezza: %d", this->rx_buffer_.size());
    this->rx_buffer_.clear();
  }
}

void SeplosComponent::register_sensor(uint8_t address, const std::string &type, sensor::Sensor *s) {
  // Salva il puntatore al sensore per usarlo in seguito
  this->sensors_.push_back({address, type, s});
}

void SeplosComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "Seplos V3 Sniffer:");
  ESP_LOGCONFIG(TAG, "  UART ID: %s", this->parent_->get_hw_serial());
  for (auto &si : this->sensors_) {
    ESP_LOGCONFIG(TAG, "  Sensore: %s (BMS Addr: %d)", si.type.c_str(), si.address);
  }
}

}  // namespace seplos_v3
}  // namespace esphome
