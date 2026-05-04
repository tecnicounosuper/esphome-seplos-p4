#include "seplos_v3.h"
#include "esphome/core/log.h"

namespace esphome {
namespace seplos_v3 {

static const char *const TAG = "seplos_v3";

void SeplosComponent::setup() {
  ESP_LOGI(TAG, "Seplos V3 Sniffer inizializzato correttamente!");
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
  // Se il buffer diventa troppo grande, svuotalo per sicurezza
  if (this->rx_buffer_.size() > 512) {
    this->rx_buffer_.clear();
  }
}

void SeplosComponent::register_sensor(uint8_t address, std::string type, sensor::Sensor *s) {
  this->sensors_.push_back({address, type, s});
}

void SeplosComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "Seplos V3 Sniffer:");
  for (auto &si : this->sensors_) {
    ESP_LOGCONFIG(TAG, "  Sensore: %s per BMS indirizzo %d", si.type.c_str(), si.address);
  }
}

}  // namespace seplos_v3
}  // namespace esphome
