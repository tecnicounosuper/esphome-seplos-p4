#include "seplos_v3.h"
#include "esphome/core/log.h"

namespace esphome {
namespace seplos_v3 {

static const char *const TAG = "seplos_v3";

void SeplosComponent::setup() {
  ESP_LOGI(TAG, "Seplos V3 Sniffer - TEST RICEZIONE RAW");
}

void SeplosComponent::loop() {
  while (this->available()) {
    uint8_t byte;
    this->read_byte(&byte);
    
    // STAMPA OGNI SINGOLO BYTE RICEVUTO
    ESP_LOGD(TAG, "UART RAW RX: %02X", byte);

    this->rx_buffer_.push_back(byte);
    if (this->rx_buffer_.size() >= 41) {
      this->decode_pia_(this->rx_buffer_[0]);
      this->rx_buffer_.clear();
    }
  }
}

void SeplosComponent::decode_pia_(uint8_t address) {
  // Solo per evitare errori di compilazione, logica minima
  if (this->rx_buffer_.size() < 15) return;
  uint16_t v_raw = (uint16_t)this->rx_buffer_[3] << 8 | this->rx_buffer_[4];
  float voltage = v_raw * 0.01f;
  
  for (auto &si : this->sensors_) {
    if (si.address == address && si.type == "battery_voltage") {
      si.sensor->publish_state(voltage);
    }
  }
}

void SeplosComponent::register_sensor(uint8_t address, std::string type, sensor::Sensor *s) {
  this->sensors_.push_back({address, type, s});
}

void SeplosComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "Sniffer in ascolto...");
}

}  // namespace seplos_v3
}  // namespace esphome
