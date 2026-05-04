#include "seplos_v3.h"
#include "esphome/core/log.h"

namespace esphome {
namespace seplos_v3 {

static const char *const TAG = "seplos_v3";

void SeplosComponent::loop() {
  while (this->available()) {
    uint8_t byte;
    this->read_byte(&byte);
    this->process_byte_(byte);
  }
}

void SeplosComponent::process_byte_(uint8_t byte) {
  this->rx_buffer_.push_back(byte);

  // Un pacchetto dati PIA (Pack Info A) tipico è di 41 byte (ADDR, CMD, LEN, 36 byte dati, 2 byte CRC)
  if (this->rx_buffer_.size() == 41) {
    uint8_t addr = this->rx_buffer_[0];
    uint8_t func = this->rx_buffer_[1];
    uint8_t len  = this->rx_buffer_[2];

    // Verifica se è una risposta corretta (Comando 0x04 e lunghezza 0x24)
    if (func == 0x04 && len == 0x24) {
      ESP_LOGI(TAG, "Ricevuto pacchetto PIA da BMS Indirizzo: %d", addr);
      this->decode_pia_(addr);
    }
    this->rx_buffer_.clear();
  } 
  else if (this->rx_buffer_.size() > 41) {
    this->rx_buffer_.erase(this->rx_buffer_.begin());
  }
}

void SeplosComponent::decode_pia_(uint8_t address) {
  // Tensione Totale: Byte 3 e 4 (Indirizzo 1000) - Unità 10mV
  uint16_t raw_voltage = (uint16_t)this->rx_buffer_[3] << 8 | this->rx_buffer_[4];
  float voltage = raw_voltage * 0.01f; // Trasforma 10mV in Volt

  // SOC: Byte 13 e 14 (Indirizzo 1005) - Unità 0.1%
  uint16_t raw_soc = (uint16_t)this->rx_buffer_[13] << 8 | this->rx_buffer_[14];
  float soc = raw_soc * 0.1f;

  ESP_LOGD(TAG, "BMS %d -> Volt: %.2fV, SOC: %.1f%%", address, voltage, soc);

  // Aggiorna i sensori di ESPHome
  for (auto &si : this->sensors_) {
    if (si.address == address) {
      if (si.type == "battery_voltage") si.sensor->publish_state(voltage);
      if (si.type == "battery_soc") si.sensor->publish_state(soc);
    }
  }
}

void SeplosComponent::register_sensor(uint8_t address, std::string type, sensor::Sensor *s) {
  this->sensors_.push_back({address, type, s});
}

}  // namespace seplos_v3
}  // namespace esphome
