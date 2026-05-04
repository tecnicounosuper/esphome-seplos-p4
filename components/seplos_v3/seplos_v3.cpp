#include "seplos_v3.h"
#include "esphome/core/log.h"

namespace esphome {
namespace seplos_v3 {

static const char *const TAG = "seplos_v3";

void SeplosComponent::setup() {
  ESP_LOGI(TAG, "Seplos V3 Sniffer pronto per 19200 baud.");
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

  // Il pacchetto PIA standard è ADDR(1) + FUNC(1) + LEN(1) + DATA(36) + CRC(2) = 41 byte
  if (this->rx_buffer_.size() >= 41) {
    uint8_t addr = this->rx_buffer_[0];
    uint8_t func = this->rx_buffer_[1];
    uint8_t len  = this->rx_buffer_[2];

    // Verifica Modbus 0x04 (Read Input Registers) con lunghezza 0x24 (36 dec)
    if (func == 0x04 && len == 0x24) {
      this->decode_pia_(addr);
      this->rx_buffer_.clear();
    } else {
      // Se non è il pacchetto giusto, slitta di un byte e riprova
      this->rx_buffer_.erase(this->rx_buffer_.begin());
    }
  }
}

void SeplosComponent::decode_pia_(uint8_t address) {
  // Basato su SEPLOS-3.0 Modbus RTU Protocol[cite: 1]
  
  // Tensione: Offset 3-4 (Reg 1000H), Unità 10mV
  uint16_t v_raw = (uint16_t)this->rx_buffer_[3] << 8 | this->rx_buffer_[4];
  float voltage = v_raw * 0.01f;

  // Corrente: Offset 5-6 (Reg 1001H), Unità 10mA (INT16 con segno)
  int16_t c_raw = (int16_t)((this->rx_buffer_[5] << 8) | this->rx_buffer_[6]);
  float current = c_raw * 0.01f;

  // SOC: Offset 13-14 (Reg 1005H), Unità 0.1%
  uint16_t s_raw = (uint16_t)this->rx_buffer_[13] << 8 | this->rx_buffer_[14];
  float soc = s_raw * 0.1f;

  for (auto &si : this->sensors_) {
    if (si.address == address) {
      if (si.type == "battery_voltage") si.sensor->publish_state(voltage);
      if (si.type == "battery_soc") si.sensor->publish_state(soc);
      if (si.type == "current") si.sensor->publish_state(current);
    }
  }
}

void SeplosComponent::register_sensor(uint8_t address, std::string type, sensor::Sensor *s) {
  this->sensors_.push_back({address, type, s});
}

void SeplosComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "Seplos V3 Sniffer Configurato.");
}

}  // namespace seplos_v3
}  // namespace esphome
