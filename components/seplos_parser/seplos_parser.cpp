#include "seplos_parser.h"
#include "esphome/core/log.h"
#include <cstdio>
#include <cstring>
#include <cstdlib>

namespace esphome {
namespace seplos_parser {

static const char *const TAG = "seplos_parser";

void SeplosParserHub::setup() {
  ESP_LOGI(TAG, "Inizializzazione Seplos V3 Sniffer su UART ESP32-P4...");
  rx_buffer_.reserve(512);
}

void SeplosParserHub::loop() {
  while (available()) {
    uint8_t c;
    read_byte(&c);
    rx_buffer_.push_back(c);
    last_rx_time_ = millis();

    // Rilevamento fine frame ASCII (CR '\r' = 0x0D) o limite buffer
    if (c == 0x0D || rx_buffer_.size() >= 256) {
      parse_rx_buffer_();
      rx_buffer_.clear();
    }
  }

  // Timeout frame (inattivo per più di 50ms)
  if (!rx_buffer_.empty() && (millis() - last_rx_time_ > 50)) {
    parse_rx_buffer_();
    rx_buffer_.clear();
  }
}

void SeplosParserHub::update() {
  ESP_LOGD(TAG, "Heartbeat Sniffer: in ascolto sulla linea RS485...");
}
void SeplosParserHub::parse_rx_buffer_() {
  if (rx_buffer_.size() < 7) return;

  // Scansione buffer alla ricerca di frame Modbus RTU (FC 0x03 o 0x04) o ASCII
  for (size_t i = 0; i < rx_buffer_.size() - 6; i++) {
    uint8_t addr = rx_buffer_[i];
    uint8_t func = rx_buffer_[i + 1];

    if ((func == 0x03 || func == 0x04) && addr <= 16) {
      parse_seplos_modbus_frame_(&rx_buffer_[i], rx_buffer_.size() - i);
      break;
    } else if (addr == 0x20 || addr == 0x7E) {
      parse_seplos_ascii_frame_(rx_buffer_);
      break;
    }
  }
}

void SeplosParserHub::parse_seplos_ascii_frame_(const std::vector<uint8_t> &frame) {
  std::string ascii_str(frame.begin(), frame.end());
  
  if (ascii_str.length() < 20) return;

  char adr_buf[3] = {ascii_str[3], ascii_str[4], '
