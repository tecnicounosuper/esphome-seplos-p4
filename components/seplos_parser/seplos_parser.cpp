#include "seplos_parser.h"
#include "esphome/core/log.h"
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <algorithm>

namespace esphome {
namespace seplos_parser {

static const char *const TAG = "seplos_parser";

void SeplosParserHub::setup() {
  ESP_LOGI(TAG, "Inizializzazione Seplos V3 Sniffer su UART ESP32-P4 (19200 Baud)...");
  rx_buffer_.reserve(512);
}

void SeplosParserHub::loop() {
  while (available()) {
    uint8_t c;
    read_byte(&c);
    rx_buffer_.push_back(c);
    last_rx_time_ = millis();

    // Fine frame ASCII (CR '\r' = 0x0D) o limite buffer
    if (c == 0x0D || rx_buffer_.size() >= 300) {
      parse_rx_buffer_();
      rx_buffer_.clear();
    }
  }

  // Timeout frame (inattività > 50ms)
  if (!rx_buffer_.empty() && (millis() - last_rx_time_ > 50)) {
    parse_rx_buffer_();
    rx_buffer_.clear();
  }
}

void SeplosParserHub::update() {
  ESP_LOGD(TAG, "Heartbeat Sniffer: ascolto passivo sulla linea RS485 Seplos V3...");
}

void SeplosParserHub::parse_rx_buffer_() {
  if (rx_buffer_.empty()) return;

  // 1. ISOLAMENTO FRAME ASCII
  std::vector<uint8_t> ascii_chars;
  bool ascii_found = false;
  for (uint8_t b : rx_buffer_) {
    if (b == 0x7E || b == 0x20 || (b >= '0' && b <= '9') || (b >= 'A' && b <= 'F') || (b >= 'a' && b <= 'f') || b == 0x0D || b == 0x0A) {
      ascii_chars.push_back(b);
      if (b == 0x7E) ascii_found = true;
    }
  }

  // 2. PARSING FRAME ASCII SEPLOS V3 (~20ADR...)
  if (ascii_found && ascii_chars.size() >= 15) {
    parse_seplos_ascii_frame_(ascii_chars);
  }

  // 3. PARSING STREAM MODBUS RTU
  parse_seplos_modbus_frame_(rx_buffer_.data(), rx_buffer_.size());
}

void SeplosParserHub::parse_seplos_ascii_frame_(const std::vector<uint8_t> &frame) {
  std::string ascii_str(frame.begin(), frame.end());
  
  if (ascii_str.length() < 15) return;

  uint8_t raw_adr = 0;
  if (ascii_str.length() >= 5) {
    char adr_buf[3] = {ascii_str[3], ascii_str[4], '
