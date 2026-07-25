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
  if (rx_buffer_.empty()) return;

  // 1. SEPARAZIONE CARATTERI ASCII DA BYTE BINARI
  std::vector<uint8_t> ascii_chars;
  std::vector<uint8_t> binary_bytes;

  bool ascii_found = false;
  for (uint8_t b : rx_buffer_) {
    // Caratteri ASCII validi (Header '~'=0x7E, Spazio 0x20, cifre 0-9, A-F, a-f, CR 0x0D, LF 0x0A)
    if (b == 0x7E || b == 0x20 || (b >= '0' && b <= '9') || (b >= 'A' && b <= 'F') || (b >= 'a' && b <= 'f') || b == 0x0D || b == 0x0A) {
      ascii_chars.push_back(b);
      if (b == 0x7E || b == 0x20) ascii_found = true;
    } else {
      binary_bytes.push_back(b);
    }
  }

  // 2. PARSING FRAME ASCII (se presenti sequenze ASCII con header Seplos ~ / 0x20)
  if (ascii_found && ascii_chars.size() >= 15) {
    parse_seplos_ascii_frame_(ascii_chars);
  }

  // 3. PARSING STREAM BINARIO / TELEMETRIA MODBUS SENZA FILTRI VINCOLANTI SU ADDRESS MODBUS
  parse_seplos_modbus_frame_(rx_buffer_.data(), rx_buffer_.size());
}

void SeplosParserHub::parse_seplos_ascii_frame_(const std::vector<uint8_t> &frame) {
  std::string ascii_str(frame.begin(), frame.end());
  
  if (ascii_str.length() < 15) return;

  uint8_t bms_idx = 0;
  if (ascii_str.length() >= 5) {
    char adr_buf[3] = {ascii_str[3], ascii_str[4], '
