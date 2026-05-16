#include "seplos_v3.h"
#include "esphome/core/log.h"

namespace esphome {
namespace seplos_v3 {

static const char *TAG = "seplos_v3";

void SeplosV3::setup() {
    ESP_LOGI(TAG, "Seplos V3 Sniffer Dinamico Inizializzato");
}

void SeplosV3::register_sensor(uint8_t address, std::string type, sensor::Sensor *obj) {
    this->bms_list_[address].sensors[type] = obj;
}

void SeplosV3::loop() {
    while (this->available()) {
        uint8_t data;
        this->read_byte(&data);
        this->rx_buffer_.push_back(data);

        // Controllo minimo: abbiamo almeno i primi 3 byte (Addr, Func, Byte Count)?
        if (this->rx_buffer_.size() >= 3) {
            uint8_t expected_payload_len = this->rx_buffer_[2]; // Il BMS dichiara quanti byte di dati seguono
            size_t total_expected_frame_len = 3 + expected_payload_len + 2; // Intestazione(3) + Dati + CRC(2)

            // Se abbiamo ricevuto l'intero pacchetto dichiarato dal BMS
            if (this->rx_buffer_.size() >= total_expected_frame_len) {
                this->parse_modbus_frame_(this->rx_buffer_.data(), total_expected_frame_len);
                
                // Rimuoviamo il frame elaborato dal buffer mantenendo eventuali byte successivi
                this->rx_buffer_.erase(this->rx_buffer_.begin(), this->rx_buffer_.begin() + total_expected_frame_len);
            }
        }
    }
    
    // Protezione anti-saturazione se lo stream si sfascia
    if (this->rx_buffer_.size() > 512) {
        this->rx_buffer_.clear();
    }
}

void SeplosV3::parse_modbus_frame_(const uint8_t *frame, size_t length) {
    if (length < 5 || frame[1] != 0x03) return;

    uint8_t bms_addr = frame[0];
    
    // Verifica se il BMS è censito nello YAML
    if (this->bms_list_.find(bms_addr) == this->bms_list_.end()) return;

    // Controllo di integrità CRC16
    uint16_t computed_crc = this->crc16_(frame, length - 2);
    uint16_t received_crc = (frame[length - 1] << 8) | frame[length - 2];
    if (computed_crc != received_crc) {
        ESP_LOGW(TAG, "CRC errato da BMS ID %d (Ricevuto: %04X, Calcolato: %04X)", bms_addr, received_crc, computed_crc);
        return;
    }

    ESP_LOGD(TAG, "Frame Modbus valido ricevuto da BMS ID %d. Lunghezza totale: %d byte", bms_addr, length);

    auto &bmsSensors = this->bms_list_[bms_addr].sensors;
    size_t offset = 3; // Salta Addr, Func, ByteCount

    // 1. TENSIONI CELLE (16 celle, 2 byte l'una -> millivolt)
    for (int i = 1; i <= 16; i++) {
        std::string cell_key = "cell_" + std::to_string(i) + "_voltage";
        if (offset + 1 < length) {
            uint16_t raw_volt = (frame[offset] << 8) | frame[offset + 1];
            if (bmsSensors.find(cell_key) != bmsSensors.end()) {
                bmsSensors[cell_key]->publish_state(raw_volt / 1000.0f);
            }
            offset += 2;
        }
    }

    // 2. TEMPERATURE (5 sensori, 2 byte l'uno -> decimi di °C o Kelvin)
    // Se i valori letti sono vicini a ~2900-3000, sono espressi in Kelvin (es. 2981K = 25.0°C).
    // Se sono espressi in decimi di grado con offset, eseguiamo il parsing standard:
    for (int i = 1; i <= 5; i++) {
        std::string temp_key = "temperature_" + std::to_string(i);
        if (offset + 1 < length) {
            int16_t raw_temp = (frame[offset] << 8) | frame[offset + 1];
            if (bmsSensors.find(temp_key) != bmsSensors.end()) {
                // Controllo se il dato è in Kelvin (es. > 1000) o in decimi di grado diretti
                float final_temp = (raw_temp > 1000) ? (raw_temp / 10.0f) - 273.15f : (raw_temp / 10.0f);
                bmsSensors[temp_key]->publish_state(final_temp);
            }
            offset += 2;
        }
    }

    // 3. CORRENTE (2 byte, Signed Int, decimi di Ampere)
    if (offset + 1 < length) {
        int16_t raw_current = (frame[offset] << 8) | frame[offset + 1];
        if (bmsSensors.find("current") != bmsSensors.end()) {
            bmsSensors["current"]->publish_state(raw_current / 10.0f);
        }
        offset += 2;
    }

    // 4. TENSIONE TOTALE PACCO (2 byte, centesimi di Volt)
    if (offset + 1 < length) {
        uint16_t raw_pack_volt = (frame[offset] << 8) | frame[offset + 1];
        if (bmsSensors.find("battery_voltage") != bmsSensors.end()) {
            bmsSensors["battery_voltage"]->publish_state(raw_pack_volt / 100.0f);
        }
        offset += 2;
    }

    // 5. SOC (2 byte, decimi di %)
    if (offset + 1 < length) {
        uint16_t raw_soc = (frame[offset] << 8) | frame[offset + 1];
        if (bmsSensors.find("battery_soc") != bmsSensors.end()) {
            bmsSensors["battery_soc"]->publish_state(raw_soc / 10.0f);
        }
    }
}

void SeplosV3::update() {}

uint16_t SeplosV3::crc16_(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

void SeplosV3::dump_config() {
    ESP_LOGCONFIG(TAG, "Seplos V3 Sniffer Dinamico:");
    for (auto const& [addr, bms] : this->bms_list_) {
        ESP_LOGCONFIG(TAG, "  BMS Monitorato all'indirizzo Modbus: %d", addr);
    }
}

}  // namespace seplos_v3
}  // namespace esphome
