#include "seplos_v3.h"
#include "esphome/core/log.h"

namespace esphome {
namespace seplos_v3 {

static const char *TAG = "seplos_v3";

void SeplosV3::setup() {
    ESP_LOGI(TAG, "Seplos V3 Sniffer Avanzato Inizializzato");
}

void SeplosV3::register_sensor(uint8_t address, std::string type, sensor::Sensor *obj) {
    this->bms_list_[address].sensors[type] = obj;
}

void SeplosV3::loop() {
    while (this->available()) {
        uint8_t data;
        this->read_byte(&data);
        this->rx_buffer_.push_back(data);

        // Cerchiamo un header valido all'interno del buffer accumulato
        while (this->rx_buffer_.size() >= 3) {
            uint8_t bms_addr = this->rx_buffer_[0];
            uint8_t func = this->rx_buffer_[1];
            uint8_t byte_count = this->rx_buffer_[2];

            // Verifica se l'inizio sembra un pacchetto Modbus Seplos valido (ID 1 o 2, Funzione 3 o 4)
            if ((bms_addr == 1 || bms_addr == 2) && (func == 0x03 || func == 0x04) && (byte_count > 0 && byte_count <= 100)) {
                size_t total_expected_len = 3 + byte_count + 2; // Header(3) + Dati + CRC(2)

                // Abbiamo abbastanza byte per l'intero pacchetto?
                if (this->rx_buffer_.size() >= total_expected_len) {
                    this->parse_modbus_frame_(this->rx_buffer_.data(), total_expected_len);
                    // Rimuoviamo SOLO i byte del pacchetto elaborato
                    this->rx_buffer_.erase(this->rx_buffer_.begin(), this->rx_buffer_.begin() + total_expected_len);
                    continue; 
                } else {
                    // Il pacchetto è valido ma non è ancora arrivato tutto, usciamo dal loop e aspettiamo i prossimi byte
                    break;
                }
            } else {
                // Non è un inizio pacchetto valido, scartiamo il primo byte per riallinearci al prossimo giro
                this->rx_buffer_.erase(this->rx_buffer_.begin());
            }
        }
    }

    // Sicurezza per evitare sovraccarico di memoria
    if (this->rx_buffer_.size() > 512) {
        this->rx_buffer_.clear();
    }
}

void SeplosV3::parse_modbus_frame_(const uint8_t *frame, size_t length) {
    uint8_t bms_addr = frame[0];
    uint8_t byte_count = frame[2];

    if (this->bms_list_.find(bms_addr) == this->bms_list_.end()) return;

    // Verifica del CRC16 Modbus
    uint16_t computed_crc = this->crc16_(frame, length - 2);
    uint16_t received_crc = (frame[length - 1] << 8) | frame[length - 2];
    if (computed_crc != received_crc) {
        return; // Salta se il CRC fallisce
    }

    auto &bmsSensors = this->bms_list_[bms_addr].sensors;
    size_t offset = 3; // Salta Addr, Func, ByteCount

    // CASO 1: Pacchetto Celle e Temperature (Visto a 0x34 = 52 byte nel log)
    if (byte_count >= 36) {
        ESP_LOGI(TAG, "<<< Ricevuto Pacchetto Celle/Temperature da BMS %d (%d byte) >>>", bms_addr, byte_count);
        
        // 16 Celle (32 byte totali)
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

        // 5 Temperature (10 byte totali)
        for (int i = 1; i <= 5; i++) {
            std::string temp_key = "temperature_" + std::to_string(i);
            if (offset + 1 < length) {
                int16_t raw_temp = (frame[offset] << 8) | frame[offset + 1];
                if (bmsSensors.find(temp_key) != bmsSensors.end()) {
                    float final_temp = (raw_temp > 1000) ? (raw_temp / 10.0f) - 273.15f : (raw_temp / 10.0f);
                    bmsSensors[temp_key]->publish_state(final_temp);
                }
                offset += 2;
            }
        }
    } 
    // CASO 2: Pacchetto dati generali corto (Visto nei log di bms0 che aggiornano V, A, SOC)
    else {
        if (bmsSensors.find("current") != bmsSensors.end() && (offset + 1 < length)) {
            int16_t raw_current = (frame[offset] << 8) | frame[offset + 1];
            bmsSensors["current"]->publish_state(raw_current / 10.0f);
            offset += 2;
        }
        if (bmsSensors.find("battery_voltage") != bmsSensors.end() && (offset + 1 < length)) {
            uint16_t raw_pack_volt = (frame[offset] << 8) | frame[offset + 1];
            bmsSensors["battery_voltage"]->publish_state(raw_pack_volt / 100.0f);
            offset += 2;
        }
        if (bmsSensors.find("battery_soc") != bmsSensors.end() && (offset + 1 < length)) {
            uint16_t raw_soc = (frame[offset] << 8) | frame[offset + 1];
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
    ESP_LOGCONFIG(TAG, "Seplos V3 Sniffer:");
}

}  // namespace seplos_v3
}  // namespace esphome
