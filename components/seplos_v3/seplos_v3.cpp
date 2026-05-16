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
    // 1. Accumuliamo i byte dalla UART nel buffer
    while (this->available()) {
        uint8_t data;
        this->read_byte(&data);
        this->rx_buffer_.push_back(data);
    }

    // 2. Analisi dinamica dei pacchetti con finestra scorrevole
    while (this->rx_buffer_.size() >= 5) { 
        uint8_t bms_addr = this->rx_buffer_[0];
        uint8_t func = this->rx_buffer_[1];
        uint8_t byte_count = this->rx_buffer_[2];

        // Filtro flessibile sugli indirizzi BMS (da 0 a 4) e funzioni Modbus standard (0x03 e 0x04)
        if ((bms_addr <= 4) && (func == 0x03 || func == 0x04) && (byte_count > 0 && byte_count <= 100)) {
            size_t total_expected_len = 3 + byte_count + 2; 

            // Se il frame non è ancora completo in seriale, aspettiamo il prossimo ciclo
            if (this->rx_buffer_.size() < total_expected_len) {
                break; 
            }

            // Elaborazione del pacchetto integro
            this->parse_modbus_frame_(this->rx_buffer_.data(), total_expected_len);
            
            // Svuotiamo il buffer solo per i byte consumati
            this->rx_buffer_.erase(this->rx_buffer_.begin(), this->rx_buffer_.begin() + total_expected_len);
            continue; 
        } else {
            // Se non c'è allineamento, scartiamo il primo byte e avanziamo
            this->rx_buffer_.erase(this->rx_buffer_.begin());
        }
    }

    if (this->rx_buffer_.size() > 1024) {
        this->rx_buffer_.clear();
    }
}

void SeplosV3::parse_modbus_frame_(const uint8_t *frame, size_t length) {
    uint8_t bms_addr = frame[0];
    uint8_t byte_count = frame[2];

    if (this->bms_list_.find(bms_addr) == this->bms_list_.end()) return;

    auto &bmsSensors = this->bms_list_[bms_addr].sensors;
    size_t offset = 3; 

    // =========================================================================
    // ESTRAZIONE DELLE CELLE (Attiva su qualsiasi pacchetto lungo, >= 32 byte)
    // =========================================================================
    if (byte_count >= 32) {
        ESP_LOGD(TAG, "BMS %d: Rilevato pacchetto celle/temperature (Byte count: %d)", bms_addr, byte_count);

        // Estrazione sequenziale delle 16 celle (2 byte l'una = 32 byte)
        for (int i = 1; i <= 16; i++) {
            std::string cell_key = "cell_" + std::to_string(i) + "_voltage";
            if (offset + 1 < length) {
                uint16_t raw_volt = (frame[offset] << 8) | frame[offset + 1];
                // Validazione range LiFePO4 (1.5V - 4.2V) per scartare letture sporche
                if (raw_volt >= 1500 && raw_volt <= 4200) { 
                    if (bmsSensors.find(cell_key) != bmsSensors.end()) {
                        bmsSensors[cell_key]->publish_state(raw_volt / 1000.0f);
                    }
                }
                offset += 2;
            }
        }

        // Estrazione delle temperature (se avanzano byte nel pacchetto prima del CRC)
        int temp_index = 1;
        while (offset + 1 < length - 2 && temp_index <= 4) {
            std::string temp_key = "temperature_" + std::to_string(temp_index);
            int16_t raw_temp = (frame[offset] << 8) | frame[offset + 1];
            
            // Gestione conversione Kelvin / Celsius
            float final_temp = (raw_temp > 1000) ? (raw_temp / 10.0f) - 273.15f : (raw_temp / 10.0f);
            if (final_temp > -20.0f && final_temp < 80.0f) {
                if (bmsSensors.find(temp_key) != bmsSensors.end()) {
                    bmsSensors[temp_key]->publish_state(final_temp);
                }
            }
            offset += 2;
            temp_index++;
        }
    } 
    // =========================================================================
    // ESTRAZIONE DATI GENERALI (Pacchetti di riepilogo corti, < 32 byte)
    // =========================================================================
    else {
        // Applichiamo il controllo CRC rigido sui riepiloghi per evitare sbalzi nei grafici di Home Assistant
        uint16_t computed_crc = this->crc16_(frame, length - 2);
        uint16_t received_crc = (frame[length - 1] << 8) | frame[length - 2];
        if (computed_crc != received_crc) return;

        ESP_LOGD(TAG, "BMS %d: Rilevato pacchetto riepilogo generale (Byte count: %d)", bms_addr, byte_count);

        // Se il pacchetto corto contiene almeno 6 byte di dati utili (es. i classici 3 registri standard)
        if (byte_count >= 6) {
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

void SeplosV3::dump_config() {}

}  // namespace seplos_v3
}  // namespace esphome
