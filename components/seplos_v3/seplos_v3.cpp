#include "seplos_v3.h"
#include "esphome/core/log.h"

namespace esphome {
namespace seplos_v3 {

static const char *TAG = "seplos_v3";

void SeplosV3::setup() {
    ESP_LOGI(TAG, "Seplos V3 Sniffer Asincrono Inizializzato");
}

void SeplosV3::register_sensor(uint8_t address, std::string type, sensor::Sensor *obj) {
    this->bms_list_[address].sensors[type] = obj;
}

void SeplosV3::loop() {
    // 1. Leggiamo tutti i byte disponibili dalla UART e accumuliamoli nel buffer
    while (this->available()) {
        uint8_t data;
        this->read_byte(&data);
        this->rx_buffer_.push_back(data);
    }

    // 2. Analizziamo il buffer alla ricerca di frame Modbus validi
    while (this->rx_buffer_.size() >= 5) { // Almeno l'intestazione minima (Addr + Func + Len + 2 Byte CRC)
        uint8_t bms_addr = this->rx_buffer_[0];
        uint8_t func = this->rx_buffer_[1];
        uint8_t byte_count = this->rx_buffer_[2];

        // Validazione preliminare dei preamboli Seplos V3 (Indirizzi 1 o 2, Funzioni standard 3 o 4)
        if ((bms_addr == 1 || bms_addr == 2) && (func == 0x03 || func == 0x04) && (byte_count > 0 && byte_count <= 100)) {
            size_t total_expected_len = 3 + byte_count + 2; 

            // Se non sono ancora arrivati tutti i byte previsti per questo specifico pacchetto,
            // ci fermiamo e aspettiamo i prossimi cicli di loop senza distruggere il buffer
            if (this->rx_buffer_.size() < total_expected_len) {
                break; 
            }

            // Abbiamo l'intero pacchetto integro nel buffer: procediamo al parsing!
            this->parse_modbus_frame_(this->rx_buffer_.data(), total_expected_len);
            
            // Rimuoviamo SOLO i byte elaborati di questo pacchetto, preservando quello che c'è dopo
            this->rx_buffer_.erase(this->rx_buffer_.begin(), this->rx_buffer_.begin() + total_expected_len);
            continue; 
        } else {
            // Se i primi byte non corrispondono a un inizio frame valido, scartiamo solo il primo byte per riallinearci
            this->rx_buffer_.erase(this->rx_buffer_.begin());
        }
    }

    // Protezione per evitare saturazione della memoria in caso di rumore elettrico persistente sulla linea
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

    // CASO 1: Pacchetto Celle e Temperature (52 byte di dati = 0x34)
    if (byte_count == 0x34) {
        // Estrazione 16 Celle (2 byte ciascuna = 32 byte)
        for (int i = 1; i <= 16; i++) {
            std::string cell_key = "cell_" + std::to_string(i) + "_voltage";
            if (offset + 1 < length) {
                uint16_t raw_volt = (frame[offset] << 8) | frame[offset + 1];
                // Controllo di sanità del dato per escludere spike anomali
                if (raw_volt >= 2000 && raw_volt <= 4500) { 
                    if (bmsSensors.find(cell_key) != bmsSensors.end()) {
                        bmsSensors[cell_key]->publish_state(raw_volt / 1000.0f);
                    }
                }
                offset += 2;
            }
        }

        // Estrazione 5 Temperature (2 byte ciascuna = 10 byte)
        for (int i = 1; i <= 5; i++) {
            std::string temp_key = "temperature_" + std::to_string(i);
            if (offset + 1 < length) {
                int16_t raw_temp = (frame[offset] << 8) | frame[offset + 1];
                if (bmsSensors.find(temp_key) != bmsSensors.end()) {
                    // Conversione da Kelvin a Gradi Celsius se il valore è grezzo in K (>1000)
                    float final_temp = (raw_temp > 1000) ? (raw_temp / 10.0f) - 273.15f : (raw_temp / 10.0f);
                    if (final_temp > -20.0f && final_temp < 80.0f) {
                        bmsSensors[temp_key]->publish_state(final_temp);
                    }
                }
                offset += 2;
            }
        }
    } 
    // CASO 2: Pacchetti corti (Dati generali di riepilogo)
    else {
        // Eseguiamo il controllo CRC standard per garantire l'accuratezza dei parametri principali
        uint16_t computed_crc = this->crc16_(frame, length - 2);
        uint16_t received_crc = (frame[length - 1] << 8) | frame[length - 2];
        if (computed_crc != received_crc) return;

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

void SeplosV3::dump_config() {}

}  // namespace seplos_v3
}  // namespace esphome
