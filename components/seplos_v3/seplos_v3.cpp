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

        // Se non arrivano byte per un po', consideriamo chiuso il pacchetto (Timeout Modbus)
        // In alternativa, se usiamo uno sniffer passivo, cerchiamo lunghezze tipiche dei frame Seplos V3 (es. ~75-80 byte)
        if (this->rx_buffer_.size() >= 79) { 
            this->parse_modbus_frame_(this->rx_buffer_.data(), this->rx_buffer_.size());
            this->rx_buffer_.clear();
        }
    }
    
    // Pulizia di sicurezza se il buffer si satura in modo anomalo
    if (this->rx_buffer_.size() > 256) {
        this->rx_buffer_.clear();
    }
}

void SeplosV3::parse_modbus_frame_(const uint8_t *frame, size_t length) {
    // Un pacchetto di risposta Modbus standard 0x03 valido ha almeno: 
    // Addr(1) + Func(1) + ByteCount(1) + ... data ... + CRC(2)
    if (length < 5 || frame[1] != 0x03) return;

    uint8_t bms_addr = frame[0];
    
    // Controlliamo se stiamo tracciando questo specifico indirizzo BMS
    if (this->bms_list_.find(bms_addr) == this->bms_list_.end()) return;

    // Verifica del CRC per evitare letture sporche sulla seriale
    uint16_t computed_crc = this->crc16_(frame, length - 2);
    uint16_t received_crc = (frame[length - 1] << 8) | frame[length - 2];
    if (computed_crc != received_crc) {
        ESP_LOGD(TAG, "Errore CRC Modbus da BMS ID %d", bms_addr);
        return;
    }

    auto &bmsSensors = this->bms_list_[bms_addr].sensors;
    
    // Prendendo spunto dal posizionamento dei registri Seplos V3:
    // L'offset varia a seconda che il frame sia una risposta parziale o totale.
    // Assumendo il frame completo standard mappato da DpunktS:
    size_t offset = 3; // Salta Addr, Func, ByteCount

    // 1. Lettura Tensioni 16 Celle (2 byte per cella in millivolt -> convertiti in Volt)
    for (int i = 1; i <= 16; i++) {
        std::string cell_key = "cell_" + std::to_string(i) + "_voltage";
        if (bmsSensors.find(cell_key) != bmsSensors.end() && (offset + 1 < length)) {
            uint16_t raw_volt = (frame[offset] << 8) | frame[offset + 1];
            bmsSensors[cell_key]->publish_state(raw_volt / 1000.0f);
            offset += 2;
        }
    }

    // 2. Lettura 5 Temperature (2 byte ciascuna, valore in Kelvin o decimi di grado a seconda della config)
    // Lo sniffer standard Seplos riporta il valore in decimi di grado Celsius con offset o Kelvin.
    // Ipotizzando decimi di Celsius (es: 250 = 25.0°C):
    for (int i = 1; i <= 5; i++) {
        std::string temp_key = "temperature_" + std::to_string(i);
        if (bmsSensors.find(temp_key) != bmsSensors.end() && (offset + 1 < length)) {
            int16_t raw_temp = (frame[offset] << 8) | frame[offset + 1];
            bmsSensors[temp_key]->publish_state(raw_temp / 10.0f);
            offset += 2;
        }
    }

    // 3. Corrente (2 byte - Signed Int - decimi di Ampere)
    if (bmsSensors.find("current") != bmsSensors.end() && (offset + 1 < length)) {
        int16_t raw_current = (frame[offset] << 8) | frame[offset + 1];
        bmsSensors["current"]->publish_state(raw_current / 10.0f);
        offset += 2;
    }

    // 4. Tensione Totale Pacco (2 byte - centesimi o decimi di Volt)
    if (bmsSensors.find("battery_voltage") != bmsSensors.end() && (offset + 1 < length)) {
        uint16_t raw_pack_volt = (frame[offset] << 8) | frame[offset + 1];
        bmsSensors["battery_voltage"]->publish_state(raw_pack_volt / 100.0f);
        offset += 2;
    }

    // 5. SOC (2 byte - percentuale o decimi di %)
    if (bmsSensors.find("battery_soc") != bmsSensors.end() && (offset + 1 < length)) {
        uint16_t raw_soc = (frame[offset] << 8) | frame[offset + 1];
        bmsSensors["battery_soc"]->publish_state(raw_soc / 10.0f);
    }
}

void SeplosV3::update() {
    // Se non siamo in ascolto passivo puro, possiamo forzare l'invio della richiesta Modbus qui
}

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
    ESP_LOGCONFIG(TAG, "Seplos V3 Sniffer Completo:");
    for (auto const& [addr, bms] : this->bms_list_) {
        ESP_LOGCONFIG(TAG, "  BMS Monitorato all'indirizzo Modbus: %d", addr);
    }
}

}  // namespace seplos_v3
}  // namespace esphome
