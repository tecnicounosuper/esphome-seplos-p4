#include "seplos_v3.h"
#include "esphome/core/log.h"

namespace esphome {
namespace seplos_v3 {

static const char *TAG = "seplos_v3";

void SeplosV3::setup() {
    ESP_LOGI(TAG, "Seplos V3 Sniffer inizializzato");
}

void SeplosV3::register_sensor(uint8_t address, std::string type, sensor::Sensor *obj) {
    this->bms_list_[address].sensors[type] = obj;
}

void SeplosV3::loop() {
    while (this->available()) {
        uint8_t data;
        this->read_byte(&data);
        ESP_LOGVV(TAG, "Dato ricevuto: %02X", data);
    }
}

void SeplosV3::update() {
    // Gestione aggiornamento dati ciclico
}

void SeplosV3::dump_config() {
    ESP_LOGCONFIG(TAG, "Seplos V3:");
    for (auto const& [addr, bms] : this->bms_list_) {
        ESP_LOGCONFIG(TAG, "  BMS Indirizzo: %d monitorato", addr);
    }
}

}  // namespace seplos_v3
}  // namespace esphome
