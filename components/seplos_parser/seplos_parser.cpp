#include "seplos_parser.h"
#include "esphome/core/log.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/helpers.h"
#include
#include
namespace esphome {
namespace seplos_parser {
static const char *TAG = "seplos_parser.component";
void SeplosParser::setup() {
// Initialisierung der Sensorvektoren
std::vector *> sensor_vectors = {
&pack_voltage_, ¤t_, &remaining_capacity_, &total_capacity_,
&total_discharge_capacity_, &soc_, &soh_, &cycle_count_,
&average_cell_voltage_, &average_cell_temp_,
&max_cell_voltage_,
&min_cell_voltage_, &delta_cell_voltage_, &max_cell_temp_,
&min_cell_temp_,
&maxdiscurt_, &maxchgcurt_, &cell_1_, &cell_2_, &cell_3_,
&cell_4_,
&cell_5_, &cell_6_, &cell_7_, &cell_8_, &cell_9_, &cell_10_,
&cell_11_, &cell_12_, &cell_13_, &cell_14_, &cell_15_,
&cell_16_,
&cell_temp_1_, &cell_temp_2_, &cell_temp_3_, &cell_temp_4_,
&case_temp_, &power_temp_
};
}
void SeplosParser::loop() {
while (this->available()) {
uint8_t byte;
this->read_byte(&byte);
this->rx_buffer_.push_back(byte);
}
while (this->rx_buffer_.size() >= 4) {
// Ricerca dell'inizio del frame (0x01)
if (this->rx_buffer_[0] != 0x01) {

this->rx_buffer_.pop_front();
continue;
}
// Verifica della funzione e della lunghezza (Logica estesa per
0x03 e 0x04)
uint8_t func = this->rx_buffer_[1];
uint8_t byte_count = this->rx_buffer_[2];
// Si assume una struttura standard BMS Seplos V3
size_t frame_len = 3 + byte_count + 2; // Funzione + Payload + CRC
if (this->rx_buffer_.size() < frame_len) break;
// Controllo CRC (implementazione standard Modbus)
// Qui andrebbe la logica di calcolo CRC per validare il pacchetto
// Esempio di elaborazione
this->process_packet();
// Rimuovi il pacchetto elaborato dal buffer
for(size_t i = 0; i < frame_len; i++)
this->rx_buffer_.pop_front();
}
}
void SeplosParser::process_packet() {
// Logica di parsing specifica per mappare i byte sui sensori
ESP_LOGD(TAG, "Pacchetto ricevuto e processato.");
}
void SeplosParser::set_bms_count(int bms_count) {
this->bms_count_ = bms_count;
ESP_LOGI(TAG, "BMS Count impostato a: %d", bms_count);
}
void SeplosParser::dump_config() {
ESP_LOGCONFIG(TAG, "Seplos Parser configurato.");
}
} // namespace seplos_parser
} // namespace esphome
