# ESPHome Seplos V3 Sniffer per ESP32-P4 (Guition M3)

Componente esterno ESPHome per intercettare passivamente (Sniffer UART RS485 a 19200 baud) i dati dei BMS Seplos V3 (Master/Slave).

## Struttura Repository
- `components/seplos_parser/__init__.py`: Definizione schema hub ESPHome
- `components/seplos_parser/sensor.py`: Schema dei sensori per BMS Index
- `components/seplos_parser/seplos_parser.h`: Header C++
- `components/seplos_parser/seplos_parser.cpp`: Parser frame ASCII / Modbus

## Uso nello YAML ESPHome

```yaml
external_components:
  - source:
      type: git
      url: https://github.com/tecnicounosuper/esphome-seplos-p4.git
      ref: main
    refresh: 0s

seplos_parser:
  id: seplos_hub
  uart_id: uart_0
  bms_count: 2
  update_interval: 10s

sensor:
  - platform: seplos_parser
    seplos_parser_id: seplos_hub
    bms_index: 0
    type: pack_voltage
    name: "BMS 0 Tensione Pacco"
```
