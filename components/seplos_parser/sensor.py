import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor

from . import SeplosParser, CONF_SEPLOS_PARSER_ID, CONF_BMS_INDEX

DEPENDENCIES = ["seplos_parser"]

# chiave_yaml: (unit, decimali_precisione, device_class, state_class)
SCALAR_SENSORS = {
    "pack_voltage": ("V", 2, "voltage", "measurement"),
    "current": ("A", 2, "current", "measurement"),
    "remaining_capacity": ("Ah", 2, "", "measurement"),
    "total_capacity": ("Ah", 2, "", "measurement"),
    "total_discharge_capacity": ("Ah", 2, "", "measurement"),
    "soc": ("%", 1, "battery", "measurement"),
    "soh": ("%", 1, "battery", "measurement"),
    "cycle_count": ("", 0, "", "measurement"),
    "average_cell_voltage": ("V", 3, "voltage", "measurement"),
    "average_cell_temp": ("°C", 1, "temperature", "measurement"),
    "max_cell_voltage": ("V", 3, "voltage", "measurement"),
    "min_cell_voltage": ("V", 3, "voltage", "measurement"),
    "delta_cell_voltage": ("V", 3, "voltage", "measurement"),
    "max_cell_temp": ("°C", 1, "temperature", "measurement"),
    "min_cell_temp": ("°C", 1, "temperature", "measurement"),
    "maxdiscurt": ("A", 1, "current", "measurement"),
    "maxchgcurt": ("A", 1, "current", "measurement"),
    "case_temp": ("°C", 1, "temperature", "measurement"),
    "power_temp": ("°C", 1, "temperature", "measurement"),
}

CELL_KEYS = [f"cell_{i}" for i in range(1, 17)]
TEMP_KEYS = [f"cell_temp_{i}" for i in range(1, 5)]

_schema_dict = {
    cv.GenerateID(CONF_SEPLOS_PARSER_ID): cv.use_id(SeplosParser),
    cv.Required(CONF_BMS_INDEX): cv.int_range(min=0, max=15),
}

for _key, (_unit, _decimals, _device_class, _state_class) in SCALAR_SENSORS.items():
    _schema_dict[cv.Optional(_key)] = sensor.sensor_schema(
        unit_of_measurement=_unit,
        accuracy_decimals=_decimals,
        device_class=_device_class,
        state_class=_state_class,
    )

for _key in CELL_KEYS:
    _schema_dict[cv.Optional(_key)] = sensor.sensor_schema(
        unit_of_measurement="V",
        accuracy_decimals=3,
        device_class="voltage",
        state_class="measurement",
    )

for _key in TEMP_KEYS:
    _schema_dict[cv.Optional(_key)] = sensor.sensor_schema(
        unit_of_measurement="°C",
        accuracy_decimals=1,
        device_class="temperature",
        state_class="measurement",
    )

CONFIG_SCHEMA = cv.Schema(_schema_dict)

ALL_KEYS = list(SCALAR_SENSORS.keys()) + CELL_KEYS + TEMP_KEYS


async def to_code(config):
    hub = await cg.get_variable(config[CONF_SEPLOS_PARSER_ID])
    bms_index = config[CONF_BMS_INDEX]

    for key in ALL_KEYS:
        if key in config:
            sens = await sensor.new_sensor(config[key])
            cg.add(hub.set_sensor(bms_index, key, sens))
