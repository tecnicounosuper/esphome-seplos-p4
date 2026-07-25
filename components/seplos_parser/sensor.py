import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    CONF_TYPE,
    STATE_CLASS_MEASUREMENT,
    DEVICE_CLASS_VOLTAGE,
    DEVICE_CLASS_CURRENT,
    DEVICE_CLASS_BATTERY,
    DEVICE_CLASS_TEMPERATURE,
    UNIT_VOLT,
    UNIT_AMPERE,
    UNIT_PERCENT,
    UNIT_CELSIUS,
)
from . import seplos_parser_ns, SeplosParserHub

CONF_SEPLOS_PARSER_ID = "seplos_parser_id"
CONF_BMS_INDEX = "bms_index"

# Tipi sensori registrati con i relativi setter C++ sull'hub Seplos
TYPES = {
    "pack_voltage": "set_pack_voltage_sensor",
    "current": "set_current_sensor",
    "soc": "set_soc_sensor",
    "soh": "set_soh_sensor",
    "remaining_capacity": "set_remaining_capacity_sensor",
    "cycles": "set_cycles_sensor",
    "min_cell_voltage": "set_min_cell_voltage_sensor",
    "max_cell_voltage": "set_max_cell_voltage_sensor",
    "cell_delta_voltage": "set_cell_delta_voltage_sensor",
    "temp_1": "set_temp1_sensor",
    "temp_2": "set_temp2_sensor",
    "temp_3": "set_temp3_sensor",
    "temp_4": "set_temp4_sensor",
    "mos_temp": "set_mos_temp_sensor",
}

# Genera dinamicamente le opzioni per le 16 celle (cell_1 ... cell_16)
for i in range(1, 17):
    TYPES[f"cell_{i}"] = f"set_cell_{i}_sensor"

CONFIG_SCHEMA = (
    sensor.sensor_schema(
        accuracy_decimals=2,
        state_class=STATE_CLASS_MEASUREMENT,
    )
    .extend(
        {
            cv.GenerateID(CONF_SEPLOS_PARSER_ID): cv.use_id(SeplosParserHub),
            cv.Required(CONF_BMS_INDEX): cv.int_range(min=0, max=15),
            cv.Required(CONF_TYPE): cv.one_of(*TYPES, lower=True),
        }
    )
)

async def to_code(config):
    hub = await cg.get_variable(config[CONF_SEPLOS_PARSER_ID])
    var = await sensor.new_sensor(config)
    
    setter_name = TYPES[config[CONF_TYPE]]
    setter = getattr(hub, setter_name)
    cg.add(setter(config[CONF_BMS_INDEX], var))
