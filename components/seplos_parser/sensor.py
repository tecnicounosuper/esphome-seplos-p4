import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    CONF_TYPE,
    STATE_CLASS_MEASUREMENT,
)
from . import seplos_parser_ns, SeplosParserHub

CONF_SEPLOS_PARSER_ID = "seplos_parser_id"
CONF_BMS_INDEX = "bms_index"

TYPES = {
    "pack_voltage": "set_pack_voltage_sensor",
    "current": "set_current_sensor",
    "soc": "set_soc_sensor",
    "remaining_capacity": "set_remaining_capacity_sensor",
}

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
