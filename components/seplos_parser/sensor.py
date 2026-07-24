import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    CONF_ID,
    CONF_NAME,
    CONF_TYPE,
    CONF_UNIT_OF_MEASUREMENT,
    CONF_DEVICE_CLASS,
    CONF_STATE_CLASS,
    CONF_ACCURACY_DECIMALS,
    DEVICE_CLASS_VOLTAGE,
    DEVICE_CLASS_CURRENT,
    DEVICE_CLASS_BATTERY,
    STATE_CLASS_MEASUREMENT,
)
from . import seplos_parser_ns, SeplosParserHub

CONF_SEPLOS_PARSER_ID = "seplos_parser_id"
CONF_BMS_INDEX = "bms_index"

SeplosSensor = seplos_parser_ns.class_("SeplosSensor", sensor.Sensor)

TYPES = [
    "pack_voltage",
    "current",
    "soc",
    "remaining_capacity",
]

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(SeplosSensor),
        cv.Required(CONF_SEPLOS_PARSER_ID): cv.use_id(SeplosParserHub),
        cv.Required(CONF_BMS_INDEX): cv.int_range(min=0, max=15),
        cv.Required(CONF_TYPE): cv.one_of(*TYPES, lower=True),
    }
).extend(sensor.sensor_schema(
    accuracy_decimals=2,
    state_class=STATE_CLASS_MEASUREMENT,
))

async def to_code(config):
    hub = await cg.get_variable(config[CONF_SEPLOS_PARSER_ID])
    var = cg.new_Pvariable(config[CONF_ID])
    await sensor.register_sensor(var, config)
    
    cg.add(var.set_bms_index(config[CONF_BMS_INDEX]))
    cg.add(var.set_sensor_type(config[CONF_TYPE]))
    cg.add(hub.register_sensor(var))
