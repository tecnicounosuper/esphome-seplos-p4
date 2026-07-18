import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import text_sensor

from . import SeplosParser, CONF_SEPLOS_PARSER_ID, CONF_BMS_INDEX

DEPENDENCIES = ["seplos_parser"]

_schema_dict = {
    cv.GenerateID(CONF_SEPLOS_PARSER_ID): cv.use_id(SeplosParser),
    cv.Required(CONF_BMS_INDEX): cv.int_range(min=0, max=15),
    cv.Optional("system_status"): text_sensor.text_sensor_schema(),
    cv.Optional("active_alarms"): text_sensor.text_sensor_schema(),
}

CONFIG_SCHEMA = cv.Schema(_schema_dict)

ALL_KEYS = ["system_status", "active_alarms"]


async def to_code(config):
    hub = await cg.get_variable(config[CONF_SEPLOS_PARSER_ID])
    bms_index = config[CONF_BMS_INDEX]

    for key in ALL_KEYS:
        if key in config:
            sens = await text_sensor.new_text_sensor(config[key])
            cg.add(hub.set_text_sensor(bms_index, key, sens))
