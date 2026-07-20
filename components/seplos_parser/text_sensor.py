import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import text_sensor
from . import SeplosParser

CONF_SEPLOS_PARSER_ID = "seplos_parser_id"

CONFIG_SCHEMA = text_sensor.text_sensor_schema().extend(
    {
        cv.GenerateID(CONF_SEPLOS_PARSER_ID): cv.use_id(SeplosParser),
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_SEPLOS_PARSER_ID])
    sens = await text_sensor.new_text_sensor(config)
    cg.add(hub.register_text_sensor(sens))
