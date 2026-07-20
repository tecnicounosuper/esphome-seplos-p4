import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor
from . import SeplosParser

CONF_SEPLOS_PARSER_ID = "seplos_parser_id"

CONFIG_SCHEMA = binary_sensor.binary_sensor_schema().extend(
    {
        cv.GenerateID(CONF_SEPLOS_PARSER_ID): cv.use_id(SeplosParser),
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_SEPLOS_PARSER_ID])
    sens = await binary_sensor.new_binary_sensor(config)
    cg.add(hub.register_binary_sensor(sens))
