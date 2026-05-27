import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from . import SeplosParser

CONF_SEPLOS_PARSER_ID = "seplos_parser_id"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_SEPLOS_PARSER_ID): cv.use_id(SeplosParser),
    }
)

async def to_code(config):
    hub = await cg.get_variable(config[CONF_SEPLOS_PARSER_ID])
    cg.add(hub.register_sensor(hub))
