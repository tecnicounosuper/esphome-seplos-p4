import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart
from esphome.const import CONF_ID

DEPENDENCIES = ["uart"]
MULTI_CONF = True

CONF_BMS_COUNT = "bms_count"
CONF_UPDATE_INTERVAL = "update_interval"
CONF_SEPLOS_PARSER_ID = "seplos_parser_id"

seplos_parser_ns = cg.esphome_ns.namespace("seplos_parser")
SeplosParser = seplos_parser_ns.class_("SeplosParser", cg.Component, uart.UARTDevice)

HUB_CHILD_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_SEPLOS_PARSER_ID): cv.use_id(SeplosParser),
    }
)

CONFIG_SCHEMA = (
    cv.Schema({
        cv.GenerateID(): cv.declare_id(SeplosParser),
        cv.Optional(CONF_UPDATE_INTERVAL, default=5): cv.int_,
        cv.Optional(CONF_BMS_COUNT, default=1): cv.int_,
    })
    .extend(cv.COMPONENT_SCHEMA)
    .extend(uart.UART_DEVICE_SCHEMA)
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)
    
    cg.add(var.set_bms_count(config[CONF_BMS_COUNT]))
    cg.add(var.set_update_interval(config[CONF_UPDATE_INTERVAL]))
