import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart
from esphome.const import CONF_ID

DEPENDENCIES = ["uart"]
AUTO_LOAD = ["sensor", "binary_sensor", "text_sensor"]

# Definiamo il namespace del componente
seplos_parser_ns = cg.esphome_ns.namespace("seplos_parser")
SeplosParser = seplos_parser_ns.class_("SeplosParser", cg.Component, uart.UARTDevice)

CONF_BMS_COUNT = "bms_count"
CONF_UPDATE_INTERVAL = "update_interval"

# Usate anche da sensor.py / binary_sensor.py / text_sensor.py
CONF_SEPLOS_PARSER_ID = "seplos_parser_id"
CONF_BMS_INDEX = "bms_index"

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(SeplosParser),
            cv.Optional(CONF_BMS_COUNT, default=1): cv.int_range(min=1, max=16),
            cv.Optional(CONF_UPDATE_INTERVAL, default=10): cv.int_range(min=1),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(uart.UART_DEVICE_SCHEMA)
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    cg.add(var.set_bms_count(config[CONF_BMS_COUNT]))
    cg.add(var.set_update_interval(config[CONF_UPDATE_INTERVAL]))
