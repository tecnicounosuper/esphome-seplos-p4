import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart
from esphome.const import CONF_ID, CONF_UPDATE_INTERVAL

DEPENDENCIES = ["uart"]
AUTO_LOAD = ["sensor"]

seplos_parser_ns = cg.esphome_ns.namespace("seplos_parser")
SeplosParserHub = seplos_parser_ns.class_("SeplosParserHub", cg.PollingComponent, uart.UARTDevice)

CONF_BMS_COUNT = "bms_count"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(SeplosParserHub),
        cv.Optional(CONF_BMS_COUNT, default=2): cv.int_range(min=1, max=16),
    }
).extend(cv.polling_component_schema("10s")).extend(uart.UART_DEVICE_SCHEMA)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)
    cg.add(var.set_bms_count(config[CONF_BMS_COUNT]))
