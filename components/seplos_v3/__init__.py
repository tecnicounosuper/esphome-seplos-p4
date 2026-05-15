import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart
from esphome.const import CONF_ID

# Definiamo le dipendenze
DEPENDENCIES = ['uart']
AUTO_LOAD = ['sensor']

seplos_v3_ns = cg.esphome_ns.namespace('seplos_v3')
SeplosV3 = seplos_v3_ns.class_('SeplosV3', cg.PollingComponent, uart.UARTDevice)

# La correzione principale è qui sotto
CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(SeplosV3),
}).extend(cv.polling_component_schema('60s')).extend(uart.UART_DEVICE_SCHEMA)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)
