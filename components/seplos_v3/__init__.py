import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart
from esphome.const import CONF_ID

# Nome del componente (deve corrispondere alla cartella su GitHub)
DEPENDENCIES = ['uart']

# Namespace: usa 'seplos_v3' se la cartella si chiama così
seplos_v3_ns = cg.esphome_ns.namespace('seplos_v3')
SeplosComponent = seplos_v3_ns.class_('SeplosComponent', cg.Component, uart.UARTDevice)

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(SeplosComponent),
}).extend(cv.COMPONENT_SCHEMA).extend(uart.UART_DEVICE_SCHEMA)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)
