import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart
from esphome.const import CONF_ID

DEPENDENCIES = ['uart']
AUTO_LOAD = ['sensor', 'text_sensor']

seplos_v3_ns = cg.esphome_ns.namespace('seplos_v3')
SeplosV3 = seplos_v3_ns.class_('SeplosV3', cg.PollingComponent, uart.UARTDevice)

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(SeplosV3),
}).extend(cv.polling_component_schema('60s')).extend(uart.uart_device_schema(bauds=[19200]))

def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    yield cg.register_component(var, config)
    yield uart.register_uart_device(var, config)
