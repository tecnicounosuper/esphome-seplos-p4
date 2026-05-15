import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    CONF_ID, CONF_TYPE, CONF_ADDRESS, 
    DEVICE_CLASS_VOLTAGE, DEVICE_CLASS_CURRENT, DEVICE_CLASS_BATTERY,
    UNIT_VOLT, UNIT_AMPERE, UNIT_PERCENT
)
from . import seplos_v3_ns, SeplosV3

CONF_SEPLOS_V3_ID = "seplos_v3_id"

# Dizionario dei sensori supportati
TYPES = {
    "battery_voltage": sensor.sensor_schema(unit_of_measurement=UNIT_VOLT, accuracy_decimals=2, device_class=DEVICE_CLASS_VOLTAGE),
    "current": sensor.sensor_schema(unit_of_measurement=UNIT_AMPERE, accuracy_decimals=2, device_class=DEVICE_CLASS_CURRENT),
    "battery_soc": sensor.sensor_schema(unit_of_measurement=UNIT_PERCENT, accuracy_decimals=1, device_class=DEVICE_CLASS_BATTERY),
}

# Aggiungiamo automaticamente le 16 celle per evitare errori di battitura
for i in range(1, 17):
    TYPES[f"cell_{i}_voltage"] = sensor.sensor_schema(unit_of_measurement=UNIT_VOLT, accuracy_decimals=3, device_class=DEVICE_CLASS_VOLTAGE)

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(CONF_SEPLOS_V3_ID): cv.use_id(SeplosV3),
    cv.Required(CONF_ADDRESS): cv.int_range(min=1, max=16),
    cv.Required(CONF_TYPE): cv.one_of(*TYPES, lower=True),
}).extend(sensor.sensor_schema()).extend(cv.COMPONENT_SCHEMA)

def to_code(config):
    hub = yield cg.get_variable(config[CONF_SEPLOS_V3_ID])
    var = yield sensor.new_sensor(config)
    cg.add(hub.register_sensor(config[CONF_ADDRESS], config[CONF_TYPE], var))
