import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    CONF_ID, CONF_TYPE, CONF_ADDRESS, 
    DEVICE_CLASS_VOLTAGE, DEVICE_CLASS_CURRENT, DEVICE_CLASS_BATTERY, DEVICE_CLASS_TEMPERATURE,
    UNIT_VOLT, UNIT_AMPERE, UNIT_PERCENT, UNIT_CELSIUS, UNIT_AMPERE_HOUR
)
from . import seplos_v3_ns, SeplosV3

CONF_SEPLOS_V3_ID = "seplos_v3_id"

TYPES = {
    "battery_voltage": sensor.sensor_schema(unit_of_measurement=UNIT_VOLT, accuracy_decimals=2, device_class=DEVICE_CLASS_VOLTAGE),
    "current": sensor.sensor_schema(unit_of_measurement=UNIT_AMPERE, accuracy_decimals=2, device_class=DEVICE_CLASS_CURRENT),
    "battery_soc": sensor.sensor_schema(unit_of_measurement=UNIT_PERCENT, accuracy_decimals=1, device_class=DEVICE_CLASS_BATTERY),
    "remaining_capacity": sensor.sensor_schema(unit_of_measurement=UNIT_AMPERE_HOUR, accuracy_decimals=2, icon="mdi:battery-charging-100"),
    "total_capacity": sensor.sensor_schema(unit_of_measurement=UNIT_AMPERE_HOUR, accuracy_decimals=2, icon="mdi:battery-100"),
}

# Genera automaticamente cell_1_voltage ... cell_16_voltage
for i in range(1, 17):
    TYPES[f"cell_{i}_voltage"] = sensor.sensor_schema(unit_of_measurement=UNIT_VOLT, accuracy_decimals=3, device_class=DEVICE_CLASS_VOLTAGE)

# Genera automaticamente temperature_1 ... temperature_4 (Sensori celle)
# e temperature_5 (Temperatura ambiente/MOSFET)
for i in range(1, 6):
    TYPES[f"temperature_{i}"] = sensor.sensor_schema(unit_of_measurement=UNIT_CELSIUS, accuracy_decimals=1, device_class=DEVICE_CLASS_TEMPERATURE)

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(CONF_SEPLOS_V3_ID): cv.use_id(SeplosV3),
    cv.Required(CONF_ADDRESS): cv.int_range(min=1, max=16),
    cv.Required(CONF_TYPE): cv.one_of(*TYPES, lower=True),
}).extend(sensor.sensor_schema()).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    hub = await cg.get_variable(config[CONF_SEPLOS_V3_ID])
    var = await sensor.new_sensor(config)
    cg.add(hub.register_sensor(config[CONF_ADDRESS], config[CONF_TYPE], var))
