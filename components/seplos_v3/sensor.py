import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    CONF_ID,
    CONF_ADDRESS,
    CONF_TYPE,
    ICON_BATTERY,
    ICON_FLASH,
    UNIT_VOLT,
    UNIT_PERCENT,
    UNIT_AMPERE,
)
from . import seplos_v3_ns, SeplosComponent

DEPENDENCIES = ["seplos_v3"]

# Tipi di sensori supportati basati sul manuale Seplos V3
TYPES = {
    "battery_voltage": sensor.sensor_schema(
        unit_of_measurement=UNIT_VOLT,
        icon=ICON_FLASH,
        accuracy_decimals=2,
    ),
    "battery_soc": sensor.sensor_schema(
        unit_of_measurement=UNIT_PERCENT,
        icon=ICON_BATTERY,
        accuracy_decimals=1,
    ),
    "current": sensor.sensor_schema(
        unit_of_measurement=UNIT_AMPERE,
        icon="mdi:lightning-bolt",
        accuracy_decimals=2,
    ),
}

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID("seplos_v3_id"): cv.use_id(SeplosComponent),
        cv.Required(CONF_ADDRESS): cv.int_range(min=0, max=16),
        cv.Required(CONF_TYPE): cv.one_of(*TYPES, lower=True),
    }
).extend(sensor.sensor_schema()).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    parent = await cg.get_variable(config["seplos_v3_id"])
    var = await sensor.new_sensor(config)
    cg.add(parent.register_sensor(config[CONF_ADDRESS], config[CONF_TYPE], var))
