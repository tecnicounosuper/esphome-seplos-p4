import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    CONF_ID,
    CONF_CURRENT,
    CONF_VOLTAGE,
    DEVICE_CLASS_VOLTAGE,
    DEVICE_CLASS_CURRENT,
    DEVICE_CLASS_BATTERY,
    DEVICE_CLASS_TEMPERATURE,
    STATE_CLASS_MEASUREMENT,
    UNIT_VOLT,
    UNIT_AMPERE,
    UNIT_PERCENT,
    UNIT_CELSIUS,
)
from . import seplos_v3_ns, SeplosV3

CONF_SEPLOS_V3_ID = "seplos_v3_id"
CONF_PACK_VOLTAGE = "pack_voltage"
CONF_SOC = "soc"

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(CONF_SEPLOS_V3_ID): cv.use_id(SeplosV3),
    cv.Optional(CONF_PACK_VOLTAGE): sensor.sensor_schema(
        unit_of_measurement=UNIT_VOLT,
        accuracy_decimals=2,
        device_class=DEVICE_CLASS_VOLTAGE,
        state_class=STATE_CLASS_MEASUREMENT,
        icon="mdi:battery-high",
    ),
    cv.Optional(CONF_CURRENT): sensor
