import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    CONF_ID,
    CONF_TYPE,
    CONF_ADDRESS,
    UNIT_VOLT,
    UNIT_AMPERE,
    UNIT_PERCENT,
    DEVICE_CLASS_VOLTAGE,
    DEVICE_CLASS_CURRENT,
    DEVICE_CLASS_BATTERY,
    STATE_CLASS_MEASUREMENT,
)
from . import seplos_v3_ns, SeplosComponent

# Definizione dei tipi di sensore supportati
TYPES = {
    "battery_voltage": sensor.sensor_schema(
        unit_of_measurement=UNIT_VOLT,
        accuracy_decimals=2,
        device_class=DEVICE_CLASS_VOLTAGE,
        state_class=STATE_CLASS_MEASUREMENT,
    ),
    "current": sensor.sensor_schema(
        unit_of_measurement=UNIT_AMPERE,
        accuracy_decimals=2,
        device_class=DEVICE_CLASS_CURRENT,
        state_class=STATE_CLASS_MEASUREMENT,
    ),
    "soc": sensor.sensor_schema(
        unit_of_measurement=UNIT_PERCENT,
        accuracy_decimals=1,
        device_class=DEVICE_CLASS_BATTERY,
        state_class=STATE_CLASS_MEASUREMENT,
    ),
}

CONF_SEPLOS_V3_ID = "seplos_v3_id"

# CORREZIONE: Usiamo sensor.sensor_schema().extend(...) invece di sensor.SENSOR_SCHEMA
CONFIG_SCHEMA = sensor.sensor_schema().extend({
    cv.GenerateID(CONF_SEPLOS_V3_ID): cv.use_id(SeplosComponent),
    cv.Required(CONF_ADDRESS): cv.int_range(min=0, max=15),
    cv.Required(CONF_TYPE): cv.one_of(*TYPES, lower=True),
})

async def to_code(config):
    hub = await cg.get_variable(config[CONF_SEPLOS_V3_ID])
    
    # Crea l'oggetto sensore basandosi sul tipo scelto nel YAML
    # Recuperiamo lo schema specifico dal dizionario TYPES
    type_config = TYPES[config[CONF_TYPE]]
    var = await sensor.new_sensor(config)
    
    cg.add(hub.register_sensor(config[CONF_ADDRESS], config[CONF_TYPE], var))
