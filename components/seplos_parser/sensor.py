import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    CONF_ID,
    CONF_TYPE,
    CONF_VOLTAGE,
    CONF_CURRENT,
    CONF_BATTERY_LEVEL,
    CONF_CAPACITY,
    DEVICE_CLASS_VOLTAGE,
    DEVICE_CLASS_CURRENT,
    DEVICE_CLASS_BATTERY,
    STATE_CLASS_MEASUREMENT,
    UNIT_VOLT,
    UNIT_AMPERE,
    UNIT_PERCENT,
    UNIT_AMPERE_HOURS,
)

# Riferimento al namespace e all'Hub definiti in __init__.py
from . import seplos_parser_ns, SeplosParser, CONF_SEPLOS_PARSER_ID, CONF_BMS_INDEX

# Definizione del tipo C++ per il sensore
SeplosSensor = seplos_parser_ns.class_("SeplosSensor", sensor.Sensor, cg.Component)

# Schema dei tipi di sensori supportati
SENSOR_TYPES = {
    "voltage": sensor.sensor_schema(
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
        accuracy_decimals=0,
        device_class=DEVICE_CLASS_BATTERY,
        state_class=STATE_CLASS_MEASUREMENT,
    ),
    "capacity": sensor.sensor_schema(
        unit_of_measurement=UNIT_AMPERE_HOURS,
        accuracy_decimals=2,
        state_class=STATE_CLASS_MEASUREMENT,
    ),
}

CONFIG_SCHEMA = cv.All(
    sensor.sensor_schema(SeplosSensor)
    .extend(
        {
            cv.GenerateID(CONF_SEPLOS_PARSER_ID): cv.use_id(SeplosParser),
            cv.Required(CONF_BMS_INDEX): cv.int_range(min=0, max=15),
            cv.Required(CONF_TYPE): cv.enum(SENSOR_TYPES, lower=True),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
)

async def to_code(config):
    var = cg.new_variable(config[CONF_ID], config[CONF_TYPE])
    await sensor.register_sensor(var, config)
    await cg.register_component(var, config)

    hub = await cg.get_variable(config[CONF_SEPLOS_PARSER_ID])
    cg.add(hub.register_sensor(config[CONF_BMS_INDEX], config[CONF_TYPE], var))
