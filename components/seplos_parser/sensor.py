import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    DEVICE_CLASS_BATTERY,
    DEVICE_CLASS_CURRENT,
    DEVICE_CLASS_VOLTAGE,
    STATE_CLASS_MEASUREMENT,
    UNIT_AMPERE,
    UNIT_PERCENT,
    UNIT_VOLT,
)
from . import SeplosParser

CONF_SEPLOS_PARSER_ID = "seplos_parser_id"

CONFIG_SCHEMA = sensor.sensor_schema().extend(
    {
        cv.GenerateID(CONF_SEPLOS_PARSER_ID): cv.use_id(SeplosParser),
    }
)

async def to_code(config):
    hub = await cg.get_variable(config[CONF_SEPLOS_PARSER_ID])
    
    # Assegnazione automatica di unità e device class in base al nome dello YAML
    name = config.get("name", "").lower()
    
    if "voltage" in name or "pack_voltage" in name:
        config.setdefault("unit_of_measurement", UNIT_VOLT)
        config.setdefault("device_class", DEVICE_CLASS_VOLTAGE)
        config.setdefault("state_class", STATE_CLASS_MEASUREMENT)
        config.setdefault("accuracy_decimals", 2)
    elif "current" in name:
        config.setdefault("unit_of_measurement", UNIT_AMPERE)
        config.setdefault("device_class", DEVICE_CLASS_CURRENT)
        config.setdefault("state_class", STATE_CLASS_MEASUREMENT)
        config.setdefault("accuracy_decimals", 2)
    elif "soc" in name:
        config.setdefault("unit_of_measurement", UNIT_PERCENT)
        config.setdefault("device_class", DEVICE_CLASS_BATTERY)
        config.setdefault("state_class", STATE_CLASS_MEASUREMENT)
        config.setdefault("accuracy_decimals", 1)
    elif "remaining_capacity" in name or "capacity" in name:
        config.setdefault("unit_of_measurement", "Ah")
        config.setdefault("state_class", STATE_CLASS_MEASUREMENT)
        config.setdefault("icon", "mdi:battery-charging-100")
        config.setdefault("accuracy_decimals", 2)

    sens = await sensor.new_sensor(config)
    cg.add(hub.register_sensor(sens))
