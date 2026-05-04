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

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(CONF_SEPLOS_V3_ID): cv.use_id(SeplosComponent),
    cv.Required(CONF_ADDRESS): cv.int_range(min=0, max=15),
    cv.Required(CONF_TYPE): cv.one_of(*TYPES, lower=True),
}).extend(sensor.SENSOR_SCHEMA)

async def to_code(config):
    # Recupera l'hub principale (il componente seplos_v3)
    hub = await cg.get_variable(config[CONF_SEPLOS_V3_ID])
    
    # Crea l'oggetto sensore
    var = await sensor.new_sensor(config)
    
    # Chiama la funzione C++ per registrare il sensore nell'hub
    # Esempio: hub->register_sensor(address, type, sensor_object)
    cg.add(hub.register_sensor(config[CONF_ADDRESS], config[CONF_TYPE], var))
