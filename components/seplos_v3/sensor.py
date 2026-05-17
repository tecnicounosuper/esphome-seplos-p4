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
    cv.Optional(CONF_CURRENT): sensor.sensor_schema(
        unit_of_measurement=UNIT_AMPERE,
        accuracy_decimals=2,
        device_class=DEVICE_CLASS_CURRENT,
        state_class=STATE_CLASS_MEASUREMENT,
        icon="mdi:current-ac",
    ),
    cv.Optional(CONF_SOC): sensor.sensor_schema(
        unit_of_measurement=UNIT_PERCENT,
        accuracy_decimals=1,
        device_class=DEVICE_CLASS_BATTERY,
        state_class=STATE_CLASS_MEASUREMENT,
        icon="mdi:battery-charging-100",
    ),
})

# Generazione dinamica dei nodi per le 16 celle con icone personalizzate
for i in range(1, 17):
    CONFIG_SCHEMA = CONFIG_SCHEMA.extend({
        cv.Optional(f"cell_{i}"): sensor.sensor_schema(
            unit_of_measurement=UNIT_VOLT,
            accuracy_decimals=3,
            device_class=DEVICE_CLASS_VOLTAGE,
            state_class=STATE_CLASS_MEASUREMENT,
            icon="mdi:battery-outline",
        )
    })

# Generazione dinamica dei nodi per le 4 temperature con icone del termometro
for i in range(1, 5):
    CONFIG_SCHEMA = CONFIG_SCHEMA.extend({
        cv.Optional(f"cell_temp_{i}"): sensor.sensor_schema(
            unit_of_measurement=UNIT_CELSIUS,
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_TEMPERATURE,
            state_class=STATE_CLASS_MEASUREMENT,
            icon="mdi:thermometer",
        )
    })

async def to_code(config):
    hub = await cg.get_variable(config[CONF_SEPLOS_V3_ID])
    
    if CONF_PACK_VOLTAGE in config:
        sens = await sensor.new_sensor(config[CONF_PACK_VOLTAGE])
        cg.add(hub.set_pack_voltage_sensor(sens))
        
    if CONF_CURRENT in config:
        sens = await sensor.new_sensor(config[CURRENT])
        cg.add(hub.set_current_sensor(sens))
        
    if CONF_SOC in config:
        sens = await sensor.new_sensor(config[CONF_SOC])
        cg.add(hub.set_soc_sensor(sens))

    for i in range(1, 17):
        if f"cell_{i}" in config:
            sens = await sensor.new_sensor(config[f"cell_{i}"])
            cg.add(hub.set_cell_sensor(i - 1, sens))

    for i in range(1, 5):
        if f"cell_temp_{i}" in config:
            sens = await sensor.new_sensor(config[f"cell_temp_{i}"])
            cg.add(hub.set_cell_temp_sensor(i - 1, sens))
