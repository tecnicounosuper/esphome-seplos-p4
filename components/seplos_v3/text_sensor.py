import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import text_sensor
from . import seplos_v3_ns, SeplosV3

CONF_SEPLOS_V3_ID = "seplos_v3_id"
CONF_SYSTEM_STATUS = "system_status"
CONF_FET_STATUS = "fet_status"

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(CONF_SEPLOS_V3_ID): cv.use_id(SeplosV3),
    cv.Optional(CONF_SYSTEM_STATUS): text_sensor.text_sensor_schema(
        icon="mdi:info-outline"
    ),
    cv.Optional(CONF_FET_STATUS): text_sensor.text_sensor_schema(
        icon="mdi:gate"
    ),
})

async def to_code(config):
    hub = await cg.get_variable(config[CONF_SEPLOS_V3_ID])
    
    if CONF_SYSTEM_STATUS in config:
        sens = await text_sensor.new_text_sensor(config[CONF_SYSTEM_STATUS])
        cg.add(hub.set_system_status_text_sensor(sens))
        
    if CONF_FET_STATUS in config:
        sens = await text_sensor.new_text_sensor(config[CONF_FET_STATUS])
        cg.add(hub.set_fet_status_text_sensor(sens))
