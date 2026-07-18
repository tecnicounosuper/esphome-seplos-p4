import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor

from . import SeplosParser, CONF_SEPLOS_PARSER_ID, CONF_BMS_INDEX

DEPENDENCIES = ["seplos_parser"]

BALANCING_KEYS = [f"balancing_{i}" for i in range(1, 17)]

_schema_dict = {
    cv.GenerateID(CONF_SEPLOS_PARSER_ID): cv.use_id(SeplosParser),
    cv.Required(CONF_BMS_INDEX): cv.int_range(min=0, max=15),
    cv.Optional("chg_mos"): binary_sensor.binary_sensor_schema(device_class="power"),
    cv.Optional("dischg_mos"): binary_sensor.binary_sensor_schema(device_class="power"),
}

for _key in BALANCING_KEYS:
    _schema_dict[cv.Optional(_key)] = binary_sensor.binary_sensor_schema()

CONFIG_SCHEMA = cv.Schema(_schema_dict)

ALL_KEYS = ["chg_mos", "dischg_mos"] + BALANCING_KEYS


async def to_code(config):
    hub = await cg.get_variable(config[CONF_SEPLOS_PARSER_ID])
    bms_index = config[CONF_BMS_INDEX]

    for key in ALL_KEYS:
        if key in config:
            sens = await binary_sensor.new_binary_sensor(config[key])
            cg.add(hub.set_binary_sensor(bms_index, key, sens))
