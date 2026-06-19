import esphome.codegen as cg
from esphome.components import binary_sensor
import esphome.config_validation as cv
from esphome.const import (
    CONF_ID,
    CONF_STATUS,
    DEVICE_CLASS_CONNECTIVITY,
)

from . import DucoComponent

DEPENDENCIES = ["duco"]


CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.use_id(DucoComponent),
        cv.Optional(CONF_STATUS): sensor.binary_sensor_schema(
            device_class=DEVICE_CLASS_CONNECTIVITY,
        ),
    }
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_ID])

    if CONF_STATUS in config:
        sens = await  binary_sensor.new_binary_sensor(config[CONF_STATUS])
        cg.add(parent.set_status(sens))
