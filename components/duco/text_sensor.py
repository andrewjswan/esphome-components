import esphome.codegen as cg
from esphome.components import text_sensor
import esphome.config_validation as cv
from esphome.const import (
    CONF_ID,
    ENTITY_CATEGORY_DIAGNOSTIC,
)

CONF_CORES_STATUS = "cores_status"
CONF_POOL = "pool"

ICON_CPU = "mdi:cpu-64-bit"
ICON_WEB = "mdi:web"

from . import DucoComponent

DEPENDENCIES = ["duco"]


CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.use_id(DucoComponent),
        cv.Optional(CONF_POOL): text_sensor.text_sensor_schema(
            icon = ICON_WEB,
        ),
        cv.Optional(CONF_CORES_STATUS): text_sensor.text_sensor_schema(
            icon = ICON_CPU,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
    }
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_ID])

    if CONF_POOL in config:
        sens = await  text_sensor.new_text_sensor(config[CONF_POOL])
        cg.add(parent.set_pool(sens))

    if CONF_CORES_STATUS in config:
        sens = await  text_sensor.new_text_sensor(config[CONF_CORES_STATUS])
        cg.add(parent.set_cores_status(sens))
