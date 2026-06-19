import esphome.codegen as cg
from esphome.components import sensor
import esphome.config_validation as cv
from esphome.const import (
    CONF_ID,
    CONF_STATE,
    DEVICE_CLASS_DURATION,
    ENTITY_CATEGORY_DIAGNOSTIC,
    STATE_CLASS_MEASUREMENT,
    UNIT_PERCENT,
    UNIT_MILLISECOND,
)

CONF_HASHRATE = "hashrate"
CONF_ACCEPTED_SHARES = "accepted_shares"
CONF_TOTAL_SHARES = "total_shares"
CONF_DIFFICULTY = "difficulty"
CONF_SHARE_RATE = "share_rate"
CONF_ACCEPTED_RATE = "accept_rate"
CONF_PING = "ping"

UNIT_KILO_HASH_PER_SECOND = "KH/s"

ICON_SPEEDOMETER = "mdi:speedometer"
ICON_CHART_ARC = "mdi:chart-arc"
ICON_TROPHY_AWARD = "mdi:trophy-award"
ICON_POUND_BOX = "mdi:pound-box-outline"
ICON_VECTOR_DIFFERENCE = "mdi:vector-difference"
ICON_PUZZLE = "mdi:puzzle"

from . import DucoComponent

DEPENDENCIES = ["duco"]


CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.use_id(DucoComponent),
        cv.Optional(CONF_HASHRATE): sensor.sensor_schema(
            icon = ICON_SPEEDOMETER,
            unit_of_measurement=UNIT_KILO_HASH_PER_SECOND,
            accuracy_decimals=2,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_ACCEPTED_SHARES): sensor.sensor_schema(
            icon = ICON_TROPHY_AWARD,
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_TOTAL_SHARES): sensor.sensor_schema(
            icon = ICON_POUND_BOX,
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_DIFFICULTY): sensor.sensor_schema(
            icon = ICON_PUZZLE,
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_SHARE_RATE): sensor.sensor_schema(
            icon = ICON_CHART_ARC,
            accuracy_decimals=2,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_ACCEPTED_RATE): sensor.sensor_schema(
            icon = ICON_VECTOR_DIFFERENCE,
            accuracy_decimals=1,
            unit_of_measurement=UNIT_PERCENT,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_PING): sensor.sensor_schema(
            accuracy_decimals=0,
            unit_of_measurement=UNIT_MILLISECOND,
            device_class=DEVICE_CLASS_DURATION,
            state_class=STATE_CLASS_MEASUREMENT,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
    }
)

async def to_code(config):
    parent = await cg.get_variable(config[CONF_ID])

    if CONF_HASHRATE in config:
        sens = await sensor.new_sensor(config[CONF_HASHRATE])
        cg.add(parent.set_hashrate(sens))

    if CONF_ACCEPTED_SHARES in config:
        sens = await sensor.new_sensor(config[CONF_ACCEPTED_SHARES])
        cg.add(parent.set_accepted_shares(sens))

    if CONF_TOTAL_SHARES in config:
        sens = await sensor.new_sensor(config[CONF_TOTAL_SHARES])
        cg.add(parent.set_total_shares(sens))

    if CONF_DIFFICULTY in config:
        sens = await sensor.new_sensor(config[CONF_DIFFICULTY])
        cg.add(parent.set_difficulty(sens))

    if CONF_SHARE_RATE in config:
        sens = await sensor.new_sensor(config[CONF_SHARE_RATE])
        cg.add(parent.set_share_rate(sens))

    if CONF_ACCEPTED_RATE in config:
        sens = await sensor.new_sensor(config[CONF_ACCEPTED_RATE])
        cg.add(parent.set_accept_rate(sens))

    if CONF_PING in config:
        sens = await sensor.new_sensor(config[CONF_PING])
        cg.add(parent.set_ping(sens))
