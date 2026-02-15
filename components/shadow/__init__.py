"""Shadow component for ESPHome."""

import logging

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import ota, script
from esphome.const import CONF_ID, CONF_INTERVAL, CONF_STARTUP_DELAY

_LOGGER = logging.getLogger(__name__)

CODEOWNERS = ["@andrewjswan"]

DEPENDENCIES = ["script"]

AUTO_LOAD = ["shadow"]

MULTI_CONF = True

logging.info("Load Shadow component https://github.com/andrewjswan/esphome-components")

shadow_ns = cg.esphome_ns.namespace("shadow")
SHADOW_ = shadow_ns.class_("Shadow", cg.Component)

CONF_SCRIPT_ID = "script_id"

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(SHADOW_),
            cv.Optional(
                CONF_STARTUP_DELAY,
                default="0s",
            ): cv.positive_time_period_seconds,
            cv.Optional(CONF_INTERVAL, default="60s"): cv.positive_time_period_seconds,
            cv.Required(CONF_SCRIPT_ID): cv.use_id(script),
        },
    ).extend(cv.COMPONENT_SCHEMA),
)


async def to_code(config) -> None:
    """Code generation entry point."""
    var = cg.new_Pvariable(config[CONF_ID])

    ota.request_ota_state_listeners()

    shadow_script = await cg.get_variable(config[CONF_SCRIPT_ID])
    cg.add(var.set_script(shadow_script))

    cg.add(var.set_startup_delay(config[CONF_STARTUP_DELAY]))
    cg.add(var.set_shadow_interval(config[CONF_INTERVAL]))

    await cg.register_component(var, config)
