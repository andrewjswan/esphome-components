"""Duino Coin Miner component for ESPHome."""

import logging

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import ota
from esphome.const import CONF_ID, CONF_KEY, CONF_NAME, CONF_USERNAME

_LOGGER = logging.getLogger(__name__)

CODEOWNERS = ["@andrewjswan"]

DEPENDENCIES = ["wifi", "http_request"]

AUTO_LOAD = ["duco"]

logging.info("Load Duino Coin Miner component https://github.com/andrewjswan/esphome-components")
logging.info("If you like the Duino Coin Miner component, you can support it with a star ⭐ on GitHub.")

duco_ns = cg.esphome_ns.namespace("duco")
DUCO_ = duco_ns.class_("Duco", cg.Component)

DUCO_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_ID): cv.declare_id(DUCO_),
        cv.Required(CONF_USERNAME): cv.string,
        cv.Required(CONF_KEY): cv.string,
        cv.Optional(CONF_NAME, default="esphomeminer"): cv.string,
    },
)

CONFIG_SCHEMA = cv.All(DUCO_SCHEMA)


async def to_code(config) -> None:
    """Code generation entry point."""
    var = cg.new_Pvariable(config[CONF_ID])

    ota.request_ota_state_listeners()

    cg.add_define("DUCO_USER", config[CONF_USERNAME])
    cg.add_define("DUCO_KEY", config[CONF_KEY])
    cg.add_define("DUCO_WORKER", config[CONF_NAME])

    await cg.register_component(var, config)
