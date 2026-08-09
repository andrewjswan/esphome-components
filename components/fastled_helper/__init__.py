"""Fastled Helper component for ESPHome."""

import logging

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID, CONF_LIGHT
from esphome.core import CORE

CONF_PALETTES = "palettes"
CONF_MUSIC_LEDS = "music_leds"

_LOGGER = logging.getLogger(__name__)

CODEOWNERS = ["@andrewjswan"]

AUTO_LOAD = ["fastled_helper"]

logging.info("Load Fastled Helper component https://github.com/andrewjswan/esphome-components")
logging.info("If you like the Fastled Helper component, you can support it with a star ⭐ on GitHub.")

fastled_helper_ns = cg.esphome_ns.namespace("fastled_helper")
FASTLED_HELPER_ = fastled_helper_ns.class_("FastledHelper", cg.Component)

FASTLED_HELPER_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_ID): cv.declare_id(FASTLED_HELPER_),
        cv.Optional(CONF_PALETTES, default=True): cv.templatable(cv.boolean),
        cv.Optional(CONF_MUSIC_LEDS, default=False): cv.templatable(cv.boolean),
    },
)

CONFIG_SCHEMA = cv.All(FASTLED_HELPER_SCHEMA)


async def to_code(config) -> None:
    """Code generation entry point."""
    var = cg.new_Pvariable(config[CONF_ID])

    if config[CONF_PALETTES]:
        cg.add_define("USE_PALETTES")
        if config[CONF_MUSIC_LEDS]:
            cg.add_define("USE_MUSIC_LEDS")

    gamma_value = 2.8
    if CONF_LIGHT in CORE.config:
        first_light = CORE.config[CONF_LIGHT][0]
        gamma_value = first_light.get("gamma_correct", 2.8)
        logging.info("Gamma value: %s", gamma_value)

    cg.add_define("GAMMA_CORRECT", cg.RawExpression(f"{float(gamma_value):.2f}f"))

    await cg.register_component(var, config)
