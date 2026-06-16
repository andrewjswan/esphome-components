"""Duino Coin Miner component for ESPHome."""

import logging

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import ota, socket
from esphome.const import CONF_ID, CONF_KEY, CONF_NAME, CONF_USERNAME
from esphome.core import CORE
from esphome.types import ConfigType

_LOGGER = logging.getLogger(__name__)

CODEOWNERS = ["@andrewjswan"]

DEPENDENCIES = ["wifi"]

AUTO_LOAD = ["duco", "socket"]

logging.info("Load Duino Coin Miner (Duco) component https://github.com/andrewjswan/esphome-components")
logging.info("If you like the Duino Coin Miner (Duco) component, you can support it with a star ⭐ on GitHub.")

duco_ns = cg.esphome_ns.namespace("duco")
DUCO_ = duco_ns.class_("Duco", cg.Component)


def _consume_sockets(config: ConfigType) -> ConfigType:
    """Register socket need for Duco component."""
    # Duco needs 1 socket per Miner
    if CORE.is_esp8266:
        socket.consume_sockets(1, "duco")(config)
    elif CORE.is_esp32:
        from esphome.components.esp32 import (
            VARIANT_ESP32C2,
            VARIANT_ESP32C3,
            VARIANT_ESP32C5,
            VARIANT_ESP32C6,
            VARIANT_ESP32C61,
            VARIANT_ESP32H2,
            # VARIANT_ESP32H21,
            VARIANT_ESP32S2,
            get_esp32_variant,
        )

        single_core_variants = (
            VARIANT_ESP32C2, VARIANT_ESP32C3, VARIANT_ESP32C5, 
            VARIANT_ESP32C6, VARIANT_ESP32C61, VARIANT_ESP32H2, 
            #VARIANT_ESP32H21, 
            VARIANT_ESP32S2,
        )
        if get_esp32_variant() in single_core_variants:
            socket.consume_sockets(1, "duco")(config)
        else:
            socket.consume_sockets(2, "duco")(config)
    return config


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.Required(CONF_ID): cv.declare_id(DUCO_),
            cv.Required(CONF_USERNAME): cv.string,
            cv.Required(CONF_KEY): cv.string,
            cv.Optional(CONF_NAME, default="Auto"): cv.string,
        },
    ).extend(cv.COMPONENT_SCHEMA),
    _consume_sockets,
    cv.only_on_esp32,
)

async def to_code(config) -> None:
    """Code generation entry point."""
    var = cg.new_Pvariable(config[CONF_ID])

    ota.request_ota_state_listeners()

    cg.add_define("DUCO_USERNAME", config[CONF_USERNAME])
    cg.add_define("DUCO_KEY", config[CONF_KEY])
    cg.add_define("DUCO_WORKER", config[CONF_NAME])

    # if CORE.is_esp8266:
    #     cg.add_define("DUCO_START_DIFF", "ESP8266H")
    #     cg.add_define("DUCO_MINER_BANNER", "ESPHome ESP8266 Miner")
    # elif CORE.is_esp32:
    #     cg.add_define("DUCO_START_DIFF", "ESP32")
    #     cg.add_define("DUCO_MINER_BANNER", "ESPHome ESP32 Miner")

    await cg.register_component(var, config)
