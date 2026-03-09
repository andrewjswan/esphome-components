"""NerdMiner component for ESPHome."""

import logging

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import ota, socket
from esphome.const import CONF_ID
from esphome.core import CORE
from esphome.types import ConfigType

_LOGGER = logging.getLogger(__name__)

CODEOWNERS = ["@andrewjswan"]

DEPENDENCIES = ["wifi", "http_request"]

AUTO_LOAD = ["nerdminer", "socket"]

logging.info("Load NerdMiner component https://github.com/andrewjswan/esphome-components")
logging.info("If you like the NerdMiner component, you can support it with a star ⭐ on GitHub.")

nerdminer_ns = cg.esphome_ns.namespace("nerdminer")
NERDMINER_ = nerdminer_ns.class_("NerdMiner", cg.Component)

CONF_WALLETID = "walletid"
CONF_WORKER = "worker"
CONF_POOL = "pool"
CONF_POOL_PORT = "port"
CONF_POOL_PASS = "password"  # noqa: S105
CONF_DIFFICULTY = "difficulty"
DIFFICULTY_VALUES = [0.00001, 0.0001, 0.001, 0.0014, 0.01, 0.1, 1.0]


def _consume_sockets(config: ConfigType) -> ConfigType:
    """Register socket need for Nerminer component."""
    # Nerdminer needs 1 socket (Stratum)
    socket.consume_sockets(1, "nerdminer")(config)
    return config


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.Required(CONF_ID): cv.declare_id(NERDMINER_),
            cv.Required(CONF_WALLETID): cv.string,
            cv.Optional(CONF_WORKER, default="esphomeminer"): cv.string,
            cv.Optional(CONF_POOL, default="public-pool.io"): cv.string,
            cv.Optional(CONF_POOL_PORT, default=21496): cv.port,
            cv.Optional(CONF_POOL_PASS, default="x"): cv.string,
            cv.Optional(CONF_DIFFICULTY, default=0.0014): cv.one_of(*DIFFICULTY_VALUES, float=True),
        },
    ).extend(cv.COMPONENT_SCHEMA),
    _consume_sockets,
    cv.only_on_esp32,
)


async def to_code(config) -> None:
    """Code generation entry point."""
    var = cg.new_Pvariable(config[CONF_ID])

    cg.add_define("USE_NERDMINER")

    if CORE.is_esp32:
        from esphome.components.esp32 import VARIANT_ESP32, VARIANT_ESP32C3, get_esp32_variant
        
        variant = get_esp32_variant()
        if variant == VARIANT_ESP32:
            cg.add_define("HARDWARE_SHA256", 1)
        elif variant == VARIANT_ESP32C3:
            cg.add_define("USE_SOFTWARE_SHA", 1)
        else:
            cg.add_define("USE_HARDWARE_SHA", 1)

    ota.request_ota_state_listeners()

    cg.add(var.set_wallet_id(config[CONF_WALLETID]))
    cg.add(var.set_worker_name(config[CONF_WORKER]))
    cg.add(var.set_pool(config[CONF_POOL]))
    cg.add(var.set_pool_pass(config[CONF_POOL_PASS]))
    cg.add(var.set_pool_port(config[CONF_POOL_PORT]))

    if config[CONF_DIFFICULTY]:
        cg.add_define("DESIRED_DIFFICULTY", config[CONF_DIFFICULTY])

    await cg.register_component(var, config)
