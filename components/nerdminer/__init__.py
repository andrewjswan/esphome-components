"""NerdMiner component for ESPHome."""

import logging

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import ota
from esphome.const import CONF_ID

_LOGGER = logging.getLogger(__name__)

CODEOWNERS = ["@andrewjswan"]

DEPENDENCIES = ["wifi", "http_request"]

AUTO_LOAD = ["nerdminer"]

logging.info("Load NerdMiner component https://github.com/andrewjswan/esphome-components")

nerdminer_ns = cg.esphome_ns.namespace("nerdminer")
NERDMINER_ = nerdminer_ns.class_("NerdMiner", cg.Component)

CONF_WALLETID = "walletid"
CONF_WORKER = "worker"
CONF_POOL = "pool"
CONF_POOL_PORT = "port"
CONF_POOL_PASS = "password"  # noqa: S105

NERDMINER_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_ID): cv.declare_id(NERDMINER_),
        cv.Required(CONF_WALLETID): cv.string,
        cv.Optional(CONF_WORKER, default="esphomeminer"): cv.string,
        cv.Optional(CONF_POOL, default="public-pool.io"): cv.string,
        cv.Optional(CONF_POOL_PORT, default=21496): cv.port,
        cv.Optional(CONF_POOL_PASS, default="x"): cv.string,
    },
)

CONFIG_SCHEMA = cv.All(NERDMINER_SCHEMA)


async def to_code(config) -> None:
    """Code generation entry point."""
    var = cg.new_Pvariable(config[CONF_ID])

    ota.request_ota_state_listeners()

    cg.add_define("NERDMINER_WALLETID", config[CONF_WALLETID])
    cg.add_define("NERDMINER_WORKER", config[CONF_WORKER])
    cg.add_define("NERDMINER_POOL_WORKER", config[CONF_WALLETID] + "." + config[CONF_WORKER])
    cg.add_define("NERDMINER_POOL", config[CONF_POOL])
    cg.add_define("NERDMINER_POOL_PORT", config[CONF_POOL_PORT])
    cg.add_define("NERDMINER_POOL_PASS", config[CONF_POOL_PASS])

    await cg.register_component(var, config)
