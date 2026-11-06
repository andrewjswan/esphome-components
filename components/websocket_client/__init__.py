"""Websocket Client component for ESPHome."""

from typing import Any

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import automation
from esphome.components import esp32
from esphome.const import CONF_ID, CONF_PORT, CONF_URL, __version__
from esphome.cpp_generator import MockObj

DOMAIN = "websocket_client"

CODEOWNERS = ["@andrewjswan"]

DEPENDENCIES = ["network"]
AUTO_LOAD = ["json"]

websocket_client_ns = cg.esphome_ns.namespace("websocket_client")
WebSocketClient = websocket_client_ns.class_("WebSocketClient", cg.Component)

# Triggers
ConnectTrigger = websocket_client_ns.class_("WebSocketConnectTrigger", automation.Trigger)
DisconnectTrigger = websocket_client_ns.class_("WebSocketDisconnectTrigger", automation.Trigger)
ReceiveTrigger = websocket_client_ns.class_("WebSocketReceiveTrigger", automation.Trigger)

# Actions
SendAction = websocket_client_ns.class_("WebSocketSendAction", automation.Action)
EnableAction = websocket_client_ns.class_("WebSocketEnableAction", automation.Action)
DisableAction = websocket_client_ns.class_("WebSocketDisableAction", automation.Action)

CONF_ON_CONNECT = "on_connect"
CONF_ON_DISCONNECT = "on_disconnect"
CONF_ON_RECEIVE = "on_receive"

CONF_PING_INTERVAL = "ping_interval"
CONF_BUFFER_SIZE = "buffer_size"
CONF_MIN_RECONNECT_DELAY = "min_reconnect_delay"
CONF_MAX_RECONNECT_DELAY = "max_reconnect_delay"
CONF_STABLE_SESSION_DURATION = "stable_session_duration"
CONF_JITTER_PERCENT = "jitter_percent"
CONF_SSL = "ssl"
CONF_ENABLE_ON_BOOT = "enable_on_boot"
CONF_USER_AGENT = "user_agent"


def validate_url_scheme(config) -> dict[str, Any]:
    """Validate that the URL scheme is either ws:// or wss:// and set defaults."""
    url = config[CONF_URL]

    if url.startswith("wss://"):
        if CONF_SSL not in config:
            config[CONF_SSL] = True
        if CONF_PORT not in config:
            config[CONF_PORT] = 443

    elif url.startswith("ws://"):
        if CONF_SSL not in config:
            config[CONF_SSL] = False
        if CONF_PORT not in config:
            config[CONF_PORT] = 80
    else:
        msg = "URL must start with 'ws://' or 'wss://'"
        raise cv.Invalid(msg)

    return config


BASE_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(WebSocketClient),
        cv.Required(CONF_URL): cv.url,
        cv.Optional(CONF_PORT): cv.port,
        cv.Optional(CONF_SSL): cv.boolean,
        cv.Optional(
            CONF_USER_AGENT, f"ESPHome/WSC ({__version__})"
        ): cv.string,
        cv.Optional(CONF_PING_INTERVAL, default="20s"): cv.positive_time_period_seconds,
        cv.Optional(CONF_BUFFER_SIZE, default=1024): cv.positive_not_null_int,
        cv.Optional(CONF_MIN_RECONNECT_DELAY, default="5s"): cv.positive_time_period_seconds,
        cv.Optional(CONF_MAX_RECONNECT_DELAY, default="130s"): cv.positive_time_period_seconds,
        cv.Optional(CONF_STABLE_SESSION_DURATION, default="60s"): cv.positive_time_period_seconds,
        cv.Optional(CONF_JITTER_PERCENT, default=20): cv.int_range(min=0, max=100),
        cv.Optional(CONF_ENABLE_ON_BOOT, default=True): cv.boolean,
        cv.Optional(CONF_ON_CONNECT): automation.validate_automation(
            {
                cv.GenerateID(CONF_ID): cv.declare_id(ConnectTrigger),
            },
        ),
        cv.Optional(CONF_ON_DISCONNECT): automation.validate_automation(
            {
                cv.GenerateID(CONF_ID): cv.declare_id(DisconnectTrigger),
            },
        ),
        cv.Optional(CONF_ON_RECEIVE): automation.validate_automation(
            {
                cv.GenerateID(CONF_ID): cv.declare_id(ReceiveTrigger),
            },
        ),
    },
).extend(cv.COMPONENT_SCHEMA)

CONFIG_SCHEMA = cv.All(BASE_SCHEMA, validate_url_scheme)


async def to_code(config) -> None:  # noqa: C901
    """Code generation entry point."""
    esp32.add_idf_component(name="espressif/esp_websocket_client", ref="1.8.0")

    if hasattr(esp32, "request_tls"):
        esp32.request_tls()
    elif hasattr(esp32, "include_builtin_idf_component"):
        esp32.include_builtin_idf_component("esp-tls")

    if hasattr(esp32, "add_idf_sdkconfig_option"):
        esp32.add_idf_sdkconfig_option("CONFIG_MBEDTLS_TLS_ENABLED", value=True)

    if config[CONF_SSL]:
        cg.add_define("USE_WS_BRIDGE_CERT_BUNDLE")
        if hasattr(esp32, "require_certificate_bundle"):
            esp32.require_certificate_bundle()
        elif hasattr(esp32, "require_full_certificate_bundle"):
            esp32.require_full_certificate_bundle()

    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    cg.add(var.set_url(config[CONF_URL]))
    cg.add(var.set_port(config[CONF_PORT]))
    cg.add(var.set_ssl(config[CONF_SSL]))
    cg.add(var.set_ping_interval(config[CONF_PING_INTERVAL]))
    cg.add(var.set_buffer_size(config[CONF_BUFFER_SIZE]))

    if CONF_USER_AGENT in config:
        cg.add(var.set_user_agent(config[CONF_USER_AGENT]))

    cg.add(var.set_min_reconnect_delay(config[CONF_MIN_RECONNECT_DELAY]))
    cg.add(var.set_max_reconnect_delay(config[CONF_MAX_RECONNECT_DELAY]))
    cg.add(var.set_stable_session_duration(config[CONF_STABLE_SESSION_DURATION]))
    cg.add(var.set_jitter_percent(config[CONF_JITTER_PERCENT]))

    cg.add(var.set_enable_on_boot(config[CONF_ENABLE_ON_BOOT]))

    if CONF_ON_CONNECT in config:
        for conf in config[CONF_ON_CONNECT]:
            trigger = cg.new_Pvariable(conf[CONF_ID], var)
            await automation.build_automation(trigger, [], conf)

    if CONF_ON_DISCONNECT in config:
        for conf in config[CONF_ON_DISCONNECT]:
            trigger = cg.new_Pvariable(conf[CONF_ID], var)
            await automation.build_automation(trigger, [(cg.int_, "reason")], conf)

    if CONF_ON_RECEIVE in config:
        for conf in config[CONF_ON_RECEIVE]:
            trigger = cg.new_Pvariable(conf[CONF_ID], var)
            await automation.build_automation(trigger, [(cg.std_string, "x")], conf)


@automation.register_action(
    "websocket_client.send",
    SendAction,
    cv.Schema(
        {
            cv.Required(CONF_ID): cv.use_id(WebSocketClient),
            cv.Required("message"): cv.templatable(cv.string),
        },
    ),
    synchronous=True,
)
async def websocket_client_send_to_code(config, action_id, template_arg, args) -> MockObj:
    """Code generation for send action."""
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    template_ = await cg.templatable(config["message"], args, cg.std_string)
    cg.add(var.set_message(template_))
    return var


@automation.register_action(
    "websocket_client.enable",
    EnableAction,
    cv.Schema({
        cv.Required(CONF_ID): cv.use_id(WebSocketClient),
    }),
    synchronous=True,
)
async def websocket_client_enable_to_code(config, action_id, template_arg, args):
    """Code generation for enable action."""
    parent = await cg.get_variable(config[CONF_ID])
    return cg.new_Pvariable(action_id, template_arg, parent)


@automation.register_action(
    "websocket_client.disable",
    DisableAction,
    cv.Schema({
        cv.Required(CONF_ID): cv.use_id(WebSocketClient),
    }),
    synchronous=True,
)
async def websocket_client_disable_to_code(config, action_id, template_arg, args):
    """Code generation for disable action."""
    parent = await cg.get_variable(config[CONF_ID])
    return cg.new_Pvariable(action_id, template_arg, parent)
