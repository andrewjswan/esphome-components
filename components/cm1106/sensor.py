"""CM1106 Sensor component for ESPHome."""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import automation
from esphome.automation import maybe_simple_id
from esphome.components import sensor, uart
from esphome.const import (
    CONF_CO2,
    CONF_ID,
    DEVICE_CLASS_CARBON_DIOXIDE,
    ICON_MOLECULE_CO2,
    STATE_CLASS_MEASUREMENT,
    UNIT_PARTS_PER_MILLION,
)

DEPENDENCIES = ["uart"]
CODEOWNERS = ["@andrewjswan"]

CONF_AUTOMATIC_BASELINE_CALIBRATION = "automatic_baseline_calibration"

cm1106_ns = cg.esphome_ns.namespace("cm1106")
CM1106Component = cm1106_ns.class_("CM1106Component", cg.PollingComponent, uart.UARTDevice)
CM1106CalibrateZeroAction = cm1106_ns.class_(
    "CM1106CalibrateZeroAction",
    automation.Action,
)
CM1106ABCEnableAction = cm1106_ns.class_("CM1106ABCEnableAction", automation.Action)
CM1106ABCDisableAction = cm1106_ns.class_("CM1106ABCDisableAction", automation.Action)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(CM1106Component),
            cv.Required(CONF_CO2): sensor.sensor_schema(
                unit_of_measurement=UNIT_PARTS_PER_MILLION,
                icon=ICON_MOLECULE_CO2,
                accuracy_decimals=0,
                device_class=DEVICE_CLASS_CARBON_DIOXIDE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_AUTOMATIC_BASELINE_CALIBRATION): cv.boolean,
        },
    )
    .extend(cv.polling_component_schema("60s"))
    .extend(uart.UART_DEVICE_SCHEMA)
)


async def to_code(config) -> None:
    """Code generation entry point."""
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    if CONF_CO2 in config:
        sens = await sensor.new_sensor(config[CONF_CO2])
        cg.add(var.set_co2_sensor(sens))


CALIBRATION_ACTION_SCHEMA = maybe_simple_id(
    {
        cv.Required(CONF_ID): cv.use_id(CM1106Component),
    },
)


@automation.register_action(
    "cm1106.calibrate_zero",
    CM1106CalibrateZeroAction,
    CALIBRATION_ACTION_SCHEMA,
)
async def cm1106_calibration_to_code(config, action_id, template_arg, args) -> None:  # noqa: ARG001
    """Service code generation entry point."""
    paren = await cg.get_variable(config[CONF_ID])
    return cg.new_Pvariable(action_id, template_arg, paren)
