"""Music Leds component for ESPHome."""

import logging

import esphome.codegen as cg
import esphome.config_validation as cv
import esphome.final_validate as fv
from esphome.components.i2s_audio import microphone
from esphome.components.light.effects import register_addressable_effect
from esphome.components.light.types import AddressableLightEffect
from esphome.const import (
    CONF_BITS_PER_SAMPLE,
    CONF_ID,
    CONF_MODE,
    CONF_NAME,
    CONF_SAMPLE_RATE,
)

from .const import (
    CONF_INPUT_FILTER,
    CONF_MIC_ID,
    CONF_MUSIC_LEDS_ID,
    CONF_TASK_CORE,
    MODE_GRAVICENTRIC,
    MUSIC_LEDS_EFFECTS,
)

_LOGGER = logging.getLogger(__name__)

CODEOWNERS = ["@andrewjswan"]

DEPENDENCIES = ["light"]

AUTO_LOAD = ["music_leds", "fastled_helper"]

logging.info("Load Music Leds component https://github.com/andrewjswan/esphome-components")

music_leds_ns = cg.esphome_ns.namespace("music_leds")
MUSIC_LEDS = music_leds_ns.class_("MusicLeds", cg.Component)
MUSIC_LEDS_EFECT = music_leds_ns.class_("MusicLedsLightEffect", AddressableLightEffect)

MUSIC_LEDS_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_ID): cv.declare_id(MUSIC_LEDS),
        cv.Required(CONF_MIC_ID): cv.use_id(microphone),
        cv.Optional(CONF_INPUT_FILTER, default="3"): cv.templatable(cv.int_range(min=0, max=5)),
        cv.Optional(CONF_BITS_PER_SAMPLE): cv.float_,
        cv.Optional(CONF_SAMPLE_RATE): cv.int_,
        cv.Optional(CONF_TASK_CORE, default=1): cv.int_range(0, 1),
    },
)

CONFIG_SCHEMA = cv.All(MUSIC_LEDS_SCHEMA)


def _final_validate(config):  # noqa: ANN001 ANN202
    full_config = fv.full_config.get()
    mic_path = full_config.get_path_for_id(config[CONF_MIC_ID])[:-1]
    mic_conf = full_config.get_config_for_path(mic_path)
    bits = mic_conf.get(CONF_BITS_PER_SAMPLE)
    rate = mic_conf.get(CONF_SAMPLE_RATE)

    if bits not in [16, 32]:
        msg = "Music Leds support only 16 or 32 Bits Per Sample"
        raise cv.Invalid(msg)

    path = full_config.get_path_for_id(config[CONF_ID])[:-1]
    this_config = full_config.get_config_for_path(path)
    this_config[CONF_BITS_PER_SAMPLE] = bits
    this_config[CONF_SAMPLE_RATE] = rate


FINAL_VALIDATE_SCHEMA = _final_validate


async def to_code(config) -> None:  # noqa: ANN001
    """Code generation entry point."""
    var = cg.new_Pvariable(config[CONF_ID])

    cg.add_library("kosme/arduinoFFT", None)

    cg.add_build_flag("-Wno-narrowing")

    mic = await cg.get_variable(config[CONF_MIC_ID])
    cg.add(var.set_microphone(mic))
    cg.add(var.set_task_core(config[CONF_TASK_CORE]))

    cg.add_define("BITS_PER_SAMPLE", int(config[CONF_BITS_PER_SAMPLE]))
    cg.add_define("SAMPLE_RATE", config[CONF_SAMPLE_RATE])
    cg.add_define("INPUT_FILTER", config[CONF_INPUT_FILTER])

    await cg.register_component(var, config)


@register_addressable_effect(
    "music_leds_effect",
    MUSIC_LEDS_EFECT,
    "Music Leds",
    {
        cv.GenerateID(CONF_MUSIC_LEDS_ID): cv.use_id(MUSIC_LEDS),
        cv.Optional(CONF_MODE, default=MODE_GRAVICENTRIC): cv.one_of(
            *MUSIC_LEDS_EFFECTS,
            upper=True,
        ),
    },
)
async def music_leds_light_effect_to_code(config, effect_id) -> AddressableLightEffect:  # noqa: ANN001
    """Effect registration entry point."""
    parent = await cg.get_variable(config[CONF_MUSIC_LEDS_ID])

    effect = cg.new_Pvariable(effect_id, config[CONF_NAME])

    cg.add(effect.set_mode(MUSIC_LEDS_EFFECTS.index(config[CONF_MODE])))
    cg.add_define("DEF_" + config[CONF_MODE])
    cg.add(effect.set_music_leds(parent))
    return effect
