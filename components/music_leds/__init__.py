"""Music Leds component for ESPHome."""

import logging

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import automation, core
from esphome.components import microphone, ota
from esphome.components.light.effects import register_addressable_effect
from esphome.components.light.types import AddressableLightEffect
from esphome.const import (
    CONF_ID,
    CONF_MICROPHONE,
    CONF_MODE,
    CONF_NAME,
    CONF_TRIGGER_ID,
)

from .const import (
CONF_BEAT_SENSITIVITY,
CONF_MUSIC_LEDS_ID,
CONF_NOISE_GATE_FLOOR,
CONF_ON_SOUND_LOOP,
CONF_PRE_AMP_GAIN,
CONF_SCALING_MODE,
CONF_TASK_CORE,
CONF_TASK_PRIORITY,
)


_LOGGER = logging.getLogger(__name__)

CODEOWNERS = ["@andrewjswan"]

DEPENDENCIES = ["light", "microphone"]

AUTO_LOAD = ["fastled_helper"]

logging.info("Load Music Leds component https://github.com/andrewjswan/esphome-components")
logging.info("If you like the Music Leds component, you can support it with a star ⭐ on GitHub.")

music_leds_ns = cg.esphome_ns.namespace("music_leds")
MUSIC_LEDS = music_leds_ns.class_("MusicLeds", cg.Component)
MUSIC_LEDS_EFECT = music_leds_ns.class_("MusicLedsLightEffect", AddressableLightEffect)

FFTScalingMode = music_leds_ns.enum("FFTScalingMode", is_class=True)
SCALING_MODES = {
    "LINEAR": FFTScalingMode.LINEAR,
    "LOGARITHMIC": FFTScalingMode.LOGARITHMIC,
    "SQUARE_ROOT": FFTScalingMode.SQUARE_ROOT,
}

SoundLoopTrigger = music_leds_ns.class_(
    "MusicLedsSoundLoopTrigger",
    automation.Trigger.template(cg.std_string),
)

PlayMode = music_leds_ns.enum("PLAYMODE")
MUSIC_LEDS_EFFECTS = {
    "GRAV": PlayMode.MODE_GRAV,
    "GRAVICENTER": PlayMode.MODE_GRAVICENTER,
    "GRAVICENTRIC": PlayMode.MODE_GRAVICENTRIC,
    "GRAVIMETER": PlayMode.MODE_GRAVIMETER,
    "PIXELS": PlayMode.MODE_PIXELS,
    "JUNGLES": PlayMode.MODE_JUNGLES,
    "MIDNOISE": PlayMode.MODE_MIDNOISE,
    "RIPPLEPEAK": PlayMode.MODE_RIPPLEPEAK,
    "MATRIPIX": PlayMode.MODE_MATRIPIX,
    "NOISEFIRE": PlayMode.MODE_NOISEFIRE,
    "PIXELWAVE": PlayMode.MODE_PIXELWAVE,
    "PLASMOID": PlayMode.MODE_PLASMOID,
    "PUDDLEPEAK": PlayMode.MODE_PUDDLEPEAK,
    "PUDDLES": PlayMode.MODE_PUDDLES,
    "DJLIGHT": PlayMode.MODE_DJLIGHT,
    "WATERFALL": PlayMode.MODE_WATERFALL,
}

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_ID): cv.declare_id(MUSIC_LEDS),
        cv.GenerateID(CONF_MICROPHONE): cv.use_id(microphone.Microphone),
        cv.Optional(CONF_TASK_CORE, default=1): cv.int_range(0, 1),
        cv.Optional(CONF_TASK_PRIORITY, default=10): cv.int_range(1, 24),
        cv.Optional(CONF_SCALING_MODE, default="SQUARE_ROOT"): cv.enum(SCALING_MODES, upper=True),
        cv.Optional(CONF_BEAT_SENSITIVITY, default=65): cv.int_range(1, 100),
        cv.Optional(CONF_NOISE_GATE_FLOOR, default=0.10): cv.float_range(0.001, 0.5),
        cv.Optional(CONF_PRE_AMP_GAIN, default=1.0): cv.float_range(1.0, 20.0),
        cv.Optional(CONF_ON_SOUND_LOOP): automation.validate_automation(
            {
                cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(SoundLoopTrigger),
            },
        ),
    }
)


async def to_code(config) -> None:
    """Code generation entry point."""
    var = cg.new_Pvariable(config[CONF_ID])

    ota.request_ota_state_listeners()

    cg.add_library("kosme/arduinoFFT", None)
    cg.add_build_flag("-Wno-narrowing")

    cg.add_define("USE_MUSIC_LEDS")

    mic = await cg.get_variable(config[CONF_MICROPHONE])
    cg.add(var.set_microphone(mic))

    cg.add(var.set_scaling_mode(config[CONF_SCALING_MODE]))
    cg.add(var.set_beat_sensitivity(int(config[CONF_BEAT_SENSITIVITY])))
    cg.add(var.set_noise_gate_floor(float(config[CONF_NOISE_GATE_FLOOR])))
    cg.add(var.set_pre_amp_gain(float(config[CONF_PRE_AMP_GAIN])))

    # FFTTASK_CORE 0 standard: Core #0
    # FFTTASK_CORE 1 standard: Core #1
    cg.add_define("FFTTASK_CORE", config[CONF_TASK_CORE])

    # FFTTASK_PRIORITY 1 standard: looptask prio
    # FFTTASK_PRIORITY 2 above looptask, below asyc_tcp
    # FFTTASK_PRIORITY 4 above asyc_tcp
    cg.add_define("FFTTASK_PRIORITY", config[CONF_TASK_PRIORITY])

    if config.get(CONF_ON_SOUND_LOOP, []):
        cg.add_define("MUSIC_LEDS_TRIGGERS")
        logging.info("[X] On Sound Loop trigger")
        for conf in config.get(CONF_ON_SOUND_LOOP, []):
            trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
            await automation.build_automation(
                trigger,
                [
                    (cg.float_, "volume_smth"),
                    (cg.int16, "volume_raw"),
                    (cg.float_, "fft_major_peak"),
                    (cg.bool_, "sample_peak"),
                ],
                conf,
            )

    await cg.register_component(var, config)


@register_addressable_effect(
    "music_leds_effect",
    MUSIC_LEDS_EFECT,
    "Music Leds",
    {
        cv.GenerateID(CONF_MUSIC_LEDS_ID): cv.use_id(MUSIC_LEDS),
        cv.Optional(CONF_MODE, default="PIXELS"): cv.enum(MUSIC_LEDS_EFFECTS, upper=True),
    },
)
async def music_leds_light_effect_to_code(config, effect_id) -> AddressableLightEffect:
    """Effect registration entry point."""
    parent = await cg.get_variable(config[CONF_MUSIC_LEDS_ID])

    effect = cg.new_Pvariable(effect_id, config[CONF_NAME])

    cg.add(effect.set_mode(config[CONF_MODE]))
    cg.add_define("DEF_" + config[CONF_MODE])
    cg.add(effect.set_music_leds(parent))
    return effect
