"""Music Leds component for ESPHome."""

import logging

import esphome.codegen as cg
import esphome.config_validation as cv
import esphome.final_validate as fv
from esphome import core
from esphome.components import microphone
from esphome.components.light.effects import register_addressable_effect
from esphome.components.light.types import AddressableLightEffect
from esphome.const import (
    CONF_BITS_PER_SAMPLE,
    CONF_ID,
    CONF_MICROPHONE,
    CONF_MODE,
    CONF_NAME,
    CONF_PLATFORM,
    CONF_SAMPLE_RATE,
)

from .const import (
    CONF_BAND_PASS_FILTER,
    CONF_FFT_SCALING,
    CONF_GAINCONTROL,
    CONF_MUSIC_LEDS_ID,
    CONF_SOUND_DYNAMICS_LIMITER,
    CONF_SR_GAIN,
    CONF_SR_SQUELCH,
    CONF_TASK_CORE,
    CONF_TASK_PRIORITY,
    SAMPLE_RATE_10,
    SAMPLE_RATE_16,
    SAMPLE_RATE_20,
    SAMPLE_RATE_22,
)

_LOGGER = logging.getLogger(__name__)

CODEOWNERS = ["@andrewjswan"]

DEPENDENCIES = ["light", "microphone"]

AUTO_LOAD = ["music_leds", "fastled_helper"]

logging.info("Load Music Leds component https://github.com/andrewjswan/esphome-components")

music_leds_ns = cg.esphome_ns.namespace("music_leds")
MUSIC_LEDS = music_leds_ns.class_("MusicLeds", cg.Component)
MUSIC_LEDS_EFECT = music_leds_ns.class_("MusicLedsLightEffect", AddressableLightEffect)

PlayMode = music_leds_ns.enum("PLAYMODE")
MUSIC_LEDS_EFFECTS = {
    "GRAV": PlayMode.MODE_GRAV,
    "GRAVICENTER": PlayMode.MODE_GRAVICENTER,
    "GRAVICENTRIC": PlayMode.MODE_GRAVICENTRIC,
    "GRAVIMETER": PlayMode.MODE_GRAVIMETER,
    "PIXELS": PlayMode.MODE_PIXELS,
    "JUNGLES": PlayMode.MODE_JUNGLES,
    "MIDNOISE": PlayMode.MODE_MIDNOISE,
}

MUSIC_LEDS_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_ID): cv.declare_id(MUSIC_LEDS),
        cv.Optional(CONF_BAND_PASS_FILTER, default=False): cv.boolean,
        cv.Optional(CONF_SOUND_DYNAMICS_LIMITER, default=True): cv.boolean,
        cv.Optional(CONF_BITS_PER_SAMPLE): cv.float_,
        cv.Optional(CONF_SAMPLE_RATE): cv.int_,
        cv.Optional(CONF_TASK_CORE, default=1): cv.int_range(0, 1),
        cv.Optional(CONF_TASK_PRIORITY, default=3): cv.int_range(1, 10),
        cv.Optional(CONF_GAINCONTROL, default=0): cv.int_range(0, 3),
        cv.Optional(CONF_FFT_SCALING, default=3): cv.int_range(0, 3),
        cv.Optional(CONF_SR_GAIN, default=60): cv.int_range(0, 255),
        cv.Optional(CONF_SR_SQUELCH, default=10): cv.int_range(0, 255),
    },
).extend(
    {
        cv.GenerateID(CONF_MICROPHONE): cv.use_id(microphone.Microphone),
    },
)

CONFIG_SCHEMA = cv.All(MUSIC_LEDS_SCHEMA)


def _final_validate(config):  # noqa: ANN202
    full_config = fv.full_config.get()

    path = full_config.get_path_for_id(config[CONF_ID])[:-1]
    this_config = full_config.get_config_for_path(path)

    mic_path = full_config.get_path_for_id(config[CONF_MICROPHONE])[:-1]
    mic_conf = full_config.get_config_for_path(mic_path)
    logging.info("Microphone: %s", mic_conf.get(CONF_PLATFORM))

    if CONF_SAMPLE_RATE in mic_conf:
        rate = mic_conf.get(CONF_SAMPLE_RATE)
        this_config[CONF_SAMPLE_RATE] = rate
        logging.info("Sample Rate: %s", rate)

    if CONF_BITS_PER_SAMPLE in mic_conf:
        bits = mic_conf.get(CONF_BITS_PER_SAMPLE)
        if bits not in [16, 32]:
            msg = "Music Leds support only 16 or 32 Bits Per Sample"
            raise cv.Invalid(msg)
        this_config[CONF_BITS_PER_SAMPLE] = bits
        logging.info("Bits Per Sample: %s", bits)


FINAL_VALIDATE_SCHEMA = _final_validate


async def to_code(config) -> None:
    """Code generation entry point."""
    var = cg.new_Pvariable(config[CONF_ID])

    cg.add_library("kosme/arduinoFFT", None)
    # Below options are forcing ArduinoFFT to use sqrtf() instead of sqrt()
    # #define sqrt_internal sqrtf // see https://github.com/kosme/arduinoFFT/pull/83 - since v2.0.0 this must be done in build_flags
    cg.add_define("sqrt_internal", "sqrtf")

    cg.add_define("USE_MUSIC_LEDS")
    cg.add_define("USE_OTA_STATE_CALLBACK")

    cg.add_build_flag("-Wno-narrowing")

    mic = await cg.get_variable(config[CONF_MICROPHONE])
    cg.add(var.set_microphone(mic))

    cg.add_define("BITS_PER_SAMPLE", int(config[CONF_BITS_PER_SAMPLE]))

    # FFTTASK_CORE 0 standard: Core #0
    # FFTTASK_CORE 1 standard: Core #1
    cg.add_define("FFTTASK_CORE", config[CONF_TASK_CORE])

    # FFTTASK_PRIORITY 1 standard: looptask prio
    # FFTTASK_PRIORITY 2 above looptask, below asyc_tcp
    # FFTTASK_PRIORITY 4 above asyc_tcp
    cg.add_define("FFTTASK_PRIORITY", config[CONF_TASK_PRIORITY])

    # Sample gain
    cg.add_define("SR_GAIN", config[CONF_SR_GAIN])

    # Squelch value for volume reactive routines
    cg.add_define("SR_SQUELCH", config[CONF_SR_SQUELCH])

    # FFTResult scaling: 0 none; 1 optimized logarithmic; 2 optimized linear; 3 optimized square root
    cg.add_define("FFT_SCALING", config[CONF_FFT_SCALING])

    # Automagic gain control: 0 - none, 1 - normal, 2 - vivid, 3 - lazy (config value)
    cg.add_define("GAIN_CONTROL", config[CONF_GAINCONTROL])

    # bool: enable / disable Sound Dynamics Limiter
    cg.add_define("USE_SOUND_DYNAMICS_LIMITER", config[CONF_SOUND_DYNAMICS_LIMITER])

    # Band Pass Filter - can reduce noise floor by a factor of 50
    # downside: frequencies below 100Hz will be ignored
    cg.add_define("USE_BANDPASSFILTER", config[CONF_BAND_PASS_FILTER])

    # SAMPLE_RATE 22050        // Base sample rate in Hz - 22Khz is a standard rate. Physical sample time -> 23ms
    # SAMPLE_RATE 20480        // Base sample rate in Hz - 20Khz is experimental.    Physical sample time -> 25ms
    # SAMPLE_RATE 16000        // 16kHz - use if FFTtask takes more than 20ms.       Physical sample time -> 32ms
    # SAMPLE_RATE 10240        // Base sample rate in Hz - previous default.         Physical sample time -> 50ms
    cg.add_define("SAMPLE_RATE", config[CONF_SAMPLE_RATE])

    # FFT_MIN_CYCLE 21         // minimum time before FFT task is repeated. Use with 22Khz sampling
    # FFT_MIN_CYCLE 23         // minimum time before FFT task is repeated. Use with 20Khz sampling
    # FFT_MIN_CYCLE 30         // Use with 16Khz sampling
    # FFT_MIN_CYCLE 46         // minimum time before FFT task is repeated. Use with 10Khz sampling
    if config[CONF_SAMPLE_RATE] >= SAMPLE_RATE_22:
        cg.add_define("FFT_MIN_CYCLE", 21)
    elif config[CONF_SAMPLE_RATE] >= SAMPLE_RATE_20:
        cg.add_define("FFT_MIN_CYCLE", 23)
    elif config[CONF_SAMPLE_RATE] >= SAMPLE_RATE_16:
        cg.add_define("FFT_MIN_CYCLE", 30)
    elif config[CONF_SAMPLE_RATE] >= SAMPLE_RATE_10:
        cg.add_define("FFT_MIN_CYCLE", 46)
    else:
        msg = "Low Sample rate for Music Leds plz increase."
        raise core.EsphomeError(msg)

    await cg.register_component(var, config)


@register_addressable_effect(
    "music_leds_effect",
    MUSIC_LEDS_EFECT,
    "Music Leds",
    {
        cv.GenerateID(CONF_MUSIC_LEDS_ID): cv.use_id(MUSIC_LEDS),
        cv.Optional(CONF_MODE, default="GRAVICENTRIC"): cv.enum(MUSIC_LEDS_EFFECTS, upper=True),
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
