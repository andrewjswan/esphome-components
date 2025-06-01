#pragma once

#include "esphome/components/audio/audio_transfer_buffer.h"
#include "esphome/components/light/addressable_light.h"
#include "esphome/components/microphone/microphone_source.h"

#include "esphome/core/color.h"
#include "esphome/core/component.h"
#include "esphome/core/ring_buffer.h"

#define FASTLED_INTERNAL  // remove annoying pragma messages

#include <FastLED.h>

namespace esphome {
namespace music_leds {

static const char *const TAG = "music_leds";
static const char *const MUSIC_LEDS_VERSION = "2025.6.1";

enum PLAYMODE { MODE_GRAV, MODE_GRAVICENTER, MODE_GRAVICENTRIC, MODE_GRAVIMETER, MODE_PIXELS, MODE_JUNGLES, MODE_MIDNOISE };

class MusicLeds : public Component {
 public:
  float get_setup_priority() const override { return setup_priority::LATE; }

  void setup() override;
  void loop() override;
  void dump_config() override;
  void on_shutdown() override;

  void set_microphone(microphone::Microphone *microphone) { this->microphone_ = microphone; }

  void set_speed(int index);
  void set_variant(int index);

  void getSamples(float *buffer, uint16_t num_samples);
  void StartFrame() { this->start_effect_ = true; };
  void ShowFrame(PLAYMODE CurrentMode, Color current_color, light::AddressableLight *p_it);

  bool microphone_is_running() { return this->microphone_->is_running(); }

protected:
  microphone::Microphone *microphone_{nullptr};

  std::unique_ptr<audio::AudioSourceTransferBuffer> audio_buffer_;
  std::weak_ptr<RingBuffer> ring_buffer_;

  /// @brief Internal start command that, if necessary, allocates ``audio_buffer_`` and a ring buffer which
  /// ``audio_buffer_`` owns and ``ring_buffer_`` points to. Returns true if allocations were successful.
  bool buffer_allocate();

  /// @brief Internal stop command the deallocates ``audio_buffer_`` (which automatically deallocates its ring buffer)
  void buffer_deallocate();

  void on_start();
  void on_loop();
  void on_stop();

  // variables used by getSample() and agcAvg()
  int16_t micIn{0};                // Current sample starts with negative values and large values, which is why it's 16 bit signed
  double sampleMax{0.0};           // Max sample over a few seconds. Needed for AGC controller.
  double micLev{0.0};              // Used to convert returned value to have '0' as minimum. A leveller
  float expAdjF{0.0f};             // Used for exponential filter.
  float sampleReal{0.0f};          // "sampleRaw" as float, to provide bits that are lost otherwise (before amplification by sampleGain or inputLevel). Needed for AGC.
  int16_t sampleRaw{0};            // Current sample. Must only be updated ONCE!!! (amplified mic value by sampleGain and inputLevel)
  int16_t rawSampleAgc{0};         // not smoothed AGC sample

  void agcAvg(unsigned long the_time);
  void getSample();
  #ifdef USE_SOUND_DYNAMICS_LIMITER  
  void limitSampleDynamics(void);
  #endif

  // Used for AGC
  int last_soundAgc{-1};           // used to detect AGC mode change (for resetting AGC internal error buffers)
  double control_integrated{0.0};  // persistent across calls to agcAvg(); "integrator control" = accumulated error

  // Variables used in effects
  float volumeSmth{0.0f};          // Either sampleAvg or sampleAgc depending on soundAgc; smoothed sample
  int16_t volumeRaw{0};            // Either sampleRaw or rawSampleAgc depending on soundAgc
  float my_magnitude{0.0f};        // FFT_Magnitude, scaled by multAgc

  CRGB main_color;                 // SEGCOLOR(0) - First Color in WLED
  CRGB back_color;                 // SEGCOLOR(1) - Second Color in WLED (Background)

  uint16_t leds_num{0};            // Count of Leds
  uint8_t speed{128};              // Speed
  uint8_t variant{128};            // Variant
  bool start_effect_{false};       // Effect start?
  byte *data;                      // Effect data pointer
  unsigned _dataLen;               // Data length

  bool allocateData(size_t len);
  void deallocateData();

#if defined(DEF_GRAV) || defined(DEF_GRAVICENTER) || defined (DEF_GRAVICENTRIC) || defined(DEF_GRAVIMETER)
  void mode_gravcenter_base(unsigned mode, CRGB *physic_leds);
#endif

#ifdef DEF_GRAV
  void visualize_gravfreq(CRGB *physic_leds);
#endif
#ifdef DEF_GRAVICENTER
  void visualize_gravcenter(CRGB *physic_leds);
#endif
#ifdef DEF_GRAVICENTRIC
  void visualize_gravcentric(CRGB *physic_leds);
#endif
#ifdef DEF_GRAVIMETER
  void visualize_gravmeter(CRGB *physic_leds);
#endif
#ifdef DEF_PIXELS
  void visualize_pixels(CRGB *physic_leds);
#endif
#ifdef DEF_JUNGLES
  void visualize_juggles(CRGB *physic_leds);
#endif
#ifdef DEF_MIDNOISE
  void visualize_midnoise(CRGB *physic_leds);
#endif
};

}  // namespace music_leds
}  // namespace esphome
