#pragma once

#include "constants.h"

#include "band_aggregator.h"
#include "beat_detector.h"
#include "dynamics_processor.h"
#include "fft_engine.h"
#include "geq_processor.h"
#include "noise_gate.h"
#include "peak_latch.h"
#include "pre_amplifier.h"
#include "ring_buffer.h"

#include "esphome/components/light/addressable_light.h"
#include "esphome/components/fastled_helper/utils.h"
#include "esphome/components/microphone/microphone_source.h"

#include "esphome/core/automation.h"
#include "esphome/core/color.h"
#include "esphome/core/component.h"
#include "esphome/core/defines.h"

#ifdef USE_OTA_STATE_LISTENER
#include "esphome/components/ota/ota_backend.h"
#endif

namespace esphome::music_leds {

static const char *const TAG = "music_leds";
static const char *const MUSIC_LEDS_VERSION = "2026.7.5";

enum PLAYMODE {
  MODE_BLURZ,
  MODE_FREQWAVE,
  MODE_GRAV,
  MODE_GRAVICENTER,
  MODE_GRAVICENTRIC,
  MODE_GRAVIMETER,
  MODE_PIXELS,
  MODE_JUNGLES,
  MODE_MIDNOISE,
  MODE_RIPPLEPEAK,
  MODE_MATRIPIX,
  MODE_NOISEFIRE,
  MODE_NOISEMETER,
  MODE_PIXELWAVE,
  MODE_PLASMOID,
  MODE_PUDDLEPEAK,
  MODE_PUDDLES,
  MODE_DJLIGHT,
  MODE_WATERFALL
};

#if defined(MUSIC_LEDS_TRIGGERS)
class MusicLedsSoundLoopTrigger;
#endif

struct AudioPipelineFeatures {
  // Amplitude & Volume (0.0f To 1.0f)
  float smoothed_volume{0.0f};  // Filtered overall loudness (perfect for smooth brightness)
  float raw_volume{0.0f};       // Instantaneous frame loudness (perfect for strobes and sharp pulses)

  // Frequency Analysis
  float dominant_frequency_hz{1.0f};        // Major pitch tone in Hz (e.g., 440.0f for dynamic color hues)
  float magnitude{0.0f};                    // Un-normalized raw peak magnitude
  uint8_t fft_result[NUM_GEQ_CHANNELS]{0};  // 16-channel array / fftResult

  // Band Energies (0.0f To 1.0f, Agc Normalized)
  float bass_energy{0.0f};  // Sub-bass & low kick punch power (reds / physical thumping)
  float mid_energy{0.0f};   // Vocals, guitars, and main instrumentation (greens / core movement)
  float high_energy{0.0f};  // Cymbals, hi-hats, shakers, and crisp air (blues / sparkles)

  // Musical Beat & Attacks
  bool is_beat_detected{false};  // True for a single frame when a sharp audio attack occurs (onset)

  bool sample_peak{false};  // Time-locked high activity latch (Auto-resets after 50ms)

  // Returns overall loudness scaled to standard 8-bit byte integer [0 .. 255]
  inline uint8_t volume_smth() const {
    float scaled_vol = this->smoothed_volume * 255.0f;
    return static_cast<uint8_t>(std::clamp(scaled_vol, 0.0f, 255.0f));
  }

  // Returns instantaneous loudness scaled to standard 8-bit byte integer [0 .. 255]
  inline uint8_t volume_raw() const {
    float scaled_vol = this->raw_volume * 255.0f;
    return static_cast<uint8_t>(std::clamp(scaled_vol, 0.0f, 255.0f));
  }
};

class MusicLeds final : public Component
#ifdef USE_OTA_STATE_LISTENER
    ,
                        public ota::OTAGlobalStateListener
#endif
{
 public:
  ~MusicLeds() override;

  float get_setup_priority() const override { return setup_priority::LATE; }

  void setup() override;
  void loop() override;
  void dump_config() override;
  void on_shutdown() override;

  void start();
  void stop();

  bool is_running() const { return this->state_ == State::RUNNING; }
  bool is_stopped() const { return this->state_ == State::STOPPED; }

  void set_microphone(microphone::Microphone *microphone) { this->microphone_ = microphone; }

  void set_speed(int index) { this->speed = index; }
  void set_variant(int index) { this->variant = index; }

  void set_scaling_mode(FFTScalingMode mode) { this->scaling_mode_ = mode; }
  void set_beat_sensitivity(int sensitivity) { this->beat_sensitivity_ = sensitivity; }
  void set_noise_gate_floor(float floor) { this->noise_gate_floor_ = floor; }
  void set_pre_amp_gain(float gain) { this->pre_amp_gain_ = gain; }
  void set_sample_gain(uint8_t gain) { this->sample_gain_ = gain; }
  void set_sample_scale(uint8_t scale) { this->sample_scale_ = 1.0f / static_cast<float>(scale); }

  void start_frame() { this->start_effect_ = true; };
  void show_frame(PLAYMODE CurrentMode, Color current_color, light::AddressableLight *p_it);

  bool microphone_is_running() { return this->microphone_->is_running(); }

#if defined(MUSIC_LEDS_TRIGGERS)
  void add_on_sound_loop_trigger(MusicLedsSoundLoopTrigger *t) { this->on_sound_loop_triggers_.push_back(t); }
#endif

#ifdef USE_OTA_STATE_LISTENER
  void on_ota_global_state(ota::OTAState state, float progress, uint8_t error, ota::OTAComponent *comp) override;
#endif

 protected:
  microphone::Microphone *microphone_{nullptr};

  void on_start();
  void on_loop();
  void on_stop();

  RingBuffer<float, RING_BUFFER_SIZE> ring_buffer_;
  void process_audio_to_ring_(const std::vector<uint8_t> &data);

  static void FFT_Code(void *params);
  TaskHandle_t FFT_Task{nullptr};

  FFTScalingMode scaling_mode_{FFTScalingMode::SQUARE_ROOT};
  int beat_sensitivity_{65};
  float noise_gate_floor_{0.05f};
  float pre_amp_gain_{4.5f};
  uint8_t sample_gain_{60};
  float sample_scale_{1.0f / 24.0f};
  uint32_t sample_rate_{22050};

  float *fft_buffer_{nullptr};
  std::unique_ptr<FFTEngine> fft_engine_{nullptr};
  std::unique_ptr<BandAggregator> band_aggregator_{nullptr};
  std::unique_ptr<DynamicsProcessor> dynamics_processor_{nullptr};
  std::unique_ptr<BeatDetector> beat_detector_{nullptr};
  std::unique_ptr<PeakLatch> peak_latch_{nullptr};
  std::unique_ptr<NoiseGate> noise_gate_{nullptr};
  std::unique_ptr<PreAmplifier> pre_amplifier_{nullptr};
  std::unique_ptr<GEQProcessor> geq_processor_{nullptr};

  State state_{State::STOPPED};
  void set_state_(State state);

  // Handles managing the stop/state of the FFT task
  EventGroupHandle_t event_group_;

  // AudioPipelineFeatures
  AudioPipelineFeatures features_;

  CRGB main_color;  // SEGCOLOR(0) - First Color in WLED
  CRGB back_color;  // SEGCOLOR(1) - Second Color in WLED (Background)

  uint16_t leds_num{0};       // Count of Leds
  uint8_t speed{128};         // Speed
  uint8_t variant{128};       // Variant
  bool start_effect_{false};  // Effect start?
  byte *data;                 // Effect data pointer
  unsigned _dataLen;          // Data length
  uint8_t store{UINT8_MAX};   // Internal storage

  bool allocateData(size_t len);
  void deallocateData();

#if defined(DEF_GRAV) || defined(DEF_GRAVICENTER) || defined(DEF_GRAVICENTRIC) || defined(DEF_GRAVIMETER)
  void mode_gravcenter_base(unsigned mode, CRGB *physic_leds);
#endif

#if defined(DEF_PUDDLES) || defined(DEF_PUDDLEPEAK)
  void puddles_base(CRGB *physic_leds, bool peakdetect);
#endif

#ifdef DEF_BLURZ
  void visualize_blurz(CRGB *physic_leds);
#endif
#ifdef DEF_FREQWAVE
  void visualize_freqwave(CRGB *physic_leds);
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
#ifdef DEF_RIPPLEPEAK
  void visualize_ripplepeak(CRGB *physic_leds);
#endif
#ifdef DEF_MATRIPIX
  void visualize_matripix(CRGB *physic_leds);
#endif
#ifdef DEF_NOISEFIRE
  void visualize_noisefire(CRGB *physic_leds);
#endif
#ifdef DEF_NOISEMETER
  void visualize_noisemeter(CRGB *physic_leds);
#endif
#ifdef DEF_PIXELWAVE
  void visualize_pixelwave(CRGB *physic_leds);
#endif
#ifdef DEF_PLASMOID
  void visualize_plasmoid(CRGB *physic_leds);
#endif
#ifdef DEF_PUDDLEPEAK
  void visualize_puddlepeak(CRGB *physic_leds);
#endif
#ifdef DEF_PUDDLES
  void visualize_puddles(CRGB *physic_leds);
#endif
#ifdef DEF_DJLIGHT
  void visualize_DJLight(CRGB *physic_leds);
#endif
#ifdef DEF_WATERFALL
  void visualize_waterfall(CRGB *physic_leds);
#endif

#if defined(MUSIC_LEDS_TRIGGERS)
  std::vector<MusicLedsSoundLoopTrigger *> on_sound_loop_triggers_;
#endif
};  // class MusicLeds

#if defined(MUSIC_LEDS_TRIGGERS)
class MusicLedsSoundLoopTrigger : public Trigger<float, int16_t, float, bool> {
 public:
  explicit MusicLedsSoundLoopTrigger(MusicLeds *parent) { parent->add_on_sound_loop_trigger(this); }
  void process(float, int16_t, float, bool);
};  // class MusicLedsSoundLoopTrigger
#endif

}  // namespace esphome::music_leds
