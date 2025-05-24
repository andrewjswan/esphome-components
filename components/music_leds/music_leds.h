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

#if BITS_PER_SAMPLE == 16
  #define I2S_datatype int16_t
  #define I2S_unsigned_datatype uint16_t
  #undef  I2S_SAMPLE_DOWNSCALE_TO_16BIT
#else
  #define I2S_datatype int32_t
  #define I2S_unsigned_datatype uint32_t
  #define I2S_SAMPLE_DOWNSCALE_TO_16BIT
#endif

// Uncomment the line below to utilize ADC1 _exclusively_ for I2S sound input.
// benefit: analog mic inputs will be sampled contiously -> better response times and less "glitches"
// WARNING: this option WILL lock-up your device in case that any other analogRead() operation is performed;
//          for example if you want to read "analog buttons"
// #define I2S_GRAB_ADC1_COMPLETELY // (experimental) continously sample analog ADC microphone. WARNING will cause
// analogRead() lock-up

static const char *const TAG = "music_leds";
static const char *const MUSIC_LEDS_VERSION = "2025.5.1";

enum PLAYMODE { MODE_BINMAP, MODE_GRAV, MODE_GRAVICENTER, MODE_GRAVICENTRIC, MODE_PIXELS, MODE_JUNGLES, MODE_MIDNOISE };

class MusicLeds : public Component {
 public:
  float get_setup_priority() const override { return setup_priority::LATE; }

  void setup() override;
  void dump_config() override;
  void on_shutdown() override;

  void set_microphone(microphone::MicrophoneSource *microphone) { this->microphone_ = microphone; }
  void set_task_core(uint8_t task_core) { this->task_core_ = task_core; }

  void set_speed(int index);
  void set_variant(int index);

  void ShowFrame(PLAYMODE CurrentMode, Color current_color, light::AddressableLight *p_it);

  bool microphone_is_running() { return this->microphone_->is_running(); }

protected:
  /// @brief Internal start command that, if necessary, allocates ``audio_buffer_`` and a ring buffer which
  /// ``audio_buffer_`` owns and ``ring_buffer_`` points to. Returns true if allocations were successful.
  bool buffer_allocate();

  /// @brief Internal stop command the deallocates ``audio_buffer_`` (which automatically deallocates its ring buffer)
  void buffer_deallocate();

  microphone::MicrophoneSource *microphone_{nullptr};

  std::unique_ptr<audio::AudioSourceTransferBuffer> audio_buffer_;
  std::weak_ptr<RingBuffer> ring_buffer_;

  TaskHandle_t FFT_Task{nullptr};
  uint8_t task_core_{1};

  uint32_t _mask{0xFFFFFFFF};                                   // Bitmask for sample data after shifting. Bitmask 0X0FFF means that we need to convert
                                                                // 12bit ADC samples from unsigned to signed
  int16_t _shift{0};                                            // Shift obtained samples to the right (positive) or left(negative) by this amount
  int8_t _myADCchannel{0x0F};                                   // current ADC channel, in case of analog input. 0x0F if undefined
  float _sampleScale{1.0f};                                     // pre-scaling factor for I2S samples
  unsigned int _broken_samples_counter{0};                      // counts number of broken (and fixed) ADC samples
  I2S_datatype _lastADCsample{0};                               // last sample from ADC
  I2S_datatype decodeADCsample(I2S_unsigned_datatype rawData);  // function to handle ADC samples
  static void FFTcode(void *parameter);
 
  void getSamples(float *buffer);

  PLAYMODE CurrentMode = MODE_GRAV;

  CRGB main_color;  // SEGCOLOR(0) - First Color in WLED
  CRGB back_color;  // SEGCOLOR(1) - Second Color in WLED (Background)

  int topLED{0};
  int gravityCounter{0};

  uint8_t speed{128};
  uint8_t variant{128};
  uint16_t leds_num;

#ifdef DEF_GRAV
  void visualize_gravfreq(CRGB *physic_leds);
#endif
#ifdef DEF_GRAVICENTER
  void visualize_gravcenter(CRGB *physic_leds);
#endif
#ifdef DEF_GRAVICENTRIC
  void visualize_gravcentric(CRGB *physic_leds);
#endif
#ifdef DEF_BINMAP
  void visualize_binmap(CRGB *physic_leds);
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
