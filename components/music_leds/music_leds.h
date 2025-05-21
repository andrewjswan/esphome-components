#pragma once

#include "esphome.h"

namespace esphome {
namespace music_leds {

#define I2S_SAMPLE_RESOLUTION I2S_BITS_PER_SAMPLE_16BIT
#define I2S_datatype int16_t
#define I2S_unsigned_datatype uint16_t
#define I2S_buffer_size (BITS_PER_SAMPLE / I2S_SAMPLE_RESOLUTION) * samplesFFT
#define READ_DURATION_MS 16UL

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

  void set_microphone(i2s_audio::I2SAudioMicrophone *microphone) { this->microphone_ = microphone; }
  void set_task_core(uint8_t task_core) { this->task_core_ = task_core; }

  void set_speed(int index);
  void set_variant(int index);

  void ShowFrame(PLAYMODE CurrentMode, Color current_color, light::AddressableLight *p_it);

  bool microphone_is_running() { return this->microphone_->is_running(); }

 private:
  i2s_audio::I2SAudioMicrophone *microphone_;
  TaskHandle_t FFT_Task{nullptr};
  uint8_t task_core_{1};

  uint32_t _mask{0xFFFFFFFF};  // Bitmask for sample data after shifting. Bitmask 0X0FFF means that we need to convert
                               // 12bit ADC samples from unsigned to signed
  int16_t _shift{0};           // Shift obtained samples to the right (positive) or left(negative) by this amount
  int8_t _myADCchannel{0x0F};  // current ADC channel, in case of analog input. 0x0F if undefined
  float _sampleScale{1.0f};    // pre-scaling factor for I2S samples
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
