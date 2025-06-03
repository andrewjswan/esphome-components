#include "music_leds.h"

#include "esphome/components/fastled_helper/utils.h"
#include "esphome/components/light/addressable_light_effect.h"

#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

#define FASTLED_INTERNAL  // remove annoying pragma messages
#include <FastLED.h>

#ifdef USE_OTA
#include "esphome/components/ota/ota_backend.h"
#endif

#include <arduinoFFT.h>

#if BITS_PER_SAMPLE == 16
  #define I2S_datatype int16_t
  #define I2S_unsigned_datatype uint16_t
  #undef  I2S_SAMPLE_DOWNSCALE_TO_16BIT
#else
  #define I2S_datatype int32_t
  #define I2S_unsigned_datatype uint32_t
  #define I2S_SAMPLE_DOWNSCALE_TO_16BIT
#endif

#define NUM_GEQ_CHANNELS 16 // number of frequency channels. Don't change !!
#define SAMPLES_FFT 512     // Samples in an FFT batch - This value MUST ALWAYS be a power of 2

#define WDT_SOUND_TIMEOUT 5

namespace esphome {
namespace music_leds {

static const size_t DATA_TIMEOUT_MS = 50;
static const uint32_t BUFFER_SIZE = sizeof(I2S_datatype) * SAMPLES_FFT;

void MusicLeds::setup() {
  this->microphone_->add_data_callback([this](const std::vector<uint8_t> &data) {
    std::shared_ptr<RingBuffer> temp_ring_buffer = this->ring_buffer_.lock();
    if (this->ring_buffer_.use_count() > 1) {
      size_t bytes_free = temp_ring_buffer->free();
      if (bytes_free < data.size()) {
        temp_ring_buffer->reset();
      }
      temp_ring_buffer->write((void *) data.data(), data.size());
    }
  });

#ifdef USE_OTA
  ota::get_global_ota_callback()->add_on_state_callback(
      [this](ota::OTAState state, float progress, uint8_t error, ota::OTAComponent *comp) {
        if (state == ota::OTA_STARTED) {
          this->on_shutdown();
        }
      });
#endif

  this->on_start();
  this->microphone_->start();

  ESP_LOGCONFIG(TAG, "Music Leds initialized");
}

void MusicLeds::loop() {
  if (this->microphone_is_running()) {
    this->on_loop();
  }
}

void MusicLeds::dump_config() {
  ESP_LOGCONFIG(TAG, "Music Leds version: %s", MUSIC_LEDS_VERSION);
  if (this->is_failed()) {
    ESP_LOGCONFIG(TAG, "Music Leds failed!");
  }
  ESP_LOGCONFIG(TAG, "         Task Core: %u", FFTTASK_CORE);
  ESP_LOGCONFIG(TAG, "     Task Priority: %u", FFTTASK_PRIORITY);
  ESP_LOGCONFIG(TAG, "           Samples: %dbit", BITS_PER_SAMPLE);
  ESP_LOGCONFIG(TAG, "       Sample rate: %d", SAMPLE_RATE);
  ESP_LOGCONFIG(TAG, "       Sample gain: %u", SR_GAIN);
  ESP_LOGCONFIG(TAG, "     Squelch value: %u", SR_SQUELCH);
  ESP_LOGCONFIG(TAG, " FFTResult scaling: %u", FFT_SCALING);
  ESP_LOGCONFIG(TAG, "      Gain control: %u", GAIN_CONTROL);
#ifdef USE_BANDPASSFILTER
  ESP_LOGCONFIG(TAG, "  Band Pass Filter: Yes");
#endif
#ifdef USE_SOUND_DYNAMICS_LIMITER
  ESP_LOGCONFIG(TAG, "  Dynamics Limiter: Yes");
#else
  ESP_LOGCONFIG(TAG, "  Dynamics Limiter: No");
#endif
}  // dump_config()

void MusicLeds::on_shutdown() {
  this->microphone_->stop();
  this->on_stop();
}

// *****************************************************************************
// Audioreactive variables
// *****************************************************************************
static float micDataReal = 0.0f;                  // MicIn data with full 24bit resolution - lowest 8bit after decimal point
static float multAgc = 1.0f;                      // sample * multAgc = sampleAgc. Our AGC multiplier
static float sampleAvg = 0.0f;                    // Smoothed Average sample - sampleAvg < 1 means "quiet" (simple noise gate)
static float sampleAgc = 0.0f;                    // Smoothed AGC sample
static float FFT_MajorPeak = 1.0f;                // FFT: strongest (peak) frequency
static float FFT_Magnitude = 0.0f;                // FFT: volume (magnitude) of peak frequency
static bool samplePeak = false;                   // Boolean flag for peak - used in effects. Responding routine may reset this flag. Auto-reset after 50ms
static unsigned long timeOfPeak = 0;              // time of last sample peak detection
static uint8_t fftResult[NUM_GEQ_CHANNELS]= {0};  // Our calculated freq. channel result table to be used by effects

// Peak detection
static void detectSamplePeak(void);               // peak detection function (needs scaled FFT results in vReal[])
static void autoResetPeak(void);                  // peak auto-reset function
static uint8_t maxVol = 31;                       // (was 10) Reasonable value for constant volume for 'peak detector', as it won't always trigger  (deprecated)
static uint8_t binNum = 8;                        // Used to select the bin for FFT based beat detection  (deprecated)

// AGC presets
#define AGC_NUM_PRESETS 3 // AGC presets:          normal,   vivid,    lazy
const double agcSampleDecay[AGC_NUM_PRESETS]  = { 0.9994f, 0.9985f, 0.9997f}; // decay factor for sampleMax, in case the current sample is below sampleMax
const float agcZoneLow[AGC_NUM_PRESETS]       = {      32,      28,      36}; // low volume emergency zone
const float agcZoneHigh[AGC_NUM_PRESETS]      = {     240,     240,     248}; // high volume emergency zone
const float agcZoneStop[AGC_NUM_PRESETS]      = {     336,     448,     304}; // disable AGC integrator if we get above this level
const float agcTarget0[AGC_NUM_PRESETS]       = {     112,     144,     164}; // first AGC setPoint -> between 40% and 65%
const float agcTarget0Up[AGC_NUM_PRESETS]     = {      88,      64,     116}; // setpoint switching value (a poor man's bang-bang)
const float agcTarget1[AGC_NUM_PRESETS]       = {     220,     224,     216}; // second AGC setPoint -> around 85%
const double agcFollowFast[AGC_NUM_PRESETS]   = { 1/192.f, 1/128.f, 1/256.f}; // quickly follow setpoint - ~0.15 sec
const double agcFollowSlow[AGC_NUM_PRESETS]   = {1/6144.f,1/4096.f,1/8192.f}; // slowly follow setpoint  - ~2-15 secs
const double agcControlKp[AGC_NUM_PRESETS]    = {    0.6f,    1.5f,   0.65f}; // AGC - PI control, proportional gain parameter
const double agcControlKi[AGC_NUM_PRESETS]    = {    1.7f,   1.85f,    1.2f}; // AGC - PI control, integral gain parameter
const float agcSampleSmooth[AGC_NUM_PRESETS]  = {  1/12.f,   1/6.f,  1/16.f}; // smoothing factor for sampleAgc (use rawSampleAgc if you want the non-smoothed value)
// AGC presets end

// Globals
static uint8_t inputLevel = 128;                  // UI slider value
uint8_t soundSquelch = SR_SQUELCH;                // squelch value for volume reactive routines (config value)
uint8_t sampleGain = SR_GAIN;                     // sample gain (config value)

// User settable options
static uint8_t FFTScalingMode = FFT_SCALING;      // FFTResult scaling: 0 none; 1 optimized logarithmic; 2 optimized linear; 3 optimized square root
static uint8_t soundAgc = GAIN_CONTROL;           // Automagic gain control: 0 - none, 1 - normal, 2 - vivid, 3 - lazy (config value)

// User settable parameters for limitSoundDynamics()
#ifdef USE_SOUND_DYNAMICS_LIMITER
static uint16_t attackTime = 80;                  // int: attack time in milliseconds. Default 0.08sec
static uint16_t decayTime = 1400;                 // int: decay time in milliseconds.  Default 1.40sec
#endif

// *****************************************************************************
// Begin FFT Code
// *****************************************************************************

// some prototypes, to ensure consistent interfaces
static float fftAddAvg(int from, int to);  // average of several FFT result bins
#ifdef USE_BANDPASSFILTER
static void runMicFilter(uint16_t numSamples, float *sampleBuffer);  // pre-filtering of raw samples (band-pass)
#endif
static void postProcessFFTResults(bool noiseGateOpen, int numberOfChannels);  // post-processing and post-amp of GEQ channels
static I2S_datatype postProcessSample(I2S_datatype sample_in);   // samples post-processing

// Table of multiplication factors so that we can even out the frequency response.
static float fftResultPink[NUM_GEQ_CHANNELS] = { 1.70f, 1.71f, 1.73f, 1.78f, 1.68f, 1.56f, 1.55f, 1.63f, 1.79f, 1.62f, 1.80f, 2.06f, 2.47f, 3.35f, 6.83f, 9.55f };

// FFT Task variables (filtering and post-processing)
static float fftCalc[NUM_GEQ_CHANNELS] = {0.0f};  // Try and normalize fftBin values to a max of 4096, so that 4096/16 = 256.
#ifdef USE_SOUND_DYNAMICS_LIMITER
static float fftAvg[NUM_GEQ_CHANNELS] = {0.0f};   // Calculated frequency channel results, with smoothing (used if dynamics limiter is ON)
#endif

// audio source parameters and constant
// constexpr SRate_t SAMPLE_RATE = 22050;  // Base sample rate in Hz - 22Khz is a standard rate. Physical sample time -> 23ms
// constexpr SRate_t SAMPLE_RATE = 16000;  // 16kHz - use if FFTtask takes more than 20ms.       Physical sample time -> 32ms
// constexpr SRate_t SAMPLE_RATE = 20480;  // Base sample rate in Hz - 20Khz is experimental.    Physical sample time -> 25ms
// constexpr SRate_t SAMPLE_RATE = 10240;  // Base sample rate in Hz - previous default.         Physical sample time -> 50ms

// #define FFT_MIN_CYCLE 21                // minimum time before FFT task is repeated. Use with 22Khz sampling
// #define FFT_MIN_CYCLE 30                // Use with 16Khz sampling
// #define FFT_MIN_CYCLE 23                // minimum time before FFT task is repeated. Use with 20Khz sampling
// #define FFT_MIN_CYCLE 46                // minimum time before FFT task is repeated. Use with 10Khz sampling

// FFT Constants
// the following are observed values, supported by a bit of "educated guessing"
// #define FFT_DOWNSCALE 0.65f             // 20kHz - downscaling factor for FFT results - "Flat-Top" window @20Khz, old freq channels 
#define FFT_DOWNSCALE 0.46f                // downscaling factor for FFT results - for "Flat-Top" window @22Khz, new freq channels
#define LOG_256 5.54517744f                // log(256)

// These are the input and output vectors.  Input vectors receive computed results from FFT.
static float* vReal = nullptr;             // FFT sample inputs / freq output -  these are our raw result bins
static float* vImag = nullptr;             // imaginary parts

// these options actually cause slow-downs on all esp32 processors, don't use them.
// #define FFT_SPEED_OVER_PRECISION        // enables use of reciprocals (1/x etc) - not faster on ESP32
// #define FFT_SQRT_APPROXIMATION          // enables "quake3" style inverse sqrt  - slower on ESP32

///////////////////////////
// Helper functions      //
///////////////////////////

// Compute average of several FFT result bins
static float fftAddAvg(int from, int to) {
  float result = 0.0f;
  for (int i = from; i <= to; i++) {
    result += vReal[i];
  }
  return result / float(to - from + 1);
}

///////////////////////////
// Pre / Postprocessing  //
///////////////////////////

#ifdef USE_BANDPASSFILTER
static void runMicFilter(uint16_t numSamples, float *sampleBuffer)  // pre-filtering of raw samples (band-pass)
{
  // low frequency cutoff parameter - see https://dsp.stackexchange.com/questions/40462/exponential-moving-average-cut-off-frequency
  // constexpr float alpha = 0.04f;     // 150Hz
  // constexpr float alpha = 0.03f;     // 110Hz
  constexpr float alpha = 0.0225f;      // 80hz
  // constexpr float alpha = 0.01693f;  // 60hz
  // high frequency cutoff  parameter
  // constexpr float beta1 = 0.75f;     // 11Khz
  // constexpr float beta1 = 0.82f;     // 15Khz
  // constexpr float beta1 = 0.8285f;   // 18Khz
  constexpr float beta1 = 0.85f;        // 20Khz

  constexpr float beta2 = (1.0f - beta1) / 2.0f;
  static float last_vals[2] = {0.0f};   // FIR high freq cutoff filter
  static float lowfilt = 0.0f;          // IIR low frequency cutoff filter

  for (int i=0; i < numSamples; i++) {
    // FIR lowpass, to remove high frequency noise
    float highFilteredSample;
    if (i < (numSamples-1)) {
      // smooth out spikes
      highFilteredSample = beta1*sampleBuffer[i] + beta2*last_vals[0] + beta2*sampleBuffer[i+1];
    } else {
      // special handling for last sample in array
      highFilteredSample = beta1*sampleBuffer[i] + beta2*last_vals[0]  + beta2*last_vals[1];
    }
    last_vals[1] = last_vals[0];
    last_vals[0] = sampleBuffer[i];
    sampleBuffer[i] = highFilteredSample;
    // IIR highpass, to remove low frequency noise
    lowfilt += alpha * (sampleBuffer[i] - lowfilt);
    sampleBuffer[i] = sampleBuffer[i] - lowfilt;
  }
}
#endif

// post-processing and post-amp of GEQ channels
static void postProcessFFTResults(bool noiseGateOpen, int numberOfChannels)
{
  for (int i=0; i < numberOfChannels; i++) {
    if (noiseGateOpen) { // noise gate open
      // Adjustment for frequency curves.
      fftCalc[i] *= fftResultPink[i];
      // Adjustment related to FFT windowing function
      if (FFTScalingMode > 0) fftCalc[i] *= FFT_DOWNSCALE;
      // Manual linear adjustment of gain using sampleGain adjustment for different input types.
      // Apply gain, with inputLevel adjustment
      fftCalc[i] *= soundAgc ? multAgc : ((float)sampleGain/40.0f * (float)inputLevel / 128.0f + 1.0f / 16.0f);
      if(fftCalc[i] < 0) fftCalc[i] = 0;
    }
    // Constrain internal vars - just to be sure
    fftCalc[i] = constrain(fftCalc[i], 0.0f, 1023.0f);

    #ifdef USE_SOUND_DYNAMICS_LIMITER
    // Smooth results - rise fast, fall slower
    if(fftCalc[i] > fftAvg[i]) {  // rise fast 
      fftAvg[i] = fftCalc[i] *0.75f + 0.25f*fftAvg[i];  // will need approx 2 cycles (50ms) for converging against fftCalc[i]
    } else {                      // fall slow
      if (decayTime < 1000) fftAvg[i] = fftCalc[i]*0.22f + 0.78f*fftAvg[i];       // approx  5 cycles (225ms) for falling to zero
      else if (decayTime < 2000) fftAvg[i] = fftCalc[i]*0.17f + 0.83f*fftAvg[i];  // default - approx  9 cycles (225ms) for falling to zero
      else if (decayTime < 3000) fftAvg[i] = fftCalc[i]*0.14f + 0.86f*fftAvg[i];  // approx 14 cycles (350ms) for falling to zero
      else fftAvg[i] = fftCalc[i]*0.1f  + 0.9f*fftAvg[i];                         // approx 20 cycles (500ms) for falling to zero
    }
    // Constrain internal vars - just to be sure
    fftAvg[i] = constrain(fftAvg[i], 0.0f, 1023.0f);
    #endif

    float currentResult;
    #ifdef USE_SOUND_DYNAMICS_LIMITER
    currentResult = fftAvg[i];
    #else
    currentResult = fftCalc[i];
    #endif

    switch (FFTScalingMode) {
      case 1:
          // Logarithmic scaling
          currentResult *= 0.42f;                      // 42 is the answer ;-)
          currentResult -= 8.0f;                       // this skips the lowest row, giving some room for peaks
          if (currentResult > 1.0f) currentResult = logf(currentResult); // log to base "e", which is the fastest log() function
          else currentResult = 0.0f;                   // special handling, because log(1) = 0; log(0) = undefined
          currentResult *= 0.85f + (float(i)/18.0f);   // extra up-scaling for high frequencies
          currentResult = remap(currentResult, 0.0f, LOG_256, 0.0f, 255.0f); // map [log(1) ... log(255)] to [0 ... 255]
      break;
      case 2:
          // Linear scaling
          currentResult *= 0.30f;                      // needs a bit more damping, get stay below 255
          currentResult -= 4.0f;                       // giving a bit more room for peaks
          if (currentResult < 1.0f) currentResult = 0.0f;
          currentResult *= 0.85f + (float(i)/1.8f);    // extra up-scaling for high frequencies
      break;
      case 3:
          // square root scaling
          currentResult *= 0.38f;
          currentResult -= 6.0f;
          if (currentResult > 1.0f) currentResult = sqrtf(currentResult);
          else currentResult = 0.0f;                   // special handling, because sqrt(0) = undefined
          currentResult *= 0.85f + (float(i)/4.5f);    // extra up-scaling for high frequencies
          currentResult = remap(currentResult, 0.0f, 16.0f, 0.0f, 255.0f); // map [sqrt(1) ... sqrt(256)] to [0 ... 255]
      break;
      case 0:
      default:
          // no scaling - leave freq bins as-is
          currentResult -= 4; // just a bit more room for peaks
      break;
    }

    // Now, let's dump it all into fftResult. 
    // Need to do this, otherwise other routines might grab fftResult values prematurely.
    if (soundAgc > 0) {  // apply extra "GEQ Gain" if set by user
      float post_gain = (float)inputLevel / 128.0f;
      if (post_gain < 1.0f) post_gain = ((post_gain -1.0f) * 0.8f) + 1.0f;
      currentResult *= post_gain;
    }
    fftResult[i] = constrain((int)currentResult, 0, 255);
  }
}

static I2S_datatype postProcessSample(I2S_datatype sample_in) {
  static I2S_datatype lastADCsample = 0;                       // last good sample
  // static unsigned int broken_samples_counter = 0;           // number of consecutive broken (and fixed) ADC samples
  // static uint8_t _myADCchannel = 0x0F;                      // current ADC channel, in case of analog input. 0x0F if undefined
  I2S_datatype sample_out = 0;

  // bring sample down down to 16bit unsigned
  I2S_unsigned_datatype rawData = * reinterpret_cast<I2S_unsigned_datatype *> (&sample_in); // C++ acrobatics to get sample as "unsigned"
  #if BITS_PER_SAMPLE == 16
    rawData = rawData & 0xFFFF;                               // input is already in 16bit, just mask off possible junk
    I2S_datatype lastGoodSample = lastADCsample * 4;          // prepare "last good sample" accordingly (10bit-> 12bit)
  #else
    rawData = (rawData >> 16) & 0xFFFF;                       // scale input down from 32bit -> 16bit
    I2S_datatype lastGoodSample = lastADCsample / 16384 ;     // prepare "last good sample" accordingly (26bit-> 12bit with correct sign handling)
  #endif

  // decode ADC sample data fields
  // uint16_t the_channel = (rawData >> 12) & 0x000F;         // upper 4 bit = ADC channel
  uint16_t the_sample = rawData & 0x0FFF;                     // lower 12bit -> ADC sample (unsigned)
  I2S_datatype finalSample = (int(the_sample) - 2048);        // convert unsigned sample to signed (centered at 0);

  /*
  if ((the_channel != _myADCchannel) && (_myADCchannel != 0x0F)) { // 0x0F means "don't know what my channel is" 
    // fix bad sample
    finalSample = lastGoodSample;                             // replace with last good ADC sample
    broken_samples_counter ++;
    if (broken_samples_counter > 256) {
      _myADCchannel = 0x0F;                                   // too  many bad samples in a row -> disable sample corrections
    }
  } else broken_samples_counter = 0;                          // good sample - reset counter
  */

  // back to original resolution
  #if BITS_PER_SAMPLE == 32
    finalSample = finalSample << 16;                          // scale up from 16bit -> 32bit;
  #endif

  finalSample = finalSample / 4;                              // mimic old analog driver behaviour (12bit -> 10bit)
  sample_out = (3 * finalSample + lastADCsample) / 4;         // apply low-pass filter (2-tap FIR)
  // sample_out = (finalSample + lastADCsample) / 2;          // apply stronger low-pass filter (2-tap FIR)

  lastADCsample = sample_out;                                 // update ADC last sample
  return(sample_out);
}

////////////////////
// Peak detection //
////////////////////

// peak detection is called from FFT task when vReal[] contains valid FFT results
static void detectSamplePeak(void) {
  // softhack007: this code continuously triggers while amplitude in the selected bin is above a certain threshold.
  // So it does not detect peaks - it detects high activity in a frequency bin.
  // Poor man's beat detection by seeing if sample > Average + some value.
  // This goes through ALL of the 255 bins - but ignores stupid settings
  // Then we got a peak, else we don't. The peak has to time out on its own in order to support UDP sound sync.
  if ((sampleAvg > 1) && (maxVol > 0) && (binNum > 4) && (vReal[binNum] > maxVol) && ((millis() - timeOfPeak) > 100)) {
    samplePeak = true;
    timeOfPeak = millis();
  }
}

static void autoResetPeak(void) {
  if (millis() - timeOfPeak > uint16_t(50)) {  // Auto-reset of samplePeak after 50ms
    samplePeak = false;
  }
}

// *****************************************************************************
// FFT main task 
// audio processing task: read samples, run FFT, fill GEQ channels from FFT results
// *****************************************************************************
void MusicLeds::FFTcode(void * parameter)
{
  MusicLeds *this_task = (MusicLeds *) parameter;
  ESP_LOGCONFIG(TAG, "FFT: started on core: %u", FFTTASK_CORE);

  // Allocate FFT buffers on first call
  if (vReal == nullptr) {
    vReal = (float*) calloc(sizeof(float), SAMPLES_FFT);
  }
  if (vImag == nullptr) {
    vImag = (float*) calloc(sizeof(float), SAMPLES_FFT);
  }

  if ((vReal == nullptr) || (vImag == nullptr)) {
    // Something went wrong
    if (vReal) {
      free(vReal); 
      vReal = nullptr;
    }
    if (vImag) {
      free(vImag); 
      vImag = nullptr;
    }
    ESP_LOGW(TAG, "Allocate FFT buffers failed.");
    this_task->status_set_warning();
    return;
  }

  {  // Ensures any C++ objects fall out of scope to deallocate before deleting the task
    std::unique_ptr<audio::AudioSourceTransferBuffer> audio_buffer;
    // Allocate audio transfer buffer
    audio_buffer = audio::AudioSourceTransferBuffer::create(BUFFER_SIZE);
    if (audio_buffer == nullptr) {
      ESP_LOGW(TAG, "Allocate Audio buffer failed.");
      this_task->status_set_warning();
      return;
    }
    // Allocate ring buffer
    std::shared_ptr<RingBuffer> temp_ring_buffer = RingBuffer::create(BUFFER_SIZE);
    if (temp_ring_buffer.use_count() == 0) {
      ESP_LOGW(TAG, "Allocate Ring buffer failed.");
      this_task->status_set_warning();
      return;
    }
    audio_buffer->set_source(temp_ring_buffer);
    this_task->ring_buffer_ = temp_ring_buffer;

    // Create FFT object with weighing factor storage
    ArduinoFFT<float> FFT = ArduinoFFT<float>(vReal, vImag, SAMPLES_FFT, SAMPLE_RATE, true);

    for(;;) {
      if (this_task == nullptr) {
        if (millis() % 50 == 0) ESP_LOGCONFIG(TAG, "Dead");
        ESP_LOGW(TAG, "Music Leds dead?");
        continue;
      }

      // Only run the FFT computing code if microphone running
      if (!this_task->microphone_is_running()) {
        if (millis() % 50 == 0) ESP_LOGCONFIG(TAG, "Mute");
        ESP_LOGW(TAG, "Microphone not running...");
        this_task->status_set_warning();
        continue;
      }
      this_task->status_clear_warning();

      audio_buffer->transfer_data_from_source(pdMS_TO_TICKS(DATA_TIMEOUT_MS));
      if (audio_buffer->available() < BUFFER_SIZE) {
        // Insufficient data for processing, read more next iteration
        if (millis() % 50 == 0) ESP_LOGCONFIG(TAG, "Insufficient data %d", audio_buffer->available());
        continue;
      }

      // Get a fresh batch of samples from microphone
      // Intermediary sample storage
      I2S_datatype *newSamples = reinterpret_cast<I2S_datatype *>(audio_buffer->get_buffer_start());

      float sum = 0.0f;
      // Store samples in sample buffer and update DC offset
      for (int i = 0; i < SAMPLES_FFT; i++) {
        newSamples[i] = postProcessSample(newSamples[i]);  // perform postprocessing (needed for ADC samples)
        float currSample = 0.0f;
#ifdef I2S_SAMPLE_DOWNSCALE_TO_16BIT
        currSample = (float) newSamples[i] / 65536.0f;     // 32bit input -> 16bit; keeping lower 16bits as decimal places
#else
        currSample = (float) newSamples[i];                // 16bit input -> use as-is
#endif
        vReal[i] = currSample;
        // vReal[i] *= _sampleScale;                       // scale samples float _sampleScale{1.0f}; // pre-scaling factor for I2S samples
        sum = sum + vReal[i];
      }
      audio_buffer->decrease_buffer_length(BUFFER_SIZE);   // Remove the processed samples from audio_buffer
      memset(vImag, 0, SAMPLES_FFT * sizeof(float));       // Set imaginary parts to 0

      #ifdef USE_BANDPASSFILTER
      // band pass filter - can reduce noise floor by a factor of 50
      // downside: frequencies below 100Hz will be ignored
      runMicFilter(SAMPLES_FFT, vReal);
      #endif

      // find highest sample in the batch
      float maxSample = 0.0f;  // max sample from FFT batch
      for (int i=0; i < SAMPLES_FFT; i++) {
        // pick our  our current mic sample - we take the max value from all samples that go into FFT
        // skip extreme values - normally these are artefacts
        if ((vReal[i] <= (INT16_MAX - 1024)) && (vReal[i] >= (INT16_MIN + 1024))) {
          float cSample = fabsf(vReal[i]);
          if (cSample > maxSample) {
            maxSample = cSample;
          }
        }
      }
  
      // release highest sample to volume reactive effects early - not strictly necessary here - could also be done at the end of the function
      // early release allows the filters (getSample() and agcAvg()) to work with fresh values - we will have matching gain and noise gate values when we want to process the FFT results.
      micDataReal = maxSample;
      if (millis() % 50 == 0) ESP_LOGCONFIG(TAG, "Samples %f | High %f", sum, micDataReal);

      if (sampleAvg > 0.25f) { // noise gate open means that FFT results will be used. Don't run FFT if results are not needed.
        // run FFT (takes 3-5ms on ESP32, ~12ms on ESP32-S2)
        FFT.dcRemoval();                                            // remove DC offset
        FFT.windowing( FFTWindow::Flat_top, FFTDirection::Forward); // Weigh data using "Flat Top" function - better amplitude accuracy
        //FFT.windowing(FFTWindow::Blackman_Harris, FFTDirection::Forward);  // Weigh data using "Blackman- Harris" window - sharp peaks due to excellent sideband rejection
        FFT.compute( FFTDirection::Forward );                       // Compute FFT
        FFT.complexToMagnitude();                                   // Compute magnitudes
        vReal[0] = 0;   // The remaining DC offset on the signal produces a strong spike on position 0 that should be eliminated to avoid issues.

        FFT.majorPeak(&FFT_MajorPeak, &FFT_Magnitude);              // let the effects know which freq was most dominant
        FFT_MajorPeak = constrain(FFT_MajorPeak, 1.0f, 11025.0f);   // restrict value to range expected by effects
      } else { // noise gate closed - only clear results as FFT was skipped. MIC samples are still valid when we do this.
        memset(vReal, 0, SAMPLES_FFT * sizeof(float));
        FFT_MajorPeak = 1;
        FFT_Magnitude = 0.001;
        if (millis() % 50 == 0) ESP_LOGCONFIG(TAG, "Noise");
      }

      for (int i = 0; i < SAMPLES_FFT; i++) {
        float cSample = fabsf(vReal[i]);                            // just to be sure - values in fft bins should be positive any way
        vReal[i] = cSample / 16.0f;                                 // Reduce magnitude. Want end result to be scaled linear and ~4096 max.
      } // for()

      // mapping of FFT result bins to frequency channels
      if (fabsf(sampleAvg) > 0.5f) { // noise gate open
        /*
        * This FFT post processing is a DIY endeavour.
        * What we really need is someone with sound engineering expertise to do a great job here AND most importantly, that the animations look GREAT as a result.
        * Andrew's updated mapping of 256 bins down to the 16 result bins with Sample Freq = 10240, samplesFFT = 512 and some overlap.
        * Based on testing, the lowest/Start frequency is 60 Hz (with bin 3) and a highest/End frequency of 5120 Hz in bin 255.
        * Now, Take the 60Hz and multiply by 1.320367784 to get the next frequency and so on until the end. Then determine the bins.
        * End frequency = Start frequency * multiplier ^ 16
        * Multiplier = (End frequency/ Start frequency) ^ 1/16
        * Multiplier = 1.320367784
        * new mapping, optimized for 22050 Hz by softhack007
        */
        //                                          // bins frequency  range
        #ifdef USE_BANDPASSFILTER
        // skip frequencies below 100hz
        fftCalc[ 0] = 0.8f * fftAddAvg(3,4);
        fftCalc[ 1] = 0.9f * fftAddAvg(4,5);
        fftCalc[ 2] = fftAddAvg(5,6);
        fftCalc[ 3] = fftAddAvg(6,7);
        // don't use the last bins from 206 to 255. 
        fftCalc[15] = fftAddAvg(165,205) * 0.75f;   // 40 7106 - 8828 high  -- with some damping
        #else
        fftCalc[ 0] = fftAddAvg(1,2);               // 1    43 - 86   sub-bass
        fftCalc[ 1] = fftAddAvg(2,3);               // 1    86 - 129  bass
        fftCalc[ 2] = fftAddAvg(3,5);               // 2   129 - 216  bass
        fftCalc[ 3] = fftAddAvg(5,7);               // 2   216 - 301  bass + midrange
        // don't use the last bins from 216 to 255. They are usually contaminated by aliasing (aka noise) 
        fftCalc[15] = fftAddAvg(165,215) * 0.70f;   // 50 7106 - 9259 high  -- with some damping
        #endif
        fftCalc[ 4] = fftAddAvg(7,10);              // 3   301 - 430  midrange
        fftCalc[ 5] = fftAddAvg(10,13);             // 3   430 - 560  midrange
        fftCalc[ 6] = fftAddAvg(13,19);             // 5   560 - 818  midrange
        fftCalc[ 7] = fftAddAvg(19,26);             // 7   818 - 1120 midrange  -- 1Khz should always be the center !
        fftCalc[ 8] = fftAddAvg(26,33);             // 7  1120 - 1421 midrange
        fftCalc[ 9] = fftAddAvg(33,44);             // 9  1421 - 1895 midrange
        fftCalc[10] = fftAddAvg(44,56);             // 12 1895 - 2412 midrange + high mid
        fftCalc[11] = fftAddAvg(56,70);             // 14 2412 - 3015 high mid
        fftCalc[12] = fftAddAvg(70,86);             // 16 3015 - 3704 high mid
        fftCalc[13] = fftAddAvg(86,104);            // 18 3704 - 4479 high mid
        fftCalc[14] = fftAddAvg(104,165) * 0.88f;   // 61 4479 - 7106 high mid + high  -- with slight damping
      } else {  // noise gate closed - just decay old values
        for (int i=0; i < NUM_GEQ_CHANNELS; i++) {
          fftCalc[i] *= 0.85f;  // decay to zero
          if (fftCalc[i] < 4.0f) fftCalc[i] = 0.0f;
        }
      }

      // post-processing of frequency channels (pink noise adjustment, AGC, smoothing, scaling)
      postProcessFFTResults((fabsf(sampleAvg) > 0.25f)? true : false , NUM_GEQ_CHANNELS);

      // run peak detection
      autoResetPeak();
      detectSamplePeak();
    } // for(;;)ever
  }
} // FFTcode() task end

// *****************************************************************************
// Audio Processing
// *****************************************************************************

/*
* A "PI controller" multiplier to automatically adjust sound sensitivity.
* 
* A few tricks are implemented so that sampleAgc does't only utilize 0% and 100%:
* 0. don't amplify anything below squelch (but keep previous gain)
* 1. gain input = maximum signal observed in the last 5-10 seconds
* 2. we use two setpoints, one at ~60%, and one at ~80% of the maximum signal
* 3. the amplification depends on signal level:
*    a) normal zone - very slow adjustment
*    b) emergency zone (<10% or >90%) - very fast adjustment
*/
void MusicLeds::agcAvg(unsigned long the_time)
{
  // Make sure the _compiler_ knows this value will not change while we are inside the function
  const int AGC_preset = (soundAgc > 0) ? (soundAgc - 1) : 0;

  float lastMultAgc = multAgc;                // last multiplier used
  float multAgcTemp = multAgc;                // new multiplier
  float tmpAgc = this->sampleReal * multAgc;  // what-if amplified signal

  float control_error;                        // "control error" input for PI control

  if (this->last_soundAgc != soundAgc) {
    this->control_integrated = 0.0;           // new preset - reset integrator
  }

  // For PI controller, we need to have a constant "frequency"
  // so let's make sure that the control loop is not running at insane speed
  static unsigned long last_time = 0;
  unsigned long time_now = millis();
  if ((the_time > 0) && (the_time < time_now)) time_now = the_time;  // allow caller to override my clock

  if (time_now - last_time > 2) {
    last_time = time_now;

    if((fabsf(this->sampleReal) < 2.0f) || (this->sampleMax < 1.0)) {
      // MIC signal is "squelched" - deliver silence
      tmpAgc = 0;
      // we need to "spin down" the intgrated error buffer
      if (fabs(this->control_integrated) < 0.01) {
        this->control_integrated  = 0.0;
      } else {
        this->control_integrated *= 0.91;
      }
    } else {
      // compute new setpoint
      if (tmpAgc <= agcTarget0Up[AGC_preset]) {
        multAgcTemp = agcTarget0[AGC_preset] / this->sampleMax;   // Make the multiplier so that sampleMax * multiplier = first setpoint
      } else {
        multAgcTemp = agcTarget1[AGC_preset] / this->sampleMax;   // Make the multiplier so that sampleMax * multiplier = second setpoint
      }
    }

    // limit amplification
    if (multAgcTemp > 32.0f)      multAgcTemp = 32.0f;
    if (multAgcTemp < 1.0f/64.0f) multAgcTemp = 1.0f/64.0f;

    // compute error terms
    control_error = multAgcTemp - lastMultAgc;

    if (((multAgcTemp > 0.085f) && (multAgcTemp < 6.5f))          // integrator anti-windup by clamping
        && (multAgc*this->sampleMax < agcZoneStop[AGC_preset]))   // integrator ceiling (>140% of max)
      this->control_integrated += control_error * 0.002 * 0.25;   // 2ms = integration time; 0.25 for damping
    else
      this->control_integrated *= 0.9;                            // spin down that beasty integrator

    // apply PI Control 
    tmpAgc = this->sampleReal * lastMultAgc;                      // check "zone" of the signal using previous gain
    if ((tmpAgc > agcZoneHigh[AGC_preset]) || (tmpAgc < soundSquelch + agcZoneLow[AGC_preset])) {  // upper/lower energy zone
      multAgcTemp = lastMultAgc + agcFollowFast[AGC_preset] * agcControlKp[AGC_preset] * control_error;
      multAgcTemp += agcFollowFast[AGC_preset] * agcControlKi[AGC_preset] * this->control_integrated;
    } else {                                                      // "normal zone"
      multAgcTemp = lastMultAgc + agcFollowSlow[AGC_preset] * agcControlKp[AGC_preset] * control_error;
      multAgcTemp += agcFollowSlow[AGC_preset] * agcControlKi[AGC_preset] * this->control_integrated;
    }

    // limit amplification again - PI controller sometimes "overshoots"
    //multAgcTemp = constrain(multAgcTemp, 0.015625f, 32.0f); // 1/64 < multAgcTemp < 32
    if (multAgcTemp > 32.0f)      multAgcTemp = 32.0f;
    if (multAgcTemp < 1.0f/64.0f) multAgcTemp = 1.0f/64.0f;
  }

  // NOW finally amplify the signal
  tmpAgc = this->sampleReal * multAgcTemp;                  // apply gain to signal
  if (fabsf(this->sampleReal) < 2.0f) tmpAgc = 0.0f;        // apply squelch threshold
  //tmpAgc = constrain(tmpAgc, 0, 255);
  if (tmpAgc > 255) tmpAgc = 255.0f;                  // limit to 8bit
  if (tmpAgc < 1)   tmpAgc = 0.0f;                    // just to be sure

  // update global vars ONCE - multAgc, sampleAGC, rawSampleAgc
  multAgc = multAgcTemp;
  this->rawSampleAgc = 0.8f * tmpAgc + 0.2f * (float)this->rawSampleAgc;
  // update smoothed AGC sample
  if (fabsf(tmpAgc) < 1.0f) 
    sampleAgc =  0.5f * tmpAgc + 0.5f * sampleAgc;                   // fast path to zero
  else
    sampleAgc += agcSampleSmooth[AGC_preset] * (tmpAgc - sampleAgc); // smooth path

  sampleAgc = fabsf(sampleAgc);                                      // make sure we have a positive value
  this->last_soundAgc = soundAgc;
} // agcAvg()

// post-processing and filtering of MIC sample (micDataReal) from FFTcode()
void MusicLeds::getSample() {
  float sampleAdj;                                            // Gain adjusted sample value
  float tmpSample;                                            // An interim sample variable used for calculations.
  const float weighting = 0.2f;                               // Exponential filter weighting. Will be adjustable in a future release.
  const int AGC_preset = (soundAgc > 0) ? (soundAgc - 1) : 0; // make sure the _compiler_ knows this value will not change while we are inside the function

  this->micIn = int(micDataReal);  // micDataSm = ((micData * 3) + micData)/4;

  this->micLev += (micDataReal-this->micLev) / 12288.0f;
  if(this->micIn < this->micLev) {
    this->micLev = ((this->micLev * 31.0f) + micDataReal) / 32.0f;       // align MicLev to lowest input signal
  }

  this->micIn -= this->micLev;                                           // Let's center it to 0 now
  // Using an exponential filter to smooth out the signal. We'll add controls for this in a future release.
  float micInNoDC = fabsf(micDataReal - this->micLev);
  this->expAdjF = (weighting * micInNoDC + (1.0f-weighting) * this->expAdjF);
  this->expAdjF = fabsf(this->expAdjF);                                  // Now (!) take the absolute value

  this->expAdjF = (this->expAdjF <= soundSquelch) ? 0: this->expAdjF;    // simple noise gate
  if ((soundSquelch == 0) && (this->expAdjF < 0.25f)) this->expAdjF = 0; // do something meaningfull when "squelch = 0"

  tmpSample = this->expAdjF;
  this->micIn = abs(this->micIn);                                        // And get the absolute value of each sample

  sampleAdj = tmpSample * sampleGain / 40.0f * inputLevel / 128.0f + tmpSample / 16.0f; // Adjust the gain. with inputLevel adjustment
  this->sampleReal = tmpSample;

  sampleAdj = fmax(fmin(sampleAdj, 255), 0);  // Question: why are we limiting the value to 8 bits ???
  this->sampleRaw = (int16_t)sampleAdj;       // ONLY update sample ONCE!!!!

  // keep "peak" sample, but decay value if current sample is below peak
  if ((this->sampleMax < this->sampleReal) && (this->sampleReal > 0.5f)) {
    this->sampleMax = this->sampleMax + 0.5f * (this->sampleReal - this->sampleMax);  // new peak - with some filtering
    // another simple way to detect samplePeak - cannot detect beats, but reacts on peak volume
    if (((binNum < 12) || ((maxVol < 1))) && (millis() - timeOfPeak > 80) && (sampleAvg > 1)) {
      samplePeak    = true;
      timeOfPeak    = millis();
    }
  } else {
    if ((multAgc*this->sampleMax > agcZoneStop[AGC_preset]) && (soundAgc > 0))
      this->sampleMax += 0.5f * (this->sampleReal - this->sampleMax);        // over AGC Zone - get back quickly
    else
      this->sampleMax *= agcSampleDecay[AGC_preset];             // signal to zero --> 5-8sec
  }
  if (this->sampleMax < 0.5f) this->sampleMax = 0.0f;

  sampleAvg = ((sampleAvg * 15.0f) + sampleAdj) / 16.0f;   // Smooth it out over the last 16 samples.
  sampleAvg = fabsf(sampleAvg);                            // make sure we have a positive value
} // getSample()

#ifdef USE_SOUND_DYNAMICS_LIMITER
/* 
* Limits the dynamics of volumeSmth (= sampleAvg or sampleAgc). 
* does not affect FFTResult[] or volumeRaw ( = sample or rawSampleAgc) 
* effects: Gravimeter, Gravcenter, Gravcentric, Noisefire, Plasmoid, Freqpixels, Freqwave, Gravfreq, (2D Swirl, 2D Waverly)
*/
void MusicLeds::limitSampleDynamics(void) {
  const float bigChange = 196;  // just a representative number - a large, expected sample value
  static unsigned long last_time = 0;
  static float last_volumeSmth = 0.0f;

  long delta_time = millis() - last_time;
  delta_time = constrain(delta_time, 1, 1000);  // below 1ms -> 1ms; above 1sec -> sily lil hick-up
  float deltaSample = this->volumeSmth - last_volumeSmth;

  if (attackTime > 0) {                         // user has defined attack time > 0
    float maxAttack =   bigChange * float(delta_time) / float(attackTime);
    if (deltaSample > maxAttack) deltaSample = maxAttack;
  }
  if (decayTime > 0) {                          // user has defined decay time > 0
    float maxDecay  = - bigChange * float(delta_time) / float(decayTime);
    if (deltaSample < maxDecay) deltaSample = maxDecay;
  }

  this->volumeSmth = last_volumeSmth + deltaSample; 

  last_volumeSmth = this->volumeSmth;
  last_time = millis();
}
#endif

// *****************************************************************************************************************************************************************

void MusicLeds::on_start() {
  // Init sound data
  micDataReal = 0.0f;
  sampleAgc = 0;
  sampleAvg = 0;
  FFT_Magnitude = 0;
  FFT_MajorPeak = 1;
  multAgc = 1;

  this->volumeRaw = 0;
  this->volumeSmth = 0;
  this->sampleRaw = 0;
  this->rawSampleAgc = 0;
  this->my_magnitude = 0;

  // Reset FFT data
  memset(fftCalc, 0, sizeof(fftCalc)); 
  #ifdef USE_SOUND_DYNAMICS_LIMITER
  memset(fftAvg, 0, sizeof(fftAvg)); 
  #endif
  memset(fftResult, 0, sizeof(fftResult)); 
  for(int i = 0; i < NUM_GEQ_CHANNELS; i += 2) {
    fftResult[i] = 16; // make a tiny pattern
  }
  inputLevel = 128;    // reset level slider to default
  autoResetPeak();

  // Define the FFT Task and lock it to core
  xTaskCreatePinnedToCore(MusicLeds::FFTcode,  // Function to implement the task
                          "FFT",               // Name of the task
                          5000,                // Stack size in words
                          (void *) this,       // Task input parameter
                          FFTTASK_PRIORITY,    // Priority of the task
                          &this->FFT_Task,     // Task handle
                          FFTTASK_CORE);       // Core where the task should run
  if (this->FFT_Task == nullptr) {
    this->status_momentary_error("Task failed to start...", 1000);
  }
}

void MusicLeds::on_stop() {
  vTaskDelete(FFT_Task);
  fastled_helper::FreeLeds();
}

void MusicLeds::on_loop()
{
  static unsigned long lastUMRun = millis();

  if (soundAgc > AGC_NUM_PRESETS) soundAgc = 0; // make sure that AGC preset is valid (to avoid array bounds violation)

  unsigned long t_now = millis();               // remember current time
  int userloopDelay = int(t_now - lastUMRun);
  if (lastUMRun == 0)
    userloopDelay = 0;                          // startup - don't have valid data from last run.

  // run filters, and repeat in case of loop delays (hick-up compensation)
  if (userloopDelay <2)
    userloopDelay = 0;                          // minor glitch, no problem
  if (userloopDelay >200)
    userloopDelay = 200;                        // limit number of filter re-runs  
  do {
    this->getSample();                          // run microphone sampling filters
    this->agcAvg(t_now - userloopDelay);        // Calculated the PI adjusted value as sampleAvg
    userloopDelay -= 2;                         // advance "simulated time" by 2ms
  } while (userloopDelay > 0);
  lastUMRun = t_now;                            // update time keeping

  // update samples for effects (raw, smooth) 
  this->volumeSmth = (soundAgc) ? sampleAgc : sampleAvg;
  this->volumeRaw  = (soundAgc) ? this->rawSampleAgc: this->sampleRaw;

  // update FFTMagnitude, taking into account AGC amplification
  this->my_magnitude = FFT_Magnitude;           // / 16.0f, 8.0f, 4.0f done in effects
  if (soundAgc) this->my_magnitude *= multAgc;
  if (this->volumeSmth < 1 ) this->my_magnitude = 0.001f;   // noise gate closed - mute

  #ifdef USE_SOUND_DYNAMICS_LIMITER
  this->limitSampleDynamics();
  #endif
  autoResetPeak();                              // auto-reset sample peak after strip minShowDelay
}

// *****************************************************************************
// Data
// *****************************************************************************

// allocates effect data buffer on heap and initialises (erases) it
bool MusicLeds::allocateData(size_t len) {
  if (len == 0) {
    return false;                             // nothing to do
  }
  if (this->data && this->_dataLen >= len) {  // already allocated enough (reduce fragmentation)
    if (this->start_effect_) {
      memset(data, 0, len);                   // erase buffer if called during effect initialisation
    }
    return true;
  }

  this->deallocateData();                     // if the old buffer was smaller release it first
  // Do not use SPI RAM on ESP32 since it is slow
  this->data = (byte*)calloc(len, sizeof(byte));
  if (!this->data) {
    ESP_LOGW(TAG, "Effect Data !!! Allocation failed. !!!");
    this->status_set_warning();
    return false;
  } // allocation failed

  this->_dataLen = len;
  return true;
}

void MusicLeds::deallocateData() {
  if (!this->data) {
    this->_dataLen = 0;
    return;
  }
  // check that we don't have a dangling / inconsistent data pointer
  if (this->_dataLen > 0) {
    free(this->data);
  }
  this->data = nullptr;
  _dataLen = 0;
}

// *****************************************************************************
// Speed and Variant
// *****************************************************************************
void MusicLeds::set_speed(int index) { this->speed = index; }

void MusicLeds::set_variant(int index) { this->variant = index; }

// *****************************************************************************
// Effects
// *****************************************************************************
void MusicLeds::ShowFrame(PLAYMODE CurrentMode, esphome::Color current_color, light::AddressableLight *p_it) {
  if (!this->microphone_is_running()) {
    return;
  }

  fastled_helper::InitLeds(p_it->size());

  this->leds_num = p_it->size();

  this->main_color = CRGB(current_color.r, current_color.g, current_color.b);
  if ((int) fastled_helper::current_palette == 0) {
    // 5% from main color
    this->back_color = CRGB(current_color.r / 100 * 5, current_color.g / 100 * 5, current_color.b / 100 * 5);
  } else {
    this->back_color = CRGB::Black;
  }

  switch (CurrentMode) {
    case MODE_GRAV:
#ifdef DEF_GRAV
      this->visualize_gravfreq(fastled_helper::leds);
#endif
      break;
    case MODE_GRAVICENTER:
#ifdef DEF_GRAVICENTER
      this->visualize_gravcenter(fastled_helper::leds);
#endif
      break;
    case MODE_GRAVICENTRIC:
#ifdef DEF_GRAVICENTRIC
      this->visualize_gravcentric(fastled_helper::leds);
#endif
      break;
    case MODE_GRAVIMETER:
#ifdef DEF_GRAVIMETER
      this->visualize_gravmeter(fastled_helper::leds);
#endif
      break;
    case MODE_PIXELS:
#ifdef DEF_PIXELS
      this->visualize_pixels(fastled_helper::leds);
#endif
      break;
    case MODE_JUNGLES:
#ifdef DEF_JUNGLES
      this->visualize_juggles(fastled_helper::leds);
#endif
      break;
    case MODE_MIDNOISE:
#ifdef DEF_MIDNOISE
      this->visualize_midnoise(fastled_helper::leds);
#endif
      break;
  }

  for (int i = 0; i < p_it->size(); i++) {
    (*p_it)[i] = Color(fastled_helper::leds[i].r, fastled_helper::leds[i].g, fastled_helper::leds[i].b);
  }

  this->start_effect_ = false;
  delay_microseconds_safe(1);
}

// *****************************************************************************************************************************************************************
#if defined(DEF_GRAV) || defined(DEF_GRAVICENTER) || defined (DEF_GRAVICENTRIC) || defined(DEF_GRAVIMETER)

#define MAX_FREQUENCY   11025    // sample frequency / 2 (as per Nyquist criterion)
#define MAX_FREQ_LOG10  4.04238f // log10(MAX_FREQUENCY)

// Gravity struct requited for GRAV* effects
typedef struct Gravity {
  int    topLED;
  int    gravityCounter;
} gravity;

// Gravcenter effects By Andrew Tuline.
// Gravcenter base function for Gravcenter (0), Gravcentric (1), Gravimeter (2), Gravfreq (3)
void MusicLeds::mode_gravcenter_base(unsigned mode, CRGB *physic_leds) {

  const unsigned dataSize = sizeof(gravity);
  if (!this->allocateData(dataSize)) {
    return; // allocation failed
  }
  Gravity* gravcen = reinterpret_cast<Gravity*>(this->data);

  if (mode == 1) {        // Gravcentric
    fastled_helper::fade_out(physic_leds, this->leds_num, 253, this->back_color);
  } else if(mode == 2) {  // Gravimeter
    fastled_helper::fade_out(physic_leds, this->leds_num, 249, this->back_color);
  } else if(mode == 3) {  // Gravfreq
    fastled_helper::fade_out(physic_leds, this->leds_num, 250, this->back_color);
  } else {                // Gravcenter
    fastled_helper::fade_out(physic_leds, this->leds_num, 251, this->back_color);
  }

  float mySampleAvg;
  int tempsamp;
  float segmentSampleAvg = this->volumeSmth * (float)this->variant / 255.0f;

  if(mode == 2) {         // Gravimeter
    segmentSampleAvg *= 0.25; // divide by 4, to compensate for later "sensitivity" upscaling
    // map to pixels availeable in current segment
    mySampleAvg = remap(segmentSampleAvg * 2.0f, 0.0f, 64.0f, 0.0f, (float)(this->leds_num - 1));
    tempsamp = constrain(mySampleAvg, 0, this->leds_num - 1);  // Keep the sample from overflowing.
  }
  else {                  // Gravcenter or Gravcentric or Gravfreq
    segmentSampleAvg *= 0.125f; // divide by 8, to compensate for later "sensitivity" upscaling
    // map to pixels availeable in current segment
    mySampleAvg = remap(segmentSampleAvg * 2.0f, 0.0f, 32.0f, 0.0f, (float)this->leds_num / 2.0f);
    tempsamp = constrain(mySampleAvg, 0, this->leds_num / 2);  // Keep the sample from overflowing.
  }

  uint8_t gravity = 8 - this->speed / 32;
  int offset = (mode == 2) ? 0 : 1;  
  if (tempsamp >= gravcen->topLED) gravcen->topLED = tempsamp-offset;
  else if (gravcen->gravityCounter % gravity == 0) gravcen->topLED--;
  
  if(mode == 1) {         // Gravcentric
    for (int i = 0; i < tempsamp; i++) {
      uint8_t index = segmentSampleAvg * 24 + millis() / 200;
      physic_leds[i + this->leds_num / 2] = fastled_helper::color_from_palette(index, this->main_color);
      physic_leds[this->leds_num / 2 - 1 - i] = fastled_helper::color_from_palette(index, this->main_color);
    }
    if (gravcen->topLED >= 0) {
      physic_leds[gravcen->topLED + this->leds_num / 2] = CRGB::Gray;
      physic_leds[this->leds_num / 2 - 1 - gravcen->topLED] = CRGB::Gray;
    }
  }
  else if(mode == 2) {    // Gravimeter
    for (int i=0; i<tempsamp; i++) {
      uint8_t index = fastled_helper::perlin8(i*segmentSampleAvg+millis(), 5000+i*segmentSampleAvg);
      physic_leds[i] = fastled_helper::color_blend(this->back_color, fastled_helper::color_from_palette(index, this->main_color), segmentSampleAvg * 8);
    }
    if (gravcen->topLED > 0) {
      physic_leds[gravcen->topLED] = fastled_helper::color_from_palette(millis(), this->main_color);
    }
  }
  else if(mode == 3) {    // Gravfreq
    for (int i=0; i<tempsamp; i++) {
      float fft_MajorPeak = FFT_MajorPeak; // used in mode 3: Gravfreq
      if (fft_MajorPeak < 1) {
        fft_MajorPeak = 1;
      }
      uint8_t index = (log10f(fft_MajorPeak) - (MAX_FREQ_LOG10 - 1.78f)) * 255;
      physic_leds[i + this->leds_num / 2] = fastled_helper::color_from_palette(index, this->main_color);
      physic_leds[this->leds_num / 2 - i - 1] = fastled_helper::color_from_palette(index, this->main_color);
    }
    if (gravcen->topLED >= 0) {
      physic_leds[gravcen->topLED + this->leds_num / 2] = CRGB::Gray;
      physic_leds[this->leds_num / 2 - 1 - gravcen->topLED] = CRGB::Gray;
    }
  }
  else {                  // Gravcenter
    for (int i=0; i<tempsamp; i++) {
      uint8_t index = fastled_helper::perlin8(i * segmentSampleAvg + millis(), 5000 + i * segmentSampleAvg);
      physic_leds[i + this->leds_num / 2] = fastled_helper::color_blend(this->back_color, fastled_helper::color_from_palette(index, this->main_color), segmentSampleAvg * 8);
      physic_leds[this->leds_num / 2 - i - 1] = fastled_helper::color_blend(this->back_color, fastled_helper::color_from_palette(index, this->main_color), segmentSampleAvg * 8);
    }
    if (gravcen->topLED >= 0) {
      physic_leds[gravcen->topLED + this->leds_num / 2] = fastled_helper::color_from_palette(millis(), this->main_color);
      physic_leds[this->leds_num / 2 - 1 - gravcen->topLED] = fastled_helper::color_from_palette(millis(), this->main_color);
    }
  } 
  gravcen->gravityCounter = (gravcen->gravityCounter + 1) % gravity;
  if (millis() % 50 == 0) ESP_LOGCONFIG(TAG, "this->volumeSmth %f", this->volumeSmth);
}
#endif

// *****************************************************************************************************************************************************************
#ifdef DEF_GRAVICENTER
void MusicLeds::visualize_gravcenter(CRGB *physic_leds)  // Gravcenter. By Andrew Tuline.
{
  mode_gravcenter_base(0, physic_leds);
}  // visualize_gravcenter()
#endif

// *****************************************************************************************************************************************************************
#ifdef DEF_GRAVICENTRIC
void MusicLeds::visualize_gravcentric(CRGB *physic_leds)  // Gravcentric. By Andrew Tuline.
{
  mode_gravcenter_base(1, physic_leds);
}  // visualize_gravcentric
#endif

// *****************************************************************************************************************************************************************
#ifdef DEF_GRAVIMETER
void MusicLeds::visualize_gravmeter(CRGB *physic_leds)    // Gravmeter. By Andrew Tuline.
{
  mode_gravcenter_base(2, physic_leds);
}  // visualize_gravcentric
#endif

// *****************************************************************************************************************************************************************
#ifdef DEF_GRAV
void MusicLeds::visualize_gravfreq(CRGB *physic_leds)     // Gravfreq. By Andrew Tuline.
{
  return mode_gravcenter_base(3, physic_leds);
}  // visualize_gravfreq
#endif

// *****************************************************************************************************************************************************************
#ifdef DEF_PIXELS
void MusicLeds::visualize_pixels(CRGB *physic_leds)       // Pixels. By Andrew Tuline.
{
  if (!this->allocateData(32 * sizeof(uint8_t))) {
    return; //allocation failed
  }
  uint8_t *myVals = reinterpret_cast<uint8_t*>(this->data);
  
  myVals[millis() % 32] = this->volumeSmth;  // filling values semi randomly

  fastled_helper::fade_out(physic_leds, this->leds_num, 64 + (this->speed >> 1), this->back_color);

  for (int i = 0; i < (int) this->variant / 8; i++) {
    uint16_t segLoc = random16(this->leds_num);  // 16 bit for larger strands of LED's.
    physic_leds[segLoc] = fastled_helper::color_blend(this->back_color, fastled_helper::color_from_palette(myVals[i % 32] + i * 4, this->main_color), this->volumeSmth);
  }
}  // visualize_pixels()
#endif

// *****************************************************************************************************************************************************************
#ifdef DEF_JUNGLES
void MusicLeds::visualize_juggles(CRGB *physic_leds)      // Juggles. By Andrew Tuline.
{
  fastled_helper::fade_out(physic_leds, this->leds_num, 224, this->back_color);

  int my_sampleAgc = fmax(fmin(this->volumeSmth, 255.0), 0);

  for (int i = 0; i < (int) this->variant / 32 + 1; i++) {
    physic_leds[beatsin16((int) this->speed / 4 + i * 2, 0, this->leds_num - 1)] = fastled_helper::color_blend(
        this->back_color, fastled_helper::color_from_palette(millis() / 4 + i * 2, this->main_color), my_sampleAgc);
  }
}  // visualize_juggles()
#endif

// *****************************************************************************************************************************************************************
#ifdef DEF_MIDNOISE
void MusicLeds::visualize_midnoise(CRGB *physic_leds)     // Midnoise. By Andrew Tuline.
{
  static int x = 0;
  static int y = 0;

  // Same as two fade-out runs
  fastled_helper::fade_out(physic_leds, this->leds_num, (int) this->speed * (int) this->speed / 255, this->back_color);

  float tmpSound = this->volumeSmth;
  float tmpSound2 = tmpSound * (float) this->variant / 256.0;  // Too sensitive.
  tmpSound2 *= (float) this->variant / 128.0;                  // Reduce sensitity/length.

  unsigned maxLen = remap((float) tmpSound2, 0.0f, 127.0f, 0.0f, (float) this->leds_num / 2);
  if (maxLen > this->leds_num / 2)
    maxLen = this->leds_num / 2;

  for (int i = (this->leds_num / 2 - maxLen); i < (this->leds_num / 2 + maxLen); i++) {
    // Get a value from the noise function. I'm using both x and y axis.
    uint8_t index = fastled_helper::perlin8(i * tmpSound + x, y + i * tmpSound);
    physic_leds[i] = fastled_helper::color_from_palette(index, this->main_color);
  }

  x = x + beatsin8(5, 0, 10);
  y = y + beatsin8(4, 0, 10);
}  // visualize_midnoise()
#endif

}  // namespace music_leds
}  // namespace esphome
