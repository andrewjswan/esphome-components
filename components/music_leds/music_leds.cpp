#include "esphome.h"
#include "audio_reactive.h"
#include "esphome/components/fastled_helper/fastled_helper.h"

namespace esphome {
namespace music_leds {

void MusicLeds::setup() {
  // Define the FFT Task and lock it to core 0
  xTaskCreatePinnedToCore(FFTcode,            // Function to implement the task
                          "FFT",              // Name of the task
                          5000,               // Stack size in words
                          (void *) this,      // Task input parameter
                          1,                  // Priority of the task
                          &FFT_Task,          // Task handle
                          this->task_core_);  // Core where the task should run

  this->microphone_->start();
}

void MusicLeds::dump_config() {
  ESP_LOGCONFIG(TAG, "Music Leds version: %s", MUSIC_LEDS_VERSION);
  ESP_LOGCONFIG(TAG, "           Samples: %dbit", BITS_PER_SAMPLE);
  ESP_LOGCONFIG(TAG, "       Sample rate: %d", SAMPLE_RATE);
  ESP_LOGCONFIG(TAG, "      Input filter: %d", INPUT_FILTER);
#ifdef I2S_GRAB_ADC1_COMPLETELY
  ESP_LOGCONFIG(TAG, "          Grab ADC: Completely (experimental)");
#endif
  ESP_LOGCONFIG(TAG, "         Task Core: %u", this->task_core_);
}  // dump_config()

void MusicLeds::on_shutdown() {
  this->microphone_->stop();

  vTaskDelete(FFT_Task);  // OTA: Avoid crash due to angry watchdog

  fastled_helper::FreeLeds();
}

void MusicLeds::set_speed(int index) { speed = index; }

void MusicLeds::set_variant(int index) { variant = index; }

void MusicLeds::ShowFrame(PLAYMODE CurrentMode, esphome::Color current_color, light::AddressableLight *p_it) {
  if (!this->microphone_is_running()) {
    return;
  }

  fastled_helper::InitLeds(p_it->size());

  static unsigned long lastUMRun = millis();  // time of last filter run

  int userloopDelay = int(millis() - lastUMRun);
  if (lastUMRun == 0)
    userloopDelay = 0;  // startup - don't have valid data from last run.

  unsigned long t_now = millis();
  lastUMRun = t_now;
  if (soundAgc > AGC_NUM_PRESETS)
    soundAgc = 0;  // make sure that AGC preset is valid (to avoid array bounds violation)

  if (userloopDelay < 2)
    userloopDelay = 0;  // minor glitch, no problem
  if (userloopDelay > 150)
    userloopDelay = 150;  // limit number of filter re-runs
  do {
    getSample();                    // Sample the microphone
    agcAvg(t_now - userloopDelay);  // Calculated the PI adjusted value as sampleAvg
    userloopDelay -= 2;             // advance "simulated time" by 2ms
  } while (userloopDelay > 0);

  myVals[millis() % 32] = sampleAgc;

  // limit dynamics (experimental)
  limitSampleDynamics();

  leds_num = p_it->size();

  main_color = CRGB(current_color.r, current_color.g, current_color.b);
  if ((int) fastled_helper::current_palette == 0) {
    back_color =
        CRGB(current_color.r / 100 * 5, current_color.g / 100 * 5, current_color.b / 100 * 5);  // 5% from main color
  } else {
    back_color = CRGB::Black;
  }

  switch (CurrentMode) {
    case MODE_BINMAP:
#ifdef DEF_BINMAP
      this->visualize_binmap(fastled_helper::leds);
#endif
      break;
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

  delay(1);
}

void MusicLeds::getSamples(float *buffer) {
  if (!this->microphone_is_running()) {
    return;
  }

  // Counter variable to check if we actually got enough data
  size_t bytes_read = 0;
  // Intermediary sample storage
  I2S_datatype newSamples[I2S_buffer_size];
  // Reset ADC broken samples counter
  _broken_samples_counter = 0;

  // Get fresh samples
  bytes_read = this->microphone_->read_(newSamples, sizeof(newSamples), 2 * pdMS_TO_TICKS(READ_DURATION_MS));
  bytes_read = bytes_read * BITS_PER_SAMPLE / 16;

  // For correct operation, we need to read exactly sizeof(samples) bytes from i2s
  if (bytes_read != sizeof(newSamples)) {
    ESP_LOGE("ASR", "AS: Failed to get enough samples: wanted: %d read: %d", sizeof(newSamples), bytes_read);
    return;
  }

  // Store samples in sample buffer and update DC offset
  for (int i = 0; i < samplesFFT; i++) {
    if (_mask == 0x0FFF)  // mask = 0x0FFF means we are in I2SAdcSource
    {
      I2S_unsigned_datatype rawData =
          *reinterpret_cast<I2S_unsigned_datatype *>(newSamples + i);  // C++ acrobatics to get sample as "unsigned"
      I2S_datatype sampleNoFilter = this->decodeADCsample(rawData);
      if (_broken_samples_counter >=
          samplesFFT - 1)  // kill-switch: ADC sample correction off when all samples in a batch were "broken"
      {
        _myADCchannel = 0x0F;
        ESP_LOGE("ASR", "AS: Too many broken audio samples from ADC - sample correction switched off.");
      }
      newSamples[i] = (3 * sampleNoFilter + _lastADCsample) / 4;  // apply low-pass filter (2-tap FIR)
      // newSamples[i] = (sampleNoFilter + lastADCsample) / 2;    // apply stronger low-pass filter (2-tap FIR)
      _lastADCsample = sampleNoFilter;  // update ADC last sample
    }

    // pre-shift samples down to 16bit
    float currSample = 0.0;
    if (_shift > 0)
      currSample = (float) (newSamples[i] >> _shift);
    else {
      if (_shift < 0)
        currSample =
            (float) (newSamples[i]
                     << (-_shift));  // need to "pump up" 12bit ADC to full 16bit as delivered by other digital mics
      else
        currSample = (float) newSamples[i];
    }
    buffer[i] = currSample;     // store sample
    buffer[i] *= _sampleScale;  // scale sample
  }
}

// function to handle ADC samples
I2S_datatype MusicLeds::decodeADCsample(I2S_unsigned_datatype rawData) {
  rawData = rawData & 0xFFFF;                        // input is already in 16bit, just mask off possible junk
  I2S_datatype lastGoodSample = _lastADCsample * 4;  // 10bit-> 12bit

  // decode ADC sample
  uint16_t the_channel = (rawData >> 12) & 0x000F;      // upper 4 bit = ADC channel
  uint16_t the_sample = rawData & 0x0FFF;               // lower 12bit -> ADC sample (unsigned)
  I2S_datatype finalSample = (int(the_sample) - 2048);  // convert to signed (centered at 0);

  // fix bad samples
  if ((the_channel != _myADCchannel) && (_myADCchannel != 0x0F)) {  // 0x0F means "don't know what my channel is"
    finalSample = lastGoodSample;                                   // replace with the last good ADC sample
    _broken_samples_counter++;
  }

  finalSample = finalSample / 4;  // mimic old analog driver behaviour (12bit -> 10bit)
  return (finalSample);
}

// FFT main code
void MusicLeds::FFTcode(void *parameter) {
  MusicLeds *this_task = (MusicLeds *) parameter;

  // see https://www.freertos.org/vtaskdelayuntil.html
  // constexpr TickType_t xFrequency = FFT_MIN_CYCLE * portTICK_PERIOD_MS;
  constexpr TickType_t xFrequency_2 = (FFT_MIN_CYCLE * portTICK_PERIOD_MS) / 2;

  for (;;) {
    TickType_t xLastWakeTime = xTaskGetTickCount();

    // ::delay(1);              // DO NOT DELETE THIS LINE! It is needed to give the IDLE(0) task enough time and to
    // keep the watchdog happy.
    delay_microseconds_safe(1);

    if (this_task == nullptr) {
      vTaskDelayUntil(&xLastWakeTime, xFrequency_2);  // release CPU
      continue;
    }

    // Only run the FFT computing code if we're not in "realime mode" or in Receive mode
    if (!this_task->microphone_is_running()) {
      vTaskDelayUntil(&xLastWakeTime, xFrequency_2);  // release CPU
      continue;
    }

#if !defined(I2S_GRAB_ADC1_COMPLETELY)
    if (dmType > 0)  // the "delay trick" does not help for analog, because I2S ADC is disabled outside of getSamples()
#endif
      vTaskDelayUntil(&xLastWakeTime, xFrequency_2);  // release CPU, and give I2S some time to fill its buffers. Might
                                                      // not work well with ADC analog sources.

    this_task->getSamples(vReal);

    xLastWakeTime = xTaskGetTickCount();  // update "last unblocked time" for vTaskDelay

#ifdef INPUT_FILTER
    // input filters applied before FFT
    if (INPUT_FILTER > 0) {
      // filter parameter - we use constexpr as it does not need any RAM (evaluted at compile time)
      // value = 1 - exp(-2*PI * FFilter / FSample);  // FFilter: filter cutoff frequency; FSample: sampling frequency
      constexpr float filter30Hz = 0.01823938f;   // rumbling = 10-25hz
      constexpr float filter70Hz = 0.04204211f;   // mains hum = 50-60hz
      constexpr float filter120Hz = 0.07098564f;  // bad microphones deliver noise below 120Hz
      constexpr float filter185Hz = 0.10730882f;  // environmental noise is strongest below 180hz:
                                                  // wind, engine noise, ...
      switch (INPUT_FILTER) {
        case 1:
          runMicFilter(samplesFFT, vReal);
          break;  // PDM microphone bandpass
        case 2:
          runHighFilter12db(filter30Hz, samplesFFT, vReal);
          break;  // Rejects rumbling noise
        case 3:
          runMicSmoothing_v2(samplesFFT, vReal);             // Slightly reduce high frequency noise and artefacts
          runHighFilter12db(filter70Hz, samplesFFT, vReal);  // Rejects rumbling + mains hum
          break;
        case 4:
          runMicSmoothing_v2(samplesFFT, vReal);             // Slightly reduce high frequency noise and artefacts
          runHighFilter6db(filter120Hz, samplesFFT, vReal);  // Rejects everything below 110Hz
          break;
        case 5:
          runMicSmoothing(samplesFFT, vReal);                // Reduce high frequency noise and artefacts
          runHighFilter6db(filter185Hz, samplesFFT, vReal);  // Reject low frequency noise
          break;
      }
    }
#endif

    // find highest sample in the batch
    const int halfSamplesFFT = samplesFFT / 2;  // samplesFFT divided by 2
    float maxSample1 = 0.0;                     // max sample from first half of FFT batch
    float maxSample2 = 0.0;                     // max sample from second half of FFT batch
    for (int i = 0; i < samplesFFT; i++) {
      // set imaginary parts to 0
      vImag[i] = 0;
      // pick our  our current mic sample - we take the max value from all samples that go into FFT
      if ((vReal[i] <= (INT16_MAX - 1024)) &&
          (vReal[i] >= (INT16_MIN + 1024)))  // skip extreme values - normally these are artefacts
      {
        if (i <= halfSamplesFFT) {
          if (fabsf(vReal[i]) > maxSample1)
            maxSample1 = fabsf(vReal[i]);
        } else {
          if (fabsf(vReal[i]) > maxSample2)
            maxSample2 = fabsf(vReal[i]);
        }
      }
    }

    // release first sample to volume reactive effects
    micDataSm = (uint16_t) maxSample1;
    micDataReal = maxSample1;

    FFT.dcRemoval();  // remove DC offset
    FFT.windowing(FFTWindow::Blackman_Harris,
                  FFTDirection::Forward);  // Weigh data using "Blackman- Harris" window - sharp peaks due to excellent
                                           // sideband rejection
    FFT.compute(FFTDirection::Forward);    // Compute FFT
    FFT.complexToMagnitude();              // Compute magnitudes

    //
    // vReal[3 .. 255] contain useful data, each a 20Hz interval (60Hz - 5120Hz).
    // There could be interesting data at bins 0 to 2, but there are too many artifacts.
    //

    FFT.majorPeak(&FFT_MajorPeak, &FFT_Magnitude);            // let the effects know which freq was most dominant
    FFT_MajorPeak = constrain(FFT_MajorPeak, 1.0f, 5120.0f);  // restrict value to range expected by effects
    FFT_Magnitude = fabsf(FFT_Magnitude);

    for (int i = 0; i < samplesFFT; i++) {  // Values for bins 0 and 1 are WAY too large. Might as well start at 3.
      float t = 0.0;
      t = fabsf(vReal[i]);  // just to be sure - values in fft bins should be positive any way
      t = t / 16.0f;        // Reduce magnitude. Want end result to be linear and ~4096 max.
      fftBin[i] = t;
    }  // for()

    /* This FFT post processing is a DIY endeavour. What we really need is someone with sound engineering expertise to
     * do a great job here AND most importantly, that the animations look GREAT as a result.
     *
     * Andrew's updated mapping of 256 bins down to the 16 result bins with Sample Freq = 10240, samplesFFT = 512 and
     * some overlap. Based on testing, the lowest/Start frequency is 60 Hz (with bin 3) and a highest/End frequency of
     * 5120 Hz in bin 255. Now, Take the 60Hz and multiply by 1.320367784 to get the next frequency and so on until the
     * end. Then detetermine the bins. End frequency = Start frequency * multiplier ^ 16 Multiplier = (End frequency/
     * Start frequency) ^ 1/16 Multiplier = 1.320367784
     */
    //                                            Range
    fftCalc[0] = (fftAdd(3, 4)) / 2;        // 60 - 100
    fftCalc[1] = (fftAdd(4, 5)) / 2;        // 80 - 120
    fftCalc[2] = (fftAdd(5, 7)) / 3;        // 100 - 160
    fftCalc[3] = (fftAdd(7, 9)) / 3;        // 140 - 200
    fftCalc[4] = (fftAdd(9, 12)) / 4;       // 180 - 260
    fftCalc[5] = (fftAdd(12, 16)) / 5;      // 240 - 340
    fftCalc[6] = (fftAdd(16, 21)) / 6;      // 320 - 440
    fftCalc[7] = (fftAdd(21, 28)) / 8;      // 420 - 600
    fftCalc[8] = (fftAdd(28, 37)) / 10;     // 580 - 760
    fftCalc[9] = (fftAdd(37, 48)) / 12;     // 740 - 980
    fftCalc[10] = (fftAdd(48, 64)) / 17;    // 960 - 1300
    fftCalc[11] = (fftAdd(64, 84)) / 21;    // 1280 - 1700
    fftCalc[12] = (fftAdd(84, 111)) / 28;   // 1680 - 2240
    fftCalc[13] = (fftAdd(111, 147)) / 37;  // 2220 - 2960
    fftCalc[14] = (fftAdd(147, 194)) / 48;  // 2940 - 3900
    fftCalc[15] = (fftAdd(194, 255)) / 62;  // 3880 - 5120

    //   Noise supression of fftCalc bins using soundSquelch adjustment for different input types.
    for (int i = 0; i < 16; i++) {
      fftCalc[i] = fftCalc[i] - (float) soundSquelch * (float) linearNoise[i] / 4.0 <= 0 ? 0 : fftCalc[i];
    }

    // Adjustment for frequency curves.
    for (int i = 0; i < 16; i++) {
      fftCalc[i] = fftCalc[i] * fftResultPink[i];
    }

    // Manual linear adjustment of gain using sampleGain adjustment for different input types.
    for (int i = 0; i < 16; i++) {
      if (soundAgc)
        fftCalc[i] = fftCalc[i] * multAgc;
      else
        fftCalc[i] = fftCalc[i] * (float) sampleGain / 40.0 * (float) inputLevel / 128.0 +
                     (float) fftCalc[i] / 16.0;  // with inputLevel adjustment
    }

    // Now, let's dump it all into fftResult. Need to do this, otherwise other routines might grab fftResult values
    // prematurely.
    for (int i = 0; i < 16; i++) {
      // fftResult[i] = (int)fftCalc[i];
      fftResult[i] = constrain((int) fftCalc[i], 0, 254);  // question: why do we constrain values to 8bit here ???
      fftAvg[i] = (float) fftResult[i] * 0.05 + (1.0 - 0.05) * fftAvg[i];
    }

#if !defined(I2S_GRAB_ADC1_COMPLETELY)
    if (dmType > 0)  // the "delay trick" does not help for analog
#endif
      vTaskDelayUntil(&xLastWakeTime, xFrequency_2);  // release CPU, by waiting until FFT_MIN_CYCLE is over

    // release second sample to volume reactive effects.
    // Releasing a second sample now effectively doubles the "sample rate"
    micDataSm = (uint16_t) maxSample2;
    micDataReal = maxSample2;
  }  // for(;;)
}  // FFTcode()

// *****************************************************************************************************************************************************************
#ifdef DEF_GRAV
void MusicLeds::visualize_gravfreq(CRGB *physic_leds)  // Gravfreq. By Andrew Tuline.
{
  fastled_helper::fade_out(physic_leds, leds_num, 240, back_color);

  float tmpSound = (soundAgc) ? sampleAgc : sampleAvg;
  float segmentSampleAvg = tmpSound * (float) variant / 255.0;
  segmentSampleAvg *= 0.125;  // divide by 8,  to compensate for later "sensitivty" upscaling

  float mySampleAvg = remap((float) segmentSampleAvg * 2.0, 0.0, 32.0, 0.0,
                            (float) leds_num / 2.0);       // Map to pixels availeable in current segment
  int tempsamp = constrain(mySampleAvg, 0, leds_num / 2);  // Keep the sample from overflowing.
  uint8_t gravity = 8 - (int) speed / 32;

  for (int i = 0; i < tempsamp; i++) {
    int index = (log10f(FFT_MajorPeak) - (3.71 - 1.78)) * 255;
    if (index < 0)
      index = 0;
    index = scale8(index, 240);  // Cut off blend at palette "end"

    physic_leds[i + leds_num / 2] = fastled_helper::color_from_palette(index, main_color);
    physic_leds[leds_num / 2 - i - 1] = fastled_helper::color_from_palette(index, main_color);
  }

  if (tempsamp >= topLED) {
    topLED = tempsamp - 1;
  } else if (gravityCounter % gravity == 0) {
    topLED--;
  }

  if (topLED >= 0) {
    physic_leds[topLED + leds_num / 2] = CRGB::Gray;
    physic_leds[leds_num / 2 - 1 - topLED] = CRGB::Gray;
  }
  gravityCounter = (gravityCounter + 1) % gravity;
}  // visualize_gravfreq
#endif

// *****************************************************************************************************************************************************************
#ifdef DEF_GRAVICENTER
void MusicLeds::visualize_gravcenter(CRGB *physic_leds)  // Gravcenter. By Andrew Tuline.
{
  fastled_helper::fade_out(physic_leds, leds_num, 240, back_color);

  float tmpSound = (soundAgc) ? sampleAgc : sampleAvg;
  float segmentSampleAvg = tmpSound * (float) variant / 255.0;
  segmentSampleAvg *= 0.125;  // divide by 8, to compensate for later "sensitivty" upscaling

  float mySampleAvg = remap((float) segmentSampleAvg * 2.0, 0.0, 32.0, 0.0,
                            (float) leds_num / 2.0);       // map to pixels availeable in current segment
  int tempsamp = constrain(mySampleAvg, 0, leds_num / 2);  // Keep the sample from overflowing.
  uint8_t gravity = 8 - speed / 32;

  for (int i = 0; i < tempsamp; i++) {
    uint8_t index = inoise8(i * segmentSampleAvg + millis(), 5000 + i * segmentSampleAvg);
    physic_leds[i + leds_num / 2] = fastled_helper::color_blend(
        back_color, fastled_helper::color_from_palette(index, main_color), segmentSampleAvg * 8);
    physic_leds[leds_num / 2 - i - 1] = fastled_helper::color_blend(
        back_color, fastled_helper::color_from_palette(index, main_color), segmentSampleAvg * 8);
  }

  if (tempsamp >= topLED)
    topLED = tempsamp - 1;
  else if (gravityCounter % gravity == 0)
    topLED--;

  if (topLED >= 0) {
    physic_leds[topLED + leds_num / 2] = fastled_helper::color_from_palette(millis(), main_color);
    physic_leds[leds_num / 2 - 1 - topLED] = fastled_helper::color_from_palette(millis(), main_color);
  }
  gravityCounter = (gravityCounter + 1) % gravity;
}  // visualize_gravcenter()
#endif

// *****************************************************************************************************************************************************************
#ifdef DEF_GRAVICENTRIC
void MusicLeds::visualize_gravcentric(CRGB *physic_leds)  // Gravcentric. By Andrew Tuline.
{
  fastled_helper::fade_out(physic_leds, leds_num, 226, back_color);

  float tmpSound = (soundAgc) ? sampleAgc : sampleAvg;
  float segmentSampleAvg = tmpSound * (float) variant / 255.0;
  segmentSampleAvg *= 0.125;  // divide by 8, to compensate for later "sensitivty" upscaling

  float mySampleAvg = remap((float) segmentSampleAvg * 2.0, 0.0, 32.0, 0.0,
                            (float) leds_num / 2.0);       // map to pixels availeable in current segment
  int tempsamp = constrain(mySampleAvg, 0, leds_num / 2);  // Keep the sample from overflowing.
  uint8_t gravity = 8 - speed / 32;

  for (int i = 0; i < tempsamp; i++) {
    uint8_t index = segmentSampleAvg * 24 + millis() / 200;
    physic_leds[i + leds_num / 2] = fastled_helper::color_from_palette(index, main_color);
    physic_leds[leds_num / 2 - 1 - i] = fastled_helper::color_from_palette(index, main_color);
  }

  if (tempsamp >= topLED)
    topLED = tempsamp - 1;
  else if (gravityCounter % gravity == 0)
    topLED--;

  if (topLED >= 0) {
    physic_leds[topLED + leds_num / 2] = CRGB::Gray;
    physic_leds[leds_num / 2 - 1 - topLED] = CRGB::Gray;
  }
  gravityCounter = (gravityCounter + 1) % gravity;
}  // visualize_gravcentric
#endif

// *****************************************************************************************************************************************************************
#ifdef DEF_BINMAP
void MusicLeds::visualize_binmap(
    CRGB *physic_leds)  // Binmap. Scale raw fftBin[] values to SEGLEN. Shows just how noisy those bins are.
{
#define FIRSTBIN 3   // The first 3 bins are garbage.
#define LASTBIN 255  // Don't use the highest bins, as they're (almost) a mirror of the first 256.

  float maxVal = 512;  // Kind of a guess as to the maximum output value per combined logarithmic bins.

  float binScale = (((float) sampleGain / 40.0) + 1.0 / 16) * ((float) variant / 128.0);  // non-AGC gain multiplier
  if (soundAgc)
    binScale = multAgc;  // AGC gain
  if (sampleAvg < 1)
    binScale = 0.001;  // silentium!

  for (int i = 0; i < leds_num; i++) {
    uint16_t startBin =
        FIRSTBIN + i * (LASTBIN - FIRSTBIN) / leds_num;  // This is the START bin for this particular pixel.
    uint16_t endBin =
        FIRSTBIN + (i + 1) * (LASTBIN - FIRSTBIN) / leds_num;  // This is the END bin for this particular pixel.
    if (endBin > startBin)
      endBin--;  // avoid overlapping

    float sumBin = 0;

    for (int j = startBin; j <= endBin; j++) {
      sumBin += (fftBin[j] < soundSquelch * 1.75)
                    ? 0
                    : fftBin[j];  // We need some sound temporary squelch for fftBin, because we didn't do it for the
                                  // raw bins in audio_reactive.h
    }

    sumBin = sumBin / (endBin - startBin + 1);            // Normalize it.
    sumBin = sumBin * (i + 5) / (endBin - startBin + 5);  // Disgusting frequency adjustment calculation. Lows were too
                                                          // bright. Am open to quick 'n dirty alternatives.

    sumBin = sumBin * 8;  // Need to use the 'log' version for this. Why " * 8" ??
    sumBin *= binScale;   // apply gain

    if (sumBin > maxVal)
      sumBin = maxVal;  // Make sure our bin isn't higher than the max . . which we capped earlier.

    uint8_t bright = constrain(remap((float) sumBin, (float) 0, (float) maxVal, (float) 0, (float) 255), 0,
                               255);  // Map the brightness in relation to maxVal and crunch to 8 bits.
    physic_leds[i] = fastled_helper::color_blend(
        back_color, fastled_helper::color_from_palette(i * 8 + millis() / 50, main_color),
        bright);  // 'i' is just an index in the palette. The FFT value, bright, is the intensity.
  }

}  // visualize_binmap
#endif

// *****************************************************************************************************************************************************************
#ifdef DEF_PIXELS
void MusicLeds::visualize_pixels(CRGB *physic_leds)  // Pixels. By Andrew Tuline.
{
  fastled_helper::fade_out(physic_leds, leds_num, (int) speed, back_color);

  for (int i = 0; i < (int) variant / 16; i++) {
    uint16_t segLoc = random(leds_num);  // 16 bit for larger strands of LED's.
    physic_leds[segLoc] = fastled_helper::color_blend(
        back_color, fastled_helper::color_from_palette(myVals[i % 32] + i * 4, main_color), sampleAgc);
  }
}  // visualize_pixels()
#endif

// *****************************************************************************************************************************************************************
#ifdef DEF_JUNGLES
void MusicLeds::visualize_juggles(CRGB *physic_leds)  // Juggles. By Andrew Tuline.
{
  fastled_helper::fade_out(physic_leds, leds_num, 224, back_color);

  int my_sampleAgc = fmax(fmin(sampleAgc, 255.0), 0);

  for (int i = 0; i < (int) variant / 32 + 1; i++) {
    physic_leds[beatsin16((int) speed / 4 + i * 2, 0, leds_num - 1)] = fastled_helper::color_blend(
        back_color, fastled_helper::color_from_palette(millis() / 4 + i * 2, main_color), my_sampleAgc);
  }
}  // visualize_juggles()
#endif

// *****************************************************************************************************************************************************************
#ifdef DEF_MIDNOISE
void MusicLeds::visualize_midnoise(CRGB *physic_leds)  // Midnoise. By Andrew Tuline.
{
  static int x = 0;
  static int y = 0;

  fastled_helper::fade_out(physic_leds, leds_num, (int) speed * (int) speed / 255,
                           back_color);  // Same as two fade-out runs

  float tmpSound = (soundAgc) ? sampleAgc : sampleAvg;
  float tmpSound2 = tmpSound * (float) variant / 256.0;  // Too sensitive.
  tmpSound2 *= (float) variant / 128.0;                  // Reduce sensitity/length.

  int maxLen = remap((float) tmpSound2, (float) 0, (float) 127, (float) 0, (float) leds_num / 2);
  if (maxLen > leds_num / 2)
    maxLen = leds_num / 2;

  for (int i = (leds_num / 2 - maxLen); i < (leds_num / 2 + maxLen); i++) {
    uint8_t index = inoise8(i * tmpSound + x,
                            y + i * tmpSound);  // Get a value from the noise function. I'm using both x and y axis.
    physic_leds[i] = fastled_helper::color_from_palette(index, main_color);
  }

  x = x + beatsin8(5, 0, 10);
  y = y + beatsin8(4, 0, 10);
}  // visualize_midnoise()
#endif

}  // namespace music_leds
}  // namespace esphome
