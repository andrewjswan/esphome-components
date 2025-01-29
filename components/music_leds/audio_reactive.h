#pragma once

// WARNING Sound reactive variables that are used by the animations or other asynchronous
// routines must NOT have interim values, but only updated in a single calculation.
// These are:
//
// sample     sampleAvg     sampleAgc       samplePeak    myVals[]
//
// fftBin[]   fftResult[]   FFT_MajorPeak   FFT_Magnitude
//
// Otherwise, the animations may asynchronously read interim values of these variables.

#include <driver/i2s.h>
#include "arduinoFFT.h"

namespace esphome {
namespace music_leds {

#undef sqrt
#undef sqrt_internal

#define SOUND_REACTIVE

#define WLED_GLOBAL static
#define _INIT(x) = x

// WLEDSR
WLED_GLOBAL byte inputLevelS _INIT(255);          // WLEDSR default inputLevel
#if !defined(SR_SQUELCH)
WLED_GLOBAL byte soundSquelch _INIT(10);          // default squelch value for volume reactive routines
WLED_GLOBAL byte soundAgc _INIT(0);               // default Automagic gain control
#else
WLED_GLOBAL byte soundSquelch _INIT(SR_SQUELCH);  // default squelch value
WLED_GLOBAL byte soundAgc _INIT(2);               // squelch was provided - we can enable AGC by default
#endif
#if !defined(SR_GAIN)
WLED_GLOBAL byte sampleGain _INIT(30);            // default sample gain
#else
WLED_GLOBAL byte sampleGain _INIT(SR_GAIN);       // default sample gain
#endif

#ifndef DMENABLED  // aka DOUT
WLED_GLOBAL uint8_t dmType _INIT(0);
#else
WLED_GLOBAL uint8_t dmType _INIT(DMENABLED);
#endif

WLED_GLOBAL byte inputLevel _INIT(inputLevelS);  // inputLevel (set by sliderinputLevel)

// if true, sound processing (FFT, filters, AGC) will be suspended. "volatile" as its shared between tasks.
static volatile bool disableSoundProcessing = false;
// if > 0 , enables a bandpass filter 80Hz-8Khz to remove noise. Applies before FFT.
static unsigned useInputFilter = 0;

// hackers corner
// #define SOUND_DYNAMICS_LIMITER             // experimental: define to enable a dynamics limiter that avoids "sudden
// flashes" at onsets. Make some effects look more "smooth and fluent"

// constexpr int BLOCK_SIZE = 128;
// constexpr int SAMPLE_RATE = 10240;         // Base sample rate in Hz - standard.                 Physical sample time
// -> 50ms constexpr int SAMPLE_RATE = 20480;         // Base sample rate in Hz - 20Khz is experimental.    Physical
// sample time -> 25ms constexpr int SAMPLE_RATE = 22050;         // Base sample rate in Hz - 22Khz is a standard rate.
// Physical sample time -> 23ms

// minimum time before FFT task is repeated. Must be less than time for reading 512 samples at SAMPLE_RATE.
#define FFT_MIN_CYCLE 45

#define MAX_VOL 31  // (was 10) Reasonable value for constant volume for 'peak detector', as it won't always trigger
#define BIN_NUM 14  // (was 8)  Used to select the bin for FFT based beat detection. 14 = 280Hz

//
// AGC presets
//  Note: in C++, "const" implies "static" - no need to explicitly declare everything as "static const"
//
#define AGC_NUM_PRESETS 3  // AGC currently has 3 presets: normal, vivid, lazy

//                                                  Normal,      Vivid,    Lazy
const double agcSampleDecay[AGC_NUM_PRESETS] = {
    0.9994, 0.9985, 0.9997};  // decay factor for sampleMax, in case the current sample is below sampleMax

const float agcZoneLow[AGC_NUM_PRESETS] = {32, 28, 36};      // low volume emergency zone
const float agcZoneHigh[AGC_NUM_PRESETS] = {240, 240, 248};  // high volume emergency zone
const float agcZoneStop[AGC_NUM_PRESETS] = {336, 448, 304};  // disable AGC integrator if we get above this level

const float agcTarget0[AGC_NUM_PRESETS] = {112, 144, 164};  // first AGC setPoint -> between 40% and 65%
const float agcTarget0Up[AGC_NUM_PRESETS] = {88, 64, 116};  // setpoint switching value (a poor man's bang-bang)
const float agcTarget1[AGC_NUM_PRESETS] = {220, 224, 216};  // second AGC setPoint -> around 85%

const double agcFollowFast[AGC_NUM_PRESETS] = {1.0 / 192.0, 1.0 / 128.0,
                                               1.0 / 256.0};  // quickly follow setpoint - ~0.15 sec
const double agcFollowSlow[AGC_NUM_PRESETS] = {1.0 / 6144.0, 1.0 / 4096.0,
                                               1.0 / 8192.0};  // slowly follow setpoint  - ~2-15 secs

const double agcControlKp[AGC_NUM_PRESETS] = {0.6, 1.5, 0.65};  // AGC - PI control, proportional gain parameter
const double agcControlKi[AGC_NUM_PRESETS] = {1.7, 1.85, 1.2};  // AGC - PI control, integral gain parameter

const float agcSampleSmooth[AGC_NUM_PRESETS] = {
    1.0 / 12.0, 1.0 / 6.0,
    1.0 / 16.0};  // smoothing factor for sampleAgc (use rawSampleAgc if you want the non-smoothed value)
//
// AGC presets end
//

static double sampleMax = 0;  // Max sample over a few seconds. Needed for AGC controler.
static uint8_t myVals[32];    // Used to store a pile of samples because WLED frame rate and WLED sample rate are not
                              // synchronized. Frame rate is too low.
static int micIn = 0.0;  // Current sample starts with negative values and large values, which is why it's 16 bit signed
static float sampleReal = 0.0;  // "sample" as float, to provide bits that are lost otherwise. Needed for AGC.
static float tmpSample;         // An interim sample variable used for calculations.
static int rawSampleAgc = 0;    // Our AGC sample - raw
static float sampleAgc = 0.0;   // AGC sample, smoothed
static uint16_t micData;        // Analog input for FFT
static uint16_t micDataSm;      // Smoothed mic data, as it's a bit twitchy
static float micDataReal =
    0.0;  // future support - this one has the full 24bit MicIn data - lowest 8bit after decimal point
static double micLev = 0.0;     // Used to convert returned value to have '0' as minimum. A leveller
static float multAgc = 1.0;     // sample * multAgc = sampleAgc. Our multiplier
static float sampleAvg = 0;     // Smoothed Average
static float sampleAdj;         // Gain adjusted sample value
static uint8_t samplePeak = 0;  // Flag for peak. 0 = no peak, 1 = possible peak (often), 2=sure peak (every 1-2seconds)

static float expAdjF;          // Used for exponential filter.
static float weighting = 0.2;  // Exponential filter weighting. Will be adjustable in a future release.

static unsigned long timeOfPeak = 0;

// FFT Variables
constexpr uint16_t samplesFFT = 512;  // Samples in an FFT batch - This value MUST ALWAYS be a power of 2

static float FFT_MajorPeak = 1.0f;
static float FFT_Magnitude = 0.0001;

// These are the input and output vectors.  Input vectors receive computed results from FFT.
static float vReal[samplesFFT];
static float vImag[samplesFFT];
static float fftBin[samplesFFT];

// Try and normalize fftBin values to a max of 4096, so that 4096/16 = 256.
// Oh, and bins 0,1,2 are no good, so we'll zero them out.
static float fftCalc[16];
static int fftResult[16];  // Our calculated result table, which we feed to the animations.
static float fftAvg[16];

// Table of linearNoise results to be multiplied by soundSquelch in order to reduce squelch across fftResult bins.
static int linearNoise[16] = {34, 28, 26, 25, 20, 12, 9, 6, 4, 4, 3, 2, 2, 2, 2, 2};

// Table of multiplication factors so that we can even out the frequency response.
static float fftResultPink[16] = {1.70, 1.71, 1.73, 1.78, 1.68, 1.56, 1.55, 1.63,
                                  1.79, 1.62, 1.80, 2.06, 2.47, 3.35, 6.83, 9.55};

/* get current max sample ("published" by the I2S and FFT thread) and perform some sound processing */
static void getSample() {
  const int AGC_preset =
      (soundAgc > 0) ? (soundAgc - 1)
                     : 0;  // make sure the _compiler_ knows this value will not change while we are inside the function
  static unsigned long lastSoundTime = 0;  // for delaying noise gate
  constexpr long MinTimeSilence = 1600;    // 1600ms "grace time" before closing noise gate - to avoid chattering

  micIn = micDataSm;

  // remove remaining DC offset from sound signal
  micLev = ((micLev * 8191.0) + micDataReal) / 8192.0;  // takes a few seconds to "catch up" with the Mic Input
  // if(micIn < micLev) micLev = ((micLev * 31.0) + micDataReal) / 32.0;           // align MicLev to lowest input
  // signal
  if (micDataReal < (micLev - 1.2))
    micLev = ((micLev * 31.0) + micDataReal) /
             32.0;  // align with lowest input, but allow some "overlap" to stabilize the filter
  micIn -= micLev;  // Let's center it to 0 now

  // Using an exponential filter to smooth out the signal. We'll add controls for this in a future release.
  float micInNoDC = fabsf(micDataReal - micLev);
  expAdjF = weighting * micInNoDC + ((1.0 - weighting) * expAdjF);
  expAdjF = fabsf(expAdjF);  // Now (!) take the absolute value

  // expAdjF = (expAdjF <= soundSquelch) ? 0: expAdjF; // super simple noise gate
  // if ((soundSquelch == 0) && (expAdjF < 0.25f)) expAdjF = 0;
  if ((expAdjF <= soundSquelch) || ((soundSquelch == 0) && (expAdjF < 0.25f))) {  // noise gate with "closing delay"
    if ((millis() - lastSoundTime) > MinTimeSilence)
      expAdjF = 0.0;
  } else {
    lastSoundTime = millis();
  }

  tmpSample = expAdjF;
  micIn = abs(micIn);  // And get the absolute value of each sample

  sampleAdj =
      tmpSample * sampleGain / 40 * inputLevel / 128 + tmpSample / 16;  // Adjust the gain. with inputLevel adjustment
  sampleReal = tmpSample;

  sampleAdj = fmax(fmin(sampleAdj, 255), 0);  // Question: why are we limiting the value to 8 bits ???

  // keep "peak" sample, but decay value if current sample is below peak
  if ((sampleMax < sampleReal) && (sampleReal > 0.5)) {
    sampleMax = sampleMax + 0.5 * (sampleReal - sampleMax);  // new peak - with some filtering
    if ((millis() - timeOfPeak > 80) && (sampleAvg > 1)) {   // no iffs-n-butts
      samplePeak = 2;
      timeOfPeak = millis();
    }
  } else {
    if ((multAgc * sampleMax > agcZoneStop[AGC_preset]) && (soundAgc > 0))
      sampleMax = sampleMax + 0.5 * (sampleReal - sampleMax);  // over AGC Zone - get back quickly
    else
      sampleMax = sampleMax * agcSampleDecay[AGC_preset];  // signal to zero --> 5-8sec
  }
  if (sampleMax < 0.5)
    sampleMax = 0.0;

  sampleAvg = ((sampleAvg * 15.0) + sampleAdj) / 16.0;  // Smooth it out over the last 16 samples.
  sampleAvg = fabsf(sampleAvg);                         // make sure we have a positive value

  if (millis() - timeOfPeak > 33) {  // Auto-reset of samplePeak after a complete frame has passed.
    samplePeak = 0;
  }

  // Poor man's beat detection by seeing if sample > Average + some value.
  if ((MAX_VOL > 1) && (BIN_NUM > 6) && (fftBin[BIN_NUM] > MAX_VOL) && (millis() - timeOfPeak > 100) &&
      (sampleAvg > 1)) {  // This goes through ALL of the 255 bins - but ignores stupid settings
    // Then we got a peak, else we don't.
    samplePeak = max(uint8_t(1), samplePeak);  // ignore this peak if we already have an active "good peak"
    if (samplePeak == 1)
      timeOfPeak = millis();
  }
}  // getSample()

/*
 * A "PI control" multiplier to automatically adjust sound sensitivity.
 *
 * A few tricks are implemented so that sampleAgc does't only utilize 0% and 100%:
 * 0. don't amplify anything below squelch (but keep previous gain)
 * 1. gain input = maximum signal observed in the last 5-10 seconds
 * 2. we use two setpoints, one at ~60%, and one at ~80% of the maximum signal
 * 3. the amplification depends on signal level:
 *    a) normal zone - very slow adjustment
 *    b) emergency zome (<10% or >90%) - very fast adjustment
 */
static void agcAvg(unsigned long the_time) {
  const int AGC_preset =
      (soundAgc > 0) ? (soundAgc - 1)
                     : 0;  // make sure the _compiler_ knows this value will not change while we are inside the function
  static int last_soundAgc = -1;

  float lastMultAgc = multAgc;          // last muliplier used
  float multAgcTemp = multAgc;          // new multiplier
  float tmpAgc = sampleReal * multAgc;  // what-if amplified signal

  float control_error;                     // "control error" input for PI control
  static double control_integrated = 0.0;  // "integrator control" = accumulated error

  if (last_soundAgc != soundAgc)
    control_integrated = 0.0;  // new preset - reset integrator

  // For PI control, we need to have a contant "frequency"
  // so let's make sure that the control loop is not running at insane speed
  static unsigned long last_time = 0;
  unsigned long time_now = millis();
  if ((the_time > 0) && (the_time < time_now))
    time_now = the_time;  // allow caller to override my clock

  if (time_now - last_time > 2) {
    last_time = time_now;

    if ((fabsf(sampleReal) < 2.0) || (sampleMax < 1.0)) {
      // MIC signal is "squelched" - deliver silence
      multAgcTemp = multAgc;  // keep old control value (no change)
      tmpAgc = 0;
      // we need to "spin down" the intgrated error buffer
      if (fabs(control_integrated) < 0.01)
        control_integrated = 0.0;
      else
        control_integrated = control_integrated * 0.91;
    } else {
      // compute new setpoint
      if (tmpAgc <= agcTarget0Up[AGC_preset])
        multAgcTemp =
            agcTarget0[AGC_preset] / sampleMax;  // Make the multiplier so that sampleMax * multiplier = first setpoint
      else
        multAgcTemp =
            agcTarget1[AGC_preset] / sampleMax;  // Make the multiplier so that sampleMax * multiplier = second setpoint
    }
    // limit amplification
    if (multAgcTemp > 32.0)
      multAgcTemp = 32.0;
    if (multAgcTemp < 1.0 / 64.0)
      multAgcTemp = 1.0 / 64.0;

    // compute error terms
    control_error = multAgcTemp - lastMultAgc;

    if (((multAgcTemp > 0.085) && (multAgcTemp < 6.5))       // integrator anti-windup by clamping
        && (multAgc * sampleMax < agcZoneStop[AGC_preset]))  // integrator ceiling (>140% of max)
      control_integrated += control_error * 0.002 * 0.25;    // 2ms = intgration time; 0.25 for damping
    else
      control_integrated *= 0.9;  // spin down that beasty integrator

    // apply PI Control
    tmpAgc = sampleReal * lastMultAgc;  // check "zone" of the signal using previous gain
    if ((tmpAgc > agcZoneHigh[AGC_preset]) ||
        (tmpAgc < soundSquelch + agcZoneLow[AGC_preset])) {  // upper/lower emergy zone
      multAgcTemp = lastMultAgc + agcFollowFast[AGC_preset] * agcControlKp[AGC_preset] * control_error;
      multAgcTemp += agcFollowFast[AGC_preset] * agcControlKi[AGC_preset] * control_integrated;
    } else {  // "normal zone"
      multAgcTemp = lastMultAgc + agcFollowSlow[AGC_preset] * agcControlKp[AGC_preset] * control_error;
      multAgcTemp += agcFollowSlow[AGC_preset] * agcControlKi[AGC_preset] * control_integrated;
    }

    // limit amplification again - PI controler sometimes "overshoots"
    if (multAgcTemp > 32.0)
      multAgcTemp = 32.0;
    if (multAgcTemp < 1.0 / 64.0)
      multAgcTemp = 1.0 / 64.0;
  }

  // NOW finally amplify the signal
  tmpAgc = sampleReal * multAgcTemp;  // apply gain to signal
  if (fabsf(sampleReal) < 2.0)
    tmpAgc = 0;  // apply squelch threshold
  if (tmpAgc > 255)
    tmpAgc = 255;  // limit to 8bit
  if (tmpAgc < 1)
    tmpAgc = 0;  // just to be sure

  // update global vars ONCE - multAgc, sampleAGC, rawSampleAgc
  multAgc = multAgcTemp;
  rawSampleAgc = 0.8 * tmpAgc + 0.2 * (float) rawSampleAgc;

  // update smoothed AGC sample
  if (fabsf(tmpAgc) < 1.0)
    sampleAgc = 0.5 * tmpAgc + 0.5 * sampleAgc;  // fast path to zero
  else
    sampleAgc = sampleAgc + agcSampleSmooth[AGC_preset] * (tmpAgc - sampleAgc);  // smooth path

  sampleAgc = fabsf(sampleAgc);

  last_soundAgc = soundAgc;
}  // agcAvg()

/* limit sound dynamics by contraining "attack" and "decay" times */
constexpr float bigChange = 196;  // just a representative number - a large, expected sample value
/* values below will be made user-configurable later */
constexpr float attackTime = 800;  // attack time -> 0.8sec
constexpr float decayTime = 2800;  // decay time  -> 2.8sec

/* This fuctions limits the dynamics of sampleAvg and sampleAgc. It does not affect FFTResult[] or raw samples (sample,
 * rawSampleAgc) */
// effects: Gravimeter, Gravcenter, Gravcentric, Noisefire, Plasmoid, Freqpixels, Freqwave, Gravfreq, (2D Swirl, 2D
// Waverly) experimental, as it still has side-effects on AGC - AGC detects "silence" to late (due to long decay time)
// and ditches up the gain multiplier.
static void limitSampleDynamics(void) {
#ifdef SOUND_DYNAMICS_LIMITER
  static unsigned long last_time = 0;
  static float last_sampleAvg = 0.0f;
  static float last_sampleAgc = 0.0f;

  long delta_time = millis() - last_time;
  delta_time = constrain(delta_time, 1, 1000);  // below 1ms -> 1ms; above 1sec -> sily lil hick-up
  float maxAttack = bigChange * float(delta_time) / attackTime;
  float maxDecay = -bigChange * float(delta_time) / decayTime;
  float deltaSample;

  // non-AGC sample
  if ((attackTime > 0) && (decayTime > 0)) {
    deltaSample = sampleAvg - last_sampleAvg;
    if (deltaSample > maxAttack)
      deltaSample = maxAttack;
    if (deltaSample < maxDecay)
      deltaSample = maxDecay;
    sampleAvg = last_sampleAvg + deltaSample;
  }
  // same for AGC sample
  if ((attackTime > 0) && (decayTime > 0)) {
    deltaSample = sampleAgc - last_sampleAgc;
    if (deltaSample > maxAttack)
      deltaSample = maxAttack;
    if (deltaSample < maxDecay)
      deltaSample = maxDecay;
    sampleAgc = last_sampleAgc + deltaSample;
  }

  last_sampleAvg = sampleAvg;
  last_sampleAgc = sampleAgc;
  last_time = millis();
#endif
}

////////////////////
// Begin FFT Code //
////////////////////

// using latest AruinoFFT lib, because it supports float and its much faster!
// lib_deps += https://github.com/kosme/arduinoFFT#develop @

// The following optimizations seem to be optimal on ESP32 (~1.5ms per FFT run)
// #define FFT_SPEED_OVER_PRECISION     // enables use of reciprocals (1/x etc), and an a few other speedups - WLEDMM
// not faster on ESP32 #define FFT_SQRT_APPROXIMATION       // enables "quake3" style inverse sqrt - WLEDMM slower on
// ESP32
#define sqrt(x) \
  sqrtf(x)  // little hack that reduces FFT time by 10-50% on ESP32 (as alternative to FFT_SQRT_APPROXIMATION)
#define sqrt_internal sqrtf  // see https://github.com/kosme/arduinoFFT/pull/83

// Create FFT object
static ArduinoFFT<float> FFT = ArduinoFFT<float>(vReal, vImag, samplesFFT, SAMPLE_RATE, true);

static float fftAdd(int from, int to) {
  int i = from;
  float result = 0;
  while (i <= to) {
    result += fftBin[i++];
  }
  return result;
}

// Bandpass filter for PDM microphones
static void runMicFilter(uint16_t numSamples, float *sampleBuffer) {  // pre-filtering of raw samples (band-pass)
  // band pass filter - can reduce noise floor by a factor of 50
  // downside: frequencies below 60Hz will be ignored

  // low frequency cutoff parameter - see
  // https://dsp.stackexchange.com/questions/40462/exponential-moving-average-cut-off-frequency
  // constexpr float alpha = 0.062f;   // 100Hz
  constexpr float alpha = 0.04883f;  //  80Hz
  // constexpr float alpha = 0.03662f; //  60Hz
  // constexpr float alpha = 0.0225f;  //  40Hz
  //  high frequency cutoff  parameter
  // constexpr float beta1 = 0.75;    //  5Khz
  // constexpr float beta1 = 0.82;    //  7Khz
  constexpr float beta1 = 0.8285;  //  8Khz
  // constexpr float beta1 = 0.85;    // 10Khz

  constexpr float beta2 = (1.0f - beta1) / 2.0;
  static float last_vals[2] = {0.0f};  // FIR high freq cutoff filter
  static float lowfilt = 0.0f;         // IIR low frequency cutoff filter

  for (int i = 0; i < numSamples; i++) {
    // FIR lowpass, to remove high frequency noise
    float highFilteredSample;
    if (i < (numSamples - 1))
      highFilteredSample =
          beta1 * sampleBuffer[i] + beta2 * last_vals[0] + beta2 * sampleBuffer[i + 1];  // smooth out spikes
    else
      highFilteredSample = beta1 * sampleBuffer[i] + beta2 * last_vals[0] +
                           beta2 * last_vals[1];  // spcial handling for last sample in array
    last_vals[1] = last_vals[0];
    last_vals[0] = sampleBuffer[i];
    sampleBuffer[i] = highFilteredSample;
    // IIR highpass, to remove low frequency noise
    lowfilt += alpha * (sampleBuffer[i] - lowfilt);
    sampleBuffer[i] = sampleBuffer[i] - lowfilt;
  }
}

// sample smoothing, by using a sliding average FIR highpass filter (first half of MicFilter from above)
static void runMicSmoothing(uint16_t numSamples, float *sampleBuffer) {
  constexpr float beta1 = 0.8285;                //  ~8Khz
  constexpr float beta2 = (1.0f - beta1) / 2.0;  // note to self: better use biquad ?
  static float last_vals[2] = {0.0f};            // FIR filter buffer

  for (int i = 0; i < numSamples; i++) {
    float highFilteredSample;
    if (i < (numSamples - 1))
      highFilteredSample =
          beta1 * sampleBuffer[i] + beta2 * last_vals[0] + beta2 * sampleBuffer[i + 1];  // smooth out spikes
    else
      highFilteredSample = beta1 * sampleBuffer[i] + beta2 * last_vals[0] +
                           beta2 * last_vals[1];  // spcial handling for last sample in array
    last_vals[1] = last_vals[0];
    last_vals[0] = sampleBuffer[i];
    sampleBuffer[i] = highFilteredSample;
  }
}

// a variation of above, with higher cut-off frequency
static void runMicSmoothing_v2(uint16_t numSamples, float *sampleBuffer) {
  constexpr float beta1 = 0.85;                  // ~10Khz
  constexpr float beta2 = (1.0f - beta1) / 2.0;  // note to self: better use biquad ?
  static float last_vals[2] = {0.0f};            // FIR filter buffer

  for (int i = 0; i < numSamples; i++) {
    float highFilteredSample;
    if (i < (numSamples - 1))
      highFilteredSample =
          beta1 * sampleBuffer[i] + beta2 * last_vals[0] + beta2 * sampleBuffer[i + 1];  // smooth out spikes
    else
      highFilteredSample = beta1 * sampleBuffer[i] + beta2 * last_vals[0] +
                           beta2 * last_vals[1];  // spcial handling for last sample in array
    last_vals[1] = last_vals[0];
    last_vals[0] = sampleBuffer[i];
    sampleBuffer[i] = highFilteredSample;
  }
}

// High-Pass filter, 6db per octave
static void runHighFilter6db(const float filter, uint16_t numSamples, float *sampleBuffer) {
  static float lowfilt = 0.0f;  // IIR low frequency cutoff filter
  for (int i = 0; i < numSamples; i++) {
    lowfilt += filter * (sampleBuffer[i] - lowfilt);  // lowpass
    sampleBuffer[i] = sampleBuffer[i] - lowfilt;      // lowpass --> highpass
  }
}

// High-Pass filter, 12db per octave
static void runHighFilter12db(const float filter, uint16_t numSamples, float *sampleBuffer) {
  static float lowfilt1 = 0.0f;  // IIR low frequency cutoff filter - first pass = 6db
  static float lowfilt2 = 0.0f;  // IIR low frequency cutoff filter - second pass = 12db
  for (int i = 0; i < numSamples; i++) {
    lowfilt1 += filter * (sampleBuffer[i] - lowfilt1);  // first lowpass 6db
    // lowfilt2 += filter * (lowfilt1 - lowfilt2);     // second lowpass +6db
    // sampleBuffer[i] = sampleBuffer[i] - lowfilt2;   // lowpass --> highpass
    // implementation below has better results, compared to the code above
    float pass1Out = sampleBuffer[i] - lowfilt1;  // output from first stage (lowpass --> highpass)
    lowfilt2 += filter * (pass1Out - lowfilt2);   // second lowpass +6db
    sampleBuffer[i] = pass1Out - lowfilt2;        // lowpass --> highpass
  }
}

}  // namespace music_leds
}  // namespace esphome
