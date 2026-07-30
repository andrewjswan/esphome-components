#include "music_leds.h"

#include "esphome/components/fastled_helper/utils.h"
#include "esphome/components/light/addressable_light_effect.h"

#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

#define DEBUG

#ifdef DEBUG
#include "debug.h"
#endif

namespace esphome::music_leds {

enum EventGroupBits : uint32_t {
  COMMAND_STOP = (1 << 0),  // Signals the FFT task should stop

  TASK_STARTING = (1 << 3),
  TASK_RUNNING = (1 << 4),
  TASK_STOPPING = (1 << 5),
  TASK_STOPPED = (1 << 6),
#ifdef DEBUG
  TASK_INFO = (1 << 7),
#endif

  ERROR_MEMORY = (1 << 9),
  ERROR_FFT = (1 << 10),

  WARNING_FULL_RING_BUFFER = (1 << 13),

  ERROR_BITS = ERROR_MEMORY | ERROR_FFT,
  ALL_BITS = 0xfffff,  // 24 total bits available in an event group
};

static const LogString *music_leds_state_to_string(State state) {
  switch (state) {
    case State::STARTING:
      return LOG_STR("STARTING");
    case State::STOPPING:
      return LOG_STR("STOPPING");
    case State::STOPPED:
      return LOG_STR("STOPPED");
    case State::RUNNING:
      return LOG_STR("RUNNING");
    default:
      return LOG_STR("UNKNOWN");
  }
}

void MusicLeds::setup() {
  this->event_group_ = xEventGroupCreate();
  if (this->event_group_ == nullptr) {
    ESP_LOGE(TAG, "Failed to create event group");
    this->mark_failed();
    return;
  }

  if (this->microphone_ == nullptr) {
    ESP_LOGE(TAG, "Microphone is not defined in configuration!");
    this->mark_failed();
    return;
  }

  if (this->fft_buffer_ == nullptr) {
    this->fft_buffer_ = (float *) calloc(SAMPLES_FFT, sizeof(float));
  }
  if (this->fft_buffer_ == nullptr) {
    ESP_LOGE(TAG, "Allocation of the dynamic sliding window buffer failed!");
    this->mark_failed();
    return;
  }

#ifdef USE_OTA_STATE_LISTENER
  ota::get_global_ota_callback()->add_global_state_listener(this);
#endif

  // Register callback for incoming raw byte vector
  this->microphone_->add_data_callback([this](const std::vector<uint8_t> &data) {
    if (data.empty()) return;

    // Stream bytes straight into the float ring buffer
    this->process_audio_to_ring_(data);

    if (this->ring_buffer_.available() >= SAMPLES_FFT && this->FFT_Task != nullptr) {
      xTaskNotifyGive(this->FFT_Task);
    }
  });

  // Initialize the standalone processing module prior to spinning up the worker thread
  const uint32_t sample_rate = this->microphone_->get_audio_stream_info().get_sample_rate();
  this->fft_engine_ = std::make_unique<FFTEngine>(sample_rate);
  this->band_aggregator_ = std::make_unique<BandAggregator>(sample_rate);
  this->dynamics_processor_ = std::make_unique<DynamicsProcessor>(this->sample_scale_);
  this->dynamics_processor_->set_scaling_mode(this->scaling_mode_);
  this->beat_detector_ = std::make_unique<BeatDetector>(this->sample_scale_, this->beat_sensitivity_);
  this->noise_gate_ = std::make_unique<NoiseGate>(this->sample_scale_, this->noise_gate_floor_);
  this->pre_amplifier_ = std::make_unique<PreAmplifier>(this->sample_scale_, this->pre_amp_gain_); 
  this->peak_latch_ = std::make_unique<PeakLatch>(100, 80, 50, 0.5f);  // 100ms freq lockout, 80ms vol lockout, 50ms hold window, 0.5 threshold
  this->geq_processor_ = std::make_unique<GEQProcessor>(this->sample_gain_);
  this->geq_processor_->set_scaling_mode(this->scaling_mode_);

  ESP_LOGCONFIG(TAG, "Music Leds initialized");
  this->start();
}

MusicLeds::~MusicLeds() {
  if (this->fft_buffer_ != nullptr) {
    free(this->fft_buffer_);
    this->fft_buffer_ = nullptr;
  }
  this->fft_engine_.reset();
  this->band_aggregator_.reset();
  this->dynamics_processor_.reset();
  this->beat_detector_.reset();
  this->peak_latch_.reset();
  this->noise_gate_.reset();
  this->pre_amplifier_.reset();
  this->geq_processor_.reset();
}

#ifdef USE_OTA_STATE_LISTENER
void MusicLeds::on_ota_global_state(ota::OTAState state, float progress, uint8_t error, ota::OTAComponent *comp) {
  if (state == ota::OTA_STARTED) {
    this->on_shutdown();
  }
}
#endif

void MusicLeds::loop() {
  uint32_t event_group_bits = xEventGroupGetBits(this->event_group_);

  if (event_group_bits & EventGroupBits::ERROR_MEMORY) {
    xEventGroupClearBits(this->event_group_, EventGroupBits::ERROR_MEMORY);
    ESP_LOGE(TAG, "Encountered an error allocating buffers");
  }

  if (event_group_bits & EventGroupBits::ERROR_FFT) {
    xEventGroupClearBits(this->event_group_, EventGroupBits::ERROR_FFT);
    ESP_LOGE(TAG, "Encountered an error while performing an FFT");
  }

  if (event_group_bits & EventGroupBits::WARNING_FULL_RING_BUFFER) {
    xEventGroupClearBits(this->event_group_, EventGroupBits::WARNING_FULL_RING_BUFFER);
    ESP_LOGW(TAG, "Not enough free bytes in ring buffer to store incoming audio data. Resetting the ring buffer.");
  }

  if (event_group_bits & EventGroupBits::TASK_STARTING) {
    ESP_LOGD(TAG, "FFT task has started, attempting to allocate memory for buffers");
    xEventGroupClearBits(this->event_group_, EventGroupBits::TASK_STARTING);
  }

  if (event_group_bits & EventGroupBits::TASK_RUNNING) {
    ESP_LOGD(TAG, "FFT task is running");
    xEventGroupClearBits(this->event_group_, EventGroupBits::TASK_RUNNING);
    this->set_state_(State::RUNNING);
  }

  if (event_group_bits & EventGroupBits::TASK_STOPPING) {
    ESP_LOGD(TAG, "FFT task is stopping, deallocating buffers");
    xEventGroupClearBits(this->event_group_, EventGroupBits::TASK_STOPPING);
  }

  if ((event_group_bits & EventGroupBits::TASK_STOPPED)) {
    ESP_LOGD(TAG, "FFT task is finished, freeing task resources");
    this->on_stop();
    xEventGroupClearBits(this->event_group_, ALL_BITS);
    this->set_state_(State::STOPPED);
  }

  switch (this->state_) {
    case State::STARTING:
      this->on_start();
      break;
    case State::RUNNING:
      this->on_loop();
      break;
    case State::STOPPING:
      xEventGroupSetBits(this->event_group_, EventGroupBits::COMMAND_STOP);
      break;
    case State::STOPPED:
      break;
  }
}

void MusicLeds::dump_config() {
  ESP_LOGCONFIG(TAG, "Music Leds version: %s", MUSIC_LEDS_VERSION);
  if (this->is_failed()) {
    ESP_LOGCONFIG(TAG, "Music Leds initialization FAILED!");
    return;
  }

  // Core scheduling parameters
  ESP_LOGCONFIG(TAG, "         Task Core: %u", FFTTASK_CORE);
  ESP_LOGCONFIG(TAG, "     Task Priority: %u", FFTTASK_PRIORITY);

  // Fetch runtime stream info dynamically straight from the active microphone object
  if (this->microphone_ != nullptr) {
    const auto &stream_info = this->microphone_->get_audio_stream_info();
    ESP_LOGCONFIG(TAG, "  Stream Bit Depth: %d bit", stream_info.get_bits_per_sample());
    ESP_LOGCONFIG(TAG, "       Sample rate: %ld Hz", static_cast<int32_t>(stream_info.get_sample_rate()));
    
  }

  // Extract independent internal pipeline features and scaling styles
  ESP_LOGCONFIG(TAG, "     Pre-Amp Gain: %.1f", this->pre_amp_gain_);
  ESP_LOGCONFIG(TAG, " Noise Gate Floor: %.3f", this->noise_gate_floor_);
  ESP_LOGCONFIG(TAG, " Beat Sensitivity: %d (1-100)", this->beat_sensitivity_);
  ESP_LOGCONFIG(TAG, "  Sample Scale Factor: %.6f (1.0f / %d)", 
                this->sample_scale_, 
                (this->sample_scale_ > 0.0f) ? static_cast<int16_t>(1.0f / this->sample_scale_) : 0);

  // Map the strongly-typed scaling enum to descriptive human logs
  const char *scaling_str = "UNKNOWN";
  switch (this->scaling_mode_) {
    case FFTScalingMode::LINEAR:      scaling_str = "Linear (None)"; break;
    case FFTScalingMode::LOGARITHMIC: scaling_str = "Logarithmic (True Hearing Curve)"; break;
    case FFTScalingMode::SQUARE_ROOT: scaling_str = "Square Root (Psychoacoustic Standard)"; break;
  }
  ESP_LOGCONFIG(TAG, " FFTResult scaling: %s", scaling_str);
}  // dump_config() end

void MusicLeds::on_shutdown() { this->stop(); }

void MusicLeds::start() {
  if (this->state_ != State::STOPPED)
    return;

  ESP_LOGD(TAG, "Starting MusicLeds");
  this->state_ = State::STARTING;

  if (this->microphone_ != nullptr) {
    this->microphone_->start();
  }
}

void MusicLeds::stop() {
  if (this->state_ == State::STOPPED)
    return;

  ESP_LOGD(TAG, "Stopping MusicLeds...");
  this->set_state_(State::STOPPING);

  if (this->microphone_ != nullptr) {
    this->microphone_->stop();
  }
}

void MusicLeds::set_state_(State state) {
  if (this->state_ != state) {
    ESP_LOGD(TAG, "State changed from %s to %s", LOG_STR_ARG(music_leds_state_to_string(this->state_)),
             LOG_STR_ARG(music_leds_state_to_string(state)));
    this->state_ = state;
  }
}

void MusicLeds::on_start() {
  this->ring_buffer_.clear();

  // Define the FFT Task and lock it to core
  xTaskCreatePinnedToCore(MusicLeds::FFTcode,  // Function to implement the task
                          "FFT",               // Name of the task
                          5000,                // Stack size in words
                          (void *) this,       // Task input parameter
                          FFTTASK_PRIORITY,    // Priority of the task
                          &this->FFT_Task,     // Task handle
                          FFTTASK_CORE);       // Core where the task should run

  if (this->FFT_Task == nullptr) {
    this->status_momentary_error("MusicLeds task failed to start...", 1000);
  }
}

void MusicLeds::on_stop() {
  vTaskDelete(this->FFT_Task);
  this->FFT_Task = nullptr;

  fastled_helper::FreeLeds();

  this->status_clear_error();
}

void MusicLeds::on_loop() {
  asm volatile("memw" ::: "memory");

#if defined(MUSIC_LEDS_TRIGGERS)
  static unsigned long lastTrigger = millis();
  unsigned long current_millis = millis();
#endif

#ifdef DEBUG
  uint32_t event_group_bits = xEventGroupGetBits(this->event_group_);
  if ((event_group_bits & EventGroupBits::TASK_INFO)) {
    ESP_LOGE(TAG, "LOOP: Samples: VolSmth: %f | High: %f | Mid: %f | Bass: %f",
             this->features_.smoothed_volume, 
             this->features_.high_energy, 
             this->features_.mid_energy,
             this->features_.bass_energy) 
    xEventGroupClearBits(this->event_group_, EventGroupBits::TASK_INFO);
  }
#endif

#if defined(MUSIC_LEDS_TRIGGERS)
  if (current_millis - lastTrigger > 200) {
    float scaled_smth = this->features_.volume_smth();
    float scaled_raw  = this->features_.volume_raw();
    float pitch_val   = this->features_.dominant_frequency_hz;
    bool sample_peak = this->features_.sample_peak;

    for (auto *t : on_sound_loop_triggers_) {
      t->process(scaled_smth, scaled_raw, pitch_val, sample_peak);
    }
    lastTrigger = current_millis;
  }
#endif
}

#if defined(MUSIC_LEDS_TRIGGERS)
void MusicLedsSoundLoopTrigger::process(float volume_smth, int16_t volume_raw, float fft_major_peak, bool sample_peak) {
  this->trigger(volume_smth, volume_raw, fft_major_peak, sample_peak);
}
#endif

void MusicLeds::process_audio_to_ring_(const std::vector<uint8_t> &data) {
  const auto &stream_info = this->microphone_->get_audio_stream_info();

  const size_t source_bytes_per_sample = stream_info.samples_to_bytes(1);
  const uint32_t source_channels = stream_info.get_channels();
  const size_t source_bytes_per_frame = stream_info.frames_to_bytes(1);
  const uint32_t total_frames = stream_info.bytes_to_frames(data.size());

  if (total_frames == 0) return;

  // Authentic Audio Scaler: Integrates standard Q31 bit-depth translation 
  // with the native initialization scale factor (e.g., 1.0f / 24.0f) matched to your hardware profile.
  // This positions the incoming amplitude domain perfectly within the reference bounds.
  const float Q31_TO_FLOAT = (AMPLITUDE_SCALE_16BIT * this->sample_scale_) / 2147483648.0f;
  
  const float channel_weight_multiplier = 1.0f / static_cast<float>(source_channels);

  for (uint32_t frame_index = 0; frame_index < total_frames; ++frame_index) {
    float frame_mono_mix = 0.0f;
    for (uint32_t channel_index = 0; channel_index < source_channels; ++channel_index) {
      const uint32_t sample_index = (frame_index * source_bytes_per_frame) + 
                                    (channel_index * source_bytes_per_frame / source_channels); // Secure precise indexing bounds

      // Unpack raw hardware bytes natively using ESPHome's internal adaptive bit-depth parser
      int32_t q31_sample = audio::unpack_audio_sample_to_q31(&data[sample_index], source_bytes_per_sample);

      frame_mono_mix += static_cast<float>(q31_sample) * Q31_TO_FLOAT;
    }

    // Mix down to a single mono float sample point
    float final_sample = frame_mono_mix * channel_weight_multiplier;
    this->ring_buffer_.write_overwrite(&final_sample, 1);
  }
}

// *****************************************************************************
// FFT main task
// audio processing task: read samples, run FFT, fill GEQ channels from FFT results
// *****************************************************************************
void MusicLeds::FFTcode(void *parameter) {
  MusicLeds *this_task = (MusicLeds *) parameter;
  ESP_LOGCONFIG(TAG, "FFT: started on core: %u", FFTTASK_CORE);

  xEventGroupSetBits(this_task->event_group_, EventGroupBits::TASK_STARTING);

  float *fft_buffer = this_task->fft_buffer_;

  xEventGroupSetBits(this_task->event_group_, EventGroupBits::TASK_RUNNING);

  while (!(xEventGroupGetBits(this_task->event_group_) & EventGroupBits::COMMAND_STOP)) {
    // Only run the FFT computing code if microphone running
    if (!this_task->microphone_is_running()) {
      this_task->status_momentary_warning("Microphone not running!");
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }
    this_task->status_clear_warning();

    if (this_task->ring_buffer_.available() < SAMPLES_FFT) {
      ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));  // 100ms timeout as safety fallback
      continue;
    }

    this_task->ring_buffer_.peek(fft_buffer, SAMPLES_FFT);

#ifdef DEBUG
    if (esphome::music_leds::debug::should_log()) {
      ESP_LOGD(TAG, "=================== DEBUG START =====================");
    }
#endif

    // Fast Fourier Transform
    this_task->fft_engine_->process(fft_buffer);
    float pitch = this_task->fft_engine_->dominant_frequency_hz();
    if (std::isnan(pitch)) {
      this_task->ring_buffer_.advance(HOP_SIZE);
      continue;
    }
    this_task->features_.dominant_frequency_hz = pitch;

    // Aggregate frequency bins from the FFT magnitudes spectrum into macro bands
    this_task->band_aggregator_->process(
        this_task->fft_engine_->magnitudes(),
        this_task->features_.bass_energy,
        this_task->features_.mid_energy,
        this_task->features_.high_energy
    );

#ifdef DEBUG
    // Diagnostics: Snapshot immediately after Aggregator (Pure physical bins energy)
    float agg_b = this_task->features_.bass_energy;
    float agg_m = this_task->features_.mid_energy;
    float agg_h = this_task->features_.high_energy;
#endif

    // Noise Gate (Enforced prior to pre-amplification and AGC loops)
    // Evaluates pure un-amplified hardware macro band lines.
    // Intercepts the signal chain and purges registers to absolute 0.0f if silence is hit.
    this_task->noise_gate_->process(
        this_task->features_.bass_energy,
        this_task->features_.mid_energy,
        this_task->features_.high_energy
    );

#ifdef DEBUG
    // Diagnostics: Snapshot immediately after NoiseGate (Did it successfully zero values?)
    float gate_b = this_task->features_.bass_energy;
    float gate_m = this_task->features_.mid_energy;
    float gate_h = this_task->features_.high_energy;
    bool gate_active = this_task->noise_gate_->is_closed();
#endif

    // Execute the self-contained 16-band graphic equalizer pass
    // Completely standalone - accepts raw magnitudes and writes directly to target features array
    this_task->geq_processor_->process(
        this_task->fft_engine_->magnitudes(),
        this_task->features_.fft_result,
        this_task->noise_gate_->is_closed()
    );

    if (this_task->noise_gate_->is_closed()) {
      this_task->features_.magnitude = 0.0f;
    } else {
      this_task->features_.magnitude = this_task->fft_engine_->magnitude();
    }

    // Calibrate aggregated macro band magnitudes using the Pre-Amplifier gain
    // If the gate is closed, multiplying 0.0f by any pink noise curves remains safely 0.0f
    this_task->pre_amplifier_->process(
        this_task->features_.bass_energy,
        this_task->features_.mid_energy,
        this_task->features_.high_energy
    );

#ifdef DEBUG
    // Diagnostics: Snapshot immediately after Pre-Amplifier (Amplified raw signal)
    float amp_b = this_task->features_.bass_energy;
    float amp_m = this_task->features_.mid_energy;
    float amp_h = this_task->features_.high_energy;
#endif

    // Transient Beat Onset Detection runs on filtered linear energy
    this_task->features_.is_beat_detected = this_task->beat_detector_->process(
        this_task->features_.bass_energy
    );

    // Apply Linear AGC normalization and temporal Slew-Rate limiting
    // Controlled by the zero-line decay tail lock to reset persistent history registers instantly.
    this_task->dynamics_processor_->process(
        this_task->features_.smoothed_volume,
        this_task->features_.raw_volume,
        this_task->features_.bass_energy,
        this_task->features_.mid_energy,
        this_task->features_.high_energy
    );

#ifdef DEBUG
    // Diagnostics: Snapshot immediately after Dynamics Stage 1 (AGC applied, still linear)
    float dyn_b = this_task->features_.bass_energy;
    float dyn_m = this_task->features_.mid_energy;
    float dyn_h = this_task->features_.high_energy;
    float dyn_vol = this_task->features_.raw_volume;
#endif

    // Psychoacoustic Scaling Stage - Enforced strictly after gate and beat tasks!
    // Compresses clean linear bands using selected curves (e.g. Square Root)
    this_task->dynamics_processor_->apply_psychoacoustic_scaling(
        this_task->features_.smoothed_volume,
        this_task->features_.raw_volume,
        this_task->features_.bass_energy,
        this_task->features_.mid_energy,
        this_task->features_.high_energy
    );

    // Dual-Channel temporal peak latch window calculation for WLED effect compatibility
    this_task->peak_latch_->process(
        this_task->features_.is_beat_detected,
        this_task->features_.raw_volume,
        this_task->features_.sample_peak
    );

#ifdef DEBUG
    if (esphome::music_leds::debug::should_log()) {
      float min_val = fft_buffer[0];
      float max_val = fft_buffer[0];
      float sum_val = 0.0f;

      for (int i = 0; i < SAMPLES_FFT; i++) {
        if (fft_buffer[i] < min_val) min_val = fft_buffer[i];
        if (fft_buffer[i] > max_val) max_val = fft_buffer[i];
        sum_val += fft_buffer[i];
      }
      float dc_offset = sum_val / SAMPLES_FFT;

      ESP_LOGD("FFT_WAVE", "[RAW WAVE SHAPE] Min: %.1f | Max: %.1f | DC_Offset (Avg): %.1f",
               min_val, max_val, dc_offset);

      const float* raw_mags = this_task->fft_engine_->magnitudes();
      ESP_LOGD(TAG, "================ BINS SPECTRUM RADAR ================");
      // Print first 20 bins with their calculated center frequencies (assuming 10240Hz / 512 window)
      char bin_log_buffer[128];
      for (int i = 0; i < 20; i += 5) {
        snprintf(bin_log_buffer, sizeof(bin_log_buffer), 
                 "Bin[%02d..%02d]: #%02d(%.1fHz):%.3f | #%02d(%.1fHz):%.3f | #%02d(%.1fHz):%.3f | #%02d(%.1fHz):%.3f | #%02d(%.1fHz):%.3f",
                 i, i+4,
                 i,   (i * 20.0f),   raw_mags[i],
                 i+1, ((i+1) * 20.0f), raw_mags[i+1],
                 i+2, ((i+2) * 20.0f), raw_mags[i+2],
                 i+3, ((i+3) * 20.0f), raw_mags[i+3],
                 i+4, ((i+4) * 20.0f), raw_mags[i+4]);
        ESP_LOGD(TAG, "%s", bin_log_buffer);
      }

      ESP_LOGD(TAG, "=======================================================");
      char geq_log_buffer[128];
      snprintf(geq_log_buffer, sizeof(geq_log_buffer),
               "GEQ CH[00..07]: %03d | %03d | %03d | %03d | %03d | %03d | %03d | %03d",
               this_task->features_.fft_result[0], this_task->features_.fft_result[1],
               this_task->features_.fft_result[2], this_task->features_.fft_result[3],
               this_task->features_.fft_result[4], this_task->features_.fft_result[5],
               this_task->features_.fft_result[6], this_task->features_.fft_result[7]);
      ESP_LOGD(TAG, "%s", geq_log_buffer);

      snprintf(geq_log_buffer, sizeof(geq_log_buffer),
               "GEQ CH[08..15]: %03d | %03d | %03d | %03d | %03d | %03d | %03d | %03d",
               this_task->features_.fft_result[8],  this_task->features_.fft_result[9],
               this_task->features_.fft_result[10], this_task->features_.fft_result[11],
               this_task->features_.fft_result[12], this_task->features_.fft_result[13],
               this_task->features_.fft_result[14], this_task->features_.fft_result[15]);
      ESP_LOGD(TAG, "%s", geq_log_buffer);

      ESP_LOGD(TAG, "=======================================================");
      ESP_LOGD(TAG, "[STEP AGGREGATOR] Bass: %.4f | Mid: %.4f | High: %.4f", agg_b, agg_m, agg_h);
      ESP_LOGD(TAG, "[STEP NOISEGATE ] Bass: %.4f | Mid: %.4f | High: %.4f | GateClosed: %s", gate_b, gate_m, gate_h, gate_active ? "YES" : "NO");
      ESP_LOGD(TAG, "[STEP PRE-AMP   ] Bass: %.4f | Mid: %.4f | High: %.4f", amp_b, amp_m, amp_h);
      ESP_LOGD(TAG, "[STEP DYNAMICS  ] Bass: %.4f | Mid: %.4f | High: %.4f | VolRaw: %.4f", dyn_b, dyn_m, dyn_h, dyn_vol);
      ESP_LOGD(TAG, "[ENGINE STATUS  ] Raw Peak Magnitude: %.4f", this_task->features_.magnitude);
      ESP_LOGD(TAG, "[FINAL FEATURES ] VolRaw: %.3f | VolSmth: %.3f | Bass: %.3f | Mid: %.3f | Hi: %.3f | Beat: %s | Peak: %s",
               this_task->features_.raw_volume, 
               this_task->features_.smoothed_volume, 
               this_task->features_.bass_energy, 
               this_task->features_.mid_energy,
               this_task->features_.high_energy,
               this_task->features_.is_beat_detected ? "YES" : "NO",
               this_task->features_.sample_peak ? "YES" : "NO");
      esphome::music_leds::debug::commit_timestamp();
    }
#endif

    asm volatile("memw" ::: "memory");
    this_task->ring_buffer_.advance(HOP_SIZE);
  }  // while (!(xEventGroupGetBits(this_task->event_group_) & COMMAND_STOP))

  xEventGroupSetBits(this_task->event_group_, EventGroupBits::TASK_STOPPING);
  this_task->microphone_->stop();
  xEventGroupSetBits(this_task->event_group_, EventGroupBits::TASK_STOPPED);

  vTaskDelete(nullptr);
}  // FFTcode() task end

}  // namespace esphome::music_leds
