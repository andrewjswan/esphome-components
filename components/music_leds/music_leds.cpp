#include "music_leds.h"

#include "esphome/components/fastled_helper/utils.h"
#include "esphome/components/light/addressable_light_effect.h"

#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

#define DEBUG

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

#ifdef DEBUG
    static uint32_t last_cb_log = 0;
    if (millis() - last_cb_log > 2000) {
      ESP_LOGD(TAG, "DEBUG AUDIO: Callback triggered. Vector bytes: %d | Ring available: %d/%d",
               data.size(), this->ring_buffer_.available(), RING_BUFFER_SIZE);
      last_cb_log = millis();
    }
#endif

    if (this->ring_buffer_.available() >= SAMPLES_FFT && this->FFT_Task != nullptr) {
      xTaskNotifyGive(this->FFT_Task);
    }
  });

  // Initialize the standalone processing module prior to spinning up the worker thread
  const uint32_t sample_rate = this->microphone_->get_audio_stream_info().get_sample_rate();
  this->fft_engine_ = std::make_unique<FFTEngine>(sample_rate);
  this->band_aggregator_ = std::make_unique<BandAggregator>(sample_rate);
  this->dynamics_processor_ = std::make_unique<DynamicsProcessor>();
  this->beat_detector_ = std::make_unique<BeatDetector>(65);           // Sensitivity 65 (1-100)
  this->peak_latch_ = std::make_unique<PeakLatch>(100, 80, 50, 0.5f);  // 100ms freq lockout, 80ms vol lockout, 50ms hold window, 0.5 threshold
  this->noise_gate_ = std::make_unique<NoiseGate>(0.05f);              // 0.05f silence floor threshold
  this->pre_amplifier_ = std::make_unique<PreAmplifier>(4.5f);

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
    static uint32_t last_task_log = 0;
    if (millis() - last_task_log > 2000) {
      ESP_LOGE(TAG, "DEBUG LOOP: Samples: High: %f | volumeSmth: %f | Bass: %f | Mid: %f",
               this->features_.high_energy,
               this->features_.smoothed_volume,
               this->features_.bass_energy,
               this->features_.mid_energy);
      last_task_log = millis();
    }
    xEventGroupClearBits(this->event_group_, EventGroupBits::TASK_INFO);
  }
#endif

#if defined(MUSIC_LEDS_TRIGGERS)
  if (current_millis - lastTrigger > 200) {
    float scaled_smth = this->features_.smoothed_volume * 255.0f;
    float scaled_raw  = this->features_.raw_volume * 255.0f;
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

  constexpr float Q31_TO_FLOAT = 1.0f / 2147483648.0f;
  const float channel_weight_multiplier = 1.0f / static_cast<float>(source_channels);

  for (uint32_t frame_index = 0; frame_index < total_frames; ++frame_index) {
    float frame_mono_mix = 0.0f;
    for (uint32_t channel_index = 0; channel_index < source_channels; ++channel_index) {
      const uint32_t sample_index = (frame_index * source_bytes_per_frame) +
                                    (channel_index * source_bytes_per_sample);

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
      vTaskDelay(FFT_MIN_CYCLE / portTICK_PERIOD_MS);
      continue;
    }
    this_task->status_clear_warning();

    if (this_task->ring_buffer_.available() < SAMPLES_FFT) {
      ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));  // 100ms timeout as safety fallback
      continue;
    }

#ifdef DEBUG
    static uint32_t last_task_log = 0;
    if (millis() - last_task_log > 2000) {
      ESP_LOGD(TAG, "DEBUG DSP: Core 1 Wakeup. Processing FFT buffer...");
    }
#endif

    this_task->ring_buffer_.peek(fft_buffer, SAMPLES_FFT);

    // Fast Fourier Transform
    this_task->fft_engine_->process(fft_buffer);
    float pitch = this_task->fft_engine_->dominant_frequency_hz();
    if (std::isnan(pitch)) {
      this_task->ring_buffer_.advance(HOP_SIZE);
      continue;
    }
    this_task->features_.dominant_frequency_hz = pitch;

    // Aggregate frequency bands from the FFT magnitudes spectrum
    this_task->band_aggregator_->process(
        this_task->fft_engine_->magnitudes(),
        this_task->features_.bass_energy,
        this_task->features_.mid_energy,
        this_task->features_.high_energy
    );

    // Pre-Amplifier Stage (Calibrating Micro-Scale Fft Magnitudes)
    this_task->pre_amplifier_->process(
        this_task->features_.bass_energy,
        this_task->features_.mid_energy,
        this_task->features_.high_energy
    );

    // Compute raw preliminary mean volume to feed the AGC dynamics engine directly
    float raw_volume_bridge = (this_task->features_.bass_energy +
                               this_task->features_.mid_energy +
                               this_task->features_.high_energy) / 3.0f;

    // Apply temporal rate limiting and AGC normalization
    // The AGC engine MUST see the raw continuous signal to track historical peaks accurately!
    this_task->dynamics_processor_->process(
        raw_volume_bridge,
        this_task->features_.smoothed_volume,
        this_task->features_.bass_energy,
        this_task->features_.mid_energy,
        this_task->features_.high_energy
    );

    // Re-compute final calibrated mean volume from the clean normalized bands
    raw_volume_bridge  = (this_task->features_.bass_energy +
                          this_task->features_.mid_energy +
                          this_task->features_.high_energy) / 3.0f;

    // Transient Beat Onset Detection runs on normalized energy
    this_task->features_.is_beat_detected = this_task->beat_detector_->process(
        this_task->features_.bass_energy
    );

    // Noise Gate Filter
    // Clears the shared frame registers immediately if below the silence threshold,
    // while perfectly preserving the internal historical peaks of the AGC engine.
    this_task->noise_gate_->process(
        raw_volume_bridge,
        this_task->features_.smoothed_volume,
        this_task->features_.bass_energy,
        this_task->features_.mid_energy,
        this_task->features_.high_energy,
        this_task->features_.is_beat_detected
    );
    this_task->features_.raw_volume = raw_volume_bridge;

    // Dual-Channel temporal peak latch window calculation
    this_task->peak_latch_->process(
        this_task->features_.is_beat_detected,
        this_task->features_.raw_volume,
        this_task->features_.sample_peak
    );

#ifdef DEBUG
    if (millis() - last_task_log >= 2000) {
      ESP_LOGD(TAG, "DEBUG_FEATURES: DSP Done -> VolRaw: %.3f | VolSmth: %.3f | Bass: %.3f | Mid: %.3f | Beat: %d | Peak: %d",
               this_task->features_.raw_volume,
               this_task->features_.smoothed_volume,
               this_task->features_.bass_energy,
               this_task->features_.mid_energy,
               this_task->features_.is_beat_detected,
               this_task->features_.sample_peak);
      last_task_log = millis();
    }
#endif

    asm volatile("memw" ::: "memory");
    this_task->ring_buffer_.advance(HOP_SIZE);
  }  // while (!(xEventGroupGetBits(this_task->event_group_) & COMMAND_STOP))

  xEventGroupSetBits(this_task->event_group_, EventGroupBits::TASK_STOPPING);
  this_task->microphone_->stop();
  xEventGroupSetBits(this_task->event_group_, EventGroupBits::TASK_STOPPED);

  while (true) {
    // Continuously delay until the loop method deletes the task
    vTaskDelay(FFT_MIN_CYCLE / portTICK_PERIOD_MS);
  }
}  // FFTcode() task end

}  // namespace esphome::music_leds
