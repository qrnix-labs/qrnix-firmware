/*
 * HF Noise Reduction — Standalone Teensy 4.0 + Audio Shield
 *
 * Extracted from Thetis SDR libspecbleach (LGPL 2.1).
 * Supports NR1 (spectral, manual noise profile) and NR2 (adaptive, always-on).
 *
 * Copyright (C) 2026 Rui Barbosa
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published
 * by the Free Software Foundation, either version 2.1 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 * Wiring:
 *   A0    — Reduction pot        (10kΩ linear, outer pins to 3.3V/GND, wiper to A0)
 *   A1    — Smoothing pot
 *   A2    — Whitening pot
 *   A3    — Aggression pot
 *   D3/D4 — Mode switch          (D3=adaptive, center=bypass, D4=spectral)
 *   D2    — Encoder button       (active LOW, internal pullup) — NR1 noise capture
 *   SDA/SCL — SSD1306 OLED        (I²C, 0x3C)
 *
 * Compile with: -DARM_MATH_CM7
 */

#include <Audio.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>

extern "C" {
#include "specbleach_denoiser.h"    // NR1 API + SpectralBleachParameters struct
// NR2 uses a different by-value parameter layout from NR1, so keep an explicit
// ABI-compatible structure instead of passing SpectralBleachParameters.
typedef struct AdaptiveSpectralBleachParameters {
    bool residual_listen;
    float reduction_amount;
    float smoothing_factor;
    float whitening_factor;
    int noise_scaling_type;
    float noise_rescale;
    float post_filter_threshold;
    bool post_filter_enabled;
    bool tone_kill_enabled;
} AdaptiveSpectralBleachParameters;
typedef struct SpectralBleachDiagnostics {
    float minimum_snr;
    float average_snr;
    float maximum_snr;
    uint32_t aggression_bands;
    uint32_t bypassed_bands;
    float average_gain;
    float average_mixed_gain;
} SpectralBleachDiagnostics;
SpectralBleachHandle specbleach_adaptive_initialize(uint32_t sample_rate, float frame_size);
void specbleach_adaptive_free(SpectralBleachHandle instance);
bool specbleach_adaptive_process(SpectralBleachHandle instance, uint32_t number_of_samples, const float *input, float *output);
bool specbleach_adaptive_load_parameters(SpectralBleachHandle instance, AdaptiveSpectralBleachParameters parameters);
uint32_t specbleach_adaptive_get_latency(SpectralBleachHandle instance);
bool specbleach_adaptive_get_diagnostics(SpectralBleachHandle instance,
                                         SpectralBleachDiagnostics *diagnostics);
#include "tonekill/tone_kill_processor.h"
}

// ── Pin map ──────────────────────────────────────────────────────────────────

#define PIN_REDUCTION      A0
#define PIN_SMOOTHING      A1
#define PIN_WHITENING      A2
#define PIN_AGGRESSION     A3
#define PIN_MODE_ADAPTIVE  3
#define PIN_MODE_SPECTRAL  4
#define PIN_ENC_BUTTON     2

// 30 dB leaves about 3.16% of the rejected spectrum's amplitude.
constexpr float REDUCTION_MAX_DB = 30.0f;
constexpr const char *SOFTWARE_VERSION = "0.3.9";
constexpr unsigned long BOOT_SPLASH_MS = 2000;

// ── Audio pipeline ───────────────────────────────────────────────────────────

AudioInputI2S        audio_input;       // Audio Shield line-in
AudioOutputI2S       audio_output;      // Audio Shield line-out
AudioRecordQueue     record_queue_l;    // captures left input blocks for processing
AudioRecordQueue     record_queue_r;    // captures right input blocks for metering
AudioPlayQueue       play_queue;        // feeds output blocks
AudioConnection      patch_in_l(audio_input, 0, record_queue_l, 0);
AudioConnection      patch_in_r(audio_input, 1, record_queue_r, 0);
AudioConnection      patch_out_l(play_queue, 0, audio_output, 0);
AudioConnection      patch_out_r(play_queue, 0, audio_output, 1);
AudioControlSGTL5000 codec;

// ── Noise reduction state ────────────────────────────────────────────────────

SpectralBleachHandle nr1     = nullptr;  // NR1 — spectral denoiser
SpectralBleachHandle nr2     = nullptr;  // NR2 — adaptive denoiser
SpectralBleachParameters params;         // shared parameter struct
int  current_mode             = 2;       // 0=OFF  1=NR1  2=NR2
bool nr1_noise_learning       = false;   // true = capturing noise profile
unsigned long nr1_capture_start = 0;     // millis() when capture started
float *nr1_cached_profile       = nullptr;
uint32_t nr1_cached_profile_size = 0;
uint32_t nr1_cached_profile_blocks = 0;

// ── Feature state (tone-kill / post-filter) ─────────────────────────────────

SpectralProcessorHandle tk_bypass = nullptr;  // lazy notch STFT, bypass+TK only
constexpr unsigned long BUTTON_DEBOUNCE_MS = 30;
constexpr unsigned long LONG_PRESS_MS = 500;

// ── Display ──────────────────────────────────────────────────────────────────

#define OLED_ADDR 0x3C
Adafruit_SSD1306 display(128, 64, &Wire, -1);
bool display_ready = false;
unsigned long boot_splash_until = 0;

// ── Audio buffers ────────────────────────────────────────────────────────────

const int BLOCK_SAMPLES = 128;
const float INPUT_GAIN = 1.0f;
const float INPUT_LEVEL_SMOOTHING = 0.05f;
const float INPUT_ACTIVITY_RMS = 64.0f;
const float INPUT_DOMINANCE_POWER_RATIO = 10.0f; // 10 dB
const unsigned long INPUT_SWITCH_CONFIRM_MS = 300;
constexpr uint16_t CLIP_THRESHOLD = 30000;      // ~0.92 full-scale: ADC saturation onset
constexpr unsigned long CLIP_LATCH_MS = 400;    // flash hold so transients stay visible
float float_in  [BLOCK_SAMPLES];
float float_out [BLOCK_SAMPLES];
int selected_input = 0;         // 0=left, 1=right
int pending_input = 0;
unsigned long input_dominance_started = 0;
float input_power_l = 0.0f;
float input_power_r = 0.0f;
uint32_t input_blocks_l = 0;
uint32_t input_blocks_r = 0;
uint16_t input_peak_l = 0;
uint16_t input_peak_r = 0;
unsigned long clip_latch_until = 0;   // millis() deadline for the CLIP flash
uint16_t output_peak_l = 0;
uint16_t output_peak_r = 0;
uint32_t output_nonfinite = 0;

// ── Forward declarations ─────────────────────────────────────────────────────

void set_default_params();
void apply_params();
bool activate_mode(int mode);
int  read_mode_switch();
void handle_button_tap();
void handle_button_hold();
void advance_feature_circle();
void start_noise_capture();
void abort_noise_capture();
void sync_tk_bypass_processor();
void update_boot_splash();
void update_display();
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max);

// ═══════════════════════════════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════════════════════════════

void setup() {
    // -- USB serial diagnostics -----------------------------------------------
    Serial.begin(115200);
    const unsigned long serial_wait_start = millis();
    while (!Serial && millis() - serial_wait_start < 3000) {
        yield();
    }
    Serial.println("setup: USB serial ready");
    if (CrashReport) {
        Serial.print(CrashReport);
    }

    // -- Audio memory pool (60 × 128-sample blocks = ~15 KB) -------------------
    AudioMemory(60);
    Serial.println("setup: audio memory ready");

    // -- Codec setup -----------------------------------------------------------
    Serial.println("setup: starting codec");
    codec.enable();
    codec.inputSelect(AUDIO_INPUT_LINEIN);
    codec.lineInLevel(15);      // maximum sensitivity (0.24 Vpp full scale)
    codec.volume(0.65);         // output level
    record_queue_l.begin();     // left input drives both line-output channels
    record_queue_r.begin();     // right input is monitored but not processed
    Serial.println("setup: codec ready");

    // -- Controls and NR initialisation ----------------------------------------
    // SPDT ON-OFF-ON switch: common to GND, outer terminals to D3 and D4.
    pinMode(PIN_MODE_ADAPTIVE, INPUT_PULLUP);
    pinMode(PIN_MODE_SPECTRAL, INPUT_PULLUP);
    pinMode(PIN_ENC_BUTTON, INPUT_PULLUP);
    set_default_params();
    current_mode = read_mode_switch();
    if (!activate_mode(current_mode)) {
        current_mode = 0;
    }

    // -- Display ---------------------------------------------------------------
    Serial.println("setup: starting display");
    display_ready = display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
    if (display_ready) {
        display.clearDisplay();
        display.setTextSize(1, 2);
        display.setTextColor(SSD1306_WHITE);
        boot_splash_until = millis() + BOOT_SPLASH_MS;
        update_boot_splash();
        Serial.println("setup: display ready");
    } else {
        Serial.println("setup: display not found (continuing without it)");
    }

    Serial.println("setup: complete");
}

// ═══════════════════════════════════════════════════════════════════════════════
//  LOOP
// ═══════════════════════════════════════════════════════════════════════════════

void loop() {
    // ── Read knobs ───────────────────────────────────────────────────────────

    params.reduction_amount = mapfloat(analogRead(PIN_REDUCTION),  0, 1023,
                                       0, REDUCTION_MAX_DB);
    params.smoothing_factor = mapfloat(analogRead(PIN_SMOOTHING),  0, 1023, 0, 100);
    params.whitening_factor = mapfloat(analogRead(PIN_WHITENING),  0, 1023, 0, 100);
    const float aggression_position =
        mapfloat(analogRead(PIN_AGGRESSION), 0, 1023, 0, 1);
    params.noise_rescale = 2.0f * aggression_position * aggression_position;

    // -- USB serial status -----------------------------------------------------
    static unsigned long last_log = 0;
    if (millis() - last_log >= 1000) {
        last_log = millis();
        SpectralBleachDiagnostics diagnostics = {};
        const bool have_diagnostics = current_mode == 2 && nr2 &&
            specbleach_adaptive_get_diagnostics(nr2, &diagnostics);
        Serial.printf("m=%d src=%c red=%.1f sm=%.1f wh=%.1f ag=%.2f tk=%d pp=%d clip=%d blk_l=%lu blk_r=%lu in_l=%u in_r=%u lvl_l=%.0f lvl_r=%.0f out_l=%u out_r=%u bad=%lu",
                      current_mode,
                      selected_input == 0 ? 'L' : 'R',
                      params.reduction_amount,
                      params.smoothing_factor,
                      params.whitening_factor,
                      params.noise_rescale,
                      params.tone_kill_enabled ? 1 : 0,
                      params.post_filter_enabled ? 1 : 0,
                      (int32_t)(clip_latch_until - millis()) > 0 ? 1 : 0,
                      input_blocks_l,
                      input_blocks_r,
                      input_peak_l,
                      input_peak_r,
                      sqrtf(input_power_l),
                      sqrtf(input_power_r),
                      output_peak_l,
                      output_peak_r,
                      output_nonfinite);
        if (have_diagnostics) {
            Serial.printf(" snr=%.1f/%.1f/%.1f bands=%lu/%lu gain=%.3f mix=%.3f",
                          diagnostics.minimum_snr,
                          diagnostics.average_snr,
                          diagnostics.maximum_snr,
                          diagnostics.aggression_bands,
                          diagnostics.bypassed_bands,
                          diagnostics.average_gain,
                          diagnostics.average_mixed_gain);
        }
        Serial.println();
        input_blocks_l = 0;
        input_blocks_r = 0;
        input_peak_l = 0;
        input_peak_r = 0;
        output_peak_l = 0;
        output_peak_r = 0;
        output_nonfinite = 0;
    }

    // ── Read mode switch ─────────────────────────────────────────────────────

    static int pending_mode = current_mode;
    static unsigned long mode_changed_at = 0;
    const int sampled_mode = read_mode_switch();
    if (sampled_mode != pending_mode) {
        pending_mode = sampled_mode;
        mode_changed_at = millis();
    }

    if (pending_mode != current_mode && millis() - mode_changed_at >= 50) {
        int new_mode = pending_mode;
        if (!activate_mode(new_mode)) {
            new_mode = 0;
        }
        current_mode = new_mode;
        apply_params();
    }

    // ── Encoder button — tap cycles features, hold runs the mode action ──────
    // Tap (< 500 ms, on release): advance the feature circle (NR modes) or
    // toggle tone-kill (bypass). Hold (>= 500 ms, at crossing, consumed):
    // capture noise floor in NR1, clear all features in bypass, no-op in NR2.

    static bool button_raw = false;
    static bool button_stable = false;
    static unsigned long button_changed_at = 0;
    static unsigned long press_started_at = 0;
    static bool long_dispatched = false;

    const bool button_sample = digitalRead(PIN_ENC_BUTTON) == LOW;
    if (button_sample != button_raw) {
        button_raw = button_sample;
        button_changed_at = millis();
    }
    if (button_raw != button_stable &&
        millis() - button_changed_at >= BUTTON_DEBOUNCE_MS) {
        button_stable = button_raw;
        if (button_stable) {
            press_started_at = millis();
            long_dispatched = false;
            // A press during an active capture cancels it (atomic: the old
            // profile is kept). The press is consumed so it cannot also
            // dispatch a tap or hold.
            if (nr1_noise_learning) {
                abort_noise_capture();
                long_dispatched = true;
            }
        } else {
            // Release: a tap advances the circle. Release never aborts an
            // active capture — a natural long press (0.5-1.4 s) must run the
            // capture to completion.
            if (!long_dispatched) {
                handle_button_tap();
            }
        }
    }
    if (button_stable && !long_dispatched &&
        millis() - press_started_at >= LONG_PRESS_MS) {
        long_dispatched = true;  // consumed: the release never dispatches a tap
        handle_button_hold();
    }

    // ── End NR1 noise capture after 1 second (atomic adopt) ─────────────────

    if (nr1_noise_learning && (millis() - nr1_capture_start > 1000)) {
        nr1_noise_learning = false;
        params.learn_noise = 0;  // OFF — use captured profile
        apply_params();
        if (nr1 && specbleach_noise_profile_available(nr1)) {
            const uint32_t profile_size = specbleach_get_noise_profile_size(nr1);
            float *captured = (float *)malloc(profile_size * sizeof(float));
            if (captured) {
                memcpy(captured, specbleach_get_noise_profile(nr1),
                       profile_size * sizeof(float));
                free(nr1_cached_profile);
                nr1_cached_profile = captured;
                nr1_cached_profile_size = profile_size;
                nr1_cached_profile_blocks =
                    specbleach_get_noise_profile_blocks_averaged(nr1);
            }
        }
        Serial.println("capture: complete");
    }

    // ── Apply params (only when changed) ─────────────────────────────────────

    static SpectralBleachParameters last_params;
    if (memcmp(&params, &last_params, sizeof(params)) != 0) {
        apply_params();
        last_params = params;
    }

    // ── Process audio block ──────────────────────────────────────────────────

    if (record_queue_l.available() >= 1 && record_queue_r.available() >= 1) {
        int16_t *in_samples_l = record_queue_l.readBuffer();
        int16_t *in_samples_r = record_queue_r.readBuffer();
        uint64_t block_energy_l = 0;
        uint64_t block_energy_r = 0;
        uint16_t block_peak_l = 0;
        uint16_t block_peak_r = 0;

        // Meter both synchronized inputs before choosing the DSP source.
        for (int i = 0; i < BLOCK_SAMPLES; i++) {
            const int32_t sample_l = in_samples_l[i];
            const int32_t sample_r = in_samples_r[i];
            const uint16_t magnitude_l = sample_l == INT16_MIN
                                       ? 32768
                                       : abs(sample_l);
            const uint16_t magnitude_r = sample_r == INT16_MIN
                                       ? 32768
                                       : abs(sample_r);
            if (magnitude_l > block_peak_l) block_peak_l = magnitude_l;
            if (magnitude_r > block_peak_r) block_peak_r = magnitude_r;
            if (magnitude_l > input_peak_l) input_peak_l = magnitude_l;
            if (magnitude_r > input_peak_r) input_peak_r = magnitude_r;
            block_energy_l += (uint64_t)(sample_l * sample_l);
            block_energy_r += (uint64_t)(sample_r * sample_r);
        }
        // ADC saturation has no hardware flag; it shows up as samples pinned at
        // the digital ceiling. Any block with a sample near full-scale latches
        // the CLIP flash (the shared input trim affects both channels alike).
        if (block_peak_l >= CLIP_THRESHOLD || block_peak_r >= CLIP_THRESHOLD) {
            clip_latch_until = millis() + CLIP_LATCH_MS;
        }
        input_blocks_l++;
        input_blocks_r++;

        const float block_power_l = (float)block_energy_l / BLOCK_SAMPLES;
        const float block_power_r = (float)block_energy_r / BLOCK_SAMPLES;
        input_power_l += INPUT_LEVEL_SMOOTHING * (block_power_l - input_power_l);
        input_power_r += INPUT_LEVEL_SMOOTHING * (block_power_r - input_power_r);

        const float activity_power = INPUT_ACTIVITY_RMS * INPUT_ACTIVITY_RMS;
        int desired_input = selected_input;
        if (input_power_l >= activity_power || input_power_r >= activity_power) {
            // Right must be at least 10 dB stronger; otherwise left has priority.
            desired_input = input_power_r > input_power_l * INPUT_DOMINANCE_POWER_RATIO
                          ? 1
                          : 0;
        }

        if (desired_input == selected_input) {
            pending_input = selected_input;
        } else if (desired_input != pending_input) {
            pending_input = desired_input;
            input_dominance_started = millis();
        } else if (millis() - input_dominance_started >= INPUT_SWITCH_CONFIRM_MS) {
            selected_input = desired_input;
        }

        const int16_t *selected_samples = selected_input == 0
                                        ? in_samples_l
                                        : in_samples_r;
        for (int i = 0; i < BLOCK_SAMPLES; i++) {
            float amplified = (selected_samples[i] / 32768.0f) * INPUT_GAIN;
            if (amplified > 1.0f) amplified = 1.0f;
            if (amplified < -1.0f) amplified = -1.0f;
            float_in[i] = amplified;
        }
        record_queue_l.freeBuffer();
        record_queue_r.freeBuffer();

        // Dispatch
        switch (current_mode) {
        case 2:  // NR2 — adaptive
            if (!nr2 || !specbleach_adaptive_process(nr2, BLOCK_SAMPLES, float_in, float_out)) {
                memcpy(float_out, float_in, BLOCK_SAMPLES * sizeof(float));
            }
            break;
        case 1:  // NR1 — spectral
            if (!nr1 || !specbleach_process(nr1, BLOCK_SAMPLES, float_in, float_out)) {
                memcpy(float_out, float_in, BLOCK_SAMPLES * sizeof(float));
            }
            break;
        default: // OFF — bypass
            if (params.tone_kill_enabled && tk_bypass) {
                if (!tone_kill_processor_process(tk_bypass, BLOCK_SAMPLES,
                                                 float_in, float_out)) {
                    memcpy(float_out, float_in, BLOCK_SAMPLES * sizeof(float));
                }
            } else {
                memcpy(float_out, float_in, BLOCK_SAMPLES * sizeof(float));
            }
            break;
        }

        // float → int16
        int16_t *out_samples = play_queue.getBuffer();
        for (int i = 0; i < BLOCK_SAMPLES; i++) {
            float sample = float_out[i];
            if (!isfinite(sample)) {
                sample = 0.0f;
                output_nonfinite++;
            }
            if (sample > 1.0f) sample = 1.0f;
            if (sample < -1.0f) sample = -1.0f;
            const uint16_t magnitude = (uint16_t)(fabsf(sample) * 32767.0f);
            if (magnitude > output_peak_l) output_peak_l = magnitude;
            if (magnitude > output_peak_r) output_peak_r = magnitude;
            out_samples[i] = (int16_t)(sample * 32767.0f);
        }
        play_queue.playBuffer();
    }

    // ── Update display (10 Hz) ───────────────────────────────────────────────

    static unsigned long last_display = 0;
    if (millis() - last_display > 100) {
        if ((int32_t)(boot_splash_until - millis()) > 0) {
            update_boot_splash();
        } else {
            update_display();
        }
        last_display = millis();
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
//  HELPERS
// ═══════════════════════════════════════════════════════════════════════════════

void set_default_params() {
    params = (SpectralBleachParameters){
        .learn_noise          = 0,       // NR2 ignores this; NR1 = use profile
        .residual_listen      = false,
        .reduction_amount     = 20.0f,   // dB
        .smoothing_factor     = 50.0f,   // percent
        .transient_protection = false,   // NR2 ignores this
        .whitening_factor     = 30.0f,   // percent
        .noise_scaling_type   = 1,       // 0=SNR  1=critical bands  2=masking
        .noise_rescale        = 0.5f,    // additive oversubtraction strength
        .post_filter_threshold = 0.0f,   // dB; Thetis C# production default.
                                         // 0 dB engages whenever suppression is
                                         // active (ratio < 1); -10 dB (raw C
                                         // default) is dormant over the normal
                                         // operating range.
        .post_filter_enabled   = false,  // runtime gate; define stays compile-time gate
        .tone_kill_enabled     = false,
    };
}

void apply_params() {
    if (nr2) {
        const AdaptiveSpectralBleachParameters adaptive_params = {
            .residual_listen = params.residual_listen,
            .reduction_amount = params.reduction_amount,
            .smoothing_factor = params.smoothing_factor,
            .whitening_factor = params.whitening_factor,
            .noise_scaling_type = params.noise_scaling_type,
            .noise_rescale = params.noise_rescale,
            .post_filter_threshold = params.post_filter_threshold,
            .post_filter_enabled = params.post_filter_enabled,
            .tone_kill_enabled = params.tone_kill_enabled,
        };
        specbleach_adaptive_load_parameters(nr2, adaptive_params);
    }
    if (nr1) specbleach_load_parameters(nr1, params);
}

// ── Feature helpers ──────────────────────────────────────────────────────────

void advance_feature_circle() {
    const bool tk = params.tone_kill_enabled;
    const bool pp = params.post_filter_enabled;
    if (!tk && !pp) {
        params.tone_kill_enabled = true;          // none -> TK
    } else if (tk && !pp) {
        params.post_filter_enabled = true;        // TK -> TK+PP
    } else if (tk && pp) {
        params.tone_kill_enabled = false;         // TK+PP -> PP
    } else {
        params.post_filter_enabled = false;       // PP -> none
    }
}

void sync_tk_bypass_processor() {
    const bool needed = current_mode == 0 && params.tone_kill_enabled;
    if (needed && !tk_bypass) {
        AudioNoInterrupts();
        tk_bypass = tone_kill_processor_initialize(44100);
        AudioInterrupts();
        if (!tk_bypass) {
            // Never show an armed flag the DSP cannot honor.
            params.tone_kill_enabled = false;
            Serial.println("tk: ERROR - bypass processor allocation failed");
        }
    } else if (!needed && tk_bypass) {
        AudioNoInterrupts();
        tone_kill_processor_free(tk_bypass);
        tk_bypass = nullptr;
        AudioInterrupts();
    }
}

void handle_button_tap() {
    if (nr1_noise_learning) return;  // guarded during capture
    if (current_mode == 0) {
        // Bypass: 2-state circle — TK only. PP is unreachable here by design.
        params.tone_kill_enabled = !params.tone_kill_enabled;
        sync_tk_bypass_processor();
    } else {
        advance_feature_circle();
    }
}

void handle_button_hold() {
    if (nr1_noise_learning) return;
    if (current_mode == 1) {
        start_noise_capture();
    } else if (current_mode == 0) {
        // Bypass: deliberate clear-all escape hatch for both feature flags.
        params.tone_kill_enabled = false;
        params.post_filter_enabled = false;
        sync_tk_bypass_processor();
        Serial.println("tk: cleared");
    }
    // NR2: hold is a no-op.
}

void start_noise_capture() {
    if (current_mode != 1 || !nr1 || nr1_noise_learning) return;
    specbleach_reset_noise_profile(nr1);  // fresh internal profile; cache intact
    nr1_noise_learning = true;
    nr1_capture_start = millis();
    params.learn_noise = 2;
    apply_params();
    Serial.println("capture: start");
}

void abort_noise_capture() {
    if (!nr1_noise_learning) return;
    nr1_noise_learning = false;
    params.learn_noise = 0;
    apply_params();
    // The partial capture is discarded; the last good profile (if any) is
    // restored so the abort is a true no-op.
    if (nr1_cached_profile) {
        specbleach_load_noise_profile(nr1, nr1_cached_profile,
                                      nr1_cached_profile_size,
                                      nr1_cached_profile_blocks);
    } else {
        specbleach_reset_noise_profile(nr1);
    }
    Serial.println("capture: aborted");
}

bool activate_mode(int mode) {
    // A 2048-point NR1 and NR2 do not fit in the Teensy 4.0 heap together.
    // Stop audio callbacks while replacing the active DSP processor.
    AudioNoInterrupts();

    if (nr1) {
        if (specbleach_noise_profile_available(nr1)) {
            const uint32_t profile_size =
                specbleach_get_noise_profile_size(nr1);
            float *cached_profile =
                (float *)malloc(profile_size * sizeof(float));
            if (cached_profile) {
                memcpy(cached_profile, specbleach_get_noise_profile(nr1),
                       profile_size * sizeof(float));
                free(nr1_cached_profile);
                nr1_cached_profile = cached_profile;
                nr1_cached_profile_size = profile_size;
                nr1_cached_profile_blocks =
                    specbleach_get_noise_profile_blocks_averaged(nr1);
            }
        }
        specbleach_free(nr1);
        nr1 = nullptr;
    }
    if (nr2) {
        specbleach_adaptive_free(nr2);
        nr2 = nullptr;
    }
    if (tk_bypass) {
        tone_kill_processor_free(tk_bypass);
        tk_bypass = nullptr;
    }

    bool ready = true;
    if (mode == 1) {
        Serial.println("mode: starting NR1");
        nr1 = specbleach_initialize(44100, 25.0f);
        ready = nr1 != nullptr;
        if (ready && nr1_cached_profile) {
            ready = specbleach_load_noise_profile(
                nr1, nr1_cached_profile, nr1_cached_profile_size,
                nr1_cached_profile_blocks);
        }
        Serial.println(ready ? "mode: NR1 ready" : "mode: ERROR - NR1 allocation failed");
    } else if (mode == 2) {
        Serial.println("mode: starting NR2");
        nr2 = specbleach_adaptive_initialize(44100, 25.0f);
        ready = nr2 != nullptr;
        Serial.println(ready ? "mode: NR2 ready" : "mode: ERROR - NR2 allocation failed");
    } else {
        Serial.println("mode: bypass");
    }

    apply_params();
    AudioInterrupts();
    // Entering bypass with tone-kill armed needs the lazy notch processor.
    sync_tk_bypass_processor();
    return ready;
}

int read_mode_switch() {
    const bool adaptive_selected = digitalRead(PIN_MODE_ADAPTIVE) == LOW;
    const bool spectral_selected = digitalRead(PIN_MODE_SPECTRAL) == LOW;

    // Treat invalid/both-active wiring as bypass for safety.
    if (adaptive_selected && spectral_selected) return 0;
    if (adaptive_selected) return 2;
    if (spectral_selected) return 1;
    return 0;  // Center-off position: neither outer terminal is connected.
}

void update_boot_splash() {
    if (!display_ready) return;

    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("QRNix");
    display.println("HF Noise Reduction");
    display.print("Firmware v");
    display.println(SOFTWARE_VERSION);
    display.print("Auto Input: ");
    display.println(selected_input == 0 ? "LEFT" : "RIGHT");
    display.display();
}

void update_display() {
    if (!display_ready) return;

    display.clearDisplay();
    display.setCursor(0, 0);

    // Line 1 — mode; the whole line inverts as the CLIP indicator
    const char *mode_str = (current_mode == 2) ? "QRNix Adaptive"
                         : (current_mode == 1) ? "QRNix Spectral"
                         :                       "QRNix Bypass";
    if ((int32_t)(clip_latch_until - millis()) > 0) {
        display.fillRect(0, 0, 128, 16, SSD1306_WHITE);
        display.setTextColor(SSD1306_BLACK);
    } else {
        display.setTextColor(SSD1306_WHITE);
    }
    display.println(mode_str);
    display.setTextColor(SSD1306_WHITE);

    // Line 2 — reduction bar
    display.print("Red: ");
    int bar = (int)(params.reduction_amount / REDUCTION_MAX_DB * 80);
    for (int i = 0; i < bar / 8; i++) display.print("\xDB");  // full block
    display.print(" ");
    display.print((int)roundf(params.reduction_amount));
    display.println("dB");

    // Line 3 — smoothing + whitening
    display.print("Sm:");
    display.print((int)params.smoothing_factor);
    display.print("%  Wh:");
    display.print((int)params.whitening_factor);
    display.println("%");

    // Line 4 — aggression + status + feature indicators
    // Status (Prof/Learn/None/Auto/Bypass) sits at a fixed position after the
    // value; indicators follow in TK, PP order. Reversed video = feature
    // active in this mode; normal-video PP in bypass = armed but dormant
    // (the post-filter has no effect there).
    display.setCursor(0, 48);
    display.print("Ag:");
    display.print(params.noise_rescale, 2);

    int16_t x = 48;  // "Ag:1.93" (42 px) + separator space
    const char *status = "Bypass";
    if (nr1_noise_learning) {
        status = "Learn";
    } else if (current_mode == 1 && specbleach_noise_profile_available(nr1)) {
        status = "Prof";
    } else if (current_mode == 1) {
        status = "None";
    } else if (current_mode == 2) {
        status = "Auto";
    }
    display.setCursor(x, 48);
    display.print(status);
    x += (int16_t)(strlen(status) * 6);

    if (params.tone_kill_enabled) {
        x += 6;  // separator
        display.fillRect(x, 48, 12, 16, SSD1306_WHITE);
        display.setCursor(x, 48);
        display.setTextColor(SSD1306_BLACK);
        display.print("TK");
        display.setTextColor(SSD1306_WHITE);
        x += 12;
    }
    if (params.post_filter_enabled) {
        x += 6;
        if (current_mode == 0) {
            // Armed but dormant: normal video, never reversed.
            display.setCursor(x, 48);
            display.print("PP");
        } else {
            display.fillRect(x, 48, 12, 16, SSD1306_WHITE);
            display.setCursor(x, 48);
            display.setTextColor(SSD1306_BLACK);
            display.print("PP");
            display.setTextColor(SSD1306_WHITE);
        }
    }

    display.display();
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
