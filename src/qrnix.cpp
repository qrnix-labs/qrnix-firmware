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
#include <string.h>

#include "serial_contract.h"
#include "serial_identity.h"
#include "autotune/tuner_core.h"
#include "autotune/catch_lock.h"

// OCOTP read wrapper — defined in serial_identity_teensy.cpp (Teensy-only).
void serial_identity_early_init(void);

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
#define PIN_VOLUME         A8

// 30 dB leaves about 3.16% of the rejected spectrum's amplitude.
constexpr float REDUCTION_MAX_DB = 30.0f;
constexpr const char *SOFTWARE_VERSION = "0.3.90";
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

// ── Auto-tune state (issue 8) ────────────────────────────────────────────────

int16_t *tune_ring = nullptr;            // raw int16 capture ring, allocated once
uint32_t tune_ring_pos = 0;              // wrap write position during capture
bool tune_active = false;                // tune run in progress (stub until issue 9)
int tune_notice = 0;                     // persistent status chip: 0 none, 1 CLIP, 2 QUIET
unsigned long tune_notice_until = 0;     // full-screen notice deadline (millis)
bool capture_clipped = false;            // clip seen during the capture window
uint64_t capture_power_sum = 0;          // quiet-guard accumulator (selected channel)
uint32_t capture_blocks = 0;             // blocks metered during the capture window

// ── Auto-tune run (issue 9) ──────────────────────────────────────────────────

TunerCore tune_core;                     // resumable search state (issue 6)
TunerConfig tune_config;                 // search configuration (built at boot)
bool tuned_latch = false;                // completed tune governs the four params
float tune_score_db = 0.0f;              // last completed tune score
bool saved_tk = false;                   // TK/PP flags restored after the run
bool saved_pp = false;
uint8_t tune_last_tenth = 0;             // progress serial throttle (10% steps)
uint8_t tune_progress_pct = 0;           // search progress for the display
TunerParams tune_last_candidate;         // adapter: last candidate loaded
bool tune_candidate_loaded = false;      // adapter: per-candidate setup done
SpectralBleachParameters last_params;    // loop apply-change detection
CatchLock tune_lock;                     // per-param catch windows (issue 12)

// ── Feature state (tone-kill / post-filter) ─────────────────────────────────

SpectralProcessorHandle tk_bypass = nullptr;  // lazy notch STFT, bypass+TK only
constexpr unsigned long BUTTON_DEBOUNCE_MS = 30;
constexpr unsigned long LONG_PRESS_MS = 500;
// Volume pot: commit at most once per 50 ms and only outside a 3 LSB
// noise band, so I2C codec writes do not happen per loop iteration.
constexpr unsigned long VOLUME_WRITE_MIN_MS = 50;
constexpr int VOLUME_HYSTERESIS_LSB = 3;

// ── Auto-tune (issue 8): capture ring, guards, tune skeleton ────────────────

constexpr uint32_t TUNE_RING_CAPACITY = 44100;        // 1 s @ 44.1 kHz (~88 KB)
constexpr uint64_t QUIET_POWER_THRESHOLD = 107374ull; // per-sample power of RMS 327.68 (~-40 dBFS)
constexpr unsigned long TUNE_NOTICE_MS = 3000;        // full-screen guard notice
constexpr unsigned long TUNE_CANCEL_NOTICE_MS = 1500; // brief press-cancel notice

// Audition gate (issue 11): the configurations header owns the default;
// this fallback keeps the sketch self-contained when built without the
// vendored header in the include path. Production overrides with
// -DTUNE_AUDITION_ENABLED=0.
#ifndef TUNE_AUDITION_ENABLED
#define TUNE_AUDITION_ENABLED 1
#endif

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
void start_tune();
void abort_tune();
void show_tune_notice(int mode);
void show_tune_cancel_notice();
void clear_tune_notice();
static uint32_t tune_now_ms(void);
static bool tune_process_block(const float *input, uint32_t samples,
                               const TunerParams *tp, float *output,
                               void *userdata);
static void tune_progress_cb(float fraction, const TunerParams *current,
                             float score_db, void *userdata);
void tune_search_advance();
void sync_tk_bypass_processor();
void update_boot_splash();
void update_display();
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max);
// ── Wire contract emission (ADR-0003) ───────────────────────────────────────

static void emit_boot_line(const char *stage) {
    char line[160];
    contract_boot_line(line, sizeof(line), stage);
    Serial.println(line);
}

static void emit_crash_line(const char *detail) {
    char line[512];
    contract_crash_line(line, sizeof(line), detail);
    Serial.println(line);
}

// Captures a Printable (CrashReport) into a buffer for per-line emission.
struct BufPrint : public Print {
    char *buf;
    size_t cap;
    size_t n;
    BufPrint(char *b, size_t c) : buf(b), cap(c), n(0) {}
    size_t write(uint8_t byte) override {
        if (n + 1 < cap) {
            buf[n++] = (char)byte;
        }
        return 1;
    }
};

// ═══════════════════════════════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════════════════════════════

void setup() {
    // -- Unit serial identity (ADR-0006) ---------------------------------------
    // OCOTP shadow registers, read once: the value is factory-blown and
    // constant for the life of the chip. The very first status envelope
    // already carries it (PRD scenario A3).
    serial_identity_early_init();

    // -- USB serial diagnostics -----------------------------------------------
    Serial.begin(115200);
    const unsigned long serial_wait_start = millis();
    while (!Serial && millis() - serial_wait_start < 3000) {
        yield();
    }
    emit_boot_line("USB serial ready");
    if (CrashReport) {
        char report[512];
        BufPrint sink(report, sizeof(report));
        CrashReport.printTo(sink);
        report[sink.n] = '\0';
        char *line = report;
        for (char *nl = strchr(line, '\n'); nl != NULL; nl = strchr(line, '\n')) {
            *nl = '\0';
            emit_crash_line(line);
            line = nl + 1;
        }
        if (*line != '\0') {
            emit_crash_line(line);
        }
    }

    // -- Audio memory pool (60 × 128-sample blocks = ~15 KB) -------------------
    AudioMemory(60);
    emit_boot_line("audio memory ready");

    // -- Codec setup -----------------------------------------------------------
    emit_boot_line("starting codec");
    codec.enable();
    codec.inputSelect(AUDIO_INPUT_LINEIN);
    codec.lineInLevel(15);      // maximum sensitivity (0.24 Vpp full scale)
    codec.volume(0.65);         // output level
    record_queue_l.begin();     // left input drives both line-output channels
    record_queue_r.begin();     // right input is monitored but not processed
    emit_boot_line("codec ready");

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

    // -- Auto-tune capture ring (1 s int16, ~88 KB) ----------------------------
    // Allocated once at boot: a mid-run failure would otherwise strand a
    // capture without a tune path. Tune simply stays unavailable if it fails.
    tune_ring = (int16_t *)malloc(TUNE_RING_CAPACITY * sizeof(int16_t));
    if (tune_ring) {
        emit_boot_line("tune ring ready");
    } else {
        Serial.println("tune: ERROR - ring allocation failed");
        emit_boot_line("tune ring unavailable");
    }

    // -- Auto-tune search core (issue 9) ---------------------------------------
    memset(&tune_config, 0, sizeof(tune_config));
    tune_config.discard_samples = 4410;    // ~100 ms convergence discard
    tune_config.eval_samples = 13230;      // ~0.3 s scoring window
    tune_config.time_budget_ms = 45000;    // hard search budget
    tune_config.start.reduction_db = 10.0f;
    tune_config.start.smoothing_pct = 10.0f;
    tune_config.start.whitening_pct = 10.0f;
    tune_config.start.noise_rescale = 0.20f;
    tune_config.now_ms = tune_now_ms;
    tuner_core_init(&tune_core, &tune_config);

    // -- Display ---------------------------------------------------------------
    emit_boot_line("starting display");
    display_ready = display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
    if (display_ready) {
        display.clearDisplay();
        display.setTextSize(1, 2);
        display.setTextColor(SSD1306_WHITE);
        boot_splash_until = millis() + BOOT_SPLASH_MS;
        update_boot_splash();
        emit_boot_line("display ready");
    } else {
        emit_boot_line("display not found (continuing without it)");
    }

    emit_boot_line("complete");
}

// ═══════════════════════════════════════════════════════════════════════════════
//  LOOP
// ═══════════════════════════════════════════════════════════════════════════════

void loop() {
#ifdef TUNE_TEST_TRIGGER
    // Test-build-only serial trigger (issue 9): simulates the button
    // gestures so the full capture + tune loop is automatable over serial.
    // Compiled out of production builds (define via PLATFORMIO_BUILD_FLAGS).
    while (Serial.available() > 0) {
        char cmd[24];
        const int n = Serial.readBytesUntil('\n', cmd, sizeof(cmd) - 1);
        if (n <= 0) continue;
        cmd[n] = '\0';
        if (strcmp(cmd, "tune:test") == 0) {
            if (current_mode == 1) {
                handle_button_hold();  // same path as the physical hold
            } else {
                Serial.println("tune: ERROR - trigger needs NR1 mode");
            }
        } else if (strcmp(cmd, "tune:cancel") == 0) {
            if (tune_active) {
                abort_tune();
            } else if (nr1_noise_learning) {
                abort_noise_capture();
            }
        }
    }
#endif

    // ── Read knobs ───────────────────────────────────────────────────────────
    // A completed tune governs the four params (pots ignored) until each
    // pot parks inside its catch window; a running search must not let pot
    // reads push mid-candidate parameters into the denoiser.
    const uint16_t pot_red = analogRead(PIN_REDUCTION);
    const uint16_t pot_sm = analogRead(PIN_SMOOTHING);
    const uint16_t pot_wh = analogRead(PIN_WHITENING);
    const uint16_t pot_ag = analogRead(PIN_AGGRESSION);
    // Volume (A8) — standalone output control, never a tune parameter:
    // independent of the tuned_latch / catch-window logic above and below.
    // codec.volume(0.65) in setup() is the pre-loop default; the first
    // committed read replaces it. Linear pot on the codec's digital volume
    // (0.5 dB steps) gives a conventional audio-taper feel.
    static bool vol_init = false;
    static uint16_t last_vol_raw = 0;
    static unsigned long last_vol_write = 0;
    static int volume_pct = 65;  // matches the codec.volume(0.65) boot default
    const uint16_t vol_raw = (uint16_t)analogRead(PIN_VOLUME);
    const int vol_delta = (int)vol_raw - (int)last_vol_raw;
    if (!vol_init ||
        ((vol_delta >= VOLUME_HYSTERESIS_LSB ||
          vol_delta <= -VOLUME_HYSTERESIS_LSB) &&
         millis() - last_vol_write >= VOLUME_WRITE_MIN_MS)) {
        vol_init = true;
        last_vol_raw = vol_raw;
        last_vol_write = millis();
        codec.volume(vol_raw * (1.0f / 1023.0f));
        volume_pct = (int)lroundf(vol_raw * (100.0f / 1023.0f));
    }
    if (tuned_latch && !tune_active) {
        // Catch-window handoff (issue 12): each pot that parks inside its
        // window takes over its parameter; all four unlocked returns the
        // box to manual pots.
        const uint16_t pots[CATCH_PARAM_COUNT] = {pot_red, pot_sm, pot_wh, pot_ag};
        const uint16_t mask_before = tune_lock.lock_mask;
        const bool all_unlocked = catch_lock_update(&tune_lock, pots);
        for (int p = 0; p < CATCH_PARAM_COUNT; p++) {
            const uint16_t bit = (uint16_t)(1u << p);
            if ((mask_before & bit) != 0 && (tune_lock.lock_mask & bit) == 0) {
                const char *name = p == CATCH_PARAM_REDUCTION ? "red"
                                 : p == CATCH_PARAM_SMOOTHING ? "sm"
                                 : p == CATCH_PARAM_WHITENING ? "wh"
                                 :                             "ag";
                // Handed-over value (issue 24): the pot-mapped parameter at
                // handover, matching the `tune: complete` vocabulary so the
                // no-jump check is exact without cross-referencing cadence.
                float handover = 0.0f;
                int prec = 1;
                if (p == CATCH_PARAM_REDUCTION) {
                    handover = mapfloat(pots[p], 0, 1023, 0, REDUCTION_MAX_DB);
                } else if (p == CATCH_PARAM_SMOOTHING || p == CATCH_PARAM_WHITENING) {
                    handover = mapfloat(pots[p], 0, 1023, 0, 100);
                    prec = 0;
                } else {
                    const float position = mapfloat(pots[p], 0, 1023, 0, 1);
                    handover = 2.0f * position * position;
                    prec = 2;
                }
                Serial.print("tune: unlock ");
                Serial.print(name);
                Serial.print(" val=");
                Serial.print(handover, prec);
                Serial.println();
            }
        }
        if (!catch_lock_is_locked(&tune_lock, CATCH_PARAM_REDUCTION)) {
            params.reduction_amount = mapfloat(pot_red, 0, 1023, 0, REDUCTION_MAX_DB);
        }
        if (!catch_lock_is_locked(&tune_lock, CATCH_PARAM_SMOOTHING)) {
            params.smoothing_factor = mapfloat(pot_sm, 0, 1023, 0, 100);
        }
        if (!catch_lock_is_locked(&tune_lock, CATCH_PARAM_WHITENING)) {
            params.whitening_factor = mapfloat(pot_wh, 0, 1023, 0, 100);
        }
        if (!catch_lock_is_locked(&tune_lock, CATCH_PARAM_AGGRESSION)) {
            const float aggression_position = mapfloat(pot_ag, 0, 1023, 0, 1);
            params.noise_rescale = 2.0f * aggression_position * aggression_position;
        }
        if (all_unlocked) {
            tuned_latch = false;  // the box returns to manual pots
            Serial.println("tune: unlock all - manual");
        }
    } else if (!tuned_latch && !tune_active) {
        params.reduction_amount = mapfloat(pot_red, 0, 1023, 0, REDUCTION_MAX_DB);
        params.smoothing_factor = mapfloat(pot_sm, 0, 1023, 0, 100);
        params.whitening_factor = mapfloat(pot_wh, 0, 1023, 0, 100);
        const float aggression_position = mapfloat(pot_ag, 0, 1023, 0, 1);
        params.noise_rescale = 2.0f * aggression_position * aggression_position;
    }

    // -- USB serial status -----------------------------------------------------
    static unsigned long last_log = 0;
    if (millis() - last_log >= 1000) {
        last_log = millis();
        SpectralBleachDiagnostics diagnostics = {};
        const bool have_diagnostics = current_mode == 2 && nr2 &&
            specbleach_adaptive_get_diagnostics(nr2, &diagnostics);
        ContractStatus st;
        st.mode = current_mode;
        st.src = selected_input == 0 ? 'L' : 'R';
        st.red = (int)lroundf(params.reduction_amount);
        st.sm = (int)lroundf(params.smoothing_factor);
        st.wh = (int)lroundf(params.whitening_factor);
        st.ag = (int)lroundf(params.noise_rescale);
        st.vol = volume_pct;
        st.lk = tuned_latch ? (int)tune_lock.lock_mask : 0;
        st.tk = params.tone_kill_enabled ? 1 : 0;
        st.pp = params.post_filter_enabled ? 1 : 0;
        st.clip = (int32_t)(clip_latch_until - millis()) > 0 ? 1 : 0;
        st.blk_l = input_blocks_l;
        st.blk_r = input_blocks_r;
        st.in_l = input_peak_l;
        st.in_r = input_peak_r;
        st.lvl_l = (int)lroundf(sqrtf(input_power_l));
        st.lvl_r = (int)lroundf(sqrtf(input_power_r));
        st.out_l = output_peak_l;
        st.out_r = output_peak_r;
        st.bad = output_nonfinite;
        st.have_tail = have_diagnostics ? 1 : 0;
        st.snr_min = diagnostics.minimum_snr;
        st.snr_avg = diagnostics.average_snr;
        st.snr_max = diagnostics.maximum_snr;
        st.bands_aggression = diagnostics.aggression_bands;
        st.bands_bypassed = diagnostics.bypassed_bands;
        st.gain = diagnostics.average_gain;
        st.mix = diagnostics.average_mixed_gain;
        st.up = (uint64_t)(millis() / 1000);
        st.ver = SOFTWARE_VERSION;
        st.sn = serial_identity_get();
        char line[512];
        contract_status_line(line, sizeof(line), &st);
        Serial.println(line);
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
        // A mode switch aborts a running tune (per-session preset rules
        // arrive with the tune latch in issue 13) and clears the guard chip.
        if (tune_active) {
            abort_tune();
        }
        clear_tune_notice();
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
            } else if (tune_active) {
                // A press during a tune cancels it with prior state restored.
                abort_tune();
                long_dispatched = true;
            }
            if (long_dispatched) {
                // Any press action clears the persistent guard chip and
                // shows the brief cancel notice.
                clear_tune_notice();
                show_tune_cancel_notice();
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
        if (tune_ring) {
            // Report the raw ring stats so the capture side is automatable.
            Serial.print("tune: ring ");
            Serial.print(capture_blocks * BLOCK_SAMPLES);
            Serial.println(" samples");
        }

        // Auto-tune guards (issue 8): clip or quiet during the capture
        // window aborts the tune path; the profile is kept either way.
        const bool clipped = capture_clipped;
        const bool quiet = capture_blocks > 0 &&
            capture_power_sum <
                (uint64_t)capture_blocks * QUIET_POWER_THRESHOLD * BLOCK_SAMPLES;
        capture_clipped = false;
        capture_power_sum = 0;
        capture_blocks = 0;
        if (clipped) {
            show_tune_notice(1);  // CLIP chip
            Serial.println("tune: abort clip");
        } else if (quiet) {
            show_tune_notice(2);  // QUIET chip
            Serial.println("tune: abort quiet");
        } else {
            start_tune();  // capture leads into the real search (issue 9)
        }
    }

    // ── Step the tune search ─────────────────────────────────────────────────
    // Silent mode: advance one 128-sample block per loop iteration (DSP
    // speed), keeping the display, button, and serial live for the whole
    // budgeted run. Audition mode (issue 11) paces the search from the
    // audio dispatch below instead; this driver stays idle there.
#if !TUNE_AUDITION_ENABLED
    tune_search_advance();
#endif

    // ── Apply params (only when changed) ─────────────────────────────────────
    // Skipped while a search runs: the adapter owns the NR1 parameters
    // mid-candidate; the completion/abort paths sync `last_params` and
    // re-arm the live state.

    if (!tune_active && memcmp(&params, &last_params, sizeof(params)) != 0) {
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
            if (nr1_noise_learning) {
                capture_clipped = true;  // tune guard: clip aborts the tune path
            }
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
        if (nr1_noise_learning) {
            // Issue 8: record the raw capture into the tune ring (wrap at
            // capacity) and meter the selected channel for the quiet guard.
            const uint64_t selected_energy = selected_input == 0
                                           ? block_energy_l
                                           : block_energy_r;
            capture_power_sum += selected_energy;
            capture_blocks++;
            if (tune_ring) {
                for (int i = 0; i < BLOCK_SAMPLES; i++) {
                    tune_ring[tune_ring_pos] = selected_samples[i];
                    tune_ring_pos = (tune_ring_pos + 1) % TUNE_RING_CAPACITY;
                }
            }
        }
        for (int i = 0; i < BLOCK_SAMPLES; i++) {
            float amplified = (selected_samples[i] / 32768.0f) * INPUT_GAIN;
            if (amplified > 1.0f) amplified = 1.0f;
            if (amplified < -1.0f) amplified = -1.0f;
            float_in[i] = amplified;
        }
        record_queue_l.freeBuffer();
        record_queue_r.freeBuffer();

        // Dispatch
        if (tune_active) {
#if TUNE_AUDITION_ENABLED
            // Audition mode (issue 11): the search is paced by the input
            // clock — one step per audio block — and the processed test
            // signal replaces live passthrough for the duration.
            tune_search_advance();
            memcpy(float_out, tune_core.output_buf, sizeof(float_out));
#else
            // Silent mode: the search runs at DSP speed from the loop
            // driver; live audio is silent for the duration.
            memset(float_out, 0, sizeof(float_out));
#endif
        } else switch (current_mode) {
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
    clear_tune_notice();  // any button action clears the guard chip
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
    clear_tune_notice();  // any button action clears the guard chip
    if (current_mode == 1) {
        start_noise_capture();
    } else if (current_mode == 0) {
        // Bypass: deliberate clear-all escape hatch for both feature flags
        // and any completed tune preset (issue 13): the pots govern again.
        params.tone_kill_enabled = false;
        params.post_filter_enabled = false;
        sync_tk_bypass_processor();
        Serial.println("tk: cleared");
        if (tuned_latch) {
            tuned_latch = false;
            Serial.println("tune: cleared");
        }
    }
    // NR2: hold is a no-op.
}

void start_noise_capture() {
    if (current_mode != 1 || !nr1 || nr1_noise_learning) return;
    if (tune_active) {
        abort_tune();  // re-capture replaces a pending tune
    }
    clear_tune_notice();
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

// ── Auto-tune run (issue 9) ──────────────────────────────────────────────────

static uint32_t tune_now_ms(void) {
    return (uint32_t)millis();
}

// DSP adapter: per candidate, restore the captured noise profile and load
// the candidate parameters into the NR1 instance, then process one block.
// TK and PP are forced off so they cannot fight the reference tones.
static bool tune_process_block(const float *input, uint32_t samples,
                               const TunerParams *tp, float *output,
                               void *userdata) {
    (void)userdata;
    if (!nr1 || samples == 0) {
        return false;
    }
    if (!tune_candidate_loaded ||
        memcmp(&tune_last_candidate, tp, sizeof(TunerParams)) != 0) {
        tune_candidate_loaded = true;
        tune_last_candidate = *tp;
        specbleach_reset_noise_profile(nr1);
        if (nr1_cached_profile) {
            specbleach_load_noise_profile(nr1, nr1_cached_profile,
                                          nr1_cached_profile_size,
                                          nr1_cached_profile_blocks);
        }
        SpectralBleachParameters candidate = params;
        candidate.reduction_amount = tp->reduction_db;
        candidate.smoothing_factor = tp->smoothing_pct;
        candidate.whitening_factor = tp->whitening_pct;
        candidate.noise_rescale = tp->noise_rescale;
        candidate.tone_kill_enabled = false;   // forced off during the search
        candidate.post_filter_enabled = false;
        specbleach_load_parameters(nr1, candidate);
    }
    return specbleach_process(nr1, samples, input, output);
}
// Periodic serial progress (one line per 10%); the display reads the
// latest percentage for the "Tuning NN%" state (issue 10).
static void tune_progress_cb(float fraction, const TunerParams *current,
                             float score_db, void *userdata) {
    (void)current;
    (void)score_db;
    (void)userdata;
    tune_progress_pct = (uint8_t)(fraction * 100.0f);
    const uint8_t tenth = (uint8_t)(fraction * 10.0f);
    if (tenth != tune_last_tenth) {
        tune_last_tenth = tenth;
        Serial.print("tune: progress ");
        Serial.print((int)(fraction * 100.0f));
        Serial.println("%");
    }
}
// Apply the tuned parameters, restore TK/PP, and latch the tuned state
// (the pots are ignored from here on; per-param catch windows land in
// issue 12, exit semantics in issue 13).
static void apply_tune_result(const TunerResult *result) {
    params.reduction_amount = result->params.reduction_db;
    params.smoothing_factor = result->params.smoothing_pct;
    params.whitening_factor = result->params.whitening_pct;
    params.noise_rescale = result->params.noise_rescale;
    params.tone_kill_enabled = saved_tk;
    params.post_filter_enabled = saved_pp;
    sync_tk_bypass_processor();
    apply_params();
    last_params = params;
    tuned_latch = true;
    tune_score_db = result->score_db;
    // Arm the catch windows on the freshly tuned values (issue 12); the
    // mask resets here after every completed tune.
    catch_lock_init(&tune_lock);
    CatchTuned tuned;
    tuned.reduction_db = result->params.reduction_db;
    tuned.smoothing_pct = result->params.smoothing_pct;
    tuned.whitening_pct = result->params.whitening_pct;
    tuned.noise_rescale = result->params.noise_rescale;
    catch_lock_set_tuned(&tune_lock, &tuned);
    Serial.println("tune: lock 0xF");
    Serial.print("tune: complete red=");
    Serial.print(params.reduction_amount, 1);
    Serial.print(" sm=");
    Serial.print(params.smoothing_factor, 0);
    Serial.print(" wh=");
    Serial.print(params.whitening_factor, 0);
    Serial.print(" ag=");
    Serial.print(params.noise_rescale, 2);
    Serial.print(" score=");
    Serial.println(tune_score_db, 2);
}

void start_tune() {
    // Capture led to a real search: the resumable tuner core steps one
    // 128-sample block per loop iteration (issue 9).
    if (!tune_ring || tune_active || !nr1 || !nr1_cached_profile) {
        return;
    }
    tune_candidate_loaded = false;
    if (!tuner_core_begin(&tune_core, tune_ring, TUNE_RING_CAPACITY,
                          tune_process_block, NULL, tune_progress_cb, NULL)) {
        Serial.println("tune: ERROR - search failed to start");
        return;
    }
    tune_active = true;
    tuned_latch = false;
    tune_last_tenth = 0;
    tune_progress_pct = 0;
    saved_tk = params.tone_kill_enabled;
    saved_pp = params.post_filter_enabled;
    params.tone_kill_enabled = false;   // forced off during the search
    params.post_filter_enabled = false;
    sync_tk_bypass_processor();
    Serial.println("tune: start");
}

void abort_tune() {
    if (!tune_active) return;
    tune_active = false;
    tuner_core_finish(&tune_core);      // discard the search state
    params.tone_kill_enabled = saved_tk;
    params.post_filter_enabled = saved_pp;
    sync_tk_bypass_processor();
    apply_params();                     // restore the live parameters
    last_params = params;
    Serial.println("tune: aborted");
}

// Advance the search by one 128-sample block and handle completion.
// Silent mode calls this from the loop driver (DSP speed); audition mode
// (issue 11) calls it once per audio block so the sweep is paced by the
// input clock.
void tune_search_advance() {
    if (!tune_active) return;
    if (tuner_core_step(&tune_core)) {
        const TunerResult result = tuner_core_finish(&tune_core);
        tune_active = false;
        if (result.ok) {
            apply_tune_result(&result);
        } else {
            // Search never produced a result: restore prior state.
            params.tone_kill_enabled = saved_tk;
            params.post_filter_enabled = saved_pp;
            sync_tk_bypass_processor();
            apply_params();
            last_params = params;
            Serial.println("tune: aborted");
        }
    }
}

void show_tune_notice(int mode) {
    // mode 1 = CLIP, 2 = QUIET. Full-screen notice for TUNE_NOTICE_MS; the
    // inverted status chip persists until the next capture/button/mode action.
    tune_notice = mode;
    tune_notice_until = millis() + TUNE_NOTICE_MS;
}

void show_tune_cancel_notice() {
    // Brief notice only — the persistent chip is untouched.
    tune_notice_until = millis() + TUNE_CANCEL_NOTICE_MS;
}

void clear_tune_notice() {
    tune_notice = 0;
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

    // Full-screen guard/cancel notice (3 s; 1.5 s for press-cancel). The
    // persistent chip below survives the overlay until the next action.
    if ((int32_t)(tune_notice_until - millis()) > 0) {
        const char *notice = tune_notice == 1 ? "CLIP"
                          : tune_notice == 2 ? "QUIET"
                          :                    "TUNE CANCELLED";
        display.fillRect(0, 0, 128, 64, SSD1306_WHITE);
        display.setTextColor(SSD1306_BLACK);
        display.setCursor((int16_t)((128 - (int16_t)strlen(notice) * 6) / 2), 28);
        display.print(notice);
        display.display();
        return;
    }

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

    // Line 2 — reduction bar (the bar stays normal; the value inverts
    // while the tune is latched)
    display.print("Red: ");
    int bar = (int)(params.reduction_amount / REDUCTION_MAX_DB * 80);
    for (int i = 0; i < bar / 8; i++) display.print("\xDB");  // full block
    display.print(" ");
    char red_buf[8];
    snprintf(red_buf, sizeof(red_buf), "%ddB", (int)roundf(params.reduction_amount));
    if (tuned_latch && current_mode != 0 &&
        catch_lock_is_locked(&tune_lock, CATCH_PARAM_REDUCTION)) {
        display.fillRect(display.getCursorX(), 16, (int16_t)(strlen(red_buf) * 6), 16, SSD1306_WHITE);
        display.setTextColor(SSD1306_BLACK);
    }
    display.print(red_buf);
    display.setTextColor(SSD1306_WHITE);
    display.println();

    // Line 3 — smoothing + whitening (values invert while latched)
    display.print("Sm:");
    char sm_buf[8];
    snprintf(sm_buf, sizeof(sm_buf), "%d%%", (int)params.smoothing_factor);
    if (tuned_latch && current_mode != 0 &&
        catch_lock_is_locked(&tune_lock, CATCH_PARAM_SMOOTHING)) {
        display.fillRect(display.getCursorX(), 32, (int16_t)(strlen(sm_buf) * 6), 16, SSD1306_WHITE);
        display.setTextColor(SSD1306_BLACK);
    }
    display.print(sm_buf);
    display.setTextColor(SSD1306_WHITE);
    display.print("  Wh:");
    char wh_buf[8];
    snprintf(wh_buf, sizeof(wh_buf), "%d%%", (int)params.whitening_factor);
    if (tuned_latch && current_mode != 0 &&
        catch_lock_is_locked(&tune_lock, CATCH_PARAM_WHITENING)) {
        display.fillRect(display.getCursorX(), 32, (int16_t)(strlen(wh_buf) * 6), 16, SSD1306_WHITE);
        display.setTextColor(SSD1306_BLACK);
    }
    display.print(wh_buf);
    display.setTextColor(SSD1306_WHITE);
    display.println();

    // Line 4 — aggression + status + feature indicators
    // Status (Learning../Tuning NN%/Tuned/Prof/None/Auto/Bypass) sits at a
    // fixed position after the value; indicators follow in TK, PP order.
    // Reversed video = feature active in this mode; normal-video PP in
    // bypass = armed but dormant (the post-filter has no effect there).
    display.setCursor(0, 48);
    display.print("Ag:");
    if (tuned_latch && current_mode != 0 &&
        catch_lock_is_locked(&tune_lock, CATCH_PARAM_AGGRESSION)) {
        display.fillRect(display.getCursorX(), 48, 24, 16, SSD1306_WHITE);  // "1.20" is always 4 chars
        display.setTextColor(SSD1306_BLACK);
    }
    display.print(params.noise_rescale, 2);
    display.setTextColor(SSD1306_WHITE);

    int16_t x = 48;  // "Ag:1.93" (42 px) + separator space
    const char *status = "Bypass";
    char tune_status_buf[16];
    bool chip = false;
    if (tune_notice != 0) {
        // Persistent inverted guard chip (CLIP/QUIET) until the next
        // capture/button/mode action.
        status = tune_notice == 1 ? "CLIP" : "QUIET";
        chip = true;
    } else if (nr1_noise_learning) {
        status = "Learning...";
    } else if (tune_active) {
        // Search progress: "Tuning NN%".
        snprintf(tune_status_buf, sizeof(tune_status_buf), "Tuning %u%%",
                 (unsigned)tune_progress_pct);
        status = tune_status_buf;
    } else if (tuned_latch && current_mode != 0) {
        // Completed tune: inverted "Tuned" chip; the locked values above
        // are inverted per-param. Dormant in Bypass (issue 13): the preset
        // survives the mode switch but shows normal video there.
        status = "Tuned";
        chip = true;
    } else if (current_mode == 1 && specbleach_noise_profile_available(nr1)) {
        status = "Prof";
    } else if (current_mode == 1) {
        status = "None";
    } else if (current_mode == 2) {
        status = "Auto";
    }
    if (chip) {
        display.fillRect(x, 48, (int16_t)(strlen(status) * 6), 16, SSD1306_WHITE);
        display.setCursor(x, 48);
        display.setTextColor(SSD1306_BLACK);
        display.print(status);
        display.setTextColor(SSD1306_WHITE);
    } else {
        display.setCursor(x, 48);
        display.print(status);
    }
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
