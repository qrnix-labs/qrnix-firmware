// Auto-tune tuner core (issue 6) — see tuner_core.c.
//
// Pure-C, DSP-agnostic search core: synthesizes a voice-like test signal
// (400/800/1600/2400 Hz harmonic stack with syllabic envelope), mixes it
// with a caller-provided noise ring at 10 dB SNR, normalizes to ~6 dBFS
// peak, runs each candidate parameter set through a caller-supplied
// process callback (the DSP adapter in the firmware), scores the output
// with Goertzel tone power vs broadband residual, and searches the
// parameter space deterministically under a hard time budget.
//
// No DSP or Teensy dependencies; no malloc; no global state.
#ifndef AUTOTUNE_TUNER_CORE_H
#include <stdbool.h>
#include <stdint.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define TUNER_TONE_COUNT     4
#define TUNER_SAMPLE_RATE    44100u
#define TUNER_BLOCK_SAMPLES  128u

// Candidate parameter set (mirrors the denoiser's tunable knobs).
typedef struct {
    float reduction_db;   // 0..30
    float smoothing_pct;  // 0..100
    float whitening_pct;  // 0..100
    float noise_rescale;  // 0..2 (aggression)
} TunerParams;

typedef struct {
    TunerParams params;        // best params found
    float score_db;            // 10*log10(tone/residual) of the best candidate
    uint32_t candidates_run;   // candidates evaluated
    bool budget_exhausted;     // search stopped early by the time budget
    bool ok;                   // false when config/ring/process invalid
} TunerResult;

typedef struct {
    uint32_t discard_samples;  // convergence discard window (~100 ms = 4410)
    uint32_t eval_samples;     // scoring window (~0.3 s = 13230)
    uint32_t time_budget_ms;   // hard search budget (45000)
    TunerParams start;         // search start point (10 dB, 10%, 10%, 0.20)
    // Monotonic clock for the budget; NULL selects the built-in host clock
    // (clock()). The firmware adapter passes millis()-based time.
    uint32_t (*now_ms)(void);
} TunerConfig;

// Process one block of the normalized test-signal mix through the DSP
// under `params`. Return false to mark the candidate failed (score -60 dB).
typedef bool (*TunerProcessFn)(const float *input, uint32_t samples,
                               const TunerParams *params, float *output,
                               void *userdata);

// Called after each evaluated candidate. `fraction` is monotone
// nondecreasing, capped at 1.0; `current` is the best-so-far params.
typedef void (*TunerProgressFn)(float fraction, const TunerParams *current,
                                float score_db, void *userdata);

// All state is caller-owned (the firmware declares this statically).
// Fields are implementation details; treat them as private.
typedef struct TunerCore {
    uint32_t discard_samples;
    uint32_t eval_samples;
    uint32_t time_budget_ms;
    TunerParams start;
    uint32_t (*now_ms)(void);
    float tone_coef[TUNER_TONE_COUNT];  // Goertzel coefficients
    float noise_gain;                   // per-run mix gain (10 dB SNR)
    float mix_scale;                    // per-run normalization (~6 dBFS peak)
    float input_buf[TUNER_BLOCK_SAMPLES];
    float output_buf[TUNER_BLOCK_SAMPLES];
    float bin_prev[TUNER_TONE_COUNT];   // Goertzel recurrence state
    float bin_prev2[TUNER_TONE_COUNT];
    double total_power;                 // eval-window sum of squares
    uint32_t abs_pos;                   // absolute sample position in window
    uint32_t candidates_run;
    uint32_t planned_total;             // fixed candidate plan (progress basis)
    uint32_t started_ms;
    float best_score;
    TunerParams best;
    bool have_best;
} TunerCore;

void tuner_core_init(TunerCore *core, const TunerConfig *config);

TunerResult tuner_core_run(TunerCore *core, const int16_t *noise_ring,
                           uint32_t noise_ring_samples, TunerProcessFn process,
                           void *process_ud, TunerProgressFn progress,
                           void *progress_ud);

// Synthesize the mixed + normalized test signal used by run() into `out`
// (public so tests can verify spec conformance). Returns the number of
// samples written (min(cap, discard+eval)); 0 on invalid input.
uint32_t tuner_core_synthesize(const TunerConfig *config,
                               const int16_t *noise_ring,
                               uint32_t noise_ring_samples,
                               float *out, uint32_t cap);

#endif
