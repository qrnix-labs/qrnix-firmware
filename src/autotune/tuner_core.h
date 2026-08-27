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
#define AUTOTUNE_TUNER_CORE_H

#include <stdbool.h>
#include <stdint.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define TUNER_TONE_COUNT     4
#define TUNER_SAMPLE_RATE    44100u
#define TUNER_BLOCK_SAMPLES  128u

// Search plan (single source of truth): the coarse 2-D grid covers
// reduction x noise_rescale; refine probes one midpoint per axis around
// the best coarse cell; polish sweeps smoothing/whitening in 10s with a
// +-5 second pass. TUNER_PLANNED_TOTAL drives the progress fraction and
// TUNER_COARSE_COUNT sizes the dedup grid.
#define TUNER_COARSE_RED_STEPS   4u    // reduction {0,10,20,30} dB
#define TUNER_COARSE_RED_STEP    10.0f
#define TUNER_COARSE_RES_STEPS   5u    // noise_rescale {0,0.5,1,1.5,2}
#define TUNER_COARSE_RES_STEP    0.5f
#define TUNER_REFINE_STEPS       4u    // +- half a coarse step per axis
#define TUNER_REFINE_RED_STEP    (TUNER_COARSE_RED_STEP * 0.5f)  // 5 dB
#define TUNER_REFINE_RES_STEP    (TUNER_COARSE_RES_STEP * 0.5f)  // 0.25
#define TUNER_POLISH_STEPS       11u   // smoothing/whitening 0..100 in 10s
#define TUNER_POLISH_STEP        10.0f
#define TUNER_POLISH2_STEPS      2u    // +-5 around the best polish value
#define TUNER_POLISH2_STEP       5.0f
#define TUNER_COARSE_COUNT       (TUNER_COARSE_RED_STEPS * TUNER_COARSE_RES_STEPS)
#define TUNER_PLANNED_TOTAL      (TUNER_COARSE_COUNT + TUNER_REFINE_STEPS + \
                                  2 * TUNER_POLISH_STEPS + 2 * TUNER_POLISH2_STEPS)

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
    /* Resumable search state (issue 9): one 128-sample block per step. */
    const int16_t *ring;                // current run's noise ring
    uint32_t ring_n;
    TunerProcessFn process;
    void *process_ud;
    TunerProgressFn progress;
    void *progress_ud;
    int phase;                          // internal search phase
    int grid_ri, grid_ni;               // coarse-grid indices
    int refine_idx, polish_idx;         // refine/polish indices
    int coarse_count, refine_count;     // scored-grid dedup lists
    float coarse_grid[TUNER_COARSE_COUNT][2];
    float refine_grid[TUNER_REFINE_STEPS][2];
    uint32_t block_pos;                 // block index within current candidate
    bool candidate_failed;              // current candidate process failed
    bool budget_exhausted;              // search stopped early by the budget
    TunerParams current;                // candidate being evaluated
} TunerCore;

void tuner_core_init(TunerCore *core, const TunerConfig *config);

// Resumable search (issue 9): the firmware steps one 128-sample block per
// loop iteration so the display, button, and serial stay live. begin()
// validates, computes the mix, and arms the first candidate; step()
// processes one block and returns true when the search is finished (plan
// complete, budget exhausted, or aborted via finish()); finish() returns
// the result. run() is the blocking wrapper (begin + steps + finish) used
// by the host tests.
bool tuner_core_begin(TunerCore *core, const int16_t *noise_ring,
                      uint32_t noise_ring_samples, TunerProcessFn process,
                      void *process_ud, TunerProgressFn progress,
                      void *progress_ud);
bool tuner_core_step(TunerCore *core);
TunerResult tuner_core_finish(TunerCore *core);

TunerResult tuner_core_run(TunerCore *core, const int16_t *noise_ring,
                           uint32_t noise_ring_samples, TunerProcessFn process,
                           void *process_ud, TunerProgressFn progress,
                           void *progress_ud);

// Synthesize the mixed + normalized test signal used by run() into `out`
// (public so tests can verify spec conformance). Returns the number of
// samples written (min(cap, discard+eval)); 0 on invalid input.
#ifdef __cplusplus
}
#endif

#endif
