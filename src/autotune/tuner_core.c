// Auto-tune tuner core (issue 6 + 9) — see tuner_core.h.
//
// Signal: harmonic stack {400, 800, 1600, 2400} Hz with weights
// {0.5, 0.8, 1.0, 0.6} inside a syllabic envelope (4 + 7 Hz, depth 0.7),
// mixed with the caller's noise ring at 10 dB SNR and normalized to a
// 0.5 peak (~6 dBFS).
//
// Scoring: Goertzel power at the four tone bins vs broadband residual
// over the eval window: score = 10*log10(tone/residual), clamped to
// [-60, +60] dB.
//
// Search (deterministic; no RNG, closed-form synthesis, fixed order,
// first-best tie-break):
//   1. coarse 2-D grid: reduction {0,10,..,30} x noise_rescale {0,0.5,..,2}
//   2. local refine: one midpoint probe per axis around the best cell
//      (+-5 dB, +-0.25 res), recovering the pre-coarsening resolution
//   3. 1-D polish: smoothing {0,10,..,100}, then whitening {0,10,..,100}
//      (whitening is unclamped at the low end: the search may drive it to 0)
//   4. budget-permitting second polish at 5% steps around the best
//      smoothing and whitening values
// Planned total = 20 + 4 + 11 + 11 + 2 + 2 = 50 candidates (issue 27,
// TUNER_PLANNED_TOTAL in tuner_core.h); the progress fraction is
// candidates_run / 50 capped at 1.0. The hard time budget is checked
// after every candidate; a full plan never starts when exhausted.
//
// Execution is resumable (issue 9): begin()/step()/finish() drive the
// search one 128-sample block per step so the firmware loop keeps polling
// the button, rendering the display, and emitting serial. run() is the
// blocking convenience wrapper used by the host tests.
//
// Pure C (math.h/time.h only): no DSP, no Teensy, no malloc, no globals.

#include "tuner_core.h"

#include <math.h>
#include <time.h>

#define TUNER_SNR_DB          10.0f   // test-signal SNR
#define TUNER_PEAK_AMP        0.5f    // ~6 dBFS peak normalization
#define TUNER_SCORE_MAX_DB    60.0f
#define TUNER_SCORE_MIN_DB    -60.0f
#define TUNER_RESIDUAL_FLOOR  1e-9f
#define TUNER_ENV_RATE_A      4.0f    // syllabic envelope rates (Hz)
#define TUNER_ENV_RATE_B      7.0f
#define TUNER_ENV_DEPTH       0.7f

static const float TONE_FREQS[TUNER_TONE_COUNT] = {400.0f, 800.0f, 1600.0f, 2400.0f};
static const float TONE_WEIGHTS[TUNER_TONE_COUNT] = {0.5f, 0.8f, 1.0f, 0.6f};

// ── Search phases (stored in TunerCore.phase) ─────────────────────────────────

enum {
    SEARCH_COARSE = 0,
    SEARCH_REFINE,
    SEARCH_SMOOTH,
    SEARCH_WHITEN,
    SEARCH_SMOOTH2,
    SEARCH_WHITEN2,
    SEARCH_DONE,
};

// ── Synthesis ─────────────────────────────────────────────────────────────────

static float speech_sample(float t) {
    float e = 1.0f - TUNER_ENV_DEPTH *
        (0.5f * sinf(2.0f * (float)M_PI * TUNER_ENV_RATE_A * t) +
         0.5f * sinf(2.0f * (float)M_PI * TUNER_ENV_RATE_B * t));
    if (e < 0.0f) {
        e = 0.0f;
    }
    float s = 0.0f;
    for (int k = 0; k < TUNER_TONE_COUNT; k++) {
        s += TONE_WEIGHTS[k] * sinf(2.0f * (float)M_PI * TONE_FREQS[k] * t);
    }
    return e * s;
}

// ── Goertzel scoring ──────────────────────────────────────────────────────────

#if defined(__GNUC__)
__attribute__((unused))  // host-test seam; not referenced by the firmware
#endif
static float goertzel_score(const float *x, uint32_t n, float tones[TUNER_TONE_COUNT]) {
    float prev[TUNER_TONE_COUNT] = {0};
    float prev2[TUNER_TONE_COUNT] = {0};
    double total = 0.0;
    for (uint32_t i = 0; i < n; i++) {
        const float sample = x[i];
        total += (double)sample * (double)sample;
        for (int k = 0; k < TUNER_TONE_COUNT; k++) {
            const float coef = 2.0f * cosf(2.0f * (float)M_PI * TONE_FREQS[k] /
                                           (float)TUNER_SAMPLE_RATE);
            const float next = sample + coef * prev[k] - prev2[k];
            prev2[k] = prev[k];
            prev[k] = next;
        }
    }
    double tone = 0.0;
    for (int k = 0; k < TUNER_TONE_COUNT; k++) {
        // |X|^2 = prev^2 + prev2^2 - coef*prev*prev2; convert to sum-of-
        // squares units via |X|^2 * 2 / n.
        const float coef = 2.0f * cosf(2.0f * (float)M_PI * TONE_FREQS[k] /
                                       (float)TUNER_SAMPLE_RATE);
        const double power = ((double)prev[k] * prev[k] +
                              (double)prev2[k] * prev2[k] -
                              (double)coef * prev[k] * prev2[k]) * 2.0 / (double)n;
        tones[k] = (float)power;
        tone += power;
    }
    if (tone <= 0.0) {
        return TUNER_SCORE_MIN_DB;
    }
    const double residual = fmax((double)TUNER_RESIDUAL_FLOOR, total - tone);
    float score = 10.0f * log10f((float)(tone / residual));
    if (score > TUNER_SCORE_MAX_DB) score = TUNER_SCORE_MAX_DB;
    if (score < TUNER_SCORE_MIN_DB) score = TUNER_SCORE_MIN_DB;
    return score;
}

// ── Clock ─────────────────────────────────────────────────────────────────────

static uint32_t default_now_ms(void) {
#ifdef __arm__
    // The firmware always supplies a millis()-based clock; a missing clock
    // trips the budget immediately (safe fail). clock() would pull in the
    // newlib _times stub, which the Teensy toolchain does not provide.
    return 0;
#else
    return (uint32_t)((uint64_t)clock() * 1000u / CLOCKS_PER_SEC);
#endif
}

// ── Mix construction ──────────────────────────────────────────────────────────

// Compute the per-run noise gain and mix scale from the ring and the
// window length. gain = speech_rms / (noise_rms * sqrt(10)) so the mix has
// exactly 10 dB SNR; scale = 0.5 / peak normalizes to ~6 dBFS.
static void compute_mix(TunerCore *core, const int16_t *ring, uint32_t ring_n) {
    const uint32_t win = core->discard_samples + core->eval_samples;
    double noise_ss = 0.0;
    for (uint32_t i = 0; i < ring_n; i++) {
        const float v = (float)ring[i] / 32768.0f;
        noise_ss += (double)v * (double)v;
    }
    double speech_ss = 0.0;
    float peak = 0.0f;
    for (uint32_t i = 0; i < win; i++) {
        const float s = speech_sample((float)i / (float)TUNER_SAMPLE_RATE);
        speech_ss += (double)s * (double)s;
    }
    const float speech_rms = sqrtf((float)(speech_ss / (double)win));
    const float noise_rms = sqrtf((float)(noise_ss / (double)ring_n));
    float gain = 0.0f;
    if (speech_rms > 0.0f && noise_rms > 0.0f) {
        gain = speech_rms / (noise_rms * sqrtf(10.0f));
    }
    for (uint32_t i = 0; i < win; i++) {
        const float s = speech_sample((float)i / (float)TUNER_SAMPLE_RATE);
        const float v = (float)ring[i % ring_n] / 32768.0f;
        const float m = s + gain * v;
        const float a = fabsf(m);
        if (a > peak) peak = a;
    }
    core->noise_gain = gain;
    core->mix_scale = peak > 0.0f ? TUNER_PEAK_AMP / peak : 0.0f;
}

// ── Public API: init / synthesize ─────────────────────────────────────────────

void tuner_core_init(TunerCore *core, const TunerConfig *config) {
    core->discard_samples = config->discard_samples;
    core->eval_samples = config->eval_samples;
    core->time_budget_ms = config->time_budget_ms;
    core->start = config->start;
    core->now_ms = config->now_ms != NULL ? config->now_ms : default_now_ms;
    for (int k = 0; k < TUNER_TONE_COUNT; k++) {
        core->tone_coef[k] = 2.0f * cosf(2.0f * (float)M_PI * TONE_FREQS[k] /
                                         (float)TUNER_SAMPLE_RATE);
    }
    core->noise_gain = 0.0f;
    core->mix_scale = 0.0f;
    core->phase = SEARCH_DONE;
}

uint32_t tuner_core_synthesize(const TunerConfig *config,
                               const int16_t *noise_ring,
                               uint32_t noise_ring_samples,
                               float *out, uint32_t cap) {
    if (config == NULL || noise_ring == NULL || noise_ring_samples == 0 ||
        out == NULL || config->eval_samples == 0 || config->discard_samples == 0) {
        return 0;
    }
    const uint32_t win = config->discard_samples + config->eval_samples;
    if (cap == 0) {
        return 0;
    }
    const uint32_t count = cap < win ? cap : win;

    TunerCore tmp;
    tuner_core_init(&tmp, config);
    compute_mix(&tmp, noise_ring, noise_ring_samples);

    for (uint32_t i = 0; i < count; i++) {
        const float t = (float)i / (float)TUNER_SAMPLE_RATE;
        const float s = speech_sample(t);
        const float v = (float)noise_ring[i % noise_ring_samples] / 32768.0f;
        out[i] = (s + tmp.noise_gain * v) * tmp.mix_scale;
    }
    return count;
}

// ── Search state machine ──────────────────────────────────────────────────────

// True when (red, res) is already in a scored list. Exact equality is
// safe: all values come from the same grid arithmetic.
static bool already_scored(const float (*scored)[2], uint32_t count,
                           float red, float res) {
    for (uint32_t i = 0; i < count; i++) {
        if (scored[i][0] == red && scored[i][1] == res) {
            return true;
        }
    }
    return false;
}

static TunerParams clamp_to_bounds(TunerParams p) {
    if (p.reduction_db < 0.0f) p.reduction_db = 0.0f;
    if (p.reduction_db > 30.0f) p.reduction_db = 30.0f;
    if (p.smoothing_pct < 0.0f) p.smoothing_pct = 0.0f;
    if (p.smoothing_pct > 100.0f) p.smoothing_pct = 100.0f;
    if (p.whitening_pct < 0.0f) p.whitening_pct = 0.0f;
    if (p.whitening_pct > 100.0f) p.whitening_pct = 100.0f;
    if (p.noise_rescale < 0.0f) p.noise_rescale = 0.0f;
    if (p.noise_rescale > 2.0f) p.noise_rescale = 2.0f;
    return p;
}

// Reset the per-candidate scoring state (called before the first block).
static void reset_candidate_state(TunerCore *core) {
    for (int k = 0; k < TUNER_TONE_COUNT; k++) {
        core->bin_prev[k] = 0.0f;
        core->bin_prev2[k] = 0.0f;
    }
    core->total_power = 0.0;
    core->abs_pos = 0;
    core->candidate_failed = false;
}

// Fill input_buf with the normalized mix for block `b` of the window.
static void build_block(TunerCore *core, uint32_t b) {
    const uint32_t win = core->discard_samples + core->eval_samples;
    for (uint32_t i = 0; i < TUNER_BLOCK_SAMPLES; i++) {
        const uint32_t n = b * TUNER_BLOCK_SAMPLES + i;
        if (n >= win) {
            core->input_buf[i] = 0.0f;
        } else {
            const float t = (float)n / (float)TUNER_SAMPLE_RATE;
            const float s = speech_sample(t);
            const float v = (float)core->ring[n % core->ring_n] / 32768.0f;
            core->input_buf[i] = (s + core->noise_gain * v) * core->mix_scale;
        }
    }
}

// Accumulate one processed output sample into the Goertzel state when it
// falls inside the eval window.
static void accumulate_sample(TunerCore *core, float sample) {
    core->abs_pos++;
    if (core->abs_pos <= core->discard_samples) {
        return;
    }
    if (core->abs_pos > core->discard_samples + core->eval_samples) {
        return;
    }
    core->total_power += (double)sample * (double)sample;
    for (int k = 0; k < TUNER_TONE_COUNT; k++) {
        const float next = sample + core->tone_coef[k] * core->bin_prev[k] -
                           core->bin_prev2[k];
        core->bin_prev2[k] = core->bin_prev[k];
        core->bin_prev[k] = next;
    }
}

// Score the finished candidate, track the best, report progress, and
// check the hard time budget.
static void finish_candidate(TunerCore *core) {
    float score = TUNER_SCORE_MIN_DB;
    if (!core->candidate_failed) {
        double tone = 0.0;
        for (int k = 0; k < TUNER_TONE_COUNT; k++) {
            const double power = ((double)core->bin_prev[k] * core->bin_prev[k] +
                                  (double)core->bin_prev2[k] * core->bin_prev2[k] -
                                  (double)core->tone_coef[k] * core->bin_prev[k] *
                                      core->bin_prev2[k]) *
                                 2.0 / (double)core->eval_samples;
            tone += power;
        }
        if (tone > 0.0) {
            const double residual = fmax((double)TUNER_RESIDUAL_FLOOR,
                                         core->total_power - tone);
            score = 10.0f * log10f((float)(tone / residual));
            if (score > TUNER_SCORE_MAX_DB) score = TUNER_SCORE_MAX_DB;
            if (score < TUNER_SCORE_MIN_DB) score = TUNER_SCORE_MIN_DB;
        }
    }
    core->candidates_run++;
    if (!core->have_best || score > core->best_score) {
        core->have_best = true;
        core->best = core->current;
        core->best_score = score;
    }
    if (core->progress != NULL) {
        float fraction = (float)core->candidates_run / (float)core->planned_total;
        if (fraction > 1.0f) fraction = 1.0f;
        core->progress(fraction, &core->best, core->best_score, core->progress_ud);
    }
    const uint32_t elapsed = core->now_ms() - core->started_ms;
    if (elapsed >= core->time_budget_ms) {
        core->budget_exhausted = true;
    }
}

// Advance to the next candidate of the fixed plan. Returns false when the
// plan is exhausted (phase becomes SEARCH_DONE); the caller then reports
// completion. On success sets core->current and resets candidate state.
static bool next_candidate(TunerCore *core) {
    for (;;) {
        switch (core->phase) {
        case SEARCH_COARSE: {
            if (core->grid_ni > TUNER_COARSE_RES_STEPS - 1) {
                core->grid_ni = 0;
                core->grid_ri++;
            }
            if (core->grid_ri > TUNER_COARSE_RED_STEPS - 1) {
                core->phase = SEARCH_REFINE;
                core->refine_idx = 0;
                continue;
            }
            TunerParams p = core->start;
            p.reduction_db = (float)core->grid_ri * TUNER_COARSE_RED_STEP;
            p.noise_rescale = (float)core->grid_ni * TUNER_COARSE_RES_STEP;
            core->coarse_grid[core->coarse_count][0] = p.reduction_db;
            core->coarse_grid[core->coarse_count][1] = p.noise_rescale;
            core->coarse_count++;
            core->grid_ni++;
            core->current = p;
            core->block_pos = 0;
            reset_candidate_state(core);
            return true;
        }
        case SEARCH_REFINE: {
            if (core->refine_idx >= TUNER_REFINE_STEPS) {
                core->phase = SEARCH_SMOOTH;
                core->polish_idx = 0;
                continue;
            }
            const float red = core->best.reduction_db;
            const float res = core->best.noise_rescale;
            float cr = red;
            float cn = res;
            switch (core->refine_idx) {
            case 0: cr = red - TUNER_REFINE_RED_STEP; break;
            case 1: cr = red + TUNER_REFINE_RED_STEP; break;
            case 2: cn = res - TUNER_REFINE_RES_STEP; break;
            default: cn = res + TUNER_REFINE_RES_STEP; break;
            }
            core->refine_idx++;
            if (cr < 0.0f) cr = 0.0f;
            if (cr > 30.0f) cr = 30.0f;
            if (cn < 0.0f) cn = 0.0f;
            if (cn > 2.0f) cn = 2.0f;
            if (already_scored(core->coarse_grid, core->coarse_count, cr, cn) ||
                already_scored(core->refine_grid, core->refine_count, cr, cn)) {
                continue;
            }
            core->refine_grid[core->refine_count][0] = cr;
            core->refine_grid[core->refine_count][1] = cn;
            core->refine_count++;
            TunerParams p = core->best;
            p.reduction_db = cr;
            p.noise_rescale = cn;
            core->current = p;
            core->block_pos = 0;
            reset_candidate_state(core);
            return true;
        }
        case SEARCH_SMOOTH: {
            if (core->polish_idx > TUNER_POLISH_STEPS - 1) {
                core->phase = SEARCH_WHITEN;
                core->polish_idx = 0;
                continue;
            }
            TunerParams p = core->best;
            p.smoothing_pct = (float)(core->polish_idx * TUNER_POLISH_STEP);
            core->polish_idx++;
            core->current = p;
            core->block_pos = 0;
            reset_candidate_state(core);
            return true;
        }
        case SEARCH_WHITEN: {
            if (core->polish_idx > TUNER_POLISH_STEPS - 1) {
                core->phase = SEARCH_SMOOTH2;
                core->polish_idx = 0;
                continue;
            }
            TunerParams p = core->best;
            p.whitening_pct = (float)(core->polish_idx * TUNER_POLISH_STEP);
            core->polish_idx++;
            core->current = p;
            core->block_pos = 0;
            reset_candidate_state(core);
            return true;
        }
        case SEARCH_SMOOTH2: {
            if (core->polish_idx >= TUNER_POLISH2_STEPS) {
                core->phase = SEARCH_WHITEN2;
                core->polish_idx = 0;
                continue;
            }
            TunerParams p = core->best;
            p.smoothing_pct += core->polish_idx == 0 ? -TUNER_POLISH2_STEP
                                                     : TUNER_POLISH2_STEP;
            core->polish_idx++;
            core->current = clamp_to_bounds(p);
            core->block_pos = 0;
            reset_candidate_state(core);
            return true;
        }
        case SEARCH_WHITEN2: {
            if (core->polish_idx >= TUNER_POLISH2_STEPS) {
                core->phase = SEARCH_DONE;
                return false;
            }
            TunerParams p = core->best;
            p.whitening_pct += core->polish_idx == 0 ? -TUNER_POLISH2_STEP
                                                     : TUNER_POLISH2_STEP;
            core->polish_idx++;
            core->current = clamp_to_bounds(p);
            core->block_pos = 0;
            reset_candidate_state(core);
            return true;
        }
        default:
            core->phase = SEARCH_DONE;
            return false;
        }
    }
}

// ── Public API: begin / step / finish / run ───────────────────────────────────

bool tuner_core_begin(TunerCore *core, const int16_t *noise_ring,
                      uint32_t noise_ring_samples, TunerProcessFn process,
                      void *process_ud, TunerProgressFn progress,
                      void *progress_ud) {
    if (core == NULL || process == NULL || noise_ring == NULL ||
        noise_ring_samples == 0 || core->eval_samples == 0 ||
        core->discard_samples == 0 || core->time_budget_ms == 0) {
        return false;
    }
    core->ring = noise_ring;
    core->ring_n = noise_ring_samples;
    core->process = process;
    core->process_ud = process_ud;
    core->progress = progress;
    core->progress_ud = progress_ud;
    core->started_ms = core->now_ms();
    core->candidates_run = 0;
    core->have_best = false;
    core->best_score = TUNER_SCORE_MIN_DB;
    core->budget_exhausted = false;
    core->planned_total = TUNER_PLANNED_TOTAL;  // see tuner_core.h
    core->phase = SEARCH_COARSE;
    core->grid_ri = 0;
    core->grid_ni = 0;
    core->refine_idx = 0;
    core->polish_idx = 0;
    core->coarse_count = 0;
    core->refine_count = 0;
    compute_mix(core, noise_ring, noise_ring_samples);
    return next_candidate(core);
}

// Process one 128-sample block of the current candidate; returns true when
// the whole search is finished (success, abort via finish(), or budget).
bool tuner_core_step(TunerCore *core) {
    if (core->phase == SEARCH_DONE) {
        return true;
    }
    const uint32_t win = core->discard_samples + core->eval_samples;
    const uint32_t nblocks = (win + TUNER_BLOCK_SAMPLES - 1) / TUNER_BLOCK_SAMPLES;
    if (core->block_pos < nblocks) {
        build_block(core, core->block_pos);
        if (!core->process(core->input_buf, TUNER_BLOCK_SAMPLES, &core->current,
                           core->output_buf, core->process_ud)) {
            core->candidate_failed = true;  // failed candidate: score -60 dB
        } else {
            for (uint32_t i = 0; i < TUNER_BLOCK_SAMPLES; i++) {
                accumulate_sample(core, core->output_buf[i]);
            }
        }
        core->block_pos++;
        if (core->block_pos >= nblocks) {
            finish_candidate(core);
            if (core->budget_exhausted) {
                core->phase = SEARCH_DONE;
                return true;
            }
            return !next_candidate(core);  // true when the plan is done
        }
        return false;
    }
    core->phase = SEARCH_DONE;
    return true;
}

TunerResult tuner_core_finish(TunerCore *core) {
    TunerResult result = {{0}, 0.0f, 0, false, false};
    if (core == NULL) {
        return result;
    }
    result.ok = core->have_best;
    result.budget_exhausted = core->budget_exhausted;
    result.candidates_run = core->candidates_run;
    result.params = core->best;
    result.score_db = core->best_score;
    return result;
}

TunerResult tuner_core_run(TunerCore *core, const int16_t *noise_ring,
                           uint32_t noise_ring_samples, TunerProcessFn process,
                           void *process_ud, TunerProgressFn progress,
                           void *progress_ud) {
    TunerResult result = {{0}, 0.0f, 0, false, false};
    if (!tuner_core_begin(core, noise_ring, noise_ring_samples, process,
                          process_ud, progress, progress_ud)) {
        return result;
    }
    while (!tuner_core_step(core)) {
        // block-by-block; the loop body does nothing else
    }
    return tuner_core_finish(core);
}
