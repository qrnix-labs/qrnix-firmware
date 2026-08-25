// Auto-tune tuner core (issue 6) — see tuner_core.h.
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
//   1. coarse 2-D grid: reduction {0,5,..,30} x noise_rescale {0,0.25,..,2}
//   2. local refine: +-1 step around the best cell in both axes
//   3. 1-D polish: smoothing {0,10,..,100}, then whitening {0,10,..,100}
//      (whitening is unclamped at the low end: the search may drive it to 0)
//   4. budget-permitting second polish at 5% steps around the best
//      smoothing and whitening values
// Planned total = 63 + 4 + 11 + 11 + 2 + 2 = 93 candidates; the progress
// fraction is candidates_run / 93 capped at 1.0. The hard time budget is
// checked after every candidate; a full plan never starts when exhausted.
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

// One-shot score over a whole buffer (the test harness exercises this
// directly; run() uses the same recurrence incrementally). Fills `tones`
// with per-bin power in sum-of-squares units. Returns dB, clamped.
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
    return (uint32_t)((uint64_t)clock() * 1000u / CLOCKS_PER_SEC);
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

// ── Public API ────────────────────────────────────────────────────────────────

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

// Evaluate one candidate: stream the mix through the process callback in
// TUNER_BLOCK_SAMPLES blocks, discard the convergence window, score the
// eval window. Returns the clamped score.
static float evaluate_candidate(TunerCore *core, const TunerParams *params,
                                const int16_t *ring, uint32_t ring_n,
                                TunerProcessFn process, void *process_ud) {
    const uint32_t win = core->discard_samples + core->eval_samples;
    const uint32_t nblocks = (win + TUNER_BLOCK_SAMPLES - 1) / TUNER_BLOCK_SAMPLES;
    for (int k = 0; k < TUNER_TONE_COUNT; k++) {
        core->bin_prev[k] = 0.0f;
        core->bin_prev2[k] = 0.0f;
    }
    core->total_power = 0.0;
    core->abs_pos = 0;

    for (uint32_t b = 0; b < nblocks; b++) {
        for (uint32_t i = 0; i < TUNER_BLOCK_SAMPLES; i++) {
            const uint32_t n = b * TUNER_BLOCK_SAMPLES + i;
            if (n >= win) {
                core->input_buf[i] = 0.0f;
            } else {
                const float t = (float)n / (float)TUNER_SAMPLE_RATE;
                const float s = speech_sample(t);
                const float v = (float)ring[n % ring_n] / 32768.0f;
                core->input_buf[i] = (s + core->noise_gain * v) * core->mix_scale;
            }
        }
        if (!process(core->input_buf, TUNER_BLOCK_SAMPLES, params,
                     core->output_buf, process_ud)) {
            return TUNER_SCORE_MIN_DB;  // failed candidate
        }
        for (uint32_t i = 0; i < TUNER_BLOCK_SAMPLES; i++) {
            core->abs_pos++;
            if (core->abs_pos <= core->discard_samples) {
                continue;
            }
            if (core->abs_pos > core->discard_samples + core->eval_samples) {
                continue;
            }
            const float sample = core->output_buf[i];
            core->total_power += (double)sample * (double)sample;
            for (int k = 0; k < TUNER_TONE_COUNT; k++) {
                const float next = sample + core->tone_coef[k] * core->bin_prev[k] -
                                   core->bin_prev2[k];
                core->bin_prev2[k] = core->bin_prev[k];
                core->bin_prev[k] = next;
            }
        }
    }

    double tone = 0.0;
    for (int k = 0; k < TUNER_TONE_COUNT; k++) {
        const double power = ((double)core->bin_prev[k] * core->bin_prev[k] +
                              (double)core->bin_prev2[k] * core->bin_prev2[k] -
                              (double)core->tone_coef[k] * core->bin_prev[k] *
                                  core->bin_prev2[k]) *
                             2.0 / (double)core->eval_samples;
        tone += power;
    }
    if (tone <= 0.0) {
        return TUNER_SCORE_MIN_DB;
    }
    const double residual = fmax((double)TUNER_RESIDUAL_FLOOR,
                                 core->total_power - tone);
    float score = 10.0f * log10f((float)(tone / residual));
    if (score > TUNER_SCORE_MAX_DB) score = TUNER_SCORE_MAX_DB;
    if (score < TUNER_SCORE_MIN_DB) score = TUNER_SCORE_MIN_DB;
    return score;
}

// Evaluate a candidate, track the best, report progress, return true when
// the time budget is exhausted (search must stop).
static bool evaluate_and_track(TunerCore *core, const TunerParams *params,
                               const int16_t *ring, uint32_t ring_n,
                               TunerProcessFn process, void *process_ud,
                               TunerProgressFn progress, void *progress_ud) {
    const float score = evaluate_candidate(core, params, ring, ring_n,
                                           process, process_ud);
    core->candidates_run++;
    if (!core->have_best || score > core->best_score) {
        core->have_best = true;
        core->best = *params;
        core->best_score = score;
    }
    if (progress != NULL) {
        float fraction = (float)core->candidates_run / (float)core->planned_total;
        if (fraction > 1.0f) fraction = 1.0f;
        progress(fraction, &core->best, core->best_score, progress_ud);
    }
    const uint32_t elapsed = core->now_ms() - core->started_ms;
    return elapsed >= core->time_budget_ms;
}

// True when (red, res) is already in the scored list (coarse grid or a
// previously taken refine candidate). Exact equality is safe: all values
// come from the same grid arithmetic.
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

TunerResult tuner_core_run(TunerCore *core, const int16_t *noise_ring,
                           uint32_t noise_ring_samples, TunerProcessFn process,
                           void *process_ud, TunerProgressFn progress,
                           void *progress_ud) {
    TunerResult result = {{0}, 0.0f, 0, false, false};
    if (core == NULL || process == NULL || noise_ring == NULL ||
        noise_ring_samples == 0 || core->eval_samples == 0 ||
        core->discard_samples == 0 || core->time_budget_ms == 0) {
        return result;
    }
    core->started_ms = core->now_ms();
    core->candidates_run = 0;
    core->have_best = false;
    core->best_score = TUNER_SCORE_MIN_DB;
    core->planned_total = 63 + 4 + 11 + 11 + 2 + 2;  // see header comment

    compute_mix(core, noise_ring, noise_ring_samples);

    // 1. Coarse 2-D grid: reduction x noise_rescale.
    float coarse[63][2];
    uint32_t coarse_count = 0;
    bool stop = false;
    for (int ri = 0; ri <= 6 && !stop; ri++) {
        for (int ni = 0; ni <= 8 && !stop; ni++) {
            TunerParams p = core->start;
            p.reduction_db = (float)(ri * 5);
            p.noise_rescale = (float)ni * 0.25f;
            coarse[coarse_count][0] = p.reduction_db;
            coarse[coarse_count][1] = p.noise_rescale;
            coarse_count++;
            stop = evaluate_and_track(core, &p, noise_ring, noise_ring_samples,
                                      process, process_ud, progress, progress_ud);
        }
    }

    // 2. Local refine around the best coarse cell (+-1 step, clamped, dedup).
    if (!stop) {
        float refine[4][2];
        uint32_t refine_count = 0;
        const float red = core->best.reduction_db;
        const float res = core->best.noise_rescale;
        const float candidates[4][2] = {
            {red - 5.0f, res}, {red + 5.0f, res},
            {red, res - 0.25f}, {red, res + 0.25f},
        };
        for (int c = 0; c < 4; c++) {
            float cr = candidates[c][0];
            float cn = candidates[c][1];
            if (cr < 0.0f) cr = 0.0f;
            if (cr > 30.0f) cr = 30.0f;
            if (cn < 0.0f) cn = 0.0f;
            if (cn > 2.0f) cn = 2.0f;
            if (already_scored(coarse, coarse_count, cr, cn) ||
                already_scored(refine, refine_count, cr, cn)) {
                continue;
            }
            refine[refine_count][0] = cr;
            refine[refine_count][1] = cn;
            refine_count++;
            TunerParams p = core->best;
            p.reduction_db = cr;
            p.noise_rescale = cn;
            stop = evaluate_and_track(core, &p, noise_ring, noise_ring_samples,
                                      process, process_ud, progress, progress_ud);
            if (stop) break;
        }
    }

    // 3. 1-D polish: smoothing, then whitening (whitening may go to 0).
    if (!stop) {
        for (int si = 0; si <= 10 && !stop; si++) {
            TunerParams p = core->best;
            p.smoothing_pct = (float)(si * 10);
            stop = evaluate_and_track(core, &p, noise_ring, noise_ring_samples,
                                      process, process_ud, progress, progress_ud);
        }
    }
    if (!stop) {
        for (int wi = 0; wi <= 10 && !stop; wi++) {
            TunerParams p = core->best;
            p.whitening_pct = (float)(wi * 10);
            stop = evaluate_and_track(core, &p, noise_ring, noise_ring_samples,
                                      process, process_ud, progress, progress_ud);
        }
    }
    // 4. Budget-permitting second polish at 5% steps around the best.
    if (!stop) {
        for (int k = 0; k < 2 && !stop; k++) {
            TunerParams p = core->best;
            p.smoothing_pct += k == 0 ? -5.0f : 5.0f;
            p = clamp_to_bounds(p);
            stop = evaluate_and_track(core, &p, noise_ring, noise_ring_samples,
                                      process, process_ud, progress, progress_ud);
        }
    }
    if (!stop) {
        for (int k = 0; k < 2 && !stop; k++) {
            TunerParams p = core->best;
            p.whitening_pct += k == 0 ? -5.0f : 5.0f;
            p = clamp_to_bounds(p);
            stop = evaluate_and_track(core, &p, noise_ring, noise_ring_samples,
                                      process, process_ud, progress, progress_ud);
        }
    }

    result.ok = true;
    result.budget_exhausted = stop;
    result.candidates_run = core->candidates_run;
    result.params = core->best;
    result.score_db = core->best_score;
    return result;
}
