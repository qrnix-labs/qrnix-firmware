// Host-side unit tests for the auto-tune tuner core (issue 6).
// Built against the pure-C module only; no DSP or Teensy includes.
// Prior art: test/test_contract.c (Unity, module compiled into the test).

#include <math.h>
#include <stdint.h>
#include <string.h>
#include <unity.h>

// The pure-C module is compiled into the test directly (native env).
#include "autotune/tuner_core.c"

// ── Helpers ───────────────────────────────────────────────────────────────────

static const uint32_t TEST_RING_LEN = 44100;

// Deterministic pseudo-noise ring (LCG), roughly uniform +/-0.5 full scale.
static void make_noise_ring(int16_t *ring, uint32_t n, uint32_t seed) {
    uint32_t s = seed;
    for (uint32_t i = 0; i < n; i++) {
        s = s * 1664525u + 1013904223u;
        int32_t v = (int32_t)(s >> 8) % 65536 - 32768;
        ring[i] = (int16_t)v;
    }
}

static TunerConfig default_config(void) {
    TunerConfig cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.discard_samples = 4410;   // ~100 ms convergence
    cfg.eval_samples = 13230;     // ~0.3 s scoring window
    cfg.time_budget_ms = 45000;   // hard budget
    cfg.start.reduction_db = 10.0f;
    cfg.start.smoothing_pct = 10.0f;
    cfg.start.whitening_pct = 10.0f;
    cfg.start.noise_rescale = 0.20f;
    cfg.now_ms = NULL;            // built-in host clock
    return cfg;
}

// Identity processor: candidate params do not affect the output.
static bool identity_process(const float *input, uint32_t samples,
                             const TunerParams *params, float *output,
                             void *userdata) {
    (void)params;
    (void)userdata;
    memcpy(output, input, samples * sizeof(float));
    return true;
}

// Scripted clock for the budget test: 0 ms for the first 5 calls (run
// start + 4 candidate checks), then 1001 ms so the budget trips after
// exactly 5 candidates. Module clock callbacks take no userdata, so the
// test uses a plain static counter (single-threaded).
static uint32_t script_calls = 0;

static uint32_t script_now_ms(void) {
    return script_calls++ < 5 ? 0 : 1001;
}

// ── Goertzel scoring on known signals (white-box: scoring is static) ─────────

void test_goertzel_pure_tone_scores_at_cap(void) {
    const uint32_t n = 13230;  // 400 Hz = 120 integer cycles
    float x[13230];
    for (uint32_t i = 0; i < n; i++) {
        x[i] = 0.5f * sinf(2.0f * (float)M_PI * 400.0f * (float)i / 44100.0f);
    }
    float tones[TUNER_TONE_COUNT];
    float score = goertzel_score(x, n, tones);
    // Pure tone: residual is below the floor, score clamps to +60.
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 60.0f, score);
    // Bin power in sum-of-squares units == A^2*N/2.
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 1653.75f, tones[0]);
    // Other bins see no power.
    TEST_ASSERT_TRUE(tones[1] < 1.0f);
    TEST_ASSERT_TRUE(tones[2] < 1.0f);
    TEST_ASSERT_TRUE(tones[3] < 1.0f);
}

void test_goertzel_white_noise_scores_negative(void) {
    const uint32_t n = 13230;
    float x[13230];
    for (uint32_t i = 0; i < n; i++) {
        // Uniform in [-0.5, 0.5]: power ~0.0833. Four bins of a white
        // spectrum hold ~4*2*sigma^2 of 13230*sigma^2 -> ~ -32 dB.
        x[i] = (float)((int32_t)(i * 2654435761u % 65536) - 32768) / 65536.0f;
    }
    float tones[TUNER_TONE_COUNT];
    float score = goertzel_score(x, n, tones);
    TEST_ASSERT_TRUE(score <= -30.0f);
    TEST_ASSERT_TRUE(score >= -60.0f);
}

void test_goertzel_tone_plus_noise_10db(void) {
    const uint32_t n = 13230;
    const float tone_power = 0.25f * (float)n / 2.0f;   // A^2*N/2, A = 0.5
    const float noise_power = tone_power / 10.0f;       // 10 dB SNR
    const float sigma = sqrtf(noise_power / (float)n);  // per-sample variance
    float x[13230];
    for (uint32_t i = 0; i < n; i++) {
        float tone = 0.5f * sinf(2.0f * (float)M_PI * 400.0f * (float)i / 44100.0f);
        float noise = (float)((int32_t)(i * 2654435761u % 65536) - 32768) / 65536.0f;
        x[i] = tone + noise * (sigma / 0.288675f);  // uniform sigma = a/sqrt(3)
    }
    float tones[TUNER_TONE_COUNT];
    float score = goertzel_score(x, n, tones);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 10.0f, score);
}

// ── Synthesis conformance: independent reference implementation ──────────────

static float ref_speech(float t) {
    static const float freqs[TUNER_TONE_COUNT] = {400, 800, 1600, 2400};
    static const float weights[TUNER_TONE_COUNT] = {0.5f, 0.8f, 1.0f, 0.6f};
    float e = 1.0f - 0.7f * (0.5f * sinf(2.0f * (float)M_PI * 4.0f * t) +
                             0.5f * sinf(2.0f * (float)M_PI * 7.0f * t));
    if (e < 0.0f) e = 0.0f;
    float s = 0.0f;
    for (int k = 0; k < TUNER_TONE_COUNT; k++) {
        s += weights[k] * sinf(2.0f * (float)M_PI * freqs[k] * t);
    }
    return e * s;
}

void test_synthesis_matches_reference_peak_and_snr(void) {
    TunerConfig cfg = default_config();
    const uint32_t win = cfg.discard_samples + cfg.eval_samples;
    int16_t ring[TEST_RING_LEN];
    make_noise_ring(ring, TEST_RING_LEN, 12345u);

    float out[17640];
    uint32_t written = tuner_core_synthesize(&cfg, ring, TEST_RING_LEN, out, win);
    TEST_ASSERT_EQUAL_UINT32(win, written);

    // Independent reconstruction per the issue-6 spec.
    float speech_ss = 0.0f;
    float noise_ss = 0.0f;
    for (uint32_t i = 0; i < TEST_RING_LEN; i++) {
        float v = (float)ring[i] / 32768.0f;
        noise_ss += v * v;
    }
    for (uint32_t i = 0; i < win; i++) {
        float s = ref_speech((float)i / 44100.0f);
        speech_ss += s * s;
    }
    float speech_rms = sqrtf(speech_ss / (float)win);
    float noise_rms = sqrtf(noise_ss / (float)TEST_RING_LEN);
    float gain = speech_rms / (noise_rms * sqrtf(10.0f));
    float peak = 0.0f;
    for (uint32_t i = 0; i < win; i++) {
        float s = ref_speech((float)i / 44100.0f);
        float v = (float)ring[i % TEST_RING_LEN] / 32768.0f;
        float m = s + gain * v;
        float a = fabsf(m);
        if (a > peak) peak = a;
    }
    float scale = 0.5f / peak;

    float ref[17640];
    float ref_speech_power = 0.0f;
    float ref_noise_power = 0.0f;
    for (uint32_t i = 0; i < win; i++) {
        float s = ref_speech((float)i / 44100.0f);
        float v = (float)ring[i % TEST_RING_LEN] / 32768.0f;
        ref[i] = (s + gain * v) * scale;
        ref_speech_power += s * s;
        ref_noise_power += (gain * v) * (gain * v);
    }

    // Peak normalized to about 6 dBFS (0.5).
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.5f, peak * scale);
    // 10 dB SNR by construction.
    float snr_db = 10.0f * log10f(ref_speech_power / ref_noise_power);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 10.0f, snr_db);
    // Sample-for-sample equivalence with the reference.
    for (uint32_t i = 0; i < win; i++) {
        TEST_ASSERT_FLOAT_WITHIN(1e-4f, ref[i], out[i]);
    }
}

// ── run(): determinism, budget, bounds, whitening, progress, guards ──────────

void test_step_api_matches_blocking_run(void) {
    // The resumable path (issue 9: one 128-sample block per step) must
    // produce exactly the same result as the blocking run() wrapper.
    TunerConfig cfg = default_config();
    int16_t ring[TEST_RING_LEN];
    make_noise_ring(ring, TEST_RING_LEN, 2024u);

    TunerCore blocking, stepped;
    tuner_core_init(&blocking, &cfg);
    tuner_core_init(&stepped, &cfg);
    TunerResult rb = tuner_core_run(&blocking, ring, TEST_RING_LEN,
                                    identity_process, NULL, NULL, NULL);
    TEST_ASSERT_TRUE(tuner_core_begin(&stepped, ring, TEST_RING_LEN,
                                      identity_process, NULL, NULL, NULL));
    uint32_t steps = 0;
    while (!tuner_core_step(&stepped)) {
        steps++;
        TEST_ASSERT_TRUE(steps < 20000);  // never hangs
    }
    TunerResult rs = tuner_core_finish(&stepped);
    TEST_ASSERT_TRUE(rs.ok);
    TEST_ASSERT_TRUE(rb.ok);
    TEST_ASSERT_EQUAL_UINT32(rb.candidates_run, rs.candidates_run);
    TEST_ASSERT_TRUE(rb.params.reduction_db == rs.params.reduction_db);
    TEST_ASSERT_TRUE(rb.params.smoothing_pct == rs.params.smoothing_pct);
    TEST_ASSERT_TRUE(rb.params.whitening_pct == rs.params.whitening_pct);
    TEST_ASSERT_TRUE(rb.params.noise_rescale == rs.params.noise_rescale);
    TEST_ASSERT_TRUE(rb.score_db == rs.score_db);
}

void test_run_is_deterministic(void) {
    TunerConfig cfg = default_config();
    int16_t ring[TEST_RING_LEN];
    make_noise_ring(ring, TEST_RING_LEN, 777u);

    TunerCore core_a, core_b;
    tuner_core_init(&core_a, &cfg);
    tuner_core_init(&core_b, &cfg);
    TunerResult ra = tuner_core_run(&core_a, ring, TEST_RING_LEN,
                                    identity_process, NULL, NULL, NULL);
    TunerResult rb = tuner_core_run(&core_b, ring, TEST_RING_LEN,
                                    identity_process, NULL, NULL, NULL);
    TEST_ASSERT_TRUE(ra.ok);
    TEST_ASSERT_TRUE(rb.ok);
    TEST_ASSERT_EQUAL_UINT32(ra.candidates_run, rb.candidates_run);
    TEST_ASSERT_EQUAL_INT(ra.budget_exhausted, rb.budget_exhausted);
    TEST_ASSERT_TRUE(ra.params.reduction_db == rb.params.reduction_db);
    TEST_ASSERT_TRUE(ra.params.smoothing_pct == rb.params.smoothing_pct);
    TEST_ASSERT_TRUE(ra.params.whitening_pct == rb.params.whitening_pct);
    TEST_ASSERT_TRUE(ra.params.noise_rescale == rb.params.noise_rescale);
    TEST_ASSERT_TRUE(ra.score_db == rb.score_db);
}

void test_run_respects_hard_budget(void) {
    TunerConfig cfg = default_config();
    cfg.time_budget_ms = 1000;
    int16_t ring[TEST_RING_LEN];
    make_noise_ring(ring, TEST_RING_LEN, 42u);

    script_calls = 0;  // budget expires after 5 candidates
    cfg.now_ms = script_now_ms;
    TunerCore core;
    tuner_core_init(&core, &cfg);
    TunerResult r = tuner_core_run(&core, ring, TEST_RING_LEN,
                                   identity_process, NULL, NULL, NULL);
    TEST_ASSERT_TRUE(r.ok);
    TEST_ASSERT_TRUE(r.budget_exhausted);
    TEST_ASSERT_EQUAL_UINT32(5, r.candidates_run);
}

void test_run_respects_param_bounds(void) {
    TunerConfig cfg = default_config();
    int16_t ring[TEST_RING_LEN];
    make_noise_ring(ring, TEST_RING_LEN, 99u);

    TunerCore core;
    tuner_core_init(&core, &cfg);
    TunerResult r = tuner_core_run(&core, ring, TEST_RING_LEN,
                                   identity_process, NULL, NULL, NULL);
    TEST_ASSERT_TRUE(r.ok);
    TEST_ASSERT_FALSE(r.budget_exhausted);
    TEST_ASSERT_TRUE(r.params.reduction_db >= 0.0f && r.params.reduction_db <= 30.0f);
    TEST_ASSERT_TRUE(r.params.smoothing_pct >= 0.0f && r.params.smoothing_pct <= 100.0f);
    TEST_ASSERT_TRUE(r.params.whitening_pct >= 0.0f && r.params.whitening_pct <= 100.0f);
    TEST_ASSERT_TRUE(r.params.noise_rescale >= 0.0f && r.params.noise_rescale <= 2.0f);
    TEST_ASSERT_TRUE(r.score_db >= -60.0f && r.score_db <= 60.0f);
}

// Fake denoiser whose output degrades monotonically as whitening increases:
// out = (1-a)*in + a*ring, a = whitening/100. The tone stack then vanishes
// with whitening, so the search must drive whitening to its unclamped 0.
typedef struct {
    const int16_t *ring;
    uint32_t ring_len;
} FakeUd;

static bool whitening_fake_process(const float *input, uint32_t samples,
                                   const TunerParams *params, float *output,
                                   void *userdata) {
    FakeUd *ud = (FakeUd *)userdata;
    float alpha = params->whitening_pct / 100.0f;
    for (uint32_t i = 0; i < samples; i++) {
        float ring_v = (float)ud->ring[i % ud->ring_len] / 32768.0f;
        output[i] = input[i] * (1.0f - alpha) + ring_v * alpha;
    }
    return true;
}

void test_whitening_drives_to_zero_under_fake_process(void) {
    TunerConfig cfg = default_config();
    int16_t ring[TEST_RING_LEN];
    make_noise_ring(ring, TEST_RING_LEN, 555u);
    FakeUd ud = {ring, TEST_RING_LEN};

    TunerCore core;
    tuner_core_init(&core, &cfg);
    TunerResult r = tuner_core_run(&core, ring, TEST_RING_LEN,
                                   whitening_fake_process, &ud, NULL, NULL);
    TEST_ASSERT_TRUE(r.ok);
    TEST_ASSERT_FALSE(r.budget_exhausted);
    TEST_ASSERT_TRUE(r.params.whitening_pct == 0.0f);
    TEST_ASSERT_TRUE(r.params.smoothing_pct == 10.0f);  // fake ignores it: ties keep the start value
}

// ── Progress callback ─────────────────────────────────────────────────────────

typedef struct {
    float fractions[128];
    uint32_t count;
} ProgressLog;

static void progress_cb(float fraction, const TunerParams *current,
                        float score_db, void *userdata) {
    ProgressLog *log = (ProgressLog *)userdata;
    (void)current;
    (void)score_db;
    if (log->count < 128) {
        log->fractions[log->count++] = fraction;
    }
}

void test_progress_fractions_monotone_and_bounded(void) {
    TunerConfig cfg = default_config();
    int16_t ring[TEST_RING_LEN];
    make_noise_ring(ring, TEST_RING_LEN, 31337u);

    ProgressLog log = {{0}, 0};
    TunerCore core;
    tuner_core_init(&core, &cfg);
    TunerResult r = tuner_core_run(&core, ring, TEST_RING_LEN,
                                   identity_process, NULL, progress_cb, &log);
    TEST_ASSERT_TRUE(r.ok);
    TEST_ASSERT_EQUAL_UINT32(r.candidates_run, log.count);
    for (uint32_t i = 1; i < log.count; i++) {
        TEST_ASSERT_TRUE(log.fractions[i] >= log.fractions[i - 1]);
        TEST_ASSERT_TRUE(log.fractions[i] <= 1.0f);
    }
}

// ── Input guards ──────────────────────────────────────────────────────────────

void test_invalid_inputs_rejected(void) {
    TunerConfig cfg = default_config();
    int16_t ring[16] = {0};
    TunerCore core;
    tuner_core_init(&core, &cfg);
    TunerResult r;

    r = tuner_core_run(NULL, ring, 16, identity_process, NULL, NULL, NULL);
    TEST_ASSERT_FALSE(r.ok);
    r = tuner_core_run(&core, NULL, 16, identity_process, NULL, NULL, NULL);
    TEST_ASSERT_FALSE(r.ok);
    r = tuner_core_run(&core, ring, 0, identity_process, NULL, NULL, NULL);
    TEST_ASSERT_FALSE(r.ok);
    r = tuner_core_run(&core, ring, 16, NULL, NULL, NULL, NULL);
    TEST_ASSERT_FALSE(r.ok);
}

void test_zero_ring_is_valid_and_scores_positive(void) {
    TunerConfig cfg = default_config();
    int16_t ring[TEST_RING_LEN];
    memset(ring, 0, sizeof(ring));  // silent noise floor

    TunerCore core;
    tuner_core_init(&core, &cfg);
    TunerResult r = tuner_core_run(&core, ring, TEST_RING_LEN,
                                   identity_process, NULL, NULL, NULL);
    TEST_ASSERT_TRUE(r.ok);
    TEST_ASSERT_TRUE(isfinite(r.score_db));
    TEST_ASSERT_TRUE(r.score_db > 0.0f);  // tone stack dominates silence
}

void run_tuner_core_tests(void) {
    RUN_TEST(test_goertzel_pure_tone_scores_at_cap);
    RUN_TEST(test_goertzel_white_noise_scores_negative);
    RUN_TEST(test_goertzel_tone_plus_noise_10db);
    RUN_TEST(test_synthesis_matches_reference_peak_and_snr);
    RUN_TEST(test_step_api_matches_blocking_run);
    RUN_TEST(test_run_is_deterministic);
    RUN_TEST(test_run_respects_hard_budget);
    RUN_TEST(test_run_respects_param_bounds);
    RUN_TEST(test_whitening_drives_to_zero_under_fake_process);
    RUN_TEST(test_progress_fractions_monotone_and_bounded);
    RUN_TEST(test_invalid_inputs_rejected);
    RUN_TEST(test_zero_ring_is_valid_and_scores_positive);
}
