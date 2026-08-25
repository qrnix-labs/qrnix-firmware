// Host-side unit tests for the auto-tune catch-window lock (issue 7).
// Built by the [env:native] PlatformIO environment against the pure-C
// module only; the Teensy sketch is not compiled here.

#include <unity.h>

// The pure-C catch-lock module is compiled into the test directly (the
// native env builds no project sources; the sketch is Teensy-only).
#include "autotune/catch_lock.c"

static CatchLock lock;

// reduction 15 dB -> centre 512 -> [462,562]; the other three tuned to 0
// -> windows [0,50]. 700 sits outside every window.
static CatchTuned interior_tuned(void) {
    CatchTuned t;
    t.reduction_db = 15.0f;
    t.smoothing_pct = 0.0f;
    t.whitening_pct = 0.0f;
    t.noise_rescale = 0.0f;
    return t;
}

static CatchTuned zero_tuned(void) {
    CatchTuned t;
    t.reduction_db = 0.0f;
    t.smoothing_pct = 0.0f;
    t.whitening_pct = 0.0f;
    t.noise_rescale = 0.0f;
    return t;
}

static CatchTuned max_tuned(void) {
    CatchTuned t;
    t.reduction_db = 30.0f;
    t.smoothing_pct = 100.0f;
    t.whitening_pct = 100.0f;
    t.noise_rescale = 2.0f;
    return t;
}

// Drive one pot inside its window for CATCH_DEBOUNCE_READS reads while the
// other three sit at 700 (outside every window used by these tests).
static void unlock_one(CatchLock *l, CatchParam p, uint16_t pot_value) {
    uint16_t pot[CATCH_PARAM_COUNT];
    for (int i = 0; i < CATCH_PARAM_COUNT; i++) {
        pot[i] = 700;
    }
    pot[p] = pot_value;
    for (int i = 0; i < CATCH_DEBOUNCE_READS; i++) {
        catch_lock_update(l, pot);
    }
}

static void unlock_all_four(CatchLock *l) {
    unlock_one(l, CATCH_PARAM_REDUCTION, 462); // inside [462,562]
    unlock_one(l, CATCH_PARAM_SMOOTHING, 25);  // inside [0,50]
    unlock_one(l, CATCH_PARAM_WHITENING, 25);  // inside [0,50]
    unlock_one(l, CATCH_PARAM_AGGRESSION, 25); // inside [0,50]
}

void test_window_edges_lo_and_hi_unlock(void) {
    CatchTuned t = interior_tuned();
    uint16_t pot[CATCH_PARAM_COUNT] = {700, 700, 700, 700};

    // lo boundary: 462 is inside [462,562].
    catch_lock_init(&lock);
    catch_lock_set_tuned(&lock, &t);
    pot[CATCH_PARAM_REDUCTION] = 462;
    for (int i = 0; i < CATCH_DEBOUNCE_READS; i++) {
        catch_lock_update(&lock, pot);
    }
    TEST_ASSERT_FALSE(catch_lock_is_locked(&lock, CATCH_PARAM_REDUCTION));

    // hi boundary: 562 is inside [462,562].
    catch_lock_init(&lock);
    catch_lock_set_tuned(&lock, &t);
    pot[CATCH_PARAM_REDUCTION] = 562;
    for (int i = 0; i < CATCH_DEBOUNCE_READS; i++) {
        catch_lock_update(&lock, pot);
    }
    TEST_ASSERT_FALSE(catch_lock_is_locked(&lock, CATCH_PARAM_REDUCTION));

    // The other three never moved: still locked.
    TEST_ASSERT_TRUE(catch_lock_is_locked(&lock, CATCH_PARAM_SMOOTHING));
    TEST_ASSERT_TRUE(catch_lock_is_locked(&lock, CATCH_PARAM_WHITENING));
    TEST_ASSERT_TRUE(catch_lock_is_locked(&lock, CATCH_PARAM_AGGRESSION));
}

void test_window_edges_just_outside_stay_locked(void) {
    CatchTuned t = interior_tuned();
    uint16_t pot[CATCH_PARAM_COUNT] = {700, 700, 700, 700};

    // 461 is one ADC count below lo: never unlocks, counter never moves.
    catch_lock_init(&lock);
    catch_lock_set_tuned(&lock, &t);
    pot[CATCH_PARAM_REDUCTION] = 461;
    for (int i = 0; i < CATCH_DEBOUNCE_READS * 10; i++) {
        TEST_ASSERT_FALSE(catch_lock_update(&lock, pot));
    }
    TEST_ASSERT_TRUE(catch_lock_is_locked(&lock, CATCH_PARAM_REDUCTION));
    TEST_ASSERT_EQUAL(0, lock.inside_count[CATCH_PARAM_REDUCTION]);

    // 563 is one ADC count above hi: same.
    catch_lock_init(&lock);
    catch_lock_set_tuned(&lock, &t);
    pot[CATCH_PARAM_REDUCTION] = 563;
    for (int i = 0; i < CATCH_DEBOUNCE_READS * 10; i++) {
        TEST_ASSERT_FALSE(catch_lock_update(&lock, pot));
    }
    TEST_ASSERT_TRUE(catch_lock_is_locked(&lock, CATCH_PARAM_REDUCTION));
}

void test_clamped_min_windows(void) {
    CatchTuned t = zero_tuned();
    uint16_t pot[CATCH_PARAM_COUNT] = {51, 51, 51, 51};

    // Tuned 0 for all: centre 0 clamps to window [0,50] at the travel end.
    catch_lock_init(&lock);
    catch_lock_set_tuned(&lock, &t);
    for (int i = 0; i < CATCH_PARAM_COUNT; i++) {
        TEST_ASSERT_EQUAL(0, lock.window_lo[i]);
        TEST_ASSERT_EQUAL(CATCH_WINDOW_ADC, lock.window_hi[i]);
    }

    // 51 is just above hi: stays locked.
    for (int i = 0; i < CATCH_DEBOUNCE_READS; i++) {
        catch_lock_update(&lock, pot);
    }
    TEST_ASSERT_TRUE(catch_lock_is_locked(&lock, CATCH_PARAM_REDUCTION));
    TEST_ASSERT_TRUE(catch_lock_is_locked(&lock, CATCH_PARAM_SMOOTHING));
    TEST_ASSERT_TRUE(catch_lock_is_locked(&lock, CATCH_PARAM_WHITENING));
    TEST_ASSERT_TRUE(catch_lock_is_locked(&lock, CATCH_PARAM_AGGRESSION));

    // Pot 0 is genuinely inside [0,50]: it unlocks all four.
    catch_lock_init(&lock);
    catch_lock_set_tuned(&lock, &t);
    pot[0] = 0;
    pot[1] = 0;
    pot[2] = 0;
    pot[3] = 0;
    for (int i = 0; i < CATCH_DEBOUNCE_READS; i++) {
        catch_lock_update(&lock, pot);
    }
    TEST_ASSERT_EQUAL(0, lock.lock_mask);
}

void test_clamped_max_windows(void) {
    CatchTuned t = max_tuned();
    uint16_t pot[CATCH_PARAM_COUNT] = {972, 972, 972, 972};

    // Centre 1023 clamps to window [973,1023] at the top travel end.
    catch_lock_init(&lock);
    catch_lock_set_tuned(&lock, &t);
    for (int i = 0; i < CATCH_PARAM_COUNT; i++) {
        TEST_ASSERT_EQUAL(1023 - CATCH_WINDOW_ADC, lock.window_lo[i]);
        TEST_ASSERT_EQUAL(1023, lock.window_hi[i]);
    }

    // 972 is just below lo: stays locked.
    for (int i = 0; i < CATCH_DEBOUNCE_READS; i++) {
        catch_lock_update(&lock, pot);
    }
    TEST_ASSERT_TRUE(catch_lock_is_locked(&lock, CATCH_PARAM_REDUCTION));
    TEST_ASSERT_TRUE(catch_lock_is_locked(&lock, CATCH_PARAM_SMOOTHING));
    TEST_ASSERT_TRUE(catch_lock_is_locked(&lock, CATCH_PARAM_WHITENING));
    TEST_ASSERT_TRUE(catch_lock_is_locked(&lock, CATCH_PARAM_AGGRESSION));

    // 1023 is inside [973,1023]: unlocks all four.
    catch_lock_init(&lock);
    catch_lock_set_tuned(&lock, &t);
    pot[0] = 1023;
    pot[1] = 1023;
    pot[2] = 1023;
    pot[3] = 1023;
    for (int i = 0; i < CATCH_DEBOUNCE_READS; i++) {
        catch_lock_update(&lock, pot);
    }
    TEST_ASSERT_EQUAL(0, lock.lock_mask);
}

void test_linear_inverse_mapping_centers(void) {
    CatchTuned t = interior_tuned();
    t.smoothing_pct = 50.0f;
    t.whitening_pct = 50.0f;

    // reduction 15 dB -> 512; smoothing 50 -> 512; whitening 50 -> 512.
    catch_lock_init(&lock);
    catch_lock_set_tuned(&lock, &t);
    TEST_ASSERT_EQUAL(512 - CATCH_WINDOW_ADC, lock.window_lo[CATCH_PARAM_REDUCTION]);
    TEST_ASSERT_EQUAL(512 + CATCH_WINDOW_ADC, lock.window_hi[CATCH_PARAM_REDUCTION]);
    TEST_ASSERT_EQUAL(512 - CATCH_WINDOW_ADC, lock.window_lo[CATCH_PARAM_SMOOTHING]);
    TEST_ASSERT_EQUAL(512 + CATCH_WINDOW_ADC, lock.window_hi[CATCH_PARAM_SMOOTHING]);
    TEST_ASSERT_EQUAL(512 - CATCH_WINDOW_ADC, lock.window_lo[CATCH_PARAM_WHITENING]);
    TEST_ASSERT_EQUAL(512 + CATCH_WINDOW_ADC, lock.window_hi[CATCH_PARAM_WHITENING]);

    // reduction 30 -> 1023 -> [973,1023]; whitening 100 -> 1023.
    CatchTuned m = max_tuned();
    catch_lock_init(&lock);
    catch_lock_set_tuned(&lock, &m);
    TEST_ASSERT_EQUAL(1023 - CATCH_WINDOW_ADC, lock.window_lo[CATCH_PARAM_REDUCTION]);
    TEST_ASSERT_EQUAL(1023, lock.window_hi[CATCH_PARAM_REDUCTION]);
    TEST_ASSERT_EQUAL(1023 - CATCH_WINDOW_ADC, lock.window_lo[CATCH_PARAM_WHITENING]);
    TEST_ASSERT_EQUAL(1023, lock.window_hi[CATCH_PARAM_WHITENING]);

    // smoothing 0 -> 0 -> [0,50].
    CatchTuned z = zero_tuned();
    catch_lock_init(&lock);
    catch_lock_set_tuned(&lock, &z);
    TEST_ASSERT_EQUAL(0, lock.window_lo[CATCH_PARAM_SMOOTHING]);
    TEST_ASSERT_EQUAL(CATCH_WINDOW_ADC, lock.window_hi[CATCH_PARAM_SMOOTHING]);
}

void test_quadratic_aggression_inverse_mapping(void) {
    CatchTuned t = interior_tuned();

    // noise_rescale 0.5 -> pos = sqrt(0.5/2) = 0.5 -> centre 512.
    t.noise_rescale = 0.5f;
    catch_lock_init(&lock);
    catch_lock_set_tuned(&lock, &t);
    TEST_ASSERT_EQUAL(512 - CATCH_WINDOW_ADC, lock.window_lo[CATCH_PARAM_AGGRESSION]);
    TEST_ASSERT_EQUAL(512 + CATCH_WINDOW_ADC, lock.window_hi[CATCH_PARAM_AGGRESSION]);
    TEST_ASSERT_EQUAL(512,
                      (lock.window_lo[CATCH_PARAM_AGGRESSION] +
                       lock.window_hi[CATCH_PARAM_AGGRESSION]) / 2);

    // 0 -> centre 0 -> [0,50].
    t.noise_rescale = 0.0f;
    catch_lock_init(&lock);
    catch_lock_set_tuned(&lock, &t);
    TEST_ASSERT_EQUAL(0, lock.window_lo[CATCH_PARAM_AGGRESSION]);
    TEST_ASSERT_EQUAL(CATCH_WINDOW_ADC, lock.window_hi[CATCH_PARAM_AGGRESSION]);

    // 2.0 -> centre 1023 -> [973,1023].
    t.noise_rescale = 2.0f;
    catch_lock_init(&lock);
    catch_lock_set_tuned(&lock, &t);
    TEST_ASSERT_EQUAL(1023 - CATCH_WINDOW_ADC, lock.window_lo[CATCH_PARAM_AGGRESSION]);
    TEST_ASSERT_EQUAL(1023, lock.window_hi[CATCH_PARAM_AGGRESSION]);
}

void test_jitter_deadband_requires_consecutive_reads(void) {
    CatchTuned t = interior_tuned();
    uint16_t inside[CATCH_PARAM_COUNT] = {512, 700, 700, 700};
    uint16_t outside[CATCH_PARAM_COUNT] = {461, 700, 700, 700};

    catch_lock_init(&lock);
    catch_lock_set_tuned(&lock, &t);

    // CATCH_DEBOUNCE_READS - 1 inside reads: still locked, counter primed.
    for (int i = 0; i < CATCH_DEBOUNCE_READS - 1; i++) {
        catch_lock_update(&lock, inside);
    }
    TEST_ASSERT_TRUE(catch_lock_is_locked(&lock, CATCH_PARAM_REDUCTION));
    TEST_ASSERT_EQUAL(CATCH_DEBOUNCE_READS - 1,
                      lock.inside_count[CATCH_PARAM_REDUCTION]);

    // One outside read resets the counter.
    catch_lock_update(&lock, outside);
    TEST_ASSERT_TRUE(catch_lock_is_locked(&lock, CATCH_PARAM_REDUCTION));
    TEST_ASSERT_EQUAL(0, lock.inside_count[CATCH_PARAM_REDUCTION]);

    // A fresh CATCH_DEBOUNCE_READS inside reads unlocks.
    for (int i = 0; i < CATCH_DEBOUNCE_READS; i++) {
        catch_lock_update(&lock, inside);
    }
    TEST_ASSERT_FALSE(catch_lock_is_locked(&lock, CATCH_PARAM_REDUCTION));

    // Exactly CATCH_DEBOUNCE_READS inside reads unlock (fresh lock).
    catch_lock_init(&lock);
    catch_lock_set_tuned(&lock, &t);
    for (int i = 0; i < CATCH_DEBOUNCE_READS - 1; i++) {
        catch_lock_update(&lock, inside);
    }
    TEST_ASSERT_TRUE(catch_lock_is_locked(&lock, CATCH_PARAM_REDUCTION));
    catch_lock_update(&lock, inside);
    TEST_ASSERT_FALSE(catch_lock_is_locked(&lock, CATCH_PARAM_REDUCTION));
}

void test_all_four_unlock_returns_true_when_mask_clears(void) {
    CatchTuned t = interior_tuned();
    uint16_t pot[CATCH_PARAM_COUNT] = {462, 700, 700, 700};

    catch_lock_init(&lock);
    catch_lock_set_tuned(&lock, &t);

    // Reduction: 462 inside [462,562].
    for (int i = 0; i < CATCH_DEBOUNCE_READS; i++) {
        TEST_ASSERT_FALSE(catch_lock_update(&lock, pot));
    }
    TEST_ASSERT_EQUAL(0x0E, lock.lock_mask);

    // Smoothing: 25 inside [0,50].
    pot[CATCH_PARAM_REDUCTION] = 700;
    pot[CATCH_PARAM_SMOOTHING] = 25;
    for (int i = 0; i < CATCH_DEBOUNCE_READS; i++) {
        TEST_ASSERT_FALSE(catch_lock_update(&lock, pot));
    }
    TEST_ASSERT_EQUAL(0x0C, lock.lock_mask);

    // Whitening: 25 inside [0,50].
    pot[CATCH_PARAM_SMOOTHING] = 700;
    pot[CATCH_PARAM_WHITENING] = 25;
    for (int i = 0; i < CATCH_DEBOUNCE_READS; i++) {
        TEST_ASSERT_FALSE(catch_lock_update(&lock, pot));
    }
    TEST_ASSERT_EQUAL(0x08, lock.lock_mask);

    // Aggression: 25 inside [0,50] — the last one: update turns true.
    pot[CATCH_PARAM_WHITENING] = 700;
    pot[CATCH_PARAM_AGGRESSION] = 25;
    for (int i = 0; i < CATCH_DEBOUNCE_READS - 1; i++) {
        TEST_ASSERT_FALSE(catch_lock_update(&lock, pot));
    }
    TEST_ASSERT_TRUE(catch_lock_update(&lock, pot));
    TEST_ASSERT_EQUAL(0, lock.lock_mask);
    TEST_ASSERT_FALSE(catch_lock_is_locked(&lock, CATCH_PARAM_REDUCTION));
    TEST_ASSERT_FALSE(catch_lock_is_locked(&lock, CATCH_PARAM_SMOOTHING));
    TEST_ASSERT_FALSE(catch_lock_is_locked(&lock, CATCH_PARAM_WHITENING));
    TEST_ASSERT_FALSE(catch_lock_is_locked(&lock, CATCH_PARAM_AGGRESSION));
}

void test_init_locks_all_and_reset_relocks(void) {
    CatchTuned t = interior_tuned();

    catch_lock_init(&lock);
    TEST_ASSERT_EQUAL(0x0F, lock.lock_mask);
    TEST_ASSERT_TRUE(catch_lock_is_locked(&lock, CATCH_PARAM_REDUCTION));
    TEST_ASSERT_TRUE(catch_lock_is_locked(&lock, CATCH_PARAM_SMOOTHING));
    TEST_ASSERT_TRUE(catch_lock_is_locked(&lock, CATCH_PARAM_WHITENING));
    TEST_ASSERT_TRUE(catch_lock_is_locked(&lock, CATCH_PARAM_AGGRESSION));

    // Full unlock cycle; update stays true once everything is unlocked.
    catch_lock_set_tuned(&lock, &t);
    unlock_all_four(&lock);
    TEST_ASSERT_EQUAL(0, lock.lock_mask);
    TEST_ASSERT_TRUE(
        catch_lock_update(&lock, (uint16_t[CATCH_PARAM_COUNT]){700, 700, 700, 700}));

    // A new session (after the completed tune) re-locks everything.
    catch_lock_init(&lock);
    TEST_ASSERT_EQUAL(0x0F, lock.lock_mask);
    TEST_ASSERT_TRUE(catch_lock_is_locked(&lock, CATCH_PARAM_REDUCTION));
    TEST_ASSERT_TRUE(catch_lock_is_locked(&lock, CATCH_PARAM_SMOOTHING));
    TEST_ASSERT_TRUE(catch_lock_is_locked(&lock, CATCH_PARAM_WHITENING));
    TEST_ASSERT_TRUE(catch_lock_is_locked(&lock, CATCH_PARAM_AGGRESSION));
}

void test_isolation_one_param_does_not_touch_others(void) {
    CatchTuned t = interior_tuned();
    uint16_t pot[CATCH_PARAM_COUNT] = {462, 700, 700, 700};

    catch_lock_init(&lock);
    catch_lock_set_tuned(&lock, &t);

    // Two inside reads prime only the reduction counter.
    catch_lock_update(&lock, pot);
    catch_lock_update(&lock, pot);
    TEST_ASSERT_EQUAL(2, lock.inside_count[CATCH_PARAM_REDUCTION]);
    TEST_ASSERT_EQUAL(0, lock.inside_count[CATCH_PARAM_SMOOTHING]);
    TEST_ASSERT_EQUAL(0, lock.inside_count[CATCH_PARAM_WHITENING]);
    TEST_ASSERT_EQUAL(0, lock.inside_count[CATCH_PARAM_AGGRESSION]);
    TEST_ASSERT_TRUE(catch_lock_is_locked(&lock, CATCH_PARAM_REDUCTION));

    // Reduction jitters out: only its own counter resets.
    pot[CATCH_PARAM_REDUCTION] = 461;
    catch_lock_update(&lock, pot);
    TEST_ASSERT_EQUAL(0, lock.inside_count[CATCH_PARAM_REDUCTION]);
    TEST_ASSERT_EQUAL(0, lock.inside_count[CATCH_PARAM_SMOOTHING]);

    // Smoothing unlocks on its own; reduction (counter 0) stays locked and
    // the untouched params keep their lock bits and zero counters.
    pot[CATCH_PARAM_REDUCTION] = 700;
    pot[CATCH_PARAM_SMOOTHING] = 25;
    for (int i = 0; i < CATCH_DEBOUNCE_READS; i++) {
        catch_lock_update(&lock, pot);
    }
    TEST_ASSERT_FALSE(catch_lock_is_locked(&lock, CATCH_PARAM_SMOOTHING));
    TEST_ASSERT_TRUE(catch_lock_is_locked(&lock, CATCH_PARAM_REDUCTION));
    TEST_ASSERT_TRUE(catch_lock_is_locked(&lock, CATCH_PARAM_WHITENING));
    TEST_ASSERT_TRUE(catch_lock_is_locked(&lock, CATCH_PARAM_AGGRESSION));
    TEST_ASSERT_EQUAL(0, lock.inside_count[CATCH_PARAM_REDUCTION]);
    TEST_ASSERT_EQUAL(0, lock.inside_count[CATCH_PARAM_WHITENING]);
    TEST_ASSERT_EQUAL(0, lock.inside_count[CATCH_PARAM_AGGRESSION]);
    TEST_ASSERT_EQUAL(0x0D, lock.lock_mask);
}

void test_no_window_configured_never_unlocks(void) {
    uint16_t pot[CATCH_PARAM_COUNT] = {512, 25, 700, 25};

    // No set_tuned: windows stay zeroed (lo=0, hi=0) -> nothing unlocks.
    catch_lock_init(&lock);
    for (int i = 0; i < CATCH_PARAM_COUNT; i++) {
        TEST_ASSERT_EQUAL(0, lock.window_lo[i]);
        TEST_ASSERT_EQUAL(0, lock.window_hi[i]);
    }
    for (int i = 0; i < CATCH_DEBOUNCE_READS * 10; i++) {
        TEST_ASSERT_FALSE(catch_lock_update(&lock, pot));
    }
    TEST_ASSERT_EQUAL(0x0F, lock.lock_mask);
    for (int i = 0; i < CATCH_PARAM_COUNT; i++) {
        TEST_ASSERT_EQUAL(0, lock.inside_count[i]);
        TEST_ASSERT_TRUE(catch_lock_is_locked(&lock, (CatchParam)i));
    }
}

void run_catch_lock_tests(void) {
    RUN_TEST(test_window_edges_lo_and_hi_unlock);
    RUN_TEST(test_window_edges_just_outside_stay_locked);
    RUN_TEST(test_clamped_min_windows);
    RUN_TEST(test_clamped_max_windows);
    RUN_TEST(test_linear_inverse_mapping_centers);
    RUN_TEST(test_quadratic_aggression_inverse_mapping);
    RUN_TEST(test_jitter_deadband_requires_consecutive_reads);
    RUN_TEST(test_all_four_unlock_returns_true_when_mask_clears);
    RUN_TEST(test_init_locks_all_and_reset_relocks);
    RUN_TEST(test_isolation_one_param_does_not_touch_others);
    RUN_TEST(test_no_window_configured_never_unlocks);
}
