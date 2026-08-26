// Auto-tune catch-window lock (issue 7) — see catch_lock.h.
//
// After a tune completes, the four tuned values stay in control of the
// box until the user parks each pot inside the matching ADC window for
// CATCH_DEBOUNCE_READS consecutive reads. Unlock is one-way per session:
// once a bit clears it never re-locks (catch_lock_init starts a new
// session after each completed tune).
//
// Pure C (math.h only): no Arduino, no malloc, no global state.

#include "catch_lock.h"

#include <math.h>

// Tuned value -> ADC centre of the catch window. `scale` is the tuned
// value at the top of the pot travel (30 dB / 100% / 100%).
static uint16_t center_linear(float value, float scale) {
    float c = lroundf(value / scale * 1023.0f);
    if (c < 0.0f) {
        c = 0.0f;
    }
    if (c > 1023.0f) {
        c = 1023.0f;
    }
    return (uint16_t)c;
}

// The aggression pot is quadratic (noise_rescale = 2 * pos^2), so the ADC
// centre of a tuned value is pos * 1023 with pos = sqrt(nr / 2).
static uint16_t center_aggression(float noise_rescale) {
    float pos = noise_rescale / 2.0f;
    if (pos < 0.0f) {
        pos = 0.0f;
    }
    float c = lroundf(sqrtf(pos) * 1023.0f);
    if (c > 1023.0f) {
        c = 1023.0f;
    }
    return (uint16_t)c;
}

// Clamp the window around `center` to the 0..1023 pot travel, inclusive.
static void set_window(CatchLock *lock, CatchParam p, uint16_t center) {
    int lo = (int)center - CATCH_WINDOW_ADC;
    int hi = (int)center + CATCH_WINDOW_ADC;
    if (lo < 0) {
        lo = 0;
    }
    if (hi > 1023) {
        hi = 1023;
    }
    lock->window_lo[p] = (uint16_t)lo;
    lock->window_hi[p] = (uint16_t)hi;
}

void catch_lock_init(CatchLock *lock) {
    lock->lock_mask = (uint16_t)((1u << CATCH_PARAM_COUNT) - 1u);
    for (int i = 0; i < CATCH_PARAM_COUNT; i++) {
        lock->inside_count[i] = 0;
        lock->window_lo[i] = 0;
        lock->window_hi[i] = 0;
    }
}

void catch_lock_set_tuned(CatchLock *lock, const CatchTuned *tuned) {
    set_window(lock, CATCH_PARAM_REDUCTION, center_linear(tuned->reduction_db, 30.0f));
    set_window(lock, CATCH_PARAM_SMOOTHING, center_linear(tuned->smoothing_pct, 100.0f));
    set_window(lock, CATCH_PARAM_WHITENING, center_linear(tuned->whitening_pct, 100.0f));
    set_window(lock, CATCH_PARAM_AGGRESSION, center_aggression(tuned->noise_rescale));
}

bool catch_lock_update(CatchLock *lock, const uint16_t pot_adc[CATCH_PARAM_COUNT]) {
    for (int i = 0; i < CATCH_PARAM_COUNT; i++) {
        if ((lock->lock_mask & (1u << i)) == 0) {
            continue;  // already unlocked; one-way within this session
        }
        // lo=0, hi=0 means no window configured yet (set_tuned not called).
        if (lock->window_lo[i] == 0 && lock->window_hi[i] == 0) {
            continue;
        }
        if (pot_adc[i] >= lock->window_lo[i] && pot_adc[i] <= lock->window_hi[i]) {
            lock->inside_count[i]++;
            if (lock->inside_count[i] >= CATCH_DEBOUNCE_READS) {
                lock->lock_mask = (uint16_t)(lock->lock_mask & ~(1u << i));
            }
        } else {
            lock->inside_count[i] = 0;
        }
    }
    return lock->lock_mask == 0;
}

bool catch_lock_is_locked(const CatchLock *lock, CatchParam p) {
    return (lock->lock_mask & (1u << (unsigned)p)) != 0;
}
