#ifndef AUTOTUNE_CATCH_LOCK_H
#define AUTOTUNE_CATCH_LOCK_H
#include <stdbool.h>
#include <stdint.h>

#define CATCH_PARAM_COUNT    4
#define CATCH_WINDOW_ADC     50   /* single tunable constant: window width in ADC counts */
#define CATCH_DEBOUNCE_READS 3    /* jitter deadband: consecutive inside-reads to unlock */

typedef enum {
    CATCH_PARAM_REDUCTION = 0,
    CATCH_PARAM_SMOOTHING = 1,
    CATCH_PARAM_WHITENING = 2,
    CATCH_PARAM_AGGRESSION = 3,
} CatchParam;

/* Tuned values in parameter space (what the auto-tune found). */
typedef struct {
    float reduction_db;   /* 0..30   */
    float smoothing_pct;  /* 0..100  */
    float whitening_pct;  /* 0..100  */
    float noise_rescale;  /* 0..2    */
} CatchTuned;

typedef struct {
    uint16_t lock_mask;                      /* bit per param: 1 = still locked (tune governs) */
    uint8_t  inside_count[CATCH_PARAM_COUNT]; /* consecutive inside-window reads (deadband)     */
    uint16_t window_lo[CATCH_PARAM_COUNT];    /* current clamped window, ADC counts             */
    uint16_t window_hi[CATCH_PARAM_COUNT];
} CatchLock;

void catch_lock_init(CatchLock *lock);
void catch_lock_set_tuned(CatchLock *lock, const CatchTuned *tuned);
/* Returns true when ALL four parameters are unlocked (mask == 0) -> caller
   returns the box to manual. Pot order: [red, sm, wh, ag]. */
bool catch_lock_update(CatchLock *lock, const uint16_t pot_adc[CATCH_PARAM_COUNT]);
bool catch_lock_is_locked(const CatchLock *lock, CatchParam p);
#endif
