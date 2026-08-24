// Wire contract v1 emission (ADR-0003).
//
// Pure C, no Arduino dependencies: builds the JSON envelopes the QRNix
// manager parses. One object per line, `\n`-terminated by the caller,
// no embedded newlines, no pretty-printing.
//
// Contract version is a compile-time constant here; bumping it is an
// append-only protocol change per ADR-0003 (managers ignore unknown keys).
#ifndef SERIAL_CONTRACT_H
#define SERIAL_CONTRACT_H

#include <stddef.h>
#include <stdint.h>

#define WIRE_CONTRACT_VERSION 1
#ifdef __cplusplus
extern "C" {
#endif

/// All values of one status envelope. Tail fields are only emitted when
/// `have_tail` is non-zero (NR2 mode); `up`/`ver` are always emitted.
typedef struct {
    int mode;                  // 0 = OFF, 1 = NR1, 2 = NR2
    char src;                  // 'L' or 'R' (selected input channel)
    int red;                   // knob position, integer
    int sm;                    // knob position, integer
    int wh;                    // knob position, integer
    int ag;                    // aggression (0..2), integer
    int tk;                    // tone-kill enabled, 0/1
    int pp;                    // post-filter enabled, 0/1
    int clip;                  // clip latch active, 0/1
    uint32_t blk_l;            // block count, left
    uint32_t blk_r;            // block count, right
    uint32_t in_l;             // input peak, left
    uint32_t in_r;             // input peak, right
    int lvl_l;                 // input level (sqrt power), left
    int lvl_r;                 // input level (sqrt power), right
    uint32_t out_l;            // output peak, left
    uint32_t out_r;            // output peak, right
    uint32_t bad;              // non-finite output counter
    int have_tail;             // non-zero when the NR2 tail is present
    float snr_min;             // NR2 tail: SNR minimum
    float snr_avg;             // NR2 tail: SNR average
    float snr_max;             // NR2 tail: SNR maximum
    uint32_t bands_aggression; // NR2 tail: aggression bands
    uint32_t bands_bypassed;   // NR2 tail: bypassed bands
    float gain;                // NR2 tail: average gain
    float mix;                 // NR2 tail: average mixed gain
    uint64_t up;               // seconds since boot
    const char *ver;           // firmware version (SOFTWARE_VERSION)
} ContractStatus;

/// Build a status envelope into `buf` (NUL-terminated). Returns the number
/// of bytes written (excluding NUL), clamped to `cap - 1` on truncation —
/// callers use buffers sized for the full line.
size_t contract_status_line(char *buf, size_t cap, const ContractStatus *s);

/// Build a boot-stage envelope: `{"t":"boot","stage":"<stage>"}`.
size_t contract_boot_line(char *buf, size_t cap, const char *stage);

/// Build a crash envelope: `{"t":"crash","detail":"<detail>"}`.
/// One envelope per CrashReport line; `detail` must not contain newlines.
size_t contract_crash_line(char *buf, size_t cap, const char *detail);
#ifdef __cplusplus
}
#endif

#endif
