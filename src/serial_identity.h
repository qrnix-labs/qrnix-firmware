// Unit serial identity (ADR-0006): the factory-blown 64-bit OCOTP MAC
// value, formatted as 16 uppercase hex digits with no prefix.
//
// Pure C (snprintf only): no Arduino, no hardware access. The Teensy-side
// wrapper (serial_identity_teensy.cpp) is the only place that touches
// imxrt.h; this module also compiles into the native test env.
#ifndef SERIAL_IDENTITY_H
#define SERIAL_IDENTITY_H

#include <stdint.h>

#define SERIAL_IDENTITY_HEX_LEN 16

#ifdef __cplusplus
extern "C" {
#endif

/// Format `mac0`/`mac1` (HW_OCOTP_MAC0/MAC1) into `out` as 16 uppercase
/// hex digits, no prefix, NUL-terminated (`out` must hold 17 bytes).
/// Word order is MAC1:MAC0, so the low 6 digits decode to `mac0 & 0xFFFFFF`
/// — the host-visible USB serial the manager cross-checks (ADR-0006).
void serial_identity_format(uint32_t mac0, uint32_t mac1, char out[SERIAL_IDENTITY_HEX_LEN + 1]);

/// Cache the formatted serial. Call once at startup: the OCOTP value is
/// factory-blown and constant for the life of the chip.
void serial_identity_set(uint32_t mac0, uint32_t mac1);

/// The cached formatted serial ("" until `serial_identity_set`).
const char *serial_identity_get(void);

#ifdef __cplusplus
}
#endif

#endif
