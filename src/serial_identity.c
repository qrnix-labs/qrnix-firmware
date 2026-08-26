// Unit serial identity (ADR-0006) — see serial_identity.h.
//
// Pure C (snprintf only): no Arduino, no malloc, no imxrt.h. The OCOTP
// shadow registers are read once by the Teensy-side wrapper
// (serial_identity_teensy.cpp) and handed in as plain 32-bit words, so
// this file compiles into the native test env too.

#include "serial_identity.h"

#include <stdio.h>

static char cached[SERIAL_IDENTITY_HEX_LEN + 1] = "";

void serial_identity_format(uint32_t mac0, uint32_t mac1, char out[SERIAL_IDENTITY_HEX_LEN + 1]) {
    snprintf(out, SERIAL_IDENTITY_HEX_LEN + 1, "%016llX",
             (unsigned long long)(((uint64_t)mac1 << 32) | mac0));
}

void serial_identity_set(uint32_t mac0, uint32_t mac1) {
    serial_identity_format(mac0, mac1, cached);
}

const char *serial_identity_get(void) {
    return cached;
}
