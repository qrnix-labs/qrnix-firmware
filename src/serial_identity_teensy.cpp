// Teensy-side wrapper for the serial-identity module (ADR-0006).
//
// The ONLY translation unit allowed to touch imxrt.h: reads the
// factory-blown OCOTP shadow registers once at startup and caches the
// formatted serial in the pure module. Not compiled by [env:native]
// (default `-<*>` source filter).
#include "serial_identity.h"

#include "imxrt.h"

void serial_identity_early_init(void) {
    serial_identity_set(HW_OCOTP_MAC0, HW_OCOTP_MAC1);
}
