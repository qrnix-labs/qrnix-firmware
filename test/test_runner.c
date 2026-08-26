// Shared Unity runner for the native test suite (ADR-0003 contract,
// auto-tune tuner core, catch-window lock).
//
// PlatformIO's Unity framework links every test file in test/ into one
// binary per suite, so exactly one main() may exist. Each test file
// exposes a run_<suite>_tests() registration function instead of its own
// main; this file is the single entry point.
#include <unity.h>

void run_contract_tests(void);
void run_serial_identity_tests(void);
void run_catch_lock_tests(void);
void run_tuner_core_tests(void);

int main(void) {
    UNITY_BEGIN();
    run_contract_tests();
    run_serial_identity_tests();
    run_catch_lock_tests();
    run_tuner_core_tests();
    return UNITY_END();
}
