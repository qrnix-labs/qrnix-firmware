// Host-side unit tests for the serial-identity formatter (ADR-0006).
// Built by the [env:native] PlatformIO environment against the pure-C
// module only; the OCOTP read wrapper (serial_identity_teensy.cpp) is
// Teensy-only and not compiled here.

#include <string.h>
#include <unity.h>

// The pure-C module is compiled into the test directly, mirroring
// test_contract.c.
#include "serial_identity.c"

static char sn[SERIAL_IDENTITY_HEX_LEN + 1];

void test_format_full_width(void) {
    serial_identity_format(0x89ABCDEFu, 0x01234567u, sn);
    TEST_ASSERT_EQUAL_STRING("0123456789ABCDEF", sn);
}

void test_format_leading_zeros_preserved(void) {
    serial_identity_format(0x0000002Au, 0x00000000u, sn);
    TEST_ASSERT_EQUAL_STRING("000000000000002A", sn);
}

void test_format_uppercase_no_prefix(void) {
    serial_identity_format(0xCAFEF00Du, 0xDEADBEEFu, sn);
    TEST_ASSERT_EQUAL_STRING("DEADBEEFCAFEF00D", sn);
    TEST_ASSERT_NULL(strstr(sn, "0x"));
}

void test_low_6_digits_match_mac0_low(void) {
    // The low 6 hex digits are MAC0 & 0xFFFFFF — the host-visible USB
    // serial the manager cross-checks (ADR-0006).
    serial_identity_format(0x00123456u, 0x00000000u, sn);
    TEST_ASSERT_EQUAL_STRING("0000000000123456", sn);
    const char *low = sn + SERIAL_IDENTITY_HEX_LEN - 6;
    TEST_ASSERT_EQUAL_STRING("123456", low);
}

void test_set_get_cache(void) {
    serial_identity_set(0x89ABCDEFu, 0x01234567u);
    TEST_ASSERT_EQUAL_STRING("0123456789ABCDEF", serial_identity_get());
    // A second set replaces the cached value (the device calls set exactly
    // once at startup; the cache itself is plain storage).
    serial_identity_set(0x0000002Au, 0x00000000u);
    TEST_ASSERT_EQUAL_STRING("000000000000002A", serial_identity_get());
}

void run_serial_identity_tests(void) {
    RUN_TEST(test_format_full_width);
    RUN_TEST(test_format_leading_zeros_preserved);
    RUN_TEST(test_format_uppercase_no_prefix);
    RUN_TEST(test_low_6_digits_match_mac0_low);
    RUN_TEST(test_set_get_cache);
}
