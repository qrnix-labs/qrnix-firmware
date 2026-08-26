// Host-side unit tests for the wire-contract emitter (ADR-0003).
// Built by the [env:native] PlatformIO environment against the pure-C
// emitter only; the Teensy sketch is not compiled here.

#include <string.h>
#include <unity.h>

// The pure-C emitter is compiled into the test directly (the native env
// builds no project sources; the sketch is Teensy-only).
#include "serial_contract.c"

static char line[512];

static ContractStatus full_status(void) {
    ContractStatus s;
    s.mode = 2;
    s.src = 'L';
    s.red = 40;
    s.sm = 55;
    s.wh = 30;
    s.ag = 1;
    s.tk = 1;
    s.pp = 0;
    s.clip = 0;
    s.blk_l = 12;
    s.blk_r = 10;
    s.in_l = 45;
    s.in_r = 47;
    s.lvl_l = 60;
    s.lvl_r = 62;
    s.out_l = 55;
    s.out_r = 58;
    s.bad = 3;
    s.have_tail = 1;
    s.snr_min = 12.5f;
    s.snr_avg = 14.2f;
    s.snr_max = 15.1f;
    s.bands_aggression = 4;
    s.bands_bypassed = 2;
    s.gain = 0.9f;
    s.mix = 0.5f;
    s.up = 100;
    s.ver = "0.3.12";
    s.sn = "0123456789ABCDEF";
    return s;
}

// One line, no embedded newlines, a JSON object.
static void assert_envelope(const char *l) {
    TEST_ASSERT_NOT_NULL(strchr(l, '{'));
    TEST_ASSERT_NULL(strchr(l, '\n'));
    TEST_ASSERT_NULL(strchr(l, '\r'));
    TEST_ASSERT_EQUAL('}', l[strlen(l) - 1]);
}

void setUp(void) {}
void tearDown(void) {}

void test_status_full_envelope(void) {
    ContractStatus s = full_status();
    size_t n = contract_status_line(line, sizeof(line), &s);

    TEST_ASSERT_TRUE(n > 0);
    TEST_ASSERT_TRUE(strstr(line, "{\"t\":\"status\",\"cv\":1,") != NULL);
    TEST_ASSERT_TRUE(strstr(line, "\"m\":2") != NULL);
    TEST_ASSERT_TRUE(strstr(line, "\"src\":\"L\"") != NULL);
    TEST_ASSERT_TRUE(strstr(line, "\"red\":40,") != NULL);
    TEST_ASSERT_TRUE(strstr(line, "\"sm\":55,") != NULL);
    TEST_ASSERT_TRUE(strstr(line, "\"wh\":30,") != NULL);
    TEST_ASSERT_TRUE(strstr(line, "\"ag\":1,") != NULL);
    TEST_ASSERT_TRUE(strstr(line, "\"tk\":1,") != NULL);
    TEST_ASSERT_TRUE(strstr(line, "\"pp\":0,") != NULL);
    TEST_ASSERT_TRUE(strstr(line, "\"clip\":0,") != NULL);
    TEST_ASSERT_TRUE(strstr(line, "\"blk_l\":12,") != NULL);
    TEST_ASSERT_TRUE(strstr(line, "\"blk_r\":10,") != NULL);
    TEST_ASSERT_TRUE(strstr(line, "\"in_l\":45,") != NULL);
    TEST_ASSERT_TRUE(strstr(line, "\"in_r\":47,") != NULL);
    TEST_ASSERT_TRUE(strstr(line, "\"lvl_l\":60,") != NULL);
    TEST_ASSERT_TRUE(strstr(line, "\"lvl_r\":62,") != NULL);
    TEST_ASSERT_TRUE(strstr(line, "\"out_l\":55,") != NULL);
    TEST_ASSERT_TRUE(strstr(line, "\"out_r\":58,") != NULL);
    TEST_ASSERT_TRUE(strstr(line, "\"bad\":3,") != NULL);
    TEST_ASSERT_TRUE(strstr(line, "\"snr_min\":12.5,") != NULL);
    TEST_ASSERT_TRUE(strstr(line, "\"snr_avg\":14.2,") != NULL);
    TEST_ASSERT_TRUE(strstr(line, "\"snr_max\":15.1,") != NULL);
    TEST_ASSERT_TRUE(strstr(line, "\"bands_aggression\":4,") != NULL);
    TEST_ASSERT_TRUE(strstr(line, "\"bands_bypassed\":2,") != NULL);
    TEST_ASSERT_TRUE(strstr(line, "\"gain\":0.900,") != NULL);
    TEST_ASSERT_TRUE(strstr(line, "\"mix\":0.500,") != NULL);
    TEST_ASSERT_TRUE(strstr(line, "\"up\":100,") != NULL);
    TEST_ASSERT_TRUE(strstr(line, "\"ver\":\"0.3.12\"") != NULL);
    TEST_ASSERT_TRUE(strstr(line, "\"ver\":\"0.3.12\",\"sn\":\"0123456789ABCDEF\"") != NULL);
    assert_envelope(line);
}

// The sn key is exactly 16 uppercase hex digits with no prefix (AC #17).
void test_status_sn_format(void) {
    ContractStatus s = full_status();
    contract_status_line(line, sizeof(line), &s);

    const char *sn = strstr(line, "\"sn\":\"");
    TEST_ASSERT_NOT_NULL(sn);
    sn += 6;  // skip `"sn":"`
    for (int i = 0; i < 16; i++) {
        const char c = sn[i];
        const int is_hex = (c >= '0' && c <= '9') || (c >= 'A' && c <= 'F');
        TEST_ASSERT_TRUE_MESSAGE(is_hex, "sn digit must be 0-9 or A-F");
    }
    TEST_ASSERT_EQUAL('\"', sn[16]);
    TEST_ASSERT_NULL(strstr(line, "\"sn\":\"0x"));
    assert_envelope(line);
}

void test_status_no_tail(void) {
    ContractStatus s = full_status();
    s.have_tail = 0;
    contract_status_line(line, sizeof(line), &s);

    TEST_ASSERT_NULL(strstr(line, "snr_"));
    TEST_ASSERT_NULL(strstr(line, "bands_"));
    TEST_ASSERT_NULL(strstr(line, "\"gain\""));
    TEST_ASSERT_NULL(strstr(line, "\"mix\""));
    // up/ver/sn are always present.
    TEST_ASSERT_TRUE(strstr(line, "\"up\":100,") != NULL);
    TEST_ASSERT_TRUE(strstr(line, "\"ver\":\"0.3.12\"") != NULL);
    TEST_ASSERT_TRUE(strstr(line, "\"sn\":\"0123456789ABCDEF\"") != NULL);
    assert_envelope(line);
}

void test_status_src_right_channel(void) {
    ContractStatus s = full_status();
    s.src = 'R';
    contract_status_line(line, sizeof(line), &s);
    TEST_ASSERT_TRUE(strstr(line, "\"src\":\"R\"") != NULL);
    assert_envelope(line);
}

void test_status_knobs_integer_no_decimal(void) {
    // The emitter prints integers; the legacy float format must not leak.
    ContractStatus s = full_status();
    contract_status_line(line, sizeof(line), &s);
    TEST_ASSERT_TRUE(strstr(line, "\"red\":40,") != NULL);
    TEST_ASSERT_NULL(strstr(line, "\"red\":40.0"));
}

void test_status_zero_mode_and_values(void) {
    ContractStatus s = full_status();
    s.mode = 0;
    s.red = 0;
    s.sm = 0;
    s.wh = 0;
    s.ag = 0;
    s.tk = 0;
    s.pp = 0;
    s.clip = 0;
    s.bad = 0;
    s.up = 0;
    contract_status_line(line, sizeof(line), &s);
    TEST_ASSERT_TRUE(strstr(line, "\"m\":0") != NULL);
    TEST_ASSERT_TRUE(strstr(line, "\"red\":0,") != NULL);
    TEST_ASSERT_TRUE(strstr(line, "\"bad\":0,") != NULL);
    TEST_ASSERT_TRUE(strstr(line, "\"up\":0,") != NULL);
    assert_envelope(line);
}

void test_boot_envelope(void) {
    size_t n = contract_boot_line(line, sizeof(line), "audio ready");
    TEST_ASSERT_TRUE(n > 0);
    TEST_ASSERT_EQUAL_STRING("{\"t\":\"boot\",\"stage\":\"audio ready\"}", line);
    assert_envelope(line);
}

void test_crash_envelope(void) {
    size_t n = contract_crash_line(line, sizeof(line), "CrashReport:");
    TEST_ASSERT_TRUE(n > 0);
    TEST_ASSERT_EQUAL_STRING("{\"t\":\"crash\",\"detail\":\"CrashReport:\"}", line);
    assert_envelope(line);
}

void test_string_escaping_quotes_and_backslash(void) {
    size_t n = contract_boot_line(line, sizeof(line), "a\"b\\c");
    TEST_ASSERT_TRUE(n > 0);
    TEST_ASSERT_EQUAL_STRING("{\"t\":\"boot\",\"stage\":\"a\\\"b\\\\c\"}", line);
    assert_envelope(line);
}

void test_string_escaping_control_bytes(void) {
    // Tab becomes \t, a raw control byte becomes \uXXXX.
    size_t n = contract_boot_line(line, sizeof(line), "x\ty\x01z");
    TEST_ASSERT_TRUE(n > 0);
    TEST_ASSERT_TRUE(strstr(line, "x\\ty\\u0001z") != NULL);
    assert_envelope(line);
}

void test_empty_strings(void) {
    contract_boot_line(line, sizeof(line), "");
    TEST_ASSERT_EQUAL_STRING("{\"t\":\"boot\",\"stage\":\"\"}", line);
    contract_crash_line(line, sizeof(line), NULL);
    TEST_ASSERT_EQUAL_STRING("{\"t\":\"crash\",\"detail\":\"\"}", line);
}

void test_truncation_is_safe(void) {
    ContractStatus s = full_status();
    char small[16];
    size_t n = contract_status_line(small, sizeof(small), &s);
    TEST_ASSERT_TRUE(n <= sizeof(small) - 1);
    TEST_ASSERT_EQUAL('\0', small[sizeof(small) - 1]);
}

void run_contract_tests(void) {
    RUN_TEST(test_status_full_envelope);
    RUN_TEST(test_status_sn_format);
    RUN_TEST(test_status_no_tail);
    RUN_TEST(test_status_src_right_channel);
    RUN_TEST(test_status_knobs_integer_no_decimal);
    RUN_TEST(test_status_zero_mode_and_values);
    RUN_TEST(test_boot_envelope);
    RUN_TEST(test_crash_envelope);
    RUN_TEST(test_string_escaping_quotes_and_backslash);
    RUN_TEST(test_string_escaping_control_bytes);
    RUN_TEST(test_empty_strings);
    RUN_TEST(test_truncation_is_safe);
}
