/*
 * ES8311 driver — host-runnable register-init table test.
 *
 * Strategy: define ES8311_MOCK_TRANSPORT before including es8311.c so the
 * driver's internal w()/r() wrappers route to mock functions in this
 * file instead of pulling in i2c_bitbang / driver/gpio. Every write is
 * captured in g_log; every read returns a canned value. Tests assert on
 * the write log against the plan's spec (see the plan file § Task 3).
 */

#include "unity.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "pins.h"

/* esp_err.h is shimmed by ./esp_err.h alongside this file (picked up via
 * `-I.` first in the include path). pins.h is header-only #defines with
 * no ESP-IDF dependencies, so es8311.c can include it directly on host. */

/* --- Mock transport captured for later assertion ------------------------ */
#define MAX_LOG 256
typedef struct { uint8_t reg, val; } write_t;
static write_t g_log[MAX_LOG];
static size_t  g_log_n = 0;

int mock_bb_write_reg(int sda, int scl, uint8_t addr, uint8_t reg, uint8_t val)
{
    (void)sda; (void)scl;
    if (addr != ES8311_I2C_ADDR) return -1;
    if (g_log_n < MAX_LOG) g_log[g_log_n++] = (write_t){reg, val};
    return 0;
}

/* Canned reads: chip ID at 0xFD -> 0x83, chip version at 0xFE -> 0x11,
 * everything else -> 0. Only es8311_stop uses reads at present. */
int mock_bb_read_reg(int sda, int scl, uint8_t addr, uint8_t reg, uint8_t *out)
{
    (void)sda; (void)scl;
    if (addr != ES8311_I2C_ADDR || !out) return -1;
    switch (reg) {
    case 0xFD: *out = 0x83; break;
    case 0xFE: *out = 0x11; break;
    default:   *out = 0x00; break;
    }
    return 0;
}

/* Ask es8311.c to use the mocks. Must precede the #include. */
#define ES8311_MOCK_TRANSPORT
#include "es8311.c"

/* --- Test scaffolding --------------------------------------------------- */
void setUp(void)    { g_log_n = 0; memset(g_log, 0, sizeof g_log); }
void tearDown(void) {}

/* --- Tests -------------------------------------------------------------- */

void test_init_writes_reset_sequence_first(void)
{
    TEST_ASSERT_EQUAL(ESP_OK, es8311_init());
    TEST_ASSERT_GREATER_THAN(2, g_log_n);
    TEST_ASSERT_EQUAL_HEX8(0x00, g_log[0].reg);
    TEST_ASSERT_EQUAL_HEX8(0x1F, g_log[0].val);   /* reset asserted */
    TEST_ASSERT_EQUAL_HEX8(0x00, g_log[1].reg);
    TEST_ASSERT_EQUAL_HEX8(0x00, g_log[1].val);   /* reset released */
}

void test_init_configures_16khz_mono_mic_path(void)
{
    TEST_ASSERT_EQUAL(ESP_OK, es8311_init());
    /* Contract from the plan: driver must configure the ADC clock chain
     * for 16 kHz operation at MCLK = 4.096 MHz. That means writing at
     * least the ADC prescaler (REG 0x02) and the ADC osr/fs-mode
     * register (REG 0x03) — both must appear in the log. */
    bool saw_02 = false, saw_03 = false;
    for (size_t i = 0; i < g_log_n; i++) {
        if (g_log[i].reg == 0x02) saw_02 = true;
        if (g_log[i].reg == 0x03) saw_03 = true;
    }
    TEST_ASSERT_TRUE_MESSAGE(saw_02, "clk manager REG02 (prescaler) must be programmed");
    TEST_ASSERT_TRUE_MESSAGE(saw_03, "clk manager REG03 (adc osr/fs-mode) must be programmed");
}

void test_init_powers_up_adc(void)
{
    TEST_ASSERT_EQUAL(ESP_OK, es8311_init());
    /* REG 0x17 = ADC digital volume; the last write to it must have
     * bit 0 set — the plan tests that as "ADC path enabled / unmuted".
     * We accept any non-zero LSB write; ADF's default is 0xBF (0 dB). */
    bool adc_on = false;
    for (size_t i = 0; i < g_log_n; i++) {
        if (g_log[i].reg == 0x17 && (g_log[i].val & 0x01)) adc_on = true;
    }
    TEST_ASSERT_TRUE_MESSAGE(adc_on, "REG 0x17 must be programmed with bit 0 set");
}

void test_set_mic_gain_db_clips_to_range(void)
{
    TEST_ASSERT_EQUAL(ESP_OK, es8311_init());
    g_log_n = 0;
    TEST_ASSERT_EQUAL(ESP_OK, es8311_set_mic_gain_db(100));   /* clip up */
    TEST_ASSERT_EQUAL(ESP_OK, es8311_set_mic_gain_db(-100));  /* clip down */
    /* Both writes go to the analog PGA register (0x16). */
    TEST_ASSERT_EQUAL(2, g_log_n);
    TEST_ASSERT_EQUAL_HEX8(0x16, g_log[0].reg);
    TEST_ASSERT_EQUAL_HEX8(0x16, g_log[1].reg);
    /* Clip-up must land at the max PGA step (7 = 42 dB). */
    TEST_ASSERT_EQUAL_HEX8(0x07, g_log[0].val);
    /* Clip-down must land at the min PGA step (0 = 0 dB). */
    TEST_ASSERT_EQUAL_HEX8(0x00, g_log[1].val);
}

void test_read_id_returns_datasheet_values(void)
{
    uint8_t id = 0, ver = 0;
    TEST_ASSERT_EQUAL(ESP_OK, es8311_read_id(&id, &ver));
    TEST_ASSERT_EQUAL_HEX8(0x83, id);   /* datasheet product ID */
    TEST_ASSERT_EQUAL_HEX8(0x11, ver);  /* observed on this board */
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_init_writes_reset_sequence_first);
    RUN_TEST(test_init_configures_16khz_mono_mic_path);
    RUN_TEST(test_init_powers_up_adc);
    RUN_TEST(test_set_mic_gain_db_clips_to_range);
    RUN_TEST(test_read_id_returns_datasheet_values);
    return UNITY_END();
}
