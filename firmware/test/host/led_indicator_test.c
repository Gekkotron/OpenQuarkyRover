/*
 * led_indicator FSM — host-runnable state-machine tests. Redirects
 * led_set to a mock and drives led_indicator_tick with a virtual now_ms
 * so transient states can be walked deterministically.
 */

#include "unity.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "command_bus.h"   /* led_state_t, command_source_t, led_rgb_t */

/* --- Mock the WS2812B writer ------------------------------------------
 * Direct strong-symbol definition — led_indicator.c's target-only
 * extern is guarded by LED_INDICATOR_HOST_TEST so it doesn't conflict. */
static uint8_t g_r, g_g, g_b;
static int     g_writes;
int led_set(uint8_t r, uint8_t g, uint8_t b)
    { g_r = r; g_g = g; g_b = b; g_writes++; return 0; }

/* Compile-in the module under test. LED_INDICATOR_HOST_TEST skips the
 * FreeRTOS + esp_timer glue. */
#define LED_INDICATOR_HOST_TEST

/* Virtual clock — led_indicator_tick reads this to time transient states. */
static uint32_t g_now_ms = 0;
uint32_t led_indicator_now_ms(void) { return g_now_ms; }

/* led_indicator_start is target-only — no host test calls it, so no
 * shim is needed. led_indicator.c compiles without the FreeRTOS glue
 * when LED_INDICATOR_HOST_TEST is defined. */

#include "led_indicator.c"

/* --- Test scaffolding -------------------------------------------------- */
void setUp(void)
{
    g_r = g_g = g_b = 0;
    g_writes = 0;
    g_now_ms = 0;
    led_indicator_reset_for_test();
}
void tearDown(void) {}

/* --- Tests ------------------------------------------------------------- */

void test_idle_is_dim_white(void)
{
    led_indicator_tick(g_now_ms);
    TEST_ASSERT_EQUAL_HEX8(16, g_r);
    TEST_ASSERT_EQUAL_HEX8(16, g_g);
    TEST_ASSERT_EQUAL_HEX8(16, g_b);
}

void test_wake_goes_blue(void)
{
    led_indicator_apply_state(SRC_VOICE, LED_STATE_LISTENING);
    led_indicator_tick(g_now_ms);
    TEST_ASSERT_EQUAL_HEX8(0,   g_r);
    TEST_ASSERT_EQUAL_HEX8(0,   g_g);
    TEST_ASSERT_EQUAL_HEX8(255, g_b);
}

void test_ok_flashes_green_then_returns_to_idle(void)
{
    led_indicator_apply_state(SRC_VOICE, LED_STATE_OK);
    led_indicator_tick(g_now_ms);
    TEST_ASSERT_TRUE(g_g > 0 && g_r == 0);          /* green on entry */
    g_now_ms += 250;                                 /* past 200 ms flash */
    led_indicator_tick(g_now_ms);
    TEST_ASSERT_EQUAL_HEX8(16, g_r);                 /* back to dim white */
    TEST_ASSERT_EQUAL_HEX8(16, g_g);
    TEST_ASSERT_EQUAL_HEX8(16, g_b);
}

void test_nack_flashes_red_then_returns_to_idle(void)
{
    led_indicator_apply_state(SRC_VOICE, LED_STATE_NACK);
    led_indicator_tick(g_now_ms);
    TEST_ASSERT_TRUE(g_r > 0 && g_g == 0);           /* red on entry */
    g_now_ms += 250;
    led_indicator_tick(g_now_ms);
    TEST_ASSERT_EQUAL_HEX8(16, g_r);                 /* dim white after */
}

void test_user_rgb_survives_a_voice_cycle(void)
{
    /* User → red, then voice wake → blue, then OK flash → red restored. */
    led_rgb_t red = { 255, 0, 0 };
    led_indicator_apply_rgb(SRC_REPL, red);
    led_indicator_tick(g_now_ms);
    TEST_ASSERT_EQUAL_HEX8(255, g_r);

    led_indicator_apply_state(SRC_VOICE, LED_STATE_LISTENING);
    led_indicator_tick(g_now_ms);
    TEST_ASSERT_EQUAL_HEX8(255, g_b);                /* blue while listening */

    led_indicator_apply_state(SRC_VOICE, LED_STATE_OK);
    led_indicator_tick(g_now_ms);
    TEST_ASSERT_TRUE(g_g > 0);                       /* green flash */

    g_now_ms += 250;
    led_indicator_tick(g_now_ms);
    TEST_ASSERT_EQUAL_HEX8(255, g_r);                /* red restored */
    TEST_ASSERT_EQUAL_HEX8(0,   g_g);
    TEST_ASSERT_EQUAL_HEX8(0,   g_b);
}

void test_new_repl_led_during_listening_defers_until_after(void)
{
    /* Voice is listening; user tries to set green mid-window; the green
     * should NOT take effect until listening resolves. */
    led_indicator_apply_state(SRC_VOICE, LED_STATE_LISTENING);
    led_indicator_tick(g_now_ms);
    TEST_ASSERT_EQUAL_HEX8(255, g_b);

    led_rgb_t green = { 0, 255, 0 };
    led_indicator_apply_rgb(SRC_REPL, green);
    led_indicator_tick(g_now_ms);
    TEST_ASSERT_EQUAL_HEX8(255, g_b);                /* still blue */

    /* Listening ends via OK path → green flash → deferred green wins. */
    led_indicator_apply_state(SRC_VOICE, LED_STATE_OK);
    led_indicator_tick(g_now_ms);
    g_now_ms += 250;
    led_indicator_tick(g_now_ms);
    TEST_ASSERT_EQUAL_HEX8(255, g_g);                /* the deferred green */
    TEST_ASSERT_EQUAL_HEX8(0,   g_r);
    TEST_ASSERT_EQUAL_HEX8(0,   g_b);
}

void test_listening_times_out_to_nack_at_listen_ms(void)
{
    led_indicator_apply_state(SRC_VOICE, LED_STATE_LISTENING);
    led_indicator_tick(g_now_ms);
    TEST_ASSERT_EQUAL_HEX8(255, g_b);

    g_now_ms += 5760 + 1;                            /* past LISTEN_MS */
    led_indicator_tick(g_now_ms);
    TEST_ASSERT_TRUE(g_r > 0 && g_g == 0);           /* auto → NACK red */
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_idle_is_dim_white);
    RUN_TEST(test_wake_goes_blue);
    RUN_TEST(test_ok_flashes_green_then_returns_to_idle);
    RUN_TEST(test_nack_flashes_red_then_returns_to_idle);
    RUN_TEST(test_user_rgb_survives_a_voice_cycle);
    RUN_TEST(test_new_repl_led_during_listening_defers_until_after);
    RUN_TEST(test_listening_times_out_to_nack_at_listen_ms);
    return UNITY_END();
}
