/*
 * command_bus — host-runnable routing test.
 *
 * Compiles command_bus.c with COMMAND_BUS_HOST_TEST defined so the
 * FreeRTOS / esp_log dependencies are stubbed out and the weak WEAK
 * expands to nothing (mocks below become the strong symbols). Every
 * handler the dispatcher can call is captured; assertions run against
 * that capture.
 */

#include "unity.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* Mock handler state (populated by the mocks below, checked by tests). */
static int     g_motor_calls = 0;
static int     g_motor_l = -1, g_motor_r = -1;
static int     g_led_rgb_calls = 0;
static uint8_t g_led_r, g_led_g, g_led_b;
static int     g_led_state_calls = 0;
static int     g_led_state_last = -1;
static int     g_servo_calls = 0;
static uint8_t g_servo_ch;   static uint16_t g_servo_us;
static int     g_tlc_calls = 0;
static uint8_t g_tlc_ch;     static uint8_t  g_tlc_pct;

/* Strong-symbol handlers — these ARE the handlers command_bus.c calls. */
int bb_motor_set(int l, int r)                     { g_motor_calls++; g_motor_l=l; g_motor_r=r; return 0; }
int servo_set_us(uint8_t ch, uint16_t us)          { g_servo_calls++; g_servo_ch=ch; g_servo_us=us; return 0; }
int bb_tlc_set_pct(uint8_t ch, uint8_t pct)        { g_tlc_calls++;   g_tlc_ch=ch; g_tlc_pct=pct; return 0; }

/* Need the type up here so the LED handler prototypes can name it. */
#define COMMAND_BUS_HOST_TEST
#include "command_bus.h"
int led_indicator_apply_rgb(command_source_t src, led_rgb_t rgb)
    { (void)src; g_led_rgb_calls++; g_led_r=rgb.r; g_led_g=rgb.g; g_led_b=rgb.b; return 0; }
int led_indicator_apply_state(command_source_t src, led_state_t s)
    { (void)src; g_led_state_calls++; g_led_state_last=(int)s; return 0; }

#include "command_bus.c"

void setUp(void)
{
    g_motor_calls = 0; g_motor_l = -1; g_motor_r = -1;
    g_led_rgb_calls = 0; g_led_r = g_led_g = g_led_b = 0;
    g_led_state_calls = 0; g_led_state_last = -1;
    g_servo_calls = 0; g_servo_ch = 0; g_servo_us = 0;
    g_tlc_calls = 0; g_tlc_ch = 0; g_tlc_pct = 0;
}
void tearDown(void) {}

void test_motor_command_calls_bb_motor_set(void)
{
    command_t c = { .id = CMD_MOTOR, .source = SRC_REPL,
                    .as.motor = { .left = 60, .right = -40 } };
    command_bus_dispatch_one(&c);
    TEST_ASSERT_EQUAL(1, g_motor_calls);
    TEST_ASSERT_EQUAL(60,  g_motor_l);
    TEST_ASSERT_EQUAL(-40, g_motor_r);
}

void test_stop_command_zeros_motors(void)
{
    command_t c = { .id = CMD_STOP, .source = SRC_VOICE };
    command_bus_dispatch_one(&c);
    TEST_ASSERT_EQUAL(1, g_motor_calls);
    TEST_ASSERT_EQUAL(0, g_motor_l);
    TEST_ASSERT_EQUAL(0, g_motor_r);
}

void test_led_rgb_command_calls_apply_rgb(void)
{
    command_t c = { .id = CMD_LED_RGB, .source = SRC_REPL,
                    .as.led_rgb = { 255, 128, 0 } };
    command_bus_dispatch_one(&c);
    TEST_ASSERT_EQUAL(1, g_led_rgb_calls);
    TEST_ASSERT_EQUAL_HEX8(255, g_led_r);
    TEST_ASSERT_EQUAL_HEX8(128, g_led_g);
    TEST_ASSERT_EQUAL_HEX8(0,   g_led_b);
}

void test_led_state_command_calls_apply_state(void)
{
    command_t c = { .id = CMD_LED_STATE, .source = SRC_VOICE,
                    .as.led_state = { .state = LED_STATE_LISTENING } };
    command_bus_dispatch_one(&c);
    TEST_ASSERT_EQUAL(1, g_led_state_calls);
    TEST_ASSERT_EQUAL(LED_STATE_LISTENING, g_led_state_last);
}

void test_servo_command_calls_servo_set_us(void)
{
    command_t c = { .id = CMD_SERVO, .source = SRC_REPL,
                    .as.servo = { .channel = 1, .us = 1500 } };
    command_bus_dispatch_one(&c);
    TEST_ASSERT_EQUAL(1, g_servo_calls);
    TEST_ASSERT_EQUAL(1, g_servo_ch);
    TEST_ASSERT_EQUAL(1500, g_servo_us);
}

void test_bb_tlc_command_calls_bb_tlc_set_pct(void)
{
    command_t c = { .id = CMD_BB_TLC_SET, .source = SRC_REPL,
                    .as.bb_tlc = { .channel = 4, .percent = 50 } };
    command_bus_dispatch_one(&c);
    TEST_ASSERT_EQUAL(1, g_tlc_calls);
    TEST_ASSERT_EQUAL(4, g_tlc_ch);
    TEST_ASSERT_EQUAL(50, g_tlc_pct);
}

void test_unknown_command_is_a_noop(void)
{
    command_t c = { .id = CMD_NONE, .source = SRC_INTERNAL };
    command_bus_dispatch_one(&c);
    TEST_ASSERT_EQUAL(0, g_motor_calls);
    TEST_ASSERT_EQUAL(0, g_led_rgb_calls);
    TEST_ASSERT_EQUAL(0, g_led_state_calls);
    TEST_ASSERT_EQUAL(0, g_servo_calls);
    TEST_ASSERT_EQUAL(0, g_tlc_calls);
}

void test_null_command_pointer_is_safe(void)
{
    command_bus_dispatch_one(NULL);
    TEST_ASSERT_EQUAL(0, g_motor_calls);
    TEST_ASSERT_EQUAL(0, g_led_rgb_calls);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_motor_command_calls_bb_motor_set);
    RUN_TEST(test_stop_command_zeros_motors);
    RUN_TEST(test_led_rgb_command_calls_apply_rgb);
    RUN_TEST(test_led_state_command_calls_apply_state);
    RUN_TEST(test_servo_command_calls_servo_set_us);
    RUN_TEST(test_bb_tlc_command_calls_bb_tlc_set_pct);
    RUN_TEST(test_unknown_command_is_a_noop);
    RUN_TEST(test_null_command_pointer_is_safe);
    return UNITY_END();
}
