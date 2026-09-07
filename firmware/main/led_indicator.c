/*
 * led_indicator — see led_indicator.h. The tick task runs on core 0 at
 * prio 3 (well below dispatcher and DMA). led_set is the raw WS2812B
 * writer defined in main.c; when host testing, it is macro-redirected
 * to a mock before this file is included.
 */

#include "led_indicator.h"

#ifndef LED_INDICATOR_HOST_TEST
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#endif

/* Real led_set is defined in main.c (WS2812B via RMT). Host tests
 * provide their own strong symbol directly, so the extern is omitted
 * from the host build path. */
#ifndef LED_INDICATOR_HOST_TEST
extern int led_set(uint8_t r, uint8_t g, uint8_t b);
#endif

/* --- Constants ---------------------------------------------------------- */
#define FLASH_MS  200
#define LISTEN_MS 5760                 /* MultiNet speech window (Task 9/10) */

/* Idle "dim white" — dim enough to signal power without being annoying.
 * (16, 16, 16) at 3 * 100 mA/mA per channel is ~4 mA total. */
static const led_rgb_t DIM_WHITE = { 16, 16, 16 };

/* --- Module state ------------------------------------------------------- */
static led_state_t s_state             = LED_STATE_IDLE;
static led_rgb_t   s_user_rgb          = { 16, 16, 16 };
static led_rgb_t   s_pending_user_rgb  = { 16, 16, 16 };
static bool        s_have_pending      = false;
static uint32_t    s_state_entered_ms  = 0;

/* --- Public API --------------------------------------------------------- */

int led_indicator_apply_rgb(command_source_t src, led_rgb_t rgb)
{
    (void)src;
    if (s_state == LED_STATE_IDLE || s_state == LED_STATE_USER_RGB) {
        s_user_rgb         = rgb;
        s_pending_user_rgb = rgb;
        s_have_pending     = false;
        s_state            = LED_STATE_USER_RGB;
    } else {
        /* Voice is holding the LED — remember but don't display yet. */
        s_pending_user_rgb = rgb;
        s_have_pending     = true;
    }
    return 0;
}

int led_indicator_apply_state(command_source_t src, led_state_t state)
{
    (void)src;
    s_state            = state;
    s_state_entered_ms = led_indicator_now_ms();
    return 0;
}

/* One tick of the FSM — checked in the tick task every 30 ms, or by host
 * tests directly with a virtual now_ms. Handles auto-transitions out of
 * transient states (OK/NACK flash timeout, LISTEN 5.76 s cap) then
 * writes the color for the current state. */
void led_indicator_tick(uint32_t now_ms)
{
    if (s_state == LED_STATE_OK || s_state == LED_STATE_NACK) {
        if (now_ms - s_state_entered_ms >= FLASH_MS) {
            if (s_have_pending) {
                s_user_rgb     = s_pending_user_rgb;
                s_have_pending = false;
            }
            /* Return to USER_RGB if the user ever set a colour, else IDLE. */
            s_state = (s_user_rgb.r == DIM_WHITE.r &&
                       s_user_rgb.g == DIM_WHITE.g &&
                       s_user_rgb.b == DIM_WHITE.b)
                    ? LED_STATE_IDLE : LED_STATE_USER_RGB;
        }
    } else if (s_state == LED_STATE_LISTENING) {
        if (now_ms - s_state_entered_ms >= LISTEN_MS) {
            /* Auto-timeout without a command → treat as NACK. */
            s_state            = LED_STATE_NACK;
            s_state_entered_ms = now_ms;
        }
    }

    led_rgb_t out;
    switch (s_state) {
    case LED_STATE_LISTENING: out = (led_rgb_t){ 0,   0,   255 }; break;
    case LED_STATE_OK:        out = (led_rgb_t){ 0,   200, 0   }; break;
    case LED_STATE_NACK:      out = (led_rgb_t){ 200, 0,   0   }; break;
    case LED_STATE_USER_RGB:  out = s_user_rgb;                    break;
    default:                  out = DIM_WHITE;                     break;
    }
    led_set(out.r, out.g, out.b);
}

void led_indicator_reset_for_test(void)
{
    s_state             = LED_STATE_IDLE;
    s_user_rgb          = DIM_WHITE;
    s_pending_user_rgb  = DIM_WHITE;
    s_have_pending      = false;
    s_state_entered_ms  = 0;
}

/* --- Target-only glue --------------------------------------------------- */
#ifndef LED_INDICATOR_HOST_TEST

uint32_t led_indicator_now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

static void led_indicator_task(void *arg)
{
    (void)arg;
    for (;;) {
        led_indicator_tick(led_indicator_now_ms());
        vTaskDelay(pdMS_TO_TICKS(30));
    }
}

esp_err_t led_indicator_start(void)
{
    led_indicator_reset_for_test();
    BaseType_t ok = xTaskCreatePinnedToCore(led_indicator_task, "led_ind",
                                            2048, NULL, 3, NULL, 0);
    return (ok == pdPASS) ? ESP_OK : ESP_ERR_NO_MEM;
}

#endif /* !LED_INDICATOR_HOST_TEST */
