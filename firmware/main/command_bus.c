/*
 * command_bus — see command_bus.h for the API.
 *
 * The dispatcher task lives on core 0 at priority 5. Every action verb
 * eventually routes to a handler in another translation unit — motor
 * drive in main.c's bit-bang TLC59108 helpers, LED in led_indicator.c
 * (Task 6), servo/TLC in extracted helpers from Task 7. Until those
 * later tasks provide strong symbols, weak stubs below let this module
 * build in isolation and keep host tests happy.
 */

#include "command_bus.h"

#ifndef COMMAND_BUS_HOST_TEST
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_log.h"
static const char *TAG = "command_bus";
#endif

/* --- Handler entry points -------------------------------------------------
 * Real definitions land in other files. Weak stubs let the dispatcher
 * link and (harmlessly) no-op until those handlers arrive. */
#if defined(__GNUC__) && !defined(COMMAND_BUS_HOST_TEST)
#  define WEAK __attribute__((weak))
#else
#  define WEAK
#endif

extern int bb_motor_set(int left_pct, int right_pct);
extern int led_indicator_apply_rgb(command_source_t src, led_rgb_t rgb);
extern int led_indicator_apply_state(command_source_t src, led_state_t state);
extern int servo_set_us(uint8_t channel, uint16_t us);
extern int bb_tlc_set_pct(uint8_t channel, uint8_t percent);

/* Target build: weak stubs so this module links before Tasks 6/7 supply
 * the real handlers. Host build: the test file provides strong symbols,
 * so the stubs must NOT be defined (or the linker sees duplicates). */
#ifndef COMMAND_BUS_HOST_TEST
WEAK int led_indicator_apply_rgb(command_source_t src, led_rgb_t rgb)
    { (void)src; (void)rgb; return 0; }
WEAK int led_indicator_apply_state(command_source_t src, led_state_t state)
    { (void)src; (void)state; return 0; }
WEAK int servo_set_us(uint8_t channel, uint16_t us)
    { (void)channel; (void)us; return 0; }
WEAK int bb_tlc_set_pct(uint8_t channel, uint8_t percent)
    { (void)channel; (void)percent; return 0; }
#endif

void command_bus_dispatch_one(const command_t *c)
{
    if (!c) return;
    switch (c->id) {
    case CMD_MOTOR:
        bb_motor_set(c->as.motor.left, c->as.motor.right);
        break;
    case CMD_STOP:
        bb_motor_set(0, 0);
        break;
    case CMD_LED_RGB:
        led_indicator_apply_rgb(c->source, c->as.led_rgb);
        break;
    case CMD_LED_STATE:
        led_indicator_apply_state(c->source, c->as.led_state.state);
        break;
    case CMD_SERVO:
        servo_set_us(c->as.servo.channel, c->as.servo.us);
        break;
    case CMD_BB_TLC_SET:
        bb_tlc_set_pct(c->as.bb_tlc.channel, c->as.bb_tlc.percent);
        break;
    case CMD_VOICE_WAKE:
        /* Signaling event — logged so the bus trace shows wake fired.
         * MultiNet integration will replace this with a call that opens
         * the recognition window and transitions the LED to LISTENING. */
#ifndef COMMAND_BUS_HOST_TEST
        ESP_LOGI(TAG, "wake fired (src=%d model_idx=%u)",
                 (int)c->source, (unsigned)c->as.voice_wake.model_index);
#endif
        break;
    case CMD_NONE:
    default:
        break;
    }
}

/* Everything below is target-only — the host test only exercises
 * command_bus_dispatch_one and never links FreeRTOS. */
#ifndef COMMAND_BUS_HOST_TEST

static QueueHandle_t s_queue  = NULL;

static void dispatcher_task(void *arg)
{
    (void)arg;
    command_t c;
    for (;;) {
        if (xQueueReceive(s_queue, &c, portMAX_DELAY) != pdTRUE) continue;
        ESP_LOGD(TAG, "src=%d id=%d", (int)c.source, (int)c.id);
        command_bus_dispatch_one(&c);
    }
}

esp_err_t command_bus_start(void)
{
    if (s_queue) return ESP_ERR_INVALID_STATE;
    s_queue = xQueueCreate(8, sizeof(command_t));
    if (!s_queue) return ESP_ERR_NO_MEM;
    BaseType_t ok = xTaskCreatePinnedToCore(dispatcher_task, "cmd_disp",
                                            4096, NULL, 5, NULL, 0);
    if (ok != pdPASS) {
        vQueueDelete(s_queue);
        s_queue = NULL;
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

esp_err_t command_bus_publish(const command_t *cmd)
{
    if (!s_queue) return ESP_ERR_INVALID_STATE;
    if (!cmd)     return ESP_ERR_INVALID_ARG;
    return (xQueueSend(s_queue, cmd, 0) == pdTRUE) ? ESP_OK : ESP_ERR_NO_MEM;
}

#endif /* !COMMAND_BUS_HOST_TEST */
