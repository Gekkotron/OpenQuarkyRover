#pragma once

/*
 * Central command bus for the rover.
 *
 * The M1 REPL parsed verbs straight to hardware calls. That doesn't
 * scale to a second input source (voice) without either duplicating
 * parsing or forcing voice to serialize commands into REPL strings.
 * A tagged-union `command_t` published to a single FreeRTOS queue —
 * consumed by one dispatcher task on core 0 — keeps every input
 * source thin and every hardware handler singular.
 *
 * Diagnostic verbs that don't act on the robot (bb-scan, es-verify,
 * voice-record …) do NOT go through the bus; they call their target
 * directly. Bus latency would only mask bugs there.
 */

#include "esp_err.h"
#include <stdint.h>

typedef enum {
    CMD_NONE = 0,
    CMD_MOTOR,        /* args: int8_t left, right   (-100..100 percent) */
    CMD_STOP,         /* no args */
    CMD_LED_RGB,      /* args: r, g, b              (0..255)            */
    CMD_LED_STATE,    /* args: led_state_t          (voice FSM claim)   */
    CMD_SERVO,        /* args: uint8_t channel; uint16_t us             */
    CMD_BB_TLC_SET,   /* args: uint8_t channel, percent                 */
    CMD_VOICE_WAKE,   /* args: uint8_t model_index                       *
                       * Emitted by wake_word.c on WAKENET_DETECTED.     *
                       * No hardware effect yet — MultiNet (later task)  *
                       * hooks here to open its recognition window.      */
} command_id_t;

typedef enum { SRC_REPL, SRC_VOICE, SRC_INTERNAL } command_source_t;

typedef enum {
    LED_STATE_IDLE = 0,
    LED_STATE_LISTENING,
    LED_STATE_OK,
    LED_STATE_NACK,
    LED_STATE_USER_RGB,
} led_state_t;

/* Named RGB struct — the union member and every handler share this
 * exact type, so functions taking it by value link correctly.
 * Anonymous structs with the same layout are DIFFERENT types in C. */
typedef struct { uint8_t r, g, b; } led_rgb_t;

typedef struct {
    command_id_t     id;
    command_source_t source;
    union {
        struct { int8_t  left, right;         } motor;
        led_rgb_t                                led_rgb;
        struct { led_state_t state;           } led_state;
        struct { uint8_t channel; uint16_t us;} servo;
        struct { uint8_t channel, percent;    } bb_tlc;
        struct { uint8_t model_index;         } voice_wake;
    } as;
} command_t;

/* Start the dispatcher task on core 0. Idempotent-guarded — a second
 * call returns ESP_ERR_INVALID_STATE. */
esp_err_t command_bus_start(void);

/* Non-blocking enqueue. Returns ESP_ERR_NO_MEM if the queue is full;
 * dropped commands are the caller's problem (voice/REPL should log). */
esp_err_t command_bus_publish(const command_t *cmd);

/* Dispatch a single command synchronously. Split out of the task loop
 * so host tests can drive routing without a FreeRTOS scheduler. */
void command_bus_dispatch_one(const command_t *cmd);
