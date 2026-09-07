#pragma once

/*
 * WS2812B indicator FSM. Owns the on-board RGB LED so voice states can
 * preempt user RGB without racing the raw led_set path.
 *
 * States:
 *   IDLE        — dim white, default at boot
 *   LISTENING   — solid blue while the wake-triggered speech window is open
 *   OK          — green flash for FLASH_MS, then return to previous user RGB
 *   NACK        — red flash for FLASH_MS (rejected / timeout), then return
 *   USER_RGB    — whatever the last REPL / CMD_LED_RGB set, held indefinitely
 *
 * Arbitration:
 *   - IDLE / USER_RGB accept new user RGB immediately.
 *   - LISTENING / OK / NACK defer new user RGB to `pending`; applied when
 *     the transient state returns to idle.
 *   - Any incoming led_state_t transition takes effect immediately.
 */

#include "esp_err.h"
#include "command_bus.h"
#include <stdint.h>

esp_err_t led_indicator_start(void);
int led_indicator_apply_rgb   (command_source_t src, led_rgb_t rgb);
int led_indicator_apply_state (command_source_t src, led_state_t state);

/* --- Testing hooks — host tests drive the FSM synchronously; firmware
 * code does not call these. */
void     led_indicator_reset_for_test(void);
void     led_indicator_tick(uint32_t now_ms);
uint32_t led_indicator_now_ms(void);   /* wraps esp_timer_get_time on device */
