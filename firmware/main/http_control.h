#pragma once

/*
 * Minimal HTTP control surface for the rover. Serves a single-page web
 * UI at "/" plus a handful of JSON endpoints that map straight onto
 * the existing board helpers (led_set / servo_set_deg / bb_motor_set):
 *
 *   GET  /                 — mobile-friendly HTML control page
 *   POST /api/motor        — {"l":int,"r":int}   both in -100..100
 *   POST /api/servo        — {"angle":int}       0..180
 *   POST /api/led          — {"r":int,"g":int,"b":int}  each 0..255
 *   POST /api/stop         — halt both motors, no body needed
 *
 * All endpoints reply with `{"ok":true}` on success or an HTTP 4xx
 * with `{"error":"..."}`. Requires Wi-Fi (soft-AP started elsewhere).
 */

#include "esp_err.h"

esp_err_t http_control_start(void);
