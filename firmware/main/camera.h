#pragma once

/*
 * OV5640 bring-up via Espressif's esp32-camera component.
 *
 * Pins live-verified from stock firmware on 2026-09-07 — see the
 * comment block above PIN_CAM_* in camera.c for the exact
 * IN_SEL / OUT_SEL evidence.
 *
 * camera_start() calls esp_camera_init() with the Quarky-specific
 * pin table; on success the sensor is producing QVGA JPEG frames that
 * esp_camera_fb_get() can pull.
 */

#include "esp_err.h"

esp_err_t camera_start(void);
