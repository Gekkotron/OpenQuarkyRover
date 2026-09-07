#pragma once

/*
 * OV5640 bring-up via Espressif's esp32-camera component.
 *
 * PIN CONFIGURATION IS A PLACEHOLDER. The values in camera.c come from
 * a common ESP32-S3-CAM board profile; they almost certainly do not
 * match the STEMpedia Quarky. Run the live-capture snippet described
 * in the README ("How we resolved the audio pin map" section — same
 * mem32 technique on IN_SEL signals 133..152 and OUT_SEL for the LEDC
 * XCLK pin) to get the real pins, then update PIN_CAM_* in camera.c.
 *
 * Once pins are correct, camera_start() succeeds and camera_capture()
 * returns a JPEG frame from esp_camera_fb_get().
 */

#include "esp_err.h"

esp_err_t camera_start(void);
