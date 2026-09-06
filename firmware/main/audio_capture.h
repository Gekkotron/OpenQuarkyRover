#pragma once

/*
 * I²S RX capture for the ES8311 MEMS microphone.
 *
 * Fixed format: 16 kHz, 16-bit signed, mono, little-endian PCM — matches
 * ESP-SR (WakeNet + MultiNet) expectations, so no resampling downstream.
 *
 * Producer task on core 1 pulls DMA-delivered frames from the I²S RX
 * channel and enqueues each 512-sample block (1024 bytes, 32 ms) onto
 * a caller-owned FreeRTOS queue. If the consumer can't keep up the
 * frame is dropped and audio_capture_dropped_frames() ticks — this is
 * how the voice pipeline notices its own starvation.
 *
 * Prerequisite: es8311_init() must have run first. audio_capture also
 * drives MCLK on ES8311_I2S_MCLK via the I²S peripheral (not LEDC), so
 * any prior LEDC channel on that pin must be released.
 */

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <stdint.h>

/* 512 samples × 2 bytes = 1024 B per frame = 32 ms @ 16 kHz. Matches ESP-SR's
 * feed chunk size so downstream doesn't have to re-block. */
#define AUDIO_CAPTURE_FRAME_SAMPLES 512
#define AUDIO_CAPTURE_FRAME_BYTES   (AUDIO_CAPTURE_FRAME_SAMPLES * sizeof(int16_t))

/*
 * Which I²S slot to sample from in mono mode. Some codecs put the mic on
 * the LEFT slot (LRCK low half), some on the RIGHT — try both if the
 * default gives silence. `AUDIO_CAPTURE_SLOT_BOTH` reads a 2-channel
 * stream (unused in M3, kept for M4 duplex).
 */
typedef enum {
    AUDIO_CAPTURE_SLOT_LEFT = 0,
    AUDIO_CAPTURE_SLOT_RIGHT,
    AUDIO_CAPTURE_SLOT_BOTH,
} audio_capture_slot_t;

/*
 * Configuration for one capture session. Zero-init defaults to the
 * ES8311 pins from pins.h with LEFT-slot mono — matches the ES-ADF
 * Korvo-2 v3 defaults.
 */
typedef struct {
    int                   din_gpio;   /* 0 → use MIC_I2S_SD from pins.h */
    int                   bclk_gpio;  /* 0 → use MIC_I2S_SCK from pins.h */
    int                   ws_gpio;    /* 0 → use MIC_I2S_WS  from pins.h */
    audio_capture_slot_t  slot;       /* mono slot select */
} audio_capture_config_t;

/*
 * Start the I²S RX path with default pins + LEFT slot. `out_queue` must
 * be created by the caller with item_size = AUDIO_CAPTURE_FRAME_BYTES;
 * the producer task takes a non-owning reference to it. Idempotent —
 * a second call while running returns ESP_ERR_INVALID_STATE.
 */
esp_err_t audio_capture_start(QueueHandle_t out_queue);

/*
 * Same as audio_capture_start but with an explicit override for pins /
 * slot select — used by voice-scan diagnostics to probe DIN candidates.
 */
esp_err_t audio_capture_start_ex(QueueHandle_t out_queue,
                                 const audio_capture_config_t *cfg);

/*
 * Tear down the producer task, disable + delete the I²S channel, and
 * release the queue reference. Safe to call when not running.
 */
esp_err_t audio_capture_stop(void);

/*
 * Running total of frames dropped because out_queue was full. Reset on
 * every audio_capture_start.
 */
uint32_t audio_capture_dropped_frames(void);
