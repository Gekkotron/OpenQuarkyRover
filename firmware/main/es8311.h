#pragma once

/*
 * Everest Semiconductor ES8311 low-power mono audio codec — minimal driver.
 *
 * M3 scope is the ADC (mic) path only: 16 kHz, 16-bit, mono, slave-mode,
 * MCLK sourced from the ESP32 (ES8311_MCLK_SOURCE = 0 in the ADF board
 * profile this hardware clones). No DAC / speaker output in M3 — that
 * comes when M4 adds voice feedback.
 *
 * Transport is bit-bang I²C (see i2c_bitbang.h; the ESP-IDF v5.3
 * i2c_master peripheral does not work reliably on the internal pull-ups
 * this board provides — see project memory `espidf-i2c-master-broken`).
 * Every call goes through bb_* on ES8311_I2C_SDA / ES8311_I2C_SCL from
 * pin_map... err, `pins.h`.
 *
 * Register recipe adapted from ESP-ADF
 * (components/audio_hal/driver/es8311/es8311.c @
 * e019b05cc424b2f3a303d0d5808c1b6814a2eb73, MIT), pared down to the
 * (MCLK=4.096 MHz, 16 kHz, mono ADC, slave) configuration.
 */

#include "esp_err.h"
#include <stdint.h>

/*
 * Register-level init: reset, clock config for the M3 audio format,
 * ADC path power-up, and default mic gain = 0 dB. Idempotent — calling
 * a second time re-applies the sequence. Must be preceded by an active
 * MCLK on ES8311_I2S_MCLK (LEDC or I²S).
 */
esp_err_t es8311_init(void);

/*
 * Analog PGA gain in dB, quantized to the codec's 6 dB steps (0..42 dB).
 * Values below 0 are clipped to 0 dB (the plan documents a -12..+30 API,
 * but the ES8311 PGA has no attenuation below 0 dB — signed values are
 * accepted for API parity but negative maps to 0). Above +42 clips to 42.
 */
esp_err_t es8311_set_mic_gain_db(int gain_db);

/*
 * Read chip ID (register 0xFD, expected 0x83) and chip version (0xFE).
 * Returns ESP_FAIL if either I²C read did not complete.
 */
esp_err_t es8311_read_id(uint8_t *id_out, uint8_t *ver_out);

/*
 * Unmute the ADC and start ramping digital volume — required before the
 * I²S RX path will deliver non-silent samples. In this driver es8311_init
 * already enables the ADC path in its final block, so es8311_start is a
 * no-op today; kept as a public API so the audio_capture / voice_pipeline
 * layer can call it symmetrically with es8311_stop.
 */
esp_err_t es8311_start(void);

/*
 * Mute the ADC and power down the analog path (SDPOUT reg 0x0A bit 6 = 1).
 * Preserves clock configuration — a subsequent es8311_start resumes without
 * a full re-init.
 */
esp_err_t es8311_stop(void);

/*
 * Diagnostic: read back the codec's control registers over bit-bang I²C
 * and printf a hex dump. Non-modifying (reads only) — safe to call any
 * time after `es8311_init`. Used by the `es-dump` REPL command and
 * inline by `voice-record` to check what the codec is actually set to
 * when the ADC path comes back silent.
 */
esp_err_t es8311_dump(void);
