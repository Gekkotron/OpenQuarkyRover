#pragma once

/*
 * Bit-banged I²C — low-level transport primitives.
 *
 * The ESP-IDF v5.3 `i2c_master` peripheral driver refuses to talk to the
 * TLC59108 on this board even when the chip ACKs a bit-bang probe (see
 * project memory `espidf-i2c-master-broken`). We use the same code path
 * for the ES8311 audio codec — its bus has the same characteristics
 * (internal pull-ups only, no external 2.2 kΩ resistors), so the
 * peripheral driver is not trusted there either.
 *
 * Open-drain convention on every driven pin:
 *   line HIGH → configure as INPUT, internal pull-up floats it high
 *   line LOW  → configure as OUTPUT_OD, driven to 0
 *
 * These helpers deliberately reconfigure `sda` / `scl` on every call so
 * that a single physical pin pair can be time-shared across multiple
 * higher-level drivers (motor / TLC59108 on A1/A2, ES8311 on GPIO 17/18).
 * Bus speed is fixed at ~100 kHz.
 */

#include "esp_err.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Pin-idle + pull-up reset. Call before the first probe on a fresh pair.
 * Named bb_i2c_init (not bb_init) to avoid a link-time collision with
 * esp_phy's prebuilt libphy.a, which exports its own `bb_init` symbol
 * (baseband init, unrelated) — duplicate-symbol at link time once Wi-Fi
 * pulls libphy.a into the build. */
void bb_i2c_init(int sda, int scl);

/* Address-only ACK probe (sends START + addr<<1|W + STOP). Returns true on
 * ACK. Safe on any (sda, scl) — reconfigures pins as OD outputs. */
bool bb_probe(int sda, int scl, uint8_t addr7);

/* Write N bytes to a device: START + addr<<1|W + b0 + b1 + … + STOP.
 * Returns true iff every byte (address included) was ACKed. */
bool bb_write(int sda, int scl, uint8_t addr7, const uint8_t *buf, size_t n);

/* Single-register write: START + addr<<1|W + reg + val + STOP. */
bool bb_write_reg(int sda, int scl, uint8_t addr7, uint8_t reg, uint8_t val);

/* Single-register read: START + addr<<1|W + reg + REPEATED-START +
 * addr<<1|R + read 1 byte + master-NACK + STOP. Byte read into *out. */
bool bb_read_reg(int sda, int scl, uint8_t addr7, uint8_t reg, uint8_t *out);
