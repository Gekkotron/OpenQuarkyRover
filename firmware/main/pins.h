#pragma once

/*
 * OpenQuarkyRover — M1 test firmware
 *
 * Pin configuration for the STEMpedia Quarky Intellio (ESP32-S3-WROOM-1-N16R8).
 *
 * Update every value marked TODO once the corresponding GPIO number has
 * been discovered from the stock firmware — via MicroPython REPL
 * introspection (`import intellioConstants; print(vars(intellioConstants))`)
 * or by disassembling the frozen `.mpy` bytecode. See ../../README.md.
 *
 * SAFETY:
 *   - ESP32-S3-WROOM-1 uses GPIO 26..37 internally for flash and octal
 *     PSRAM (N16R8 = 8 MB octal PSRAM). NEVER drive those pins.
 *   - Strapping pins (0, 3, 45, 46) can be used but need care at boot.
 *   - USB native pins (19, 20) are free on this board because the USB-C
 *     connector goes through a serial bridge chip, not the S3's USB.
 *   - Free user I/O: 1, 2, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15,
 *     16, 17, 18, 21, 38, 39, 40, 41, 42, 47, 48 (+ strapping pins).
 */

/* --- Expansion header pins (A1, A2) — CONFIRMED I2C ------------------- */
/* Multimeter continuity between the header pins and the ESP32-S3 pads
 * plus the silkscreen on the Quarky Expansion Board:
 *
 *   A1 = SDA = GPIO 2
 *   A2 = SCL = GPIO 1
 *
 * The header is a *3.3 V I2C bus* to the expansion board's on-board
 * PWM/motor-driver chip (P1..P4 outputs are downstream of that chip,
 * not wired directly to ESP32 GPIOs). The earlier assumption that A1/A2
 * were passive H-bridge PWM inputs — captured in the stock-firmware
 * strings dump "A1 (GPIO2)" / "A2 (GPIO1)" — was wrong; those strings
 * are the pin labels, not motor role assignments.
 *
 * PIN_MOTOR_1 / PIN_MOTOR_2 below are kept only so the existing LEDC
 * motor code compiles. They MUST NOT be used simultaneously with I2C on
 * the same pins — LEDC and I2C both try to drive the pad and behavior
 * is undefined. Once we identify the on-board chip (PCA9685 at 0x40?
 * a DRV88xx variant?) we'll rip the LEDC path and drive motors over I2C
 * instead.
 */
#define PIN_I2C_SDA     2   /* A1 on expansion header */
#define PIN_I2C_SCL     1   /* A2 on expansion header */

/* DEPRECATED — see note above. */
#define PIN_MOTOR_1     PIN_I2C_SDA
#define PIN_MOTOR_2     PIN_I2C_SCL

/* --- On-board RGB LED (WS2812B) --- CONFIRMED ------------------------- */
/* Verified against a running M1 test firmware: `led 255 0 0` at the
 * REPL lights the on-board LED when the driver is bound to this pin.
 */
#define PIN_LED_WS2812  48  /* CONFIRMED */

/* --- On-board servo (S1 header pin) — CONFIRMED ---------------------- */
/* Verified by multimeter continuity between the S1 header pin and the
 * ESP32-S3 module pads: the S1 signal is on GPIO 0.
 *
 * CAUTION: GPIO 0 is the ESP32-S3 boot-mode strapping pin. It must read
 * HIGH at reset for normal boot; if it reads LOW, the ROM enters USB
 * download mode instead. The vendor's PCB relies on an external pull-up
 * (or the servo signal line's high-impedance idle) to satisfy this. If
 * the rover ever fails to boot after this pin is loaded (a servo horn
 * attached that sinks current, etc.), that's why. At runtime the pin is
 * safe to use as an LEDC PWM output.
 */
#define PIN_SERVO_S1    0   /* CONFIRMED (multimeter) */

/* --- Button ADC input — CONFIRMED (live measurement + user mapping) -- */
/* L and R tactile switches share a single ADC via a resistor ladder to
 * VCC. Idle sits near GND (line pulled low). Each single press raises
 * the line into its own band; pressing BOTH sits in the *gap* between
 * the two single bands (network is not a simple parallel-resistor
 * bump — likely diode-decoupled).
 *
 *   idle : 0..150       (0 observed)
 *   R    : 150..1000    (peak ~731  observed)
 *   BOTH : 1000..1350   (~1049..1251 observed)
 *   L    : 1350..1900   (peak ~1498 observed)
 *
 * Classification is threshold-based (no gaps between bands): the first
 * threshold `raw` fits into wins.
 */
#define PIN_BUTTON_ADC          7    /* GPIO 7 = ADC1 channel 6 */

#define BUTTON_ADC_IDLE_MAX     150   /* raw <= 150            → NONE */
#define BUTTON_ADC_R_MAX        1000  /* 150 < raw <= 1000     → R    */
#define BUTTON_ADC_BOTH_MAX     1350  /* 1000 < raw <= 1350    → BOTH */
#define BUTTON_ADC_L_MAX        1900  /* 1350 < raw <= 1900    → L    */
                                       /* raw > 1900            → ?    */

/* --- Servo pulse width bounds (from stock flags.json) ----------------- */
#define SERVO_MIN_US    500
#define SERVO_MAX_US    2500

/* --- Motor PWM config ------------------------------------------------- */
#define MOTOR_PWM_FREQ_HZ   5000
#define MOTOR_PWM_RES_BITS  8       /* 0..255 duty */

/* --- Servo PWM config ------------------------------------------------- */
#define SERVO_PWM_FREQ_HZ   50
#define SERVO_PWM_RES_BITS  14      /* ~1.22 µs resolution per LSB */

/* --- ES8311 audio codec (M3 Task 1) ------------------------------------ */
/*
 * SDA / SCL — CONFIRMED empirically (2026-09-06) by the throwaway
 * `es-scan` REPL command: brute-force bit-bang I²C probe of every
 * candidate (MCLK, SDA, SCL) triple gated on reading product-ID
 * register 0xFD == 0x83. First (and only) hit:
 *
 *     SDA = GPIO 17,  SCL = GPIO 18,  ID = 0x83, VER = 0x11
 *
 * MCLK / BCLK / LRCK / DIN — from ESP-ADF's `esp32_s3_korvo2_v3` board
 * profile (`components/audio_board/esp32_s3_korvo2_v3/board_pins_config.c`
 * upstream). The stock firmware strings dump
 * (quarkyRoverBackup/factory.strings.txt) contains both `ESP32_S3_KORVO_2`
 * and `//home/sheetal01/esp-adf/…/esp32_s3_korvo2_v3/board_pins_config.c`,
 * so STEMpedia's audio stack is built against that exact board driver.
 * The I²C pair matches ADF's canonical (17, 18) — high confidence the
 * I²S pins match too. Verified empirically in M3 Task 4 (voice-record
 * ear-check); revisit if audio is silent or garbled.
 *
 * The `es-scan` hit MCLK=GPIO 4 is a false positive: the ES8311 answers
 * I²C off its internal RC oscillator without an external MCLK, so the
 * first driven candidate in the scan appears to "cause" the ACK. Only
 * the audio path actually needs MCLK, and for that we use the ADF value.
 *
 * ES8311_MCLK_SOURCE = 0 in the ADF profile: MCLK is generated by the
 * ESP32 (i2s_std_config_t clk_cfg.mclk_multiple), not derived from BCLK.
 *
 * DOUT (speaker path) is intentionally omitted — speaker output is out
 * of M3 scope; add it here alongside a speaker driver task if M4 needs it.
 */
#define ES8311_I2C_SDA      17    /* CONFIRMED (es-scan)              */
#define ES8311_I2C_SCL      18    /* CONFIRMED (es-scan)              */
#define ES8311_I2S_MCLK     16    /* ADF Korvo-2 v3 board profile     */
#define ES8311_I2S_BCLK     9     /* ADF Korvo-2 v3 board profile     */
#define ES8311_I2S_LRCK     45    /* ADF Korvo-2 v3 board profile — WS */
#define ES8311_I2S_DIN      10    /* ADF Korvo-2 v3 board profile     */

#define ES8311_I2C_ADDR     0x18  /* 7-bit; datasheet §Register 0xFD returns 0x83 */

/* --- Digital I²S MEMS microphone (M3 Task 4) ------------------------- */
/*
 * The on-board mic is an omnidirectional digital MEMS microphone
 * (INMP441-family) that talks I²S DIRECTLY to the ESP32-S3 over its own
 * three-wire bus — completely independent of the ES8311 codec above,
 * which is on the DAC / speaker side. The mic needs no MCLK, no analog
 * routing, and no codec init: just I²S RX in master mode.
 *
 * Slot format on the wire: 24-bit PCM in a 32-bit slot, MSB-first,
 * one channel (typical INMP441). Our i2s_std config reads that as
 * 16-bit mono by asking the driver to sample the top 16 bits of a
 * 32-bit slot — matches ESP-SR's 16 kHz mono s16le expectation
 * without a resample.
 *
 * Provenance: hardware notes from the vendor (Quarky Intellio spec).
 */
#define MIC_I2S_SCK         40    /* BCLK from ESP32 (I²S master) */
#define MIC_I2S_WS          41    /* WS / LRCLK from ESP32 */
#define MIC_I2S_SD          42    /* SD / DIN — mic drives this */
