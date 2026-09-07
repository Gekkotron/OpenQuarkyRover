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

/* --- OV5640 camera (DVP parallel) — CONFIRMED via live-capture -------
 *
 * Live-verified against the stock firmware's active GPIO matrix (see
 * README "How we resolved the audio pin map" — same mem32 technique
 * applied to CAM_* signal indices 133..152 and OUT_SEL for CAM_CLK).
 *
 *   XCLK   ESP32 → sensor        GPIO 14   (OUT_SEL[14] = sig 149)
 *   PCLK   sensor → ESP32        GPIO 11
 *   VSYNC  sensor → ESP32        GPIO 38
 *   HREF   sensor → ESP32        GPIO 16   (HSYNC is unused on this board)
 *   D0..D7 sensor → ESP32        9,19,8,20,10,12,13,21
 *   SCCB   shared I²C w/ ES8311  SDA=17  SCL=18
 *   PWDN, RESET                  not wired on this board
 *
 * The concrete #defines live in camera.c so the esp32-camera driver
 * config stays adjacent to its usage. This block is for cross-reference
 * with the pin-map table in the README. */

/* --- ES8311 audio codec (M3 Task 1) ------------------------------------ */
/*
 * ALL PIN VALUES CONFIRMED by live capture of the stock firmware's GPIO
 * matrix registers on 2026-09-07 (see history: reflashed stock, drove
 * intellioAudio.test_timed_record(), read GPIO_FUNCn_IN_SEL_CFG and
 * GPIO_FUNCn_OUT_SEL_CFG on the running codec path). Signal indices
 * from esp32s3/soc/gpio_sig_map.h:
 *
 *   IN_SEL  signal 25 (I2S0I_SD_IN) <- GPIO 3    -> mic ADC DIN
 *   IN_SEL  signal 89 (I2CEXT0_SCL) <- GPIO 18   -> codec I²C SCL
 *   IN_SEL  signal 90 (I2CEXT0_SDA) <- GPIO 17   -> codec I²C SDA
 *   OUT_SEL GPIO 46 -> signal 22 (I2S0O_BCK_OUT) -> BCLK
 *   OUT_SEL GPIO 45 -> signal 23 (I2S0_MCLK_OUT) -> MCLK
 *   OUT_SEL GPIO 39 -> signal 24 (I2S0O_WS_OUT)  -> WS / LRCK
 *   OUT_SEL GPIO 15 -> signal 25 (I2S0O_SD_OUT)  -> DOUT to codec DAC
 *
 * The earlier "ADF Korvo-2 v3 board profile" values (MCLK=16 BCLK=9
 * LRCK=45 DIN=10) were wrong for every pin except the two I²C ones;
 * the Quarky's audio wiring is board-specific and not a Korvo-2 v3 clone.
 * That mistake is why voice-record on the M3 branch produced bit-exact
 * zero samples: the ESP32 was clocking BCLK/LRCK on pins that aren't
 * even wired to the codec's I²S port, and reading DIN from a pin that
 * carries no data.
 *
 * IMPORTANT — GPIO 3 is a strapping pin (must read HIGH at reset for
 * normal boot; LOW forces USB download mode). The stock design gets
 * away with using it as I²S DIN because the ES8311 keeps SDPOUT in
 * high-impedance until its ADC is fully powered up, so the on-chip
 * pull-up carries the strap at boot. Do NOT drive GPIO 3 as an output
 * before the codec is initialised, and never leave it floating LOW
 * across a hard reset.
 *
 * ES8311_MCLK_SOURCE = 0 in the ADF profile: MCLK is generated by the
 * ESP32 (i2s_std_config_t clk_cfg.mclk_multiple), not derived from BCLK.
 */
#define ES8311_I2C_SDA      17    /* live-verified (in_sel sig 90)     */
#define ES8311_I2C_SCL      18    /* live-verified (in_sel sig 89)     */
#define ES8311_I2S_MCLK     45    /* live-verified (out_sel sig 23)    */
#define ES8311_I2S_BCLK     46    /* live-verified (out_sel sig 22)    */
#define ES8311_I2S_LRCK     39    /* live-verified (out_sel sig 24)    */
#define ES8311_I2S_DIN      3     /* live-verified (in_sel sig 25) — STRAPPING PIN */
#define ES8311_I2S_DOUT     15    /* live-verified (out_sel sig 25) — for M4 speaker */

/* --- Speaker amplifier enable ----------------------------------------- */
/* Live-verified 2026-09-07: stock firmware at idle drives GPIO 47 HIGH
 * with the pin enabled as an output (GPIO_OUT1_REG bit 15 = 1,
 * GPIO_ENABLE1_REG bit 15 = 1). Held HIGH continuously; not toggled per
 * play/pause. Active-HIGH — pull LOW to mute the class-D amp behind
 * the codec's DOUT line. */
#define PIN_PA_EN           47

#define ES8311_I2C_ADDR     0x18  /* 7-bit; datasheet §Register 0xFD returns 0x83 */

/* --- Legacy "separate digital MEMS mic on 40/41/42" defs (falsified) --
 *
 * The M3 branch initially assumed the mic was an INMP441-family digital
 * MEMS on its own I²S bus at pins 40/41/42. This turned out to be wrong:
 * the stock firmware strings dump proved the mic is analog and goes
 * through the ES8311 codec (MIC_GAIN_0DB..42DB matches the codec's PGA
 * ladder exactly; ESP-ADF es8311 driver referenced directly). The real
 * mic I²S bus is the ES8311 one above: MCLK=16 BCLK=9 LRCK=45 DIN=10.
 *
 * The constants below are kept only so the legacy diagnostic commands
 * that reference them (voice-scan, mic-perm, mic-enable-scan) still
 * compile. They should be removed together with those commands once
 * the ES8311 mic path is verified working. */
#define MIC_I2S_SCK         40    /* legacy, unused by audio_capture   */
#define MIC_I2S_WS          41    /* legacy, unused by audio_capture   */
#define MIC_I2S_SD          42    /* legacy, unused by audio_capture   */
