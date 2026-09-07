/*
 * OpenQuarkyRover — M1 smoke-test firmware
 *
 * Interactive REPL over the USB-serial console. Commands drive the
 * on-board WS2812B RGB LED, the S1 servo, and the two DC motors on the
 * Mini Expansion Board. Type `help` at the `quarky>` prompt.
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_console.h"
#include "esp_rom_sys.h"    /* esp_rom_delay_us for TLC59108 oscillator wait */
#include "esp_adc/adc_oneshot.h"  /* adc-hunt: analog-read every ADC-capable pin */
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/i2s_pdm.h"    /* PDM RX probe for the on-board digital MEMS mic */
#include "led_strip.h"
#include <math.h>

#include "pins.h"
#include "wifi_softap.h"
#include "http_control.h"
#include "camera.h"

static const char *TAG = "M1";

/* --- state ------------------------------------------------------------ */

static led_strip_handle_t       s_led           = NULL;
static int                      s_led_pin       = PIN_LED_WS2812;
static int                      s_servo_pin     = PIN_SERVO_S1;

/* Persistent I2C bus + TLC59108 handle. Bus is opened once at boot on
 * GPIO 2 (SDA) / GPIO 1 (SCL) and stays up so tlc-* commands can talk
 * to the expansion board without repeatedly bringing it up/down. */
static i2c_master_bus_handle_t  s_i2c_bus       = NULL;
static i2c_master_dev_handle_t  s_tlc           = NULL;
static bool                     s_tlc_ready     = false;

#define TLC59108_ADDR_DEFAULT 0x40
static uint8_t s_tlc_addr = TLC59108_ADDR_DEFAULT;
/* Kept as a name for the family default so log lines and probes still
 * mention 0x40 alongside the currently-selected address. */
#define TLC59108_ADDR        s_tlc_addr
#define TLC59108_MODE1       0x00
#define TLC59108_MODE2       0x01
#define TLC59108_PWM0        0x02   /* PWM0..PWM7 = 0x02..0x09 (8-bit duty) */
#define TLC59108_GRPPWM      0x0A
#define TLC59108_GRPFREQ     0x0B
#define TLC59108_LEDOUT0     0x0C   /* LDR0..LDR3 (2 bits each) */
#define TLC59108_LEDOUT1     0x0D   /* LDR4..LDR7 (2 bits each) */

#define TLC59108_MODE1_OSC   (1 << 4)  /* 1 = oscillator OFF (reset default) */

/* LDRn field values (2 bits per channel in LEDOUT0/1). */
#define TLC59108_LDR_OFF     0x0
#define TLC59108_LDR_ON      0x1
#define TLC59108_LDR_PWM     0x2
#define TLC59108_LDR_GRP     0x3

/* Control-byte auto-increment prefixes (top 3 bits of the byte written
 * immediately after the slave address). OR with the starting register. */
#define TLC59108_AI_NONE     0x00
#define TLC59108_AI_ALL      0x80
#define TLC59108_AI_BRIGHT   0xA0   /* PWM0..PWM7 only */
#define TLC59108_AI_GLOBAL   0xC0   /* GRPPWM..GRPFREQ */
#define TLC59108_AI_BR_GL    0xE0   /* PWM0..GRPFREQ */

/* Candidate GPIOs for pin-sweep discovery.
 * Safe to pulse briefly on ESP32-S3-WROOM-1:
 *   - excludes 26..37 (QSPI flash + octal PSRAM — driving these hangs the chip)
 *   - excludes 43, 44 (UART0 — would kill our console)
 *   - excludes 0 (boot-mode strapping — toggles the boot path)
 * Strapping pins 3, 45, 46 ARE included: they only latch state at reset,
 * so writing to them at runtime is safe. STEMpedia may have used one of
 * these for the on-board servo signal (S1), which is why the initial
 * 27-pin sweep found nothing.
 */
static const int CANDIDATE_PINS[] = {
     1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15,
    16, 17, 18, 19, 20, 21, 38, 39, 40, 41, 42, 45, 46, 47, 48
};
static const size_t NUM_CANDIDATE_PINS = sizeof(CANDIDATE_PINS) / sizeof(CANDIDATE_PINS[0]);

#define SWEEP_HOLD_MS  2000

/* --- LED (WS2812B via RMT) ------------------------------------------- */

static void led_init(int gpio)
{
    if (s_led) {
        led_strip_del(s_led);
        s_led = NULL;
    }
    led_strip_config_t strip_cfg = {
        .strip_gpio_num  = gpio,
        .max_leds        = 1,
        .led_pixel_format = LED_PIXEL_FORMAT_GRB,
        .led_model       = LED_MODEL_WS2812,
        .flags.invert_out = false,
    };
    led_strip_rmt_config_t rmt_cfg = {
        .clk_src        = RMT_CLK_SRC_DEFAULT,
        .resolution_hz  = 10 * 1000 * 1000,
        .flags.with_dma = false,
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_cfg, &rmt_cfg, &s_led));
    led_strip_clear(s_led);
    s_led_pin = gpio;
}

void led_set(uint8_t r, uint8_t g, uint8_t b)
{
    if (!s_led) return;
    led_strip_set_pixel(s_led, 0, r, g, b);
    led_strip_refresh(s_led);
}

/* --- I2C bus + PCA9685 driver (expansion board is I2C-controlled) --- */

/*
 * The Quarky Expansion Board exposes A1/A2 as SDA/SCL (confirmed by
 * multimeter + silkscreen) and hangs a PCA9685 at 0x40 on that bus.
 * P1..P4 outputs and both motor drivers are downstream of the PCA9685,
 * so we drive them by writing 12-bit ON/OFF counts to LEDx_ON/OFF
 * registers rather than by wiggling ESP32 GPIOs directly.
 *
 * pin binding: PIN_I2C_SDA=GPIO2, PIN_I2C_SCL=GPIO1 (see pins.h).
 * external pull-ups: assumed to live on the expansion board; internal
 * pull-ups are enabled as a fallback for the bare-Intellio case.
 */

/* Currently-configured I2C clock, in Hz. Applied at boot and any time
 * `tlc-speed` reopens the bus + device. */
static int s_i2c_hz = 100000;

static esp_err_t i2c_bus_open(void)
{
    if (s_i2c_bus) return ESP_OK;
    i2c_master_bus_config_t bus_config = {
        .i2c_port                     = I2C_NUM_0,
        .sda_io_num                   = PIN_I2C_SDA,
        .scl_io_num                   = PIN_I2C_SCL,
        .clk_source                   = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt            = 7,
        .flags.enable_internal_pullup = true,
    };
    return i2c_new_master_bus(&bus_config, &s_i2c_bus);
}

/* Tear down the persistent device + bus and rebuild them, so the caller
 * can change the SCL clock (per-device config). Used by tlc-speed. */
static esp_err_t i2c_bus_reopen(int hz)
{
    if (s_tlc) {
        i2c_master_bus_rm_device(s_tlc);
        s_tlc = NULL;
        s_tlc_ready = false;
    }
    if (s_i2c_bus) {
        i2c_del_master_bus(s_i2c_bus);
        s_i2c_bus = NULL;
    }
    esp_err_t err = i2c_bus_open();
    if (err != ESP_OK) return err;

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = TLC59108_ADDR,
        .scl_speed_hz    = (uint32_t)hz,
    };
    err = i2c_master_bus_add_device(s_i2c_bus, &dev_cfg, &s_tlc);
    if (err != ESP_OK) return err;
    s_i2c_hz = hz;
    return ESP_OK;
}

/* The scan / hunt / selftest commands all need to create a fresh master
 * bus on I2C_NUM_0 (typically on a *different* SDA/SCL pair). The
 * persistent bus created at boot already owns port 0, so every such
 * i2c_new_master_bus() call would otherwise fail with ESP_ERR_INVALID_STATE
 * and get silently skipped — which historically produced misleading
 * "no device on N pairs" reports. Wrap the scan body with a pause/resume
 * of the persistent bus so port 0 is temporarily available. */
static bool i2c_bus_pause_persistent(void)
{
    if (!s_i2c_bus) return false;
    if (s_tlc) {
        i2c_master_bus_rm_device(s_tlc);
        s_tlc = NULL;
        s_tlc_ready = false;
    }
    i2c_del_master_bus(s_i2c_bus);
    s_i2c_bus = NULL;
    return true;
}

static void i2c_bus_resume_persistent(bool was_active)
{
    if (!was_active) return;
    if (i2c_bus_open() != ESP_OK) return;
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = TLC59108_ADDR,
        .scl_speed_hz    = (uint32_t)s_i2c_hz,
    };
    /* s_tlc_ready stays false — caller runs tlc-init to re-configure. */
    i2c_master_bus_add_device(s_i2c_bus, &dev_cfg, &s_tlc);
}

static esp_err_t tlc_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };   /* AI=0 → single-register write */
    return i2c_master_transmit(s_tlc, buf, 2, pdMS_TO_TICKS(50));
}

static esp_err_t tlc_read_reg(uint8_t reg, uint8_t *val)
{
    return i2c_master_transmit_receive(s_tlc, &reg, 1, val, 1, pdMS_TO_TICKS(50));
}

/* Init sequence per datasheet §7.4:
 *   1. Write MODE1 = 0x00 (clear OSC bit → oscillator ON, no sleep,
 *      no all-call, no sub-addresses).
 *   2. Wait 500 μs for the oscillator to stabilise before touching any
 *      PWM/GRPPWM/GRPFREQ register.
 *   3. Write MODE2 = 0x00 (dim mode, change-on-STOP; there is no OUTDRV
 *      bit — outputs are always constant-current open-drain sinks).
 *   4. Set LEDOUT0/1 = 0x00 to park every channel in the OFF state
 *      (sink disabled → line pulled HIGH by the expansion board's
 *      external pull-ups → both DRV8833 inputs HIGH per motor = BRAKE).
 *   5. Zero every PWM0..PWM7 register (belt-and-braces; while LDRn=OFF
 *      the PWM value is ignored anyway).
 *
 * PWM frequency is FIXED at 97 kHz — nothing to program.
 */
static esp_err_t tlc59108_init(void)
{
    esp_err_t err;

    /* Recovery pulse: sends up to 9 SCL clocks with SDA released to
     * unstick a slave that latched an aborted transaction across a
     * previous reset. Cheap and safe if the bus is already idle. */
    if (s_i2c_bus) i2c_master_bus_reset(s_i2c_bus);

    /* Probe first so a chip-not-present or bus-stuck-low condition
     * returns a diagnosable error instead of the register-write's
     * opaque ESP_ERR_INVALID_STATE. Silence the driver's per-probe
     * ERROR log so the console stays readable when the bus is dead. */
    esp_log_level_set("i2c.master", ESP_LOG_NONE);
    err = i2c_master_probe(s_i2c_bus, TLC59108_ADDR, pdMS_TO_TICKS(20));
    esp_log_level_set("i2c.master", ESP_LOG_ERROR);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "TLC59108 did not ACK on probe (0x%02x): %s — check the bus, "
                      "pull-ups, and battery power to the expansion board",
                 TLC59108_ADDR, esp_err_to_name(err));
        return err;
    }

    if ((err = tlc_write_reg(TLC59108_MODE1, 0x00)) != ESP_OK) return err;
    esp_rom_delay_us(500);

    if ((err = tlc_write_reg(TLC59108_MODE2, 0x00))   != ESP_OK) return err;
    if ((err = tlc_write_reg(TLC59108_LEDOUT0, 0x00)) != ESP_OK) return err;
    if ((err = tlc_write_reg(TLC59108_LEDOUT1, 0x00)) != ESP_OK) return err;

    /* PWM0..PWM7 = 0 via brightness auto-increment. */
    uint8_t pwm_zero[9] = { TLC59108_AI_BRIGHT | TLC59108_PWM0,
                            0, 0, 0, 0, 0, 0, 0, 0 };
    if ((err = i2c_master_transmit(s_tlc, pwm_zero, sizeof(pwm_zero),
                                   pdMS_TO_TICKS(50))) != ESP_OK) return err;

    s_tlc_ready = true;
    return ESP_OK;
}

/* Sets LDRn=PWM (10b) for one channel in LEDOUT0/1 without disturbing
 * the other three channels sharing that register. Done via read-modify-
 * write because we don't cache the value. */
static esp_err_t tlc_ensure_pwm_mode(int ch)
{
    uint8_t reg   = (ch < 4) ? TLC59108_LEDOUT0 : TLC59108_LEDOUT1;
    int     shift = 2 * (ch % 4);

    uint8_t cur;
    esp_err_t err = tlc_read_reg(reg, &cur);
    if (err != ESP_OK) return err;

    uint8_t want = (cur & ~(0x3 << shift)) | (TLC59108_LDR_PWM << shift);
    if (want == cur) return ESP_OK;
    return tlc_write_reg(reg, want);
}

/* Writes an 8-bit sink-duty to one channel and puts it in PWM mode.
 *
 * NOTE ON POLARITY: on TLC59108 the "duty" is the fraction of time the
 * open-drain sink is ON — i.e. the fraction of time the output pin is
 * pulling LOW. The Quarky expansion board is expected to have external
 * pull-ups on each output going to the DRV8833, so:
 *   pct=0   → sink always off → line HIGH  → DRV8833 input HIGH
 *   pct=100 → sink always on  → line LOW   → DRV8833 input LOW
 * For DRV8833 speed control we usually want inverted duty at the H-
 * bridge input; that mapping (positive-going motor duty) will live in a
 * future `motor` command once pca-sweep tells us which channels are
 * AIN1/AIN2/BIN1/BIN2.
 */
static esp_err_t tlc59108_set_channel(int ch, uint8_t pwm_val)
{
    if (ch < 0 || ch > 7) return ESP_ERR_INVALID_ARG;
    esp_err_t err = tlc_ensure_pwm_mode(ch);
    if (err != ESP_OK) return err;
    return tlc_write_reg(TLC59108_PWM0 + ch, pwm_val);
}

static esp_err_t tlc59108_set_pct(int ch, int pct)
{
    if (pct < 0)   pct = 0;
    if (pct > 100) pct = 100;
    return tlc59108_set_channel(ch, (uint8_t)(pct * 255 / 100));
}

static esp_err_t tlc59108_all_off(void)
{
    esp_err_t err = tlc_write_reg(TLC59108_LEDOUT0, 0x00);
    if (err != ESP_OK) return err;
    return tlc_write_reg(TLC59108_LEDOUT1, 0x00);
}

/* --- motors (LEGACY: LEDC PWM on A1/A2 — do NOT use, corrupts I2C) --- */
/* Kept only so the existing sweep/motor commands still compile. The
 * `motor` command below is now a stub that refuses to run.            */

static void motors_init(void)
{
    ledc_timer_config_t timer = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .duty_resolution = MOTOR_PWM_RES_BITS,
        .timer_num       = LEDC_TIMER_0,
        .freq_hz         = MOTOR_PWM_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    ledc_channel_config_t ch1 = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = LEDC_CHANNEL_0,
        .timer_sel  = LEDC_TIMER_0,
        .intr_type  = LEDC_INTR_DISABLE,
        .gpio_num   = PIN_MOTOR_1,
        .duty       = 0,
        .hpoint     = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ch1));

    ledc_channel_config_t ch2 = ch1;
    ch2.channel  = LEDC_CHANNEL_1;
    ch2.gpio_num = PIN_MOTOR_2;
    ESP_ERROR_CHECK(ledc_channel_config(&ch2));
}

static void motor_set(int m1_pct, int m2_pct)
{
    if (m1_pct < 0)   m1_pct = 0;
    if (m1_pct > 100) m1_pct = 100;
    if (m2_pct < 0)   m2_pct = 0;
    if (m2_pct > 100) m2_pct = 100;

    const uint32_t max_duty = (1U << MOTOR_PWM_RES_BITS) - 1;
    const uint32_t d1 = (uint32_t)m1_pct * max_duty / 100;
    const uint32_t d2 = (uint32_t)m2_pct * max_duty / 100;

    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, d1);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, d2);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
}

/* --- servo (LEDC PWM 50 Hz) ------------------------------------------ */

static void servo_init(int gpio)
{
    ledc_timer_config_t timer = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .duty_resolution = SERVO_PWM_RES_BITS,
        .timer_num       = LEDC_TIMER_1,
        .freq_hz         = SERVO_PWM_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    ledc_channel_config_t ch = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = LEDC_CHANNEL_2,
        .timer_sel  = LEDC_TIMER_1,
        .intr_type  = LEDC_INTR_DISABLE,
        .gpio_num   = gpio,
        .duty       = 0,
        .hpoint     = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ch));
    s_servo_pin = gpio;
}

void servo_set_deg(int deg)
{
    if (deg < 0)   deg = 0;
    if (deg > 180) deg = 180;

    const int      pulse_us = SERVO_MIN_US + (SERVO_MAX_US - SERVO_MIN_US) * deg / 180;
    const uint32_t max_duty = (1U << SERVO_PWM_RES_BITS) - 1;
    const uint32_t duty     = (uint32_t)pulse_us * max_duty / 20000;  /* 50 Hz -> 20000 µs period */

    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
}

/* --- CLI commands ---------------------------------------------------- */

static int cmd_led(int argc, char **argv)
{
    if (argc < 4) { printf("usage: led <r 0-255> <g 0-255> <b 0-255>\n"); return 1; }
    int r = atoi(argv[1]), g = atoi(argv[2]), b = atoi(argv[3]);
    led_set((uint8_t)r, (uint8_t)g, (uint8_t)b);
    printf("led (GPIO %d) = (%d,%d,%d)\n", s_led_pin, r, g, b);
    return 0;
}

static int cmd_led_off(int argc, char **argv)
{
    (void)argc; (void)argv;
    led_set(0, 0, 0);
    printf("led off\n");
    return 0;
}

static int cmd_led_pin(int argc, char **argv)
{
    if (argc < 2) { printf("usage: led-pin <gpio>   (current: %d)\n", s_led_pin); return 1; }
    int p = atoi(argv[1]);
    led_init(p);
    printf("led moved to GPIO %d\n", p);
    return 0;
}

/* Forward decls: motor drive is implemented via bit-bang further down.
 * The ESP-IDF i2c_master peripheral doesn't ACK on this board (see the
 * cmd_bb_scan comment), so we drive the TLC59108 via bit-bang instead. */
static bool bb_motor_set(int m1_signed, int m2_signed);
static bool s_bb_ready = false;

static int cmd_motor(int argc, char **argv)
{
    if (argc < 3) {
        printf("usage: motor <L -100..100> <R -100..100>\n"
               "  L drives M1 (channels 2/3), R drives M2 (channels 0/1).\n"
               "  Negative = reverse. 0 = brake. Run `bb-tlc-init` once at boot.\n");
        return 1;
    }
    if (!s_bb_ready) { printf("run `bb-tlc-init` first\n"); return 1; }
    int L = atoi(argv[1]);
    int R = atoi(argv[2]);
    if (!bb_motor_set(L, R)) {
        printf("motor: NACK on TLC59108 write — try `bb-tlc-init` again\n");
        return 1;
    }
    printf("motor: L=%d%% R=%d%%\n", L, R);
    return 0;
}

static int cmd_motor_stop(int argc, char **argv)
{
    (void)argc; (void)argv;
    if (!s_bb_ready) {
        printf("TLC59108 not initialised — run `bb-tlc-init` first\n");
        return 1;
    }
    if (!bb_motor_set(0, 0)) {
        printf("stop: NACK on TLC59108 write\n");
        return 1;
    }
    printf("stopped (both motors in brake)\n");
    return 0;
}

static int cmd_servo(int argc, char **argv)
{
    if (argc < 2) { printf("usage: servo <deg 0-180>\n"); return 1; }
    int d = atoi(argv[1]);
    servo_set_deg(d);
    printf("servo (GPIO %d) = %d deg\n", s_servo_pin, d);
    return 0;
}

static int cmd_servo_pin(int argc, char **argv)
{
    if (argc < 2) { printf("usage: servo-pin <gpio>   (current: %d)\n", s_servo_pin); return 1; }
    int p = atoi(argv[1]);
    servo_init(p);
    printf("servo moved to GPIO %d\n", p);
    return 0;
}

static int cmd_selftest(int argc, char **argv)
{
    (void)argc; (void)argv;

    printf("--- self-test start ---\n");

    printf("LED red\n");   led_set(64, 0,  0);  vTaskDelay(pdMS_TO_TICKS(500));
    printf("LED green\n"); led_set(0,  64, 0);  vTaskDelay(pdMS_TO_TICKS(500));
    printf("LED blue\n");  led_set(0,  0,  64); vTaskDelay(pdMS_TO_TICKS(500));
    led_set(0, 0, 0);

    printf("servo sweep 0 -> 180 -> 0\n");
    for (int d = 0;   d <= 180; d += 20) { servo_set_deg(d); vTaskDelay(pdMS_TO_TICKS(120)); }
    for (int d = 180; d >= 0;   d -= 20) { servo_set_deg(d); vTaskDelay(pdMS_TO_TICKS(120)); }

    printf("motor test skipped — use `pca-sweep` once M1/M2 channels are known\n");

    printf("--- self-test done ---\n");
    return 0;
}

static int cmd_pins(int argc, char **argv)
{
    (void)argc; (void)argv;
    printf("current pin assignments:\n");
    printf("  motor 1 : GPIO %-3d  (Mini expansion A1) [CONFIRMED]\n", PIN_MOTOR_1);
    printf("  motor 2 : GPIO %-3d  (Mini expansion A2) [CONFIRMED]\n", PIN_MOTOR_2);
    printf("  led     : GPIO %-3d  (WS2812B)           [%s]\n",
           s_led_pin,   s_led_pin   == PIN_LED_WS2812 ? "PLACEHOLDER — verify" : "runtime-set");
    printf("  servo   : GPIO %-3d  (S1 header)         [%s]\n",
           s_servo_pin, s_servo_pin == PIN_SERVO_S1   ? "PLACEHOLDER — verify" : "runtime-set");
    return 0;
}

/* --- TLC59108 REPL commands ----------------------------------------- */

/* Probes 0x40 on the persistent I2C bus and reports what happens.
 * Bypasses the device handle entirely so we can tell whether the chip
 * is even ACKing, independent of any per-device configuration. */
static int cmd_tlc_diag(int argc, char **argv)
{
    (void)argc; (void)argv;
    if (!s_i2c_bus) { printf("no persistent I2C bus (open failed at boot?)\n"); return 1; }

    printf("--- TLC59108 diag (bus SDA=%d SCL=%d) ---\n", PIN_I2C_SDA, PIN_I2C_SCL);

    /* Send SCL recovery clocks to release any stuck slave. */
    esp_err_t reset_err = i2c_master_bus_reset(s_i2c_bus);
    printf("  bus reset (9 SCL pulses) : %s\n", esp_err_to_name(reset_err));

    /* Probe with 3-consecutive-ACK stability so a lone bus glitch can't
     * masquerade as a device. */
    esp_log_level_set("i2c.master", ESP_LOG_NONE);
    esp_err_t p_target  = i2c_master_probe(s_i2c_bus, TLC59108_ADDR, pdMS_TO_TICKS(20));
    esp_err_t p_allcall = i2c_master_probe(s_i2c_bus, 0x48, pdMS_TO_TICKS(20));
    printf("  probe 0x%02x (target)      : %s\n", TLC59108_ADDR, esp_err_to_name(p_target));
    printf("  probe 0x48 (all-call def) : %s\n", esp_err_to_name(p_allcall));

    printf("  scanning 0x08..0x77 on this bus (3-ACK stable):\n");
    int found = 0;
    for (uint8_t addr = 0x08; addr < 0x78; ++addr) {
        bool stable = true;
        for (int i = 0; i < 3; ++i) {
            if (i2c_master_probe(s_i2c_bus, addr, pdMS_TO_TICKS(5)) != ESP_OK) {
                stable = false;
                break;
            }
        }
        if (stable) {
            printf("    device @ 0x%02x\n", addr);
            found++;
        }
    }
    esp_log_level_set("i2c.master", ESP_LOG_ERROR);

    if (found == 0) {
        printf("--- no device on this bus. Check: chip powered? board seated?\n");
        printf("    external pull-ups on SDA/SCL? DRV8833 nSLEEP asserted?\n");
    } else {
        printf("--- %d device(s) found. If 0x%02x is missing, the TLC59108 address\n",
               found, TLC59108_ADDR);
        printf("    pins A0..A3 may be strapped to a non-default value.\n");
    }
    return 0;
}

/* Change which 7-bit address the driver talks to. Removes and re-adds
 * the persistent device handle. Also probes the new address so the user
 * gets immediate feedback. */
static int cmd_tlc_addr(int argc, char **argv)
{
    if (argc < 2) { printf("usage: tlc-addr <hex>   (current: 0x%02x)\n", s_tlc_addr); return 1; }
    int addr = (int)strtol(argv[1], NULL, 0);
    if (addr < 0x08 || addr > 0x77) {
        printf("addr must be in the 7-bit range 0x08..0x77\n");
        return 1;
    }
    s_tlc_addr = (uint8_t)addr;
    esp_err_t err = i2c_bus_reopen(s_i2c_hz);
    if (err != ESP_OK) {
        printf("bus reopen at 0x%02x failed: %s\n", s_tlc_addr, esp_err_to_name(err));
        return 1;
    }
    esp_log_level_set("i2c.master", ESP_LOG_NONE);
    esp_err_t p = i2c_master_probe(s_i2c_bus, s_tlc_addr, pdMS_TO_TICKS(20));
    esp_log_level_set("i2c.master", ESP_LOG_ERROR);
    printf("device address now 0x%02x — probe: %s\n", s_tlc_addr, esp_err_to_name(p));
    return 0;
}

static int cmd_tlc_speed(int argc, char **argv)
{
    if (argc < 2) { printf("usage: tlc-speed <hz>   (current: %d)\n", s_i2c_hz); return 1; }
    int hz = atoi(argv[1]);
    if (hz < 1000 || hz > 400000) {
        printf("hz must be between 1000 and 400000\n");
        return 1;
    }
    esp_err_t err = i2c_bus_reopen(hz);
    if (err != ESP_OK) {
        printf("bus reopen at %d Hz failed: %s\n", hz, esp_err_to_name(err));
        return 1;
    }
    printf("I2C bus + TLC59108 device re-opened at %d Hz\n", hz);
    return 0;
}

static int cmd_tlc_init(int argc, char **argv)
{
    (void)argc; (void)argv;
    if (!s_tlc) { printf("no TLC59108 device handle — I2C bus down?\n"); return 1; }
    esp_err_t err = tlc59108_init();
    if (err != ESP_OK) {
        printf("tlc-init failed: %s\n", esp_err_to_name(err));
        s_tlc_ready = false;
        return 1;
    }
    printf("TLC59108 initialised (97 kHz fixed), all channels parked off\n");
    return 0;
}

static int cmd_tlc_set(int argc, char **argv)
{
    if (argc < 3) { printf("usage: tlc-set <ch 0-7> <sink-pct 0-100>\n"
                           "       (sink-pct is the fraction of time the open-drain sink is ON,\n"
                           "        i.e. the fraction of time the output is pulled LOW —\n"
                           "        polarity is INVERTED relative to the DRV8833's input logic)\n"); return 1; }
    if (!s_tlc_ready) { printf("run `tlc-init` first\n"); return 1; }
    int ch  = atoi(argv[1]);
    int pct = atoi(argv[2]);
    esp_err_t err = tlc59108_set_pct(ch, pct);
    if (err != ESP_OK) {
        printf("tlc-set ch=%d pct=%d failed: %s\n", ch, pct, esp_err_to_name(err));
        return 1;
    }
    printf("TLC59108 ch %d sink duty = %d%%\n", ch, pct);
    return 0;
}

/* Cycles each TLC59108 channel through 50 %% sink-duty for a short
 * hold. Since sink-ON pulls DRV8833 inputs LOW (with the board's
 * external pull-ups holding them HIGH otherwise), a 50 %% sink duty
 * gives a DRV8833 input at 50 %% HIGH → motor spins at ~50 %% in one
 * direction when the paired input is at 0 %%. The winning channel is
 * the one printed just before a motor twitches or spins. */
static int cmd_tlc_sweep(int argc, char **argv)
{
    int hold_ms = 1500;
    if (argc >= 2) hold_ms = atoi(argv[1]);
    if (hold_ms < 200)  hold_ms = 200;
    if (hold_ms > 5000) hold_ms = 5000;

    if (!s_tlc_ready) { printf("run `tlc-init` first\n"); return 1; }

    printf("--- TLC59108 sweep: 8 channels, %d ms per channel @ 50%% sink duty ---\n", hold_ms);
    for (int ch = 0; ch < 8; ++ch) {
        printf(">>> ch %d\n", ch);
        fflush(stdout);
        tlc59108_set_pct(ch, 50);
        vTaskDelay(pdMS_TO_TICKS(hold_ms));
        tlc59108_set_pct(ch, 0);
    }
    tlc59108_all_off();
    printf("--- sweep done ---\n");
    return 0;
}

/* --- adc-hunt: identify a resistor-ladder button input --------------- */

/*
 * ESP32-S3 ADC coverage: ADC1 = GPIO 1..10 (channels 0..9), ADC2 =
 * GPIO 11..20 (channels 0..9). Configures every ADC-capable pin (minus
 * pins we know are already in use: SDA=2, SCL=1, servo=0, WS2812=48)
 * for 12-bit / 12 dB (0..~3.1 V full scale), captures an idle baseline,
 * then watches for changes larger than `threshold` counts.
 *
 * A resistor-ladder button design typically shows:
 *   idle       : reading near ADC full-scale (line pulled high)
 *   button A   : reading in a low band (e.g. 0..1000)
 *   button B   : reading in a middle band (e.g. 1200..2200)
 * The GPIO whose value swings when a button is pressed is the shared
 * button-input ADC pin. The reported before/after values tell us the
 * voltage bands the vendor picked, so we can hard-code thresholds.
 */
static int cmd_adc_hunt(int argc, char **argv)
{
    int seconds   = 30;
    int threshold = 200;
    if (argc >= 2) seconds   = atoi(argv[1]);
    if (argc >= 3) threshold = atoi(argv[2]);
    if (seconds   < 3)    seconds   = 3;
    if (seconds   > 120)  seconds   = 120;
    if (threshold < 30)   threshold = 30;
    if (threshold > 4095) threshold = 4095;

    adc_oneshot_unit_handle_t adc1 = NULL, adc2 = NULL;
    adc_oneshot_unit_init_cfg_t init1 = { .unit_id = ADC_UNIT_1 };
    adc_oneshot_unit_init_cfg_t init2 = { .unit_id = ADC_UNIT_2 };
    if (adc_oneshot_new_unit(&init1, &adc1) != ESP_OK) { printf("ADC1 init failed\n"); return 1; }
    if (adc_oneshot_new_unit(&init2, &adc2) != ESP_OK) { printf("ADC2 init failed\n"); adc_oneshot_del_unit(adc1); return 1; }

    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,   /* 12-bit on ESP32-S3 */
        .atten    = ADC_ATTEN_DB_12,        /* full 0..~3.1 V range */
    };

    struct pin_state {
        int   gpio;
        adc_oneshot_unit_handle_t unit;
        adc_channel_t             ch;
        bool  configured;
        int   baseline;
        int   last;
    };
    static struct pin_state pins[18];
    int npin = 0;

    for (int gpio = 3; gpio <= 20; ++gpio) {
        if (gpio == 0 || gpio == 1 || gpio == 2 || gpio == 48) continue;
        pins[npin].gpio = gpio;
        if (gpio <= 10) {
            pins[npin].unit = adc1;
            pins[npin].ch   = (adc_channel_t)(gpio - 1);
        } else {
            pins[npin].unit = adc2;
            pins[npin].ch   = (adc_channel_t)(gpio - 11);
        }
        pins[npin].configured = (adc_oneshot_config_channel(pins[npin].unit, pins[npin].ch, &chan_cfg) == ESP_OK);
        npin++;
    }

    printf("--- adc-hunt: baseline (12-bit, 0..4095, atten 12 dB) ---\n");
    for (int i = 0; i < npin; ++i) {
        if (!pins[i].configured) continue;
        int raw = 0;
        if (adc_oneshot_read(pins[i].unit, pins[i].ch, &raw) == ESP_OK) {
            pins[i].baseline = raw;
            pins[i].last     = raw;
            printf("  GPIO %2d : %4d\n", pins[i].gpio, raw);
        }
    }

    printf("--- watching for %d s. Changes > %d counts print. Press each button. ---\n",
           seconds, threshold);

    const int poll_ms = 60;
    const int iters   = (seconds * 1000) / poll_ms;
    for (int t = 0; t < iters; ++t) {
        for (int i = 0; i < npin; ++i) {
            if (!pins[i].configured) continue;
            int raw = 0;
            if (adc_oneshot_read(pins[i].unit, pins[i].ch, &raw) != ESP_OK) continue;
            int delta = raw - pins[i].last;
            if (delta < 0) delta = -delta;
            if (delta > threshold) {
                printf("  GPIO %2d : %4d -> %4d\n", pins[i].gpio, pins[i].last, raw);
                pins[i].last = raw;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(poll_ms));
    }

    adc_oneshot_del_unit(adc1);
    adc_oneshot_del_unit(adc2);
    printf("--- adc-hunt done ---\n");
    return 0;
}

/* --- button (analog read on GPIO 7) --------------------------------- */

/*
 * L and R tactile switches share PIN_BUTTON_ADC (GPIO 7 = ADC1 ch 6)
 * via an on-board resistor ladder: idle sits at one voltage, each
 * button pressed drops (or raises) it into its own band. Two modes:
 *   button           : one-shot read, prints the current raw ADC value
 *   button <seconds> : watch loop, prints any change > 50 counts
 * `seconds` is clamped to 1..120.
 */
static int cmd_button(int argc, char **argv)
{
    int seconds = 0;
    if (argc >= 2) seconds = atoi(argv[1]);
    if (seconds < 0)   seconds = 0;
    if (seconds > 120) seconds = 120;

    adc_oneshot_unit_handle_t adc = NULL;
    adc_oneshot_unit_init_cfg_t init_cfg = { .unit_id = ADC_UNIT_1 };
    if (adc_oneshot_new_unit(&init_cfg, &adc) != ESP_OK) {
        printf("ADC1 init failed\n");
        return 1;
    }
    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten    = ADC_ATTEN_DB_12,
    };
    /* ADC channel for GPIO 7 on S3 = ADC1 channel 6. */
    const adc_channel_t ch = (adc_channel_t)(PIN_BUTTON_ADC - 1);
    if (adc_oneshot_config_channel(adc, ch, &chan_cfg) != ESP_OK) {
        printf("adc channel config failed on GPIO %d\n", PIN_BUTTON_ADC);
        adc_oneshot_del_unit(adc);
        return 1;
    }

    if (seconds == 0) {
        int raw = 0;
        if (adc_oneshot_read(adc, ch, &raw) == ESP_OK) {
            printf("GPIO %d (ADC1 ch %d): %d\n", PIN_BUTTON_ADC, (int)ch, raw);
        } else {
            printf("adc read failed\n");
        }
    } else {
        printf("--- button watch on GPIO %d for %d s (60 ms poll, print delta > 50) ---\n",
               PIN_BUTTON_ADC, seconds);
        const int poll_ms = 60;
        const int iters   = (seconds * 1000) / poll_ms;
        int last = -1;
        for (int t = 0; t < iters; ++t) {
            int raw = 0;
            if (adc_oneshot_read(adc, ch, &raw) == ESP_OK) {
                int delta = raw - last;
                if (delta < 0) delta = -delta;
                if (last < 0 || delta > 50) {
                    printf("  %5d\n", raw);
                    last = raw;
                }
            }
            vTaskDelay(pdMS_TO_TICKS(poll_ms));
        }
        printf("--- button watch done ---\n");
    }

    adc_oneshot_del_unit(adc);
    return 0;
}

/* --- button-state: decode the shared-ADC ladder into L / R / BOTH --- */

static const char *button_classify(int max_raw)
{
    if      (max_raw <= BUTTON_ADC_IDLE_MAX) return "NONE";
    else if (max_raw <= BUTTON_ADC_R_MAX)    return "R";
    else if (max_raw <= BUTTON_ADC_BOTH_MAX) return "BOTH";
    else if (max_raw <= BUTTON_ADC_L_MAX)    return "L";
    else                                      return "?";
}

/* Reads PIN_BUTTON_ADC and takes the max of several samples so a brief
 * contact captured on the rising edge still classifies correctly. Two
 * modes:
 *   button-state           : one-shot (max of 10 reads over 100 ms)
 *   button-state <seconds> : watch loop, print only when the state
 *                            changes (max of 5 reads over 100 ms per
 *                            iteration). Seconds is clamped to 1..120.
 *
 * Physical mapping confirmed by user:
 *   L pressed alone → high band  (peak ~1498)
 *   R pressed alone → low band   (peak ~731)
 *   BOTH pressed    → in the gap between the two single bands
 *                     (network is diode-decoupled or similar — pressing
 *                     both does not rise above L alone)
 */
static int cmd_button_state(int argc, char **argv)
{
    int seconds = 0;
    if (argc >= 2) seconds = atoi(argv[1]);
    if (seconds < 0)   seconds = 0;
    if (seconds > 120) seconds = 120;

    adc_oneshot_unit_handle_t adc = NULL;
    adc_oneshot_unit_init_cfg_t init_cfg = { .unit_id = ADC_UNIT_1 };
    if (adc_oneshot_new_unit(&init_cfg, &adc) != ESP_OK) { printf("ADC1 init failed\n"); return 1; }

    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten    = ADC_ATTEN_DB_12,
    };
    const adc_channel_t ch = (adc_channel_t)(PIN_BUTTON_ADC - 1);
    if (adc_oneshot_config_channel(adc, ch, &chan_cfg) != ESP_OK) {
        printf("adc channel config failed on GPIO %d\n", PIN_BUTTON_ADC);
        adc_oneshot_del_unit(adc);
        return 1;
    }

    if (seconds == 0) {
        int max_raw = 0;
        for (int i = 0; i < 10; ++i) {
            int raw = 0;
            if (adc_oneshot_read(adc, ch, &raw) == ESP_OK && raw > max_raw) max_raw = raw;
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        printf("button: %s (max ADC over 100 ms = %d)\n", button_classify(max_raw), max_raw);
    } else {
        printf("--- button-state watch on GPIO %d for %d s (2-of-2 stability filter) ---\n",
               PIN_BUTTON_ADC, seconds);
        const int iters = (seconds * 1000) / 100;    /* 100 ms per iteration */
        const char *last_reported = "";
        const char *last_seen     = "";
        int         seen_count    = 0;
        for (int t = 0; t < iters; ++t) {
            int max_raw = 0;
            for (int i = 0; i < 5; ++i) {
                int raw = 0;
                if (adc_oneshot_read(adc, ch, &raw) == ESP_OK && raw > max_raw) max_raw = raw;
                vTaskDelay(pdMS_TO_TICKS(20));
            }
            const char *state = button_classify(max_raw);

            /* Only report a state change after seeing it two iterations
             * in a row — filters out the release-decay transient
             * (e.g. L→(decay pass through R band)→NONE reports L→NONE
             * cleanly instead of L→R→NONE). */
            if (strcmp(state, last_seen) == 0) {
                seen_count++;
            } else {
                last_seen  = state;
                seen_count = 1;
            }
            if (seen_count >= 2 && strcmp(state, last_reported) != 0) {
                printf("  %-10s (adc %d)\n", state, max_raw);
                last_reported = state;
            }
        }
        printf("--- watch done ---\n");
    }
    adc_oneshot_del_unit(adc);
    return 0;
}

/* --- read-adc: general ADC read on any ADC-capable GPIO -------------- */

/*
 *   read-adc <gpio>           : one-shot read
 *   read-adc <gpio> <seconds> : watch, print any delta > 50 counts
 *
 * Valid GPIO range on the ESP32-S3 is 1..20 (ADC1 = 1..10, ADC2 = 11..20).
 * ADC2 shares hardware with Wi-Fi; if Wi-Fi is up, ADC2 reads may fail.
 */
static int cmd_read_adc(int argc, char **argv)
{
    if (argc < 2) { printf("usage: read-adc <gpio 1..20> [seconds]\n"); return 1; }
    int gpio    = atoi(argv[1]);
    int seconds = (argc >= 3) ? atoi(argv[2]) : 0;
    if (gpio < 1 || gpio > 20) {
        printf("gpio must be 1..20 (only those pins have an ADC on the ESP32-S3)\n");
        return 1;
    }
    if (seconds < 0)   seconds = 0;
    if (seconds > 120) seconds = 120;

    adc_unit_t    unit_id = (gpio <= 10) ? ADC_UNIT_1 : ADC_UNIT_2;
    adc_channel_t ch      = (adc_channel_t)((gpio <= 10) ? (gpio - 1) : (gpio - 11));

    adc_oneshot_unit_handle_t adc = NULL;
    adc_oneshot_unit_init_cfg_t init_cfg = { .unit_id = unit_id };
    if (adc_oneshot_new_unit(&init_cfg, &adc) != ESP_OK) {
        printf("ADC%d init failed\n", (int)unit_id + 1);
        return 1;
    }
    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten    = ADC_ATTEN_DB_12,
    };
    if (adc_oneshot_config_channel(adc, ch, &chan_cfg) != ESP_OK) {
        printf("channel config failed on GPIO %d (ADC%d ch %d)\n",
               gpio, (int)unit_id + 1, (int)ch);
        adc_oneshot_del_unit(adc);
        return 1;
    }

    if (seconds == 0) {
        int raw = 0;
        if (adc_oneshot_read(adc, ch, &raw) == ESP_OK) {
            printf("GPIO %d (ADC%d ch %d): %d\n", gpio, (int)unit_id + 1, (int)ch, raw);
        } else {
            printf("adc read failed\n");
        }
    } else {
        printf("--- read-adc watch on GPIO %d (ADC%d ch %d) for %d s (60 ms poll) ---\n",
               gpio, (int)unit_id + 1, (int)ch, seconds);
        const int poll_ms = 60;
        const int iters   = (seconds * 1000) / poll_ms;
        int last = -1;
        for (int t = 0; t < iters; ++t) {
            int raw = 0;
            if (adc_oneshot_read(adc, ch, &raw) == ESP_OK) {
                int delta = raw - last;
                if (delta < 0) delta = -delta;
                if (last < 0 || delta > 50) {
                    printf("  %5d\n", raw);
                    last = raw;
                }
            }
            vTaskDelay(pdMS_TO_TICKS(poll_ms));
        }
        printf("--- watch done ---\n");
    }

    adc_oneshot_del_unit(adc);
    return 0;
}

/* --- pin-discovery sweeps ------------------------------------------- */

static int cmd_sweep_led(int argc, char **argv)
{
    (void)argc; (void)argv;
    printf("--- LED pin sweep: watch the on-board RGB LED ---\n");
    printf("    %zu candidates, ~%d ms per pin (R/G/B flash)\n",
           NUM_CANDIDATE_PINS, SWEEP_HOLD_MS);
    for (size_t i = 0; i < NUM_CANDIDATE_PINS; ++i) {
        int gpio = CANDIDATE_PINS[i];
        printf(">>> GPIO %d\n", gpio);
        fflush(stdout);
        led_init(gpio);
        led_set(80, 0,  0); vTaskDelay(pdMS_TO_TICKS(SWEEP_HOLD_MS / 3));
        led_set(0,  80, 0); vTaskDelay(pdMS_TO_TICKS(SWEEP_HOLD_MS / 3));
        led_set(0,  0, 80); vTaskDelay(pdMS_TO_TICKS(SWEEP_HOLD_MS / 3));
        led_set(0, 0, 0);
    }
    printf("--- LED sweep done. The winning GPIO is the one printed just before the LED lit up. ---\n");
    return 0;
}

static int cmd_sweep_servo(int argc, char **argv)
{
    (void)argc; (void)argv;
    printf("--- servo pin sweep: watch the servo horn ---\n");
    printf("    %zu candidates, ~%d ms per pin (0° -> 180° -> 90°)\n",
           NUM_CANDIDATE_PINS, SWEEP_HOLD_MS);
    for (size_t i = 0; i < NUM_CANDIDATE_PINS; ++i) {
        int gpio = CANDIDATE_PINS[i];
        printf(">>> GPIO %d\n", gpio);
        fflush(stdout);
        servo_init(gpio);
        servo_set_deg(0);   vTaskDelay(pdMS_TO_TICKS(SWEEP_HOLD_MS / 3));
        servo_set_deg(180); vTaskDelay(pdMS_TO_TICKS(SWEEP_HOLD_MS / 3));
        servo_set_deg(90);  vTaskDelay(pdMS_TO_TICKS(SWEEP_HOLD_MS / 3));
    }
    printf("--- servo sweep done. The winning GPIO is the one printed just before the horn moved. ---\n");
    return 0;
}

static int cmd_sweep_motor(int argc, char **argv)
{
    (void)argc; (void)argv;
    printf("--- motor pin sweep: watch both DC motors ---\n");
    printf("    Run `stop` first if a motor is still spinning from a previous command.\n");
    printf("    %zu candidates, ~%d ms per pin @ 50%% PWM duty (via LEDC channel 0)\n",
           NUM_CANDIDATE_PINS, SWEEP_HOLD_MS);

    ledc_channel_config_t ch = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = LEDC_CHANNEL_0,
        .timer_sel  = LEDC_TIMER_0,
        .intr_type  = LEDC_INTR_DISABLE,
        .gpio_num   = PIN_MOTOR_1,
        .duty       = 0,
        .hpoint     = 0,
    };
    for (size_t i = 0; i < NUM_CANDIDATE_PINS; ++i) {
        int gpio = CANDIDATE_PINS[i];
        printf(">>> GPIO %d\n", gpio);
        fflush(stdout);
        ch.gpio_num = gpio;
        ch.duty     = (1U << MOTOR_PWM_RES_BITS) / 2;
        ESP_ERROR_CHECK(ledc_channel_config(&ch));
        vTaskDelay(pdMS_TO_TICKS(SWEEP_HOLD_MS));
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    }
    /* Restore channel 0 back to PIN_MOTOR_1 so `motor` commands keep working. */
    ch.gpio_num = PIN_MOTOR_1;
    ch.duty     = 0;
    ESP_ERROR_CHECK(ledc_channel_config(&ch));
    printf("--- motor sweep done. The winning GPIO is the one printed just before a motor spun. ---\n");
    return 0;
}

/* --- I²C shared helper ----------------------------------------------- */

/* Reject phantom "hits" caused by floating / weakly-pulled bus lines:
 *   - skip reserved I²C addresses (0x00-0x07 general-call / Hs-mode,
 *     0x78-0x7F 10-bit addressing) — no real device ever sits there
 *   - require 3 consecutive probes to ACK, since real devices reply
 *     deterministically at 100 kHz while capacitively-coupled phantoms
 *     flicker.
 */
static bool i2c_probe_stable(i2c_master_bus_handle_t bus, uint8_t addr)
{
    if (addr < 0x08 || addr >= 0x78) return false;
    for (int i = 0; i < 3; ++i) {
        if (i2c_master_probe(bus, addr, pdMS_TO_TICKS(5)) != ESP_OK) {
            return false;
        }
    }
    return true;
}

/* --- I²C bus scan ---------------------------------------------------- */

/*
 * Probes every 7-bit address on an I²C bus configured with the given
 * SDA and SCL GPIOs. Useful to check whether the Full Expansion Board
 * routes its motors + PCA9685 servo driver through GPIO1/GPIO2, and to
 * confirm the on-board ES8311 codec (default address 0x18) is visible.
 *
 *   i2c-scan            → SDA=GPIO1, SCL=GPIO2 (default)
 *   i2c-scan 2 1        → try the swapped ordering
 *   i2c-scan 8 9        → common ES8311 wiring on other boards
 */
static int cmd_i2c_scan(int argc, char **argv)
{
    int sda = 1;
    int scl = 2;
    if (argc >= 3) {
        sda = atoi(argv[1]);
        scl = atoi(argv[2]);
    }
    printf("--- I2C scan  SDA=GPIO%d  SCL=GPIO%d  100 kHz ---\n", sda, scl);

    bool paused = i2c_bus_pause_persistent();

    /* Silence the ESP-IDF driver's per-address timeout ERROR so the console
     * stays readable when the bus is empty. Restored at the end. */
    esp_log_level_set("i2c.master", ESP_LOG_NONE);

    i2c_master_bus_config_t bus_config = {
        .i2c_port                     = I2C_NUM_0,
        .sda_io_num                   = sda,
        .scl_io_num                   = scl,
        .clk_source                   = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt            = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus = NULL;
    esp_err_t err = i2c_new_master_bus(&bus_config, &bus);
    if (err != ESP_OK) {
        printf("  bus init failed: %s\n", esp_err_to_name(err));
        esp_log_level_set("i2c.master", ESP_LOG_ERROR);
        i2c_bus_resume_persistent(paused);
        return 1;
    }

    int found = 0;
    for (uint8_t addr = 0x08; addr < 0x78; ++addr) {
        if (i2c_probe_stable(bus, addr)) {
            printf("  device @ 0x%02x\n", addr);
            found++;
        }
    }

    i2c_del_master_bus(bus);
    esp_log_level_set("i2c.master", ESP_LOG_ERROR);
    i2c_bus_resume_persistent(paused);

    if (found == 0) {
        printf("--- nothing on this pin combo ---\n");
    } else {
        printf("--- %d device(s) found ---\n", found);
    }
    return 0;
}

/* --- I2C bus hunt: sweep candidate (SDA, SCL) pin pairs --------------- */

/*
 * Walks a curated list of likely SDA/SCL pin pairs, does a fast full
 * address scan on each, and reports any hits. Both orderings of each
 * pair are tried since we don't know which line is SDA on the header.
 * Runs in ~30 seconds total.
 */
static int cmd_i2c_hunt(int argc, char **argv)
{
    (void)argc; (void)argv;

    static const struct { int sda; int scl; } pairs[] = {
        /* --- known-candidate combos (first hunt round) --- */
        { 1,  2}, { 2,  1},    /* expansion header A1/A2 */
        { 8,  9}, { 9,  8},    /* common ESP32-S3 default I2C pair */
        {17, 18}, {18, 17},    /* frequent codec I2C on S3 dev boards */
        {21, 47}, {47, 21},
        {42, 41}, {41, 42},
        {39, 40}, {40, 39},
        { 5,  6}, { 6,  5},
        {15, 16}, {16, 15},
        {10, 11}, {11, 10},
        {12, 13}, {13, 12},
        {19, 20}, {20, 19},
        { 3, 45}, {45,  3},
        {45, 46}, {46, 45},
        /* --- gap-fill: pins 4, 7, 14, 38 (not covered above) --- */
        { 3,  4}, { 4,  3},
        { 4,  5}, { 5,  4},
        { 4,  6}, { 6,  4},
        { 4,  7}, { 7,  4},
        { 6,  7}, { 7,  6},
        { 7,  8}, { 8,  7},
        {13, 14}, {14, 13},
        {14, 15}, {15, 14},
        {38, 39}, {39, 38},
        {38, 40}, {40, 38},
        {38, 47}, {47, 38},
        /* --- extra adjacency combos --- */
        { 9, 10}, {10,  9},
        {11, 12}, {12, 11},
        {16, 17}, {17, 16},
        {18, 19}, {19, 18},
        {20, 21}, {21, 20},
        {40, 41}, {41, 40},
        {42, 47}, {47, 42},
        {46, 47}, {47, 46},
    };
    const size_t N = sizeof(pairs) / sizeof(pairs[0]);

    printf("--- I2C hunt: %zu (SDA, SCL) pairs @ 100 kHz ---\n", N);
    bool paused = i2c_bus_pause_persistent();
    esp_log_level_set("i2c.master", ESP_LOG_NONE);

    int total_hits = 0;
    int pairs_with_hits = 0;
    int pairs_bus_failed = 0;
    for (size_t p = 0; p < N; ++p) {
        int sda = pairs[p].sda;
        int scl = pairs[p].scl;

        i2c_master_bus_config_t cfg = {
            .i2c_port                     = I2C_NUM_0,
            .sda_io_num                   = sda,
            .scl_io_num                   = scl,
            .clk_source                   = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt            = 7,
            .flags.enable_internal_pullup = true,
        };
        i2c_master_bus_handle_t bus = NULL;
        esp_err_t bus_err = i2c_new_master_bus(&cfg, &bus);
        if (bus_err != ESP_OK) {
            printf("  SDA=%2d SCL=%2d : bus init failed (%s)\n",
                   sda, scl, esp_err_to_name(bus_err));
            pairs_bus_failed++;
            continue;
        }

        int hits = 0;
        char buf[128];
        int  buf_used = 0;
        for (uint8_t addr = 0x08; addr < 0x78; ++addr) {
            if (i2c_probe_stable(bus, addr)) {
                buf_used += snprintf(buf + buf_used, sizeof(buf) - buf_used,
                                     "%s0x%02x", hits ? ", " : "", addr);
                hits++;
                if (buf_used >= (int)sizeof(buf) - 8) break;
            }
        }
        if (hits > 0) {
            printf("  SDA=%2d SCL=%2d : %s  (%d device%s)\n",
                   sda, scl, buf, hits, hits == 1 ? "" : "s");
            total_hits += hits;
            pairs_with_hits++;
        }
        i2c_del_master_bus(bus);
    }

    esp_log_level_set("i2c.master", ESP_LOG_ERROR);
    i2c_bus_resume_persistent(paused);

    if (pairs_bus_failed > 0) {
        printf("--- warning: %d pair(s) could not open a bus at all ---\n",
               pairs_bus_failed);
    }
    if (total_hits == 0) {
        printf("--- no I2C device found on any tested pair ---\n");
        printf("    check that the expansion board is clipped on and powered\n");
    } else {
        printf("--- done: %d device%s across %d pin pair%s ---\n",
               total_hits, total_hits == 1 ? "" : "s",
               pairs_with_hits, pairs_with_hits == 1 ? "" : "s");
    }
    return 0;
}

/* --- I2C selftest: sanity-check the probe machinery on one pin pair ---- */

/*
 * Exercises the full GPIO + I2C driver path on a given (SDA, SCL) pair
 * so an empty `i2c-hunt` can be distinguished from a broken probe.
 *
 *   1. Configure both pins as inputs with internal pullups; they must
 *      read HIGH. If either reads LOW, the line is shorted to GND or the
 *      internal pullup isn't holding.
 *   2. Drive each pin LOW as a plain GPIO output; both must read LOW.
 *   3. Release back to input+pullup; both must return HIGH.
 *   4. Open an I2C master on the pair and probe 0x50 — on an empty bus
 *      this must return an error (timeout / not found).
 *
 * If steps 1–3 pass, the probe machinery can be trusted. Then physically
 * short SDA to GND and re-run `i2c-hunt`; if devices "appear" at every
 * address, the probe is really reaching the wire.
 */
static int cmd_i2c_selftest(int argc, char **argv)
{
    if (argc < 3) {
        printf("usage: i2c-selftest <sda> <scl>\n");
        return 1;
    }
    int sda = atoi(argv[1]);
    int scl = atoi(argv[2]);

    printf("--- I2C selftest  SDA=GPIO%d  SCL=GPIO%d ---\n", sda, scl);
    int fail = 0;

    const int pins[2] = { sda, scl };

    /* Step 1: idle with internal pullups. */
    for (int i = 0; i < 2; ++i) {
        gpio_config_t cfg = {
            .pin_bit_mask = 1ULL << pins[i],
            .mode         = GPIO_MODE_INPUT,
            .pull_up_en   = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type    = GPIO_INTR_DISABLE,
        };
        gpio_config(&cfg);
    }
    vTaskDelay(pdMS_TO_TICKS(5));
    int sda_h = gpio_get_level(sda), scl_h = gpio_get_level(scl);
    bool ok1 = sda_h && scl_h;
    printf("  idle (pullup) : SDA=%d SCL=%d  %s\n",
           sda_h, scl_h, ok1 ? "OK" : "FAIL (line stuck low or no pullup)");
    if (!ok1) fail = 1;

    /* Step 2: drive each pin low. */
    for (int i = 0; i < 2; ++i) {
        gpio_set_direction(pins[i], GPIO_MODE_OUTPUT);
        gpio_set_level(pins[i], 0);
    }
    vTaskDelay(pdMS_TO_TICKS(2));
    int sda_l = gpio_get_level(sda), scl_l = gpio_get_level(scl);
    bool ok2 = !sda_l && !scl_l;
    printf("  driven low    : SDA=%d SCL=%d  %s\n",
           sda_l, scl_l, ok2 ? "OK" : "FAIL (driver can't sink line)");
    if (!ok2) fail = 1;

    /* Step 3: release back to input+pullup. */
    for (int i = 0; i < 2; ++i) {
        gpio_set_direction(pins[i], GPIO_MODE_INPUT);
    }
    vTaskDelay(pdMS_TO_TICKS(5));
    int sda_h2 = gpio_get_level(sda), scl_h2 = gpio_get_level(scl);
    bool ok3 = sda_h2 && scl_h2;
    printf("  re-idle       : SDA=%d SCL=%d  %s\n",
           sda_h2, scl_h2, ok3 ? "OK" : "FAIL (pin stuck low after release)");
    if (!ok3) fail = 1;

    /* Step 4: open the I2C master and probe an empty address. */
    bool paused = i2c_bus_pause_persistent();
    esp_log_level_set("i2c.master", ESP_LOG_NONE);
    i2c_master_bus_config_t bus_config = {
        .i2c_port                     = I2C_NUM_0,
        .sda_io_num                   = sda,
        .scl_io_num                   = scl,
        .clk_source                   = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt            = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus = NULL;
    esp_err_t err = i2c_new_master_bus(&bus_config, &bus);
    if (err != ESP_OK) {
        printf("  i2c bus init  : FAIL (%s)\n", esp_err_to_name(err));
        esp_log_level_set("i2c.master", ESP_LOG_ERROR);
        i2c_bus_resume_persistent(paused);
        return 1;
    }
    esp_err_t probe = i2c_master_probe(bus, 0x50, pdMS_TO_TICKS(10));
    bool ok4 = (probe != ESP_OK);
    printf("  probe @0x50   : %s  %s\n",
           esp_err_to_name(probe),
           ok4 ? "OK (expected on empty bus)"
               : "FAIL (unexpected ACK — bus stuck low or phantom)");
    if (!ok4) fail = 1;
    i2c_del_master_bus(bus);
    esp_log_level_set("i2c.master", ESP_LOG_ERROR);
    i2c_bus_resume_persistent(paused);

    if (fail) {
        printf("--- selftest FAILED — do not trust `i2c-hunt` results on this pair ---\n");
    } else {
        printf("--- selftest OK — probe machinery works on this pair ---\n");
        printf("    Next: short SDA (GPIO%d) to GND with a jumper and re-run\n", sda);
        printf("    `i2c-hunt`. If every address ACKs, the probe reaches the wire.\n");
    }
    return 0;
}

/* --- pin-hunt: physically identify a header pin's GPIO -------------- */

/*
 * Configures every candidate GPIO as INPUT_PULLUP, prints the idle
 * baseline (any pin already LOW is flagged — it's being driven by
 * something, shorted, or lacks a working internal pullup), then polls
 * for the given number of seconds and reports every HIGH<->LOW
 * transition.
 *
 * Usage: jumper the header pin to the '-' (GND) pad on the expansion
 * port. The GPIO that flips LOW is that header pin's number. Un-jumper
 * and it flips back HIGH. Repeat for A2.
 *
 * NOTE: this steals pins currently owned by other drivers (LEDC motor
 * PWM on 1/2, RMT LED on 48, LEDC servo on the current servo pin).
 * Run `stop` and `led-off` first, and reboot after to restore them.
 */
static int cmd_pin_hunt(int argc, char **argv)
{
    int seconds = 20;
    if (argc >= 2) seconds = atoi(argv[1]);
    if (seconds < 3)   seconds = 3;
    if (seconds > 120) seconds = 120;

    printf("--- pin-hunt: watching %zu candidate GPIOs for %d s ---\n",
           NUM_CANDIDATE_PINS, seconds);
    printf("    Jumper the header pin to '-' (GND). The GPIO that flips LOW is its number.\n");
    printf("    Run `stop` and `led-off` first so nothing is actively driving a candidate.\n");
    printf("    Reboot afterwards to restore motor/LED/servo drivers.\n");

    for (size_t i = 0; i < NUM_CANDIDATE_PINS; ++i) {
        gpio_config_t cfg = {
            .pin_bit_mask = 1ULL << CANDIDATE_PINS[i],
            .mode         = GPIO_MODE_INPUT,
            .pull_up_en   = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type    = GPIO_INTR_DISABLE,
        };
        gpio_config(&cfg);
    }
    vTaskDelay(pdMS_TO_TICKS(20));

    bool state[NUM_CANDIDATE_PINS];
    for (size_t i = 0; i < NUM_CANDIDATE_PINS; ++i) {
        state[i] = (gpio_get_level(CANDIDATE_PINS[i]) != 0);
        if (!state[i]) {
            printf("  GPIO %2d LOW at idle (driven by peripheral, shorted, or no pullup)\n",
                   CANDIDATE_PINS[i]);
        }
    }
    printf("  idle baseline captured — jumper now\n");

    const int poll_ms = 30;
    const int iters   = (seconds * 1000) / poll_ms;
    for (int t = 0; t < iters; ++t) {
        for (size_t i = 0; i < NUM_CANDIDATE_PINS; ++i) {
            bool now = (gpio_get_level(CANDIDATE_PINS[i]) != 0);
            if (now != state[i]) {
                state[i] = now;
                printf("  GPIO %2d -> %s\n", CANDIDATE_PINS[i], now ? "HIGH" : "LOW");
            }
        }
        vTaskDelay(pdMS_TO_TICKS(poll_ms));
    }

    printf("--- pin-hunt done. Reboot to restore driver ownership of any repurposed pins. ---\n");
    return 0;
}

/* --- i2c-hunt-full: brute-force every ordered pair of candidate pins - */

/*
 * Runs a full 0x08..0x77 scan on every ordered (SDA, SCL) pair drawn
 * from CANDIDATE_PINS — 30 * 29 = 870 pairs. Takes several minutes but
 * is the exhaustive answer for "where is the I2C bus?". Prints a
 * progress marker every 100 pairs so it's clear the box isn't hung.
 */
static int cmd_i2c_hunt_full(int argc, char **argv)
{
    (void)argc; (void)argv;

    const size_t total_pairs = NUM_CANDIDATE_PINS * (NUM_CANDIDATE_PINS - 1);
    printf("--- I2C hunt-full: %zu ordered pin pairs (~5 min) ---\n", total_pairs);
    printf("    Run `stop` and `led-off` first so nothing drives a candidate pin.\n");
    bool paused = i2c_bus_pause_persistent();
    esp_log_level_set("i2c.master", ESP_LOG_NONE);

    int    total_hits        = 0;
    int    pairs_with_hits   = 0;
    int    pairs_bus_failed  = 0;
    size_t pairs_tested      = 0;

    for (size_t a = 0; a < NUM_CANDIDATE_PINS; ++a) {
        for (size_t b = 0; b < NUM_CANDIDATE_PINS; ++b) {
            if (a == b) continue;
            int sda = CANDIDATE_PINS[a];
            int scl = CANDIDATE_PINS[b];

            i2c_master_bus_config_t cfg = {
                .i2c_port                     = I2C_NUM_0,
                .sda_io_num                   = sda,
                .scl_io_num                   = scl,
                .clk_source                   = I2C_CLK_SRC_DEFAULT,
                .glitch_ignore_cnt            = 7,
                .flags.enable_internal_pullup = true,
            };
            i2c_master_bus_handle_t bus = NULL;
            esp_err_t bus_err = i2c_new_master_bus(&cfg, &bus);
            if (bus_err != ESP_OK) {
                if (pairs_bus_failed < 3) {
                    printf("  SDA=%2d SCL=%2d : bus init failed (%s)\n",
                           sda, scl, esp_err_to_name(bus_err));
                }
                pairs_bus_failed++;
                pairs_tested++;
                continue;
            }

            int  hits     = 0;
            char buf[128];
            int  buf_used = 0;
            for (uint8_t addr = 0x08; addr < 0x78; ++addr) {
                if (i2c_probe_stable(bus, addr)) {
                    buf_used += snprintf(buf + buf_used, sizeof(buf) - buf_used,
                                         "%s0x%02x", hits ? ", " : "", addr);
                    hits++;
                    if (buf_used >= (int)sizeof(buf) - 8) break;
                }
            }
            if (hits > 0) {
                printf("  SDA=%2d SCL=%2d : %s  (%d device%s)\n",
                       sda, scl, buf, hits, hits == 1 ? "" : "s");
                total_hits      += hits;
                pairs_with_hits += 1;
            }
            i2c_del_master_bus(bus);

            pairs_tested++;
            if (pairs_tested % 100 == 0) {
                printf("  ... %zu / %zu pairs tested\n", pairs_tested, total_pairs);
                fflush(stdout);
            }
        }
    }

    esp_log_level_set("i2c.master", ESP_LOG_ERROR);
    i2c_bus_resume_persistent(paused);

    if (pairs_bus_failed > 0) {
        printf("--- warning: %d/%zu pairs could not open a bus at all\n",
               pairs_bus_failed, total_pairs);
        printf("    scan results below cover only the %zu pairs that opened a bus.\n",
               total_pairs - (size_t)pairs_bus_failed);
    }
    if (total_hits == 0) {
        printf("--- no I2C device on any of %zu ordered pairs ---\n", total_pairs);
        printf("    A1/A2 are not among the candidate pins, or the target isn't answering.\n");
    } else {
        printf("--- done: %d device%s across %d pin pair%s ---\n",
               total_hits, total_hits == 1 ? "" : "s",
               pairs_with_hits, pairs_with_hits == 1 ? "" : "s");
    }
    return 0;
}

/* --- bit-banged I2C scan -------------------------------------------------
 *
 * Bypasses the ESP-IDF i2c_master driver completely — just toggles SDA
 * and SCL as open-drain GPIOs. Used to isolate "no ACK" cases: if this
 * finds the chip but i2c-scan doesn't, the ESP-IDF driver / peripheral
 * setup is at fault; if both find nothing, the chip is genuinely silent.
 *
 * Open-drain convention:
 *   line HIGH → configure as INPUT, pull-up floats it high
 *   line LOW  → configure as OUTPUT_OD driven to 0
 */
#define BB_HALF_US 5   /* ~100 kHz SCL: 5 us per half-cycle */

static inline void bb_high(int pin) { gpio_set_direction(pin, GPIO_MODE_INPUT); }
static inline void bb_low(int pin)
{
    gpio_set_level(pin, 0);
    gpio_set_direction(pin, GPIO_MODE_OUTPUT_OD);
}

static void bb_init(int sda, int scl)
{
    gpio_reset_pin(sda);
    gpio_reset_pin(scl);
    gpio_set_pull_mode(sda, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(scl, GPIO_PULLUP_ONLY);
    gpio_set_level(sda, 0);
    gpio_set_level(scl, 0);
    bb_high(sda);
    bb_high(scl);
    esp_rom_delay_us(BB_HALF_US * 4);
}

static void bb_start(int sda, int scl)
{
    bb_high(sda); bb_high(scl); esp_rom_delay_us(BB_HALF_US);
    bb_low(sda);                esp_rom_delay_us(BB_HALF_US);
    bb_low(scl);                esp_rom_delay_us(BB_HALF_US);
}

static void bb_stop(int sda, int scl)
{
    bb_low(sda);                esp_rom_delay_us(BB_HALF_US);
    bb_high(scl);               esp_rom_delay_us(BB_HALF_US);
    bb_high(sda);               esp_rom_delay_us(BB_HALF_US);
}

/* SCL must be low on entry; leaves SCL low. */
static void bb_send_bit(int sda, int scl, int bit)
{
    if (bit) bb_high(sda); else bb_low(sda);
    esp_rom_delay_us(BB_HALF_US);
    bb_high(scl);               esp_rom_delay_us(BB_HALF_US);
    bb_low(scl);                esp_rom_delay_us(BB_HALF_US);
}

/* Releases SDA, clocks SCL once, samples SDA. Returns 0 (ACK) or 1 (NACK). */
static int bb_read_bit(int sda, int scl)
{
    bb_high(sda);               esp_rom_delay_us(BB_HALF_US);
    bb_high(scl);               esp_rom_delay_us(BB_HALF_US);
    int bit = gpio_get_level(sda);
    bb_low(scl);                esp_rom_delay_us(BB_HALF_US);
    return bit;
}

/* Sends one byte MSB-first, returns 0 if slave ACKed, 1 if NACK. */
static int bb_send_byte(int sda, int scl, uint8_t byte)
{
    for (int b = 7; b >= 0; --b) {
        bb_send_bit(sda, scl, (byte >> b) & 1);
    }
    return bb_read_bit(sda, scl);
}

static bool bb_probe(int sda, int scl, uint8_t addr7)
{
    bb_start(sda, scl);
    int ack = bb_send_byte(sda, scl, (uint8_t)(addr7 << 1));  /* write */
    bb_stop(sda, scl);
    return (ack == 0);
}

/* Write N bytes to a device: START + (addr<<1|W) + byte0 + byte1 + ... + STOP.
 * Returns true iff every byte was ACKed. */
static bool bb_write(int sda, int scl, uint8_t addr7, const uint8_t *buf, size_t n)
{
    bb_start(sda, scl);
    if (bb_send_byte(sda, scl, (uint8_t)(addr7 << 1)) != 0) {
        bb_stop(sda, scl);
        return false;
    }
    for (size_t i = 0; i < n; ++i) {
        if (bb_send_byte(sda, scl, buf[i]) != 0) {
            bb_stop(sda, scl);
            return false;
        }
    }
    bb_stop(sda, scl);
    return true;
}

static bool bb_write_reg(int sda, int scl, uint8_t addr7, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return bb_write(sda, scl, addr7, buf, 2);
}

/* --- bit-banged TLC59108 driver ------------------------------------------
 *
 * Same init/set sequence as the peripheral-based tlc59108_init(), but the
 * transport is bit-bang. Kept independent of the s_i2c_bus / s_tlc state
 * so it works even when the ESP-IDF driver refuses to talk to the chip. */

static bool bb_tlc59108_init_seq(int sda, int scl)
{
    /* Per datasheet: clear OSC (MODE1=0), wait 500us, MODE2=0, LEDOUTn=0,
     * then zero all PWM channels via auto-increment. */
    if (!bb_write_reg(sda, scl, TLC59108_ADDR, TLC59108_MODE1, 0x00)) return false;
    esp_rom_delay_us(500);
    if (!bb_write_reg(sda, scl, TLC59108_ADDR, TLC59108_MODE2,   0x00)) return false;
    if (!bb_write_reg(sda, scl, TLC59108_ADDR, TLC59108_LEDOUT0, 0x00)) return false;
    if (!bb_write_reg(sda, scl, TLC59108_ADDR, TLC59108_LEDOUT1, 0x00)) return false;

    uint8_t pwm_zero[9] = { TLC59108_AI_BRIGHT | TLC59108_PWM0,
                            0, 0, 0, 0, 0, 0, 0, 0 };
    return bb_write(sda, scl, TLC59108_ADDR, pwm_zero, sizeof(pwm_zero));
}

/* Sets LDRn bits for one channel to PWM mode without disturbing others. */
static bool bb_tlc_ensure_pwm_mode(int sda, int scl, int ch)
{
    uint8_t reg   = (ch < 4) ? TLC59108_LEDOUT0 : TLC59108_LEDOUT1;
    int     shift = 2 * (ch % 4);
    /* Read-modify-write is possible via bit-bang too, but on this driver
     * we never touch other channels behind the caller's back, so a simple
     * "all channels in this half are PWM" strategy is cheaper and matches
     * how sweep/set use the chip in practice. */
    uint8_t all_pwm_half = 0;
    for (int i = 0; i < 4; ++i) all_pwm_half |= (TLC59108_LDR_PWM << (2 * i));
    (void)shift;
    return bb_write_reg(sda, scl, TLC59108_ADDR, reg, all_pwm_half);
}

static bool bb_tlc_set_channel(int sda, int scl, int ch, uint8_t pwm_val)
{
    if (ch < 0 || ch > 7) return false;
    if (!bb_tlc_ensure_pwm_mode(sda, scl, ch)) return false;
    return bb_write_reg(sda, scl, TLC59108_ADDR, TLC59108_PWM0 + ch, pwm_val);
}

static int cmd_bb_tlc_init(int argc, char **argv)
{
    (void)argc; (void)argv;
    int sda = PIN_I2C_SDA, scl = PIN_I2C_SCL;

    bool paused = i2c_bus_pause_persistent();
    bb_init(sda, scl);
    bool ok = bb_tlc59108_init_seq(sda, scl);
    if (!ok) {
        printf("bb-tlc-init: NACK on init sequence\n");
        gpio_set_direction(sda, GPIO_MODE_INPUT);
        gpio_set_direction(scl, GPIO_MODE_INPUT);
        i2c_bus_resume_persistent(paused);
        s_bb_ready = false;
        return 1;
    }
    printf("bb-tlc-init: OK — TLC59108 initialised via bit-bang @ 0x%02x\n",
           TLC59108_ADDR);
    /* Keep pins in a released state; each bb_* call re-configures as needed. */
    gpio_set_direction(sda, GPIO_MODE_INPUT);
    gpio_set_direction(scl, GPIO_MODE_INPUT);
    /* Deliberately do NOT resume the persistent bus — the peripheral
     * driver is broken for us; leaving it torn down avoids reclaiming
     * the pins and stomping on subsequent bb_* calls. */
    (void)paused;
    s_bb_ready = true;
    return 0;
}

static int cmd_bb_tlc_set(int argc, char **argv)
{
    if (!s_bb_ready) { printf("run `bb-tlc-init` first\n"); return 1; }
    if (argc < 3) { printf("usage: bb-tlc-set <ch 0-7> <pct 0-100>\n"); return 1; }
    int ch  = atoi(argv[1]);
    int pct = atoi(argv[2]);
    if (ch < 0 || ch > 7)  { printf("ch out of range (0..7)\n"); return 1; }
    if (pct < 0)   pct = 0;
    if (pct > 100) pct = 100;
    uint8_t val = (uint8_t)(pct * 255 / 100);
    if (!bb_tlc_set_channel(PIN_I2C_SDA, PIN_I2C_SCL, ch, val)) {
        printf("bb-tlc-set: NACK writing channel %d\n", ch);
        return 1;
    }
    printf("bb-tlc-set: ch %d = %d%% (raw 0x%02x sink duty)\n", ch, pct, val);
    return 0;
}

/* Cycles each of the 8 TLC59108 channels at 50%% duty for 2 s, so the
 * user can watch which channel drives which motor/port output on the
 * expansion board. Everything else is parked off. */
static int cmd_bb_tlc_sweep(int argc, char **argv)
{
    (void)argc; (void)argv;
    if (!s_bb_ready) { printf("run `bb-tlc-init` first\n"); return 1; }
    printf("--- bb-tlc-sweep: 8 channels, 50%% duty, 2 s each ---\n");
    for (int ch = 0; ch < 8; ++ch) {
        printf("  channel %d ON\n", ch);
        fflush(stdout);
        /* Park all channels first, then activate just this one. */
        bb_write_reg(PIN_I2C_SDA, PIN_I2C_SCL, TLC59108_ADDR,
                     TLC59108_LEDOUT0, 0x00);
        bb_write_reg(PIN_I2C_SDA, PIN_I2C_SCL, TLC59108_ADDR,
                     TLC59108_LEDOUT1, 0x00);
        if (!bb_tlc_set_channel(PIN_I2C_SDA, PIN_I2C_SCL, ch, 128)) {
            printf("  NACK on channel %d — aborting\n", ch);
            return 1;
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
    /* Park all off at the end. */
    bb_write_reg(PIN_I2C_SDA, PIN_I2C_SCL, TLC59108_ADDR,
                 TLC59108_LEDOUT0, 0x00);
    bb_write_reg(PIN_I2C_SDA, PIN_I2C_SCL, TLC59108_ADDR,
                 TLC59108_LEDOUT1, 0x00);
    printf("--- sweep done, all channels parked off ---\n");
    return 0;
}

/* --- motor drive (bit-bang TLC59108 → DRV8833) ---------------------------
 *
 * Discovered channel map on this board (via `bb-tlc-sweep`):
 *   M2 driven by TLC59108 channels 0 (dir+) and 1 (dir-)
 *   M1 driven by TLC59108 channels 2 (dir+) and 3 (dir-)
 * All four sit in LEDOUT0. Channels 4..7 (LEDOUT1) feed the P1..P4 header
 * outputs and are left untouched by motor commands.
 *
 * Drive scheme: fast-decay PWM. For +N% on a motor, the +dir channel goes
 * LDR=PWM at N% sink-duty (line low N% of the 97 kHz period → DRV8833
 * input low N% of the time), while the -dir channel goes LDR=OFF (sink
 * disabled → line pulled HIGH by the expansion board's external pullup →
 * DRV8833 input constantly HIGH). The chip then alternates between
 * (L,H)=drive and (H,H)=brake at 97 kHz, yielding smooth N% duty in the
 * chosen direction. Stop = LDR=OFF on both channels = brake. */
#define MOTOR_M2_PLUS   0
#define MOTOR_M2_MINUS  1
#define MOTOR_M1_PLUS   2
#define MOTOR_M1_MINUS  3

static uint8_t ledout0_for_motors(int m1_signed, int m2_signed)
{
    uint8_t r = 0;
    /* M2 pair: channels 0 and 1. */
    if      (m2_signed > 0) r |= (TLC59108_LDR_PWM << (2 * MOTOR_M2_PLUS));
    else if (m2_signed < 0) r |= (TLC59108_LDR_PWM << (2 * MOTOR_M2_MINUS));
    /* M1 pair: channels 2 and 3. */
    if      (m1_signed > 0) r |= (TLC59108_LDR_PWM << (2 * MOTOR_M1_PLUS));
    else if (m1_signed < 0) r |= (TLC59108_LDR_PWM << (2 * MOTOR_M1_MINUS));
    /* Every other pair position stays at LDR_OFF (0b00) → sink off →
     * line pulled HIGH → DRV8833 input HIGH → brake input for that side. */
    return r;
}

/* Public shim for http_control.c — 0 on ok, -1 if the bit-bang TLC path
 * hasn't been initialised (bb-tlc-init at boot or via REPL). */
int bb_motor_set_public(int m1_signed, int m2_signed)
{
    return bb_motor_set(m1_signed, m2_signed) ? 0 : -1;
}

static bool bb_motor_set(int m1_signed, int m2_signed)
{
    if (m1_signed < -100) m1_signed = -100;
    if (m1_signed > 100)  m1_signed = 100;
    if (m2_signed < -100) m2_signed = -100;
    if (m2_signed > 100)  m2_signed = 100;

    int sda = PIN_I2C_SDA;
    int scl = PIN_I2C_SCL;

    int m1_mag = m1_signed >= 0 ? m1_signed : -m1_signed;
    int m2_mag = m2_signed >= 0 ? m2_signed : -m2_signed;
    uint8_t m1_pwm = (uint8_t)(m1_mag * 255 / 100);
    uint8_t m2_pwm = (uint8_t)(m2_mag * 255 / 100);

    /* Write PWM values first so switching LEDOUT0 to PWM mode never
     * exposes a stale duty in a newly-activated channel. Unused
     * (opposite-direction) channels get the same value but are LDR=OFF,
     * so their PWM register is ignored. */
    if (!bb_write_reg(sda, scl, TLC59108_ADDR,
                      TLC59108_PWM0 + MOTOR_M2_PLUS,  m2_pwm)) return false;
    if (!bb_write_reg(sda, scl, TLC59108_ADDR,
                      TLC59108_PWM0 + MOTOR_M2_MINUS, m2_pwm)) return false;
    if (!bb_write_reg(sda, scl, TLC59108_ADDR,
                      TLC59108_PWM0 + MOTOR_M1_PLUS,  m1_pwm)) return false;
    if (!bb_write_reg(sda, scl, TLC59108_ADDR,
                      TLC59108_PWM0 + MOTOR_M1_MINUS, m1_pwm)) return false;

    return bb_write_reg(sda, scl, TLC59108_ADDR, TLC59108_LEDOUT0,
                        ledout0_for_motors(m1_signed, m2_signed));
}

static int cmd_bb_scan(int argc, char **argv)
{
    int sda = PIN_I2C_SDA;
    int scl = PIN_I2C_SCL;
    if (argc >= 3) { sda = atoi(argv[1]); scl = atoi(argv[2]); }

    printf("--- bit-banged I2C scan  SDA=GPIO%d  SCL=GPIO%d  ~100 kHz ---\n",
           sda, scl);
    printf("    Bypasses ESP-IDF i2c_master driver entirely.\n");

    bool paused = i2c_bus_pause_persistent();
    bb_init(sda, scl);

    int idle_sda = gpio_get_level(sda);
    int idle_scl = gpio_get_level(scl);
    printf("    idle: SDA=%d SCL=%d  %s\n", idle_sda, idle_scl,
           (idle_sda && idle_scl) ? "OK"
                                  : "FAIL (line stuck low — nothing to probe)");

    int found = 0;
    if (idle_sda && idle_scl) {
        for (uint8_t addr = 0x08; addr < 0x78; ++addr) {
            if (bb_probe(sda, scl, addr)) {
                printf("  device @ 0x%02x (ACKed via bit-bang)\n", addr);
                found++;
            }
        }
    }

    gpio_set_direction(sda, GPIO_MODE_INPUT);
    gpio_set_direction(scl, GPIO_MODE_INPUT);
    i2c_bus_resume_persistent(paused);

    if (found == 0) {
        printf("--- no device ACKed via bit-bang either.\n");
        printf("    Chip really isn't responding on this pair — hardware fault.\n");
    } else {
        printf("--- %d device(s) ACKed via bit-bang.\n", found);
        printf("    Chip is alive. The ESP-IDF i2c_master driver setup is at fault;\n");
        printf("    we'll fix that next.\n");
    }
    return 0;
}

/* --- PDM digital-mic probe -------------------------------------------
 *
 * Hypothesis (2026-09): the on-board mic is not routed through the
 * ES8311 codec but is a standalone PDM MEMS device (candidates:
 * ST MP34DT05TR-A, MP34DT06JTR). If so, its CLK and DAT pins land
 * directly on ESP32-S3 GPIOs — which the S3's I2S peripheral can
 * read natively via `i2s_channel_init_pdm_rx_mode`. These commands
 * brute-force that space without touching the codec path.
 *
 * Free user GPIOs on this board (from pins.h): 4, 5, 6, 8, 9, 10, 11,
 * 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 38, 39, 40, 41, 42. Excludes
 * confirmed assignments (0 servo, 1/2 I2C, 7 button ADC, 47 PA_EN,
 * 48 LED), flash/PSRAM (26..37), and UART0 (43/44).
 */

static const int PDM_CANDIDATES[] = {
    4, 5, 6, 8, 9, 10, 11, 12, 13, 14, 15,
    16, 17, 18, 19, 20, 21, 38, 39, 40, 41, 42,
};
#define PDM_CANDIDATES_N (sizeof(PDM_CANDIDATES) / sizeof(PDM_CANDIDATES[0]))

#define PDM_SAMPLE_RATE_HZ 16000

/* Read `ms` milliseconds of PDM mono audio on (clk, dat) and fill out
 * n_samples / rms / peak2peak. Fully allocates + tears down the I2S
 * channel each call so the caller can iterate pin pairs freely.
 * Returns ESP_OK on success; on failure the out-params are undefined. */
static esp_err_t pdm_capture(int clk_gpio, int dat_gpio, int ms,
                             int *out_n, double *out_rms, int *out_p2p)
{
    if (ms <= 0) ms = 100;

    i2s_chan_handle_t rx = NULL;
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    esp_err_t e = i2s_new_channel(&chan_cfg, NULL, &rx);
    if (e != ESP_OK) return e;

    i2s_pdm_rx_config_t pdm_cfg = {
        .clk_cfg  = I2S_PDM_RX_CLK_DEFAULT_CONFIG(PDM_SAMPLE_RATE_HZ),
        .slot_cfg = I2S_PDM_RX_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT,
                                                   I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .clk = (gpio_num_t)clk_gpio,
            .din = (gpio_num_t)dat_gpio,
            .invert_flags = { .clk_inv = false },
        },
    };
    e = i2s_channel_init_pdm_rx_mode(rx, &pdm_cfg);
    if (e != ESP_OK) { i2s_del_channel(rx); return e; }

    e = i2s_channel_enable(rx);
    if (e != ESP_OK) { i2s_del_channel(rx); return e; }

    int nsamples = (PDM_SAMPLE_RATE_HZ / 1000) * ms;
    size_t nbytes = nsamples * sizeof(int16_t);
    int16_t *buf = (int16_t *)malloc(nbytes);
    if (!buf) {
        i2s_channel_disable(rx);
        i2s_del_channel(rx);
        return ESP_ERR_NO_MEM;
    }

    size_t bytes_read = 0;
    e = i2s_channel_read(rx, buf, nbytes, &bytes_read, pdMS_TO_TICKS(ms + 500));

    i2s_channel_disable(rx);
    i2s_del_channel(rx);

    if (e != ESP_OK) { free(buf); return e; }

    int n = (int)(bytes_read / sizeof(int16_t));
    int16_t vmin = INT16_MAX, vmax = INT16_MIN;
    int64_t sum_sq = 0;
    for (int i = 0; i < n; i++) {
        int16_t s = buf[i];
        if (s < vmin) vmin = s;
        if (s > vmax) vmax = s;
        sum_sq += (int32_t)s * (int32_t)s;
    }
    free(buf);

    *out_n   = n;
    *out_rms = n > 0 ? sqrt((double)sum_sq / (double)n) : 0.0;
    *out_p2p = (int)vmax - (int)vmin;
    return ESP_OK;
}

static int cmd_pdm_probe(int argc, char **argv)
{
    if (argc < 3) {
        printf("usage: pdm-probe <clk_gpio> <dat_gpio> [ms]\n");
        return 1;
    }
    int clk = atoi(argv[1]);
    int dat = atoi(argv[2]);
    int ms  = (argc >= 4) ? atoi(argv[3]) : 200;

    int n = 0, p2p = 0;
    double rms = 0.0;
    esp_err_t e = pdm_capture(clk, dat, ms, &n, &rms, &p2p);
    if (e != ESP_OK) {
        printf("clk=%d dat=%d FAILED (%s)\n", clk, dat, esp_err_to_name(e));
        return 1;
    }
    printf("clk=%d dat=%d n=%d rms=%.1f peak2peak=%d\n", clk, dat, n, rms, p2p);
    return 0;
}

static int cmd_pdm_watch(int argc, char **argv)
{
    if (argc < 3) {
        printf("usage: pdm-watch <clk_gpio> <dat_gpio> [seconds]\n"
               "       prints RMS/peak every 100 ms — snap fingers to see it spike\n");
        return 1;
    }
    int clk = atoi(argv[1]);
    int dat = atoi(argv[2]);
    int secs = (argc >= 4) ? atoi(argv[3]) : 5;
    if (secs <= 0) secs = 5;

    int chunks = secs * 10;
    for (int i = 0; i < chunks; i++) {
        int n = 0, p2p = 0;
        double rms = 0.0;
        esp_err_t e = pdm_capture(clk, dat, 100, &n, &rms, &p2p);
        if (e != ESP_OK) {
            printf("[%d] FAILED (%s)\n", i, esp_err_to_name(e));
            return 1;
        }
        printf("[%3d] rms=%7.1f peak2peak=%5d\n", i, rms, p2p);
    }
    return 0;
}

static int cmd_pdm_scan(int argc, char **argv)
{
    int ms = (argc >= 2) ? atoi(argv[1]) : 100;
    if (ms < 20) ms = 20;

    /* If a specific CLK is given, restrict scan to that CLK vs all
     * candidate DAT pins. Otherwise brute-force every ordered pair
     * (~PDM_CANDIDATES_N * (PDM_CANDIDATES_N-1) probes). */
    int fixed_clk = (argc >= 3) ? atoi(argv[2]) : -1;

    printf("pdm-scan: %d candidate pins, %d ms per probe\n",
           (int)PDM_CANDIDATES_N, ms);
    printf("      make continuous noise near the mic while this runs;\n");
    printf("      the true (CLK, DAT) pair will show a large peak2peak.\n");

    int probes = 0, hits = 0;
    for (size_t i = 0; i < PDM_CANDIDATES_N; i++) {
        int clk = PDM_CANDIDATES[i];
        if (fixed_clk >= 0 && clk != fixed_clk) continue;

        for (size_t j = 0; j < PDM_CANDIDATES_N; j++) {
            if (i == j) continue;
            int dat = PDM_CANDIDATES[j];

            int n = 0, p2p = 0;
            double rms = 0.0;
            esp_err_t e = pdm_capture(clk, dat, ms, &n, &rms, &p2p);
            probes++;

            if (e != ESP_OK) {
                /* Skip silently — many pin pairs will refuse to init on
                 * strapping / input-only combinations; not a hit signal. */
                continue;
            }
            /* Baseline noise on a floating DAT pin is typically flat
             * (~0 p2p). Anything above a few hundred is worth a look. */
            if (p2p >= 200) {
                printf("  HIT clk=%2d dat=%2d rms=%7.1f peak2peak=%5d\n",
                       clk, dat, rms, p2p);
                hits++;
            }
        }
    }
    printf("pdm-scan done: %d probes, %d hits\n", probes, hits);
    return 0;
}

static void register_commands(void)
{
    const esp_console_cmd_t cmds[] = {
        { .command = "led",       .help = "set LED color: led <r> <g> <b>",             .func = cmd_led },
        { .command = "led-off",   .help = "turn LED off",                               .func = cmd_led_off },
        { .command = "led-pin",   .help = "reconfigure LED pin at runtime: led-pin <gpio>",  .func = cmd_led_pin },
        { .command = "motor",     .help = "DEPRECATED — A1/A2 are I2C now; use tlc-set",.func = cmd_motor },
        { .command = "stop",      .help = "park every TLC59108 channel off",             .func = cmd_motor_stop },
        { .command = "tlc-init",  .help = "(re)initialise TLC59108 (97 kHz fixed)",       .func = cmd_tlc_init },
        { .command = "tlc-diag",  .help = "reset the persistent I2C bus and scan for TLC59108 & neighbours", .func = cmd_tlc_diag },
        { .command = "tlc-speed", .help = "reopen the I2C bus + TLC device at a different SCL clock: tlc-speed <hz>", .func = cmd_tlc_speed },
        { .command = "tlc-addr",  .help = "point the driver at a different I2C address: tlc-addr <hex, e.g. 0x0a>", .func = cmd_tlc_addr },
        { .command = "tlc-set",   .help = "drive one TLC59108 channel: tlc-set <ch 0-7> <sink-pct 0-100>", .func = cmd_tlc_set },
        { .command = "tlc-sweep", .help = "cycle all 8 TLC59108 channels @ 50%% duty to find M1/M2/P1..P4", .func = cmd_tlc_sweep },
        { .command = "servo",     .help = "set servo: servo <deg 0-180>",               .func = cmd_servo },
        { .command = "servo-pin", .help = "reconfigure servo pin at runtime: servo-pin <gpio>", .func = cmd_servo_pin },
        { .command = "selftest",  .help = "run LED + servo + motor smoke test",         .func = cmd_selftest },
        { .command = "pins",        .help = "print current pin assignments",              .func = cmd_pins },
        { .command = "led-sweep",   .help = "cycle LED init across candidate GPIOs (2s per pin)",     .func = cmd_sweep_led },
        { .command = "servo-sweep", .help = "cycle servo signal across candidate GPIOs (2s per pin)", .func = cmd_sweep_servo },
        { .command = "motor-sweep", .help = "drive candidate GPIOs as motor PWM (2s per pin)",        .func = cmd_sweep_motor },
        { .command = "i2c-scan",    .help = "probe every I2C address: i2c-scan [sda scl]  (defaults 1 2)", .func = cmd_i2c_scan },
        { .command = "i2c-hunt",    .help = "sweep 24 SDA/SCL pin pairs and report any hits (~30 s)",     .func = cmd_i2c_hunt },
        { .command = "i2c-selftest",.help = "sanity-check the probe machinery on one pair: i2c-selftest <sda> <scl>", .func = cmd_i2c_selftest },
        { .command = "pin-hunt",    .help = "watch candidate GPIOs and report LOW transitions when you jumper a header pin to GND: pin-hunt [seconds]", .func = cmd_pin_hunt },
        { .command = "adc-hunt",    .help = "watch every ADC-capable GPIO for value swings — for resistor-ladder buttons: adc-hunt [seconds] [threshold]", .func = cmd_adc_hunt },
        { .command = "button",      .help = "read the shared button ADC on GPIO 7: button [seconds]", .func = cmd_button },
        { .command = "read-adc",    .help = "read ADC on any GPIO 1..20: read-adc <gpio> [seconds]", .func = cmd_read_adc },
        { .command = "button-state",.help = "classify GPIO 7 ADC into NONE / L / R / BOTH: button-state [seconds]", .func = cmd_button_state },
        { .command = "i2c-hunt-full", .help = "brute-force every ordered (SDA, SCL) pair of candidate pins (~5 min)", .func = cmd_i2c_hunt_full },
        { .command = "bb-scan",       .help = "bit-banged I2C scan (bypasses ESP-IDF driver): bb-scan [sda scl]", .func = cmd_bb_scan },
        { .command = "bb-tlc-init",   .help = "init TLC59108 via bit-bang (works when tlc-init doesn't)", .func = cmd_bb_tlc_init },
        { .command = "bb-tlc-set",    .help = "drive one TLC59108 channel via bit-bang: bb-tlc-set <ch 0-7> <pct 0-100>", .func = cmd_bb_tlc_set },
        { .command = "bb-tlc-sweep",  .help = "cycle all 8 TLC59108 channels via bit-bang (2 s each) — find which is M1/M2/P1..P4", .func = cmd_bb_tlc_sweep },
        { .command = "pdm-probe",     .help = "test one PDM CLK/DAT pair: pdm-probe <clk> <dat> [ms]",            .func = cmd_pdm_probe },
        { .command = "pdm-watch",     .help = "live PDM RMS every 100 ms: pdm-watch <clk> <dat> [seconds]",       .func = cmd_pdm_watch },
        { .command = "pdm-scan",      .help = "brute-force PDM CLK/DAT pairs across free GPIOs: pdm-scan [ms] [fixed_clk]", .func = cmd_pdm_scan },
    };
    for (size_t i = 0; i < sizeof(cmds) / sizeof(cmds[0]); ++i) {
        ESP_ERROR_CHECK(esp_console_cmd_register(&cmds[i]));
    }
    esp_console_register_help_command();
}

void app_main(void)
{
    ESP_LOGI(TAG, "OpenQuarkyRover M1 smoke-test firmware booting");
    ESP_LOGW(TAG, "LED pin (%d) and servo pin (%d) are PLACEHOLDERS — "
                  "use `pins`, `led-pin`, `servo-pin` at the REPL",
             PIN_LED_WS2812, PIN_SERVO_S1);

    led_init(PIN_LED_WS2812);
    servo_init(PIN_SERVO_S1);
    /* Steering wheels centered at boot so the rover starts pointed
     * straight ahead regardless of the servo's last position. */
    servo_set_deg(90);

    /* A1/A2 are the expansion board's I2C bus, NOT motor PWM pins.
     * motors_init() (LEDC on GPIO 2/1) would corrupt every I2C
     * transaction, so it is intentionally NOT called. All P1..P4 and
     * motor drive goes through the TLC59108 at 0x40 — see tlc-* cmds. */
    if (i2c_bus_open() == ESP_OK) {
        i2c_device_config_t dev_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address  = TLC59108_ADDR,
            .scl_speed_hz    = 100000,
        };
        if (i2c_master_bus_add_device(s_i2c_bus, &dev_cfg, &s_tlc) == ESP_OK) {
            if (tlc59108_init() == ESP_OK) {
                ESP_LOGI(TAG, "TLC59108 @ 0x%02x online (97 kHz fixed, all channels parked off)",
                         TLC59108_ADDR);
            } else {
                ESP_LOGW(TAG, "TLC59108 present but init failed — run `tlc-init` at the REPL");
            }
        } else {
            ESP_LOGW(TAG, "could not add TLC59108 (0x%02x) to I2C bus", TLC59108_ADDR);
        }
    } else {
        ESP_LOGW(TAG, "I2C bus init failed — expansion board control disabled");
    }

    /* Bit-bang I²C TLC59108 init — the working motor-driver path on this
     * board. Without this the web UI's /api/motor endpoint returns 503
     * "motor driver not ready". Failure is non-fatal so the REPL and UI
     * still come up for LED/servo control. */
    if (bb_tlc59108_init_seq(PIN_I2C_SDA, PIN_I2C_SCL)) {
        ESP_LOGI(TAG, "bit-bang TLC59108 online — motors ready");
    } else {
        ESP_LOGW(TAG, "bit-bang TLC59108 init failed — motors disabled until `bb-tlc-init` at REPL");
    }

    /* Boot indicator — green if LED pin happens to be right */
    led_set(0, 32, 0);

    /* Camera bring-up before HTTP so /stream has a live sensor to pull
     * from. Non-fatal: if pins are wrong the UI still loads and the
     * <img src="/stream"> falls back to an "unavailable" banner. */
    (void)camera_start();

    /* Wi-Fi soft-AP + HTTP control UI. Both are non-fatal; the REPL is
     * the fallback if Wi-Fi doesn't come up. */
    if (rover_wifi_ap_start() == ESP_OK) {
        (void)http_control_start();
    }

    esp_console_repl_t              *repl        = NULL;
    esp_console_repl_config_t        repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    repl_config.prompt                            = "quarky> ";
    repl_config.max_cmdline_length                = 256;
    repl_config.task_stack_size                   = 8192;

    esp_console_dev_uart_config_t uart_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_uart(&uart_config, &repl_config, &repl));

    register_commands();
    ESP_ERROR_CHECK(esp_console_start_repl(repl));

    ESP_LOGI(TAG, "REPL ready — try: help, pins, selftest");
    /* app_main returns; the REPL keeps running in its own task. */
}
