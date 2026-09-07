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
#include "i2c_bitbang.h"
#include "es8311.h"
#include "audio_capture.h"
#include "command_bus.h"
#include "led_indicator.h"
#include "freertos/queue.h"
#include "esp_heap_caps.h"

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

/* Non-static + int return so led_indicator.c can extern-call this from
 * the tick task. Returns 1 on success (led_strip driver invoked) or 0
 * if the strip handle isn't initialised (LED disabled / early boot). */
int led_set(uint8_t r, uint8_t g, uint8_t b)
{
    if (!s_led) return 0;
    led_strip_set_pixel(s_led, 0, r, g, b);
    led_strip_refresh(s_led);
    return 1;
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
    /* s_tlc_ready stays false — caller runs bb-tlc-init to re-configure. */
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

/* Legacy LEDC-on-A1/A2 motor path (motors_init / motor_set) removed —
 * A1/A2 are actually the expansion-board I²C bus; motor drive goes via
 * TLC59108 through bb_motor_set. See pins.h comment on PIN_MOTOR_*.  */

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

static void servo_set_deg(int deg)
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
    /* Route through the bus so led_indicator's arbitration owns the pixel —
     * calling led_set directly races the FSM tick and gets overwritten on
     * the next 30 ms tick, defeating the point of a set. */
    command_t c = { .id = CMD_LED_RGB, .source = SRC_REPL,
                    .as.led_rgb = { .r = (uint8_t)r, .g = (uint8_t)g, .b = (uint8_t)b } };
    esp_err_t rc = command_bus_publish(&c);
    if (rc != ESP_OK) {
        printf("led: publish failed: %s\n", esp_err_to_name(rc));
        return 1;
    }
    printf("led (GPIO %d) = (%d,%d,%d) via bus\n", s_led_pin, r, g, b);
    return 0;
}

static int cmd_led_off(int argc, char **argv)
{
    (void)argc; (void)argv;
    command_t c = { .id = CMD_LED_RGB, .source = SRC_REPL,
                    .as.led_rgb = { 0, 0, 0 } };
    esp_err_t rc = command_bus_publish(&c);
    if (rc != ESP_OK) {
        printf("led-off: publish failed: %s\n", esp_err_to_name(rc));
        return 1;
    }
    printf("led off (via bus)\n");
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
/* Non-static + `int` return so command_bus.c can extern-call this from the
 * dispatcher. Return values keep the old bool semantics (1 = success,
 * 0 = failure) so REPL callers of the form `if (!bb_motor_set(...))` still
 * work unchanged. Task 7 revisits this if we want an ESP-IDF-style
 * error code out of it. */
int bb_motor_set(int m1_signed, int m2_signed);
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
    if (L < -100) L = -100;
    if (L >  100) L =  100;
    if (R < -100) R = -100;
    if (R >  100) R =  100;
    command_t c = { .id = CMD_MOTOR, .source = SRC_REPL,
                    .as.motor = { .left = (int8_t)L, .right = (int8_t)R } };
    esp_err_t rc = command_bus_publish(&c);
    if (rc != ESP_OK) { printf("motor: publish failed: %s\n", esp_err_to_name(rc)); return 1; }
    printf("motor: L=%d%% R=%d%% via bus\n", L, R);
    return 0;
}

static int cmd_motor_stop(int argc, char **argv)
{
    (void)argc; (void)argv;
    if (!s_bb_ready) {
        printf("TLC59108 not initialised — run `bb-tlc-init` first\n");
        return 1;
    }
    command_t c = { .id = CMD_STOP, .source = SRC_REPL };
    esp_err_t rc = command_bus_publish(&c);
    if (rc != ESP_OK) { printf("stop: publish failed: %s\n", esp_err_to_name(rc)); return 1; }
    printf("stopped (both motors in brake) via bus\n");
    return 0;
}

static int cmd_servo(int argc, char **argv)
{
    if (argc < 2) { printf("usage: servo <deg 0-180>\n"); return 1; }
    int d = atoi(argv[1]);
    if (d < 0)   d = 0;
    if (d > 180) d = 180;
    /* Degrees → pulse-width (µs). Bus dispatcher routes to servo_set_us,
     * which does the LEDC duty write. Same math as the local servo_set_deg
     * kept below for now (called by cmd_selftest / cmd_sweep_servo). */
    uint16_t us = (uint16_t)(SERVO_MIN_US +
                             (SERVO_MAX_US - SERVO_MIN_US) * d / 180);
    command_t c = { .id = CMD_SERVO, .source = SRC_REPL,
                    .as.servo = { .channel = 1, .us = us } };
    esp_err_t rc = command_bus_publish(&c);
    if (rc != ESP_OK) { printf("servo: publish failed: %s\n", esp_err_to_name(rc)); return 1; }
    printf("servo (GPIO %d) = %d deg → %u µs via bus\n", s_servo_pin, d, (unsigned)us);
    return 0;
}

/* Non-static + int return so command_bus.c's WEAK stub is overridden.
 * Channel is currently unused (this board has one servo on LEDC_CHANNEL_2);
 * kept in the signature for M4 multi-servo. */
int servo_set_us(uint8_t channel, uint16_t us)
{
    (void)channel;
    if (us < SERVO_MIN_US) us = SERVO_MIN_US;
    if (us > SERVO_MAX_US) us = SERVO_MAX_US;
    const uint32_t max_duty = (1U << SERVO_PWM_RES_BITS) - 1;
    const uint32_t duty     = (uint32_t)us * max_duty / 20000;   /* 50 Hz → 20 ms period */
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
    return 1;
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

static int cmd_tlc_set(int argc, char **argv)
{
    if (argc < 3) { printf("usage: tlc-set <ch 0-7> <sink-pct 0-100>\n"
                           "       (sink-pct is the fraction of time the open-drain sink is ON,\n"
                           "        i.e. the fraction of time the output is pulled LOW —\n"
                           "        polarity is INVERTED relative to the DRV8833's input logic)\n"); return 1; }
    if (!s_tlc_ready) { printf("run `bb-tlc-init` first\n"); return 1; }
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

    if (!s_tlc_ready) { printf("run `bb-tlc-init` first\n"); return 1; }

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

/* Bit-bang I²C transport lives in i2c_bitbang.{c,h}. Include is at the top
 * of the file alongside the other project headers; nothing here otherwise. */

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

/* Non-static + int return so command_bus.c's WEAK stub is overridden.
 * Percent → raw sink duty conversion + fixed bus pins so the dispatcher
 * (and voice, later) can drive one channel without duplicating the
 * TLC59108 setup dance. */
int bb_tlc_set_pct(uint8_t channel, uint8_t percent)
{
    if (channel > 7) return 0;
    if (percent > 100) percent = 100;
    uint8_t pwm = (uint8_t)((uint32_t)percent * 255 / 100);
    return bb_tlc_set_channel(PIN_I2C_SDA, PIN_I2C_SCL, channel, pwm) ? 1 : 0;
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
    command_t c = { .id = CMD_BB_TLC_SET, .source = SRC_REPL,
                    .as.bb_tlc = { .channel = (uint8_t)ch, .percent = (uint8_t)pct } };
    esp_err_t rc = command_bus_publish(&c);
    if (rc != ESP_OK) { printf("bb-tlc-set: publish failed: %s\n", esp_err_to_name(rc)); return 1; }
    printf("bb-tlc-set: ch %d = %d%% via bus\n", ch, pct);
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

int bb_motor_set(int m1_signed, int m2_signed)
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
                      TLC59108_PWM0 + MOTOR_M2_PLUS,  m2_pwm)) return 0;
    if (!bb_write_reg(sda, scl, TLC59108_ADDR,
                      TLC59108_PWM0 + MOTOR_M2_MINUS, m2_pwm)) return 0;
    if (!bb_write_reg(sda, scl, TLC59108_ADDR,
                      TLC59108_PWM0 + MOTOR_M1_PLUS,  m1_pwm)) return 0;
    if (!bb_write_reg(sda, scl, TLC59108_ADDR,
                      TLC59108_PWM0 + MOTOR_M1_MINUS, m1_pwm)) return 0;

    return bb_write_reg(sda, scl, TLC59108_ADDR, TLC59108_LEDOUT0,
                        ledout0_for_motors(m1_signed, m2_signed)) ? 1 : 0;
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

/* --- ES8311 codec diagnostic (M3 Task 1, permanent) -----------------------
 *
 * Drives a 4 MHz square on the ES8311 MCLK pin via LEDC(TIMER_2, CHANNEL_3)
 * and reads product-ID register 0xFD (expect 0x83) and version register
 * 0xFE via bit-bang I²C on the confirmed control pins. LEDC channel 3 is
 * dedicated to codec MCLK — channels 0/1 belong to the (deprecated)
 * direct-motor LEDC path, channel 2 is the servo. That isolation means
 * running `es-verify` while the servo is active does not disturb it.
 *
 * See pins.h for the pin-discovery provenance (SDA/SCL empirical via
 * throwaway `es-scan`; MCLK/BCLK/LRCK/DIN from ESP-ADF Korvo-2 v3 board
 * profile, matched to this hardware by the stock-firmware strings dump).
 */
static int cmd_es_verify(int argc, char **argv)
{
    (void)argc; (void)argv;

    ledc_timer_config_t timer_cfg = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_1_BIT,
        .timer_num       = LEDC_TIMER_2,
        .freq_hz         = 4096000,   /* 4.096 MHz = 16000 * 256 (mclk_multiple), matches ES8311 config */
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    if (ledc_timer_config(&timer_cfg) != ESP_OK) {
        printf("es-verify: LEDC timer_config failed\n");
        return 1;
    }
    ledc_channel_config_t ch_cfg = {
        .gpio_num   = ES8311_I2S_MCLK,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = LEDC_CHANNEL_3,
        .timer_sel  = LEDC_TIMER_2,
        .duty       = 1,
        .hpoint     = 0,
    };
    if (ledc_channel_config(&ch_cfg) != ESP_OK) {
        printf("es-verify: LEDC channel_config on GPIO%d failed\n", ES8311_I2S_MCLK);
        return 1;
    }

    bb_init(ES8311_I2C_SDA, ES8311_I2C_SCL);

    uint8_t id = 0, ver = 0;
    esp_err_t rr = es8311_read_id(&id, &ver);

    ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, 0);

    if (rr != ESP_OK) {
        printf("es-verify: es8311_read_id failed: %s\n", esp_err_to_name(rr));
        return 1;
    }
    printf("es-verify: ES8311 id=0x%02x (expect 0x83), version=0x%02x on "
           "MCLK=GPIO%d SDA=GPIO%d SCL=GPIO%d\n",
           id, ver, ES8311_I2S_MCLK, ES8311_I2C_SDA, ES8311_I2C_SCL);
    return (id == 0x83) ? 0 : 1;
}

/* Full codec init: drives MCLK, then runs the ES8311 register recipe
 * (reset, clock config for 16 kHz mono ADC path, mic gain 0 dB, ADC
 * power-up). Prerequisite for `voice-record` / `voice-start` in later
 * tasks. Leaves MCLK running (must be driven for the codec to stay
 * responsive after init). */
static int cmd_es_init(int argc, char **argv)
{
    (void)argc; (void)argv;

    ledc_timer_config_t timer_cfg = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_1_BIT,
        .timer_num       = LEDC_TIMER_2,
        .freq_hz         = 4096000,   /* 4.096 MHz = 16000 * 256 (mclk_multiple), matches ES8311 config */
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    if (ledc_timer_config(&timer_cfg) != ESP_OK) {
        printf("es-init: LEDC timer_config failed\n");
        return 1;
    }
    ledc_channel_config_t ch_cfg = {
        .gpio_num   = ES8311_I2S_MCLK,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = LEDC_CHANNEL_3,
        .timer_sel  = LEDC_TIMER_2,
        .duty       = 1,
        .hpoint     = 0,
    };
    if (ledc_channel_config(&ch_cfg) != ESP_OK) {
        printf("es-init: LEDC channel_config on GPIO%d failed\n", ES8311_I2S_MCLK);
        return 1;
    }

    bb_init(ES8311_I2C_SDA, ES8311_I2C_SCL);

    esp_err_t r = es8311_init();
    printf("es-init: es8311_init -> %s (MCLK GPIO%d still running)\n",
           esp_err_to_name(r), ES8311_I2S_MCLK);
    return (r == ESP_OK) ? 0 : 1;
}

/* --- voice-record: capture N seconds of PCM, dump as hex over UART ------
 *
 * REPL flow: audio_capture_start (I²S RX drives MCLK on GPIO 16) →
 * es8311_init (codec sees the I²S-provided MCLK) → drain the queue into
 * a PSRAM buffer for the whole capture window → stop → hex-dump the
 * buffer between BEGIN/END markers.
 *
 * Why hex and not real-time raw: UART is 115200 baud (~11.5 kB/s),
 * raw PCM is 32 kB/s — real-time streaming drops ~2/3 of samples. The
 * PSRAM-buffer + slow-dump path loses zero samples; the dump takes
 * about 6× the capture duration but the audio is intact.
 *
 * Extract on host from monitor.sh's ANSI-stripped log:
 *
 *   sed -n '/voice-record BEGIN/,/voice-record END/p' logs/latest.clean.log \
 *     | grep -Ex '[0-9a-f]{64}' | xxd -r -p > /tmp/mic.pcm
 *   sox -t raw -r 16000 -e signed -b 16 -c 1 /tmp/mic.pcm /tmp/mic.wav
 *   afplay /tmp/mic.wav
 */
/* Bootstrap for standalone ES8311 REPL commands: LEDC drives MCLK on
 * GPIO 16 (needed for the codec to ACK I²C) and bring up the bit-bang
 * I²C bus on the codec's SDA/SCL. Idempotent — safe to call more than
 * once; ledc_timer_config / ledc_channel_config just re-apply. */
static void es_repl_bootstrap(void)
{
    ledc_timer_config_t mclk_timer = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_1_BIT,
        .timer_num       = LEDC_TIMER_2,
        .freq_hz         = 4096000,   /* 4.096 MHz = 16000 * 256 (mclk_multiple), matches ES8311 config */
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    (void)ledc_timer_config(&mclk_timer);
    ledc_channel_config_t mclk_ch = {
        .gpio_num   = ES8311_I2S_MCLK,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = LEDC_CHANNEL_3,
        .timer_sel  = LEDC_TIMER_2,
        .duty       = 1,
        .hpoint     = 0,
    };
    (void)ledc_channel_config(&mclk_ch);
    bb_init(ES8311_I2C_SDA, ES8311_I2C_SCL);
}

static int cmd_es_peek(int argc, char **argv)
{
    if (argc != 2) { printf("usage: es-peek <reg_hex>\n"); return 1; }
    unsigned reg = (unsigned)strtoul(argv[1], NULL, 16);
    if (reg > 0xFF) { printf("reg out of range\n"); return 1; }
    es_repl_bootstrap();
    uint8_t v = 0;
    if (!bb_read_reg(ES8311_I2C_SDA, ES8311_I2C_SCL,
                     ES8311_I2C_ADDR, (uint8_t)reg, &v)) {
        printf("es-peek: I2C read failed (reg 0x%02X)\n", (unsigned)reg);
        return 1;
    }
    printf("REG 0x%02X = 0x%02X\n", (unsigned)reg, v);
    return 0;
}

static int cmd_es_poke(int argc, char **argv)
{
    if (argc != 3) { printf("usage: es-poke <reg_hex> <val_hex>\n"); return 1; }
    unsigned reg = (unsigned)strtoul(argv[1], NULL, 16);
    unsigned val = (unsigned)strtoul(argv[2], NULL, 16);
    if (reg > 0xFF || val > 0xFF) { printf("reg or val out of range\n"); return 1; }
    es_repl_bootstrap();
    if (!bb_write_reg(ES8311_I2C_SDA, ES8311_I2C_SCL,
                      ES8311_I2C_ADDR, (uint8_t)reg, (uint8_t)val)) {
        printf("es-poke: I2C write failed (reg 0x%02X = 0x%02X)\n",
               (unsigned)reg, (unsigned)val);
        return 1;
    }
    /* Read-back verification. Some registers are R/O or self-updating,
     * so a mismatch here isn't necessarily an error — just report. */
    uint8_t rb = 0;
    if (bb_read_reg(ES8311_I2C_SDA, ES8311_I2C_SCL,
                    ES8311_I2C_ADDR, (uint8_t)reg, &rb)) {
        printf("REG 0x%02X <- 0x%02X (readback 0x%02X)\n",
               (unsigned)reg, (unsigned)val, rb);
    } else {
        printf("REG 0x%02X <- 0x%02X (readback failed)\n",
               (unsigned)reg, (unsigned)val);
    }
    return 0;
}

static int cmd_es_dump(int argc, char **argv)
{
    (void)argc; (void)argv;
    es_repl_bootstrap();
    return es8311_dump() == ESP_OK ? 0 : 1;
}

static int cmd_voice_record(int argc, char **argv)
{
    if (argc < 2 || argc > 3) {
        printf("usage: voice-record <sec 1..10> [slot=L|R|B]  (default: R; B=both/stereo diag)\n");
        return 1;
    }
    int sec = atoi(argv[1]);
    if (sec < 1 || sec > 10) { printf("range: 1..10\n"); return 1; }

    audio_capture_slot_t slot = AUDIO_CAPTURE_SLOT_RIGHT;
    if (argc == 3) {
        if      (argv[2][0] == 'L' || argv[2][0] == 'l') slot = AUDIO_CAPTURE_SLOT_LEFT;
        else if (argv[2][0] == 'R' || argv[2][0] == 'r') slot = AUDIO_CAPTURE_SLOT_RIGHT;
        else if (argv[2][0] == 'B' || argv[2][0] == 'b') slot = AUDIO_CAPTURE_SLOT_BOTH;
        else { printf("slot must be L, R, or B\n"); return 1; }
    }

    const size_t frames_total = ((size_t)sec * 16000) / AUDIO_CAPTURE_FRAME_SAMPLES;
    const size_t bytes_total  = frames_total * AUDIO_CAPTURE_FRAME_BYTES;

    /* 96 KB for 3 s fits comfortably in internal SRAM. Once Task 8 enables
     * PSRAM (CONFIG_SPIRAM=y for the ESP-SR models), this can grow to any
     * ceiling the user wants without touching the SRAM heap. */
    int16_t *buf = malloc(bytes_total);
    if (!buf) { printf("voice-record: alloc failed (%zu B)\n", bytes_total); return 1; }

    QueueHandle_t q = xQueueCreate(4, AUDIO_CAPTURE_FRAME_BYTES);
    if (!q) { free(buf); printf("voice-record: queue alloc failed\n"); return 1; }

    /* Order matters — this is the reverse of what the previous WIP did:
     *
     *   1. Start I²S RX FIRST so MCLK/BCLK/LRCK are all coming from the
     *      same I²S peripheral clock tree (matches stock, which drives
     *      MCLK via OUT_SEL[GPIO 45] = 23 = I2S0_MCLK_OUT). Without this
     *      the codec's PLL sees MCLK from LEDC and BCLK from I²S — two
     *      independent clock domains, no phase lock, and its REG 0x0D
     *      "clocks detected" bit refuses to leave 0x01 → SDPOUT stays
     *      digital zero regardless of everything else being correct.
     *
     *   2. Init the codec AFTER I²S is running so the ADC's decimator
     *      configures itself against a stable, phase-locked MCLK.
     *
     *   3. Drain the pre-init frames from the queue (all zeros because
     *      the codec hadn't started driving SDPOUT yet) before capture. */
    audio_capture_config_t cfg = { .slot = slot };
    esp_err_t r = audio_capture_start_ex(q, &cfg);
    if (r != ESP_OK) {
        printf("voice-record: audio_capture_start -> %s\n", esp_err_to_name(r));
        vQueueDelete(q); free(buf); return 1;
    }
    /* Give the I²S peripheral a few ms to lock and start producing
     * MCLK/BCLK/LRCK before the codec sees them. */
    vTaskDelay(pdMS_TO_TICKS(20));

    bb_init(ES8311_I2C_SDA, ES8311_I2C_SCL);
    esp_err_t codec_r = es8311_init();
    printf("voice-record: es8311_init -> %s\n", esp_err_to_name(codec_r));
    /* Codec should now report REG 0x0D = 0x02 (clocks detected) after a
     * short PLL-lock delay. If it's still 0x01, the phase-lock story is
     * still wrong. */
    vTaskDelay(pdMS_TO_TICKS(80));
    printf("voice-record: ES8311 state AFTER es8311_init + I²S running:\n");
    es8311_dump();
    /* Digital MEMS mic settles within a few LRCK edges. Drop the first
     * few frames (128 ms) — INMP441-family mics need ~50 ms after WS
     * starts before valid samples appear. */
    int16_t discard[AUDIO_CAPTURE_FRAME_SAMPLES];
    for (int i = 0; i < 4; i++) {
        (void)xQueueReceive(q, discard, pdMS_TO_TICKS(200));
    }

    printf("voice-record: capturing %d s (%zu frames, %zu bytes)...\n",
           sec, frames_total, bytes_total);

    size_t got = 0;
    for (size_t i = 0; i < frames_total; i++) {
        if (xQueueReceive(q, &buf[i * AUDIO_CAPTURE_FRAME_SAMPLES], pdMS_TO_TICKS(500)) != pdTRUE) break;
        got++;
    }
    audio_capture_stop();
    vQueueDelete(q);

    uint32_t dropped = audio_capture_dropped_frames();
    /* Signal stats: min, max, mean absolute value, and count of non-zero
     * samples. All zeros means the ADC path (or DIN pin) is wrong; a
     * tiny non-zero range means the mic is quiet but wired right. */
    int16_t smin = INT16_MAX, smax = INT16_MIN;
    int64_t sabs = 0;
    size_t  n_nz = 0;
    size_t  total_samples = got * AUDIO_CAPTURE_FRAME_SAMPLES;
    for (size_t i = 0; i < total_samples; i++) {
        int16_t s = buf[i];
        if (s < smin) smin = s;
        if (s > smax) smax = s;
        sabs += (s < 0) ? -s : s;
        if (s) n_nz++;
    }
    int mean_abs = total_samples ? (int)(sabs / (int64_t)total_samples) : 0;
    printf("voice-record: captured %zu/%zu frames, dropped=%lu, "
           "min=%d max=%d mean|s|=%d nonzero=%zu/%zu\n",
           got, frames_total, (unsigned long)dropped,
           smin, smax, mean_abs, n_nz, total_samples);

    /* Hex dump. 32 bytes per line → 64 hex chars, exactly greppable. */
    printf("--- voice-record BEGIN sec=%d samples=%zu bytes=%zu ---\n",
           sec, got * AUDIO_CAPTURE_FRAME_SAMPLES, got * AUDIO_CAPTURE_FRAME_BYTES);
    const uint8_t *p = (const uint8_t *)buf;
    size_t n = got * AUDIO_CAPTURE_FRAME_BYTES;
    for (size_t i = 0; i < n; i += 32) {
        size_t chunk = (n - i < 32) ? (n - i) : 32;
        for (size_t j = 0; j < chunk; j++) printf("%02x", p[i + j]);
        printf("\n");
        /* Feed the UART TX FIFO some slack so this doesn't wedge log output. */
        if ((i & 0x3FF) == 0) vTaskDelay(1);
    }
    printf("--- voice-record END ---\n");

    free(buf);
    return 0;
}

/* voice-scan: sweep I²S DIN candidate pins × slot (LEFT/RIGHT) and report
 * max|sample| for each. First combo with a non-trivial max wins the DIN
 * pin question. Runs es8311_init once (MCLK is briefly gapped between
 * iterations when I²S is torn down / re-set-up, but the codec re-syncs
 * on the next MCLK cycle without needing a full re-init).
 *
 * Runtime: ~22 candidate pins × 2 slots × ~400 ms ≈ 18 s. */
static const int VOICE_DIN_CAND[] = {
    4, 5, 6, 8, 10, 11, 12, 13, 14, 15, 21,
    33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 47
};
static const size_t VOICE_DIN_CAND_COUNT =
    sizeof(VOICE_DIN_CAND) / sizeof(VOICE_DIN_CAND[0]);

static int cmd_voice_scan(int argc, char **argv)
{
    (void)argc; (void)argv;

    /* No codec init needed — the mic is a digital MEMS on its own I²S bus.
     * Sweep DIN pin × slot and report max|sample|; the answer is expected
     * to be DIN=GPIO 42, slot LEFT (INMP441-family default). Kept as a
     * diagnostic so future board revisions that move the mic can be
     * re-discovered without a rebuild. */
    QueueHandle_t q = xQueueCreate(4, AUDIO_CAPTURE_FRAME_BYTES);
    if (!q) { printf("voice-scan: queue alloc failed\n"); return 1; }

    printf("--- voice-scan: %zu DIN candidates × 2 slots (BCLK=%d WS=%d) ---\n",
           VOICE_DIN_CAND_COUNT, MIC_I2S_SCK, MIC_I2S_WS);
    int best_max = 0;
    int best_din = -1;
    audio_capture_slot_t best_slot = AUDIO_CAPTURE_SLOT_LEFT;

    for (int slot_i = 0; slot_i < 2; slot_i++) {
        audio_capture_slot_t slot = (slot_i == 0)
            ? AUDIO_CAPTURE_SLOT_LEFT : AUDIO_CAPTURE_SLOT_RIGHT;
        const char *slot_s = (slot == AUDIO_CAPTURE_SLOT_LEFT) ? "L" : "R";

        for (size_t k = 0; k < VOICE_DIN_CAND_COUNT; k++) {
            int din = VOICE_DIN_CAND[k];
            /* Skip pins already claimed by the mic I²S clocks. */
            if (din == MIC_I2S_SCK || din == MIC_I2S_WS) continue;

            audio_capture_config_t cfg = { .din_gpio = din, .slot = slot };
            if (audio_capture_start_ex(q, &cfg) != ESP_OK) continue;

            /* Drain queue first (any stale frames) then read 8 fresh. */
            int16_t frame[AUDIO_CAPTURE_FRAME_SAMPLES];
            while (xQueueReceive(q, frame, 0) == pdTRUE) { }

            int mx = 0;
            for (int f = 0; f < 8; f++) {
                if (xQueueReceive(q, frame, pdMS_TO_TICKS(200)) != pdTRUE) break;
                for (size_t j = 0; j < AUDIO_CAPTURE_FRAME_SAMPLES; j++) {
                    int a = frame[j] < 0 ? -frame[j] : frame[j];
                    if (a > mx) mx = a;
                }
            }
            audio_capture_stop();
            printf("  DIN=GPIO%-2d slot=%s max|s|=%d\n", din, slot_s, mx);
            if (mx > best_max) { best_max = mx; best_din = din; best_slot = slot; }
        }
    }

    vQueueDelete(q);

    if (best_max == 0) {
        printf("--- voice-scan: no candidate produced non-zero samples.\n");
        printf("    ADC path is likely muted at the codec — not a DIN pin issue.\n");
        return 1;
    }
    printf("--- voice-scan: best DIN=GPIO%d slot=%s max|s|=%d\n",
           best_din, best_slot == AUDIO_CAPTURE_SLOT_LEFT ? "L" : "R", best_max);
    return 0;
}

/* mic-perm: try all 6 orderings of GPIOs 40/41/42 as (BCLK, WS, DIN) plus
 * LEFT/RIGHT slot. Only the correct mapping gets the mic clocked properly,
 * so exactly one row should report a non-trivial max|sample|. Useful when
 * the vendor's pin-label naming might have swapped roles. */
static int cmd_mic_perm(int argc, char **argv)
{
    (void)argc; (void)argv;

    QueueHandle_t q = xQueueCreate(4, AUDIO_CAPTURE_FRAME_BYTES);
    if (!q) { printf("mic-perm: queue alloc failed\n"); return 1; }

    const int pins[3] = { MIC_I2S_SCK, MIC_I2S_WS, MIC_I2S_SD };
    /* 3! permutation table */
    const int perms[6][3] = {
        {0,1,2}, {0,2,1}, {1,0,2}, {1,2,0}, {2,0,1}, {2,1,0}
    };
    int best_max = 0;
    int best_p   = -1;
    audio_capture_slot_t best_slot = AUDIO_CAPTURE_SLOT_LEFT;

    printf("--- mic-perm: 6 pin permutations of {GPIO%d, GPIO%d, GPIO%d} × 2 slots ---\n",
           pins[0], pins[1], pins[2]);

    for (int slot_i = 0; slot_i < 2; slot_i++) {
        audio_capture_slot_t slot = (slot_i == 0)
            ? AUDIO_CAPTURE_SLOT_LEFT : AUDIO_CAPTURE_SLOT_RIGHT;
        const char *slot_s = (slot == AUDIO_CAPTURE_SLOT_LEFT) ? "L" : "R";

        for (int p = 0; p < 6; p++) {
            int bclk = pins[perms[p][0]];
            int ws   = pins[perms[p][1]];
            int din  = pins[perms[p][2]];
            audio_capture_config_t cfg = {
                .bclk_gpio = bclk, .ws_gpio = ws, .din_gpio = din, .slot = slot,
            };
            if (audio_capture_start_ex(q, &cfg) != ESP_OK) continue;

            int16_t frame[AUDIO_CAPTURE_FRAME_SAMPLES];
            while (xQueueReceive(q, frame, 0) == pdTRUE) { }

            int mx = 0;
            int nz = 0;
            int seen_neg1 = 0;  /* count of exactly 0xFFFF samples */
            for (int f = 0; f < 12; f++) {
                if (xQueueReceive(q, frame, pdMS_TO_TICKS(200)) != pdTRUE) break;
                for (size_t j = 0; j < AUDIO_CAPTURE_FRAME_SAMPLES; j++) {
                    int a = frame[j] < 0 ? -frame[j] : frame[j];
                    if (a > mx) mx = a;
                    if (frame[j]) nz++;
                    if ((uint16_t)frame[j] == 0xFFFF) seen_neg1++;
                }
            }
            audio_capture_stop();
            printf("  BCLK=%-2d WS=%-2d DIN=%-2d slot=%s  max|s|=%-5d  nz=%-5d  neg1=%-5d\n",
                   bclk, ws, din, slot_s, mx, nz, seen_neg1);
            /* Score by max|s|, ignoring the "all -1s" degenerate case. */
            if (mx > best_max && seen_neg1 < 100) {
                best_max = mx; best_p = p; best_slot = slot;
            }
        }
    }
    vQueueDelete(q);

    if (best_p < 0) {
        printf("--- mic-perm: no permutation produced real signal.\n");
        printf("    Mic likely needs a power-enable GPIO (or L/R pin driven).\n");
        printf("    Check the schematic for a MIC_EN / AUDIO_EN line.\n");
        return 1;
    }
    printf("--- mic-perm: best BCLK=%d WS=%d DIN=%d slot=%s max|s|=%d\n",
           pins[perms[best_p][0]], pins[perms[best_p][1]], pins[perms[best_p][2]],
           best_slot == AUDIO_CAPTURE_SLOT_LEFT ? "L" : "R", best_max);
    return 0;
}

/* mic-enable-scan: sweep unused GPIOs, drive each HIGH (then LOW), and
 * check whether the mic starts producing non-degenerate samples. Answers
 * the "which pin powers or ungates the mic" question that neither the
 * strings dump nor the codec-init hypothesis could nail down.
 *
 * For each candidate:
 *   1. GPIO N → OUTPUT, HIGH
 *   2. wait 200 ms for a MOSFET / LDO to settle
 *   3. start I²S RX (fixed MIC_I2S_* pins), drain, read 8 frames
 *   4. compute max|s| and count of 0xFFFF-only samples
 *   5. reset GPIO N to INPUT (release)
 *
 * Winner: the pin that produces max|s| >> 1 with neg1 < 100 (i.e. the
 * mic is actually driving data, not the empty-slot pull-up). ~700 ms
 * per pin × ~25 candidates ≈ 18 s. */
static const int MIC_ENABLE_CAND[] = {
    3, 4, 5, 6, 8, 10, 11, 12, 13, 14, 15, 21,
    33, 34, 35, 36, 37, 38, 39, 46, 47
};
static const size_t MIC_ENABLE_CAND_COUNT =
    sizeof(MIC_ENABLE_CAND) / sizeof(MIC_ENABLE_CAND[0]);

static int cmd_mic_enable_scan(int argc, char **argv)
{
    /* Optional level arg: `mic-enable-scan 0` drives each candidate LOW
     * (active-low enable); default is HIGH. Handy after a HIGH-polarity
     * scan finds nothing. */
    int level = (argc >= 2 && atoi(argv[1]) == 0) ? 0 : 1;

    QueueHandle_t q = xQueueCreate(4, AUDIO_CAPTURE_FRAME_BYTES);
    if (!q) { printf("mic-enable-scan: queue alloc failed\n"); return 1; }

    /* Codec has to be up (init'd + LEDC MCLK running) or the ADC output
     * stays at bit-zero regardless of any enable pin. Bootstrap it once
     * up-front so every iteration starts from the same known state. */
    es_repl_bootstrap();
    (void)es8311_init();

    printf("--- mic-enable-scan: try driving each of %zu candidate GPIOs %s ---\n",
           MIC_ENABLE_CAND_COUNT, level ? "HIGH" : "LOW");
    printf("    ES8311 path (BCLK=%d WS=%d DIN=%d slot=R). Non-zero max|s| = hit.\n",
           ES8311_I2S_BCLK, ES8311_I2S_LRCK, ES8311_I2S_DIN);

    int best_max = 0;
    int best_pin = -1;
    int best_neg1 = 0;

    for (size_t k = 0; k < MIC_ENABLE_CAND_COUNT; k++) {
        int pin = MIC_ENABLE_CAND[k];
        /* Skip pins we know are the codec's own I²C / I²S — driving
         * those as an output would kill the mic path we're testing. */
        if (pin == ES8311_I2C_SDA || pin == ES8311_I2C_SCL ||
            pin == ES8311_I2S_MCLK || pin == ES8311_I2S_BCLK ||
            pin == ES8311_I2S_LRCK || pin == ES8311_I2S_DIN) {
            continue;
        }
        gpio_reset_pin(pin);
        gpio_set_direction(pin, GPIO_MODE_OUTPUT);
        gpio_set_level(pin, level);
        vTaskDelay(pdMS_TO_TICKS(200));

        audio_capture_config_t cfg = { .slot = AUDIO_CAPTURE_SLOT_RIGHT };
        if (audio_capture_start_ex(q, &cfg) != ESP_OK) {
            gpio_set_level(pin, 0);
            gpio_set_direction(pin, GPIO_MODE_INPUT);
            continue;
        }
        int16_t frame[AUDIO_CAPTURE_FRAME_SAMPLES];
        while (xQueueReceive(q, frame, 0) == pdTRUE) { }

        int mx = 0, neg1 = 0;
        for (int f = 0; f < 8; f++) {
            if (xQueueReceive(q, frame, pdMS_TO_TICKS(200)) != pdTRUE) break;
            for (size_t j = 0; j < AUDIO_CAPTURE_FRAME_SAMPLES; j++) {
                int a = frame[j] < 0 ? -frame[j] : frame[j];
                if (a > mx) mx = a;
                if ((uint16_t)frame[j] == 0xFFFF) neg1++;
            }
        }
        audio_capture_stop();

        /* Release the pin so the next iteration starts clean. */
        gpio_set_level(pin, 0);
        gpio_set_direction(pin, GPIO_MODE_INPUT);

        printf("  GPIO%-2d %s → max|s|=%-5d neg1=%d\n",
               pin, level ? "HIGH" : "LOW ", mx, neg1);

        if (mx > best_max && neg1 < 100) {
            best_max = mx; best_pin = pin; best_neg1 = neg1;
        }
    }
    vQueueDelete(q);

    if (best_pin < 0) {
        printf("--- mic-enable-scan: no GPIO woke the mic.\n");
        printf("    Try polarity flip (some enables are active-LOW), or check\n");
        printf("    the schematic — the enable may be behind a codec GPO,\n");
        printf("    a shift-register, or an I²C GPIO expander.\n");
        return 1;
    }
    printf("--- mic-enable-scan: best GPIO%d HIGH → max|s|=%d, neg1=%d\n",
           best_pin, best_max, best_neg1);
    printf("    Set this pin HIGH before every capture (via bb_init / gpio_set_level).\n");
    return 0;
}

static int cmd_voice_stats(int argc, char **argv)
{
    (void)argc; (void)argv;
    printf("voice-stats: audio_capture dropped_frames=%lu\n",
           (unsigned long)audio_capture_dropped_frames());
    return 0;
}

static int cmd_audio_diag(int argc, char **argv)
{
    (void)argc; (void)argv;
    audio_capture_diag_print();
    return 0;
}

/* Publish a synthetic voice command through the bus (bypasses the audio
 * stack). Verifies the whole dispatcher path lands on the motor helper
 * before Task 10 wires MultiNet-EN into the bus for real. Same command
 * IDs as the eventual voice vocabulary: 1=forward 2=backward 3=left
 * 4=right 5=stop 6=faster 7=slower. Speed defaults live at the top of
 * voice_pipeline.c once Task 10 lands; here they're inlined for the
 * diagnostic. */
static int cmd_voice_inject_test(int argc, char **argv)
{
    if (argc != 2) { printf("usage: voice-inject-test <cmd_id 1..7>\n"); return 1; }
    int id = atoi(argv[1]);
    command_t c = { .source = SRC_VOICE };
    switch (id) {
    case 1: c.id = CMD_MOTOR; c.as.motor = (typeof(c.as.motor)){ +60, +60 }; break;
    case 2: c.id = CMD_MOTOR; c.as.motor = (typeof(c.as.motor)){ -60, -60 }; break;
    case 3: c.id = CMD_MOTOR; c.as.motor = (typeof(c.as.motor)){ -60, +60 }; break;
    case 4: c.id = CMD_MOTOR; c.as.motor = (typeof(c.as.motor)){ +60, -60 }; break;
    case 5: c.id = CMD_STOP;  break;
    default: printf("voice-inject-test: id 1..5 only (faster/slower need voice_pipeline.c)\n"); return 1;
    }
    esp_err_t r = command_bus_publish(&c);
    if (r != ESP_OK) {
        printf("voice-inject-test: publish failed: %s\n", esp_err_to_name(r));
        return 1;
    }
    printf("voice-inject-test: published cmd id=%d\n", id);
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

/* Output sample rate. This *sets* the PDM bit-clock via the ESP-IDF
 * driver: BCLK = sample_rate * mclk_multiple / bclk_div, and the driver
 * clamps bclk_div to >= 8. With mclk_multiple = 256 (default), that gives
 * BCLK = sample_rate * 32.
 *
 * MP34DT05TR-A / MP34DT06JTR (and most modern PDM MEMS mics) need PDM
 * CLK >= 1.2 MHz to leave sleep mode. sample_rate = 48000 gives BCLK =
 * 1.536 MHz — comfortably inside the 1.2–3.25 MHz valid range while
 * staying at a standard sample rate the S3 PDM RX handles cleanly. */
#define PDM_SAMPLE_RATE_HZ 48000

/* Result of a single PDM capture. `rms` is the raw magnitude; `ac_rms` is
 * the magnitude after subtracting `mean` (DC offset). Real audio has
 * mean ≈ 0 and ac_rms carrying the signal; a floating DIN pin decoded by
 * the ESP-IDF PDM decimator produces a large DC bias (rail-hugging) with
 * tiny ac_rms — so `ac_rms` is the discriminator, not `rms` or `peak2peak`. */
typedef struct {
    int    n;
    double mean;
    double rms;
    double ac_rms;
    int    p2p;
} pdm_stats_t;

/* Read `ms` milliseconds of PDM mono audio on (clk, dat). Fully allocates
 * and tears down the I2S channel each call so the caller can iterate pin
 * pairs freely. A short settling window is captured and discarded first
 * because most PDM MEMS mics need a few ms of clock before valid output. */
static esp_err_t pdm_capture(int clk_gpio, int dat_gpio, int ms, pdm_stats_t *out)
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

    /* Settling: read and discard ~50 ms so the mic (and the decimator's
     * DC blocker) reach steady state before we start measuring. */
    const int settle_samples = (PDM_SAMPLE_RATE_HZ / 1000) * 50;
    int16_t *settle_buf = (int16_t *)malloc(settle_samples * sizeof(int16_t));
    if (settle_buf) {
        size_t discarded = 0;
        (void)i2s_channel_read(rx, settle_buf, settle_samples * sizeof(int16_t),
                               &discarded, pdMS_TO_TICKS(150));
        free(settle_buf);
    }

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
    int64_t sum = 0, sum_sq = 0;
    for (int i = 0; i < n; i++) {
        int16_t s = buf[i];
        if (s < vmin) vmin = s;
        if (s > vmax) vmax = s;
        sum    += s;
        sum_sq += (int32_t)s * (int32_t)s;
    }

    double mean = n > 0 ? (double)sum / (double)n : 0.0;
    /* var = E[s^2] - E[s]^2, then ac_rms = sqrt(var). Guard against
     * negative from floating-point rounding on stuck-constant streams. */
    double ms_val = n > 0 ? (double)sum_sq / (double)n : 0.0;
    double var    = ms_val - mean * mean;
    if (var < 0.0) var = 0.0;

    free(buf);

    out->n      = n;
    out->mean   = mean;
    out->rms    = sqrt(ms_val);
    out->ac_rms = sqrt(var);
    out->p2p    = (int)vmax - (int)vmin;
    return ESP_OK;
}

static int cmd_gpio_set(int argc, char **argv)
{
    if (argc != 3) { printf("usage: gpio-set <pin> <0|1>\n"); return 1; }
    int pin   = atoi(argv[1]);
    int level = atoi(argv[2]);
    if (pin < 0 || pin > 48) { printf("pin out of range\n"); return 1; }
    if (level != 0 && level != 1) { printf("level must be 0 or 1\n"); return 1; }
    gpio_reset_pin(pin);
    gpio_set_direction(pin, GPIO_MODE_OUTPUT);
    gpio_set_level(pin, level);
    printf("GPIO%d = %d (held until next reset or gpio-set)\n", pin, level);
    return 0;
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

    pdm_stats_t st = {0};
    esp_err_t e = pdm_capture(clk, dat, ms, &st);
    if (e != ESP_OK) {
        printf("clk=%d dat=%d FAILED (%s)\n", clk, dat, esp_err_to_name(e));
        return 1;
    }
    printf("clk=%d dat=%d n=%d mean=%.0f rms=%.0f ac_rms=%.0f peak2peak=%d\n",
           clk, dat, st.n, st.mean, st.rms, st.ac_rms, st.p2p);
    return 0;
}

static int cmd_pdm_watch(int argc, char **argv)
{
    if (argc < 3) {
        printf("usage: pdm-watch <clk_gpio> <dat_gpio> [seconds]\n"
               "       prints mean / ac_rms every 100 ms — snap fingers to see ac_rms spike\n");
        return 1;
    }
    int clk = atoi(argv[1]);
    int dat = atoi(argv[2]);
    int secs = (argc >= 4) ? atoi(argv[3]) : 5;
    if (secs <= 0) secs = 5;

    int chunks = secs * 10;
    for (int i = 0; i < chunks; i++) {
        pdm_stats_t st = {0};
        esp_err_t e = pdm_capture(clk, dat, 100, &st);
        if (e != ESP_OK) {
            printf("[%d] FAILED (%s)\n", i, esp_err_to_name(e));
            return 1;
        }
        printf("[%3d] mean=%6.0f ac_rms=%6.0f peak2peak=%5d\n",
               i, st.mean, st.ac_rms, st.p2p);
    }
    return 0;
}

/* HIT threshold: significant AC energy AND that energy dominates any DC
 * bias. Floating DIN pins produce huge rms/peak2peak but ~0 ac_rms
 * because the decimator's output is stuck near a rail. */
#define PDM_HIT_AC_RMS_MIN 400.0

static int cmd_pdm_scan(int argc, char **argv)
{
    int ms = (argc >= 2) ? atoi(argv[1]) : 100;
    if (ms < 20) ms = 20;

    int fixed_clk = (argc >= 3) ? atoi(argv[2]) : -1;

    /* Optional mic-enable pin: `pdm-scan [ms] [fixed_clk] [en_gpio]`.
     * Held HIGH throughout the whole scan and excluded from CLK/DAT
     * roles. Lets us test hypotheses like "GPIO 38 gates mic VDD"
     * without letting the scan itself toggle that pin. */
    int en_gpio = (argc >= 4) ? atoi(argv[3]) : -1;
    if (en_gpio >= 0) {
        gpio_reset_pin(en_gpio);
        gpio_set_direction(en_gpio, GPIO_MODE_OUTPUT);
        gpio_set_level(en_gpio, 1);
        vTaskDelay(pdMS_TO_TICKS(10));
        printf("pdm-scan: holding GPIO%d HIGH as mic-enable throughout scan\n",
               en_gpio);
    }

    printf("pdm-scan: %d candidate pins, %d ms per probe (settle 50 ms)\n",
           (int)PDM_CANDIDATES_N, ms);
    printf("      keep talking / making noise near the mic during the run;\n");
    printf("      a real mic shows ac_rms >= %.0f. Floating pins hit high\n"
           "      rms/peak2peak with ac_rms ~ 0 and are ignored.\n",
           PDM_HIT_AC_RMS_MIN);

    int probes = 0, hits = 0;
    double best_ac = 0.0;
    int    best_clk = -1, best_dat = -1;

    for (size_t i = 0; i < PDM_CANDIDATES_N; i++) {
        int clk = PDM_CANDIDATES[i];
        if (fixed_clk >= 0 && clk != fixed_clk) continue;
        if (clk == en_gpio) continue;   /* enable pin — don't repurpose */

        int clk_hits = 0, clk_errors = 0;
        printf("clk=%2d: ", clk);
        fflush(stdout);

        for (size_t j = 0; j < PDM_CANDIDATES_N; j++) {
            if (i == j) continue;
            int dat = PDM_CANDIDATES[j];
            if (dat == en_gpio) continue; /* enable pin — don't repurpose */

            pdm_stats_t st = {0};
            esp_err_t e = pdm_capture(clk, dat, ms, &st);
            probes++;

            if (e != ESP_OK) { clk_errors++; continue; }

            if (st.ac_rms >= PDM_HIT_AC_RMS_MIN) {
                printf("\n  HIT clk=%2d dat=%2d mean=%6.0f ac_rms=%6.0f p2p=%5d",
                       clk, dat, st.mean, st.ac_rms, st.p2p);
                fflush(stdout);
                hits++;
                clk_hits++;
                if (st.ac_rms > best_ac) {
                    best_ac  = st.ac_rms;
                    best_clk = clk;
                    best_dat = dat;
                }
            }
        }
        printf(" [%d hits, %d init errors]\n", clk_hits, clk_errors);
        fflush(stdout);
    }
    printf("pdm-scan done: %d probes, %d hits\n", probes, hits);
    if (best_clk >= 0) {
        printf("  best: clk=%d dat=%d ac_rms=%.0f — try `pdm-watch %d %d 5`\n",
               best_clk, best_dat, best_ac, best_clk, best_dat);
    } else {
        printf("  no candidate showed AC energy. mic is not PDM on any free\n"
               "  GPIO, or needs an enable/MCLK pin held first.\n");
    }
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
        { .command = "bb-tlc-init",   .help = "init TLC59108 via bit-bang I2C (the working motor-driver init on this board)", .func = cmd_bb_tlc_init },
        { .command = "bb-tlc-set",    .help = "drive one TLC59108 channel via bit-bang: bb-tlc-set <ch 0-7> <pct 0-100>", .func = cmd_bb_tlc_set },
        { .command = "bb-tlc-sweep",  .help = "cycle all 8 TLC59108 channels via bit-bang (2 s each) — find which is M1/M2/P1..P4", .func = cmd_bb_tlc_sweep },
        { .command = "es-verify",     .help = "M3 codec check: drive MCLK on ES8311_I2S_MCLK and read product ID (expect 0x83) via bit-bang I2C", .func = cmd_es_verify },
        { .command = "es-init",       .help = "M3 codec: drive MCLK and run the ES8311 register-level init recipe (16 kHz mono ADC path)", .func = cmd_es_init },
        { .command = "es-dump",       .help = "M3 codec: hex-dump the ES8311's key control registers over bit-bang I2C", .func = cmd_es_dump },
        { .command = "es-peek",       .help = "M3 codec: read one ES8311 register: es-peek <reg_hex>", .func = cmd_es_peek },
        { .command = "es-poke",       .help = "M3 codec: write one ES8311 register: es-poke <reg_hex> <val_hex>", .func = cmd_es_poke },
        { .command = "voice-record",  .help = "M3 mic: capture N s of PCM to PSRAM then hex-dump: voice-record <sec 1..10> [slot=L|R] (default R)", .func = cmd_voice_record },
        { .command = "voice-stats",   .help = "M3 diagnostic: print audio_capture dropped-frame count", .func = cmd_voice_stats },
        { .command = "audio-diag",    .help = "M3 diagnostic: print I²S TX/RX-task iteration counts + last return codes", .func = cmd_audio_diag },
        { .command = "voice-scan",    .help = "M3 diagnostic: sweep I2S DIN candidate GPIOs × slot L/R and report max|sample| (~18 s)", .func = cmd_voice_scan },
        { .command = "mic-perm",      .help = "M3 diagnostic: try all 6 permutations of GPIOs 40/41/42 as (BCLK,WS,DIN) × slot L/R (~5 s)", .func = cmd_mic_perm },
        { .command = "voice-inject-test", .help = "M3 diagnostic: publish a synthetic voice command through the bus: voice-inject-test <id 1..5>", .func = cmd_voice_inject_test },
        { .command = "mic-enable-scan",   .help = "M3 diagnostic: drive each of 21 candidate GPIOs HIGH (or LOW with `mic-enable-scan 0`) and record via the ES8311 path to find the pin that ungates the mic (~18 s)", .func = cmd_mic_enable_scan },
        { .command = "pdm-probe",     .help = "M3 mic: test one PDM CLK/DAT pair (PDM hypothesis for MP34DT05/06): pdm-probe <clk> <dat> [ms]",         .func = cmd_pdm_probe },
        { .command = "pdm-watch",     .help = "M3 mic: live PDM RMS every 100 ms — snap fingers to see spike: pdm-watch <clk> <dat> [seconds]",         .func = cmd_pdm_watch },
        { .command = "pdm-scan",      .help = "M3 mic: brute-force PDM CLK/DAT pairs. pdm-scan [ms] [fixed_clk] [en_gpio] — en_gpio (optional) is held HIGH throughout and skipped as CLK/DAT.", .func = cmd_pdm_scan },
        { .command = "gpio-set",      .help = "hold one GPIO HIGH/LOW as output: gpio-set <pin> <0|1>", .func = cmd_gpio_set },
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
                ESP_LOGW(TAG, "TLC59108 present but ESP-IDF I2C init failed — run `bb-tlc-init` at the REPL");
            }
        } else {
            ESP_LOGW(TAG, "could not add TLC59108 (0x%02x) to I2C bus", TLC59108_ADDR);
        }
    } else {
        ESP_LOGW(TAG, "I2C bus init failed — expansion board control disabled");
    }

    /* Boot indicator — green if LED pin happens to be right */
    led_set(0, 32, 0);

    esp_console_repl_t              *repl        = NULL;
    esp_console_repl_config_t        repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    repl_config.prompt                            = "quarky> ";
    repl_config.max_cmdline_length                = 256;
    repl_config.task_stack_size                   = 8192;

    esp_console_dev_uart_config_t uart_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_uart(&uart_config, &repl_config, &repl));

    /* Command bus + dispatcher (Task 5). Voice and REPL both publish here;
     * dispatcher runs on core 0 prio 5. led_indicator (Task 6) and
     * migrated action verbs (Task 7) plug in via strong-symbol overrides
     * over command_bus.c's WEAK stubs. */
    ESP_ERROR_CHECK(command_bus_start());

    /* LED indicator FSM (Task 6). Owns the WS2812B from here on — REPL
     * `led` publishes CMD_LED_RGB through the bus, and voice states
     * (LISTENING / OK / NACK) preempt user RGB with automatic restore. */
    ESP_ERROR_CHECK(led_indicator_start());

    register_commands();
    ESP_ERROR_CHECK(esp_console_start_repl(repl));

    ESP_LOGI(TAG, "REPL ready — try: help, pins, selftest");
    /* app_main returns; the REPL keeps running in its own task. */
}
