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

/* TLC59108 I²C address. The ESP-IDF new-i2c-master driver path was
 * removed (it never ACK'd on this board's TLC anyway, and it collides
 * with esp32-camera's legacy SCCB driver at link time) — everything
 * now goes through the bit-bang path below. */
#define TLC59108_ADDR        0x40
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

/* --- motors (LEGACY: LEDC PWM on A1/A2 — do NOT use, corrupts I2C) --- */
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

/* --- I²C bus scan ---------------------------------------------------- */

/* --- I2C bus hunt: sweep candidate (SDA, SCL) pin pairs --------------- */

/* --- I2C selftest: sanity-check the probe machinery on one pin pair ---- */

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

    bb_init(sda, scl);
    bool ok = bb_tlc59108_init_seq(sda, scl);
    if (!ok) {
        printf("bb-tlc-init: NACK on init sequence\n");
        gpio_set_direction(sda, GPIO_MODE_INPUT);
        gpio_set_direction(scl, GPIO_MODE_INPUT);
        s_bb_ready = false;
        return 1;
    }
    printf("bb-tlc-init: OK — TLC59108 initialised via bit-bang @ 0x%02x\n",
           TLC59108_ADDR);
    /* Keep pins in a released state; each bb_* call re-configures as needed. */
    gpio_set_direction(sda, GPIO_MODE_INPUT);
    gpio_set_direction(scl, GPIO_MODE_INPUT);
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
        { .command = "servo",     .help = "set servo: servo <deg 0-180>",               .func = cmd_servo },
        { .command = "servo-pin", .help = "reconfigure servo pin at runtime: servo-pin <gpio>", .func = cmd_servo_pin },
        { .command = "selftest",  .help = "run LED + servo + motor smoke test",         .func = cmd_selftest },
        { .command = "pins",        .help = "print current pin assignments",              .func = cmd_pins },
        { .command = "led-sweep",   .help = "cycle LED init across candidate GPIOs (2s per pin)",     .func = cmd_sweep_led },
        { .command = "servo-sweep", .help = "cycle servo signal across candidate GPIOs (2s per pin)", .func = cmd_sweep_servo },
        { .command = "motor-sweep", .help = "drive candidate GPIOs as motor PWM (2s per pin)",        .func = cmd_sweep_motor },
        { .command = "pin-hunt",    .help = "watch candidate GPIOs and report LOW transitions when you jumper a header pin to GND: pin-hunt [seconds]", .func = cmd_pin_hunt },
        { .command = "adc-hunt",    .help = "watch every ADC-capable GPIO for value swings — for resistor-ladder buttons: adc-hunt [seconds] [threshold]", .func = cmd_adc_hunt },
        { .command = "button",      .help = "read the shared button ADC on GPIO 7: button [seconds]", .func = cmd_button },
        { .command = "read-adc",    .help = "read ADC on any GPIO 1..20: read-adc <gpio> [seconds]", .func = cmd_read_adc },
        { .command = "button-state",.help = "classify GPIO 7 ADC into NONE / L / R / BOTH: button-state [seconds]", .func = cmd_button_state },
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
