#include "camera.h"

#include "esp_log.h"
#include "esp_camera.h"

static const char *TAG = "cam";

/* ============================================================
 *  Live-verified from stock firmware (2026-09-07) by reading
 *  GPIO_FUNCn_IN_SEL_CFG / OUT_SEL_CFG registers while the OV5640
 *  was already initialised. Decoded against
 *  esp32s3/soc/gpio_sig_map.h:
 *    IN_SEL[149] CAM_PCLK  <- GPIO 11
 *    IN_SEL[150] CAM_HREF  <- GPIO 16
 *    IN_SEL[152] CAM_VSYNC <- GPIO 38    (no HSYNC binding used)
 *    IN_SEL[133..140] D0..D7 <- 9,19,8,20,10,12,13,21
 *    IN_SEL[89/90] I²C0 SCL/SDA <- 18/17 (SCCB shares the ES8311 bus)
 *    OUT_SEL[14] = sig 149 (CAM_CLK)  → XCLK on GPIO 14
 *  PWDN and RESET are not connected on this board (no OUT_SEL binding
 *  observed, so -1 tells esp_camera_init to skip them).
 * ============================================================ */
#define PIN_CAM_XCLK       14
#define PIN_CAM_PCLK       11
#define PIN_CAM_VSYNC      38
#define PIN_CAM_HREF       16
#define PIN_CAM_SDA        17    /* SCCB shares GPIO 17/18 with the ES8311 I²C bus */
#define PIN_CAM_SCL        18
#define PIN_CAM_D0          9
#define PIN_CAM_D1         19
#define PIN_CAM_D2          8
#define PIN_CAM_D3         20
#define PIN_CAM_D4         10
#define PIN_CAM_D5         12
#define PIN_CAM_D6         13
#define PIN_CAM_D7         21
#define PIN_CAM_PWDN       -1    /* not wired on the Quarky */
#define PIN_CAM_RESET      -1

esp_err_t camera_start(void)
{
    camera_config_t cfg = {
        .pin_pwdn        = PIN_CAM_PWDN,
        .pin_reset       = PIN_CAM_RESET,
        .pin_xclk        = PIN_CAM_XCLK,
        .pin_sccb_sda    = PIN_CAM_SDA,
        .pin_sccb_scl    = PIN_CAM_SCL,

        .pin_d7          = PIN_CAM_D7,
        .pin_d6          = PIN_CAM_D6,
        .pin_d5          = PIN_CAM_D5,
        .pin_d4          = PIN_CAM_D4,
        .pin_d3          = PIN_CAM_D3,
        .pin_d2          = PIN_CAM_D2,
        .pin_d1          = PIN_CAM_D1,
        .pin_d0          = PIN_CAM_D0,
        .pin_vsync       = PIN_CAM_VSYNC,
        .pin_href        = PIN_CAM_HREF,
        .pin_pclk        = PIN_CAM_PCLK,

        .xclk_freq_hz    = 20000000,          /* 20 MHz XCLK — OV5640 sweet spot */
        .ledc_timer      = LEDC_TIMER_1,       /* codec uses TIMER_2 later, keep separate */
        .ledc_channel    = LEDC_CHANNEL_2,

        .pixel_format    = PIXFORMAT_JPEG,
        .frame_size      = FRAMESIZE_QVGA,     /* 320×240 — fast enough for streaming over Wi-Fi */
        .jpeg_quality    = 12,                 /* 0..63, lower = better */
        .fb_count        = 2,                  /* 2 = smoother stream but needs PSRAM */
        .fb_location     = CAMERA_FB_IN_PSRAM,
        .grab_mode       = CAMERA_GRAB_LATEST,
    };

    esp_err_t err = esp_camera_init(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_camera_init failed: 0x%x (%s) — pin map probably wrong; see camera.h",
                 err, esp_err_to_name(err));
        return err;
    }

    /* Match stock's orientation. */
    sensor_t *s = esp_camera_sensor_get();
    if (s) {
        s->set_hmirror(s, 1);
        s->set_vflip(s, 1);
    }

    ESP_LOGI(TAG, "camera up: OV5640 QVGA JPEG q12 (XCLK=GPIO%d)", PIN_CAM_XCLK);
    return ESP_OK;
}
