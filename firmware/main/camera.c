#include "camera.h"

#include "esp_log.h"
#include "esp_camera.h"

static const char *TAG = "cam";

/* ============================================================
 *  PLACEHOLDER PIN VALUES — update after live-capture on stock
 *  (see camera.h). Guessing at a stock ESP32-S3-CAM layout so
 *  the code compiles; DO NOT EXPECT IMAGES until the real pins
 *  are known and pasted here.
 * ============================================================ */
#define PIN_CAM_XCLK       15
#define PIN_CAM_PCLK       13
#define PIN_CAM_VSYNC       6
#define PIN_CAM_HREF        7
#define PIN_CAM_SDA         4    /* SCCB — separate from ES8311 I²C */
#define PIN_CAM_SCL         5
#define PIN_CAM_D0         11
#define PIN_CAM_D1          9
#define PIN_CAM_D2          8
#define PIN_CAM_D3         10
#define PIN_CAM_D4         12
#define PIN_CAM_D5         18
#define PIN_CAM_D6         17
#define PIN_CAM_D7         16
#define PIN_CAM_PWDN       -1    /* not connected on many S3 boards */
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
