/*
 * audio_capture — see audio_capture.h. This file owns the I²S RX
 * peripheral (I2S_NUM_0) for the lifetime of a capture session.
 */

#include "audio_capture.h"
#include "pins.h"

#include "driver/i2s_std.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/task.h"

static const char *TAG = "audio_capture";

static i2s_chan_handle_t s_rx_chan = NULL;
static TaskHandle_t      s_task    = NULL;
static QueueHandle_t     s_queue   = NULL;
static volatile uint32_t s_dropped = 0;
static volatile bool     s_running = false;

/* Producer task: blocking i2s_channel_read → enqueue non-blocking → count
 * drops. Runs on core 1 at high priority so DMA is serviced promptly. */
static void capture_task(void *arg)
{
    (void)arg;
    int16_t frame[AUDIO_CAPTURE_FRAME_SAMPLES];
    size_t  bytes_read = 0;

    while (s_running) {
        esp_err_t r = i2s_channel_read(s_rx_chan, frame, sizeof frame,
                                       &bytes_read, pdMS_TO_TICKS(200));
        if (r != ESP_OK || bytes_read != sizeof frame) continue;
        if (xQueueSend(s_queue, frame, 0) != pdTRUE) s_dropped++;
    }
    /* Signal cleanup: park task; audio_capture_stop deletes the handle. */
    vTaskDelete(NULL);
}

esp_err_t audio_capture_start_ex(QueueHandle_t out_queue,
                                 const audio_capture_config_t *cfg)
{
    if (s_running) return ESP_ERR_INVALID_STATE;
    if (!out_queue) return ESP_ERR_INVALID_ARG;

    int din_pin  = (cfg && cfg->din_gpio  > 0) ? cfg->din_gpio  : MIC_I2S_SD;
    int bclk_pin = (cfg && cfg->bclk_gpio > 0) ? cfg->bclk_gpio : MIC_I2S_SCK;
    int ws_pin   = (cfg && cfg->ws_gpio   > 0) ? cfg->ws_gpio   : MIC_I2S_WS;
    audio_capture_slot_t slot = cfg ? cfg->slot : AUDIO_CAPTURE_SLOT_LEFT;

    s_queue   = out_queue;
    s_dropped = 0;

    /* 8 DMA buffers × 320 frames × 4 B (32-bit slot) = 10.2 KB, 20 ms per
     * buffer, 160 ms total headroom. Plenty for a core-1 task at prio 22. */
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num  = 8;
    chan_cfg.dma_frame_num = 320;
    ESP_RETURN_ON_ERROR(i2s_new_channel(&chan_cfg, NULL, &s_rx_chan), TAG, "new_channel");

    /* INMP441-family digital MEMS: 24-bit sample in a 32-bit slot, MSB-first.
     * ESP-IDF's I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG with data=16 / slot=32
     * makes the driver take the top 16 bits of each 32-bit slot — that is
     * the signed high-order half of the 24-bit sample, which is exactly
     * what ESP-SR wants (16 kHz mono s16le, no resample). */
    i2s_slot_mode_t slot_mode = (slot == AUDIO_CAPTURE_SLOT_BOTH)
                                    ? I2S_SLOT_MODE_STEREO : I2S_SLOT_MODE_MONO;
    i2s_std_slot_config_t slot_cfg =
        I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, slot_mode);
    slot_cfg.slot_bit_width = I2S_SLOT_BIT_WIDTH_32BIT;
    if (slot == AUDIO_CAPTURE_SLOT_LEFT)  slot_cfg.slot_mask = I2S_STD_SLOT_LEFT;
    if (slot == AUDIO_CAPTURE_SLOT_RIGHT) slot_cfg.slot_mask = I2S_STD_SLOT_RIGHT;

    i2s_std_config_t std_cfg = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(16000),
        .slot_cfg = slot_cfg,
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,        /* digital MEMS mic needs no MCLK */
            .bclk = bclk_pin,
            .ws   = ws_pin,
            .dout = I2S_GPIO_UNUSED,        /* RX-only path */
            .din  = din_pin,
            .invert_flags = { .mclk_inv = false, .bclk_inv = false, .ws_inv = false },
        },
    };

    ESP_RETURN_ON_ERROR(i2s_channel_init_std_mode(s_rx_chan, &std_cfg), TAG, "init std");
    ESP_RETURN_ON_ERROR(i2s_channel_enable(s_rx_chan), TAG, "enable");

    s_running = true;
    BaseType_t ok = xTaskCreatePinnedToCore(capture_task, "aud_cap", 4096, NULL,
                                            22, &s_task, 1);
    if (ok != pdPASS) {
        s_running = false;
        i2s_channel_disable(s_rx_chan);
        i2s_del_channel(s_rx_chan);
        s_rx_chan = NULL;
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "capture started (16 kHz mono s16le, BCLK=GPIO%d WS=GPIO%d "
                  "DIN=GPIO%d slot=%s, 32-bit slot / 16-bit sample)",
             bclk_pin, ws_pin, din_pin,
             slot == AUDIO_CAPTURE_SLOT_LEFT ? "L" :
             slot == AUDIO_CAPTURE_SLOT_RIGHT ? "R" : "LR");
    return ESP_OK;
}

esp_err_t audio_capture_start(QueueHandle_t out_queue)
{
    return audio_capture_start_ex(out_queue, NULL);
}

esp_err_t audio_capture_stop(void)
{
    if (!s_running) return ESP_OK;

    s_running = false;
    /* Give capture_task at least one read-timeout to notice s_running=false
     * and vTaskDelete itself; DMA read blocks up to 200 ms. */
    vTaskDelay(pdMS_TO_TICKS(250));
    s_task = NULL;

    if (s_rx_chan) {
        i2s_channel_disable(s_rx_chan);
        i2s_del_channel(s_rx_chan);
        s_rx_chan = NULL;
    }
    s_queue = NULL;
    return ESP_OK;
}

uint32_t audio_capture_dropped_frames(void) { return s_dropped; }
