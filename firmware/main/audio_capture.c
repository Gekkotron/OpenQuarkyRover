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
static i2s_chan_handle_t s_tx_chan = NULL;   /* allocated but idle — see start_ex */
static TaskHandle_t      s_task    = NULL;
static QueueHandle_t     s_queue   = NULL;
static volatile uint32_t s_dropped = 0;
static volatile bool     s_running = false;

/* Layer-boundary diagnostics — see audio_capture_diag_print. Non-zero
 * counters + ESP_OK last-return means the task is looping cleanly; zero
 * means it never got past its first blocking call. */
static volatile uint32_t  s_rx_iters   = 0;
static volatile esp_err_t s_rx_last_r  = ESP_ERR_INVALID_STATE;
static volatile size_t    s_rx_last_bytes = 0;
static volatile uint32_t  s_tx_iters   = 0;
static volatile esp_err_t s_tx_last_r  = ESP_ERR_INVALID_STATE;
static volatile size_t    s_tx_last_bytes = 0;

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
        s_rx_iters++;
        s_rx_last_r = r;
        s_rx_last_bytes = bytes_read;
        if (r != ESP_OK || bytes_read != sizeof frame) continue;
        if (xQueueSend(s_queue, frame, 0) != pdTRUE) s_dropped++;
    }
    /* Signal cleanup: park task; audio_capture_stop deletes the handle. */
    vTaskDelete(NULL);
}

/* TX-silence task: writes zeros to TX so its DMA has data to move. On
 * ESP32-S3 in full-duplex master the TX clock generator is what drives
 * BCLK/WS on the pins — if TX DMA is dry, empirically the shared clock
 * tree stalls and RX DMA never advances. Keep the FIFO fed. */
static void tx_silence_task(void *arg)
{
    (void)arg;
    int16_t silence[AUDIO_CAPTURE_FRAME_SAMPLES] = {0};
    size_t written = 0;
    while (s_running) {
        esp_err_t r = i2s_channel_write(s_tx_chan, silence, sizeof silence,
                                        &written, pdMS_TO_TICKS(200));
        s_tx_iters++;
        s_tx_last_r = r;
        s_tx_last_bytes = written;
    }
    vTaskDelete(NULL);
}

esp_err_t audio_capture_start_ex(QueueHandle_t out_queue,
                                 const audio_capture_config_t *cfg)
{
    if (s_running) return ESP_ERR_INVALID_STATE;
    if (!out_queue) return ESP_ERR_INVALID_ARG;

    /* Mic path on this board is analog → ES8311 codec ADC → I²S. Defaults
     * point at the codec's I²S pins (BCLK=9, LRCK=45, DIN=10); the old
     * INMP441-on-pins-40/41/42 assumption was falsified by the stock
     * firmware strings dump (uses ESP-ADF ES8311 driver + MIC_GAIN_*
     * enum matching the codec's 0..42 dB PGA ladder). */
    int din_pin  = (cfg && cfg->din_gpio  > 0) ? cfg->din_gpio  : ES8311_I2S_DIN;
    int bclk_pin = (cfg && cfg->bclk_gpio > 0) ? cfg->bclk_gpio : ES8311_I2S_BCLK;
    int ws_pin   = (cfg && cfg->ws_gpio   > 0) ? cfg->ws_gpio   : ES8311_I2S_LRCK;
    audio_capture_slot_t slot = cfg ? cfg->slot : AUDIO_CAPTURE_SLOT_LEFT;

    s_queue   = out_queue;
    s_dropped = 0;

    /* 8 DMA buffers × 320 frames × 4 B (32-bit slot) = 10.2 KB, 20 ms per
     * buffer, 160 ms total headroom. Plenty for a core-1 task at prio 22.
     *
     * Allocate BOTH TX and RX handles so the peripheral runs in full-duplex
     * mode. On ESP32-S3 the ESP-IDF I²S driver in RX-only master mode binds
     * BCLK/WS to the RX-side signal indices (I2S0I_BCK=26, I2S0I_WS=27),
     * whose clock generator behaves differently from the TX-side one and
     * empirically stalled DMA on this codec. Stock firmware uses TX-side
     * indices (live-captured OUT_SEL[46]=22, OUT_SEL[39]=24) — that only
     * happens in full-duplex or TX-master mode. We init only RX in
     * STD mode and leave TX idle; the shared clocks are what matters. */
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num  = 8;
    chan_cfg.dma_frame_num = 320;
    ESP_RETURN_ON_ERROR(i2s_new_channel(&chan_cfg, &s_tx_chan, &s_rx_chan), TAG, "new_channel");

    /* ES8311 outputs standard 16-bit I²S Philips, mono on the LEFT slot in
     * its stock ADC-only config (REG44=0x58 routes internal ADCL). No
     * INMP441 24-in-32 trick; the slot width matches the sample width. */
    i2s_slot_mode_t slot_mode = (slot == AUDIO_CAPTURE_SLOT_BOTH)
                                    ? I2S_SLOT_MODE_STEREO : I2S_SLOT_MODE_MONO;
    i2s_std_slot_config_t slot_cfg =
        I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, slot_mode);
    if (slot == AUDIO_CAPTURE_SLOT_LEFT)  slot_cfg.slot_mask = I2S_STD_SLOT_LEFT;
    if (slot == AUDIO_CAPTURE_SLOT_RIGHT) slot_cfg.slot_mask = I2S_STD_SLOT_RIGHT;

    /* Keep the standard 32-BCLK-per-LRCK frame timing even in mono mode.
     * With slot_bit_width = AUTO in mono, ESP-IDF halves BCLK to 16 clocks
     * per LRCK — but the ES8311 in "16-bit I²S standard" mode (REG09/0A =
     * 0x0C) expects the classic 32 BCLK per LRCK (16 bits L + 16 bits R).
     * Under the halved framing the codec sees an incomplete cycle and
     * outputs bit-exact zero on SDPOUT. Live-confirmed: mono LEFT/RIGHT
     * both read zeros while stereo mode reads real audio on the LEFT
     * slot only. Forcing 32-bit slot width restores the correct framing
     * without paying the stereo mode's 2× DMA / RAM cost. */
    if (slot_mode == I2S_SLOT_MODE_MONO) {
        slot_cfg.slot_bit_width = I2S_SLOT_BIT_WIDTH_32BIT;
    }

    /* MCLK driven by the I²S peripheral itself on ES8311_I2S_MCLK. Stock
     * does the same (live-verified: OUT_SEL[GPIO 45] = 23 = I2S0_MCLK).
     * The earlier LEDC-driven MCLK left the codec's PLL unable to lock
     * because MCLK and BCLK came from independent clock domains — the
     * codec's REG 0x0D "clocks detected" bit stayed 0x01 (should be
     * 0x02) and SDPOUT was gated to digital zero. Sharing the I²S
     * peripheral clock tree keeps MCLK/BCLK/LRCK phase-locked, which
     * the ES8311's decimator requires. */
    i2s_std_config_t std_cfg = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(16000),
        .slot_cfg = slot_cfg,
        .gpio_cfg = {
            .mclk = ES8311_I2S_MCLK,
            .bclk = bclk_pin,
            .ws   = ws_pin,
            .dout = I2S_GPIO_UNUSED,        /* no playback yet — M4 will wire this */
            .din  = din_pin,
            .invert_flags = { .mclk_inv = false, .bclk_inv = false, .ws_inv = false },
        },
    };

    /* Full-duplex: init and enable TX too. On ESP32-S3 the TX-side clock
     * generator is what physically produces BCLK/WS on the pins in
     * full-duplex master mode — so TX must be configured (and enabled)
     * even though we never write real audio to it, or BCLK/WS stay dead
     * and the RX DMA never advances. Both directions share the same
     * std_cfg so they use the same clock tree and pin bindings. */
    ESP_RETURN_ON_ERROR(i2s_channel_init_std_mode(s_tx_chan, &std_cfg), TAG, "init tx std");
    ESP_RETURN_ON_ERROR(i2s_channel_init_std_mode(s_rx_chan, &std_cfg), TAG, "init rx std");

    /* Preload silence into TX DMA before enabling so the peripheral has
     * something to shift out from the very first BCLK tick. Empirically
     * TX with an empty FIFO seems to pause clocks on this SoC. */
    static int16_t preload_silence[AUDIO_CAPTURE_FRAME_SAMPLES * 4] = {0};
    size_t preloaded = 0;
    (void)i2s_channel_preload_data(s_tx_chan, preload_silence,
                                   sizeof preload_silence, &preloaded);

    ESP_RETURN_ON_ERROR(i2s_channel_enable(s_tx_chan), TAG, "enable tx");
    ESP_RETURN_ON_ERROR(i2s_channel_enable(s_rx_chan), TAG, "enable rx");

    s_running = true;

    /* Background writer that keeps TX DMA fed with silence (see the
     * task comment above). Starts BEFORE capture so clocks are already
     * flowing when we start reading. */
    (void)xTaskCreatePinnedToCore(tx_silence_task, "aud_tx0", 4096, NULL,
                                  21, NULL, 1);

    BaseType_t ok = xTaskCreatePinnedToCore(capture_task, "aud_cap", 4096, NULL,
                                            22, &s_task, 1);
    if (ok != pdPASS) {
        s_running = false;
        i2s_channel_disable(s_rx_chan);
        i2s_del_channel(s_rx_chan);
        s_rx_chan = NULL;
        if (s_tx_chan) { i2s_del_channel(s_tx_chan); s_tx_chan = NULL; }
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "capture started (16 kHz mono s16le, MCLK=I2S@GPIO%d "
                  "BCLK=GPIO%d WS=GPIO%d DIN=GPIO%d slot=%s)",
             ES8311_I2S_MCLK, bclk_pin, ws_pin, din_pin,
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
    if (s_tx_chan) {
        i2s_channel_disable(s_tx_chan);
        i2s_del_channel(s_tx_chan);
        s_tx_chan = NULL;
    }
    s_queue = NULL;
    return ESP_OK;
}

uint32_t audio_capture_dropped_frames(void) { return s_dropped; }

void audio_capture_diag_print(void)
{
    printf("audio-diag: running=%d\n", (int)s_running);
    printf("  RX task: iters=%lu  last_r=%s  last_bytes=%zu\n",
           (unsigned long)s_rx_iters,
           esp_err_to_name(s_rx_last_r),
           s_rx_last_bytes);
    printf("  TX task: iters=%lu  last_r=%s  last_bytes=%zu\n",
           (unsigned long)s_tx_iters,
           esp_err_to_name(s_tx_last_r),
           s_tx_last_bytes);
    printf("  dropped_frames=%lu\n", (unsigned long)s_dropped);
}
