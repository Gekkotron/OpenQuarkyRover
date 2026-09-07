/*
 * wake_word — see wake_word.h.
 *
 * Pipeline: audio_capture (I²S mono 16 kHz, 512-sample frames)
 *   -> input_queue
 *   -> feed_task  -> afe->feed()   (repacks to AFE's feed chunksize)
 *   -> AFE_SR    -> fetch_task   -> afe->fetch()
 *   -> on WAKENET_DETECTED: LED green flash + log line
 */

#include "wake_word.h"

#include "audio_capture.h"
#include "led_indicator.h"
#include "command_bus.h"

#include "esp_afe_sr_iface.h"
#include "esp_afe_sr_models.h"
#include "esp_afe_config.h"
#include "esp_wn_iface.h"
#include "model_path.h"

#include "esp_log.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include <stdio.h>
#include <string.h>

static const char *TAG = "wake_word";

/* --- Module state ------------------------------------------------------ */

static volatile bool          s_running       = false;
static srmodel_list_t         *s_models       = NULL;
static afe_config_t           *s_afe_config   = NULL;
static const esp_afe_sr_iface_t *s_afe_iface  = NULL;
static esp_afe_sr_data_t      *s_afe_data     = NULL;
static QueueHandle_t           s_audio_queue  = NULL;
static TaskHandle_t            s_feed_task    = NULL;
static TaskHandle_t            s_fetch_task   = NULL;

/* Feed-side chunk plumbing. AFE's expected feed size ("afe_feed_samples")
 * may differ from audio_capture's frame size, so we accumulate one AFE
 * chunk at a time from possibly-multiple audio_capture frames. */
static int      s_afe_feed_samples   = 0;   /* samples-per-channel AFE wants */
static int      s_afe_feed_channels  = 1;   /* channel count AFE was told about */
static int16_t *s_feed_buf           = NULL; /* holds one AFE feed chunk */
static int      s_feed_buf_fill      = 0;   /* current fill in samples */

/* Diagnostics — non-zero counters + ESP_OK last codes mean the tasks
 * are looping cleanly. */
static volatile uint32_t s_feed_iters       = 0;
static volatile uint32_t s_fetch_iters      = 0;
static volatile uint32_t s_detect_count     = 0;
static volatile int      s_last_fetch_ret   = 0;
static volatile int      s_last_wake_state  = 0;

/* --- Feed task --------------------------------------------------------- */

static void feed_task(void *arg)
{
    (void)arg;
    int16_t frame[AUDIO_CAPTURE_FRAME_SAMPLES];

    while (s_running) {
        if (xQueueReceive(s_audio_queue, frame, pdMS_TO_TICKS(200)) != pdTRUE) {
            /* Producer idle; loop and retry so we notice s_running=false. */
            continue;
        }

        /* audio_capture is single-channel mono. AFE with input_format="M"
         * expects the same. Copy into the AFE chunk buffer, feed when full. */
        int remaining = AUDIO_CAPTURE_FRAME_SAMPLES;
        int16_t *src  = frame;
        while (remaining > 0 && s_running) {
            int room = s_afe_feed_samples - s_feed_buf_fill;
            int take = (remaining < room) ? remaining : room;
            memcpy(&s_feed_buf[s_feed_buf_fill * s_afe_feed_channels],
                   src, (size_t)take * sizeof(int16_t) * s_afe_feed_channels);
            s_feed_buf_fill += take;
            src             += take;
            remaining       -= take;
            if (s_feed_buf_fill >= s_afe_feed_samples) {
                s_afe_iface->feed(s_afe_data, s_feed_buf);
                s_feed_iters++;
                s_feed_buf_fill = 0;
            }
        }
    }
    s_feed_task = NULL;
    vTaskDelete(NULL);
}

/* --- Fetch task -------------------------------------------------------- */

static void fetch_task(void *arg)
{
    (void)arg;
    while (s_running) {
        afe_fetch_result_t *res = s_afe_iface->fetch(s_afe_data);
        s_fetch_iters++;
        if (!res) { s_last_fetch_ret = -1; continue; }
        s_last_fetch_ret  = res->ret_value;
        s_last_wake_state = (int)res->wakeup_state;
        if (res->ret_value == ESP_FAIL) continue;
        if (res->wakeup_state == WAKENET_DETECTED) {
            s_detect_count++;
            ESP_LOGI(TAG, "*** WAKE #%lu — Hi ESP heard (idx=%d) ***",
                     (unsigned long)s_detect_count, res->wake_word_index);
            /* Publish through the command bus so downstream consumers
             * (MultiNet, motor arbitration) can hook the wake event; and
             * kick a green LED flash so the user gets immediate feedback. */
            command_t wake = {
                .id     = CMD_VOICE_WAKE,
                .source = SRC_VOICE,
                .as.voice_wake = { .model_index = (uint8_t)res->wake_word_index },
            };
            (void)command_bus_publish(&wake);
            command_t flash = {
                .id     = CMD_LED_STATE,
                .source = SRC_VOICE,
                .as.led_state = { .state = LED_STATE_OK },
            };
            (void)command_bus_publish(&flash);
        }
    }
    s_fetch_task = NULL;
    vTaskDelete(NULL);
}

/* --- Public API -------------------------------------------------------- */

esp_err_t wake_word_start(void)
{
    if (s_running) return ESP_ERR_INVALID_STATE;

    /* 1. Load models packed into the "model" flash partition. */
    s_models = esp_srmodel_init("model");
    if (!s_models || s_models->num == 0) {
        ESP_LOGE(TAG, "esp_srmodel_init returned no models — is the "
                       "model partition flashed?");
        return ESP_ERR_NOT_FOUND;
    }
    for (int i = 0; i < s_models->num; i++) {
        ESP_LOGI(TAG, "  model[%d] = %s", i, s_models->model_name[i]);
    }

    /* 2. Build AFE config for single-mic wake-word detection.
     *    - input_format "M"     : one microphone channel, no reference
     *    - AFE_TYPE_SR          : speech recognition preset (WebRTC NS,
     *                             no nonlinear NS which fights wake-word)
     *    - AFE_MODE_LOW_COST    : cheaper for wake-word-only work */
    s_afe_config = afe_config_init("M", s_models, AFE_TYPE_SR, AFE_MODE_LOW_COST);
    if (!s_afe_config) {
        ESP_LOGE(TAG, "afe_config_init failed");
        return ESP_FAIL;
    }

    /* 3. Create the AFE instance. */
    s_afe_iface = esp_afe_handle_from_config(s_afe_config);
    if (!s_afe_iface) {
        ESP_LOGE(TAG, "esp_afe_handle_from_config returned NULL");
        return ESP_FAIL;
    }
    s_afe_data = s_afe_iface->create_from_config(s_afe_config);
    if (!s_afe_data) {
        ESP_LOGE(TAG, "AFE create_from_config failed");
        return ESP_FAIL;
    }

    s_afe_feed_samples  = s_afe_iface->get_feed_chunksize(s_afe_data);
    s_afe_feed_channels = s_afe_iface->get_feed_channel_num(s_afe_data);
    int sr              = s_afe_iface->get_samp_rate(s_afe_data);
    ESP_LOGI(TAG, "AFE ready: feed=%d samples/ch × %d ch @ %d Hz",
             s_afe_feed_samples, s_afe_feed_channels, sr);
    if (sr != 16000) {
        ESP_LOGW(TAG, "AFE expects %d Hz but audio_capture is 16000 Hz — "
                       "wake-word accuracy will suffer", sr);
    }

    s_feed_buf = heap_caps_malloc(
        (size_t)s_afe_feed_samples * s_afe_feed_channels * sizeof(int16_t),
        MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!s_feed_buf) {
        ESP_LOGE(TAG, "feed buffer alloc failed (%d samples × %d ch)",
                 s_afe_feed_samples, s_afe_feed_channels);
        return ESP_ERR_NO_MEM;
    }
    s_feed_buf_fill = 0;

    /* 4. Start audio_capture on the LEFT slot (where the ES8311 puts
     *    the mono ADC per the live capture) into our own queue. */
    s_audio_queue = xQueueCreate(8, AUDIO_CAPTURE_FRAME_BYTES);
    if (!s_audio_queue) return ESP_ERR_NO_MEM;

    audio_capture_config_t cfg = { .slot = AUDIO_CAPTURE_SLOT_LEFT };
    esp_err_t r = audio_capture_start_ex(s_audio_queue, &cfg);
    if (r != ESP_OK) {
        ESP_LOGE(TAG, "audio_capture_start_ex: %d", r);
        vQueueDelete(s_audio_queue); s_audio_queue = NULL;
        return r;
    }

    /* 5. Reset counters, launch tasks. Fetch first so it's already
     *    blocked on AFE before feed starts pushing. Both on core 1
     *    to keep protocol / Wi-Fi work on core 0. */
    s_feed_iters = s_fetch_iters = s_detect_count = 0;
    s_last_fetch_ret = 0; s_last_wake_state = 0;
    s_running = true;

    if (xTaskCreatePinnedToCore(fetch_task, "wake_fetch", 4096, NULL,
                                20, &s_fetch_task, 1) != pdPASS) {
        ESP_LOGE(TAG, "fetch_task create failed");
        s_running = false;
        audio_capture_stop();
        return ESP_ERR_NO_MEM;
    }
    if (xTaskCreatePinnedToCore(feed_task, "wake_feed", 4096, NULL,
                                19, &s_feed_task, 1) != pdPASS) {
        ESP_LOGE(TAG, "feed_task create failed");
        s_running = false;
        vTaskDelay(pdMS_TO_TICKS(300));
        audio_capture_stop();
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "wake-word ready — say \"Hi ESP\"");
    return ESP_OK;
}

esp_err_t wake_word_stop(void)
{
    if (!s_running) return ESP_OK;

    s_running = false;
    /* Give both tasks a read-timeout to notice and vTaskDelete themselves. */
    vTaskDelay(pdMS_TO_TICKS(400));

    audio_capture_stop();

    if (s_afe_data && s_afe_iface) {
        s_afe_iface->destroy(s_afe_data);
        s_afe_data = NULL;
    }
    if (s_afe_config) {
        afe_config_free(s_afe_config);
        s_afe_config = NULL;
    }
    if (s_feed_buf)   { heap_caps_free(s_feed_buf); s_feed_buf = NULL; }
    if (s_audio_queue){ vQueueDelete(s_audio_queue); s_audio_queue = NULL; }
    s_models = NULL;  /* srmodel_list_t is owned by esp-sr's registry. */
    s_afe_iface = NULL;
    return ESP_OK;
}

bool wake_word_is_running(void) { return s_running; }

void wake_word_diag_print(void)
{
    printf("wake-diag: running=%d\n", (int)s_running);
    printf("  detections=%lu\n", (unsigned long)s_detect_count);
    printf("  feed_task:  iters=%lu\n", (unsigned long)s_feed_iters);
    printf("  fetch_task: iters=%lu  last_ret=%d  last_wake_state=%d\n",
           (unsigned long)s_fetch_iters,
           s_last_fetch_ret, s_last_wake_state);
    if (s_afe_iface && s_afe_data) {
        printf("  afe: feed_chunk=%d samples/ch × %d ch @ %d Hz\n",
               s_afe_feed_samples, s_afe_feed_channels,
               s_afe_iface->get_samp_rate(s_afe_data));
    }
}
