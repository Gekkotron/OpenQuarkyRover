/*
 * wake_word — see wake_word.h.
 *
 * Pipeline: audio_capture (I²S mono 16 kHz, 512-sample frames)
 *   -> input_queue
 *   -> feed_task  -> afe->feed()   (repacks to AFE's feed chunksize)
 *   -> AFE_SR    -> fetch_task    -> afe->fetch()
 *     -> on WAKENET_DETECTED: enter LISTENING state, publish CMD_VOICE_WAKE
 *     -> while listening: feed AFE output to multinet->detect
 *       -> on ESP_MN_STATE_DETECTED: map phrase_id → CMD_MOTOR/CMD_STOP,
 *                                    LED OK, exit listening
 *       -> on ESP_MN_STATE_TIMEOUT:  LED NACK, exit listening
 */

#include "wake_word.h"

#include "audio_capture.h"
#include "led_indicator.h"
#include "command_bus.h"
#include "pins.h"      /* SERVO_MIN_US / SERVO_MAX_US */

#include "esp_afe_sr_iface.h"
#include "esp_afe_sr_models.h"
#include "esp_afe_config.h"
#include "esp_wn_iface.h"
#include "esp_mn_iface.h"
#include "esp_mn_models.h"
#include "esp_mn_speech_commands.h"
#include "model_path.h"

#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/idf_additions.h"   /* xTaskCreatePinnedToCoreWithCaps */

#include <stdio.h>
#include <string.h>

static const char *TAG = "wake_word";

/* --- Command vocabulary (matches cmd_voice_inject_test IDs) ------------- */

enum {
    VOICE_ID_FORWARD  = 1,
    VOICE_ID_BACKWARD = 2,
    VOICE_ID_LEFT     = 3,
    VOICE_ID_RIGHT    = 4,
    VOICE_ID_STOP     = 5,
    VOICE_ID_FASTER   = 6,
    VOICE_ID_SLOWER   = 7,
};

static const struct { int id; const char *phrase; } k_commands[] = {
    { VOICE_ID_FORWARD,  "forward"  },
    { VOICE_ID_BACKWARD, "backward" },
    { VOICE_ID_LEFT,     "left"     },
    { VOICE_ID_RIGHT,    "right"    },
    { VOICE_ID_STOP,     "stop"     },
    { VOICE_ID_FASTER,   "faster"   },
    { VOICE_ID_SLOWER,   "slower"   },
};
#define K_COMMANDS_COUNT ((int)(sizeof k_commands / sizeof k_commands[0]))

/* Listening window: how long we accept a follow-up verb after "Hi ESP".
 * MultiNet's own timeout enforces this (passed to multinet->create). */
#define LISTEN_WINDOW_MS 5000

/* Motor speed range (% duty). faster/slower shift by SPEED_STEP inside
 * [SPEED_MIN, SPEED_MAX]. The value persists across wakes so "faster"
 * once, then "forward" later, uses the new speed. */
#define SPEED_DEFAULT 60
#define SPEED_MIN     20
#define SPEED_MAX    100
#define SPEED_STEP    20

static int s_speed_pct = SPEED_DEFAULT;

/* Chassis wiring polarity: sign we apply to speed so that voice
 * "forward" physically drives the rover forward. On this build a
 * negative bb_motor_set value drives forward — matches the web UI's
 * ↑ arrow, which sends d(-1). Flip to +1 if you rewire the H-bridge
 * or the motor wires. */
#define FWD_SIGN  (-1)

/* Steering maneuver: left/right point the servo away from centre and
 * drive the motor forward for STEER_HOLD_MS, then a one-shot esp_timer
 * fires the recovery — servo back to 90° (centre) + motor stop. */
#define SERVO_CHANNEL_STEER  1
#define SERVO_DEG_CENTER     90
#define SERVO_DEG_LEFT       60
#define SERVO_DEG_RIGHT     120
#define STEER_HOLD_MS       1500

static esp_timer_handle_t s_steer_timer = NULL;

static uint16_t servo_deg_to_us(int deg)
{
    /* Matches cmd_servo's formula (main.c): 0°=500 µs .. 180°=2500 µs. */
    if (deg < 0)   deg = 0;
    if (deg > 180) deg = 180;
    return (uint16_t)(SERVO_MIN_US + (SERVO_MAX_US - SERVO_MIN_US) * deg / 180);
}

/* --- Module state ------------------------------------------------------ */

static volatile bool          s_running       = false;
static srmodel_list_t         *s_models       = NULL;
static afe_config_t           *s_afe_config   = NULL;
static const esp_afe_sr_iface_t *s_afe_iface  = NULL;
static esp_afe_sr_data_t      *s_afe_data     = NULL;
static const esp_mn_iface_t   *s_mn_iface     = NULL;
static model_iface_data_t     *s_mn_data      = NULL;
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

/* Post-wake listening state — only the fetch task touches it. */
static bool     s_listening          = false;

/* Diagnostics — non-zero counters + ESP_OK last codes mean the tasks
 * are looping cleanly. */
static volatile uint32_t s_feed_iters       = 0;
static volatile uint32_t s_fetch_iters      = 0;
static volatile uint32_t s_wake_count       = 0;
static volatile uint32_t s_cmd_count        = 0;
static volatile uint32_t s_timeout_count    = 0;
static volatile int      s_last_fetch_ret   = 0;
static volatile int      s_last_wake_state  = 0;
static volatile int      s_last_mn_state    = 0;
static volatile int      s_last_cmd_id      = 0;

/* --- Command → motor mapping ------------------------------------------ */

static void publish_led(led_state_t state)
{
    command_t c = { .id = CMD_LED_STATE, .source = SRC_VOICE };
    c.as.led_state.state = state;
    (void)command_bus_publish(&c);
}

static void publish_servo(int deg)
{
    command_t c = { .id = CMD_SERVO, .source = SRC_VOICE };
    c.as.servo.channel = SERVO_CHANNEL_STEER;
    c.as.servo.us      = servo_deg_to_us(deg);
    (void)command_bus_publish(&c);
}

static void publish_motor_forward(void)
{
    command_t c = { .id = CMD_MOTOR, .source = SRC_VOICE };
    c.as.motor.left  = (int8_t)(FWD_SIGN * s_speed_pct);
    c.as.motor.right = (int8_t)(FWD_SIGN * s_speed_pct);
    (void)command_bus_publish(&c);
}

static void publish_stop(void)
{
    command_t c = { .id = CMD_STOP, .source = SRC_VOICE };
    (void)command_bus_publish(&c);
}

/* One-shot timer callback (runs in the esp_timer task, not fetch_task).
 * Recover the physical state that "left"/"right" set: servo back to
 * centre and motor stopped. */
static void steer_timer_cb(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "  steer maneuver done — centre + stop");
    publish_servo(SERVO_DEG_CENTER);
    publish_stop();
}

/* Kick off a steering maneuver: servo to <deg>, motor forward, timer
 * restarts fresh so overlapping commands don't leave a stale deadline. */
static void start_steer_maneuver(int deg)
{
    if (!s_steer_timer) return;
    (void)esp_timer_stop(s_steer_timer);   /* no-op if not armed */
    publish_servo(deg);
    publish_motor_forward();
    ESP_ERROR_CHECK(esp_timer_start_once(s_steer_timer,
                                         (uint64_t)STEER_HOLD_MS * 1000));
}

/* Act on a recognized verb. Returns true if the user was heard (LED OK),
 * false if the phrase_id was unmapped (LED NACK). Motor / servo /
 * timer state lives in the helpers above so this stays a pure map. */
static bool publish_verb(int command_id)
{
    switch (command_id) {
    case VOICE_ID_FORWARD: {
        command_t c = { .id = CMD_MOTOR, .source = SRC_VOICE };
        c.as.motor.left = c.as.motor.right = (int8_t)(FWD_SIGN * s_speed_pct);
        (void)command_bus_publish(&c);
        return true;
    }
    case VOICE_ID_BACKWARD: {
        command_t c = { .id = CMD_MOTOR, .source = SRC_VOICE };
        c.as.motor.left = c.as.motor.right = (int8_t)(-FWD_SIGN * s_speed_pct);
        (void)command_bus_publish(&c);
        return true;
    }
    case VOICE_ID_LEFT:
        /* Steer left + drive forward for STEER_HOLD_MS, then a one-shot
         * timer returns the servo to centre and stops the motor. */
        start_steer_maneuver(SERVO_DEG_LEFT);
        return true;
    case VOICE_ID_RIGHT:
        start_steer_maneuver(SERVO_DEG_RIGHT);
        return true;
    case VOICE_ID_STOP:
        /* Cancel any in-flight steer so its recovery doesn't clobber a
         * follow-up command; still centre the servo now. */
        if (s_steer_timer) (void)esp_timer_stop(s_steer_timer);
        publish_servo(SERVO_DEG_CENTER);
        publish_stop();
        return true;
    case VOICE_ID_FASTER:
        if (s_speed_pct + SPEED_STEP <= SPEED_MAX) s_speed_pct += SPEED_STEP;
        ESP_LOGI(TAG, "  speed → %d%%", s_speed_pct);
        return true;
    case VOICE_ID_SLOWER:
        if (s_speed_pct - SPEED_STEP >= SPEED_MIN) s_speed_pct -= SPEED_STEP;
        ESP_LOGI(TAG, "  speed → %d%%", s_speed_pct);
        return true;
    default:
        return false;
    }
}

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

        /* Enter listening on wake-word detection. */
        if (!s_listening && res->wakeup_state == WAKENET_DETECTED) {
            s_wake_count++;
            ESP_LOGI(TAG, "*** WAKE #%lu — Hi ESP heard (idx=%d) — listening ***",
                     (unsigned long)s_wake_count, res->wake_word_index);

            command_t wake = {
                .id     = CMD_VOICE_WAKE,
                .source = SRC_VOICE,
                .as.voice_wake = { .model_index = (uint8_t)res->wake_word_index },
            };
            (void)command_bus_publish(&wake);
            publish_led(LED_STATE_LISTENING);
            s_listening = true;
            /* MultiNet's internal state was reset on last DETECTED / TIMEOUT.
             * No explicit reset needed here — the next detect() call starts
             * the fresh recognition window. */
        }

        /* Feed the AFE-enhanced audio into MultiNet while listening.
         * mn_chunksize == afe_fetch_chunksize (asserted at start). */
        if (s_listening && s_mn_iface && s_mn_data) {
            esp_mn_state_t mn = s_mn_iface->detect(s_mn_data, res->data);
            s_last_mn_state = (int)mn;
            if (mn == ESP_MN_STATE_DETECTED) {
                esp_mn_results_t *r = s_mn_iface->get_results(s_mn_data);
                int cmd_id = (r && r->num > 0) ? r->command_id[0] : 0;
                float prob = (r && r->num > 0) ? r->prob[0]       : 0.0f;
                s_last_cmd_id = cmd_id;
                s_cmd_count++;
                ESP_LOGI(TAG, "  MN: cmd_id=%d prob=%.2f string=\"%s\"",
                         cmd_id, prob, r ? r->string : "");
                bool ok = publish_verb(cmd_id);
                publish_led(ok ? LED_STATE_OK : LED_STATE_NACK);
                s_listening = false;
            } else if (mn == ESP_MN_STATE_TIMEOUT) {
                s_timeout_count++;
                ESP_LOGI(TAG, "  MN: window timeout (no verb heard)");
                publish_led(LED_STATE_NACK);
                s_listening = false;
            }
            /* ESP_MN_STATE_DETECTING: still listening, do nothing. */
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
    int afe_fetch       = s_afe_iface->get_fetch_chunksize(s_afe_data);
    ESP_LOGI(TAG, "AFE ready: feed=%d s/ch × %d ch @ %d Hz  fetch=%d",
             s_afe_feed_samples, s_afe_feed_channels, sr, afe_fetch);
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

    /* 4. Load MultiNet (English command recognizer) and register our
     *    7-verb vocabulary. filter by prefix "mn" + language "en" picks
     *    the CONFIG_SR_MN_EN_MULTINET7_QUANT model. */
    char *mn_name = esp_srmodel_filter(s_models, ESP_MN_PREFIX, ESP_MN_ENGLISH);
    if (!mn_name) {
        ESP_LOGE(TAG, "no English MultiNet model in partition — "
                       "check CONFIG_SR_MN_EN_MULTINET7_QUANT");
        return ESP_ERR_NOT_FOUND;
    }
    s_mn_iface = esp_mn_handle_from_name(mn_name);
    if (!s_mn_iface) {
        ESP_LOGE(TAG, "esp_mn_handle_from_name(%s) returned NULL", mn_name);
        return ESP_FAIL;
    }
    s_mn_data = s_mn_iface->create(mn_name, LISTEN_WINDOW_MS);
    if (!s_mn_data) {
        ESP_LOGE(TAG, "MultiNet create failed");
        return ESP_FAIL;
    }
    int mn_chunk = s_mn_iface->get_samp_chunksize(s_mn_data);
    if (mn_chunk != afe_fetch) {
        ESP_LOGW(TAG, "AFE fetch=%d ≠ MN chunk=%d — MN feed misalignment risk",
                 afe_fetch, mn_chunk);
    }
    ESP_ERROR_CHECK(esp_mn_commands_alloc(s_mn_iface, s_mn_data));
    for (int i = 0; i < K_COMMANDS_COUNT; i++) {
        esp_err_t er = esp_mn_commands_add(k_commands[i].id, k_commands[i].phrase);
        if (er != ESP_OK) {
            ESP_LOGW(TAG, "  cmd_add(%d,\"%s\") -> %d",
                     k_commands[i].id, k_commands[i].phrase, er);
        }
    }
    esp_mn_error_t *cmd_err = esp_mn_commands_update();
    if (cmd_err && cmd_err->num > 0) {
        for (int i = 0; i < cmd_err->num; i++) {
            ESP_LOGW(TAG, "  MN rejected phrase: \"%s\"",
                     cmd_err->phrases[i]->string);
        }
    }
    ESP_LOGI(TAG, "MultiNet ready: model=%s window=%d ms chunk=%d",
             mn_name, LISTEN_WINDOW_MS, mn_chunk);

    /* 5. Start audio_capture on the LEFT slot (where the ES8311 puts
     *    the mono ADC per the live capture) into our own queue.
     *
     *    Known first-after-boot stall: the very first audio_capture_start
     *    on a fresh chip returns OK but the I²S DMA never advances until
     *    it's torn down and started again — a codec/clock warmup thing
     *    that's cheap to work around by just doing that start-stop-start
     *    dance up front. Without this, the wake_word auto-start at boot
     *    lands on the stall and AFE's fetch loop dry-fires with the
     *    "Ringbuffer of AFE is empty" warning forever. */
    s_audio_queue = xQueueCreate(8, AUDIO_CAPTURE_FRAME_BYTES);
    if (!s_audio_queue) return ESP_ERR_NO_MEM;

    audio_capture_config_t cfg = { .slot = AUDIO_CAPTURE_SLOT_LEFT };
    esp_err_t r = audio_capture_start_ex(s_audio_queue, &cfg);
    if (r == ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(150));
        (void)audio_capture_stop();
        vTaskDelay(pdMS_TO_TICKS(50));
        /* Drain any stale frames the warmup pass left in the queue. */
        int16_t drain[AUDIO_CAPTURE_FRAME_SAMPLES];
        while (xQueueReceive(s_audio_queue, drain, 0) == pdTRUE) {}
        r = audio_capture_start_ex(s_audio_queue, &cfg);
    }
    if (r != ESP_OK) {
        ESP_LOGE(TAG, "audio_capture_start_ex: %d", r);
        vQueueDelete(s_audio_queue); s_audio_queue = NULL;
        return r;
    }

    /* 6. Steer-maneuver deadline timer (armed by start_steer_maneuver,
     *    fires steer_timer_cb after STEER_HOLD_MS to recover). */
    const esp_timer_create_args_t targs = {
        .callback = steer_timer_cb,
        .name     = "wake_steer",
    };
    ESP_ERROR_CHECK(esp_timer_create(&targs, &s_steer_timer));

    /* 7. Reset counters, launch tasks. Fetch first so it's already
     *    blocked on AFE before feed starts pushing. Both on core 1
     *    to keep protocol / Wi-Fi work on core 0. */
    s_feed_iters = s_fetch_iters = 0;
    s_wake_count = s_cmd_count = s_timeout_count = 0;
    s_last_fetch_ret = s_last_wake_state = s_last_mn_state = s_last_cmd_id = 0;
    s_listening = false;
    s_running   = true;

    /* Stacks in PSRAM — MultiNet's create() eats internal DRAM so a
     * regular xTaskCreatePinnedToCore for the 6 KB fetch stack fails
     * (empirically pdFAIL). PSRAM stacks are slower per instruction
     * (~2× access latency) but we're not in a tight ISR here — voice
     * decode already tolerates that. Requires
     * CONFIG_SPIRAM_ALLOW_STACK_EXTERNAL_MEMORY=y (checked). */
    const uint32_t stack_caps = MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT;
    if (xTaskCreatePinnedToCoreWithCaps(fetch_task, "wake_fetch", 6144, NULL,
                                        20, &s_fetch_task, 1, stack_caps)
        != pdPASS) {
        ESP_LOGE(TAG, "fetch_task create failed");
        s_running = false;
        audio_capture_stop();
        return ESP_ERR_NO_MEM;
    }
    if (xTaskCreatePinnedToCoreWithCaps(feed_task, "wake_feed", 4096, NULL,
                                        19, &s_feed_task, 1, stack_caps)
        != pdPASS) {
        ESP_LOGE(TAG, "feed_task create failed");
        s_running = false;
        vTaskDelay(pdMS_TO_TICKS(300));
        audio_capture_stop();
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "wake-word + MultiNet ready — say \"Hi ESP\", then "
                   "forward | backward | left | right | stop | faster | slower");
    return ESP_OK;
}

esp_err_t wake_word_stop(void)
{
    if (!s_running) return ESP_OK;

    s_running = false;
    /* Give both tasks a read-timeout to notice and vTaskDelete themselves. */
    vTaskDelay(pdMS_TO_TICKS(400));

    audio_capture_stop();

    if (s_steer_timer) {
        (void)esp_timer_stop(s_steer_timer);
        (void)esp_timer_delete(s_steer_timer);
        s_steer_timer = NULL;
    }

    if (s_mn_data && s_mn_iface) {
        s_mn_iface->destroy(s_mn_data);
        s_mn_data = NULL;
    }
    (void)esp_mn_commands_free();

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
    s_mn_iface  = NULL;
    return ESP_OK;
}

bool wake_word_is_running(void) { return s_running; }

void wake_word_diag_print(void)
{
    printf("wake-diag: running=%d listening=%d speed=%d%%\n",
           (int)s_running, (int)s_listening, s_speed_pct);
    printf("  wakes=%lu commands=%lu timeouts=%lu\n",
           (unsigned long)s_wake_count,
           (unsigned long)s_cmd_count,
           (unsigned long)s_timeout_count);
    printf("  feed_task:  iters=%lu\n", (unsigned long)s_feed_iters);
    printf("  fetch_task: iters=%lu  last_ret=%d  last_wake=%d  last_mn=%d  last_cmd_id=%d\n",
           (unsigned long)s_fetch_iters,
           s_last_fetch_ret, s_last_wake_state, s_last_mn_state, s_last_cmd_id);
    if (s_afe_iface && s_afe_data) {
        printf("  afe: feed_chunk=%d s/ch × %d ch @ %d Hz\n",
               s_afe_feed_samples, s_afe_feed_channels,
               s_afe_iface->get_samp_rate(s_afe_data));
    }
}
