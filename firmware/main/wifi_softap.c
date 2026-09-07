#include "wifi_softap.h"

#include <string.h>
#include <math.h>

#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "wifi_ap";

/* Board helpers (defined in main.c, non-static). Used by the safety-mode
 * path when the last Wi-Fi client disconnects: stop the motors, park
 * the wheels straight, and breathe the LED as a "no operator connected"
 * indicator until someone comes back. */
extern void led_set(uint8_t r, uint8_t g, uint8_t b);
extern void servo_set_deg(int deg);
extern int  bb_motor_set_public(int m1_signed, int m2_signed);

static volatile int  s_clients          = 0;    /* connected station count */
static volatile bool s_safety_active    = false;
static TaskHandle_t  s_breathing_task   = NULL;

static void breathing_task(void *arg)
{
    (void)arg;
    /* Slow red breath — ~2 s fade-in, ~2 s fade-out. Bails out
     * immediately when a client reconnects. */
    while (s_safety_active) {
        for (int b = 0; b <= 80 && s_safety_active; b += 4) {
            led_set((uint8_t)b, 0, 0);
            vTaskDelay(pdMS_TO_TICKS(50));
        }
        for (int b = 80; b >= 0 && s_safety_active; b -= 4) {
            led_set((uint8_t)b, 0, 0);
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }
    led_set(0, 0, 0);
    s_breathing_task = NULL;
    vTaskDelete(NULL);
}

static void enter_safety(void)
{
    if (s_safety_active) return;
    ESP_LOGW(TAG, "no clients — SAFETY: motors stopped, servo centred, LED breathing");
    (void)bb_motor_set_public(0, 0);
    servo_set_deg(90);
    s_safety_active = true;
    if (!s_breathing_task) {
        xTaskCreatePinnedToCore(breathing_task, "safe_led", 2048, NULL, 3,
                                &s_breathing_task, tskNO_AFFINITY);
    }
}

static void exit_safety(void)
{
    if (!s_safety_active) return;
    ESP_LOGI(TAG, "client back — SAFETY cleared");
    s_safety_active = false;
    /* breathing_task notices s_safety_active == false on its next tick
     * and turns the LED off + self-deletes; no explicit join needed. */
}

#define AP_SSID       "OpenQuarkyRover"
#define AP_PASSWORD   "quarky1234"
#define AP_CHANNEL    1
#define AP_MAX_CONN   4

static void on_wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg; (void)base; (void)data;
    if (id == WIFI_EVENT_AP_STACONNECTED) {
        s_clients++;
        ESP_LOGI(TAG, "client joined (total=%d)", s_clients);
        exit_safety();
    } else if (id == WIFI_EVENT_AP_STADISCONNECTED) {
        if (s_clients > 0) s_clients--;
        ESP_LOGI(TAG, "client left   (total=%d)", s_clients);
        if (s_clients == 0) enter_safety();
    }
}

esp_err_t rover_wifi_ap_start(void)
{
    /* NVS is required by esp_wifi to persist calibration data. Init once;
     * recover from a full/no-free-pages flash by erasing NVS and retrying. */
    esp_err_t nvs = nvs_flash_init();
    if (nvs == ESP_ERR_NVS_NO_FREE_PAGES || nvs == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *ap_netif = esp_netif_create_default_wifi_ap();
    (void)ap_netif;

    wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init_cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, on_wifi_event, NULL, NULL));

    wifi_config_t ap_cfg = {
        .ap = {
            .ssid           = AP_SSID,
            .ssid_len       = sizeof(AP_SSID) - 1,
            .channel        = AP_CHANNEL,
            .password       = AP_PASSWORD,
            .max_connection = AP_MAX_CONN,
            .authmode       = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg        = { .required = false },
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    esp_netif_ip_info_t ip;
    esp_netif_get_ip_info(ap_netif, &ip);
    ESP_LOGI(TAG, "Wi-Fi soft-AP up: SSID=\"%s\" pass=\"%s\" ip=" IPSTR,
             AP_SSID, AP_PASSWORD, IP2STR(&ip.ip));
    return ESP_OK;
}
