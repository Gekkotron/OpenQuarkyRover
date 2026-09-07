#pragma once

/*
 * Wi-Fi soft-AP for the browser control UI. The rover advertises its own
 * network so the phone / laptop connects directly with no router in the
 * middle — matches the "no cloud, no companion device" long-term goal.
 * SSID and password are compile-time constants (see the .c). No STA mode
 * for now; that'd come later if the user wants to join a home Wi-Fi.
 */

#include "esp_err.h"

/*
 * One-shot bring-up: initialise NVS + esp_netif + esp_event, configure
 * WPA2 soft-AP, start Wi-Fi. Safe to call once from app_main after
 * peripheral drivers are up. Prints the SSID/password/IP over the
 * console so the user can find the network without guessing.
 */
esp_err_t rover_wifi_ap_start(void);
