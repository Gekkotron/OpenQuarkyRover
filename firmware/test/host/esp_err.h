#pragma once
/*
 * Host-build shim for ESP-IDF's <esp_err.h>. The real header pulls in a
 * cascade of target-only dependencies (sdkconfig.h, sys/reent.h, ...);
 * we only need the type + a couple of constants for module code under
 * test to compile natively.
 */
typedef int esp_err_t;
#define ESP_OK               0
#define ESP_FAIL            -1
#define ESP_ERR_INVALID_ARG  0x102
