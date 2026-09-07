#pragma once

/*
 * Wake-word detector — WakeNet9 "Hi ESP" over the ES8311 mic path.
 *
 * Runs esp-sr's AFE_SR pipeline (WebRTC NS + VAD, WakeNet inference) on
 * the mono 16 kHz stream that audio_capture already produces. On a
 * detection: flashes the on-board LED green via led_indicator, prints
 * one line to the console, and increments a counter that wake-diag can
 * read back.
 *
 * Nothing on the command bus yet — this is a mic-liveness test, not the
 * full voice pipeline (that's a later task; the wake event is the input
 * to it).
 */

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

esp_err_t wake_word_start(void);
esp_err_t wake_word_stop(void);
bool      wake_word_is_running(void);

/*
 * Print current feed/fetch task iteration counts, last errors, and
 * total detections to the console. Zero counts mean the corresponding
 * task never got past its first blocking call.
 */
void wake_word_diag_print(void);
