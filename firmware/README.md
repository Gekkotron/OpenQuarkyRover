# M1 smoke-test firmware

Bring-up firmware for the STEMpedia Quarky Intellio. Its only job is to
prove we can drive the **RGB LED**, the **S1 servo**, and the **two DC
motors on the Mini Expansion Board** from our own code, and to give us a
way to discover the two placeholder pins at runtime without reflashing.

## What's exercised

| Peripheral | Driver | Pin | Status |
|---|---|---|---|
| Mini-expansion motor 1 | LEDC PWM 5 kHz | GPIO 2 (A1) | **Confirmed** |
| Mini-expansion motor 2 | LEDC PWM 5 kHz | GPIO 1 (A2) | **Confirmed** |
| On-board RGB LED (WS2812B) | RMT via `led_strip` | GPIO **48** | **Confirmed** |
| On-board servo (S1) | LEDC PWM 50 Hz | GPIO **4** | **Placeholder** — verify |

The two placeholders can be changed at runtime — see `led-pin` and
`servo-pin` in the REPL section below.

## Prerequisites

- **ESP-IDF v5.2 or newer.** Already installed at `~/esp/esp-idf` on
  this machine — just source its environment:

  ```sh
  . ~/esp/esp-idf/export.sh
  ```

- Quarky Intellio plugged in via USB-C (device appears as
  `/dev/cu.usbserial-*`).

- Nothing else must hold the serial port. If a previous `esptool` or
  monitor session is still open, close it first:

  ```sh
  lsof /dev/cu.usbserial-10   # identify holder, if any
  ```

## Build, flash, monitor

From this directory:

```sh
cd firmware
idf.py set-target esp32s3        # first time only
idf.py build
idf.py -p /dev/cu.usbserial-10 flash monitor
```

Exit the monitor with `Ctrl+]`.

**Flashing our firmware overwrites the stock STEMpedia firmware in the
`factory` partition. `user_voice.wav` and the `flags.json` config in the
`vfs` partition are also wiped once we write the flash.** Both were
extracted into `../firmware-backup/extracted/vfs/` earlier. To roll back
to stock, use the restore recipe in `../README.md`.

## REPL commands

At the `quarky>` prompt:

```
help                      list every command
pins                      show current pin assignments
selftest                  run LED colors -> servo sweep -> motor nudges

led <r> <g> <b>           set LED colour (each 0..255)
led-off                   LED off
led-pin <gpio>            move the LED driver to a different GPIO

servo <deg>               set servo angle 0..180
servo-pin <gpio>          move the servo signal to a different GPIO

motor <m1> <m2>           set motor speeds 0..100 (percent duty)
stop                      motors off
```

`led-pin` and `servo-pin` are the pin-discovery workflow: try
candidates, run `led 255 0 0` or `servo 90` after each, and note which
GPIO physically responds. Once identified, edit `main/pins.h` so the
right pin is baked in.

## Suggested first-session sequence

1. `idf.py flash monitor` → wait for `REPL ready` and the `quarky>` prompt.
2. `pins` — confirm pin config on-screen matches expectations.
3. `motor 30 0`, `motor 0 30`, `stop` — verify each motor spins on its
   own (Mini Expansion single-direction, so forward only).
4. `led 255 0 0` on the placeholder pin. If the LED doesn't light,
   iterate: `led-pin 47`, `led 255 0 0`, `led-pin 38`, `led 255 0 0`, …
   until you find the physical LED. Note the winning GPIO.
5. Same drill for the servo: `servo 0`, `servo 90`, `servo 180`. If
   nothing moves, iterate `servo-pin <n>` through candidates
   (16, 17, 18, 21, 41, 42, 5, 6, 7) until the horn swings.
6. `selftest` — full sequence.
7. Update `main/pins.h` with the two discovered numbers; reflash so the
   defaults are correct on the next boot.

## Safety notes

- ESP32-S3-WROOM-1-N16R8 uses **GPIO 26..37** internally for QSPI flash
  and octal PSRAM. **Never** target those pins with `led-pin` or
  `servo-pin` — the board will hang.
- Strapping pins (0, 3, 45, 46) can be used but may glitch during boot.
- Motor commands never enter recovery-uncanny territory: with the Mini
  Expansion, both channels are one-directional PWM; there is no phase
  shorting risk.

## Not in this firmware yet

Everything else in the roadmap — Wi-Fi, web UI, camera, ES8311
audio pipeline, ESP-SR voice, TFLite adaptive layer, OTA. Those arrive
in M2..M4, each with its own spec + PR (see `../README.md`).
