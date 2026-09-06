# M3 — Voice control implementation plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Ship the M3 milestone — on-device wake-word ("Hi Quarky") gated command spotting for seven English drive commands, dispatched through a shared command bus to the existing M1 motor/servo/LED handlers, with RGB LED feedback for listening/ok/nack states.

**Architecture:** ES8311 codec (bit-bang I²C control + I²S mono 16 kHz RX) feeds Espressif ESP-SR (AFE + WakeNet + MultiNet-EN) on core 1. M1 REPL is refactored so both REPL and voice publish `command_t` values to a single FreeRTOS-queue dispatcher on core 0. LED indicator becomes a small FSM that arbitrates voice states against user-set colors. Custom "Hi Quarky" model is ordered from Espressif Skainet Studio on day 1 (long lead-time) and drops in as a single-file swap when it arrives — development proceeds against stock `wn9_hiesp`.

**Tech Stack:** ESP-IDF v5.3, `espressif/esp-sr` v2.0.0 (via `idf_component.yml`), FreeRTOS, native I²S/`i2s_std` driver, WS2812B via existing RMT `led_strip`, bit-bang I²C reused from M1 for ES8311 control, Unity for host unit tests.

**Spec:** `docs/superpowers/specs/2026-09-06-m3-voice-control-design.md`

## Global Constraints

Copied verbatim from the spec. **Every task implicitly includes these.**

- **ESP-IDF v5.3** — current M1 baseline. Do not bump the version.
- **`esp-sr` pinned to `==2.0.0`** in `idf_component.yml`. Models are runtime-coupled — a floating dependency is a silent-breakage trap.
- **PSRAM required.** `sdkconfig` must have `CONFIG_SPIRAM=y`, `CONFIG_SPIRAM_MODE_OCT=y`, `CONFIG_SPIRAM_USE_MALLOC=y`. Verify before any ESP-SR work.
- **All I²C to the ES8311 uses bit-bang** (`i2c_bitbang` module), never the ESP-IDF `i2c_master` peripheral. Same failure mode as the M1 motor bus (see memory `espidf-i2c-master-broken`).
- **Audio format is 16 kHz, 16-bit, mono, little-endian PCM** end-to-end. Anything else costs a resample.
- **Git commit identity for this repo is `Gekkotron`** (email `60887050+Gekkotron@users.noreply.github.com`). Every commit uses `git -c user.name=Gekkotron -c user.email=60887050+Gekkotron@users.noreply.github.com commit …` — never modify `.git/config` (git safety protocol).
- **Do not touch `firmware-backup/`** — the stock backup must remain byte-identical to what shipped.
- **Do not write TODO/TBD/placeholder** anywhere in shipped code — if a value is unknown, block the task on discovering it, don't stub it.
- **Every commit message ends with** `Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>`.

---

## File map

Files created or modified across the whole plan, one line each:

### Created

| Path | Task | Responsibility |
|---|---|---|
| `firmware/main/pin_map.h` | 1 | ES8311 GPIO constants (SDA, SCL, MCLK, BCLK, LRCK, DIN) |
| `firmware/main/i2c_bitbang.h` | 2 | Public bit-bang I²C API |
| `firmware/main/i2c_bitbang.c` | 2 | Bit-bang I²C implementation moved from `main.c` |
| `firmware/main/es8311.h` | 3 | ES8311 codec public API |
| `firmware/main/es8311.c` | 3 | ES8311 register init, gain, start/stop |
| `firmware/main/audio_capture.h` | 4 | I²S RX capture public API |
| `firmware/main/audio_capture.c` | 4 | `i2s_std` config, DMA, producer task, PCM queue |
| `firmware/main/command_bus.h` | 5 | `command_t` type, publish API |
| `firmware/main/command_bus.c` | 5 | Queue, dispatcher task, handler switch |
| `firmware/main/led_indicator.h` | 6 | LED FSM public API |
| `firmware/main/led_indicator.c` | 6 | FSM, arbitration, WS2812B write |
| `firmware/main/voice_pipeline.h` | 9 | Voice pipeline public API |
| `firmware/main/voice_pipeline.c` | 9,10 | ESP-SR wrapper, wake + command events |
| `firmware/partitions.csv` | 8 | New partition table (adds `model` partition) |
| `firmware/main/idf_component.yml` | 8 | Declares `espressif/esp-sr==2.0.0` |
| `firmware/model/wn9_hiesp.bin` | 8 | Stock wake word model (placeholder for Hi Quarky) |
| `firmware/model/mn7_en/mn7_en.bin` | 8 | MultiNet English model |
| `firmware/model/mn7_en/mn7_en.index` | 8 | MultiNet English index |
| `firmware/model/wn9_hiquarky.bin` | 12 | Skainet-delivered custom wake word |
| `firmware/test/host/Makefile` | 3 | Host test runner (Unity, standalone `make`) |
| `firmware/test/host/es8311_init_test.c` | 3 | Register init sequence table test |
| `firmware/test/host/command_bus_test.c` | 5 | Command struct + dispatcher routing tests |
| `firmware/test/host/led_indicator_test.c` | 6 | FSM state-transition + arbitration tests |
| `docs/model-provenance.md` | 8 | Provenance table (source, version, license) for every `.bin` |

### Modified

| Path | Task | What changes |
|---|---|---|
| `firmware/main/main.c` | 1,2,3,4,5,6,7 | Adds/removes REPL commands, extracts bb-I²C, per-verb parsers publish to command bus, LED goes through `led_indicator` |
| `firmware/main/CMakeLists.txt` | 2,3,4,5,6,8,9 | New `SRCS` entries per task; `spiffs_create_partition_image` for model partition |
| `firmware/sdkconfig.defaults` | 8 | `CONFIG_PARTITION_TABLE_CUSTOM=y`, filename, PSRAM verifications |
| `scripts/upload.sh` | 8 | Add `--erase-all` flag pass-through |
| `firmware/README.md` | 11 | Refresh (currently stale — describes pre-TLC59108 motor drive); add "Voice control" section |
| `README.md` (root) | 11 | Add `## Voice control (M3)` section, flashing note about `--erase-all`, close out open questions this milestone resolves |

---

## Task 0: Order Skainet Studio custom wake word (external — day 1)

> **Status: DEFERRED (2026-09-06).** Decision: ship M3 against the stock `wn9_hiesp` wake word ("Hi ESP") to avoid the Skainet order cost and 2–4 week lead time. Task 12 (wake word swap) becomes future work — the plan is already structured so the swap is a single-file drop-in whenever the order is placed. **Skip this task; start with Task 1.**

Not code. Kicks off the critical-path external dependency on day 1 so it never blocks the ship.

**Files:** none (append entry to spec's External Dependencies section only).

**Interfaces:**
- Consumes: nothing.
- Produces: order number + expected delivery date, recorded in spec.

- [ ] **Step 1: Place the Skainet Studio custom wake word order**

Open [https://www.espressif.com/en/products/software/esp-skainet](https://www.espressif.com/en/products/software/esp-skainet) → *Custom Wake Word Service*. Submit:

- Wake phrase: `Hi Quarky`
- Language: English (US)
- ESP-SR runtime: `2.0.0` (must match the `idf_component.yml` pin in Task 8)
- Target chip: ESP32-S3
- Delivery format: `wn9_hiquarky.bin`

- [ ] **Step 2: Log the order in the spec**

Append to `docs/superpowers/specs/2026-09-06-m3-voice-control-design.md` under a new "External dependencies — orders placed" section: order number, submission date, quoted delivery date, quoted price. Commit:

```bash
git -c user.name=Gekkotron -c user.email=60887050+Gekkotron@users.noreply.github.com \
  commit -am "M3 spec: log Skainet Hi Quarky order

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>"
```

---

## Task 1: Pin discovery — ES8311 pins into `pin_map.h`

Deliverable: six confirmed GPIO constants + a permanent `es-verify` REPL diagnostic. Blocks every downstream task.

**Files:**
- Create: `firmware/main/pin_map.h`
- Modify: `firmware/main/main.c` (add `es-scan` throwaway command + `es-verify` permanent command)
- Modify: `firmware/main/CMakeLists.txt` (no changes needed yet — `main.c` still the only SRC)

**Interfaces:**
- Consumes: existing `bb_probe(sda, scl, addr)`, `bb_read_reg(sda, scl, addr, reg)` from `main.c` (Task 2 extracts them).
- Produces:
  ```c
  // pin_map.h
  #define ES8311_I2C_SDA   GPIO_NUM_<n>
  #define ES8311_I2C_SCL   GPIO_NUM_<n>
  #define ES8311_I2S_MCLK  GPIO_NUM_<n>
  #define ES8311_I2S_BCLK  GPIO_NUM_<n>
  #define ES8311_I2S_LRCK  GPIO_NUM_<n>
  #define ES8311_I2S_DIN   GPIO_NUM_<n>
  ```

- [ ] **Step 1: Add the candidate GPIO list to `main.c`**

Above the REPL command implementations, add:

```c
/* Candidate GPIOs for ES8311 pin discovery.
 * Excludes: M1-owned (0, 1, 2, 7, 48), USB/UART bridge (19, 20),
 * strapping (3, 45, 46 — 0 already excluded), and SPI0/PSRAM (26–32).
 */
static const gpio_num_t ES_CAND[] = {
    4, 5, 6, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18,
    21, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 47
};
static const size_t ES_CAND_COUNT = sizeof(ES_CAND) / sizeof(ES_CAND[0]);
```

- [ ] **Step 2: Add the `es-scan` REPL command**

Register a new REPL verb in the existing `esp_console_cmd_register()` block:

```c
static int cmd_es_scan(int argc, char **argv) {
    (void)argc; (void)argv;
    for (size_t mi = 0; mi < ES_CAND_COUNT; mi++) {
        gpio_num_t mclk = ES_CAND[mi];
        // Drive 4 MHz square wave on MCLK candidate via LEDC
        ledc_timer_config_t t = {
            .speed_mode = LEDC_LOW_SPEED_MODE, .duty_resolution = LEDC_TIMER_1_BIT,
            .timer_num = LEDC_TIMER_2, .freq_hz = 4000000, .clk_cfg = LEDC_AUTO_CLK,
        };
        ledc_timer_config(&t);
        ledc_channel_config_t c = {
            .gpio_num = mclk, .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = LEDC_CHANNEL_2, .timer_sel = LEDC_TIMER_2,
            .duty = 1, .hpoint = 0,
        };
        ledc_channel_config(&c);

        for (size_t si = 0; si < ES_CAND_COUNT; si++) {
            for (size_t ci = 0; ci < ES_CAND_COUNT; ci++) {
                gpio_num_t sda = ES_CAND[si], scl = ES_CAND[ci];
                if (sda == scl || sda == mclk || scl == mclk) continue;
                if (!bb_probe(sda, scl, 0x18)) continue;
                uint8_t id = 0xFF;
                if (bb_read_reg(sda, scl, 0x18, 0xFD, &id) == ESP_OK && id == 0x83) {
                    ESP_LOGI("es-scan", "CONFIRMED: MCLK=%d SDA=%d SCL=%d (ID=0x83)",
                             mclk, sda, scl);
                    ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, 0);
                    return 0;
                }
            }
        }
        ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, 0);
    }
    ESP_LOGW("es-scan", "no ES8311 detected on any (MCLK, SDA, SCL) triple");
    return 1;
}
```

Register with:
```c
esp_console_cmd_t es_scan_cmd = { .command="es-scan", .help="Brute-force ES8311 pin discovery",
    .hint=NULL, .func=&cmd_es_scan };
esp_console_cmd_register(&es_scan_cmd);
```

`bb_read_reg` must accept an out-parameter for the byte read. If the existing signature doesn't, extend it in `main.c` (this survives into `i2c_bitbang.h` in Task 2).

- [ ] **Step 3: Build, flash, and run `es-scan`**

```sh
cd firmware && idf.py build flash monitor
# At quarky> prompt:
quarky> es-scan
```

Expected duration: up to ~15 minutes. On success, note the confirmed `(MCLK, SDA, SCL)` triple. On failure, proceed to Step 4.

- [ ] **Step 4 (fallback if scan finds nothing): Stock-firmware introspection**

```sh
# From repo root
esptool --chip esp32s3 -p /dev/cu.usbserial-<N> -b 921600 \
    write-flash 0 firmware-backup/stock_firmware.bin

# Open serial monitor at 115200, tap Ctrl+C during boot to catch MicroPython
screen /dev/cu.usbserial-<N> 115200
# At the MicroPython REPL:
>>> import intellioConstants
>>> print(vars(intellioConstants))
# Copy the six ES8311_* / codec pin values.
```

If Ctrl+C doesn't drop to REPL, extract `intellioConstants` from bytecode:

```sh
python3 -c "
data = open('firmware-backup/factory.bin','rb').read()
# find 'intellioConstants' string, walk back to mp_frozen_mpy_data
i = data.find(b'intellioConstants')
print(f'string at offset 0x{i:x}')
"
# Then use mpy-tool.py from ~/micropython/tools/ to disassemble the referenced module.
```

Then reflash M1:
```sh
cd firmware && idf.py -p /dev/cu.usbserial-<N> flash monitor
```

- [ ] **Step 5: Write `firmware/main/pin_map.h` with the confirmed pins**

```c
/* ES8311 codec pin map on the Quarky Intellio.
 * Discovered by: <es-scan | stock REPL | bytecode disassembly>  ← delete losers
 * Date: <YYYY-MM-DD>
 */
#pragma once
#include "driver/gpio.h"

#define ES8311_I2C_SDA   GPIO_NUM_<n>
#define ES8311_I2C_SCL   GPIO_NUM_<n>
#define ES8311_I2S_MCLK  GPIO_NUM_<n>
#define ES8311_I2S_BCLK  GPIO_NUM_<n>
#define ES8311_I2S_LRCK  GPIO_NUM_<n>   /* aka WS */
#define ES8311_I2S_DIN   GPIO_NUM_<n>   /* mic → ESP32 */
```

- [ ] **Step 6: Replace `es-scan` with permanent `es-verify` diagnostic**

Delete the `es-scan` command and its `ES_CAND[]` table from `main.c`. Add:

```c
#include "pin_map.h"

static int cmd_es_verify(int argc, char **argv) {
    (void)argc; (void)argv;
    // Drive MCLK on the confirmed pin
    ledc_timer_config_t t = { .speed_mode=LEDC_LOW_SPEED_MODE, .duty_resolution=LEDC_TIMER_1_BIT,
        .timer_num=LEDC_TIMER_2, .freq_hz=4000000, .clk_cfg=LEDC_AUTO_CLK };
    ledc_timer_config(&t);
    ledc_channel_config_t c = { .gpio_num=ES8311_I2S_MCLK, .speed_mode=LEDC_LOW_SPEED_MODE,
        .channel=LEDC_CHANNEL_2, .timer_sel=LEDC_TIMER_2, .duty=1, .hpoint=0 };
    ledc_channel_config(&c);

    uint8_t id = 0, ver = 0;
    esp_err_t r1 = bb_read_reg(ES8311_I2C_SDA, ES8311_I2C_SCL, 0x18, 0xFD, &id);
    esp_err_t r2 = bb_read_reg(ES8311_I2C_SDA, ES8311_I2C_SCL, 0x18, 0xFE, &ver);
    ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, 0);

    if (r1 || r2) { ESP_LOGE("es-verify", "I2C read failed"); return 1; }
    ESP_LOGI("es-verify", "ES8311 product ID=0x%02x (expect 0x83), version=0x%02x", id, ver);
    return (id == 0x83) ? 0 : 1;
}
```

Register it in the same block as other REPL commands.

- [ ] **Step 7: Rebuild, flash, and verify**

```sh
cd firmware && idf.py build flash monitor
quarky> es-verify
```

Expected: `ES8311 product ID=0x83 (expect 0x83), version=0x??`. Non-zero return code = fail; recheck pins.

- [ ] **Step 8: Commit**

```bash
git add firmware/main/pin_map.h firmware/main/main.c
git -c user.name=Gekkotron -c user.email=60887050+Gekkotron@users.noreply.github.com \
  commit -m "M3 task 1: discover ES8311 pins, add es-verify diagnostic

Confirmed by <es-scan | stock REPL | bytecode disassembly>.
Six pin constants committed to pin_map.h.

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>"
```

---

## Task 2: Extract bit-bang I²C into a reusable module

Pure refactor. Move **only the low-level bit-bang I²C primitives** (`bb_write`, `bb_read`, `bb_probe`, `bb_write_reg`, `bb_read_reg` — and any static helpers they use like `bb_delay`, `bb_start`, `bb_stop`) from `main.c` into `i2c_bitbang.{c,h}`. **Do NOT move** the higher-level helpers `bb_motor_set`, `bb_tlc_*`, or `bb_scan` — those are application logic that lives on top of the primitives and stays in `main.c` (they'll be exposed as non-static and given return-type-`int` signatures in Task 7 when the command bus needs to call them). No behavior change. M1 regression tests (motor drive, `bb-scan`, `tlc-init`) still pass.

**Files:**
- Create: `firmware/main/i2c_bitbang.h`
- Create: `firmware/main/i2c_bitbang.c`
- Modify: `firmware/main/main.c` (delete `bb_*` definitions, add `#include "i2c_bitbang.h"`)
- Modify: `firmware/main/CMakeLists.txt` (add `i2c_bitbang.c` to `SRCS`)

**Interfaces:**
- Consumes: nothing new (just moving code).
- Produces (public header):
  ```c
  esp_err_t bb_write(gpio_num_t sda, gpio_num_t scl, uint8_t addr,
                     const uint8_t *data, size_t len);
  esp_err_t bb_read(gpio_num_t sda, gpio_num_t scl, uint8_t addr,
                    uint8_t *data, size_t len);
  esp_err_t bb_write_reg(gpio_num_t sda, gpio_num_t scl, uint8_t addr,
                         uint8_t reg, uint8_t value);
  esp_err_t bb_read_reg(gpio_num_t sda, gpio_num_t scl, uint8_t addr,
                        uint8_t reg, uint8_t *value);
  bool      bb_probe(gpio_num_t sda, gpio_num_t scl, uint8_t addr);
  ```

- [ ] **Step 1: Create `i2c_bitbang.h`**

```c
#pragma once
#include "esp_err.h"
#include "driver/gpio.h"
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

esp_err_t bb_write(gpio_num_t sda, gpio_num_t scl, uint8_t addr,
                   const uint8_t *data, size_t len);
esp_err_t bb_read(gpio_num_t sda, gpio_num_t scl, uint8_t addr,
                  uint8_t *data, size_t len);
esp_err_t bb_write_reg(gpio_num_t sda, gpio_num_t scl, uint8_t addr,
                       uint8_t reg, uint8_t value);
esp_err_t bb_read_reg(gpio_num_t sda, gpio_num_t scl, uint8_t addr,
                      uint8_t reg, uint8_t *value);
bool      bb_probe(gpio_num_t sda, gpio_num_t scl, uint8_t addr);
```

- [ ] **Step 2: Create `i2c_bitbang.c` by moving the I²C primitives out of `main.c`**

Cut ONLY these functions from `main.c` and paste into `i2c_bitbang.c`:

- `bb_write`, `bb_read`, `bb_probe`, `bb_write_reg`, `bb_read_reg` — the public primitives declared in the header, drop `static`.
- Any static helpers they call (typically named `bb_start`, `bb_stop`, `bb_write_byte`, `bb_read_byte`, `bb_delay`, or similar) — keep `static` inside `i2c_bitbang.c`.

Leave in `main.c`:

- `bb_motor_set`, `bb_motor_dir`, and any other motor helpers — application logic on top of the primitives.
- `bb_tlc_init`, `bb_tlc_set`, `bb_tlc_sweep`, `bb_scan`, `tlc_diag` — TLC59108-specific application logic and diagnostics.

Include in `i2c_bitbang.c`:

```c
#include "i2c_bitbang.h"
#include "driver/gpio.h"
#include "rom/ets_sys.h"    // for ets_delay_us — same as main.c uses
```

Keep any internal helpers (like a `bb_delay()` inline or a start/stop routine) `static` inside `i2c_bitbang.c`.

- [ ] **Step 3: Add `#include "i2c_bitbang.h"` to `main.c` and remove the moved definitions**

Delete the moved function bodies from `main.c`; add the include at the top.

- [ ] **Step 4: Update `firmware/main/CMakeLists.txt`**

```cmake
idf_component_register(
    SRCS "main.c" "i2c_bitbang.c"
    INCLUDE_DIRS "."
    REQUIRES driver console esp_driver_ledc esp_driver_gpio esp_driver_rmt esp_driver_i2c esp_adc
)
```

- [ ] **Step 5: Build and regression-test**

```sh
cd firmware && idf.py build flash monitor
quarky> bb-scan            # expect: finds 0x40, 0x48, 0x4b (same as before)
quarky> bb-tlc-init        # expect: OK — TLC59108 initialised via bit-bang @ 0x40
quarky> motor 40 40        # expect: both wheels spin
quarky> stop
quarky> es-verify          # from Task 1, still works
```

If any command changes behavior, the refactor introduced a bug — do NOT proceed. Diff against the previous commit for `main.c`.

- [ ] **Step 6: Commit**

```bash
git add firmware/main/i2c_bitbang.h firmware/main/i2c_bitbang.c \
        firmware/main/main.c firmware/main/CMakeLists.txt
git -c user.name=Gekkotron -c user.email=60887050+Gekkotron@users.noreply.github.com \
  commit -m "M3 task 2: extract bit-bang I2C from main.c

Pure move-refactor; behavior identical. bb-scan / bb-tlc-init / motor
regressions all pass. Prepares ES8311 codec driver (Task 3) to reuse
the same transport.

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>"
```

---

## Task 3: ES8311 codec driver (with host tests)

Ships the codec bring-up: register init sequence, gain control, ID readback. `es-verify` is rewired to go through the new driver instead of raw `bb_read_reg`.

**Files:**
- Create: `firmware/main/es8311.h`, `firmware/main/es8311.c`
- Create: `firmware/test/host/Makefile`, `firmware/test/host/es8311_init_test.c`
- Modify: `firmware/main/main.c` (rewire `es-verify` to `es8311_read_id`)
- Modify: `firmware/main/CMakeLists.txt` (add `es8311.c`)

**Interfaces:**
- Consumes: `bb_write_reg`, `bb_read_reg` from `i2c_bitbang.h` (Task 2), `ES8311_I2C_*`, `ES8311_I2S_MCLK` from `pin_map.h` (Task 1).
- Produces:
  ```c
  // es8311.h
  esp_err_t es8311_init(void);           // Full register init, mic path, ADC power
  esp_err_t es8311_set_mic_gain_db(int gain_db); // -12..+30, clipped
  esp_err_t es8311_read_id(uint8_t *id_out, uint8_t *ver_out);
  esp_err_t es8311_start(void);          // Enable ADC output on I²S
  esp_err_t es8311_stop(void);           // Powerdown ADC
  ```

- [ ] **Step 1: Create `firmware/test/host/Makefile`**

```makefile
# Host-runnable unit tests. No ESP-IDF needed.
CC       := gcc
CFLAGS   := -std=c11 -Wall -Wextra -g -I. -I../../main -Iunity
LDFLAGS  :=

UNITY_SRC := unity/unity.c
TESTS     := es8311_init_test

all: $(TESTS)
	@for t in $(TESTS); do echo "== $$t =="; ./$$t; done

unity/unity.c:
	@mkdir -p unity
	@curl -sL https://raw.githubusercontent.com/ThrowTheSwitch/Unity/v2.5.2/src/unity.c > unity/unity.c
	@curl -sL https://raw.githubusercontent.com/ThrowTheSwitch/Unity/v2.5.2/src/unity.h > unity/unity.h
	@curl -sL https://raw.githubusercontent.com/ThrowTheSwitch/Unity/v2.5.2/src/unity_internals.h > unity/unity_internals.h

es8311_init_test: es8311_init_test.c $(UNITY_SRC)
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS)

clean:
	rm -f $(TESTS)
	rm -rf unity/

.PHONY: all clean
```

- [ ] **Step 2: Write the failing test — `firmware/test/host/es8311_init_test.c`**

```c
#include "unity/unity.h"
#include <stdint.h>
#include <string.h>

/* Mock bit-bang I2C: capture every write for later inspection. */
#define MAX_LOG 256
typedef struct { uint8_t reg, val; } write_t;
static write_t g_log[MAX_LOG];
static size_t  g_log_n = 0;

int mock_bb_write_reg(int sda, int scl, uint8_t addr, uint8_t reg, uint8_t val) {
    (void)sda; (void)scl;
    if (addr != 0x18) return -1;
    if (g_log_n < MAX_LOG) g_log[g_log_n++] = (write_t){reg, val};
    return 0;
}

/* Include the driver under test, redirecting bb_write_reg → mock */
#define bb_write_reg mock_bb_write_reg
#define bb_read_reg(sda, scl, addr, reg, out) (*(out) = 0x83, 0) /* pretend ID = 0x83 */
#include "es8311.c"
#undef bb_write_reg
#undef bb_read_reg

void setUp(void)    { g_log_n = 0; }
void tearDown(void) {}

void test_init_writes_reset_sequence_first(void) {
    TEST_ASSERT_EQUAL(0, es8311_init());
    TEST_ASSERT_GREATER_THAN(2, g_log_n);
    TEST_ASSERT_EQUAL_HEX8(0x00, g_log[0].reg);
    TEST_ASSERT_EQUAL_HEX8(0x1F, g_log[0].val);   // reset asserted
    TEST_ASSERT_EQUAL_HEX8(0x00, g_log[1].reg);
    TEST_ASSERT_EQUAL_HEX8(0x00, g_log[1].val);   // reset released
}

void test_init_configures_16khz_mono_mic_path(void) {
    es8311_init();
    /* Contract: driver must at some point program ADC clock divider for 16 kHz
     * sample rate at MCLK=4 MHz. Datasheet: register 0x02 = ADC clk div;
     * for MCLK=4M / 256 = 15625 Hz ≈ 16 kHz mode, expected value: 0x00 with
     * MCLK ratio config in reg 0x03. */
    bool saw_0x02 = false, saw_0x03 = false;
    for (size_t i = 0; i < g_log_n; i++) {
        if (g_log[i].reg == 0x02) saw_0x02 = true;
        if (g_log[i].reg == 0x03) saw_0x03 = true;
    }
    TEST_ASSERT_TRUE(saw_0x02);
    TEST_ASSERT_TRUE(saw_0x03);
}

void test_init_powers_up_adc(void) {
    es8311_init();
    /* Register 0x17 = ADC power management. Bit 0 = ADC enable.
     * Must be written with bit 0 set at least once. */
    bool adc_enabled = false;
    for (size_t i = 0; i < g_log_n; i++) {
        if (g_log[i].reg == 0x17 && (g_log[i].val & 0x01)) adc_enabled = true;
    }
    TEST_ASSERT_TRUE(adc_enabled);
}

void test_set_mic_gain_db_clips_to_range(void) {
    es8311_init(); g_log_n = 0;
    TEST_ASSERT_EQUAL(0, es8311_set_mic_gain_db(100));   // clip up
    TEST_ASSERT_EQUAL(0, es8311_set_mic_gain_db(-100));  // clip down
    /* Both writes go to the analog PGA register (0x16). */
    TEST_ASSERT_EQUAL(2, g_log_n);
    TEST_ASSERT_EQUAL_HEX8(0x16, g_log[0].reg);
    TEST_ASSERT_EQUAL_HEX8(0x16, g_log[1].reg);
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_init_writes_reset_sequence_first);
    RUN_TEST(test_init_configures_16khz_mono_mic_path);
    RUN_TEST(test_init_powers_up_adc);
    RUN_TEST(test_set_mic_gain_db_clips_to_range);
    return UNITY_END();
}
```

- [ ] **Step 3: Write `es8311.h`**

```c
#pragma once
#include "esp_err.h"
#include <stdint.h>

esp_err_t es8311_init(void);
esp_err_t es8311_set_mic_gain_db(int gain_db);
esp_err_t es8311_read_id(uint8_t *id_out, uint8_t *ver_out);
esp_err_t es8311_start(void);
esp_err_t es8311_stop(void);
```

- [ ] **Step 4: Write `es8311.c` (minimal implementation to pass the tests)**

Reference for the exact register writes: ES8311 datasheet §10 (System Register Description) and ESP-ADF's `components/audio_hal/driver/es8311/es8311.c` (commit hash to record in `docs/model-provenance.md` if you paste any values).

Skeleton (fill register values from datasheet — never make them up):

```c
#include "es8311.h"
#include "i2c_bitbang.h"
#include "pin_map.h"

#define ES8311_ADDR         0x18
#define REG_RESET           0x00
#define REG_CLK_MANAGER1    0x01
#define REG_CLK_MANAGER2    0x02
#define REG_CLK_MANAGER3    0x03
/* ... etc — one #define per register you touch, no magic numbers below */
#define REG_ADC_PGA         0x16
#define REG_ADC_POWER       0x17
#define REG_CHIP_ID_LO      0xFD
#define REG_CHIP_VERSION    0xFE

static esp_err_t w(uint8_t reg, uint8_t val) {
    return bb_write_reg(ES8311_I2C_SDA, ES8311_I2C_SCL, ES8311_ADDR, reg, val);
}

esp_err_t es8311_init(void) {
    ESP_RETURN_ON_ERROR(w(REG_RESET, 0x1F), TAG, "reset assert");
    ESP_RETURN_ON_ERROR(w(REG_RESET, 0x00), TAG, "reset release");

    /* Clock manager: slave, MCLK=4 MHz, sample rate 16 kHz. */
    ESP_RETURN_ON_ERROR(w(REG_CLK_MANAGER1, 0x30), TAG, "clk1");
    ESP_RETURN_ON_ERROR(w(REG_CLK_MANAGER2, 0x10), TAG, "clk2");
    ESP_RETURN_ON_ERROR(w(REG_CLK_MANAGER3, 0x10), TAG, "clk3");
    /* ... all other init writes per datasheet ... */

    /* Default mic PGA to 0 dB. */
    ESP_RETURN_ON_ERROR(es8311_set_mic_gain_db(0), TAG, "pga");

    /* Power up ADC (bit 0 of 0x17). */
    ESP_RETURN_ON_ERROR(w(REG_ADC_POWER, 0x01), TAG, "adc pwr");

    return ESP_OK;
}

esp_err_t es8311_set_mic_gain_db(int gain_db) {
    if (gain_db > 30)  gain_db = 30;
    if (gain_db < -12) gain_db = -12;
    /* PGA step per datasheet §Register 0x16 — map dB to register bits. */
    uint8_t pga = /* dB-to-code table */;
    return w(REG_ADC_PGA, pga);
}

esp_err_t es8311_read_id(uint8_t *id_out, uint8_t *ver_out) {
    esp_err_t r1 = bb_read_reg(ES8311_I2C_SDA, ES8311_I2C_SCL, ES8311_ADDR,
                               REG_CHIP_ID_LO, id_out);
    esp_err_t r2 = bb_read_reg(ES8311_I2C_SDA, ES8311_I2C_SCL, ES8311_ADDR,
                               REG_CHIP_VERSION, ver_out);
    return (r1 == ESP_OK) ? r2 : r1;
}

esp_err_t es8311_start(void) { return ESP_OK; /* ADC on = init did it */ }
esp_err_t es8311_stop(void)  { return w(REG_ADC_POWER, 0x00); }
```

Fill in every `/* ... */` from the datasheet — no invented values.

- [ ] **Step 5: Run host tests, verify they pass**

```sh
cd firmware/test/host && make
```

Expected: all four tests pass. If any fail, fix the driver (never the test — the test encodes datasheet requirements).

- [ ] **Step 6: Rewire `es-verify` in `main.c` to use `es8311_read_id`**

Replace the body of `cmd_es_verify` (from Task 1 Step 6) with:

```c
#include "es8311.h"

static int cmd_es_verify(int argc, char **argv) {
    (void)argc; (void)argv;
    // Drive MCLK on the confirmed pin (same LEDC setup as before)
    /* ... unchanged LEDC MCLK setup ... */

    uint8_t id = 0, ver = 0;
    esp_err_t r = es8311_read_id(&id, &ver);
    ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, 0);

    if (r != ESP_OK) { ESP_LOGE("es-verify", "read failed: %s", esp_err_to_name(r)); return 1; }
    ESP_LOGI("es-verify", "ES8311 id=0x%02x (expect 0x83), version=0x%02x", id, ver);
    return (id == 0x83) ? 0 : 1;
}
```

Add a new REPL verb `es-init` that runs the full init:

```c
static int cmd_es_init(int argc, char **argv) {
    /* MCLK setup as above */
    esp_err_t r = es8311_init();
    ESP_LOGI("es-init", "%s", esp_err_to_name(r));
    return (r == ESP_OK) ? 0 : 1;
}
```

Register both commands.

- [ ] **Step 7: Update `firmware/main/CMakeLists.txt`**

```cmake
SRCS "main.c" "i2c_bitbang.c" "es8311.c"
```

- [ ] **Step 8: Build, flash, verify on hardware**

```sh
cd firmware && idf.py build flash monitor
quarky> es-init        # expect: ESP_OK
quarky> es-verify      # expect: id=0x83
```

- [ ] **Step 9: Commit**

```bash
git add firmware/main/es8311.h firmware/main/es8311.c \
        firmware/main/main.c firmware/main/CMakeLists.txt \
        firmware/test/host/Makefile firmware/test/host/es8311_init_test.c
git -c user.name=Gekkotron -c user.email=60887050+Gekkotron@users.noreply.github.com \
  commit -m "M3 task 3: ES8311 codec driver + host register-init test

Driver exposes init/gain/id/start/stop. Register sequence table-tested
against the datasheet. es-verify rewired through the driver; new es-init
command runs the full init.

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>"
```

---

## Task 4: I²S RX capture + `voice-record` / `voice-stats` diagnostics

Bring up mic capture as a raw PCM stream. Prove end-to-end audio path with an ear-check.

**Files:**
- Create: `firmware/main/audio_capture.h`, `firmware/main/audio_capture.c`
- Modify: `firmware/main/main.c` (add `voice-record` and `voice-stats` commands)
- Modify: `firmware/main/CMakeLists.txt` (add `audio_capture.c`, add `esp_driver_i2s` to REQUIRES)

**Interfaces:**
- Consumes: `ES8311_I2S_*` pins from `pin_map.h`; codec must be initialized (call `es8311_init` before starting capture).
- Produces:
  ```c
  esp_err_t audio_capture_start(QueueHandle_t out_queue); // enqueues 512-sample int16 frames
  esp_err_t audio_capture_stop(void);
  uint32_t  audio_capture_dropped_frames(void);           // running total since start
  ```

- [ ] **Step 1: Write `audio_capture.h`**

```c
#pragma once
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <stdint.h>

#define AUDIO_CAPTURE_FRAME_SAMPLES 512  /* 32 ms @ 16 kHz mono */

esp_err_t audio_capture_start(QueueHandle_t out_queue);
esp_err_t audio_capture_stop(void);
uint32_t  audio_capture_dropped_frames(void);
```

- [ ] **Step 2: Write `audio_capture.c`**

```c
#include "audio_capture.h"
#include "pin_map.h"
#include "driver/i2s_std.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "audio_capture";
static i2s_chan_handle_t s_rx_chan = NULL;
static TaskHandle_t      s_task    = NULL;
static QueueHandle_t     s_queue   = NULL;
static volatile uint32_t s_dropped = 0;
static volatile bool     s_running = false;

static void capture_task(void *arg) {
    int16_t frame[AUDIO_CAPTURE_FRAME_SAMPLES];
    size_t bytes_read = 0;
    while (s_running) {
        esp_err_t r = i2s_channel_read(s_rx_chan, frame, sizeof frame, &bytes_read, portMAX_DELAY);
        if (r != ESP_OK || bytes_read != sizeof frame) continue;
        if (xQueueSend(s_queue, frame, 0) != pdTRUE) s_dropped++;
    }
    vTaskDelete(NULL);
}

esp_err_t audio_capture_start(QueueHandle_t out_queue) {
    if (s_running) return ESP_ERR_INVALID_STATE;
    s_queue = out_queue;
    s_dropped = 0;

    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num  = 8;
    chan_cfg.dma_frame_num = 320;   /* 20 ms per DMA buffer */
    ESP_RETURN_ON_ERROR(i2s_new_channel(&chan_cfg, NULL, &s_rx_chan), TAG, "new chan");

    i2s_std_config_t std_cfg = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(16000),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = ES8311_I2S_MCLK,
            .bclk = ES8311_I2S_BCLK,
            .ws   = ES8311_I2S_LRCK,
            .dout = I2S_GPIO_UNUSED,   /* speaker path deferred */
            .din  = ES8311_I2S_DIN,
        },
    };
    std_cfg.clk_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_256;  /* → 4.096 MHz */
    ESP_RETURN_ON_ERROR(i2s_channel_init_std_mode(s_rx_chan, &std_cfg), TAG, "init std");
    ESP_RETURN_ON_ERROR(i2s_channel_enable(s_rx_chan), TAG, "enable");

    s_running = true;
    BaseType_t ok = xTaskCreatePinnedToCore(capture_task, "aud_cap", 4096, NULL, 22, &s_task, 1);
    if (ok != pdPASS) { s_running = false; return ESP_ERR_NO_MEM; }
    return ESP_OK;
}

esp_err_t audio_capture_stop(void) {
    if (!s_running) return ESP_OK;
    s_running = false;
    vTaskDelay(pdMS_TO_TICKS(50));  /* let task drain */
    i2s_channel_disable(s_rx_chan);
    i2s_del_channel(s_rx_chan);
    s_rx_chan = NULL;
    s_task = NULL;
    return ESP_OK;
}

uint32_t audio_capture_dropped_frames(void) { return s_dropped; }
```

- [ ] **Step 3: Add `voice-record <sec>` REPL command to `main.c`**

```c
#include "audio_capture.h"
#include "es8311.h"

static int cmd_voice_record(int argc, char **argv) {
    if (argc != 2) { printf("usage: voice-record <sec>\n"); return 1; }
    int sec = atoi(argv[1]);
    if (sec <= 0 || sec > 30) { printf("range: 1..30\n"); return 1; }

    ESP_RETURN_ON_ERROR(es8311_init(), "voice-record", "codec init");
    QueueHandle_t q = xQueueCreate(4, AUDIO_CAPTURE_FRAME_SAMPLES * sizeof(int16_t));
    ESP_RETURN_ON_ERROR(audio_capture_start(q), "voice-record", "capture start");

    int total_frames = (sec * 16000) / AUDIO_CAPTURE_FRAME_SAMPLES;
    int16_t frame[AUDIO_CAPTURE_FRAME_SAMPLES];
    for (int i = 0; i < total_frames; i++) {
        if (xQueueReceive(q, frame, pdMS_TO_TICKS(1000)) != pdTRUE) break;
        fwrite(frame, sizeof frame, 1, stdout);   /* raw PCM to UART */
    }
    audio_capture_stop();
    vQueueDelete(q);
    return 0;
}
```

**Note**: `fwrite` to stdout will interleave with any log messages. In practice this doesn't matter — the ear-check tolerates a few bytes of garbage, and `sox` can still play the file. If it matters later, add a `voice-record-base64` variant.

Also add:

```c
static int cmd_voice_stats(int argc, char **argv) {
    (void)argc; (void)argv;
    printf("audio_capture: dropped_frames=%lu\n", audio_capture_dropped_frames());
    return 0;
}
```

Register both.

- [ ] **Step 4: Update `firmware/main/CMakeLists.txt`**

```cmake
SRCS "main.c" "i2c_bitbang.c" "es8311.c" "audio_capture.c"
REQUIRES driver console esp_driver_ledc esp_driver_gpio esp_driver_rmt esp_driver_i2c esp_adc esp_driver_i2s
```

- [ ] **Step 5: Build, flash, capture, ear-check**

```sh
cd firmware && idf.py build flash
# Kill idf.py monitor; use a raw capture instead to avoid log interleave:
idf.py -p /dev/cu.usbserial-10 monitor --print_filter=voice-record:E    # optional
# In another shell:
cat /dev/cu.usbserial-10 | head -c $((16000 * 2 * 3)) > /tmp/mic.pcm
# At the quarky> prompt (from idf.py monitor or screen):
quarky> voice-record 3
# Then on host:
sox -t raw -r 16000 -e signed -b 16 -c 1 /tmp/mic.pcm /tmp/mic.wav
afplay /tmp/mic.wav
```

Expected: you hear whatever the mic picked up. Silent = codec init or DMA path broken. Digital garbage = wrong sample rate / bit depth / channel count.

- [ ] **Step 6: Commit**

```bash
git add firmware/main/audio_capture.h firmware/main/audio_capture.c \
        firmware/main/main.c firmware/main/CMakeLists.txt
git -c user.name=Gekkotron -c user.email=60887050+Gekkotron@users.noreply.github.com \
  commit -m "M3 task 4: I2S RX mic capture + voice-record diagnostic

16 kHz mono int16 capture into a FreeRTOS queue, pinned to core 1.
voice-record dumps N seconds of raw PCM to UART for ear-check on host.
voice-stats reports DMA drops.

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>"
```

---

## Task 5: Command bus + dispatcher (with host tests)

Central `command_t` queue and dispatcher. Groundwork for both REPL and voice to publish. This task builds the plumbing; Task 7 migrates existing REPL verbs onto it.

**Files:**
- Create: `firmware/main/command_bus.h`, `firmware/main/command_bus.c`
- Create: `firmware/test/host/command_bus_test.c`
- Modify: `firmware/test/host/Makefile` (add `command_bus_test` to `TESTS`)
- Modify: `firmware/main/main.c` (start dispatcher task at boot, add `voice-inject-test <id>` dev command)
- Modify: `firmware/main/CMakeLists.txt` (add `command_bus.c`)

**Interfaces:**
- Consumes: existing `bb_motor_set(int, int)` from `main.c`; `led_indicator_apply_*` (comes in Task 6 — Task 5 leaves those handler branches empty with a TODO-free `case` returning `ESP_OK` so the LED handler wiring is Task 6's job, not this task's).
- Produces:
  ```c
  // command_bus.h
  typedef enum { CMD_NONE=0, CMD_MOTOR, CMD_STOP, CMD_LED_RGB, CMD_LED_STATE,
                 CMD_SERVO, CMD_BB_TLC_SET } command_id_t;
  typedef enum { SRC_REPL, SRC_VOICE, SRC_INTERNAL } command_source_t;
  typedef enum { LED_STATE_IDLE, LED_STATE_LISTENING, LED_STATE_OK,
                 LED_STATE_NACK, LED_STATE_USER_RGB } led_state_t;

  /* Named RGB struct — the union member and every handler share this exact
   * type, so functions taking it by value link correctly. Anonymous structs
   * with the same layout are DIFFERENT types in C — do not inline this. */
  typedef struct { uint8_t r, g, b; } led_rgb_t;

  typedef struct {
      command_id_t     id;
      command_source_t source;
      union {
          struct { int8_t  left, right;          } motor;
          led_rgb_t                                led_rgb;
          struct { led_state_t state;            } led_state;
          struct { uint8_t channel; uint16_t us; } servo;
          struct { uint8_t channel, percent;     } bb_tlc;
      } as;
  } command_t;

  esp_err_t command_bus_start(void);      /* Starts dispatcher task on core 0 */
  esp_err_t command_bus_publish(const command_t *cmd);
  ```

- [ ] **Step 1: Write the failing tests — `firmware/test/host/command_bus_test.c`**

Route the dispatcher through mocks so we can assert what got called:

```c
#include "unity/unity.h"
#include <stdint.h>
#include <string.h>

/* Mocks capture every call the dispatcher made. */
static int  g_motor_l = -1, g_motor_r = -1, g_motor_calls = 0;
static int  g_stop_calls = 0;
static int  g_led_rgb_calls = 0; static uint8_t g_last_r, g_last_g, g_last_b;

int mock_bb_motor_set(int l, int r) { g_motor_l=l; g_motor_r=r; g_motor_calls++; return 0; }
int mock_stop_all(void) { g_stop_calls++; return 0; }
int mock_led_apply_rgb(int src, uint8_t r, uint8_t g, uint8_t b) {
    (void)src; g_led_rgb_calls++; g_last_r=r; g_last_g=g; g_last_b=b; return 0;
}

/* Point the dispatcher's handler calls at the mocks by macro redirect. */
#define bb_motor_set              mock_bb_motor_set
#define led_indicator_apply_rgb(src, rgb) mock_led_apply_rgb(src, (rgb).r, (rgb).g, (rgb).b)
#define led_indicator_apply_state(src, st) (0)
/* Substitute the FreeRTOS queue with a synchronous dispatch-once helper so
 * the tests don't need a scheduler. Achieved by isolating dispatcher body
 * into `command_bus_dispatch_one(const command_t *)` in command_bus.c. */
#include "../../main/command_bus.c"
#undef bb_motor_set
#undef led_indicator_apply_rgb
#undef led_indicator_apply_state

void setUp(void) {
    g_motor_l = g_motor_r = -1; g_motor_calls = 0;
    g_stop_calls = 0; g_led_rgb_calls = 0;
}
void tearDown(void) {}

void test_motor_command_calls_bb_motor_set(void) {
    command_t c = { .id=CMD_MOTOR, .source=SRC_REPL, .as.motor={.left=60, .right=-40} };
    command_bus_dispatch_one(&c);
    TEST_ASSERT_EQUAL(1, g_motor_calls);
    TEST_ASSERT_EQUAL(60,  g_motor_l);
    TEST_ASSERT_EQUAL(-40, g_motor_r);
}

void test_stop_command_zeros_motors(void) {
    command_t c = { .id=CMD_STOP, .source=SRC_VOICE };
    command_bus_dispatch_one(&c);
    TEST_ASSERT_EQUAL(1, g_motor_calls);
    TEST_ASSERT_EQUAL(0, g_motor_l);
    TEST_ASSERT_EQUAL(0, g_motor_r);
}

void test_led_rgb_command_calls_apply_rgb(void) {
    command_t c = { .id=CMD_LED_RGB, .source=SRC_REPL, .as.led_rgb={255, 128, 0} };
    command_bus_dispatch_one(&c);
    TEST_ASSERT_EQUAL(1, g_led_rgb_calls);
    TEST_ASSERT_EQUAL_HEX8(255, g_last_r);
    TEST_ASSERT_EQUAL_HEX8(128, g_last_g);
    TEST_ASSERT_EQUAL_HEX8(0,   g_last_b);
}

void test_unknown_command_is_a_noop(void) {
    command_t c = { .id=CMD_NONE, .source=SRC_INTERNAL };
    command_bus_dispatch_one(&c);
    TEST_ASSERT_EQUAL(0, g_motor_calls);
    TEST_ASSERT_EQUAL(0, g_led_rgb_calls);
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_motor_command_calls_bb_motor_set);
    RUN_TEST(test_stop_command_zeros_motors);
    RUN_TEST(test_led_rgb_command_calls_apply_rgb);
    RUN_TEST(test_unknown_command_is_a_noop);
    return UNITY_END();
}
```

- [ ] **Step 2: Add test to Makefile**

```makefile
TESTS := es8311_init_test command_bus_test

command_bus_test: command_bus_test.c $(UNITY_SRC)
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS)
```

- [ ] **Step 3: Run tests, verify they fail**

```sh
cd firmware/test/host && make
```

Expected: compile error (`command_bus.c` doesn't exist yet).

- [ ] **Step 4: Write `command_bus.h`**

Type definitions from the Interfaces block above.

- [ ] **Step 5: Write `command_bus.c`**

```c
#include "command_bus.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_log.h"

/* Forward declarations of handler entry points. Real implementations live
 * in i2c_bitbang.c / led_indicator.c / servo.c. */
extern int bb_motor_set(int left_pct, int right_pct);
extern int led_indicator_apply_rgb(command_source_t source, led_rgb_t rgb);
extern int led_indicator_apply_state(command_source_t source, led_state_t state);
extern int servo_set_us(uint8_t channel, uint16_t us);
extern int bb_tlc_set_pct(uint8_t channel, uint8_t percent);

static const char *TAG = "command_bus";
static QueueHandle_t s_queue = NULL;

/* Split the switch out so host tests can dispatch synchronously. */
void command_bus_dispatch_one(const command_t *c) {
    switch (c->id) {
    case CMD_MOTOR:     bb_motor_set(c->as.motor.left, c->as.motor.right); break;
    case CMD_STOP:      bb_motor_set(0, 0); break;
    case CMD_LED_RGB:   led_indicator_apply_rgb(c->source, c->as.led_rgb); break;
    case CMD_LED_STATE: led_indicator_apply_state(c->source, c->as.led_state.state); break;
    case CMD_SERVO:     servo_set_us(c->as.servo.channel, c->as.servo.us); break;
    case CMD_BB_TLC_SET:bb_tlc_set_pct(c->as.bb_tlc.channel, c->as.bb_tlc.percent); break;
    case CMD_NONE: default: break;
    }
}

static void dispatcher_task(void *arg) {
    command_t c;
    while (xQueueReceive(s_queue, &c, portMAX_DELAY)) {
        ESP_LOGD(TAG, "src=%d id=%d", c.source, c.id);
        command_bus_dispatch_one(&c);
    }
}

esp_err_t command_bus_start(void) {
    if (s_queue) return ESP_ERR_INVALID_STATE;
    s_queue = xQueueCreate(8, sizeof(command_t));
    if (!s_queue) return ESP_ERR_NO_MEM;
    xTaskCreatePinnedToCore(dispatcher_task, "cmd_disp", 4096, NULL, 5, NULL, 0);
    return ESP_OK;
}

esp_err_t command_bus_publish(const command_t *cmd) {
    if (!s_queue) return ESP_ERR_INVALID_STATE;
    return xQueueSend(s_queue, cmd, 0) == pdTRUE ? ESP_OK : ESP_ERR_NO_MEM;
}
```

**Ordering note**: `led_indicator_apply_*` and `servo_set_us` don't exist yet (Task 6 / Task 7). Provide `weak` fallback stubs at the top of `command_bus.c` so the build doesn't break in the meantime:

```c
__attribute__((weak)) int led_indicator_apply_rgb(command_source_t s, led_rgb_t rgb) {
    (void)s; (void)rgb; return 0;
}
__attribute__((weak)) int led_indicator_apply_state(command_source_t s, led_state_t st) {
    (void)s; (void)st; return 0;
}
__attribute__((weak)) int servo_set_us(uint8_t c, uint16_t us) { (void)c; (void)us; return 0; }
__attribute__((weak)) int bb_tlc_set_pct(uint8_t c, uint8_t p) { (void)c; (void)p; return 0; }
```

Task 6 defines the real `led_indicator_apply_*`; Task 7 wires servo. Weak symbols are overridden at link time.

- [ ] **Step 6: Run host tests, verify they pass**

```sh
cd firmware/test/host && make
```

All 4 command_bus tests pass + all 4 es8311_init tests still pass.

- [ ] **Step 7: Call `command_bus_start()` in `app_main` and add `voice-inject-test`**

In `main.c`, after existing hardware init:

```c
ESP_ERROR_CHECK(command_bus_start());
```

Add REPL verb:

```c
static int cmd_voice_inject_test(int argc, char **argv) {
    if (argc != 2) { printf("usage: voice-inject-test <cmd_id 1..7>\n"); return 1; }
    int id = atoi(argv[1]);
    command_t c = { .source = SRC_VOICE };
    switch (id) {
    case 1: c.id=CMD_MOTOR; c.as.motor=(typeof(c.as.motor)){+60,+60}; break;
    case 2: c.id=CMD_MOTOR; c.as.motor=(typeof(c.as.motor)){-60,-60}; break;
    case 3: c.id=CMD_MOTOR; c.as.motor=(typeof(c.as.motor)){-60,+60}; break;
    case 4: c.id=CMD_MOTOR; c.as.motor=(typeof(c.as.motor)){+60,-60}; break;
    case 5: c.id=CMD_STOP; break;
    default: printf("id 1..7 only\n"); return 1;
    }
    return command_bus_publish(&c) == ESP_OK ? 0 : 1;
}
```

Register.

- [ ] **Step 8: Update CMakeLists.txt**

```cmake
SRCS "main.c" "i2c_bitbang.c" "es8311.c" "audio_capture.c" "command_bus.c"
```

- [ ] **Step 9: Build, flash, verify**

```sh
cd firmware && idf.py build flash monitor
quarky> voice-inject-test 1   # expect: both wheels spin forward (uses existing bb_motor_set)
quarky> voice-inject-test 5   # expect: brake
quarky> motor 40 40           # M1 regression — still works (bypasses bus, direct call)
```

- [ ] **Step 10: Commit**

```bash
git add firmware/main/command_bus.h firmware/main/command_bus.c \
        firmware/main/main.c firmware/main/CMakeLists.txt \
        firmware/test/host/command_bus_test.c firmware/test/host/Makefile
git -c user.name=Gekkotron -c user.email=60887050+Gekkotron@users.noreply.github.com \
  commit -m "M3 task 5: command bus + dispatcher

Tagged-union command_t published from any source, dispatched from a
single core-0 task. voice-inject-test lets us exercise the dispatcher
path without the audio stack. Host tests cover routing.

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>"
```

---

## Task 6: LED indicator FSM (with host tests)

Owns the WS2812B. Voice states preempt user RGB; user RGB restored when voice returns to IDLE.

**Files:**
- Create: `firmware/main/led_indicator.h`, `firmware/main/led_indicator.c`
- Create: `firmware/test/host/led_indicator_test.c`
- Modify: `firmware/test/host/Makefile` (add `led_indicator_test`)
- Modify: `firmware/main/CMakeLists.txt` (add `led_indicator.c`)

**Interfaces:**
- Consumes: existing `led_set(r, g, b)` from `main.c` (WS2812B write).
- Produces:
  ```c
  // led_indicator.h
  #include "command_bus.h"   /* for led_state_t + command_source_t */
  esp_err_t led_indicator_start(void);
  int led_indicator_apply_rgb(command_source_t src, led_rgb_t rgb);
  int led_indicator_apply_state(command_source_t src, led_state_t state);
  ```

- [ ] **Step 1: Write failing tests — `firmware/test/host/led_indicator_test.c`**

```c
#include "unity/unity.h"
#include "../../main/command_bus.h"  /* for led_state_t, command_source_t */
#include <stdint.h>

/* Mock the WS2812B write */
static uint8_t g_r, g_g, g_b; static int g_writes;
int mock_led_set(uint8_t r, uint8_t g, uint8_t b) { g_r=r; g_g=g; g_b=b; g_writes++; return 0; }
#define led_set mock_led_set

/* Stub the FSM's tick source (voice-window timeout etc). Tests drive
 * transitions synchronously. */
static uint32_t g_now_ms = 0;
uint32_t led_indicator_now_ms(void) { return g_now_ms; }
#include "../../main/led_indicator.c"
#undef led_set

void setUp(void) { g_r=g_g=g_b=0; g_writes=0; g_now_ms=0; led_indicator_reset_for_test(); }
void tearDown(void) {}

void test_idle_is_dim_white(void) {
    led_indicator_start();
    led_indicator_tick(g_now_ms);
    TEST_ASSERT_EQUAL_HEX8(16, g_r);
    TEST_ASSERT_EQUAL_HEX8(16, g_g);
    TEST_ASSERT_EQUAL_HEX8(16, g_b);
}

void test_wake_goes_blue(void) {
    led_indicator_start();
    led_indicator_apply_state(SRC_VOICE, LED_STATE_LISTENING);
    led_indicator_tick(g_now_ms);
    TEST_ASSERT_EQUAL_HEX8(0, g_r);
    TEST_ASSERT_EQUAL_HEX8(0, g_g);
    TEST_ASSERT_EQUAL_HEX8(255, g_b);
}

void test_ok_flashes_green_then_returns_to_idle(void) {
    led_indicator_start();
    led_indicator_apply_state(SRC_VOICE, LED_STATE_OK);
    led_indicator_tick(g_now_ms);
    TEST_ASSERT_TRUE(g_g > 0 && g_r == 0);
    g_now_ms += 250; /* past 200 ms flash */
    led_indicator_tick(g_now_ms);
    TEST_ASSERT_EQUAL_HEX8(16, g_r); TEST_ASSERT_EQUAL_HEX8(16, g_g); TEST_ASSERT_EQUAL_HEX8(16, g_b);
}

void test_user_rgb_survives_a_voice_cycle(void) {
    /* User sets red, voice wakes → blue, then command → green flash → red returns */
    led_indicator_start();
    led_rgb_t red = {255, 0, 0};
    led_indicator_apply_rgb(SRC_REPL, red);
    led_indicator_tick(g_now_ms);
    TEST_ASSERT_EQUAL_HEX8(255, g_r);

    led_indicator_apply_state(SRC_VOICE, LED_STATE_LISTENING);
    led_indicator_tick(g_now_ms);
    TEST_ASSERT_EQUAL_HEX8(255, g_b);       /* blue */

    led_indicator_apply_state(SRC_VOICE, LED_STATE_OK);
    led_indicator_tick(g_now_ms);
    TEST_ASSERT_TRUE(g_g > 0);              /* green */

    g_now_ms += 250;
    led_indicator_tick(g_now_ms);
    TEST_ASSERT_EQUAL_HEX8(255, g_r);       /* red restored */
    TEST_ASSERT_EQUAL_HEX8(0,   g_g);
    TEST_ASSERT_EQUAL_HEX8(0,   g_b);
}

void test_new_repl_led_during_listening_defers_until_after(void) {
    led_indicator_start();
    led_indicator_apply_state(SRC_VOICE, LED_STATE_LISTENING);
    led_indicator_tick(g_now_ms);
    TEST_ASSERT_EQUAL_HEX8(255, g_b);
    /* User types `led 0 255 0` mid-listening */
    led_rgb_t green = {0, 255, 0};
    led_indicator_apply_rgb(SRC_REPL, green);
    led_indicator_tick(g_now_ms);
    TEST_ASSERT_EQUAL_HEX8(255, g_b);       /* still blue — deferred */
    /* Listening ends via OK path */
    led_indicator_apply_state(SRC_VOICE, LED_STATE_OK);
    led_indicator_tick(g_now_ms);
    g_now_ms += 250;
    led_indicator_tick(g_now_ms);
    TEST_ASSERT_EQUAL_HEX8(255, g_g);       /* green from deferred set applied */
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_idle_is_dim_white);
    RUN_TEST(test_wake_goes_blue);
    RUN_TEST(test_ok_flashes_green_then_returns_to_idle);
    RUN_TEST(test_user_rgb_survives_a_voice_cycle);
    RUN_TEST(test_new_repl_led_during_listening_defers_until_after);
    return UNITY_END();
}
```

- [ ] **Step 2: Add to Makefile TESTS list**

- [ ] **Step 3: Write `led_indicator.h`**

```c
#pragma once
#include "esp_err.h"
#include "command_bus.h"

esp_err_t led_indicator_start(void);          /* Starts periodic tick task */
int led_indicator_apply_rgb(command_source_t src, led_rgb_t rgb);
int led_indicator_apply_state(command_source_t src, led_state_t state);

/* Testing hooks (host tests use these; firmware code doesn't call them). */
void     led_indicator_reset_for_test(void);
void     led_indicator_tick(uint32_t now_ms);
uint32_t led_indicator_now_ms(void);           /* wraps esp_timer_get_time on device */
```

- [ ] **Step 4: Write `led_indicator.c`**

```c
#include "led_indicator.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"

extern int led_set(uint8_t r, uint8_t g, uint8_t b);   /* implemented in main.c */

static led_state_t s_state = LED_STATE_IDLE;
static led_rgb_t   s_user_rgb = {16, 16, 16};       /* dim white default */
static led_rgb_t   s_pending_user_rgb = {16, 16, 16};
static bool        s_have_pending = false;
static uint32_t    s_state_entered_ms = 0;

#define FLASH_MS 200
#define LISTEN_MS 5760   /* MultiNet speech window */

__attribute__((weak)) uint32_t led_indicator_now_ms(void) {
    return (uint32_t)(esp_timer_get_time() / 1000);
}

int led_indicator_apply_rgb(command_source_t src, led_rgb_t rgb) {
    (void)src;
    if (s_state == LED_STATE_IDLE || s_state == LED_STATE_USER_RGB) {
        s_user_rgb = rgb;
        s_pending_user_rgb = rgb;
        s_have_pending = false;
        s_state = LED_STATE_USER_RGB;
    } else {
        /* Defer — voice is holding the LED */
        s_pending_user_rgb = rgb;
        s_have_pending = true;
    }
    return 0;
}

int led_indicator_apply_state(command_source_t src, led_state_t state) {
    (void)src;
    s_state = state;
    s_state_entered_ms = led_indicator_now_ms();
    return 0;
}

void led_indicator_tick(uint32_t now_ms) {
    /* Timeout-driven transitions out of transient states */
    if (s_state == LED_STATE_OK || s_state == LED_STATE_NACK) {
        if (now_ms - s_state_entered_ms >= FLASH_MS) {
            if (s_have_pending) { s_user_rgb = s_pending_user_rgb; s_have_pending = false; }
            s_state = (s_user_rgb.r == 16 && s_user_rgb.g == 16 && s_user_rgb.b == 16)
                    ? LED_STATE_IDLE : LED_STATE_USER_RGB;
        }
    } else if (s_state == LED_STATE_LISTENING) {
        if (now_ms - s_state_entered_ms >= LISTEN_MS) {
            /* Auto-timeout without a command → treat as NACK */
            s_state = LED_STATE_NACK;
            s_state_entered_ms = now_ms;
        }
    }

    led_rgb_t out;
    switch (s_state) {
    case LED_STATE_LISTENING: out = (led_rgb_t){0, 0, 255}; break;
    case LED_STATE_OK:        out = (led_rgb_t){0, 200, 0}; break;
    case LED_STATE_NACK:      out = (led_rgb_t){200, 0, 0}; break;
    case LED_STATE_USER_RGB:  out = s_user_rgb; break;
    default:                  out = (led_rgb_t){16, 16, 16}; break;   /* IDLE */
    }
    led_set(out.r, out.g, out.b);
}

static void led_indicator_task(void *arg) {
    for (;;) {
        led_indicator_tick(led_indicator_now_ms());
        vTaskDelay(pdMS_TO_TICKS(30));
    }
}

esp_err_t led_indicator_start(void) {
    led_indicator_reset_for_test();
    xTaskCreatePinnedToCore(led_indicator_task, "led_ind", 2048, NULL, 3, NULL, 0);
    return ESP_OK;
}

void led_indicator_reset_for_test(void) {
    s_state = LED_STATE_IDLE;
    s_user_rgb = (led_rgb_t){16, 16, 16};
    s_pending_user_rgb = (led_rgb_t){16, 16, 16};
    s_have_pending = false;
    s_state_entered_ms = 0;
}
```

- [ ] **Step 5: Run tests, verify they pass**

```sh
cd firmware/test/host && make
```

All 5 led_indicator tests pass; earlier tests still pass.

- [ ] **Step 6: Wire `led_indicator_start()` into `app_main`**

In `main.c`, right after existing `led_init`:

```c
ESP_ERROR_CHECK(led_indicator_start());
```

Regression: existing `led <r> <g> <b>` REPL command must still work. Since it currently calls `led_set` directly, and `led_indicator` also calls `led_set`, there's now a race — the FSM overwrites the manual set on next tick. Solution: change the REPL `led` handler to publish `CMD_LED_RGB` through the bus (Task 7 does this systematically; do the LED verb here for continuity).

Replace the existing `cmd_led` REPL handler:

```c
static int cmd_led(int argc, char **argv) {
    if (argc != 4) { printf("usage: led <r> <g> <b>\n"); return 1; }
    command_t c = { .id=CMD_LED_RGB, .source=SRC_REPL,
        .as.led_rgb = { .r=atoi(argv[1]), .g=atoi(argv[2]), .b=atoi(argv[3]) } };
    return command_bus_publish(&c);
}
```

- [ ] **Step 7: Update CMakeLists.txt**

```cmake
SRCS "main.c" "i2c_bitbang.c" "es8311.c" "audio_capture.c" "command_bus.c" "led_indicator.c"
```

- [ ] **Step 8: Build, flash, verify**

```sh
cd firmware && idf.py build flash monitor
quarky> led 255 0 0        # expect: LED red
quarky> led 0 0 255        # expect: LED blue
# Manually poke voice states via voice-inject-test — LED should follow. But
# voice-inject-test publishes MOTOR commands, not LED states. Add a temporary
# dev command `led-state <n>` if you want to test arbitration on hardware
# before Task 10 wires it end-to-end:
quarky> led-state 1        # (dev only; delete before shipping)
```

If skipping the temporary `led-state` command, arbitration on hardware is validated in Task 10's smoke test step 9.

- [ ] **Step 9: Commit**

```bash
git add firmware/main/led_indicator.h firmware/main/led_indicator.c \
        firmware/main/main.c firmware/main/CMakeLists.txt \
        firmware/test/host/led_indicator_test.c firmware/test/host/Makefile
git -c user.name=Gekkotron -c user.email=60887050+Gekkotron@users.noreply.github.com \
  commit -m "M3 task 6: LED indicator FSM with voice/user arbitration

FSM owns the WS2812B. Voice states preempt user RGB; user RGB restored
on return to idle. Arbitration corner case (REPL led during listening)
tested. Wired to REPL 'led' command through the bus.

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>"
```

---

## Task 7: Migrate remaining M1 REPL verbs to command bus

Bring the remaining action-verbs (`motor`, `stop`, `servo`, `bb-tlc-set`) onto the bus for consistency and to prove nothing regresses. Diagnostic verbs (`bb-scan`, `tlc-init`, `i2c-selftest`, `es-verify`, `es-init`) stay direct — they don't act on the robot.

**Files:**
- Modify: `firmware/main/main.c` (rewrite handler bodies for the four action verbs)

**Interfaces:**
- Consumes: `command_bus_publish` from Task 5, `CMD_MOTOR`/`CMD_STOP`/`CMD_SERVO`/`CMD_BB_TLC_SET` from `command_bus.h`.
- Produces: nothing new.

- [ ] **Step 1: Replace `cmd_motor` handler**

```c
static int cmd_motor(int argc, char **argv) {
    if (argc != 3) { printf("usage: motor <L> <R>\n"); return 1; }
    int L = atoi(argv[1]), R = atoi(argv[2]);
    if (L < -100 || L > 100 || R < -100 || R > 100) { printf("range: -100..100\n"); return 1; }
    command_t c = { .id=CMD_MOTOR, .source=SRC_REPL, .as.motor={.left=(int8_t)L, .right=(int8_t)R} };
    return command_bus_publish(&c);
}
```

- [ ] **Step 2: Replace `cmd_stop` handler**

```c
static int cmd_stop(int argc, char **argv) {
    (void)argc; (void)argv;
    command_t c = { .id=CMD_STOP, .source=SRC_REPL };
    return command_bus_publish(&c);
}
```

- [ ] **Step 3: Replace `cmd_servo` handler**

```c
static int cmd_servo(int argc, char **argv) {
    if (argc != 2) { printf("usage: servo <deg>\n"); return 1; }
    int deg = atoi(argv[1]);
    if (deg < 0 || deg > 180) { printf("range: 0..180\n"); return 1; }
    uint16_t us = 500 + (uint32_t)deg * (2500 - 500) / 180;
    command_t c = { .id=CMD_SERVO, .source=SRC_REPL, .as.servo={.channel=1, .us=us} };
    return command_bus_publish(&c);
}
```

Extract the existing servo-write logic into a non-static `int servo_set_us(uint8_t channel, uint16_t us)` (matches the weak-symbol signature in `command_bus.c` from Task 5, which now takes precedence over the weak stub).

- [ ] **Step 4: Replace `cmd_bb_tlc_set` handler**

```c
static int cmd_bb_tlc_set(int argc, char **argv) {
    if (argc != 3) { printf("usage: bb-tlc-set <ch> <pct>\n"); return 1; }
    int ch = atoi(argv[1]), pct = atoi(argv[2]);
    if (ch < 0 || ch > 7 || pct < 0 || pct > 100) { printf("ch 0..7, pct 0..100\n"); return 1; }
    command_t c = { .id=CMD_BB_TLC_SET, .source=SRC_REPL, .as.bb_tlc={ (uint8_t)ch, (uint8_t)pct } };
    return command_bus_publish(&c);
}
```

Extract the write logic into `int bb_tlc_set_pct(uint8_t channel, uint8_t percent)` (matches the weak stub from Task 5).

- [ ] **Step 5: Change `bb_motor_set` and `bb_tlc_set` return types to `int` and expose them non-static**

The command bus dispatcher (Task 5) `extern`s these with `int` return type. Reconcile:

```c
// In main.c — change:
//   static void bb_motor_set(int left_pct, int right_pct) { ... }
// to:
int bb_motor_set(int left_pct, int right_pct) { ...; return 0; }

// Same for the TLC helper the bus calls:
//   static void bb_tlc_set(int channel, int percent) { ... }
// to:
int bb_tlc_set_pct(uint8_t channel, uint8_t percent) { ...; return 0; }
```

The rename to `bb_tlc_set_pct` matches the `extern` in `command_bus.c` from Task 5. Keep the old `bb-tlc-set` REPL verb pointing at the new function name (only the C symbol changes; the user-facing command stays `bb-tlc-set`).

The existing `bb-tlc-set` diagnostic in `main.c` may still call the old function name — search and replace `bb_tlc_set` → `bb_tlc_set_pct` at all call sites. There should be no other callers of `bb_motor_set` outside of the bus and the REPL handler you're editing in this task.

Similarly define `int servo_set_us(uint8_t channel, uint16_t us)` non-static, extracted from the body of `cmd_servo`.

- [ ] **Step 6: Build, flash, and run full M1 regression**

```sh
cd firmware && idf.py build flash monitor
quarky> motor 40 40    # expect: both spin
quarky> motor -30 30   # expect: pivot
quarky> stop           # expect: brake
quarky> servo 90       # expect: center
quarky> bb-tlc-set 4 50 # expect: P1 header at 50% (measure with multimeter if unsure)
quarky> led 128 0 128  # expect: purple
```

Every command must still behave exactly as before. If not — the dispatcher path introduces latency; check that priorities are right (motor bit-bang at high prio during write; dispatcher at prio 5).

- [ ] **Step 7: Commit**

```bash
git add firmware/main/main.c
git -c user.name=Gekkotron -c user.email=60887050+Gekkotron@users.noreply.github.com \
  commit -m "M3 task 7: migrate M1 REPL action-verbs to command bus

motor / stop / servo / bb-tlc-set now publish command_t through the bus.
Handlers extracted to non-static functions with the signatures command_bus.c
expects. Diagnostic verbs stay direct. M1 regressions verified.

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>"
```

---

## Task 8: Partition scheme + ESP-SR dependency + model directory + build system

Prepares the flash layout and dependencies for ESP-SR. No inference yet.

**Files:**
- Create: `firmware/partitions.csv`
- Create: `firmware/main/idf_component.yml`
- Create: `firmware/model/wn9_hiesp.bin` (download from esp-sr release)
- Create: `firmware/model/mn7_en/mn7_en.bin`, `firmware/model/mn7_en/mn7_en.index`
- Create: `docs/model-provenance.md`
- Modify: `firmware/sdkconfig.defaults` (custom partition table, PSRAM verification)
- Modify: `firmware/main/CMakeLists.txt` (add `spiffs_create_partition_image` for `model`)
- Modify: `scripts/upload.sh` (add `--erase-all` pass-through)

**Interfaces:**
- Consumes: nothing new.
- Produces: `model` partition mounted at boot, populated with WakeNet + MultiNet blobs.

- [ ] **Step 1: Create `firmware/partitions.csv`**

```
# Name,     Type, SubType,  Offset,      Size,   Notes
nvs,        data, nvs,      0x9000,      0x6000,
phy_init,   data, phy,      0xf000,      0x1000,
factory,    app,  factory,  0x10000,     0x400000,
model,      data, spiffs,   0x410000,    0x400000,
sys,        data, littlefs, 0x810000,    0x100000,
vfs,        data, littlefs, 0x910000,    0x600000,
storage,    data, littlefs, 0xf10000,    0xA0000,
```

Total end: 0xfb0000. 320 KB slack at top of flash reserved for M4 (OTA metadata / TFLite model).

- [ ] **Step 2: Update `firmware/sdkconfig.defaults`**

Append (do not remove existing lines):

```
CONFIG_PARTITION_TABLE_CUSTOM=y
CONFIG_PARTITION_TABLE_CUSTOM_FILENAME="partitions.csv"

# PSRAM (verify already present; add if missing)
CONFIG_SPIRAM=y
CONFIG_SPIRAM_MODE_OCT=y
CONFIG_SPIRAM_USE_MALLOC=y

# ESP-SR wants perf builds for inference speed
CONFIG_COMPILER_OPTIMIZATION_PERF=y
```

- [ ] **Step 3: Create `firmware/main/idf_component.yml`**

```yaml
dependencies:
  idf:
    version: ">=5.3.0"
  espressif/esp-sr: "==2.0.0"
```

- [ ] **Step 4: Download stock ESP-SR models into `firmware/model/`**

```sh
# From repo root, one-time model fetch (models are ~3 MB total)
mkdir -p firmware/model firmware/model/mn7_en
# Fetch from the esp-sr release matching version 2.0.0. Exact URLs come from
# https://github.com/espressif/esp-sr/releases/tag/v2.0.0
curl -L -o firmware/model/wn9_hiesp.bin \
    https://github.com/espressif/esp-sr/raw/v2.0.0/model/wakenet_model/wn9_hiesp/wn9_hiesp.bin
curl -L -o firmware/model/mn7_en/mn7_en.bin \
    https://github.com/espressif/esp-sr/raw/v2.0.0/model/multinet_model/mn7_en/mn7_en.bin
curl -L -o firmware/model/mn7_en/mn7_en.index \
    https://github.com/espressif/esp-sr/raw/v2.0.0/model/multinet_model/mn7_en/mn7_en.index
```

Verify:
```sh
ls -lh firmware/model/
# wn9_hiesp.bin  ~600K
# mn7_en/mn7_en.bin  ~2M
```

If URLs 404, use the `idf_component_manager`'s auto-fetch by letting `esp_srmodel_init` pull from `managed_components/` instead. Adjust the CMake integration in step 6 accordingly.

- [ ] **Step 5: Create `docs/model-provenance.md`**

```markdown
# Model provenance

Every `.bin` under `firmware/model/` is listed here with its source, ESP-SR
compatibility version, and license. Add an entry whenever a model file is
added, removed, or replaced.

| File | Source | ESP-SR version | License | Notes |
|---|---|---|---|---|
| `wn9_hiesp.bin` | https://github.com/espressif/esp-sr/tree/v2.0.0/model/wakenet_model/wn9_hiesp | 2.0.0 | See esp-sr LICENSE | Stock "Hi ESP" wake word; placeholder for custom Hi Quarky |
| `mn7_en/mn7_en.bin` | https://github.com/espressif/esp-sr/tree/v2.0.0/model/multinet_model/mn7_en | 2.0.0 | See esp-sr LICENSE | MultiNet-EN command recognition, English G2P |
| `mn7_en/mn7_en.index` | (same) | 2.0.0 | (same) | MultiNet index |
| `wn9_hiquarky.bin` | Espressif Skainet Studio order #<TBD-from-Task-0> | 2.0.0 | (see order agreement) | **Not yet delivered.** Added by Task 12 when Skainet ships. |
```

- [ ] **Step 6: Update `firmware/main/CMakeLists.txt` to package the model partition**

```cmake
idf_component_register(
    SRCS "main.c" "i2c_bitbang.c" "es8311.c" "audio_capture.c"
         "command_bus.c" "led_indicator.c"
    INCLUDE_DIRS "."
    REQUIRES driver console esp_driver_ledc esp_driver_gpio esp_driver_rmt
             esp_driver_i2c esp_adc esp_driver_i2s esp_timer
    PRIV_REQUIRES esp-sr
)

# Package firmware/model/ into the `model` partition at build time.
spiffs_create_partition_image(model ../model FLASH_IN_PROJECT)
```

- [ ] **Step 7: Update `scripts/upload.sh` — add `--erase-all` pass-through**

Read the current script; add early argument handling to accept `--erase-all` before the existing arg parsing, and pass `--erase-all` to `idf.py flash` when set. Concretely:

```sh
ERASE=""
if [ "$1" = "--erase-all" ]; then ERASE="--erase-all"; shift; fi
# ...existing flash invocation...
idf.py -p "$PORT" flash $ERASE "$@"
```

(Exact placement depends on current script structure — preserve everything else.)

- [ ] **Step 8: Build the project (fetches esp-sr)**

```sh
cd firmware
. ~/esp/esp-idf/export.sh
idf.py reconfigure
idf.py build
```

Expected: `esp-sr` component pulled into `managed_components/`. Build succeeds. Look for a line like `Successfully created spiffs image` for the `model` partition.

- [ ] **Step 9: Flash with `--erase-all` (first M3 flash)**

```sh
./scripts/upload.sh --erase-all monitor
```

Expected boot log:
- Partition table shows six entries including `model`
- No ESP-SR init lines yet (voice_pipeline not started until Task 9)
- REPL commands from Tasks 1–7 all still work: `es-verify`, `motor 40 40`, `led 255 0 0`, `voice-record 3`

- [ ] **Step 10: Commit**

```bash
git add firmware/partitions.csv firmware/sdkconfig.defaults \
        firmware/main/idf_component.yml firmware/main/CMakeLists.txt \
        firmware/model/ docs/model-provenance.md scripts/upload.sh
git -c user.name=Gekkotron -c user.email=60887050+Gekkotron@users.noreply.github.com \
  commit -m "M3 task 8: partition scheme + esp-sr dependency + model images

New partitions.csv adds a 4 MB 'model' partition for ESP-SR blobs, shrinks
factory to 4 MB, grows vfs to 6 MB. sdkconfig.defaults pins PSRAM + PERF
optimisation. WakeNet (wn9_hiesp) and MultiNet-EN packaged from
firmware/model/ via spiffs_create_partition_image. Provenance table at
docs/model-provenance.md. Custom Hi Quarky slot reserved for Task 12.

First M3 flash requires --erase-all (added to scripts/upload.sh).

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>"
```

---

## Task 9: Voice pipeline — AFE + WakeNet (log-only)

Bring up ESP-SR to the point of detecting the wake word. Log detections; don't dispatch commands yet (that's Task 10).

**Files:**
- Create: `firmware/main/voice_pipeline.h`, `firmware/main/voice_pipeline.c`
- Modify: `firmware/main/main.c` (add `voice-start` REPL verb; extend `voice-stats` output)
- Modify: `firmware/main/CMakeLists.txt` (add `voice_pipeline.c`)

**Interfaces:**
- Consumes: `audio_capture_start(queue)` from Task 4; ESP-SR APIs from the `esp-sr` component; `es8311_init`, `es8311_start` from Task 3.
- Produces:
  ```c
  // voice_pipeline.h
  esp_err_t voice_pipeline_start(void);
  esp_err_t voice_pipeline_stop(void);
  uint32_t  voice_pipeline_wake_count(void);
  uint32_t  voice_pipeline_reject_count(void);
  ```

- [ ] **Step 1: Write `voice_pipeline.h`** — exactly the API in the Interfaces block.

- [ ] **Step 2: Write `voice_pipeline.c` — AFE + WakeNet only**

```c
#include "voice_pipeline.h"
#include "audio_capture.h"
#include "es8311.h"
#include "esp_afe_sr_iface.h"
#include "esp_afe_sr_models.h"
#include "esp_wn_iface.h"
#include "model_path.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

static const char *TAG = "voice";

static esp_afe_sr_iface_t *s_afe = NULL;
static esp_afe_sr_data_t  *s_afe_data = NULL;
static QueueHandle_t       s_pcm_q = NULL;
static TaskHandle_t        s_task = NULL;
static volatile bool       s_running = false;
static volatile uint32_t   s_wake_count = 0;
static volatile uint32_t   s_reject_count = 0;

static void pipeline_task(void *arg) {
    int chunk = s_afe->get_feed_chunksize(s_afe_data);   /* 512 samples */
    int16_t *buf = malloc(chunk * sizeof(int16_t));
    if (!buf) { ESP_LOGE(TAG, "buf alloc"); vTaskDelete(NULL); return; }

    while (s_running) {
        if (xQueueReceive(s_pcm_q, buf, portMAX_DELAY) != pdTRUE) continue;
        s_afe->feed(s_afe_data, buf);

        afe_fetch_result_t *r = s_afe->fetch(s_afe_data);
        if (!r || r->ret_value == ESP_FAIL) continue;

        if (r->wakeup_state == WAKENET_DETECTED) {
            s_wake_count++;
            ESP_LOGI(TAG, "WAKE (count=%lu)", s_wake_count);
        }
    }
    free(buf);
    vTaskDelete(NULL);
}

esp_err_t voice_pipeline_start(void) {
    if (s_running) return ESP_ERR_INVALID_STATE;

    ESP_RETURN_ON_ERROR(es8311_init(), TAG, "codec init");
    ESP_RETURN_ON_ERROR(es8311_start(), TAG, "codec start");

    s_pcm_q = xQueueCreate(4, AUDIO_CAPTURE_FRAME_SAMPLES * sizeof(int16_t));
    if (!s_pcm_q) return ESP_ERR_NO_MEM;
    ESP_RETURN_ON_ERROR(audio_capture_start(s_pcm_q), TAG, "capture start");

    srmodel_list_t *models = esp_srmodel_init("model");
    if (!models) { ESP_LOGE(TAG, "no model partition"); return ESP_FAIL; }

    afe_config_t *cfg = afe_config_init("MMR", models, AFE_TYPE_SR, AFE_MODE_LOW_COST);
    cfg->wakenet_init = true;
    cfg->voice_communication_init = false;
    cfg->aec_init = false;
    cfg->se_init  = true;
    cfg->vad_init = true;
    cfg->wakenet_model_name = "wn9_hiesp";   /* Skainet swap happens in Task 12 */

    s_afe = esp_afe_handle_from_config(cfg);
    s_afe_data = s_afe->create_from_config(cfg);
    afe_config_free(cfg);

    s_running = true;
    xTaskCreatePinnedToCore(pipeline_task, "voice", 8192, NULL, 5, &s_task, 1);
    return ESP_OK;
}

esp_err_t voice_pipeline_stop(void) {
    if (!s_running) return ESP_OK;
    s_running = false;
    vTaskDelay(pdMS_TO_TICKS(100));
    audio_capture_stop();
    if (s_afe_data) { s_afe->destroy(s_afe_data); s_afe_data = NULL; }
    if (s_pcm_q)    { vQueueDelete(s_pcm_q); s_pcm_q = NULL; }
    es8311_stop();
    return ESP_OK;
}

uint32_t voice_pipeline_wake_count(void)   { return s_wake_count; }
uint32_t voice_pipeline_reject_count(void) { return s_reject_count; }
```

Note the exact ESP-SR API names may drift between minor versions of esp-sr 2.x. Consult the header files under `managed_components/espressif__esp-sr/` after the build in Task 8 populates them. Adjust the function names in the code above to match the header — do NOT rewrite this from memory.

- [ ] **Step 3: Add `voice-start` / `voice-stop` REPL commands to `main.c`**

```c
#include "voice_pipeline.h"

static int cmd_voice_start(int argc, char **argv) {
    (void)argc; (void)argv;
    return voice_pipeline_start();
}
static int cmd_voice_stop(int argc, char **argv) {
    (void)argc; (void)argv;
    return voice_pipeline_stop();
}
```

Update `cmd_voice_stats` (from Task 4) to also print wake/reject counts:

```c
static int cmd_voice_stats(int argc, char **argv) {
    (void)argc; (void)argv;
    printf("dropped_frames=%lu wake=%lu reject=%lu\n",
           audio_capture_dropped_frames(),
           voice_pipeline_wake_count(),
           voice_pipeline_reject_count());
    return 0;
}
```

Register the new commands.

- [ ] **Step 4: Update `firmware/main/CMakeLists.txt`**

```cmake
SRCS "main.c" "i2c_bitbang.c" "es8311.c" "audio_capture.c"
     "command_bus.c" "led_indicator.c" "voice_pipeline.c"
```

- [ ] **Step 5: Build, flash, verify wake word detection**

```sh
cd firmware && idf.py build flash monitor
quarky> voice-start
# Wait ~1 s for AFE init logs
# Say "Hi ESP" clearly, ~30 cm from the mic:
# Expected in log: I (…) voice: WAKE (count=1)
quarky> voice-stats
# Expected: dropped_frames=0 wake=1 reject=0

# Repeat 5 times to sanity-check reliability.
quarky> voice-stop
```

If wake never fires:
1. Confirm audio capture still works (`voice-record 3` post-`voice-stop` — you should hear yourself).
2. Confirm `esp_srmodel_init("model")` found the wake model in the log (look for `wn9_hiesp`).
3. Confirm mic isn't too quiet — bump `es8311_set_mic_gain_db(15)` inside `voice_pipeline_start` before `audio_capture_start` and rebuild.

- [ ] **Step 6: Commit**

```bash
git add firmware/main/voice_pipeline.h firmware/main/voice_pipeline.c \
        firmware/main/main.c firmware/main/CMakeLists.txt
git -c user.name=Gekkotron -c user.email=60887050+Gekkotron@users.noreply.function.com \
  commit -m "M3 task 9: voice pipeline with AFE + WakeNet (log-only)

Feeds mic PCM into ESP-SR AFE (SE + VAD on, AEC off, no reference).
WakeNet loaded from the model partition; 'Hi ESP' triggers WAKE log line
and increments voice-stats wake counter. No command dispatch yet.

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>"
```

---

## Task 10: MultiNet + voice → command bus wiring

Add MultiNet inside the wake-triggered listening window, dispatch recognized commands into the bus, and drive the LED FSM through wake / ok / nack transitions.

**Files:**
- Modify: `firmware/main/voice_pipeline.c` (add MultiNet init + listening state + dispatch)
- Modify: `firmware/main/main.c` (auto-start voice pipeline at boot)

**Interfaces:**
- Consumes: MultiNet API from `esp-sr`, `command_bus_publish` from Task 5, `led_indicator_apply_state` from Task 6.
- Produces: end-to-end voice → motor.

- [ ] **Step 1: Extend `voice_pipeline.c` with MultiNet setup**

At the top, add module state:

```c
#include "esp_mn_iface.h"
#include "esp_mn_models.h"
#include "esp_process_sdkconfig.h"
#include "command_bus.h"
#include "led_indicator.h"
#include "esp_timer.h"

static esp_mn_iface_t         *s_mn = NULL;
static model_iface_data_t     *s_mn_data = NULL;
static int64_t                 s_listen_deadline_us = 0;
#define LISTEN_MS 5760
#define CONF_THRESHOLD 0.6f

/* Speed defaults — tune post-hardware */
#define V_FWD    ((int8_t) 60)
#define V_BACK   ((int8_t)-60)
#define V_TURN   ((int8_t) 60)
#define V_STEP   ((int8_t) 20)
static int8_t s_cur_left  = 0;
static int8_t s_cur_right = 0;
```

In `voice_pipeline_start`, after the AFE setup:

```c
    s_mn = esp_mn_handle_from_name("mn7_en");
    if (!s_mn) { ESP_LOGE(TAG, "no MultiNet model"); return ESP_FAIL; }
    s_mn_data = s_mn->create("mn7_en", LISTEN_MS);
    esp_mn_commands_alloc(s_mn, s_mn_data);
    esp_mn_commands_add(1, "forward");
    esp_mn_commands_add(2, "backward");
    esp_mn_commands_add(3, "left");
    esp_mn_commands_add(4, "right");
    esp_mn_commands_add(5, "stop");
    esp_mn_commands_add(6, "faster");
    esp_mn_commands_add(7, "slower");
    esp_mn_commands_update();
```

Replace the pipeline task body:

```c
static void dispatch_voice_command(int id, float prob) {
    if (prob < CONF_THRESHOLD) {
        s_reject_count++;
        led_indicator_apply_state(SRC_VOICE, LED_STATE_NACK);
        return;
    }
    command_t c = { .source = SRC_VOICE };
    switch (id) {
    case 1: c.id=CMD_MOTOR; c.as.motor=(typeof(c.as.motor)){V_FWD, V_FWD};
            s_cur_left=V_FWD; s_cur_right=V_FWD; break;
    case 2: c.id=CMD_MOTOR; c.as.motor=(typeof(c.as.motor)){V_BACK, V_BACK};
            s_cur_left=V_BACK; s_cur_right=V_BACK; break;
    case 3: c.id=CMD_MOTOR; c.as.motor=(typeof(c.as.motor)){-V_TURN, V_TURN};
            s_cur_left=-V_TURN; s_cur_right=V_TURN; break;
    case 4: c.id=CMD_MOTOR; c.as.motor=(typeof(c.as.motor)){V_TURN, -V_TURN};
            s_cur_left=V_TURN; s_cur_right=-V_TURN; break;
    case 5: c.id=CMD_STOP; s_cur_left=s_cur_right=0; break;
    case 6: /* faster */ {
            int8_t nl = s_cur_left  + (s_cur_left  >= 0 ? V_STEP : -V_STEP);
            int8_t nr = s_cur_right + (s_cur_right >= 0 ? V_STEP : -V_STEP);
            if (nl > 100) nl = 100; if (nl < -100) nl = -100;
            if (nr > 100) nr = 100; if (nr < -100) nr = -100;
            c.id=CMD_MOTOR; c.as.motor=(typeof(c.as.motor)){nl, nr};
            s_cur_left = nl; s_cur_right = nr; break; }
    case 7: /* slower */ {
            int8_t nl = s_cur_left  - (s_cur_left  >= 0 ? V_STEP : -V_STEP);
            int8_t nr = s_cur_right - (s_cur_right >= 0 ? V_STEP : -V_STEP);
            c.id=CMD_MOTOR; c.as.motor=(typeof(c.as.motor)){nl, nr};
            s_cur_left = nl; s_cur_right = nr; break; }
    default: led_indicator_apply_state(SRC_VOICE, LED_STATE_NACK); return;
    }
    command_bus_publish(&c);
    led_indicator_apply_state(SRC_VOICE, LED_STATE_OK);
}

static void pipeline_task(void *arg) {
    int chunk = s_afe->get_feed_chunksize(s_afe_data);
    int16_t *buf = malloc(chunk * sizeof(int16_t));
    if (!buf) { vTaskDelete(NULL); return; }
    while (s_running) {
        if (xQueueReceive(s_pcm_q, buf, portMAX_DELAY) != pdTRUE) continue;
        s_afe->feed(s_afe_data, buf);
        afe_fetch_result_t *r = s_afe->fetch(s_afe_data);
        if (!r || r->ret_value == ESP_FAIL) continue;

        if (r->wakeup_state == WAKENET_DETECTED) {
            s_wake_count++;
            s_listen_deadline_us = esp_timer_get_time() + (int64_t)LISTEN_MS * 1000;
            led_indicator_apply_state(SRC_VOICE, LED_STATE_LISTENING);
        }

        if (esp_timer_get_time() < s_listen_deadline_us) {
            esp_mn_state_t s = s_mn->detect(s_mn_data, r->data);
            if (s == ESP_MN_STATE_DETECTED) {
                esp_mn_results_t *res = s_mn->get_results(s_mn_data);
                if (res && res->num > 0) {
                    dispatch_voice_command(res->command_id[0], res->prob[0]);
                }
                s_listen_deadline_us = 0;   /* end window */
            } else if (s == ESP_MN_STATE_TIMEOUT) {
                led_indicator_apply_state(SRC_VOICE, LED_STATE_NACK);
                s_listen_deadline_us = 0;
            }
        }
    }
    free(buf);
    vTaskDelete(NULL);
}
```

Extend `voice_pipeline_stop` to also destroy MultiNet:

```c
    if (s_mn_data) { s_mn->destroy(s_mn_data); s_mn_data = NULL; }
```

- [ ] **Step 2: Auto-start voice pipeline at boot**

In `app_main`, after `command_bus_start()` and `led_indicator_start()`:

```c
ESP_ERROR_CHECK(voice_pipeline_start());
```

- [ ] **Step 3: Build, flash, run end-to-end**

```sh
cd firmware && idf.py build flash monitor
# Wait for REPL ready and voice pipeline init logs.
# Then say (~30 cm from mic):
#   "Hi ESP"                    → LED goes blue
#   "Hi ESP, forward"           → LED green flash, wheels forward @60%
#   "Hi ESP, faster"            → LED green flash, wheels @80%
#   "Hi ESP, stop"              → LED green flash, brake
#   "Hi ESP, blergh"            → LED red flash (rejected or timeout)
#   "Hi ESP" then say nothing   → LED red flash after ~5.7s
```

If wake works but commands don't:
- Confidence threshold too high — lower `CONF_THRESHOLD` to 0.5, rebuild.
- MultiNet model didn't load — grep boot log for `mn7_en`.
- Command ID enum mismatch — confirm `esp_mn_commands_add` order matches the `switch` cases.

- [ ] **Step 4: Commit**

```bash
git add firmware/main/voice_pipeline.c firmware/main/main.c
git -c user.name=Gekkotron -c user.email=60887050+Gekkotron@users.noreply.github.com \
  commit -m "M3 task 10: MultiNet + voice → command bus + LED feedback

MultiNet-EN loaded with the seven drive commands. Post-wake listening
window opens for 5.76s; matched commands publish through the bus and
drive the LED FSM (listening/ok/nack). faster/slower step the current
motor state by V_STEP. Voice pipeline auto-starts at boot.

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>"
```

---

## Task 11: End-to-end smoke test + tuning + README updates

Run the full 9-step smoke test from the spec, tune speeds/thresholds against real behavior, and write the M3 README sections. This is the "M3 done except for wake-word swap" gate.

**Files:**
- Modify: `firmware/main/voice_pipeline.c` (tuning constants only — no structural change)
- Modify: `README.md` (root — add `## Voice control (M3)` section, close resolved open questions)
- Modify: `firmware/README.md` (refresh — current version is stale, describes pre-TLC59108 drive path; add voice section)

**Interfaces:**
- Consumes: everything from Tasks 1–10.
- Produces: shippable M3 firmware.

- [ ] **Step 1: Run the full 9-step smoke test**

Copy-paste the block below into a session log, tick each line as it passes.

```
[ ] 0. ./scripts/upload.sh --erase-all monitor     # first flash after partition change
[ ] 1. Boot log ends with "REPL ready", voice model partition mounted
[ ] 2. es-verify                                   # ES8311 id=0x83
[ ] 3. voice-record 3 → sox → afplay               # audible voice on host
[ ] 4. voice-stats                                 # dropped_frames=0
[ ] 5. "Hi ESP" → LED blue, wake_count++
[ ] 6. "Hi ESP, forward" → LED green flash, wheels forward
[ ] 7. "Hi ESP, stop" → LED green flash, brake
[ ] 8. motor 40 40 (REPL) while pipeline active → wheels smooth, no stutter
[ ] 9. led 255 0 0 (REPL), then "Hi ESP" mid-sentence → LED blue → green → red
```

If step 8 stutters: verify bit-bang task pinning to core 0, verify `voice_pipeline_task` is on core 1. If still stuttering: disable `se_init` (drops PSRAM traffic) and retest. If STILL stuttering: mark as an escalation candidate for external pull-ups (see spec risk section).

- [ ] **Step 2: Tune speed defaults based on the physical rover**

Say `Hi ESP, forward` and observe. If the rover feels sluggish or too fast, adjust `V_FWD`/`V_BACK`/`V_TURN`/`V_STEP` at the top of `voice_pipeline.c`. Note the values you chose in the commit message. Rebuild and retest.

- [ ] **Step 3: Tune confidence threshold based on false-positive/negative rate**

Run through 20 wake+command cycles, count false rejections (voice said "forward" clearly, got NACK) and false acceptances (rover moved on a word that wasn't in the vocabulary). Adjust `CONF_THRESHOLD` in `voice_pipeline.c`:
- False rejects too high → lower to 0.5
- False accepts too high → raise to 0.7

Commit the value chosen.

- [ ] **Step 4: Add `## Voice control (M3)` section to root `README.md`**

Insert between the `## Motor drive (M1)` section and `## Roadmap`. Include:

- Wake phrase (currently "Hi ESP", "Hi Quarky" post-Task-12)
- The seven commands with their motor effect
- LED semantics table (idle / listening / ok / nack)
- Flashing note: `./scripts/upload.sh --erase-all monitor` needed for the first M3 flash
- Model provenance link to `docs/model-provenance.md`
- Skainet order status

Also under `## Open questions → Resolved during M3`, add entries for anything you actually resolved (ES8311 pins found, MCLK frequency programmed, motor auto-stop timeout confirmed/added).

- [ ] **Step 5: Refresh `firmware/README.md`**

The current `firmware/README.md` describes a pre-TLC59108 world (motors via LEDC PWM directly on GPIO 1/2). That's obsolete — motors go via bit-bang I²C → TLC59108 → DRV8833. Rewrite the peripheral table and REPL list to match reality. Add:

- New voice commands: `voice-start`, `voice-stop`, `voice-stats`, `voice-record <sec>`, `voice-inject-test <id>`
- New diagnostics: `es-init`, `es-verify`
- Note about `--erase-all` on first M3 flash

- [ ] **Step 6: Verify open question #1 (M1 motor safety timeout)**

Grep `main.c` and `command_bus.c` for anything that periodically stops the motor if no command has arrived. If it exists, note the timeout value in the README. If it doesn't exist, this is a NEW task — add a 10 s watchdog in `command_bus.c` that publishes `CMD_STOP` if no `CMD_MOTOR` has arrived in the last 10 s. Voice `forward` semantics rely on this; adding it here is in-scope for M3.

If you add the watchdog, cover it with a host test in `command_bus_test.c` and commit separately before finishing this task.

- [ ] **Step 7: Commit README + tuning**

```bash
git add README.md firmware/README.md firmware/main/voice_pipeline.c
git -c user.name=Gekkotron -c user.email=60887050+Gekkotron@users.noreply.github.com \
  commit -m "M3 task 11: end-to-end smoke test passing; README updated for voice

Speed defaults tuned: V_FWD=<n> V_BACK=<n> V_TURN=<n> V_STEP=<n>.
Confidence threshold set to <0.5|0.6|0.7> based on <n> field trials.
firmware/README.md refreshed (was describing pre-TLC59108 motor drive).
Root README gains Voice control section and closes M3-resolved open
questions.

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>"
```

---

## Task 12: Skainet wake-word swap (post-milestone, gated on Skainet delivery)

> **Status: FUTURE WORK.** Task 0 is deferred (2026-09-06 decision to ship against stock `wn9_hiesp`). This task activates if and when a Skainet order is placed and delivered — the procedure is unchanged.

Executed only when the Skainet order from Task 0 delivers `wn9_hiquarky.bin`. Kept as a plan task so the swap procedure is documented in one place.

**Files:**
- Create: `firmware/model/wn9_hiquarky.bin`
- Modify: `firmware/main/voice_pipeline.c` (one line — model name string)
- Modify: `docs/model-provenance.md` (fill in Skainet order details)

**Interfaces:**
- Consumes: everything.
- Produces: rover responds to "Hi Quarky" instead of "Hi ESP".

- [ ] **Step 1: Drop the delivered model into place**

```sh
cp ~/Downloads/wn9_hiquarky.bin firmware/model/wn9_hiquarky.bin
ls -lh firmware/model/wn9_hiquarky.bin   # verify size
```

- [ ] **Step 2: Change the model name in `voice_pipeline.c`**

```c
// - cfg->wakenet_model_name = "wn9_hiesp";
// + cfg->wakenet_model_name = "wn9_hiquarky";
```

- [ ] **Step 3: Update `docs/model-provenance.md`**

Fill in the Skainet row with real order number, delivery date, and version tag from the delivered ZIP.

- [ ] **Step 4: Rebuild and flash**

The `model` partition contents change (a new file was added), so re-flashing the partition is needed:

```sh
cd firmware && idf.py build
./scripts/upload.sh monitor
```

`--erase-all` NOT required — the partition layout is unchanged.

- [ ] **Step 5: Field test on hardware**

Say `Hi Quarky` clearly at ~30 cm. Confirm LED goes blue. Repeat 10 times to sanity-check reliability. Then run through the seven commands.

If Hi Quarky doesn't trigger:
- Double-check `esp_srmodel_init` loads `wn9_hiquarky` (grep boot log).
- Confirm the delivered model's ESP-SR runtime version matches `2.0.0`.
- If Espressif's Skainet delivery is against a newer ESP-SR, either downgrade the model or bump `idf_component.yml`'s pin (bumping requires re-running Tasks 9–11's smoke tests).

- [ ] **Step 6: Update the root `README.md`**

Change the wake phrase from "Hi ESP" to "Hi Quarky" in the `## Voice control (M3)` section from Task 11.

- [ ] **Step 7: Commit**

```bash
git add firmware/model/wn9_hiquarky.bin firmware/main/voice_pipeline.c \
        docs/model-provenance.md README.md
git -c user.name=Gekkotron -c user.email=60887050+Gekkotron@users.noreply.github.com \
  commit -m "M3 task 12: swap wake word to custom 'Hi Quarky' (Skainet)

Delivered wake model file added to firmware/model/; voice pipeline points
at wn9_hiquarky. Provenance recorded with order number and delivery date.
No architecture change.

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>"
```

---

## Definition of done (copied from spec)

M3 is done when:
1. All 9 smoke-test steps in Task 11 Step 1 pass on real hardware.
2. Host tests (`make -C firmware/test/host`) pass — `es8311_init_test`, `command_bus_test`, `led_indicator_test`.
3. Root `README.md` has a `## Voice control (M3)` section.
4. `docs/model-provenance.md` has an entry for every `.bin` under `firmware/model/`.
5. Stock backup at `firmware-backup/stock_firmware.bin` still validates against its `.sha256` (never touched during M3 work).

Custom "Hi Quarky" landing is NOT a done-gate — it's Task 12, executed post-milestone when Skainet delivers.
