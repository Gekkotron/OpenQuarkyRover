# M3 — Voice control (wake word + command spotting)

**Status:** design approved, not yet implemented.
**Author:** Gekkotron
**Date:** 2026-09-06
**Milestone:** M3
**Depends on:** M1 (motor drive via bit-bang I²C, RGB LED, servo, buttons, REPL). Does **not** depend on M2 — voice runs standalone.

---

## Summary

Bring the STEMpedia Quarky Intellio's on-board MEMS microphone online and use it for wake-word-gated command spotting. The rover listens continuously for **"Hi Quarky"**, then accepts one of seven English commands (`forward`, `backward`, `left`, `right`, `stop`, `faster`, `slower`) which dispatch to the same handlers the M1 REPL uses. Feedback via the existing RGB LED (blue = listening, green flash = accepted, red flash = rejected).

All inference is on-device: Espressif ESP-SR (AFE + WakeNet + MultiNet-EN). No Wi-Fi, no cloud, no companion device. The custom "Hi Quarky" wake word is a paid Skainet Studio order; development proceeds against the stock "Hi ESP" wake word and the custom model drops in as a single file swap when it arrives.

---

## Decisions locked in (source: brainstorming session 2026-09-06)

| # | Decision | Notes |
|---|---|---|
| 1 | Full M3: wake word + command spotting | Not just capture; not streaming |
| 2 | Pin discovery: brute-force I²C first, stock-firmware introspection for I²S | Hybrid unblocks both control and audio pins |
| 3 | Wake word: custom "Hi Quarky" via Espressif Skainet Studio (paid) | Fallback: stock "Hi ESP" ships if Skainet is delayed/blocked |
| 4 | Vocabulary: minimal drive set (7 commands, English) | `forward`, `backward`, `left`, `right`, `stop`, `faster`, `slower` |
| 5 | Language: English (MultiNet-EN with G2P) | French deferred to M3.5 (would need a second Skainet order) |
| 6 | Feedback UX: RGB LED cues only | Speaker output deferred (no I²S TX / WAV playback scope) |

---

## Approaches considered

- **A. Full Espressif stack (ESP-SR: AFE + WakeNet + MultiNet)** — **chosen.** Native to ESP-IDF v5.3, best recognition quality, matches the "on-device" ethos on line 8 of the root README.
- **B. WakeNet + custom lightweight command classifier (MFCC + DTW/kNN).** Rejected — massively worse recognition, requires collecting training clips, no upside over (A) at our scale.
- **C. WakeNet on-device + cloud STT (Whisper / Google) for commands.** Rejected — requires Wi-Fi (M2 not built), always-on internet, latency and cost, and directly contradicts the on-device voice-control goal.

---

## Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                            ESP32-S3                                  │
│                                                                      │
│  Core 0 (I/O + control)             Core 1 (audio + inference)      │
│  ┌──────────────────────────┐       ┌──────────────────────────┐   │
│  │  REPL (UART)             │       │  I²S RX capture task     │   │
│  │  Motor bit-bang I²C      │       │    ↓ (ring buffer)       │   │
│  │  Servo (LEDC)            │       │  ESP-SR AFE              │   │
│  │  Button poll             │       │    (AEC/NS/VAD)          │   │
│  │       │                  │       │    ↓                     │   │
│  │       ▼                  │       │  WakeNet ("Hi Quarky")   │   │
│  │  ┌──────────────┐        │       │    ↓ trigger             │   │
│  │  │ Command Bus  │◄───────┼───────│  MultiNet-EN             │   │
│  │  │  (queue)     │        │       │    (7 commands)          │   │
│  │  └──────┬───────┘        │       └──────────────────────────┘   │
│  │         │                │                                       │
│  │  ┌──────▼────────┐       │       ┌──────────────────────────┐   │
│  │  │ Dispatcher    │       │       │  LED indicator FSM       │   │
│  │  │  motor/servo  │───────┼──────►│  (idle/listen/ok/nack)   │   │
│  │  │  led/stop     │       │       │  ↓                       │   │
│  │  └───────────────┘       │       │  WS2812B (RMT, GPIO 48)  │   │
│  └──────────────────────────┘       └──────────────────────────┘   │
│                                                                      │
│  I²C bus #A (motor, bit-bang):   GPIO 2 (SDA), GPIO 1 (SCL)         │
│  I²C bus #B (ES8311 control):    GPIO ?, GPIO ?  (TBD by discovery)  │
│  I²S RX (mic):                   MCLK/BCLK/LRCK/DIN — TBD           │
└─────────────────────────────────────────────────────────────────────┘
```

### New components

| Path | Purpose |
|---|---|
| `firmware/main/es8311.{c,h}` | ES8311 codec driver over bit-bang I²C: register init, ADC config, mic gain, start/stop |
| `firmware/main/audio_capture.{c,h}` | I²S RX peripheral + DMA ring buffer, producer task on core 1 |
| `firmware/main/voice_pipeline.{c,h}` | ESP-SR wrapper: AFE + WakeNet + MultiNet, emits `voice_event_t` to command bus |
| `firmware/main/command_bus.{c,h}` | FreeRTOS queue + dispatcher. REPL and voice both publish |
| `firmware/main/led_indicator.{c,h}` | State machine over the WS2812B. Arbitrates with manual LED commands |
| `firmware/main/i2c_bitbang.{c,h}` | Extract `bb_*` from existing `main.c` into reusable module |
| `firmware/main/pin_map.h` | Central home for ES8311 pin constants once discovered |
| `firmware/partitions.csv` | New partition scheme with dedicated `model` partition |
| `firmware/model/` | Model blobs (`.bin`), populated into partition at build time |
| `firmware/test/host/` | Host-runnable unit tests (Unity, standalone Makefile) |
| `docs/model-provenance.md` | Provenance table for every `.bin` shipped in `firmware/model/` — outside the partition source dir so it's not packaged into flash |

### New REPL commands introduced in M3

| Verb | Kind | Purpose |
|---|---|---|
| `es-scan` | throwaway (deleted after pin discovery) | Brute-force ES8311 I²C control pins |
| `es-verify` | permanent diagnostic | Read ES8311 product ID (`0xFD` → `0x83`) and version on the configured pins |
| `voice-record <sec>` | permanent dev diagnostic | Dump N seconds of raw 16 kHz mono PCM to UART. Consumed by host with `sox`. Emitted directly by `audio_capture`, not through the command bus |
| `voice-stats` | permanent diagnostic | Print `audio_capture_dropped_frames()`, `voice_pipeline_wake_count()`, `voice_pipeline_reject_count()` |
| `voice-inject-test <cmd_id>` | dev-only | Publish a synthetic voice command (bypasses audio stack). Verifies dispatcher path without the whole voice pipeline up |

### Refactors to M1

- `main.c` slims down: motor / servo / button / REPL become thin shells that publish to the command bus instead of acting directly. Mechanical refactor, no behavior change from the REPL user's perspective.
- `bb_*` functions extracted from `main.c` into `i2c_bitbang.c`.
- LED writes migrate through `led_indicator` so voice states can preempt.

### Order of work

```
Day 1  ┃━━━ (external, weeks)                              Skainet order: "Hi Quarky"
       ┃
Step 1 ┣━━ Pin discovery (brute-force I²C + stock introspection)     [BLOCKS all]
Step 2 ┣━━ Extract bb-I²C module + ES8311 register-level bring-up    [needs pins]
Step 3 ┣━━ I²S RX raw capture + PCM dump smoke test                  [needs codec]
Step 4 ┣━━ Command bus refactor + LED indicator FSM                  [parallel with 3]
Step 5 ┣━━ ESP-SR integration with STOCK wake word ("Hi ESP")        [needs 3]
Step 6 ┣━━ Wire voice → command bus → motor/led                      [needs 4, 5]
Step 7 ┣━━ Field-test tuning (gain, noise, false positives)          [needs 6]
       ┃
End    ┗━━ Drop in Skainet "Hi Quarky" model, retest                 [when order arrives]
```

Skainet turnaround is the long pole. Everything else runs in parallel against the stock wake word so the "Hi Quarky" model swap on day-N is a one-line change, not a critical dependency.

### Out of scope for M3

- Speaker output / TTS / beeps (LED-only feedback)
- Wi-Fi / web streaming (that's M2, skipped over)
- French commands (deferred to M3.5)
- OTA partition scheme (deferred to M4)
- Custom keyword expansion beyond the 7 commands (rebuild-only when added)

---

## Pin discovery

Goal: end this step with six pin constants in `firmware/main/pin_map.h`.

```c
// pin_map.h (target)
#define ES8311_I2C_SDA       GPIO_NUM_?
#define ES8311_I2C_SCL       GPIO_NUM_?
#define ES8311_I2S_MCLK      GPIO_NUM_?
#define ES8311_I2S_BCLK      GPIO_NUM_?
#define ES8311_I2S_LRCK      GPIO_NUM_?  // aka WS
#define ES8311_I2S_DIN       GPIO_NUM_?  // mic → ESP32
// (DOUT for speaker deferred — out of scope for M3)
```

### Candidate GPIO shortlist

The ESP32-S3-WROOM-1-N16R8 exposes ~34 usable GPIOs. Eliminated before hardware contact:

- Already used by M1: 0 (servo), 1 (exp SCL), 2 (exp SDA), 7 (buttons), 48 (LED)
- USB / UART bridge (must not touch): 19, 20, plus UART0 pair
- Strapping pins that must idle a specific state: 0, 3, 45, 46
- SPI0 for flash/PSRAM (unavailable): 26–32
- Camera bank (~18 pins, block once found) — constraint, not exclusion

Leaves roughly **4, 5, 6, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 21, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 47**.

Prior: ESP-ADF example defaults on WROOM-1 are MCLK=16, BCLK=9, LRCK=45, DIN=10, DOUT=8, I²C=17/18. STEMpedia's `intellioAudio.py` was built against ADF and likely stayed close.

### Brute-force I²C scan (Step 2B)

New throwaway REPL command: `es-scan`. Iterates candidate `(MCLK, SDA, SCL)` triples, drives a 4 MHz square wave on MCLK via LEDC, and probes for ES8311 at address `0x18` using the existing bit-bang I²C. A bare ACK is not enough — a hit must confirm by reading product-ID register `0xFD` and expecting `0x83`.

Expected duration: ~15 minutes worst case (~17.5k probes × ~2 ms).

Failure mode we accept: scan finds nothing. Not blocking — falls through to Step 2C.

### Stock-firmware introspection (Step 2C)

Runs regardless of Step 2B outcome, because brute-force can't map the four I²S pins.

- **2C-i** (~30 s if it works): reflash stock, tap `Ctrl+C` during boot to catch MicroPython REPL, `import intellioConstants; print(vars(intellioConstants))`.
- **2C-ii** (~1–2 h fallback): extract `intellioConstants` bytecode from `factory.bin`'s `mp_frozen_mpy_data` region and disassemble with `mpy-tool.py`. Constants read as literal `NAME = <int>` in the disassembly.

### Verification (Step 2D)

New permanent REPL diagnostic: `es-verify`. Initializes ES8311 on the candidate pins via bit-bang I²C, reads product ID `0xFD` (expect `0x83`), reads version `0xFE`, logs both. On pass: pins get written to `pin_map.h`, README updated, throwaway `es-scan` deleted, `es-verify` kept.

### Deliverables

1. `firmware/main/pin_map.h` with six confirmed constants
2. README section "ES8311 pin discovery" documenting the method that worked
3. `es-verify` committed; `es-scan` deleted or archived under `scripts/discovery/`

### Escape hatch

If scan + REPL + bytecode all fail (very low probability), fall back to multimeter tracing of the QFN pads (~2–4 h manual work).

---

## Audio pipeline

Three subsystems stack cleanly. See ASCII diagram in the Architecture section above for the data flow.

### ES8311 codec driver

New files: `firmware/main/es8311.{c,h}`.

**Bus**: bit-bang I²C over ES8311's own SDA/SCL pins. We ship it bit-bang from day one — M1 memory (`espidf-i2c-master-broken`) is unambiguous that the ESP-IDF v5.3 `i2c_master` peripheral fails with internal-only pull-ups, and the ES8311 bus has the same characteristics. If external 2.2 kΩ pull-ups get added later, swapping to the peripheral driver is a single-file change.

**Init sequence** (exact register writes from ES8311 datasheet + ESP-ADF's `es8311.c` reference):

1. Reset codec: `[0x00] = 0x1F`, then `[0x00] = 0x00`
2. Configure clock manager for slave mode, MCLK = 4 MHz, sample rate 16 kHz
3. Set ADC path: mic input, analog PGA = 0 dB, digital gain from `mic_gain` flag (stock uses `3`)
4. Enable ADC power
5. Log product ID (`0xFD` → `0x83`) and version (`0xFE`)

**Public API** (small on purpose):

```c
esp_err_t es8311_init(void);
esp_err_t es8311_set_mic_gain_db(int gain_db);   // -12..+30
esp_err_t es8311_read_id(uint8_t *id, uint8_t *ver);
esp_err_t es8311_start(void);
esp_err_t es8311_stop(void);
```

### I²S RX capture

New files: `firmware/main/audio_capture.{c,h}`.

**Format**: 16 kHz, 16-bit, mono. Matches WakeNet/MultiNet expectations — anything else costs resampling.

**Peripheral**: ESP-IDF v5.3's `i2s_std` (modern driver).

- Slot mode: mono, left channel, 16-bit width in 16-bit slot
- Clock source: PLL, MCLK multiplier ×256 (→ 4.096 MHz, matches the codec init)
- DMA: 8 buffers × 320 frames × 2 bytes = 5.1 KB (each buffer = 20 ms, total 160 ms headroom)
- Read granularity: 512 samples (32 ms frames, matching ESP-SR expectations)

**Task**: single reader on core 1, priority 22. Reads 512-sample frames from `i2s_channel_read`, pushes into a `QueueHandle_t` (depth 4 frames = 128 ms). If queue is full, drop the frame — logged in `audio_capture_dropped_frames()`. Any non-zero drop count under normal operation signals `voice_pipeline` task starvation.

**Public API**:

```c
esp_err_t audio_capture_start(QueueHandle_t out_queue);
esp_err_t audio_capture_stop(void);
uint32_t  audio_capture_dropped_frames(void);
```

### ESP-SR wrapper (AFE + WakeNet + MultiNet)

New files: `firmware/main/voice_pipeline.{c,h}`.

**Dependency**: `espressif/esp-sr` via `idf_component.yml`, **pinned to `==2.0.0`** (models are runtime-coupled — a floating dependency is a silent-breakage trap).

**Models**: memory-mapped from the `model` partition via `esp_srmodel_init("model")`. No file I/O, no LittleFS involvement.

**AFE configuration**:

```c
srmodel_list_t *models = esp_srmodel_init("model");
afe_config_t *afe_cfg = afe_config_init("MMR",  // Mono, Mic, Reference-off (no AEC)
                                         models,
                                         AFE_TYPE_SR,
                                         AFE_MODE_LOW_COST);
afe_cfg->wakenet_init = true;
afe_cfg->voice_communication_init = false;  // command spotting, not full-duplex call
afe_cfg->aec_init = false;                  // no speaker → no echo to cancel
afe_cfg->se_init  = true;                   // speech enhancement (denoise) on
afe_cfg->vad_init = true;
afe_cfg->wakenet_model_name = "wn9_hiesp";  // Placeholder — Skainet "wn9_hiquarky" drops in later
```

AEC is deliberately off — no speaker output in M3, no reference channel to cancel against. Saves ~150 KB PSRAM and reduces latency.

**MultiNet configuration**:

```c
esp_mn_iface_t *mn = esp_mn_handle_from_name("mn7_en");
model_iface_data_t *mn_data = mn->create("mn7_en", 5760);  // 5760 ms speech timeout
esp_mn_commands_alloc(mn, mn_data);
esp_mn_commands_add(1, "forward");
esp_mn_commands_add(2, "backward");
esp_mn_commands_add(3, "left");
esp_mn_commands_add(4, "right");
esp_mn_commands_add(5, "stop");
esp_mn_commands_add(6, "faster");
esp_mn_commands_add(7, "slower");
esp_mn_commands_update();
```

Command IDs (1..7) are stable — the command bus keys off IDs, not strings.

**Pipeline task**: single task, core 1, priority 5. Feeds 32 ms frames to AFE, fetches results, tracks the listening window opened by a wake trigger, runs MultiNet inside that window, emits events to the command bus. Confidence threshold defaults to 0.6; tuning band 0.5–0.7 depending on field results.

**Public API**:

```c
esp_err_t voice_pipeline_start(void);
esp_err_t voice_pipeline_stop(void);
uint32_t  voice_pipeline_wake_count(void);
uint32_t  voice_pipeline_reject_count(void);
```

### Task / core affinity

| Task | Core | Priority | Stack | Justification |
|---|---|---|---|---|
| REPL (uart_rx) | 0 | 5 | 4 KB | Existing M1, untouched |
| Motor bit-bang (per-call) | 0 | high (temp boost) | — | Bit-bang runs inline in dispatcher; sensitive to jitter |
| `audio_capture` | 1 | 22 | 4 KB | Must service DMA promptly |
| `voice_pipeline` | 1 | 5 | 8 KB | Heavy but interruptible; shares core 1 with capture |
| `led_indicator` | 0 | 3 | 2 KB | Periodic; low priority is fine |

Bit-bang I²C on core 0 while voice runs on core 1 removes CPU-contention risk. PSRAM bus is shared and could still leak jitter into bit-bang timing — see risks (6C).

### RAM / flash budget

| Item | Internal SRAM | PSRAM | Flash |
|---|---:|---:|---:|
| AFE + SE + VAD state | ~40 KB | ~200 KB | — |
| WakeNet (`wn9_hiesp` or `wn9_hiquarky`) | ~30 KB | ~500 KB | ~600 KB |
| MultiNet-EN 7 commands (`mn7_en`) | ~50 KB | ~1.5 MB | ~2 MB |
| I²S DMA + queues | ~8 KB | — | — |
| **Subtotal M3 addition** | ~130 KB | ~2.2 MB | ~2.6 MB |
| Available | 512 KB SRAM | 8 MB PSRAM | 16 MB flash |

Comfortable. Flash budget drives the new partition scheme.

---

## Command bus + LED indicator

### Why a command bus

M1's REPL parser is also its dispatcher — verbs match directly to hardware calls. Adding voice as a second input source would either duplicate parsing or force voice to serialize into REPL strings for re-parsing. Neither is acceptable. A tagged-union `command_t` published to a single dispatcher queue solves both, and stays under 12 bytes per command.

### Command type

```c
typedef enum {
    CMD_NONE = 0,
    CMD_MOTOR,        // args: int8_t left, int8_t right   (-100..100)
    CMD_STOP,         // no args
    CMD_LED_RGB,      // args: uint8_t r, g, b             (manual — user set)
    CMD_LED_STATE,    // args: led_state_t                 (voice FSM claim)
    CMD_SERVO,        // args: uint8_t channel, uint16_t us
    CMD_BB_TLC_SET,   // args: uint8_t ch, uint8_t pct     (diagnostic)
    // ... one enum per M1 REPL verb, plus voice-specific ones
} command_id_t;

typedef struct {
    command_id_t id;
    enum { SRC_REPL, SRC_VOICE, SRC_INTERNAL } source;
    union {
        struct { int8_t left, right;            } motor;
        struct { uint8_t r, g, b;               } led_rgb;
        struct { led_state_t state;             } led_state;
        struct { uint8_t channel; uint16_t us;  } servo;
        struct { uint8_t channel, percent;      } bb_tlc;
    } as;
} command_t;

esp_err_t command_bus_publish(const command_t *cmd);
```

`source` earns its keep in LED arbitration and in one-line command logging.

### Dispatcher

Single FreeRTOS queue (depth 8), single consumer task on core 0. Handlers are plain functions — no plugin registry, no hooks. If M4 wants pluggability, we add it then.

```c
static void command_dispatcher_task(void *arg) {
    command_t cmd;
    while (xQueueReceive(cmd_queue, &cmd, portMAX_DELAY)) {
        log_command(&cmd);
        switch (cmd.id) {
        case CMD_MOTOR:     bb_motor_set(cmd.as.motor.left, cmd.as.motor.right); break;
        case CMD_STOP:      bb_motor_set(0, 0); break;
        case CMD_LED_RGB:   led_indicator_apply_rgb(cmd.source, cmd.as.led_rgb); break;
        case CMD_LED_STATE: led_indicator_apply_state(cmd.source, cmd.as.led_state.state); break;
        // ...
        }
    }
}
```

Diagnostic commands that don't act on the robot (`bb-scan`, `i2c-selftest`, `es-verify`, etc.) don't go through the bus — they call their target directly to avoid masking bugs behind queue latency.

### REPL becomes a thin parser

The REPL loop shrinks to *parse → build `command_t` → publish*. Handlers move out of `main.c` into per-verb parser functions in either `command_bus.c` or dedicated files.

### Voice publishes to the same bus

```c
static void on_command_recognized(int cmd_id, float prob) {
    if (prob < 0.6f) { emit_event(VOICE_EVENT_LOW_CONF); return; }
    command_t c = { .source = SRC_VOICE };
    switch (cmd_id) {
    case 1: c.id = CMD_MOTOR; c.as.motor = (typeof(c.as.motor)){ +60, +60 }; break; // forward
    case 2: c.id = CMD_MOTOR; c.as.motor = (typeof(c.as.motor)){ -60, -60 }; break; // backward
    case 3: c.id = CMD_MOTOR; c.as.motor = (typeof(c.as.motor)){ -60, +60 }; break; // left  (pivot)
    case 4: c.id = CMD_MOTOR; c.as.motor = (typeof(c.as.motor)){ +60, -60 }; break; // right
    case 5: c.id = CMD_STOP;  break;
    case 6: c.id = CMD_MOTOR; c.as.motor = scale_current(+20); break;               // faster
    case 7: c.id = CMD_MOTOR; c.as.motor = scale_current(-20); break;               // slower
    }
    command_bus_publish(&c);
}
```

**Persistent motion semantics**: `forward` drives until told to stop. The command sets state; the dispatcher doesn't decay it. Voice-controlled rovers that auto-stop after N seconds feel broken. The user can say `stop`, or the M1 safety timeout can kick in (see open question 1 — verify this timeout exists before locking).

Speed defaults (`+60`, `+20` step) live as `#define`s at the top of `voice_pipeline.c`; tune post-hardware.

### LED indicator FSM

New files: `firmware/main/led_indicator.{c,h}`.

Five states:

| State | Color | Trigger |
|---|---|---|
| `LED_STATE_IDLE` | dim white (16, 16, 16) | Default |
| `LED_STATE_LISTENING` | solid blue (0, 0, 255) | `VOICE_EVENT_WAKE` — held for MultiNet's speech window (~5.7 s) or until command / timeout |
| `LED_STATE_OK` | green flash (200 ms) | Command recognized & dispatched. Returns to `IDLE` after flash |
| `LED_STATE_NACK` | red flash (200 ms) | Command rejected (low confidence) or `VOICE_EVENT_TIMEOUT`. Returns to `IDLE` |
| `LED_STATE_USER_RGB` | whatever REPL asked for | User did `led 255 0 128`. Persists until another `led` or voice preempts |

### Arbitration rule

Voice FSM wins during any non-IDLE voice state; on return-to-IDLE the LED reverts to the last user RGB (state preserved, not lost).

```
       REPL sets red ────► LED_STATE_USER_RGB(255,0,0)     [LED = red]
       voice wakes    ────► LED_STATE_LISTENING            [LED = blue]
       voice recognized ──► LED_STATE_OK (200 ms flash)     [LED = green]
       flash done     ────► LED_STATE_USER_RGB(255,0,0)     [LED = red again]
```

Implementation: `led_indicator` holds `current_state` and `saved_user_rgb`. Voice events push state; return-to-idle restores from `saved_user_rgb` (or dim-white default).

**Corner case**: user types a new `led` while voice is listening. `saved_user_rgb` updates but the LED stays blue until listening ends. Matches user intuition (input took effect but is deferred) and avoids flicker.

---

## Partition scheme + Skainet + build system

### New partition table

Current stock scheme has no room reserved for ESP-SR models. Proposed replacement (in `firmware/partitions.csv`):

```
# Name,     Type, SubType,  Offset,      Size,   Notes
nvs,        data, nvs,      0x9000,      24K
phy_init,   data, phy,      0xf000,      4K
factory,    app,  factory,  0x10000,     4M      shrunk — M1+M3 app fits in <1 MB
model,      data, spiffs,   0x410000,    4M      WakeNet + MultiNet blobs
sys,        data, littlefs, 0x810000,    1M
vfs,        data, littlefs, 0x910000,    6M      grows to reclaim slack from stock
storage,    data, littlefs, 0xf10000,    640K    unchanged from stock
# 320K unallocated at 0xfb0000..0x1000000 — reserved for M4 (OTA metadata / TFLite model)
```

**Arithmetic check** (16 MB flash = 0x1000000):
`0x10000` app-start + 4M + 4M + 1M + 6M + 640K = `0xfb0000` — fits with 320 KB slack at the top of flash.

Notes:

- `model` uses `spiffs` subtype because that's what `esp_srmodel_init("model")` expects for memory-mapping. Naming quirk — it's really just a raw region we mmap.
- **No OTA partitions in M3.** OTA is real design work (needs `ota_0`/`ota_1`/`otadata`, dual-app layout, model-update-vs-app-update story). Deferred to M4.
- Migration story: first M3 flash wipes existing contents. Stock backup at `firmware-backup/stock_firmware.bin` is the safety net; M1 has nothing on `vfs` we care about; `user_voice.wav` is already extracted to disk.

`sdkconfig.defaults` additions:

```
CONFIG_PARTITION_TABLE_CUSTOM=y
CONFIG_PARTITION_TABLE_CUSTOM_FILENAME="partitions.csv"
```

### Model packaging

`firmware/main/idf_component.yml`:

```yaml
dependencies:
  espressif/esp-sr: "==2.0.0"
```

`firmware/main/CMakeLists.txt` adds:

```cmake
spiffs_create_partition_image(model ../model FLASH_IN_PROJECT)
```

New git-committed directory `firmware/model/` — contains **only** what gets packaged into the `model` partition (no README inside, or `spiffs_create_partition_image` would flash it):

```
firmware/model/
├── wn9_hiesp.bin                 # stock, dev placeholder
├── mn7_en/
│   ├── mn7_en.bin
│   └── mn7_en.index
└── wn9_hiquarky.bin              # Skainet — added when it arrives
```

Provenance for every packaged `.bin` lives **outside** the partition source, at `docs/model-provenance.md` (source URL, ESP-SR version, license, order number for Skainet). Every `.bin` must have an entry. No unlabeled blobs.

### Skainet Studio order

**Timeline reality:**
- Custom wake word turnaround: typically 2–4 weeks (Espressif does the training)
- Cost: per-word, budget on the order of a few hundred USD
- Deliverable: `wn9_hiquarky.bin` plus a version tag

**Action item, day 1:**
1. Order at [https://www.espressif.com/en/products/software/esp-skainet](https://www.espressif.com/en/products/software/esp-skainet) → Custom Wake Word Service
2. Wake phrase `"Hi Quarky"`, language English (US), ESP-SR runtime `2.0.0`
3. Log order number + expected delivery date in an "External dependencies" section of this spec

**Drop-in procedure when delivered:**
```sh
cp ~/Downloads/wn9_hiquarky.bin firmware/model/srmodels/
# Edit voice_pipeline.c:
#   afe_cfg->wakenet_model_name = "wn9_hiquarky";
./scripts/upload.sh monitor
# Add entry to firmware/model/README.md
```

No architecture change, no partition change. Five-minute integration by design.

**Fallback ladder** if Skainet is delayed/blocked:
1. Ship M3 with `wn9_hiesp`, README notes wake phrase is "Hi ESP"
2. If Skainet is permanently blocked: fall back to push-to-talk on the L/R button (already have the button driver)
3. If neither is acceptable: DIY-trained model (M-plus scope, weeks of work)

### Toolchain

- ESP-IDF v5.3 (current M1 baseline) — supported by `esp-sr` 2.0
- PSRAM required (already the case). Verify `CONFIG_SPIRAM=y`, `CONFIG_SPIRAM_MODE_OCT=y`, `CONFIG_SPIRAM_USE_MALLOC=y` in sdkconfig
- `CONFIG_COMPILER_OPTIMIZATION_PERF=y` for release builds (~20% faster inference)
- **No ESP-ADF dependency.** Too heavy for M3 scope — we use raw I²S directly and let ESP-SR's own AFE handle audio front-end

### Flashing

`scripts/upload.sh` gains a `--erase-all` flag for the first M3 flash:

```sh
./scripts/upload.sh --erase-all monitor   # first time after partition change
./scripts/upload.sh monitor               # subsequent
```

Documented in README under Flashing. Stock backup restoration procedure unchanged.

---

## Testing

### What's testable and how

| Layer | Test kind | Where | Coverage |
|---|---|---|---|
| `command_bus` (types, parsers, dispatcher routing) | Unit test | Host (native `gcc`) | Command struct construction, verb parse table, routing with mocked handlers |
| `led_indicator` FSM | Unit test | Host | State transitions, arbitration corner case from arbitration rule |
| `es8311` register init sequence | Table test | Host, mocked I²C log | Verify byte sequence matches datasheet recipe — catches typos |
| `i2c_bitbang` timing | — | Not unit-testable | Oscilloscope on real bus. Manual only |
| `audio_capture` I²S DMA + queue | Smoke test | On device | `voice-record 3 > out.pcm`, play back on host |
| `voice_pipeline` (AFE + wake + MultiNet) | Integration | On device | Say wake word + command, verify event fires. Model behavior IS the feature |
| End-to-end (voice → command bus → motor) | Field test | On device | "Hi Quarky, forward" → wheels spin |

New: `firmware/test/host/` with `command_bus_test.c`, `led_indicator_test.c`, `es8311_init_test.c`. Uses Unity (vendored in ESP-IDF, invoked standalone via `Makefile`). `make -C firmware/test/host` runs the suite in ~1 s. No ESP-IDF setup needed to run host tests.

**No on-device test infrastructure**. The board has 2 buttons + serial. Espressif's on-device Unity runner still needs manual command entry to drive tests — not worth building yet. Manual smoke tests in the README are the honest bar.

### Smoke test procedure (README section for M3)

Per-component, in order. Each step must pass before proceeding:

```
0. ./scripts/upload.sh --erase-all monitor     # first flash after partition change
1. Boot log ends with "REPL ready", model partition mounted (log line from esp-sr).
2. es-verify                                   # ES8311 product ID = 0x83, version logged
3. voice-record 3 > /tmp/mic.pcm               # 3 s raw capture, ear-check on host:
       sox -t raw -r 16000 -e signed -b 16 -c 1 /tmp/mic.pcm /tmp/mic.wav
       afplay /tmp/mic.wav                     # should hear your voice, no digital garbage
4. voice-stats                                 # dropped_frames = 0, wake_count = 0
5. Say "Hi ESP" (or "Hi Quarky" once model swap):
       LED goes solid blue, wake_count increments in voice-stats
6. Say "Hi ESP, forward":
       LED green flash, wheels spin forward at 60%
7. Say "Hi ESP, stop":
       LED green flash, wheels brake
8. motor 40 40 (REPL) while voice pipeline running:
       Wheels spin smoothly, no stutter. Voice pipeline still responsive.
9. led 255 0 0 (REPL), then "Hi ESP" mid-sentence:
       LED goes blue, then flashes green/red on command, then reverts to red.
       (Verifies arbitration.)
```

Steps 1–4 are pre-M3 gates (would pass with just codec + capture + LED refactor, no inference). Steps 5–9 are the M3 acceptance test.

---

## Risks (ranked)

**High — bit-bang I²C timing under voice load.**
M1 motor path assumes a mostly-idle CPU. Voice inference on core 1 is heavy; PSRAM bus is shared and could leak jitter into bit-bang timing on GPIO 1/2. **Mitigation**: step 8 of the smoke test is the specific probe. Escalation ladder if it fails:
1. Boost bit-bang task priority temporarily around each write
2. Disable AFE preprocessing (`se_init = false`) — drops PSRAM traffic ~40%
3. Add external 2.2 kΩ pull-ups on `A1`/`A2`, switch motor I²C to peripheral driver

Not blocking the design; blocking the ship if it manifests.

**Medium — ES8311 needs an MCLK we can't produce cleanly.**
If discovery reveals MCLK is expected on a pin we can't clock cleanly at the required rate, or the codec needs an MCLK ratio our I²S PLL can't hit, fall back to a dedicated LEDC-generated MCLK on a spare GPIO (same trick used in the discovery scan). Couple of hours of driver work, not architectural.

**Medium — MultiNet false positives in normal conversation.**
Config gates MultiNet inside the wake-triggered listening window only, so ambient chatter shouldn't fire commands. Ship at 0.6 confidence threshold; watch `reject_count` during dev. Tuning band 0.5–0.7.

**Medium — Skainet delivery slips or is blocked.**
Covered by the fallback ladder above. Design doesn't depend on custom model landing.

**Low — WakeNet false triggers.**
Espressif's models have known FPR of ~1/24h in quiet environments; more with TV/music. Blue LED cycling briefly for no reason is annoying but harmless — rover only acts on a follow-up command MultiNet must also accept.

**Low — Partition wipe surprise for anyone running M1.**
`--erase-all` first-flash is documented in the Flashing section but easy to miss. Explicit warning in README.

---

## Open questions

1. Does the M1 motor 10 s auto-stop timeout actually exist? Referenced in the voice-persistent-motion semantics — needs source verification before locking. If it doesn't exist, add it (safety-critical for voice control).
2. What MCLK frequency does the stock firmware program into the ES8311? Discovery only tells us the pin. Read from register dump after `es-verify`, or reverse from `intellioAudio.py`.
3. Does WakeNet reliably fire on "Hi Quarky" played through a phone speaker (dev/CI without human presence)? Would unblock scripted end-to-end tests.
4. Long-term: is Skainet's licensing compatible with open-source firmware distribution? Model file is redistributable per Espressif's terms but the license might attach — needs a read before publishing M3.

---

## Definition of done

M3 is done when:

1. All 9 smoke-test steps in the Testing section pass on real hardware
2. Host tests (`make -C firmware/test/host`) pass
3. README's M3 section is written (`## Voice control (M3)`) with wake phrase, vocabulary, LED semantics, flashing note
4. `firmware/model/README.md` provenance table has entries for every `.bin` shipped
5. Stock backup at `firmware-backup/stock_firmware.bin` is still valid (never touched during M3 work)

Custom "Hi Quarky" landing is **not** a done-gate — it's a follow-up drop-in.

---

## External dependencies

| Dependency | Vendor | Status | Notes |
|---|---|---|---|
| ESP-IDF | Espressif | in use (v5.3) | Existing M1 baseline |
| esp-sr (WakeNet + MultiNet + AFE) | Espressif | to add (`==2.0.0`) | Via `idf_component.yml` |
| `wn9_hiquarky.bin` (custom wake word) | Espressif Skainet Studio | **to order day 1** | Long pole; fallback is `wn9_hiesp` |
| `mn7_en` MultiNet-EN | Espressif | free, ships with esp-sr | — |

---

## References

- Root README, `## Roadmap` M3 entry
- Memory: `[[espidf-i2c-master-broken]]` — explains why ES8311 I²C also ships bit-bang
- Memory: `[[tlc59108-channel-map]]` — motor path context for coexistence testing
- [ES8311 datasheet — Everest Semiconductor](https://www.everest-semi.com/pdf/ES8311%20PB.pdf)
- [ESP-SR — GitHub](https://github.com/espressif/esp-sr)
- [ESP Skainet — Custom Wake Word](https://www.espressif.com/en/products/software/esp-skainet)
