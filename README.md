# OpenQuarkyRover

An open-source alternate firmware for the **STEMpedia Quarky Intellio** — an
ESP32-S3–based AI/AR/IoT learning board (LEGO- and Arduino-compatible).

**Long-term goals:**

- On-device voice control (wake word + command spotting via ESP-SR)
- On-device adaptive behavior (TFLite Micro — learned obstacle map,
  battery-aware motor policy)
- Wi-Fi control from a browser (web UI served from the ESP32-S3 itself, no
  cloud, no companion device)

**Status:** M1 in progress. The pin map is complete for the confirmed
peripherals (LED, servo, buttons, expansion I²C). The rover now drives
under our own firmware: **`motor <L> <R>` at the REPL spins both wheels
via the TLC59108 → DRV8833 chain** — see [Motor drive](#motor-drive-m1)
below. Foundation drivers (RGB LED, servo, buttons) are on the same REPL.

---

## Hardware

Target board is the **Quarky Intellio** (STEMpedia). Verified against the
chip itself (`esptool chip-id`), the on-flash bootloader (`esptool
image-info`), the eFuse readout (`espefuse summary`), and the printed spec
sheet.

| Item | Value |
|---|---|
| MCU | ESP32-S3 QFN56 rev v0.2 — dual-core Xtensa LX7 + LP core, 240 MHz |
| Package | ESP32-S3-WROOM-1-N16R8 |
| Flash | 16 MB QSPI, DIO @ 80 MHz |
| PSRAM | 8 MB embedded (AP Memory, `AP_3v3`, 85 °C rated) |
| Wi-Fi / BT | 802.11 b/g/n 2.4 GHz + BLE 5.0 |
| MAC | `1c:db:d4:a7:d3:44` (Espressif OUI) |
| Camera | OV5640, 5 MP, autofocus, 72° FOV, physically mounted `hmirror=1, vflip=1` |
| Audio codec | ES8311 (24-bit) — drives mic capture and 8 Ω / 1 W speaker over I²S + I²C control |
| Microphone | Omnidirectional MEMS (routed through ES8311) |
| Storage | MicroSD up to 32 GB |
| RGB LED | 1× WS2812B-4020 (`NeoPixel` driver in stock firmware) |
| Buttons | 2 programmable tactile switches (L / R) + power + reset |
| Servo | 1 on-board channel (`S1`), pulse 500–2500 µs, VCC 3.7–5 V, 1 A max |
| Expansion header | 6 GPIOs + 3.3 V + GND — connects to Mini or Full Expansion Board |
| Battery | Li-ion 1000 mAh / 3.7 V (~3.5 h continuous streaming) |
| USB | Type-C via USB-to-serial bridge (device appears as `/dev/cu.usbserial-*`, **not** native USB-JTAG) |

### Expansion boards observed

- **Quarky Expansion Board** — 4-pin header (`A2 · A1 · V · −`) between
  Intellio and expansion, all silkscreen labels confirmed by multimeter.
  `A1` and `A2` are the **I²C bus**, *not* motor PWM as originally guessed:
  - `A1` = **SDA** = GPIO 2 (multimeter-traced end-to-end to TLC59108 pin 19)
  - `A2` = **SCL** = GPIO 1 (multimeter-traced end-to-end to TLC59108 pin 18)
  - `V` = 3.3 V rail out, `−` = GND
  
  On-board silicon (confirmed from top-of-chip markings):
  - **TLC59108 (PW-20 / TSSOP-20)** — I²C 8-channel constant-current
    open-drain LED-driver, base address `0x40` (silkscreen-labeled, A0–A3
    strapped to GND). Fixed 97 kHz PWM, 8-bit duty per channel. Outputs
    can only sink; the board's own pull-ups feed the DRV8833 inputs.
  - **DRV8833** — dual H-bridge (AIN1/AIN2 for motor A, BIN1/BIN2 for
    motor B). Four of the TLC59108 channels feed its logic inputs; the
    remaining four are wired to the `P1..P4` header ports.
  
  There is *no* PCA9685 on this board — the TLC59108 fills that role.
  Ports `P1..P4` are downstream of the TLC59108, not direct ESP32 GPIOs.

---

## Stock firmware — analysis results

### Backup artifacts

Everything is stored under `firmware-backup/`:

| File | Size | Notes |
|---|---:|---|
| `stock_firmware.bin` | 16 MB | Full flash dump from offset `0x0` via `esptool read-flash`. **Plain, unencrypted bytes** — see security section. |
| `stock_firmware.sha256` | — | SHA-256 integrity anchor for the dump. |
| `efuses.txt` | 17 KB | `espefuse summary` output. |
| `partitions.bin` | 4 KB | Extracted from `stock_firmware.bin` at offset `0x8000`. |
| `factory.bin` | 9.31 MB | Application image extracted from offset `0x10000` (size 9536 KB). |
| `factory.strings.txt` | ~31 k lines | `strings -n 4` dump of the app image. |
| `vfs.spiffs.bin` | 5 MB | Extracted from offset `0xa60000`. Partition subtype says SPIFFS, but the **actual filesystem is LittleFS** (magic bytes `littlefs` at offset `0x08`, block_size 4096, block_count 1280). |
| `sys.littlefs.bin` | 1 MB | Extracted from offset `0x960000`. Fails to mount as LittleFS with the block_size=4096 / block_count=256 guess — geometry still to be reverse-engineered. |
| `extracted/vfs/` | — | Files unpacked from the `vfs` LittleFS via `littlefs-python` (see repro section). |

### Security posture (from eFuse dump)

Factory-fresh; nothing locked. Best possible situation for building
alternate firmware.

| eFuse | Value | Meaning |
|---|---|---|
| `SPI_BOOT_CRYPT_CNT` | Disable (`0b000`) | Flash encryption is OFF → the backup is plain bytes, fully analysable. |
| `SECURE_BOOT_EN` | False | Any firmware we sign will boot. |
| `WR_DIS` / `RD_DIS` | 0 / 0 | No eFuses have been permanently burned. |
| `DIS_DOWNLOAD_MODE` and siblings | all False | All USB/UART download paths remain open — recovery is always possible. |
| `DIS_PAD_JTAG` / `SOFT_DIS_JTAG` | False / 0 | JTAG usable if we ever want a hardware debugger. |
| `KEY_PURPOSE_0..5` / `BLOCK_KEY0..5` | USER / all zeros | No cryptographic keys are burned in. |

**Preserved calibration (do not overwrite):** `BLOCK1` and `BLOCK2` hold
factory ADC1 / ADC2, temperature (`-12.6 °C` reference), RTC and DIG voltage
calibration. Any `espefuse burn_efuse` on those blocks would degrade
hardware accuracy permanently. Our firmware and tooling must never touch
them.

Handy chip-unique identifiers now known:

- MAC: `1c:db:d4:a7:d3:44`
- 128-bit optional unique ID: `23bfaa09a6e993f13d3255a210fe042a`

### Partition table (from `stock_firmware.bin @ 0x8000`)

```
# Name,     Type, SubType,  Offset,     Size,  Notes
nvs,        data, nvs,      0x9000,     24K
phy_init,   data, phy,      0xf000,     4K
factory,    app,  factory,  0x10000,    9536K   (~9.31 MB app)
sys,        data, littlefs, 0x960000,   1M      (mount fails at 4096×256)
vfs,        data, spiffs,   0xa60000,   5M      (actually LittleFS 4096×1280)
storage,    data, littlefs, 0xf60000,   640K
```

**Consequence:** the stock scheme has **no OTA partitions** (no
`ota_0`/`ota_1`/`otadata`). If we want over-the-air updates in the
alternate firmware, we will design a different partition scheme.

### Bootloader identification (from `esptool image-info`)

- Built with **ESP-IDF v5.2.5-3-g6e74c52893-dirty** (3 commits past v5.2.5,
  local modifications)
- Compile time: `Mar 11 2026 18:50:42`
- Flash 16 MB DIO @ 80 MHz, min chip rev v0.0 / max v0.99
- Checksums valid, `Validation hash: 349198e3cd435ea4e93d7b277b5db614677dc8bf41ff62e7c463829486dc8641`

### Runtime is MicroPython + ESP-ADF + a custom camera API

Confirmed by artifacts extracted from the `vfs` LittleFS and by symbols
inside `factory.bin`:

- `boot.py` (139 B) — empty MicroPython template (WebREPL commented out;
  no user code has ever been installed).
- Frozen module set includes `intellio.py`, `intellioCamera.py`,
  `intellioAudio.py`, `expansion.py`, `expansion_addon.py`,
  `intellioConstants.py`, plus `_boot.py`, `boot.py`, `webrepl_cfg.py`,
  `flashbdev.py`, `inisetup.py`, `asyncio/*.py`, `dht.py`, `espnow.py`.
- `audioFiles/*.py` — a large set of pre-baked WAV clips (`Connect.py`,
  `Disconnect.py`, `IntellioIntro.py`, number wavs `one.py`–`nine.py`,
  `turningLeft.py`, `objectdetection.py`, `rec_start.py`, `rec_end.py`, …)
  wrapped as Python modules.
- Path evidence for the source tree: `/home/sheetal01/QuarkyCAM_Micropython/`
  (MicroPython fork), `/home/sheetal01/esp-adf/…/es8311/es8311.c` (ESP-ADF
  audio HAL), `/home/sheetal01/QuarkyCAM_Micropython/micropython-camera-API/`
  (camera API). This code is **not public** on
  [github.com/stempedia](https://github.com/stempedia).

### API surface discovered in the compiled app

Numeric pin values are compiled into MicroPython bytecode and therefore
invisible to `strings`, but the API names are not. Enough leaks to
plan our own drivers:

| Subsystem | Names visible in `factory.bin` |
|---|---|
| Motors | `INITIALISEMOTOR`, `RUNMOTOR`, `MOTORCALIB`, `MOTORDIR`, `changemotormax`, `LeftmotorDirFlag`, `RightmotorDirFlag`, `expansion motor error(stop):` |
| Servos | `SERVO`, `SERVO1..4`, `SERVO_PIN`, `SERVOMOVEINTIME`, `SERVOPULSE`, `_init_servo_flags`, `allservo`, `Moving servo N from …`, `expansionosc` |
| Buttons | `PUSHBUTTONS`, `READPUSHBUTTON`, `L button Pressed` |
| RGB LED | `NeoPixel` (WS2812B driver) |
| Camera | `OV5640`, `Autofocus only supported on OV5640` |
| Audio codec | `ES8311`, `es8311_codec_init`, `ES8311 in Master/Slave mode` |
| Expansion | `Expansion board I2C initialized successfully.`, `MiniExpansion Init Error:` |
| BLE GATT | `_QUARKY_UUID`, `_QUARKY_SERVICE`, `_QUARKY_RX`, `_QUARKY_TX` (Nordic-UART-style) |

### Configuration insights from `flags.json`

Extracted from the `vfs` partition. Highlights:

- **Stock Wi-Fi AP** — SSID `Quarky Intellio`, password `12345678`. A
  separate STEMpedia-provisioned AP appears as `sp_ssid=CAM` /
  `sp_password=STEM@1010`.
- **Servo limits** — `servomin=500`, `servomax=2500` µs.
- **Motor limits** — `motorLMax=100`, `motorRMax=100`.
- **Camera** — mounted upside-down: `hmirror=1`, `vflip=1`. Sane defaults:
  `quality=55`, `frame_size=11`, `contrast=1`, `saturation=1`,
  `sharpness=2`, `awb_gain=1`, `exposure_ctrl=1`, `lenc=1`.
- **Audio pipeline** — `speaker_volume=100`, `mic_gain=3`, `denoise=2`,
  `aec2=1`. ESP-ADF handles echo cancellation and denoising in the stock
  firmware.
- **Expansion** — `expansionMotorLeft/Right = 1` in the observed config;
  `servoexmin1..12` and `servoexmax1..12` allocated for up to 12 expansion
  servos (physical port count on the standard Full Expansion is 8, so 4
  slots are reserved for humanoid mode).
- **Robot form-factor offsets** — `r_frontLeftOff`, `r_frontRightOff`,
  `r_backLeftOff`, `r_backRightOff`, `r_headOff` (quadruped) and
  `hum_lh/rh/lhip/rhip/lf/rf` (humanoid).
- **Line-following** — `pbLThreshMin/Max` = 1200/1800, `pbRThreshMin/Max`
  = 700/800 (calibrated IR sensor thresholds — real values, not defaults).
- **RFID** — `masterrfid` field present (empty on our unit).
- **Firmware version** reported by the config: `1.0.0`.

### User data present on the backed-up board

- `firmware-backup/extracted/vfs/user_voice.wav` (128 KB, from
  `/user_voice.wav` in the `vfs` partition) — an unknown voice recording
  captured by the stock firmware at some point. Preserved as a copy; the
  live partition will be erased when we flash our own firmware.

---

## Confirmed pin map so far

**Definitively known:**

| Signal | ESP32-S3 GPIO | Notes |
|---|---:|---|
| Expansion header `A1` = **I²C SDA** | **GPIO2** | Multimeter-traced to TLC59108 pin 19 (SDA) |
| Expansion header `A2` = **I²C SCL** | **GPIO1** | Multimeter-traced to TLC59108 pin 18 (SCL) |
| On-board servo `S1` | **GPIO0** | Multimeter-confirmed; strapping pin, must idle HIGH at reset |
| On-board buttons (L + R, shared ADC) | **GPIO7** | Resistor-ladder; ADC1 ch 6 |
| On-board RGB LED (WS2812B) | **GPIO48** | Verified against the M1 test firmware |

**Button ladder — measured ADC bands (12-bit, 12 dB atten, ~0..3.1 V full scale):**

| State | ADC raw range | Peak observed |
|---|---|---:|
| NONE (idle) | 0..150 | 0 |
| **R pressed** | 150..1000 | 731 |
| **BOTH pressed** | 1000..1350 | ~1049–1251 |
| **L pressed** | 1350..1900 | 1498 |

Line idles near GND (pulled low). Each button connects the line to VCC
through its own resistor, giving a distinct voltage per single press.
BOTH pressed sits *between* the two single-press bands rather than
above them — the ladder is probably diode-decoupled, so the two buttons
don't simply parallel their resistances. Classification is
threshold-based; there are no gaps between adjacent bands.

**Architecturally inferred** (values unknown, functions certain):

- Internal-only pins to be identified: OV5640 camera bank (~18 pins),
  ES8311 I²S (BCK, WS, DIN, DOUT) + I²C control, MEMS mic routing.

**Path to resolve the rest**:

1. Attempt a live MicroPython REPL introspection on the running stock
   firmware over USB serial (`import intellioConstants;
   print(vars(intellioConstants))`) — 5 seconds if REPL is accessible.
2. Fallback: locate `mp_frozen_mpy_data` inside `factory.bin` and
   disassemble the `intellioConstants` module with `mpy-tool.py`.

---

## Motor drive (M1)

The rover's two DC motors sit behind the Quarky Expansion Board's DRV8833
H-bridge. The DRV8833's four logic inputs (AIN1, AIN2, BIN1, BIN2) are
driven by four of the TLC59108's eight PWM outputs; the remaining four
outputs feed the `P1..P4` header ports. The ESP32-S3 has no direct
electrical path to the motor drivers — everything is proxied over I²C to
the TLC59108 at `0x40`, which then generates the 97 kHz PWM the DRV8833
sees on its inputs.

### Discovered channel mapping

Reversed by cycling each TLC59108 output at 50 % duty for 2 s (the
`bb-tlc-sweep` REPL command) and watching which wheel moved. Confirmed
on the physical rover:

| TLC59108 channel | LEDOUT reg / bits | DRV8833 input | Effect when driven alone |
|---|---|---|---|
| **0** | LEDOUT0 bits 1..0 | B side (M2) | M2 spins one direction |
| **1** | LEDOUT0 bits 3..2 | B side (M2) | M2 spins the other direction |
| **2** | LEDOUT0 bits 5..4 | A side (M1) | M1 spins one direction |
| **3** | LEDOUT0 bits 7..6 | A side (M1) | M1 spins the other direction |
| 4..7 | LEDOUT1 | routed to `P1..P4` header outputs (not motors) |

Channels 4..7 aren't wired to the H-bridge — they feed the `P1..P4`
header ports for external LEDs / small servos / other TLC59108-driven
loads. Motor commands leave `LEDOUT1` untouched so `P1..P4` state
survives across motor writes.

### Drive scheme (fast-decay PWM)

TLC59108 outputs are open-drain sinks; the expansion board pulls the
lines HIGH with external resistors going into the DRV8833 inputs.

- `LDR = OFF`: sink disabled → line HIGH → DRV8833 sees `1`.
- `LDR = PWM at N %`: sink active `N %` of the 97 kHz period → line
  LOW `N %` of the time → DRV8833 input toggles.

To drive one motor at `+N %`:

- **+dir channel** → `LDR = PWM`, `PWMn = N * 255 / 100`.
- **−dir channel** → `LDR = OFF` (idles HIGH via the pullup).

The DRV8833 then alternates between `(L, H) = drive` and
`(H, H) = brake` at 97 kHz, yielding smooth `N %` duty in the chosen
direction with fast-decay braking between PWM cycles. `motor 0 0` sets
both channels `LDR = OFF` → both inputs HIGH → `(H, H) = brake` on both
motors (electrical stop, wheels resist rotation).

### REPL commands (from the M1 test firmware)

```
bb-tlc-init            once at boot — initialise TLC59108 via bit-bang I²C
motor <L> <R>          drive both motors, each -100..100 (signed %)
stop                   both motors braked
bb-tlc-set <ch> <pct>  drive one raw TLC59108 channel (0..7 at 0..100 %)
bb-tlc-sweep           cycle every channel 2 s each — used to derive the map
bb-scan                bit-banged I²C address scan on GPIO 2 / GPIO 1
```

Examples:

```
quarky> bb-tlc-init
bb-tlc-init: OK — TLC59108 initialised via bit-bang @ 0x40
quarky> motor 60 60          # both wheels forward at 60 %
quarky> motor 80 -80         # pivot in place
quarky> stop
```

### Why bit-bang instead of the ESP-IDF `i2c_master` driver

The first M1 attempt drove the TLC59108 through ESP-IDF v5.3.2's
`i2c_master` peripheral driver. On this board it returns `ESP_ERR_TIMEOUT`
on **every** probe and transaction, even though:

- The chip is powered (`VCC = 3.3 V`, `RESET = 3.3 V`).
- The SDA/SCL wiring is correct (multimeter continuity from
  ESP32 GPIO 2/1 to TLC59108 pin 19/18, plus a datasheet check that
  pin 19 = SDA, pin 18 = SCL — [TI SLDS156B](https://www.ti.com/lit/ds/symlink/tlc59108.pdf)).
- The stock firmware drives the same chip through the same wires
  without issue.
- `i2c-selftest` shows the ESP32 pads idle high, drive low, and
  re-idle high correctly.
- A **bit-banged** I²C probe on the same GPIOs finds the chip at
  `0x40`, `0x48` (all-call), and `0x4b` (sub-call) — every ACK
  the TLC59108 is supposed to send.

The peripheral itself fails to fire `I2C_EVENT_DONE`. This is a
known-ish class of failure with the new ESP-IDF v5 `i2c_master` driver
on ESP32-S3 when relying only on the ~45 kΩ internal pull-ups — the
driver's own error message says "please check … pull-ups are correctly
set up". Rather than solder external pull-ups onto the expansion
header, the M1 firmware ships a bit-bang I²C transport (see `bb_*`
functions and `cmd_bb_*` REPL commands in `firmware/main/main.c`). It
manually toggles GPIO 1 and GPIO 2 as open-drain outputs and reads back
the ACK bit — same protocol, more forgiving of rise-time. All motor
drive goes through this path.

The peripheral-based `tlc-init` / `tlc-set` / `tlc-sweep` commands
remain in the source as diagnostic tools (they'll start working the
day someone adds external 2.2 kΩ pull-ups, or the day upstream ESP-IDF
fixes the driver). The motor path does not depend on them.

### Test procedure to verify the motor path on a bare board

1. Flash M1 (`./scripts/upload.sh monitor`).
2. Confirm boot log ends with `REPL ready`. Ignore the warning
   `TLC59108 did not ACK on probe (0x40)` — that's the broken
   peripheral driver, not the chip.
3. At the REPL: `bb-tlc-init` should print
   `OK — TLC59108 initialised via bit-bang @ 0x40`. If it NACKs, the
   chip is unreachable and the whole motor path is dead — start over
   with the hardware checks in the debugging notes below.
4. `motor 40 40` — both wheels should spin. If both go the wrong way
   or L/R are swapped, adjust the `MOTOR_M*_PLUS/MINUS` constants at
   the top of the motor section in `main.c`.

### Debugging notes (kept for next time this bites)

If `bb-tlc-init` NACKs on a future board:

1. **Confirm chip has power**: multimeter TLC59108 pin 20 (VCC) → GND.
   Expect ~3.3 V.
2. **Confirm not stuck in reset**: TLC59108 pin 17 (RESET) → GND.
   Expect ~3.3 V (RESET is active-low).
3. **Confirm bus continuity**: header `A1` ↔ chip pin 19 (SDA),
   header `A2` ↔ chip pin 18 (SCL).
4. **Confirm bus lines aren't held low**: at the REPL,
   `i2c-selftest 2 1` — expect `idle: SDA=1 SCL=1`, `driven low`,
   `re-idle`, all `OK`.
5. **Sanity-check the ESP-IDF driver isn't hiding the failure**: run
   `bb-scan`. If it finds `0x40` but the peripheral doesn't, the chip
   is alive and the driver is the problem — same situation as this
   board.
6. **If everything above passes and `bb-scan` still finds nothing**:
   the chip is likely damaged (ESD, accidental short during
   probing). Reflash stock (`esptool write-flash 0
   firmware-backup/stock_firmware.bin`) and try the motors via the
   stock BLE / STEMpedia app — that tells you whether the chip is
   dead or the expansion board has some other fault.

---

## Roadmap

M0 (current) → M1 → M2 → M3 → M4. Each milestone is designed as its own
spec + PR, not a monolithic build.

- **M0 — Reverse engineering & backup** *(in progress)*
  Backup stock firmware, decode partitions, extract user filesystem,
  identify runtime, complete the pin map.

- **M1 — Foundation firmware** *(in progress)*. ESP-IDF v5.3 skeleton,
  drivers for the RGB LED (GPIO 48), on-board servo (GPIO 0), buttons
  (GPIO 7 ADC), and both DC motors via the Quarky Expansion Board's
  TLC59108 (I²C `0x40`) → DRV8833 chain. Motor drive uses a bit-bang
  I²C transport (see [Motor drive](#motor-drive-m1)); the ESP-IDF
  `i2c_master` peripheral doesn't ACK on this board with only internal
  pull-ups. Serial REPL for smoke tests. No Wi-Fi yet.

- **M2 — Wi-Fi + Web UI.** AP + STA modes, embedded HTTP + WebSocket,
  tiny SPA (vanilla JS in LittleFS): joystick, telemetry, LED color
  picker, servo slider. First milestone at which the rover is driven
  from a phone browser.

- **M3 — Voice on-device.** ES8311 I²S input via ESP-SR: wake word
  (e.g. *"Hi Quarky"*) plus a bounded command vocabulary
  (~20 phrases) mapping to the same command bus the web UI uses.

- **M4 — Adaptive layer.** TFLite Micro model co-resident with the
  control loop. Planned scope: (A) learned obstacle map from
  IMU + IR / ultrasonic, (D) battery-aware motor throttling from a
  learned discharge curve.

**Framework decision:** ESP-IDF (native), component-based. ESP-SR and
TFLite Micro integrate cleaner than under the Arduino wrapper, and
FreeRTOS lets the voice, web-server and control loop coexist without
stepping on each other's RAM.

---

## Reproducing this analysis

Prerequisites: recent `esptool` (v5.x), Python 3 with `venv`,
[littlefs-python](https://pypi.org/project/littlefs-python/), the
`gen_esp32part.py` script that ships with ESP-IDF, and the Quarky Intellio
plugged in via USB-C. `mkspiffs` is *not* required (the `vfs` partition
is LittleFS despite its declared subtype).

```sh
# 0. Verify the chip
esptool --chip esp32s3 -p /dev/cu.usbserial-<N> chip-id

# 1. Full flash backup (16 MB, ~2-3 min at 921600 baud)
esptool --chip esp32s3 -p /dev/cu.usbserial-<N> -b 921600 \
    read-flash 0 0x1000000 firmware-backup/stock_firmware.bin
shasum -a 256 firmware-backup/stock_firmware.bin > firmware-backup/stock_firmware.sha256

# 2. eFuse dump (security posture and calibration)
espefuse --chip esp32s3 -p /dev/cu.usbserial-<N> summary > firmware-backup/efuses.txt

# 3. Partition table
dd if=firmware-backup/stock_firmware.bin \
   of=firmware-backup/partitions.bin \
   bs=1 skip=32768 count=4096
python3 ~/esp/esp-idf/components/partition_table/gen_esp32part.py \
    firmware-backup/partitions.bin

# 4. Extract app + user filesystem
python3 -c "f=open('firmware-backup/stock_firmware.bin','rb'); \
    f.seek(0x10000); \
    open('firmware-backup/factory.bin','wb').write(f.read(9536*1024))"
dd if=firmware-backup/stock_firmware.bin \
   of=firmware-backup/vfs.spiffs.bin \
   bs=1 skip=$((0xa60000)) count=$((5*1024*1024))
strings -n 4 firmware-backup/factory.bin > firmware-backup/factory.strings.txt

# 5. Mount and extract vfs (LittleFS 4096 x 1280)
python3 -m venv scratchpad/venv
scratchpad/venv/bin/pip install littlefs-python
scratchpad/venv/bin/python - <<'PY'
from littlefs import LittleFS
import os
with open('firmware-backup/vfs.spiffs.bin','rb') as f: data = f.read()
fs = LittleFS(block_size=4096, block_count=1280, mount=False)
fs.context.buffer[:] = data
fs.mount()
os.makedirs('firmware-backup/extracted/vfs', exist_ok=True)
for root, dirs, files in fs.walk('/'):
    for name in files:
        fp = (root.rstrip('/') + '/' + name) if root != '/' else '/' + name
        with fs.open(fp,'rb') as src: content = src.read()
        local = os.path.join('firmware-backup/extracted/vfs', fp.lstrip('/'))
        os.makedirs(os.path.dirname(local), exist_ok=True)
        open(local,'wb').write(content)
        print(fp, len(content))
PY
```

---

## Restoring the stock firmware

The dump is unencrypted and complete. To restore the board to factory
state (same chip only — MAC-tied config in NVS carries over):

```sh
esptool --chip esp32s3 -p /dev/cu.usbserial-<N> -b 921600 \
    write-flash 0 firmware-backup/stock_firmware.bin
```

Verify with:

```sh
shasum -a 256 firmware-backup/stock_firmware.bin
# Should match the value in firmware-backup/stock_firmware.sha256
```

Do **not** run any `espefuse burn_*` command as part of restore or setup.
The calibration blocks are already programmed at the factory and are
one-way; overwriting them is unrecoverable.

---

## Open questions

- OV5640 camera pin block.
- ES8311 I²S + I²C control pins.
- Whether the stock MicroPython REPL is accessible on serial with
  `Ctrl+C` (Option 1 in M0) — determines whether we finish the pin map
  in 30 seconds or 30 minutes.
- LittleFS geometry of the `sys` partition (mount failed at 4096 × 256).
- Whether adding external 2.2 kΩ pull-ups on `A1`/`A2` (SDA/SCL) makes
  the ESP-IDF v5.3 `i2c_master` peripheral start working — would let us
  retire the bit-bang I²C path in the motor driver.

### Resolved during M1

- ~~TLC59108 does not ACK at `0x40`.~~ The chip *does* ACK — the ESP-IDF
  `i2c_master` peripheral was the problem, not the chip. See
  [Why bit-bang instead of the ESP-IDF `i2c_master` driver](#why-bit-bang-instead-of-the-esp-idf-i2c_master-driver).
  The earlier "phantom ACK at `0x0a`" was a lone bus glitch — a 3-consecutive-ACK
  filter (`bb-scan`, `tlc-diag`) sees `0x40` cleanly and rejects `0x0a`.
- ~~Which four TLC59108 channels feed AIN1/AIN2/BIN1/BIN2 of the DRV8833.~~
  Mapped via `bb-tlc-sweep`: channels 0, 1 → M2 (B-side of the H-bridge);
  channels 2, 3 → M1 (A-side); channels 4..7 → `P1..P4` header outputs.
  Full table in [Discovered channel mapping](#discovered-channel-mapping).

---

## References

- [Quarky Intellio product page — STEMpedia](https://thestempedia.com/product/quarky-intellio/)
- [Quarky Intellio documentation — STEMpedia](https://ai.thestempedia.com/docs/quarky-intellio/)
- [STEMpedia GitHub organization](https://github.com/stempedia) *(no Quarky repos — evive and Dabble only)*
- [Hack Your LEGO Bricks with the ESP32-Powered Quarky Intellio — Hackster.io](https://www.hackster.io/news/hack-your-lego-bricks-with-the-esp32-powered-quarky-intellio-d9b099ed3be8)
- [Quarky Intellio — CNX Software](https://www.cnx-software.com/2025/12/08/quarky-intellio-lego-compatible-ai-augmented-reality-iot-learning-platform/)
- [ESP-IDF](https://github.com/espressif/esp-idf) (target: v5.x)
- [ESP-SR](https://github.com/espressif/esp-sr) (on-device speech)
- [ESP-ADF](https://github.com/espressif/esp-adf) (audio codecs — ES8311)
- [littlefs](https://github.com/littlefs-project/littlefs) / [littlefs-python](https://pypi.org/project/littlefs-python/)

---

## Attribution

This project is authored under the handle **Gekkotron**. Any code,
`Cargo.toml`, `package.json` or LICENSE headers added later should use
this identity.

## License

TBD (will be picked before the first firmware release — likely
Apache-2.0 or MIT).
