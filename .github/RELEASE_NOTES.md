# OpenQuarkyRover — first release

Web UI + live camera stream + safe Wi-Fi control, running on the STEMpedia Quarky Intellio (ESP32-S3-WROOM-1-N16R8, OV5640 camera, TLC59108 → DRV8833 motor chain, on-board servo, WS2812 LED).

Boots into a WPA2 soft-AP; connect to **`OpenQuarkyRover`** (password **`quarky1234`**) and open **`http://192.168.4.1/`** from a phone or laptop.

## What works

- 🎥 Live camera stream (OV5640, QVGA JPEG, MJPEG over HTTP on port 81)
- 🏎️ Drive controls — single throttle on M1, servo steering with configurable presets
- ⚙️ Settings panel — camera flip H/V, M1 / M2 enable, servo left / right angles
- 💡 On-board RGB LED colour picker
- 🛑 Motor-reversal brake — 150 ms pause when direction flips to prevent battery-sag brown-outs
- 🚨 Wi-Fi safety mode — motors halt + servo centres + LED breathes red the moment the last client disconnects; resumes on reconnect

## Flash it

Download the three binaries below (`bootloader.bin`, `partition-table.bin`, `openquarkyrover_m1_test.bin`).

Install `esptool` if you don't have it:

```bash
pip install esptool
```

Find your serial port:

```bash
ls /dev/cu.usbserial-*     # macOS
ls /dev/ttyUSB*            # Linux
# Windows: check Device Manager for the COM number
```

Flash everything in one shot (replace the port):

```bash
esptool.py --chip esp32s3 --port /dev/cu.usbserial-XXX --baud 460800 \
  --before default_reset --after hard_reset write_flash \
  --flash_mode dio --flash_size 16MB --flash_freq 80m \
  0x0     bootloader.bin \
  0x8000  partition-table.bin \
  0x10000 openquarkyrover_m1_test.bin
```

If it can't enter download mode automatically: hold **BOOT** on the board, tap **RESET**, release **BOOT**, then rerun the flash command.

After flashing, open a serial monitor at 115200 baud. You should see the boot log end with something like:

```
wifi_ap:  Wi-Fi soft-AP up: SSID="OpenQuarkyRover" pass="quarky1234" ip=192.168.4.1
http_ctl: HTTP control on :80, MJPEG stream on :81 — open http://192.168.4.1/
cam:      camera up: OV5640 QVGA JPEG q12 (XCLK=GPIO14)
```

Then join the AP and open `http://192.168.4.1/`.

## Restore stock STEMpedia firmware

A full flash dump of the stock firmware lives in the repo at `quarkyRoverBackup/stock_firmware.bin` (Git LFS, SHA-256 anchored). To go back:

```bash
esptool.py --chip esp32s3 --port /dev/cu.usbserial-XXX write_flash 0x0 quarkyRoverBackup/stock_firmware.bin
```

## Known limitations

- **Microphone** — not supported in this release. The audio codec (ES8311) has been reverse-engineered but the I²S RX / PLL-lock chain still stalls; work in progress on the `worktree-2026-09-06-m3-voice-control` branch.
- **No OTA yet** — every update is a full serial flash. On the roadmap.
- **Settings persist client-side only** — the settings panel stores flip/motor/servo prefs in the browser's `localStorage`. A different phone starts with defaults. NVS-backed persistence is planned.

## Board / pin reference

All the pin mappings that took a while to reverse-engineer from the stock firmware live in the repo's `README.md` under the "Confirmed pin map so far" table — motor / servo / buttons / LED / ES8311 audio codec / OV5640 camera / SD card, each with signal-index evidence from the GPIO matrix.
