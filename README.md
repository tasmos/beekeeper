# Beekeeper Honey Vending Machine — Firmware Guide

## Hardware
| GPIO | Connected To | Function |
|------|-------------|----------|
| GPIO21 | SDA | LCD 1602 + MCP23017 (shared I²C) |
| GPIO22 | SCL | LCD 1602 + MCP23017 (shared I²C) |
| GPIO23 | MOSI | 74HC595 SER/DS |
| GPIO18 | SCK | 74HC595 SRCLK |
| GPIO4  | SS/CS | 74HC595 RCLK (Latch) |
| GPIO32 | Digital IN | Coin Acceptor PULSE OUT |
| GPIO25 | Digital OUT | DRV8825 STEP |
| GPIO26 | Digital OUT | DRV8825 DIR |
| GPIO27 | Digital OUT | DRV8825 ENABLE |

---

## Building the Firmware

```bash
platformio run --environment tasmota32-beekeeper
```

Output files are in `.pio/build/tasmota32-beekeeper/`:

| File | Use |
|------|-----|
| `firmware.factory.bin` | First time USB flash only |
| `firmware.bin` | OTA upgrades only |

---

## 🔌 First Time Flash (USB)

Use this when flashing a brand new / blank ESP32.

### Option 1 — Tasmota Web Installer (Easiest, no install needed)

1. Open [https://tasmota.github.io/install/](https://tasmota.github.io/install/) in **Chrome or Edge**
2. Connect ESP32 via USB
3. Click **CONNECT** → select your USB port
4. Drag & drop `firmware.factory.bin` onto the dashed area **or** click **"Upload factory.bin"**
5. Wait for flash to complete
6. ESP32 reboots automatically ✅

### Option 2 — ESP Web Flasher (esp.huhn.me)

1. Open [https://esp.huhn.me](https://esp.huhn.me) in **Chrome or Edge**
2. Click **CONNECT** → select your USB port
3. Set offset to `0x0` and select `firmware.factory.bin`
4. Click **Flash** ✅

### Option 3 — esptool (Command Line)

```bash
esptool.py --chip esp32 --port /dev/cu.usbserial-XXXX \
  --baud 460800 write_flash \
  0x0 .pio/build/tasmota32-beekeeper/firmware.factory.bin
```

> Find your port with: `ls /dev/cu.*` (Mac) or `ls /dev/ttyUSB*` (Linux)

### Option 4 — PlatformIO Upload

Add to `platformio_override.ini` under `[env:tasmota32-beekeeper]`:
```ini
upload_port  = /dev/cu.usbserial-XXXX
upload_speed = 460800
```
Then run:
```bash
platformio run --environment tasmota32-beekeeper --target upload
```

---

## 🌐 OTA Upgrades (Wi-Fi, no USB needed)

Use this for all upgrades after first flash.

### Option 1 — Tasmota Web UI (Easiest)

1. Open `http://<ESP32-IP>` in browser
2. Go to **Firmware Upgrade**
3. Select **Upgrade by file upload**
4. Choose `firmware.bin`
5. Click **Start Upgrade** → wait ~30 seconds ✅

### Option 2 — PlatformIO OTA

Add to `platformio_override.ini` under `[env:tasmota32-beekeeper]`:
```ini
upload_protocol = espota
upload_port     = <ESP32-IP>
```
Then run:
```bash
platformio run --environment tasmota32-beekeeper --target upload
```

---

## 📶 First Boot — WiFi Setup

After first flash the ESP32 broadcasts its own WiFi access point:

1. Connect your phone/laptop to WiFi: **`tasmota-XXXXXX`**
2. Open browser → go to **`192.168.4.1`**
3. Enter your home WiFi SSID and password
4. ESP32 reboots and connects to your network
5. Find the ESP32 IP from your router → open in browser
6. Your Honey Vending UI is ready ✅

---

## ⚠️ Important Notes

- `firmware.factory.bin` → **USB first flash only** (contains bootloader + partitions + firmware)
- `firmware.bin` → **OTA upgrades only** (firmware only, no bootloader)
- Web flasher requires **Chrome or Edge** — Firefox and Safari do not support Web Serial API
- GPIO32 has no internal pull-up — uses `INPUT_PULLUP` via GPIO32 (full GPIO pin)