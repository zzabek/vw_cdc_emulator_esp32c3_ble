# VW RCD300 / RCD310 CD Changer Emulator + BLE HID Controller (ESP32-C3)

An ESP32-C3 (`lolin_c3_mini`) firmware that emulates a factory Volkswagen 6-disc CD changer on the VW Quadlock CDC bus and translates radio button presses into **Bluetooth Low Energy (BLE) HID** media and keyboard (`F1`–`F24`) commands for Android/iOS smartphones.

Because it acts as a standard Bluetooth HID peripheral (not an A2DP sink), it pairs seamlessly alongside an external Bluetooth audio receiver (such as a dedicated A2DP module wired into the CDC audio inputs), letting your factory VW head unit control playback, navigation, or **Key Mapper** actions directly from the radio's front-panel buttons.

---

## Credits & Protocol Inspiration

- Inspired by **[ValeraBat/VWCDC_Driver](https://github.com/ValeraBat/VWCDC_Driver)** and the classic **`vwcdpic`** reverse-engineered VW CD changer protocol.

---

## Features

- **Full VW CDC Bus Emulation (`vw_cdc`)**:
  - 62.5 kHz SPI Mode 1 bit-bang transmitter (`CLOCK` + `DATA IN`) with full startup handshake (`ANNOUNCE_PRESENCE` → `LOAD_DISC` → `LEAD_IN` → `PLAYING`).
  - IRAM pulse-width ISR receiver (`DATA OUT`) decoding `0x53 0x2C <cmd> <~cmd>` head-unit frames.
  - Displays live status (`WAITING 88` while BLE is disconnected, track `01..99`, `mm:ss` timer, `SCAN`, and `MIX` indicators).
- **BLE HID Combo Device (`ble_hid`, NimBLE)**:
  - Report ID `1`: HID Consumer Control (`0x0001..0x03FF` — Play/Pause, Next, Prev, Stop, Volume, Mute).
  - Report ID `2`: HID Keyboard `F1`–`F24` (`0xF001..0xF018`) for neutral mapping in apps like Android *Key Mapper*.
- **Always-On Auto-Learn Mode (`btn_map`)**:
  - Starts with an **empty** button mapping table (no pre-seeded button assumptions).
  - Every newly observed `53 2C <cmd> <~cmd>` packet received from your VW radio is automatically captured into the table and persisted to NVS flash.
  - The Web UI displays helpful VW button hints (e.g. *Likely: DISC 1 (RCD300/310)*, *Likely: NEXT TRACK*) for recognized command bytes so configuring captured buttons is effortless.
- **Sticky vs. Momentary Disc Display Behavior**:
  - **Sticky Disc**: pressing the button switches the radio display to `CD 1`–`CD 6` and stays there.
  - **Momentary Disc**: flashes `CD 1`–`CD 6` on the radio display for `250 ms` and automatically returns to the active sticky disc (non-blocking), ideal for virtual macro/shortcut buttons.
- **Configurable BLE Keepalive**:
  - Periodically sends a configurable neutral HID key (default `0xF008` / `F8` every `5000 ms`) while connected to prevent host sleep/disconnect. Enable, disable, adjust interval, or change the key directly from the Web UI.
- **Modern Embedded Web UI & OTA**:
  - If no BLE device connects within `10 s` after power-on, ESP32-C3 automatically starts a Wi-Fi AP (`RCD310`, open network) with an embedded dark-mode configuration portal (`http://192.168.4.1` or `http://vw-bt.local`) and wireless firmware updates at `/update`.

---

## Hardware & Wiring (VW Quadlock 12-Pin Connector 4)

The CD changer port on VW **RCD300 / RCD310** head units uses **Multi-pin Connector 4 (12-pin blue/black Quadlock block)**.

### Connector Pinout Layout (Looking at the Connector Face)

```text
+-----------------------------------+
|  1    2    3    4    5    6       |
|  7    8    9   10   11   12       |
+-----------------------------------+
```

### Complete 12-Pin Quadlock Table

| Quadlock Pin | Signal Name (VW Diagram N91-10016) | Connection / Function |
| :---: | :--- | :--- |
| **1** | AUX audio input, left (+) | *Unused for CDC* |
| **2** | AUX audio input, ground (-) | *Unused for CDC* |
| **3** | **CD changer audio ground (-)** | Audio GND from external Bluetooth A2DP receiver |
| **4** | **CD changer voltage supply (+12V)** | Main +12V supply (max 1.5 A) → step-down / power stage |
| **5** | Not assigned | *Do not connect* |
| **6** | **CD changer bus `DATA OUT`** | Radio → ESP32-C3 **`GPIO 10`** (`CDC_DATA_OUT_PIN`) |
| **7** | AUX audio input, right (+) | *Unused for CDC* |
| **8** | **CD changer left audio (+)** | Left audio channel from external Bluetooth A2DP receiver |
| **9** | **CD changer right audio (+)** | Right audio channel from external Bluetooth A2DP receiver |
| **10** | **CD changer control signal** | Switched +12V enable signal from head unit (power gating / sense) |
| **11** | **CD changer bus `DATA IN`** | ESP32-C3 **`GPIO 3`** (`CDC_DATA_IN_PIN`) → Radio |
| **12** | **CD changer bus `CLOCK`** | ESP32-C3 **`GPIO 0`** (`CDC_CLK_PIN`) → Radio |

### ESP32-C3 (`LOLIN C3 Mini`) Summary

| ESP32-C3 GPIO | Macro in `src/config.h` | VW Quadlock Pin 4 | Direction |
| :---: | :--- | :---: | :--- |
| `GPIO 0` | `CDC_CLK_PIN` | Pin 12 (`CLOCK`) | ESP32-C3 → Radio |
| `GPIO 3` | `CDC_DATA_IN_PIN` | Pin 11 (`DATA IN`) | ESP32-C3 → Radio |
| `GPIO 10` | `CDC_DATA_OUT_PIN` | Pin 6 (`DATA OUT`) | Radio → ESP32-C3 |
| `GND` | — | Chassis GND / Pin 3 | Common ground |

> **Electrical Safety Note:**
> Automotive power rails (Pins 4 and 10) operate at **+12V–14.4V**. Always power the ESP32-C3 through an automotive-rated buck converter (12V → 5V/3.3V) and verify bus voltage compatibility (`DATA OUT` / `DATA IN` / `CLOCK` — use appropriate series protection / level shifting if your head unit drives 5V logic on `DATA OUT`).

---

## Configuration (`src/config.h`)

All identity strings, GPIO pins, FreeRTOS stack sizes, and default timings are centralized in [`src/config.h`](src/config.h):

```cpp
#define FW_VERSION                  "v4.0-BLE-HID"

#define DEVICE_BLE_NAME             "RCD310"
#define DEVICE_WIFI_SSID            "RCD310"
#define DEVICE_MDNS_HOST            "vw-bt"
#define DEVICE_MANUFACTURER         "VW-CDC-Emulator"

#define CDC_CLK_PIN                 0
#define CDC_DATA_IN_PIN             3
#define CDC_DATA_OUT_PIN            10

#define BUTTON_DEBOUNCE_MS          300U
#define WEBUI_AP_DELAY_MS           10000U
#define DISPLAY_TRACK_WAIT_BT       88
#define MOMENTARY_FLASH_MS          250U

#define KEEPALIVE_DEFAULT_ENABLED     true
#define KEEPALIVE_DEFAULT_INTERVAL_MS 5000U
#define KEEPALIVE_DEFAULT_HID_CODE    0xF008
```

---

## Building & Flashing (PlatformIO + pioarduino)

This project targets **Arduino-ESP32 Core 3.x** via **[pioarduino](https://github.com/pioarduino/platform-espressif32)** (`51.03.04` in [`platformio.ini`](platformio.ini)). Web UI files in [`web/`](web/) are embedded directly into flash `.rodata` (`board_build.embed_txtfiles`), so **no separate filesystem (`uploadfs`) step is needed**.

### Option 1: VS Code + PlatformIO IDE
1. Install [Visual Studio Code](https://code.visualstudio.com/) and the **PlatformIO IDE** extension.
2. Open the project folder in VS Code.
3. Wait for PlatformIO to download the `pioarduino` ESP32 platform and libraries (`AsyncTCP`, `ESPAsyncWebServer`, `NimBLE-Arduino`).
4. Connect the **LOLIN C3 Mini** via USB-C and click **PlatformIO: Upload** (or **Build** to generate `.pio/build/lolin_c3_mini/firmware.bin`).

### Option 2: PlatformIO CLI or Windows `compile.cmd`
Run `compile.cmd` in the project root to build and export an OTA-ready `firmware.bin` (with its exact byte size and MD5 checksum) directly into the repository root:
```cmd
compile.cmd
```
Or invoke PlatformIO CLI manually:
```bash
# Build firmware (embeds web/index.html, web/style.css, web/app.js, web/ota.html)
pio run

# Flash over USB
pio run -t upload

# Open serial monitor (115200 baud, USB-CDC)
pio device monitor -b 115200
```

### Wireless OTA Update & Brick Protection
1. Let the ESP32-C3 boot without connecting your phone via BLE for 10 seconds so the Wi-Fi AP starts.
2. Connect your phone or laptop to the Wi-Fi network **`RCD310`**.
3. Open `http://192.168.4.1/update` (or `http://vw-bt.local/update`).
4. Select `firmware.bin` (generated by `compile.cmd`) and click **Flash Firmware**.

**Built-in OTA safety protections (for headless in-car installations without USB access):**
- **Pre-flash validation & MD5 verification**: Browser checks the ESP32 image header (`0xE9`), file size bounds, and computes the full **MD5** checksum; ESP32-C3 verifies exact byte length, MD5 match, and bootable partition header before switching boot slots.
- **Interrupt & flash isolation**: CDC RX interrupts and BLE advertising are cleanly paused during flash writes, NVS writes are blocked, and concurrent or mutating API calls are rejected (`409` / `503`), with a 20 s stall watchdog and disconnect cleanup.
- **Automatic Bootloader Rollback**: Newly flashed firmware stays in `PENDING_VERIFY` until tasks, SoftAP WebUI (or 15 s of stable BLE operation) start cleanly. If a bad build crashes or panics on boot, the ESP32-C3 bootloader automatically rolls back to the previous working slot.
- **Manual Rollback**: You can also revert to the previous OTA slot anytime using **Rollback to Previous Firmware** on `/update`.

---

## Using the Web Portal & Auto-Learn

1. Turn on the radio (or power ESP32-C3) without a BLE connection for `10 s` until the `RCD310` Wi-Fi AP starts.
2. Open **`http://192.168.4.1`** in your browser.
3. **Auto-Learn Mode (always active)**:
   - On first boot, the button table starts empty.
   - Press any physical button on your VW radio. Whenever an unknown `53 2C <cmd> <~cmd>` packet arrives, it immediately appears in **Last Radio Packet**, is automatically appended to the table as `CMD_0xXX`, and is saved to NVS flash.
   - For common VW command bytes, the Web UI shows a subtle hint below the packet hex (e.g. `Likely: DISC 1 (RCD300/310)`) to help identify each button.
   - Give each captured row a friendly name, assign a **BLE HID Action** (Media control, `F1`–`F24`, or custom hex), and choose a **Radio Display Behavior** (`Sticky Disc`, `Momentary Disc`, `Track +1/-1`, `Scan`, `Mix`, or `None`).
   - Click **Save All Changes** to commit your edits to NVS flash.
4. **BLE Keepalive**:
   - Adjust the keepalive toggle, interval (`1000–60000 ms`), and HID key code in the **BLE Keepalive** card and click **Save Keepalive**.
5. **Re-pairing After Descriptor Changes**:
   - Use **Delete Bonds & Restart** in the portal, then forget `RCD310` in your phone's Bluetooth settings before pairing again.

---

## Project Structure

```text
├── platformio.ini      # pioarduino ESP32-C3 configuration & embedded web assets
├── compile.cmd         # Windows helper: builds and copies OTA-ready firmware.bin
├── web/                # Web portal & OTA frontend (embedded into flash .rodata)
│   ├── index.html
│   ├── ota.html
│   ├── style.css
│   └── app.js
└── src/
    ├── config.h        # Centralized #define settings, names, GPIO pinout & defaults
    ├── main.cpp        # FreeRTOS task orchestration, command queue & display state
    ├── vw_cdc.h/.cpp   # VW CDC bus protocol driver (bit-bang TX + IRAM PWM RX)
    ├── ble_hid.h/.cpp  # NimBLE HOGP combo HID device (Consumer Media + F1..F24)
    ├── btn_map.h/.cpp  # Dynamic NVS button mapping, Learn Mode & Keepalive store
    └── bt_webui.h/.cpp # AsyncWebServer API, deferred NVS/BLE loop & OTA handler
```
