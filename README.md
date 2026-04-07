# USB Host to BLE Keyboard Bridge

**Version 2.0.0**

ESP32-S3 firmware that connects to a Bluetooth Low Energy (BLE) keyboard and
forwards keystrokes as VT100 terminal data through a USB-to-serial adapter
(e.g. CH340) to a target device's UART.

```
 +----------------+        BLE        +-------------+   USB Host CDC   +---------+  UART  +--------+
 | BLE Keyboard   | ───────────────── | ESP32-S3    | ════════════════ | CH340   | ====== | Target |
 | (HID device)   |    HID Reports    | (this fw)   |   VT100 serial  | USB-TTL |        | (MCU)  |
 +----------------+                   +-------------+                 +---------+        +--------+
```

## Features

- **BLE HID Host**: auto-scans and connects to BLE keyboards, with pairing,
  bonding, and encrypted connections (Just Works / NoIO)
- **USB Host CDC-ACM**: the ESP32-S3 acts as USB Host, driving a USB-to-serial
  adapter (CH340, or any CDC-ACM compliant device)
- **VT100 Terminal Emulation**: translates HID keystrokes to standard VT100/ANSI
  escape sequences
- **Automatic reconnection**: reconnects to the keyboard after disconnects
- **Polling fallback**: if BLE notifications don't arrive, falls back to
  periodic GATT reads

### Supported Keys

| Key Category | Keys | VT100 Output |
|---|---|---|
| Printable | a-z, A-Z, 0-9, symbols | ASCII characters |
| Enter | Enter / Return | `\r` (0x0D) |
| Backspace | Backspace | `\b` (0x08) |
| Tab | Tab | `\t` (0x09) |
| Escape | Esc | `ESC` (0x1B) |
| Delete | Delete | `ESC[3~` |
| Cursor keys | Up / Down / Left / Right | `ESC[A` / `ESC[B` / `ESC[D` / `ESC[C` |
| Home / End | Home, End | `ESC[H` / `ESC[F` |
| Page Up/Down | PgUp, PgDn | `ESC[5~` / `ESC[6~` |
| Insert | Insert | `ESC[2~` |
| Function keys | F1-F12 | `ESC OP` .. `ESC[24~` |
| Ctrl combos | Ctrl+A .. Ctrl+Z | 0x01 .. 0x1A |
| Ctrl+[ | | ESC (0x1B) |
| Ctrl+\\ | | FS (0x1C) |
| Ctrl+] | | GS (0x1D) |
| Numpad | 0-9, +, -, *, /, Enter | ASCII equivalents |

## Hardware Requirements

- **ESP32-S3** development board with native USB connector
  - The USB port must be connected to the ESP32-S3 USB OTG peripheral
    (GPIO19/GPIO20), not the USB-UART bridge
- **USB-to-serial adapter** plugged into the ESP32-S3 USB port (e.g. CH340-based
  module). The adapter's TX/RX lines connect to the target device's UART.
- **BLE Keyboard** supporting HID over GATT (most modern BLE keyboards)

### Wiring Example

```
ESP32-S3 USB OTG ──── CH340 USB-TTL module ──── Target MCU UART
  (GPIO19/20)              TX → RX (e.g. PA10)
                           RX ← TX (e.g. PA9)
                           GND ── GND
```

## Software Requirements

- [ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/get-started/)
  v5.2 or later
- CMake 3.16+

## Build & Flash

```bash
# Set up ESP-IDF environment
. $IDF_PATH/export.sh

# Build (target is pre-set to esp32s3 via sdkconfig.defaults)
idf.py build

# Flash (use the UART port, not the USB OTG port)
idf.py -p /dev/ttyUSB0 flash

# Monitor logs
idf.py -p /dev/ttyUSB0 monitor

# Build + flash + monitor in one step
idf.py -p /dev/ttyUSB0 flash monitor
```

## Usage

1. Flash the firmware to the ESP32-S3
2. Plug a USB-to-serial adapter (e.g. CH340) into the ESP32-S3 USB OTG port
3. Connect the adapter's TX/RX/GND to the target device's UART
4. Put your BLE keyboard in pairing mode
5. The ESP32-S3 will automatically scan, find, pair, and connect to the keyboard
6. Keystrokes are translated to VT100 and sent over USB Host CDC to the serial
   adapter, which forwards them to the target UART

## Project Structure

```
.
├── CMakeLists.txt          # Top-level ESP-IDF project file
├── sdkconfig.defaults      # Default configuration (BLE, USB Host, partitions)
├── partitions.csv          # Custom partition table (NVS for bonding)
├── main/
│   ├── CMakeLists.txt      # Component build file
│   ├── main.c              # Application entry point and glue logic
│   ├── ble_hid_host.h/.c   # BLE HID Host (NimBLE) - scan, pair & connect
│   ├── usb_cdc_device.h/.c # USB Host CDC-ACM driver + CH340 init
│   └── vt100_keymap.h/.c   # HID keycode to VT100 translation
├── README.md
└── LICENSE
```

## Architecture

The firmware is organized as a pipeline:

1. **BLE HID Host** (`ble_hid_host.c`): Uses the NimBLE stack to scan for BLE
   peripherals advertising the HID service (UUID 0x1812). Connects to the first
   keyboard found, initiates security/pairing (Just Works), discovers HID Report
   and Boot Keyboard Input characteristics, and subscribes to notifications.
   If notifications don't arrive, falls back to polling via GATT reads. Raw HID
   keyboard reports are delivered via callback. Handles automatic reconnection
   after disconnects.

2. **VT100 Keymap** (`vt100_keymap.c`): Translates 8-byte HID boot keyboard
   reports into ASCII characters and VT100/ANSI escape sequences. Handles
   modifier keys (Shift, Ctrl), detects newly pressed keys by comparing with
   the previous report. US keyboard layout.

3. **USB Host CDC** (`usb_cdc_device.c`): Configures the ESP32-S3 as a USB Host
   using the ESP-IDF USB Host Library and the CDC-ACM host driver. Waits for a
   USB-to-serial adapter to be plugged in, opens it, and sends data. Includes
   CH340-specific vendor initialization (baud rate, LCR, modem control) for
   CH340-based adapters. Non-CH340 CDC-ACM devices are also supported.

4. **Main** (`main.c`): Initializes all subsystems and runs a FreeRTOS task
   that reads keyboard reports from a queue, translates them via `vt100_translate`,
   and forwards the output to USB Host CDC.

## Debug Mode

Define `ENABLE_DEBUG_INJECT` in `main.c` to inject test keystrokes without a
BLE keyboard. This is useful to verify the USB CDC link independently. When
enabled, BLE initialization is skipped.

Define `ENABLE_DEBUG_KEY_LOGS` in `main.c` to log hex dumps of translated
keystrokes on the serial console.

## Troubleshooting

- **Keyboard not found**: Make sure the keyboard is in pairing mode and
  supports BLE (not classic Bluetooth only). Check serial log for scan results.
- **No USB device detected**: Verify you're using the USB OTG port (GPIO19/20).
  Check that the USB-to-serial adapter is properly connected and powered.
- **CH340 not working**: The firmware sends CH340 vendor-specific init commands.
  If using a different adapter, it should work as long as it supports standard
  CDC-ACM class or has vendor-specific bulk endpoints.
- **Keys not working**: The firmware uses US keyboard layout. Some keyboards
  may send reports in Report Protocol instead of Boot Protocol; this firmware
  expects standard 8-byte boot keyboard reports.
- **Keyboard disconnects frequently**: BLE connection stability depends on
  distance and interference. The firmware will automatically reconnect.

## License

See [LICENSE](LICENSE) file.
