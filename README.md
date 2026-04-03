# USB Host to BLE Keyboard Bridge

ESP32-S3 firmware that connects to a Bluetooth Low Energy (BLE) keyboard and
forwards keystrokes to a USB host as VT100 terminal data over a USB CDC
(Virtual COM Port).

```
 +----------------+        BLE        +-------------+       USB CDC       +----------+
 | BLE Keyboard   | ───────────────── | ESP32-S3    | ════════════════════ | Host PC  |
 | (HID device)   |    HID Reports    | (this fw)   |   VT100 serial      | terminal |
 +----------------+                   +-------------+                     +----------+
```

## Features

- **BLE HID Host**: auto-scans and connects to BLE keyboards
- **USB CDC Device**: appears as a virtual serial port (COM) on the host
- **VT100 Terminal Emulation**: translates keystrokes to standard VT100/ANSI
  escape sequences

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

- **ESP32-S3** development board (any board with native USB connector)
  - The USB port must be connected to the ESP32-S3 USB OTG peripheral
    (GPIO19/GPIO20), not the USB-UART bridge
- **BLE Keyboard** supporting HID over GATT (most modern BLE keyboards)

## Software Requirements

- [ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/get-started/)
  v5.1 or later
- CMake 3.16+

## Build & Flash

```bash
# Set up ESP-IDF environment
. $HOME/esp/esp-idf/export.sh

# Set target to ESP32-S3
idf.py set-target esp32s3

# Build
idf.py build

# Flash (adjust port as needed)
idf.py -p /dev/ttyUSB0 flash

# Monitor logs (use the UART port, not the USB CDC port)
idf.py -p /dev/ttyUSB0 monitor
```

## Usage

1. Flash the firmware to the ESP32-S3
2. Connect the ESP32-S3 USB OTG port to the host PC
3. Put your BLE keyboard in pairing mode
4. The ESP32-S3 will automatically scan, find, and connect to the keyboard
5. A new virtual COM port appears on the host (e.g., `/dev/ttyACM0` on Linux,
   `COMx` on Windows)
6. Open a terminal emulator (minicom, PuTTY, screen, etc.) on the virtual
   COM port:

```bash
# Linux example
screen /dev/ttyACM0 115200

# or
minicom -D /dev/ttyACM0
```

7. Type on the BLE keyboard - keystrokes appear in the terminal

## Project Structure

```
.
├── CMakeLists.txt          # Top-level ESP-IDF project file
├── sdkconfig.defaults      # Default configuration (BLE, USB, partitions)
├── partitions.csv          # Custom partition table
├── main/
│   ├── CMakeLists.txt      # Component build file
│   ├── main.c              # Application entry point and glue logic
│   ├── ble_hid_host.h/.c   # BLE HID Host (NimBLE) - scan & connect
│   ├── usb_cdc_device.h/.c # USB CDC Device (TinyUSB) - virtual COM
│   └── vt100_keymap.h/.c   # HID keycode to VT100 translation
├── README.md
└── LICENSE
```

## Architecture

The firmware is organized as a pipeline:

1. **BLE HID Host** (`ble_hid_host.c`): Uses the NimBLE stack to scan for
   BLE peripherals advertising the HID service (UUID 0x1812). Connects to the
   first keyboard found, discovers HID Report characteristics, and subscribes
   to notifications. Raw HID keyboard reports are delivered via callback.

2. **VT100 Keymap** (`vt100_keymap.c`): Translates 8-byte HID boot keyboard
   reports into ASCII characters and VT100/ANSI escape sequences. Handles
   modifier keys (Shift, Ctrl), detects newly pressed keys by comparing with
   the previous report, and supports US keyboard layout.

3. **USB CDC Device** (`usb_cdc_device.c`): Configures the ESP32-S3 native
   USB peripheral as a CDC ACM device using TinyUSB. The translated VT100
   bytes are written to the CDC interface and appear on the host's serial port.

4. **Main** (`main.c`): Initializes all subsystems and runs a FreeRTOS task
   that reads keyboard reports from a queue, translates them, and forwards
   the output to USB CDC.

## Troubleshooting

- **Keyboard not found**: Make sure the keyboard is in pairing mode and
  supports BLE (not classic Bluetooth only). Check ESP32 log output for
  scan results.
- **No COM port on host**: Verify you're using the USB OTG port (not UART).
  Check that the ESP32-S3 board exposes GPIO19/20 on the USB connector.
- **Keys not working**: The firmware uses US keyboard layout. Some keyboards
  may send reports in Report Protocol instead of Boot Protocol; this firmware
  expects standard 8-byte boot keyboard reports.

## License

See [LICENSE](LICENSE) file.
