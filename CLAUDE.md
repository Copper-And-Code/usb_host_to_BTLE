# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ESP32-S3 firmware that bridges a BLE HID keyboard to a USB Host CDC serial port (e.g. CH340 USB-to-serial adapter), translating keystrokes into VT100/ANSI escape sequences. Built with ESP-IDF v5.2, NimBLE BLE stack, and the USB Host CDC-ACM driver.

## Build Commands

```bash
# Set up ESP-IDF environment (required before any idf.py command)
. /opt/github/esp-idf/export.sh

# Build
idf.py build

# Flash (adjust port)
idf.py -p /dev/ttyUSB0 flash

# Monitor serial logs (use UART port, not the USB OTG port)
idf.py -p /dev/ttyUSB0 monitor

# Build + flash + monitor in one step
idf.py -p /dev/ttyUSB0 flash monitor

# Full clean rebuild (also delete sdkconfig to regenerate from defaults)
rm -f sdkconfig && idf.py fullclean && idf.py build
```

Target is already set to `esp32s3` via `sdkconfig.defaults`. No need to run `idf.py set-target` unless starting from scratch.

## Architecture

Data pipeline: **BLE Keyboard → HID Report → VT100 Translation → USB Host CDC → CH340 → Target UART**

Four source modules in `main/`:

- **`ble_hid_host.c`** — NimBLE-based BLE central. Scans for HID service (UUID 0x1812), connects to the first keyboard found, initiates security/pairing (Just Works, NoIO), discovers GATT characteristics (Report 0x2A4D and Boot KB Input 0x2A22), subscribes to notifications. Falls back to polling via GATT reads if notifications don't arrive. Store callbacks (`ble_store_config_read/write/delete`) must be registered for SM to work. GATT discovery and subscription are serialized to avoid BLE_HS_EBUSY errors.
- **`vt100_keymap.c`** — Stateless translator from HID boot protocol reports to VT100/ANSI byte sequences. Compares current vs previous report to detect newly pressed keys. US keyboard layout only.
- **`usb_cdc_device.c`** — USB Host CDC-ACM driver. The ESP32-S3 acts as USB Host, waits for a CDC device (e.g. CH340) to be plugged in, opens it (vendor-specific first, then standard CDC), and sends data. Includes CH340-specific vendor initialization (baud rate divisor, LCR 8N1, modem control).
- **`main.c`** — Glue: creates a FreeRTOS queue, wires the BLE callback to a processing task that calls vt100_translate and forwards output to USB Host CDC. `ENABLE_DEBUG_INJECT` macro allows testing USB CDC without BLE. `ENABLE_DEBUG_KEY_LOGS` logs hex dumps of translated keystrokes.

## Key Configuration

- `sdkconfig.defaults` — NimBLE roles (central + observer only), USB Host, custom partition table, 16MB flash, timer task stack 4096
- `partitions.csv` — NVS (for BLE bonding), phy_init, factory app (2MB)
- `main/idf_component.yml` — managed component dependencies

## Dependencies

Managed via ESP-IDF component manager (`managed_components/`):
- `espressif/usb_host_cdc_acm` v2.x — USB Host CDC-ACM class driver
- NimBLE — included in ESP-IDF's `bt` component

## Hardware Notes

- Must use the ESP32-S3 USB OTG port (GPIO19/GPIO20), not the USB-UART bridge
- A USB-to-serial adapter (e.g. CH340) plugs into the USB OTG port
- Firmware expects standard 8-byte HID boot keyboard reports
- Only BLE keyboards supported (not classic Bluetooth)

## Important Implementation Details

- NimBLE SM (Security Manager) requires `ble_hs_cfg.store_read_cb`, `store_write_cb`, `store_delete_cb` to be set. Without these callbacks, all security operations fail with BLE_HS_ENOTSUP.
- NimBLE GATT calls should not be made from FreeRTOS timer callbacks (stack overflow risk). Use a deferred task instead.
- After `sdkconfig.defaults` changes, delete `sdkconfig` before rebuilding to force regeneration.
