/*
 * BLE HID Host - Bluetooth Low Energy keyboard scanner and connection handler
 *
 * Scans for BLE HID keyboards, connects to the first one found,
 * and delivers HID keyboard reports via a callback.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Standard HID keyboard report (Boot protocol, 8 bytes) */
typedef struct {
    uint8_t modifier;    /* Modifier keys bitmask (Ctrl, Shift, Alt, GUI) */
    uint8_t reserved;    /* Reserved byte (always 0x00) */
    uint8_t keycode[6];  /* Up to 6 simultaneous key codes */
} __attribute__((packed)) hid_keyboard_report_t;

/* Modifier key bitmask definitions */
#define HID_MOD_LEFT_CTRL   (1 << 0)
#define HID_MOD_LEFT_SHIFT  (1 << 1)
#define HID_MOD_LEFT_ALT    (1 << 2)
#define HID_MOD_LEFT_GUI    (1 << 3)
#define HID_MOD_RIGHT_CTRL  (1 << 4)
#define HID_MOD_RIGHT_SHIFT (1 << 5)
#define HID_MOD_RIGHT_ALT   (1 << 6)
#define HID_MOD_RIGHT_GUI   (1 << 7)

/*
 * Callback invoked when a keyboard report is received from BLE keyboard.
 * Called from the NimBLE task context.
 */
typedef void (*ble_hid_keyboard_cb_t)(const hid_keyboard_report_t *report);

/*
 * Initialize the BLE HID Host subsystem.
 * Starts NimBLE, begins scanning for HID peripherals, and
 * auto-connects to the first keyboard found.
 *
 * @param cb  Callback for incoming keyboard reports
 * @return    ESP_OK on success
 */
esp_err_t ble_hid_host_init(ble_hid_keyboard_cb_t cb);

/*
 * Return true if a BLE keyboard is currently connected.
 */
bool ble_hid_host_connected(void);

#ifdef __cplusplus
}
#endif
