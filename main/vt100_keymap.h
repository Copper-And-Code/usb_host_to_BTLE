/*
 * VT100 Keymap - Translates HID keyboard reports into VT100/ANSI
 * escape sequences suitable for terminal communication.
 *
 * Supports: printable ASCII, cursor keys, Home/End/Insert/Delete,
 * Page Up/Down, function keys F1-F12, Tab, Enter, Backspace, Escape,
 * and Ctrl+key combinations.
 */

#pragma once

#include <stdint.h>
#include <stddef.h>
#include "ble_hid_host.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Maximum length of a VT100 escape sequence (e.g. "\x1b[15~" = 5 bytes) */
#define VT100_SEQ_MAX_LEN  8

/*
 * Translate a HID keyboard report into VT100 byte sequences.
 *
 * Processes all pressed keys in the report, compares with the previous
 * report to detect newly pressed keys, and writes the corresponding
 * ASCII or VT100 escape sequences into the output buffer.
 *
 * @param report       Current keyboard report
 * @param prev_report  Previous keyboard report (for edge detection)
 * @param out_buf      Output buffer for VT100 bytes
 * @param out_buf_size Size of output buffer
 * @return             Number of bytes written to out_buf
 */
int vt100_translate(const hid_keyboard_report_t *report,
                    const hid_keyboard_report_t *prev_report,
                    uint8_t *out_buf, size_t out_buf_size);

#ifdef __cplusplus
}
#endif
