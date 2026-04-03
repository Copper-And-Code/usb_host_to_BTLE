/*
 * VT100 Keymap implementation.
 *
 * Converts USB HID keycodes (Usage Page 0x07) to ASCII / VT100 escape
 * sequences. Uses the standard US keyboard layout.
 *
 * Reference:
 *   - USB HID Usage Tables, Section 10 "Keyboard/Keypad Page (0x07)"
 *   - VT100 / ANSI X3.64 escape sequences
 */

#include <string.h>
#include <stdbool.h>
#include "esp_log.h"
#include "vt100_keymap.h"

static const char *TAG = "vt100";

/* ------------------------------------------------------------------ */
/* HID keycode to ASCII mapping (US layout)                            */
/* Index = HID keycode, value = ASCII character (unshifted / shifted)  */
/* ------------------------------------------------------------------ */

/* HID keycodes 0x04..0x1D = a..z */
/* HID keycodes 0x1E..0x27 = 1..9, 0 */
/* HID keycodes 0x28..0x38 = Enter, Esc, Backspace, Tab, Space, -=[]\ etc. */

/* Unshifted ASCII for HID keycodes 0x00..0x64 */
static const char hid_to_ascii_unshifted[101] = {
    /* 0x00 */ 0, 0, 0, 0,
    /* 0x04 */ 'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j',
    /* 0x0E */ 'k', 'l', 'm', 'n', 'o', 'p', 'q', 'r', 's', 't',
    /* 0x18 */ 'u', 'v', 'w', 'x', 'y', 'z',
    /* 0x1E */ '1', '2', '3', '4', '5', '6', '7', '8', '9', '0',
    /* 0x28 */ '\r',   /* Enter */
    /* 0x29 */ 0x1B,   /* Escape */
    /* 0x2A */ '\b',   /* Backspace */
    /* 0x2B */ '\t',   /* Tab */
    /* 0x2C */ ' ',    /* Space */
    /* 0x2D */ '-',
    /* 0x2E */ '=',
    /* 0x2F */ '[',
    /* 0x30 */ ']',
    /* 0x31 */ '\\',
    /* 0x32 */ '#',    /* Non-US # (hash) */
    /* 0x33 */ ';',
    /* 0x34 */ '\'',
    /* 0x35 */ '`',
    /* 0x36 */ ',',
    /* 0x37 */ '.',
    /* 0x38 */ '/',
    /* 0x39 */ 0,      /* Caps Lock (handled separately) */
    /* 0x3A..0x45: F1-F12 (handled as special keys) */
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    /* 0x46 */ 0,      /* PrintScreen */
    /* 0x47 */ 0,      /* Scroll Lock */
    /* 0x48 */ 0,      /* Pause */
    /* 0x49 */ 0,      /* Insert */
    /* 0x4A */ 0,      /* Home */
    /* 0x4B */ 0,      /* Page Up */
    /* 0x4C */ 0x7F,   /* Delete (DEL) */
    /* 0x4D */ 0,      /* End */
    /* 0x4E */ 0,      /* Page Down */
    /* 0x4F */ 0,      /* Right Arrow */
    /* 0x50 */ 0,      /* Left Arrow */
    /* 0x51 */ 0,      /* Down Arrow */
    /* 0x52 */ 0,      /* Up Arrow */
    /* 0x53 */ 0,      /* Num Lock */
    /* 0x54 */ '/',    /* Keypad / */
    /* 0x55 */ '*',    /* Keypad * */
    /* 0x56 */ '-',    /* Keypad - */
    /* 0x57 */ '+',    /* Keypad + */
    /* 0x58 */ '\r',   /* Keypad Enter */
    /* 0x59 */ '1',    /* Keypad 1 */
    /* 0x5A */ '2',    /* Keypad 2 */
    /* 0x5B */ '3',    /* Keypad 3 */
    /* 0x5C */ '4',    /* Keypad 4 */
    /* 0x5D */ '5',    /* Keypad 5 */
    /* 0x5E */ '6',    /* Keypad 6 */
    /* 0x5F */ '7',    /* Keypad 7 */
    /* 0x60 */ '8',    /* Keypad 8 */
    /* 0x61 */ '9',    /* Keypad 9 */
    /* 0x62 */ '0',    /* Keypad 0 */
    /* 0x63 */ '.',    /* Keypad . */
    /* 0x64 */ '\\',   /* Non-US backslash */
};

/* Shifted ASCII for keycodes 0x1E..0x38 (numbers and symbols row) */
static const char shifted_symbols[] = {
    /* 0x1E */ '!',    /* Shift+1 */
    /* 0x1F */ '@',    /* Shift+2 */
    /* 0x20 */ '#',    /* Shift+3 */
    /* 0x21 */ '$',    /* Shift+4 */
    /* 0x22 */ '%',    /* Shift+5 */
    /* 0x23 */ '^',    /* Shift+6 */
    /* 0x24 */ '&',    /* Shift+7 */
    /* 0x25 */ '*',    /* Shift+8 */
    /* 0x26 */ '(',    /* Shift+9 */
    /* 0x27 */ ')',    /* Shift+0 */
    /* 0x28 */ '\r',   /* Enter (unchanged) */
    /* 0x29 */ 0x1B,   /* Escape (unchanged) */
    /* 0x2A */ '\b',   /* Backspace (unchanged) */
    /* 0x2B */ '\t',   /* Tab (unchanged) */
    /* 0x2C */ ' ',    /* Space (unchanged) */
    /* 0x2D */ '_',    /* Shift+- */
    /* 0x2E */ '+',    /* Shift+= */
    /* 0x2F */ '{',    /* Shift+[ */
    /* 0x30 */ '}',    /* Shift+] */
    /* 0x31 */ '|',    /* Shift+\ */
    /* 0x32 */ '~',    /* Shift+# */
    /* 0x33 */ ':',    /* Shift+; */
    /* 0x34 */ '"',    /* Shift+' */
    /* 0x35 */ '~',    /* Shift+` */
    /* 0x36 */ '<',    /* Shift+, */
    /* 0x37 */ '>',    /* Shift+. */
    /* 0x38 */ '?',    /* Shift+/ */
};

/* ------------------------------------------------------------------ */
/* VT100 escape sequences for special keys                             */
/* ------------------------------------------------------------------ */

typedef struct {
    uint8_t hid_keycode;
    const char *sequence;
} vt100_special_key_t;

static const vt100_special_key_t special_keys[] = {
    /* Cursor keys (VT100 application mode sequences) */
    { 0x4F, "\x1b[C" },   /* Right Arrow  */
    { 0x50, "\x1b[D" },   /* Left Arrow   */
    { 0x51, "\x1b[B" },   /* Down Arrow   */
    { 0x52, "\x1b[A" },   /* Up Arrow     */

    /* Navigation keys */
    { 0x49, "\x1b[2~" },  /* Insert       */
    { 0x4C, "\x1b[3~" },  /* Delete       */
    { 0x4A, "\x1b[H" },   /* Home         */
    { 0x4D, "\x1b[F" },   /* End          */
    { 0x4B, "\x1b[5~" },  /* Page Up      */
    { 0x4E, "\x1b[6~" },  /* Page Down    */

    /* Function keys F1-F12 (VT100 / xterm sequences) */
    { 0x3A, "\x1bOP" },   /* F1           */
    { 0x3B, "\x1bOQ" },   /* F2           */
    { 0x3C, "\x1bOR" },   /* F3           */
    { 0x3D, "\x1bOS" },   /* F4           */
    { 0x3E, "\x1b[15~" }, /* F5           */
    { 0x3F, "\x1b[17~" }, /* F6           */
    { 0x40, "\x1b[18~" }, /* F7           */
    { 0x41, "\x1b[19~" }, /* F8           */
    { 0x42, "\x1b[20~" }, /* F9           */
    { 0x43, "\x1b[21~" }, /* F10          */
    { 0x44, "\x1b[23~" }, /* F11          */
    { 0x45, "\x1b[24~" }, /* F12          */

    { 0, NULL }  /* Sentinel */
};

/* ------------------------------------------------------------------ */
/* Helper functions                                                    */
/* ------------------------------------------------------------------ */

/* Check if a keycode is present in a report */
static bool keycode_in_report(const hid_keyboard_report_t *report,
                              uint8_t keycode)
{
    for (int i = 0; i < 6; i++) {
        if (report->keycode[i] == keycode) {
            return true;
        }
    }
    return false;
}

/* Append bytes to output buffer, respecting bounds */
static int buf_append(uint8_t *buf, size_t buf_size, int pos,
                      const void *data, int len)
{
    if (pos + len > (int)buf_size) {
        return pos; /* Buffer full, don't overflow */
    }
    memcpy(buf + pos, data, len);
    return pos + len;
}

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */

int vt100_translate(const hid_keyboard_report_t *report,
                    const hid_keyboard_report_t *prev_report,
                    uint8_t *out_buf, size_t out_buf_size)
{
    int pos = 0;
    bool shift = (report->modifier & (HID_MOD_LEFT_SHIFT | HID_MOD_RIGHT_SHIFT)) != 0;
    bool ctrl  = (report->modifier & (HID_MOD_LEFT_CTRL | HID_MOD_RIGHT_CTRL)) != 0;

    /* Process each keycode in the current report */
    for (int i = 0; i < 6; i++) {
        uint8_t kc = report->keycode[i];
        if (kc == 0) {
            continue; /* No key */
        }

        /* Only process newly pressed keys (not in previous report) */
        if (prev_report && keycode_in_report(prev_report, kc)) {
            continue;
        }

        /* Check for special keys first (cursor, function keys, etc.) */
        bool is_special = false;
        for (int s = 0; special_keys[s].sequence != NULL; s++) {
            if (special_keys[s].hid_keycode == kc) {
                const char *seq = special_keys[s].sequence;
                pos = buf_append(out_buf, out_buf_size, pos,
                                 seq, strlen(seq));
                is_special = true;
                break;
            }
        }

        if (is_special) {
            continue;
        }

        /* Handle Ctrl+key combinations (produce control characters 0x01..0x1A) */
        if (ctrl && kc >= 0x04 && kc <= 0x1D) {
            /* Ctrl+A = 0x01, Ctrl+B = 0x02, ... Ctrl+Z = 0x1A */
            uint8_t ctrl_char = (kc - 0x04) + 1;
            pos = buf_append(out_buf, out_buf_size, pos, &ctrl_char, 1);
            continue;
        }

        /* Handle Ctrl+[ = ESC (0x1B) */
        if (ctrl && kc == 0x2F) {
            uint8_t esc = 0x1B;
            pos = buf_append(out_buf, out_buf_size, pos, &esc, 1);
            continue;
        }

        /* Handle Ctrl+\ = FS (0x1C) */
        if (ctrl && kc == 0x31) {
            uint8_t fs = 0x1C;
            pos = buf_append(out_buf, out_buf_size, pos, &fs, 1);
            continue;
        }

        /* Handle Ctrl+] = GS (0x1D) */
        if (ctrl && kc == 0x30) {
            uint8_t gs = 0x1D;
            pos = buf_append(out_buf, out_buf_size, pos, &gs, 1);
            continue;
        }

        /* Regular printable key */
        if (kc < sizeof(hid_to_ascii_unshifted)) {
            char ch;

            if (shift && kc >= 0x04 && kc <= 0x1D) {
                /* Shifted letter -> uppercase */
                ch = hid_to_ascii_unshifted[kc] - 32; /* a->A */
            } else if (shift && kc >= 0x1E && kc <= 0x38) {
                /* Shifted number/symbol row */
                ch = shifted_symbols[kc - 0x1E];
            } else {
                ch = hid_to_ascii_unshifted[kc];
            }

            if (ch != 0) {
                pos = buf_append(out_buf, out_buf_size, pos,
                                 (uint8_t *)&ch, 1);
            }
        }
    }

    if (pos > 0) {
        ESP_LOGD(TAG, "Translated %d bytes", pos);
    }

    return pos;
}
