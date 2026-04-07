/*
 * USB Host to BLE Keyboard Bridge - Main Application
 *
 * Connects to a BLE HID keyboard, translates keystrokes into VT100
 * escape sequences, and sends them over USB CDC (Virtual COM port)
 * to the connected USB host device.
 *
 * Architecture:
 *   BLE Keyboard -> HID Report -> VT100 Translation -> USB CDC -> Host PC
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_err.h"

#include "ble_hid_host.h"
#include "usb_cdc_device.h"
#include "vt100_keymap.h"

/* Uncomment or define in CMakeLists.txt to see key hex dumps on serial log */
#define ENABLE_DEBUG_KEY_LOGS

/* Uncomment to inject test strings on the VCOM without a BLE keyboard.
 * Useful to verify that the USB CDC virtual COM link is working. */
// #define ENABLE_DEBUG_INJECT

static const char *TAG = "main";

/* Queue to pass keyboard reports from BLE callback to processing task */
static QueueHandle_t keyboard_report_queue = NULL;

/* Previous keyboard report for key press edge detection */
static hid_keyboard_report_t prev_report;

/* ------------------------------------------------------------------ */
/* BLE keyboard callback (called from NimBLE context)                  */
/* ------------------------------------------------------------------ */

#ifndef ENABLE_DEBUG_INJECT
static void on_keyboard_report(const hid_keyboard_report_t *report)
{
    /* Send report to processing queue (non-blocking) */
    if (keyboard_report_queue != NULL) {
        hid_keyboard_report_t report_copy;
        memcpy(&report_copy, report, sizeof(report_copy));
        xQueueSend(keyboard_report_queue, &report_copy, 0);
    }
}
#endif /* !ENABLE_DEBUG_INJECT */

/* ------------------------------------------------------------------ */
/* Keyboard processing task                                            */
/* ------------------------------------------------------------------ */

static void keyboard_task(void *arg)
{
    hid_keyboard_report_t report;
    uint8_t vt100_buf[64]; /* Output buffer for VT100 sequences */

    ESP_LOGI(TAG, "Keyboard processing task started");

    memset(&prev_report, 0, sizeof(prev_report));

    while (1) {
        /* Wait for a keyboard report from the BLE callback */
        if (xQueueReceive(keyboard_report_queue, &report,
                          portMAX_DELAY) == pdTRUE) {

            /* Translate HID report to VT100 */
            int len = vt100_translate(&report, &prev_report,
                                      vt100_buf, sizeof(vt100_buf));

            /* Send translated bytes over USB CDC */
            if (len > 0) {
#ifdef ENABLE_DEBUG_KEY_LOGS
                ESP_LOGI(TAG, "Received %d bytes from BLE. Hex dump:", len);
                ESP_LOG_BUFFER_HEX(TAG, vt100_buf, len);
#endif
                
                int sent = usb_cdc_send(vt100_buf, len);
                if (sent < 0) {
#ifdef ENABLE_DEBUG_KEY_LOGS
                    ESP_LOGI(TAG, "USB CDC not connected, "
                             "discarding %d bytes", len);
#else
                    ESP_LOGD(TAG, "USB CDC not connected, "
                             "discarding %d bytes", len);
#endif
                }
            }

            /* Remember this report for next comparison */
            memcpy(&prev_report, &report, sizeof(prev_report));
        }
    }
}

/* ------------------------------------------------------------------ */
/* Debug injection: send test data over USB CDC without BLE keyboard   */
/* ------------------------------------------------------------------ */

#ifdef ENABLE_DEBUG_INJECT

/* Reverse-map an ASCII character to HID keycode + modifier byte.
 * Returns false if the character cannot be mapped. */
static bool ascii_to_hid(char ch, uint8_t *keycode, uint8_t *modifier)
{
    *modifier = 0;

    if (ch >= 'a' && ch <= 'z') {
        *keycode = 0x04 + (ch - 'a');
    } else if (ch >= 'A' && ch <= 'Z') {
        *keycode = 0x04 + (ch - 'A');
        *modifier = HID_MOD_LEFT_SHIFT;
    } else if (ch >= '1' && ch <= '9') {
        *keycode = 0x1E + (ch - '1');
    } else if (ch == '0') {
        *keycode = 0x27;
    } else if (ch == '\r' || ch == '\n') {
        *keycode = 0x28;
    } else if (ch == ' ') {
        *keycode = 0x2C;
    } else if (ch == '-') {
        *keycode = 0x2D;
    } else if (ch == '=') {
        *keycode = 0x2E;
    } else if (ch == '!') {
        *keycode = 0x1E; *modifier = HID_MOD_LEFT_SHIFT;
    } else {
        return false;
    }
    return true;
}

/* Build a HID keyboard report for a single ASCII character */
static void build_hid_report(char ch, hid_keyboard_report_t *report)
{
    memset(report, 0, sizeof(*report));
    uint8_t kc, mod;
    if (ascii_to_hid(ch, &kc, &mod)) {
        report->modifier = mod;
        report->keycode[0] = kc;
    }
}

static void debug_inject_task(void *arg)
{
    static const char *test_msg = "Hello from ESP32-S3!\r\n";
    hid_keyboard_report_t report;
    hid_keyboard_report_t release;

    memset(&release, 0, sizeof(release)); /* all-zeros = key release */

    ESP_LOGI(TAG, "[DEBUG INJECT] Waiting for USB CDC device to connect...");

    /* Wait until a USB CDC device is plugged in and opened */
    while (!usb_cdc_connected()) {
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    ESP_LOGI(TAG, "[DEBUG INJECT] USB CDC device connected, injecting test string");

    /* Give the host terminal a moment to settle */
    vTaskDelay(pdMS_TO_TICKS(1000));

    /* Inject each character as a press + release HID report pair */
    for (const char *p = test_msg; *p != '\0'; p++) {
        build_hid_report(*p, &report);
        xQueueSend(keyboard_report_queue, &report, portMAX_DELAY);

        /* Send key-release so the next key is detected as "newly pressed" */
        xQueueSend(keyboard_report_queue, &release, portMAX_DELAY);

        vTaskDelay(pdMS_TO_TICKS(50)); /* pace between keystrokes */
    }

    ESP_LOGI(TAG, "[DEBUG INJECT] Test string injection complete");
    vTaskDelete(NULL);
}

#endif /* ENABLE_DEBUG_INJECT */

/* ------------------------------------------------------------------ */
/* Application entry point                                             */
/* ------------------------------------------------------------------ */

void app_main(void)
{
    ESP_LOGI(TAG, "=== USB Host to BLE Keyboard Bridge ===");
    ESP_LOGI(TAG, "VT100 terminal emulation over USB CDC");

    /* Create keyboard report queue */
    keyboard_report_queue = xQueueCreate(16, sizeof(hid_keyboard_report_t));
    if (keyboard_report_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create keyboard report queue");
        return;
    }

    /* Initialize USB Host CDC (waits for a USB CDC device to be plugged in) */
    esp_err_t ret = usb_cdc_host_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "USB CDC init failed: %s", esp_err_to_name(ret));
        return;
    }

    /* Create the keyboard processing task */
    xTaskCreate(keyboard_task, "keyboard_task", 4096, NULL, 5, NULL);

#ifdef ENABLE_DEBUG_INJECT
    /* Debug mode: inject test keystrokes without BLE, skip BLE init */
    ESP_LOGI(TAG, "DEBUG INJECT mode enabled - BLE init skipped");
    xTaskCreate(debug_inject_task, "debug_inject", 4096, NULL, 4, NULL);
#else
    /* Initialize BLE HID Host (starts scanning for keyboards) */
    ret = ble_hid_host_init(on_keyboard_report);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BLE HID Host init failed: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "Initialization complete. Scanning for BLE keyboards...");
#endif
}
