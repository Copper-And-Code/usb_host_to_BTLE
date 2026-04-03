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

static const char *TAG = "main";

/* Queue to pass keyboard reports from BLE callback to processing task */
static QueueHandle_t keyboard_report_queue = NULL;

/* Previous keyboard report for key press edge detection */
static hid_keyboard_report_t prev_report;

/* ------------------------------------------------------------------ */
/* BLE keyboard callback (called from NimBLE context)                  */
/* ------------------------------------------------------------------ */

static void on_keyboard_report(const hid_keyboard_report_t *report)
{
    /* Send report to processing queue (non-blocking) */
    if (keyboard_report_queue != NULL) {
        hid_keyboard_report_t report_copy;
        memcpy(&report_copy, report, sizeof(report_copy));
        xQueueSend(keyboard_report_queue, &report_copy, 0);
    }
}

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
                int sent = usb_cdc_send(vt100_buf, len);
                if (sent < 0) {
                    ESP_LOGD(TAG, "USB CDC not connected, "
                             "discarding %d bytes", len);
                }
            }

            /* Remember this report for next comparison */
            memcpy(&prev_report, &report, sizeof(prev_report));
        }
    }
}

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

    /* Initialize USB CDC device first (so it's ready when keys arrive) */
    esp_err_t ret = usb_cdc_device_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "USB CDC init failed: %s", esp_err_to_name(ret));
        return;
    }

    /* Initialize BLE HID Host (starts scanning for keyboards) */
    ret = ble_hid_host_init(on_keyboard_report);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BLE HID Host init failed: %s", esp_err_to_name(ret));
        return;
    }

    /* Create the keyboard processing task */
    xTaskCreate(keyboard_task, "keyboard_task", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "Initialization complete. Scanning for BLE keyboards...");
}
