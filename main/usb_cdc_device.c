/*
 * USB CDC Device implementation using ESP-IDF's TinyUSB integration.
 *
 * The ESP32-S3 native USB peripheral is configured as a CDC ACM device,
 * appearing as a virtual serial port on the USB host.
 */

#include <string.h>
#include "esp_log.h"
#include "tinyusb.h"
#include "tusb_cdc_acm.h"

#include "usb_cdc_device.h"

static const char *TAG = "usb_cdc";

/* CDC ACM interface index */
#define CDC_ITF_NUM 0

/* Callback when data is received from USB host (not used but required) */
static void cdc_rx_callback(int itf, cdcacm_event_t *event)
{
    /* We don't process incoming data from USB host in this application */
    uint8_t buf[64];
    size_t rx_size = 0;
    tinyusb_cdcacm_read(itf, buf, sizeof(buf), &rx_size);
}

/* Callback for line state changes (DTR/RTS) */
static void cdc_line_state_changed(int itf, cdcacm_event_t *event)
{
    int dtr = event->line_state_changed_data.dtr;
    int rts = event->line_state_changed_data.rts;
    ESP_LOGI(TAG, "Line state changed: DTR=%d, RTS=%d", dtr, rts);
}

esp_err_t usb_cdc_device_init(void)
{
    ESP_LOGI(TAG, "Initializing USB CDC device");

    /* TinyUSB driver configuration */
    const tinyusb_config_t tusb_cfg = {
        .device_descriptor = NULL,  /* Use default from menuconfig */
        .string_descriptor = NULL,  /* Use default */
        .external_phy = false,
        .configuration_descriptor = NULL, /* Use default */
    };

    esp_err_t ret = tinyusb_driver_install(&tusb_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install TinyUSB driver: %s",
                 esp_err_to_name(ret));
        return ret;
    }

    /* CDC ACM configuration */
    tinyusb_config_cdcacm_t acm_cfg = {
        .cdc_port = TINYUSB_CDC_ACM_0,
        .rx_unread_buf_sz = 256,
        .callback_rx = &cdc_rx_callback,
        .callback_rx_wanted_char = NULL,
        .callback_line_state_changed = &cdc_line_state_changed,
        .callback_line_coding_changed = NULL,
    };

    ret = tusb_cdc_acm_init(&acm_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init CDC ACM: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "USB CDC device ready (Virtual COM port)");
    return ESP_OK;
}

int usb_cdc_send(const uint8_t *data, size_t len)
{
    if (data == NULL || len == 0) {
        return 0;
    }

    /* Check if USB is connected and ready */
    if (!tud_connected() || !tud_cdc_connected()) {
        return -1;
    }

    size_t total_written = 0;

    while (total_written < len) {
        size_t chunk = len - total_written;
        size_t written = tinyusb_cdcacm_write_queue(CDC_ITF_NUM,
                                                     data + total_written,
                                                     chunk);
        total_written += written;

        /* Flush after writing */
        tinyusb_cdcacm_write_flush(CDC_ITF_NUM, pdMS_TO_TICKS(50));

        if (written == 0) {
            break; /* Avoid infinite loop if write fails */
        }
    }

    return (int)total_written;
}

int usb_cdc_send_str(const char *str)
{
    if (str == NULL) {
        return 0;
    }
    return usb_cdc_send((const uint8_t *)str, strlen(str));
}
