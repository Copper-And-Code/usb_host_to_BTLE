/*
 * USB Host CDC - USB Host that accepts a CDC ACM device (Virtual COM port)
 *
 * The ESP32-S3 acts as USB Host. When a USB CDC device is plugged in,
 * it is enumerated and opened. Data can then be sent to the device.
 */

#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Initialize the USB Host subsystem and install the CDC-ACM host driver.
 * Starts a background daemon task for USB host events.
 *
 * @return ESP_OK on success
 */
esp_err_t usb_cdc_host_init(void);

/*
 * Send data to the connected USB CDC device.
 *
 * @param data   Pointer to data buffer
 * @param len    Number of bytes to send
 * @return       Number of bytes actually written, or -1 if no device connected
 */
int usb_cdc_send(const uint8_t *data, size_t len);

/*
 * Send a null-terminated string to the connected USB CDC device.
 *
 * @param str    Null-terminated string
 * @return       Number of bytes written, or -1 if no device connected
 */
int usb_cdc_send_str(const char *str);

/*
 * Return true if a USB CDC device is currently connected and open.
 */
bool usb_cdc_connected(void);

#ifdef __cplusplus
}
#endif
