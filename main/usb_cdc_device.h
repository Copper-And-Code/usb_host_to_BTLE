/*
 * USB CDC Device - Virtual COM port via TinyUSB
 *
 * Presents the ESP32-S3 as a USB CDC ACM device (serial port) to the
 * connected USB host. Data written here appears on the host's virtual COM.
 */

#pragma once

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Initialize the USB CDC device.
 * Configures TinyUSB and starts the USB stack in device mode.
 *
 * @return ESP_OK on success
 */
esp_err_t usb_cdc_device_init(void);

/*
 * Send data over the USB CDC virtual COM port.
 *
 * @param data   Pointer to data buffer
 * @param len    Number of bytes to send
 * @return       Number of bytes actually written, or -1 on error
 */
int usb_cdc_send(const uint8_t *data, size_t len);

/*
 * Send a null-terminated string over USB CDC.
 *
 * @param str    Null-terminated string
 * @return       Number of bytes written, or -1 on error
 */
int usb_cdc_send_str(const char *str);

#ifdef __cplusplus
}
#endif
