/*
 * USB Host CDC implementation using ESP-IDF USB Host Library + CDC-ACM driver.
 *
 * The ESP32-S3 acts as USB Host. It waits for any CDC-ACM device to be
 * plugged in, opens it, and allows sending data to it.
 *
 * Architecture:
 *   usb_lib_task  - handles low-level USB host library events
 *   cdc_open_task - attempts to open newly connected CDC devices
 *   CDC-ACM driver runs its own internal task for class-level events
 */

#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "usb/usb_host.h"
#include "usb/cdc_acm_host.h"

#include "usb_cdc_device.h"

static const char *TAG = "usb_host_cdc";

/* ------------------------------------------------------------------ */
/* CH340 vendor-specific initialization                                */
/* ------------------------------------------------------------------ */

/* CH340 VID/PID */
#define CH340_VID  0x1A86
#define CH340_PID  0x7523

/* CH340 vendor requests */
#define CH340_REQ_READ_VERSION  0x5F
#define CH340_REQ_SERIAL_INIT   0xA1
#define CH340_REQ_WRITE_REG     0x9A
#define CH340_REQ_MODEM_CTRL    0xA4

/* CH340 register addresses */
#define CH340_REG_DIVISOR       0x1312
#define CH340_REG_LCR           0x2518

/* CH340 LCR bits */
#define CH340_LCR_ENABLE_RX     0x80
#define CH340_LCR_ENABLE_TX     0x40
#define CH340_LCR_CS8           0x03

/* Baud rate divisor table for CH340 (12 MHz reference clock).
 * Format: { baud_rate, divisor_value (wIndex for REG_DIVISOR) } */
typedef struct {
    uint32_t baud;
    uint16_t divisor;
} ch340_baud_entry_t;

static const ch340_baud_entry_t ch340_baud_table[] = {
    {   2400, 0xD901 },
    {   4800, 0x6402 },
    {   9600, 0xB202 },
    {  19200, 0xD902 },
    {  38400, 0x6403 },
    {  57600, 0x9803 },
    { 115200, 0xCC03 },
    { 230400, 0xE603 },
    { 460800, 0xF303 },
    { 921600, 0xF387 },  /* 0x87 in high byte = prescaler /2 */
    { 0, 0 }
};

static uint16_t ch340_get_divisor(uint32_t baud_rate)
{
    for (int i = 0; ch340_baud_table[i].baud != 0; i++) {
        if (ch340_baud_table[i].baud == baud_rate) {
            return ch340_baud_table[i].divisor;
        }
    }
    return 0xCC03; /* default 115200 */
}

/* Send a vendor OUT control request to CH340 */
static esp_err_t ch340_vendor_write(cdc_acm_dev_hdl_t dev,
                                    uint8_t request,
                                    uint16_t value, uint16_t index)
{
    return cdc_acm_host_send_custom_request(dev,
        0x40,       /* bmRequestType: vendor, host-to-device */
        request,
        value,
        index,
        0, NULL);
}

/* Send a vendor IN control request to CH340 */
static esp_err_t ch340_vendor_read(cdc_acm_dev_hdl_t dev,
                                   uint8_t request,
                                   uint16_t value, uint16_t index,
                                   uint8_t *buf, uint16_t len)
{
    return cdc_acm_host_send_custom_request(dev,
        0xC0,       /* bmRequestType: vendor, device-to-host */
        request,
        value,
        index,
        len, buf);
}

/* Initialize CH340 and set baud rate */
static esp_err_t ch340_init(cdc_acm_dev_hdl_t dev, uint32_t baud_rate)
{
    esp_err_t ret;
    uint8_t ver_buf[2] = {0};

    /* Read chip version */
    ret = ch340_vendor_read(dev, CH340_REQ_READ_VERSION, 0, 0, ver_buf, 2);
    ESP_LOGI(TAG, "CH340 version: 0x%02X 0x%02X (rc=%s)",
             ver_buf[0], ver_buf[1], esp_err_to_name(ret));

    /* Serial init */
    ret = ch340_vendor_write(dev, CH340_REQ_SERIAL_INIT, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "CH340 serial init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Set baud rate divisor */
    uint16_t divisor = ch340_get_divisor(baud_rate);
    ret = ch340_vendor_write(dev, CH340_REQ_WRITE_REG,
                             CH340_REG_DIVISOR, divisor);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "CH340 set baud rate failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Set LCR: 8N1, enable RX and TX */
    uint16_t lcr = CH340_LCR_ENABLE_RX | CH340_LCR_ENABLE_TX | CH340_LCR_CS8;
    ret = ch340_vendor_write(dev, CH340_REQ_WRITE_REG,
                             CH340_REG_LCR, lcr);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "CH340 set LCR failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Set modem control: DTR=1, RTS=1 */
    ret = ch340_vendor_write(dev, CH340_REQ_MODEM_CTRL, ~0x0000, 0);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "CH340 modem ctrl failed: %s (non-fatal)", esp_err_to_name(ret));
    }

    ESP_LOGI(TAG, "CH340 initialized: %lu baud, 8N1", (unsigned long)baud_rate);
    return ESP_OK;
}

/* ------------------------------------------------------------------ */

/* Handle to the currently open CDC device (NULL when none) */
static cdc_acm_dev_hdl_t cdc_dev = NULL;

/* Protects cdc_dev access from multiple tasks */
static SemaphoreHandle_t cdc_mutex = NULL;

/* Signaled when device disconnects, so the open-loop can retry */
static SemaphoreHandle_t device_disconnected_sem = NULL;

/* Flag for connection state */
static volatile bool is_connected = false;

/* ------------------------------------------------------------------ */
/* CDC-ACM callbacks                                                   */
/* ------------------------------------------------------------------ */

/* Data received from the CDC device (host ← device) */
static bool cdc_rx_callback(const uint8_t *data, size_t data_len, void *arg)
{
    ESP_LOGI(TAG, "RX from CDC device (%d bytes):", (int)data_len);
    ESP_LOG_BUFFER_HEXDUMP(TAG, data, data_len, ESP_LOG_INFO);
    return true;
}

/* Device events (disconnect, error, serial state) */
static void cdc_event_callback(const cdc_acm_host_dev_event_data_t *event,
                                void *user_ctx)
{
    switch (event->type) {
    case CDC_ACM_HOST_ERROR:
        ESP_LOGE(TAG, "CDC-ACM error: %d", event->data.error);
        break;

    case CDC_ACM_HOST_DEVICE_DISCONNECTED:
        ESP_LOGW(TAG, "CDC device disconnected");
        xSemaphoreTake(cdc_mutex, portMAX_DELAY);
        cdc_acm_host_close(event->data.cdc_hdl);
        cdc_dev = NULL;
        is_connected = false;
        xSemaphoreGive(cdc_mutex);
        xSemaphoreGive(device_disconnected_sem);
        break;

    case CDC_ACM_HOST_SERIAL_STATE:
        ESP_LOGI(TAG, "Serial state: 0x%04X", event->data.serial_state.val);
        break;

    default:
        ESP_LOGD(TAG, "Unhandled CDC event: %d", event->type);
        break;
    }
}

/* ------------------------------------------------------------------ */
/* USB Host library daemon task                                        */
/* ------------------------------------------------------------------ */

static void usb_lib_task(void *arg)
{
    ESP_LOGI(TAG, "USB Host library task started");
    while (1) {
        uint32_t event_flags;
        usb_host_lib_handle_events(portMAX_DELAY, &event_flags);

        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            usb_host_device_free_all();
        }
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE) {
            ESP_LOGI(TAG, "All USB devices freed");
        }
    }
}

/* ------------------------------------------------------------------ */
/* CDC device open loop task                                           */
/* ------------------------------------------------------------------ */

static void cdc_open_task(void *arg)
{
    const cdc_acm_host_device_config_t dev_config = {
        .connection_timeout_ms = 5000,
        .out_buffer_size = 512,
        .in_buffer_size = 512,
        .user_arg = NULL,
        .event_cb = cdc_event_callback,
        .data_cb = cdc_rx_callback,
    };

    while (1) {
        ESP_LOGI(TAG, "Waiting for USB CDC device to be plugged in...");

        /* Open as vendor-specific directly: most USB-serial adapters
         * (CH340, CP210x, FTDI) and MicroPython boards use vendor-specific
         * class (0xFF), not standard CDC (0x02). */
        cdc_acm_dev_hdl_t dev = NULL;
        esp_err_t err = cdc_acm_host_open_vendor_specific(0, 0, 0, &dev_config, &dev);

        if (err != ESP_OK || dev == NULL) {
            /* Fall back to standard CDC-ACM open */
            err = cdc_acm_host_open(0, 0, 0, &dev_config, &dev);
        }

        if (err != ESP_OK || dev == NULL) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        ESP_LOGI(TAG, "CDC device opened successfully (err=%s)", esp_err_to_name(err));
        cdc_acm_host_desc_print(dev);

        /* If this is a CH340, initialize it with the correct baud rate */
        ch340_init(dev, 115200);

        xSemaphoreTake(cdc_mutex, portMAX_DELAY);
        cdc_dev = dev;
        is_connected = true;
        xSemaphoreGive(cdc_mutex);

        /* Wait until device disconnects, then loop back to retry */
        xSemaphoreTake(device_disconnected_sem, portMAX_DELAY);
        ESP_LOGI(TAG, "Device gone, will scan for a new one...");
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */

esp_err_t usb_cdc_host_init(void)
{
    ESP_LOGI(TAG, "Initializing USB Host for CDC-ACM devices");

    cdc_mutex = xSemaphoreCreateMutex();
    if (cdc_mutex == NULL) {
        return ESP_ERR_NO_MEM;
    }

    device_disconnected_sem = xSemaphoreCreateBinary();
    if (device_disconnected_sem == NULL) {
        return ESP_ERR_NO_MEM;
    }

    /* Install USB Host Library */
    const usb_host_config_t host_config = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
    };
    esp_err_t ret = usb_host_install(&host_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "USB Host install failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Start the USB Host library daemon task */
    xTaskCreate(usb_lib_task, "usb_lib", 4096, NULL, 20, NULL);

    /* Install CDC-ACM host driver */
    ret = cdc_acm_host_install(NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "CDC-ACM host install failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Start the task that opens CDC devices as they appear */
    xTaskCreate(cdc_open_task, "cdc_open", 4096, NULL, 10, NULL);

    ESP_LOGI(TAG, "USB Host CDC-ACM ready, waiting for device...");
    return ESP_OK;
}

int usb_cdc_send(const uint8_t *data, size_t len)
{
    if (data == NULL || len == 0) {
        return 0;
    }

    xSemaphoreTake(cdc_mutex, portMAX_DELAY);
    cdc_acm_dev_hdl_t dev = cdc_dev;
    xSemaphoreGive(cdc_mutex);

    if (dev == NULL) {
        return -1;
    }

    ESP_LOGI(TAG, "TX %d bytes:", (int)len);
    ESP_LOG_BUFFER_HEX(TAG, data, len);

    esp_err_t err = cdc_acm_host_data_tx_blocking(dev, data, len, 1000);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "TX failed: %s", esp_err_to_name(err));
        return -1;
    }

    ESP_LOGI(TAG, "TX ok");
    return (int)len;
}

int usb_cdc_send_str(const char *str)
{
    if (str == NULL) {
        return 0;
    }
    return usb_cdc_send((const uint8_t *)str, strlen(str));
}

bool usb_cdc_connected(void)
{
    return is_connected;
}
