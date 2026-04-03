/*
 * BLE HID Host implementation using NimBLE stack.
 *
 * Flow: scan -> discover HID service -> connect -> discover all
 * characteristics -> sequentially subscribe to HID Report notifications
 * -> deliver reports via callback.
 *
 * NimBLE only supports a limited number of concurrent GATT procedures,
 * so all discovery and subscription steps are serialized.
 */

#include <string.h>
#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/util/util.h"

#include "ble_hid_host.h"

static const char *TAG = "ble_hid";

/* BLE HID Service UUID: 0x1812 */
static const ble_uuid16_t hid_svc_uuid = BLE_UUID16_INIT(0x1812);

/* HID Report characteristic UUID: 0x2A4D */
static const ble_uuid16_t hid_report_uuid = BLE_UUID16_INIT(0x2A4D);

/* Client Characteristic Configuration Descriptor UUID: 0x2902 */
static const ble_uuid16_t cccd_uuid = BLE_UUID16_INIT(0x2902);

/* Connection handle for the connected keyboard */
static uint16_t conn_handle = BLE_HS_CONN_HANDLE_NONE;

/* Flag: are we currently connected? */
static volatile bool is_connected = false;

/* User callback for keyboard reports */
static ble_hid_keyboard_cb_t keyboard_callback = NULL;

/* ------------------------------------------------------------------ */
/* Stored characteristics for sequential subscription                   */
/* ------------------------------------------------------------------ */

#define MAX_HID_REPORT_CHARS 8

/* Info about a discovered HID Report characteristic */
typedef struct {
    uint16_t val_handle;  /* Characteristic value handle */
    uint16_t end_handle;  /* End of characteristic handle range */
    uint8_t  properties;  /* Characteristic properties */
} hid_report_chr_t;

/* HID service handle range */
static uint16_t hid_svc_start = 0;
static uint16_t hid_svc_end = 0;

/* Discovered report characteristics */
static hid_report_chr_t report_chars[MAX_HID_REPORT_CHARS];
static int num_report_chars = 0;

/* Index of the characteristic currently being subscribed */
static int subscribe_index = 0;

/* Forward declarations */
static void ble_hid_scan_start(void);
static int ble_hid_gap_event(struct ble_gap_event *event, void *arg);
static void subscribe_next_char(void);

/* ------------------------------------------------------------------ */
/* Sequential subscription: discover descriptors then write CCCD       */
/* ------------------------------------------------------------------ */

/*
 * Callback after CCCD write completes. Move to next characteristic.
 */
static int on_cccd_write_complete(uint16_t chn_conn_handle,
                                  const struct ble_gatt_error *error,
                                  struct ble_gatt_attr *attr,
                                  void *arg)
{
    if (error->status == 0) {
        ESP_LOGI(TAG, "Notifications enabled for char index %d", subscribe_index);
    } else {
        ESP_LOGW(TAG, "CCCD write failed for char index %d: status=%d",
                 subscribe_index, error->status);
    }

    /* Move to next characteristic */
    subscribe_index++;
    subscribe_next_char();
    return 0;
}

/*
 * Descriptor discovery callback for one characteristic at a time.
 * Find the CCCD and write to enable notifications, then proceed to next.
 */
static int on_dsc_discovery(uint16_t chn_conn_handle,
                            const struct ble_gatt_error *error,
                            uint16_t chr_val_handle,
                            const struct ble_gatt_dsc *dsc,
                            void *arg)
{
    if (error->status == BLE_HS_EDONE) {
        /* No CCCD found for this characteristic, move to next */
        ESP_LOGD(TAG, "No CCCD found for char index %d, skipping",
                 subscribe_index);
        subscribe_index++;
        subscribe_next_char();
        return 0;
    }

    if (error->status != 0) {
        ESP_LOGE(TAG, "Descriptor discovery error: %d", error->status);
        subscribe_index++;
        subscribe_next_char();
        return 0;
    }

    /* Check if this is the CCCD */
    if (ble_uuid_cmp(&dsc->uuid.u, &cccd_uuid.u) == 0) {
        ESP_LOGI(TAG, "Found CCCD at handle 0x%04x for char index %d",
                 dsc->handle, subscribe_index);
        uint8_t notify_enable[] = {0x01, 0x00}; /* Enable notifications */
        int rc = ble_gattc_write_flat(chn_conn_handle, dsc->handle,
                                      notify_enable, sizeof(notify_enable),
                                      on_cccd_write_complete, NULL);
        if (rc != 0) {
            ESP_LOGE(TAG, "Failed to write CCCD: %d", rc);
            subscribe_index++;
            subscribe_next_char();
        }
        /* Return non-zero to stop descriptor discovery for this char */
        return BLE_HS_EDONE;
    }

    return 0;
}

/*
 * Subscribe to the next HID Report characteristic in the queue.
 * Called after each subscription completes (or is skipped).
 */
static void subscribe_next_char(void)
{
    while (subscribe_index < num_report_chars) {
        hid_report_chr_t *chr = &report_chars[subscribe_index];

        /* Only subscribe to characteristics that support notifications */
        if (!(chr->properties & BLE_GATT_CHR_PROP_NOTIFY)) {
            ESP_LOGD(TAG, "Char index %d (handle=0x%04x) has no NOTIFY, skipping",
                     subscribe_index, chr->val_handle);
            subscribe_index++;
            continue;
        }

        ESP_LOGI(TAG, "Subscribing to char index %d (handle=0x%04x)",
                 subscribe_index, chr->val_handle);

        /* Discover descriptors for this characteristic only */
        int rc = ble_gattc_disc_all_dscs(conn_handle,
                                         chr->val_handle,
                                         chr->end_handle,
                                         on_dsc_discovery, NULL);
        if (rc != 0) {
            ESP_LOGE(TAG, "Failed to start descriptor discovery: %d", rc);
            subscribe_index++;
            continue; /* Try next */
        }
        return; /* Wait for callback */
    }

    ESP_LOGI(TAG, "All HID Report subscriptions complete (%d chars processed)",
             num_report_chars);
}

/* ------------------------------------------------------------------ */
/* GATT discovery (collect characteristics first, subscribe later)      */
/* ------------------------------------------------------------------ */

/*
 * Characteristic discovery callback - collect all HID Report
 * characteristics, then start sequential subscription.
 */
static int on_chr_discovery(uint16_t chn_conn_handle,
                            const struct ble_gatt_error *error,
                            const struct ble_gatt_chr *chr,
                            void *arg)
{
    if (error->status == BLE_HS_EDONE) {
        ESP_LOGI(TAG, "Characteristic discovery complete, found %d report chars",
                 num_report_chars);

        /* Now subscribe to each one sequentially */
        subscribe_index = 0;

        /* Compute end_handle for each characteristic:
         * end_handle[i] = val_handle[i+1] - 1 (or svc_end for the last) */
        for (int i = 0; i < num_report_chars; i++) {
            if (i + 1 < num_report_chars) {
                report_chars[i].end_handle = report_chars[i + 1].val_handle - 1;
            } else {
                report_chars[i].end_handle = hid_svc_end;
            }
        }

        subscribe_next_char();
        return 0;
    }

    if (error->status != 0) {
        ESP_LOGE(TAG, "Characteristic discovery error: %d", error->status);
        return 0;
    }

    /* Collect HID Report characteristics */
    if (ble_uuid_cmp(&chr->uuid.u, &hid_report_uuid.u) == 0) {
        ESP_LOGI(TAG, "Found HID Report char, val_handle=0x%04x, props=0x%02x",
                 chr->val_handle, chr->properties);

        if (num_report_chars < MAX_HID_REPORT_CHARS) {
            report_chars[num_report_chars].val_handle = chr->val_handle;
            report_chars[num_report_chars].properties = chr->properties;
            report_chars[num_report_chars].end_handle = hid_svc_end;
            num_report_chars++;
        } else {
            ESP_LOGW(TAG, "Too many report chars, ignoring handle=0x%04x",
                     chr->val_handle);
        }
    }
    return 0;
}

/*
 * Service discovery callback - when HID service is found, discover its
 * characteristics.
 */
static int on_svc_discovery(uint16_t chn_conn_handle,
                            const struct ble_gatt_error *error,
                            const struct ble_gatt_svc *svc,
                            void *arg)
{
    if (error->status == BLE_HS_EDONE) {
        return 0;
    }

    if (error->status != 0) {
        ESP_LOGE(TAG, "Service discovery error: %d", error->status);
        return 0;
    }

    ESP_LOGI(TAG, "Found HID service: start=0x%04x end=0x%04x",
             svc->start_handle, svc->end_handle);

    /* Store service range for end_handle calculation */
    hid_svc_start = svc->start_handle;
    hid_svc_end = svc->end_handle;
    num_report_chars = 0;

    /* Discover all characteristics within the HID service */
    int rc = ble_gattc_disc_all_chrs(chn_conn_handle,
                                     svc->start_handle,
                                     svc->end_handle,
                                     on_chr_discovery, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to start characteristic discovery: %d", rc);
    }
    return 0;
}

/* ------------------------------------------------------------------ */
/* GAP event handler                                                   */
/* ------------------------------------------------------------------ */

static int ble_hid_gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {

    case BLE_GAP_EVENT_DISC: {
        /* Check if the discovered device advertises HID service */
        struct ble_hs_adv_fields fields;
        int rc = ble_hs_adv_parse_fields(&fields, event->disc.data,
                                         event->disc.length_data);
        if (rc != 0) {
            break;
        }

        bool has_hid = false;

        /* Check 16-bit UUIDs in advertisement */
        for (int i = 0; i < fields.num_uuids16; i++) {
            if (ble_uuid_u16(&fields.uuids16[i].u) == 0x1812) {
                has_hid = true;
                break;
            }
        }

        /* Also check appearance field (0x03C1 = keyboard) */
        if (!has_hid && fields.appearance_is_present &&
            fields.appearance == 0x03C1) {
            has_hid = true;
        }

        if (has_hid) {
            char addr_str[18];
            snprintf(addr_str, sizeof(addr_str),
                     "%02x:%02x:%02x:%02x:%02x:%02x",
                     event->disc.addr.val[5], event->disc.addr.val[4],
                     event->disc.addr.val[3], event->disc.addr.val[2],
                     event->disc.addr.val[1], event->disc.addr.val[0]);

            if (fields.name_len > 0) {
                ESP_LOGI(TAG, "Found HID keyboard: %.*s [%s]",
                         fields.name_len, fields.name, addr_str);
            } else {
                ESP_LOGI(TAG, "Found HID keyboard: [%s]", addr_str);
            }

            /* Stop scanning and connect */
            ble_gap_disc_cancel();

            struct ble_gap_conn_params conn_params = {
                .scan_itvl = 0x0010,
                .scan_window = 0x0010,
                .itvl_min = 6,       /* 7.5 ms (fast for keyboard) */
                .itvl_max = 24,      /* 30 ms */
                .latency = 0,
                .supervision_timeout = 200, /* 2 seconds */
                .min_ce_len = 0,
                .max_ce_len = 0,
            };

            rc = ble_gap_connect(BLE_OWN_ADDR_PUBLIC, &event->disc.addr,
                                 30000, &conn_params,
                                 ble_hid_gap_event, NULL);
            if (rc != 0) {
                ESP_LOGE(TAG, "Failed to connect: %d", rc);
                ble_hid_scan_start();
            }
        }
        break;
    }

    case BLE_GAP_EVENT_CONNECT: {
        if (event->connect.status == 0) {
            conn_handle = event->connect.conn_handle;
            is_connected = true;
            ESP_LOGI(TAG, "Connected (handle=%d)", conn_handle);

            /* Request security / bonding for HID */
            ble_gap_security_initiate(conn_handle);

            /* Discover HID service */
            int rc = ble_gattc_disc_svc_by_uuid(conn_handle,
                                                 &hid_svc_uuid.u,
                                                 on_svc_discovery, NULL);
            if (rc != 0) {
                ESP_LOGE(TAG, "Failed to start service discovery: %d", rc);
            }
        } else {
            ESP_LOGE(TAG, "Connection failed: status=%d",
                     event->connect.status);
            conn_handle = BLE_HS_CONN_HANDLE_NONE;
            is_connected = false;
            ble_hid_scan_start();
        }
        break;
    }

    case BLE_GAP_EVENT_DISCONNECT: {
        ESP_LOGW(TAG, "Disconnected (reason=%d)",
                 event->disconnect.reason);
        conn_handle = BLE_HS_CONN_HANDLE_NONE;
        is_connected = false;
        num_report_chars = 0;
        /* Restart scanning to reconnect */
        ble_hid_scan_start();
        break;
    }

    case BLE_GAP_EVENT_NOTIFY_RX: {
        /* Incoming notification - this is our keyboard report data */
        if (keyboard_callback && event->notify_rx.om) {
            uint8_t buf[16];
            uint16_t len = OS_MBUF_PKTLEN(event->notify_rx.om);
            if (len > sizeof(buf)) {
                len = sizeof(buf);
            }
            os_mbuf_copydata(event->notify_rx.om, 0, len, buf);

            /* Standard boot keyboard report is 8 bytes */
            if (len >= sizeof(hid_keyboard_report_t)) {
                hid_keyboard_report_t *report = (hid_keyboard_report_t *)buf;
                keyboard_callback(report);
            }
        }
        break;
    }

    case BLE_GAP_EVENT_ENC_CHANGE: {
        if (event->enc_change.status == 0) {
            ESP_LOGI(TAG, "Encryption enabled");
        } else {
            ESP_LOGW(TAG, "Encryption change failed: %d",
                     event->enc_change.status);
        }
        break;
    }

    case BLE_GAP_EVENT_REPEAT_PAIRING: {
        /* Delete old bond and allow re-pairing */
        struct ble_gap_conn_desc desc;
        ble_gap_conn_find(event->repeat_pairing.conn_handle, &desc);
        ble_store_util_delete_peer(&desc.peer_id_addr);
        return BLE_GAP_REPEAT_PAIRING_RETRY;
    }

    default:
        break;
    }

    return 0;
}

/* ------------------------------------------------------------------ */
/* Scanning                                                            */
/* ------------------------------------------------------------------ */

static void ble_hid_scan_start(void)
{
    struct ble_gap_disc_params scan_params = {
        .itvl = 0x0050,            /* 50 ms scan interval */
        .window = 0x0030,          /* 30 ms scan window */
        .filter_policy = 0,
        .limited = 0,
        .passive = 0,              /* Active scan to get scan responses */
        .filter_duplicates = 1,
    };

    ESP_LOGI(TAG, "Starting BLE scan for HID keyboards...");

    int rc = ble_gap_disc(BLE_OWN_ADDR_PUBLIC, BLE_HS_FOREVER,
                          &scan_params, ble_hid_gap_event, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to start scanning: %d", rc);
    }
}

/* ------------------------------------------------------------------ */
/* NimBLE host task and sync callback                                  */
/* ------------------------------------------------------------------ */

static void ble_hid_on_sync(void)
{
    /* Use default public address */
    int rc = ble_hs_util_ensure_addr(0);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to ensure address: %d", rc);
        return;
    }

    /* Start scanning */
    ble_hid_scan_start();
}

static void ble_hid_on_reset(int reason)
{
    ESP_LOGW(TAG, "NimBLE host reset, reason=%d", reason);
}

static void nimble_host_task(void *param)
{
    ESP_LOGI(TAG, "NimBLE host task started");
    nimble_port_run(); /* This blocks until nimble_port_stop() */
    nimble_port_freertos_deinit();
}

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */

esp_err_t ble_hid_host_init(ble_hid_keyboard_cb_t cb)
{
    if (cb == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    keyboard_callback = cb;

    /* Initialize NVS (needed for BLE bonding) */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* Initialize NimBLE */
    ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init NimBLE port: %d", ret);
        return ret;
    }

    /* Configure the NimBLE host */
    ble_hs_cfg.reset_cb = ble_hid_on_reset;
    ble_hs_cfg.sync_cb = ble_hid_on_sync;

    /* Security: allow bonding, MITM not required, SC if available */
    ble_hs_cfg.sm_io_cap = BLE_SM_IO_CAP_NO_IO;
    ble_hs_cfg.sm_bonding = 1;
    ble_hs_cfg.sm_mitm = 0;
    ble_hs_cfg.sm_sc = 1;
    ble_hs_cfg.sm_our_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
    ble_hs_cfg.sm_their_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;

    /* Start the NimBLE host task */
    nimble_port_freertos_init(nimble_host_task);

    ESP_LOGI(TAG, "BLE HID Host initialized");
    return ESP_OK;
}

bool ble_hid_host_connected(void)
{
    return is_connected;
}
