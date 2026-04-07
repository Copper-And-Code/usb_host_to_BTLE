/*
 * BLE HID Host implementation using NimBLE stack.
 *
 * Flow: scan -> connect -> encrypt/pair -> discover HID service ->
 * discover characteristics -> subscribe to HID Report notifications
 * -> deliver reports via callback.
 *
 * If notifications don't arrive within a timeout, falls back to
 * polling the report characteristic via GATT reads.
 *
 * Based on the working BTstack implementation for Raspberry Pi Pico W.
 * Key insight: always encrypt FIRST, then discover GATT services.
 * NimBLE only supports a limited number of concurrent GATT procedures,
 * so all discovery and subscription steps are serialized.
 */

#include <string.h>
#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/util/util.h"
#include "store/config/ble_store_config.h"

#include "ble_hid_host.h"

static const char *TAG = "ble_hid";

/* BLE HID Service UUID: 0x1812 */
static const ble_uuid16_t hid_svc_uuid = BLE_UUID16_INIT(0x1812);

/* HID Report characteristic UUID: 0x2A4D */
static const ble_uuid16_t hid_report_uuid = BLE_UUID16_INIT(0x2A4D);

/* Boot Keyboard Input Report UUID: 0x2A22 */
static const ble_uuid16_t boot_kb_input_uuid = BLE_UUID16_INIT(0x2A22);

/* Client Characteristic Configuration Descriptor UUID: 0x2902 */
static const ble_uuid16_t cccd_uuid = BLE_UUID16_INIT(0x2902);

/* Connection handle for the connected keyboard */
static uint16_t conn_handle = BLE_HS_CONN_HANDLE_NONE;

/* Flag: are we currently connected? */
static volatile bool is_connected = false;

/* Address of the keyboard we want to connect to */
static ble_addr_t peer_addr;
static bool connect_pending = false;

/* User callback for keyboard reports */
static ble_hid_keyboard_cb_t keyboard_callback = NULL;

/* ------------------------------------------------------------------ */
/* Stored characteristics for sequential subscription                   */
/* ------------------------------------------------------------------ */

#define MAX_HID_REPORT_CHARS 8

typedef struct {
    uint16_t val_handle;
    uint16_t end_handle;
    uint8_t  properties;
} hid_report_chr_t;

static uint16_t hid_svc_start = 0;
static uint16_t hid_svc_end = 0;

static hid_report_chr_t report_chars[MAX_HID_REPORT_CHARS];
static int num_report_chars = 0;
static int subscribe_index = 0;

/* Track whether we already started discovery (avoid duplicates) */
static volatile bool discovery_started = false;

/* Timer to handle security/discovery timeout after connection */
static TimerHandle_t connect_timeout_timer = NULL;
#define CONNECT_SECURITY_TIMEOUT_MS  2000

/* ------------------------------------------------------------------ */
/* Notification watchdog and polling fallback                           */
/* ------------------------------------------------------------------ */

/* Track whether notifications are actually arriving */
static volatile bool notifications_working = false;

/* Polling fallback timer */
static TimerHandle_t poll_timer = NULL;
static volatile bool polling_active = false;

/* Notification watchdog timer: if no notifications arrive within
 * this timeout after subscription, start polling fallback */
#define NOTIFICATION_WATCHDOG_MS 3000
static TimerHandle_t notification_watchdog_timer = NULL;

/* Last report for polling change detection */
static uint8_t last_polled_report[8];

/* ------------------------------------------------------------------ */
/* Forward declarations                                                */
/* ------------------------------------------------------------------ */

static void ble_hid_scan_start(void);
static int ble_hid_gap_event(struct ble_gap_event *event, void *arg);
static void subscribe_next_char(void);
static void start_connect(void);
static void start_service_discovery(void);

/* Wrapper task for deferred service discovery from timer context.
 * NimBLE GATT calls should not run directly in the timer task
 * to avoid stack overflow. */
static void deferred_discovery_task(void *arg)
{
    if (is_connected && !discovery_started) {
        ESP_LOGW(TAG, "Security timeout, trying GATT discovery without encryption");
        start_service_discovery();
    }
    vTaskDelete(NULL);
}

/* Called if no ENC_CHANGE arrives within timeout after security_initiate.
 * Spawns a short-lived task to run GATT discovery outside timer context. */
static void connect_timeout_callback(TimerHandle_t xTimer)
{
    if (!is_connected || discovery_started) return;

    xTaskCreate(deferred_discovery_task, "disc_defer", 4096, NULL, 5, NULL);
}
static int on_svc_discovery(uint16_t chn_conn_handle,
                            const struct ble_gatt_error *error,
                            const struct ble_gatt_svc *svc,
                            void *arg);

/* ------------------------------------------------------------------ */
/* Polling fallback: read report characteristic periodically           */
/* ------------------------------------------------------------------ */

static int on_poll_read_complete(uint16_t chn_conn_handle,
                                 const struct ble_gatt_error *error,
                                 struct ble_gatt_attr *attr,
                                 void *arg)
{
    if (error->status != 0) {
        return 0;
    }

    if (attr == NULL || attr->om == NULL) {
        return 0;
    }

    uint8_t buf[16];
    uint16_t len = OS_MBUF_PKTLEN(attr->om);
    if (len > sizeof(buf)) len = sizeof(buf);
    os_mbuf_copydata(attr->om, 0, len, buf);

    /* Only deliver if report changed */
    if (len >= sizeof(hid_keyboard_report_t) &&
        memcmp(buf, last_polled_report, sizeof(hid_keyboard_report_t)) != 0) {
        memcpy(last_polled_report, buf, sizeof(hid_keyboard_report_t));
        if (keyboard_callback) {
            keyboard_callback((hid_keyboard_report_t *)buf);
        }
    }

    return 0;
}

static void poll_timer_callback(TimerHandle_t xTimer)
{
    if (!polling_active || !is_connected || notifications_working) {
        polling_active = false;
        return;
    }

    if (num_report_chars > 0 && conn_handle != BLE_HS_CONN_HANDLE_NONE) {
        ble_gattc_read(conn_handle,
                       report_chars[0].val_handle,
                       on_poll_read_complete, NULL);
    }
}

static void notification_watchdog_callback(TimerHandle_t xTimer)
{
    if (!is_connected) return;

    if (!notifications_working && num_report_chars > 0) {
        ESP_LOGW(TAG, "No notifications received, starting poll fallback");
        polling_active = true;
        memset(last_polled_report, 0, sizeof(last_polled_report));
        if (poll_timer) {
            xTimerStart(poll_timer, 0);
        }
    }
}

static void hid_connection_ready(void)
{
    ESP_LOGI(TAG, "HID connection ready! %d report chars subscribed",
             num_report_chars);

    notifications_working = false;
    polling_active = false;

    /* Request fast connection parameters for responsive keyboard input.
     * min=7.5ms (6*1.25), max=15ms (12*1.25), latency=0, timeout=2s */
    struct ble_l2cap_sig_update_params params = {
        .itvl_min = 6,
        .itvl_max = 12,
        .slave_latency = 0,
        .timeout_multiplier = 200,
    };
    ble_l2cap_sig_update(conn_handle, &params, NULL, NULL);
    ESP_LOGI(TAG, "Requested fast connection params (7.5-15ms interval)");

    /* Start notification watchdog: if no notifications arrive within
     * the timeout, fall back to polling */
    if (notification_watchdog_timer) {
        xTimerStart(notification_watchdog_timer, 0);
    }
}

/* ------------------------------------------------------------------ */
/* Start HID service discovery                                         */
/* ------------------------------------------------------------------ */

static void start_service_discovery(void)
{
    if (discovery_started) {
        return;
    }
    discovery_started = true;

    ESP_LOGI(TAG, "Starting HID service discovery...");
    int rc = ble_gattc_disc_svc_by_uuid(conn_handle,
                                         &hid_svc_uuid.u,
                                         on_svc_discovery, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to start service discovery: %d", rc);
        discovery_started = false;
    }
}

/* ------------------------------------------------------------------ */
/* Sequential subscription: discover descriptors then write CCCD       */
/* ------------------------------------------------------------------ */

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

    subscribe_index++;
    subscribe_next_char();
    return 0;
}

static int on_dsc_discovery(uint16_t chn_conn_handle,
                            const struct ble_gatt_error *error,
                            uint16_t chr_val_handle,
                            const struct ble_gatt_dsc *dsc,
                            void *arg)
{
    if (error->status == BLE_HS_EDONE) {
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

    if (ble_uuid_cmp(&dsc->uuid.u, &cccd_uuid.u) == 0) {
        ESP_LOGI(TAG, "Found CCCD at handle 0x%04x for char index %d",
                 dsc->handle, subscribe_index);
        uint8_t notify_enable[] = {0x01, 0x00};
        int rc = ble_gattc_write_flat(chn_conn_handle, dsc->handle,
                                      notify_enable, sizeof(notify_enable),
                                      on_cccd_write_complete, NULL);
        if (rc != 0) {
            ESP_LOGE(TAG, "Failed to write CCCD: %d", rc);
            subscribe_index++;
            subscribe_next_char();
        }
        return BLE_HS_EDONE;
    }

    return 0;
}

static void subscribe_next_char(void)
{
    while (subscribe_index < num_report_chars) {
        hid_report_chr_t *chr = &report_chars[subscribe_index];

        if (!(chr->properties & BLE_GATT_CHR_PROP_NOTIFY)) {
            ESP_LOGD(TAG, "Char index %d has no NOTIFY, skipping",
                     subscribe_index);
            subscribe_index++;
            continue;
        }

        ESP_LOGI(TAG, "Subscribing to char index %d (handle=0x%04x)",
                 subscribe_index, chr->val_handle);

        int rc = ble_gattc_disc_all_dscs(conn_handle,
                                         chr->val_handle,
                                         chr->end_handle,
                                         on_dsc_discovery, NULL);
        if (rc != 0) {
            ESP_LOGE(TAG, "Failed to start descriptor discovery: %d", rc);
            subscribe_index++;
            continue;
        }
        return;
    }

    /* All subscriptions done */
    ESP_LOGI(TAG, "All HID Report subscriptions complete (%d chars processed)",
             num_report_chars);
    hid_connection_ready();
}

/* ------------------------------------------------------------------ */
/* GATT discovery callbacks                                            */
/* ------------------------------------------------------------------ */

static int on_chr_discovery(uint16_t chn_conn_handle,
                            const struct ble_gatt_error *error,
                            const struct ble_gatt_chr *chr,
                            void *arg)
{
    if (error->status == BLE_HS_EDONE) {
        ESP_LOGI(TAG, "Characteristic discovery complete, found %d report chars",
                 num_report_chars);

        subscribe_index = 0;

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

    /* Accept both Report (0x2A4D) and Boot Keyboard Input (0x2A22) chars */
    bool is_report = (ble_uuid_cmp(&chr->uuid.u, &hid_report_uuid.u) == 0);
    bool is_boot_kb = (ble_uuid_cmp(&chr->uuid.u, &boot_kb_input_uuid.u) == 0);

    if (is_report || is_boot_kb) {
        ESP_LOGI(TAG, "Found %s char, val_handle=0x%04x, props=0x%02x",
                 is_boot_kb ? "Boot KB Input" : "HID Report",
                 chr->val_handle, chr->properties);

        if (num_report_chars < MAX_HID_REPORT_CHARS) {
            report_chars[num_report_chars].val_handle = chr->val_handle;
            report_chars[num_report_chars].properties = chr->properties;
            report_chars[num_report_chars].end_handle = hid_svc_end;
            num_report_chars++;
        }
    }
    return 0;
}

static int on_svc_discovery(uint16_t chn_conn_handle,
                            const struct ble_gatt_error *error,
                            const struct ble_gatt_svc *svc,
                            void *arg)
{
    if (error->status == BLE_HS_EDONE) {
        if (hid_svc_start == 0) {
            ESP_LOGE(TAG, "HID service not found!");
        }
        return 0;
    }

    if (error->status != 0) {
        ESP_LOGE(TAG, "Service discovery error: %d (0x%04x)", error->status, error->status);
        discovery_started = false;

        /* If still connected, try to recover */
        if (is_connected && conn_handle != BLE_HS_CONN_HANDLE_NONE) {
            if (error->status == BLE_HS_ATT_ERR(0x05) ||  /* Insufficient Authentication */
                error->status == BLE_HS_ATT_ERR(0x0F) ||  /* Insufficient Encryption */
                error->status == BLE_HS_ATT_ERR(0x06)) {  /* Request Not Supported (try security) */
                ESP_LOGI(TAG, "Auth/encryption required, initiating security...");
                ble_gap_security_initiate(conn_handle);
            } else {
                ESP_LOGW(TAG, "Discovery failed but still connected, will retry after delay");
                /* The connect_timeout_timer or watchdog will handle retry */
            }
        }
        return 0;
    }

    ESP_LOGI(TAG, "Found HID service: start=0x%04x end=0x%04x",
             svc->start_handle, svc->end_handle);

    hid_svc_start = svc->start_handle;
    hid_svc_end = svc->end_handle;
    num_report_chars = 0;

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
/* Initiate BLE connection to saved peer address                       */
/* ------------------------------------------------------------------ */

static void start_connect(void)
{
    uint8_t own_addr_type;
    int rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to infer own addr type: %d", rc);
        own_addr_type = BLE_OWN_ADDR_PUBLIC;
    }

    /* Use conservative connection parameters initially.
     * Fast params (7.5-15ms) are requested later after HID is ready. */
    struct ble_gap_conn_params conn_params = {
        .scan_itvl = 0x0060,         /* 60 ms */
        .scan_window = 0x0030,       /* 30 ms */
        .itvl_min = 24,              /* 30 ms */
        .itvl_max = 40,              /* 50 ms */
        .latency = 0,
        .supervision_timeout = 400,  /* 4 seconds */
        .min_ce_len = 0,
        .max_ce_len = 0,
    };

    ESP_LOGI(TAG, "Connecting to keyboard (addr_type=%d)...", peer_addr.type);

    rc = ble_gap_connect(own_addr_type, &peer_addr,
                         30000, &conn_params,
                         ble_hid_gap_event, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to initiate connection: %d", rc);
        ble_hid_scan_start();
    }
}

/* ------------------------------------------------------------------ */
/* GAP event handler                                                   */
/* ------------------------------------------------------------------ */

static int ble_hid_gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {

    case BLE_GAP_EVENT_DISC: {
        struct ble_hs_adv_fields fields;
        int rc = ble_hs_adv_parse_fields(&fields, event->disc.data,
                                         event->disc.length_data);
        if (rc != 0) {
            break;
        }

        bool has_hid = false;

        for (int i = 0; i < fields.num_uuids16; i++) {
            if (ble_uuid_u16(&fields.uuids16[i].u) == 0x1812) {
                has_hid = true;
                break;
            }
        }

        if (!has_hid && fields.appearance_is_present &&
            (fields.appearance == 0x03C1 || fields.appearance == 0x03C0)) {
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
                ESP_LOGI(TAG, "Found HID keyboard: %.*s [%s] addr_type=%d",
                         fields.name_len, fields.name, addr_str,
                         event->disc.addr.type);
            } else {
                ESP_LOGI(TAG, "Found HID keyboard: [%s] addr_type=%d",
                         addr_str, event->disc.addr.type);
            }

            peer_addr = event->disc.addr;
            connect_pending = true;

            /* Stop scan and connect immediately.
             * On some NimBLE versions, disc_cancel does NOT generate
             * a DISC_COMPLETE event, so we connect directly. */
            int cancel_rc = ble_gap_disc_cancel();
            ESP_LOGI(TAG, "Scan cancel rc=%d, connecting now", cancel_rc);
            start_connect();
        }
        break;
    }

    case BLE_GAP_EVENT_DISC_COMPLETE: {
        ESP_LOGI(TAG, "Scan complete (reason=%d)", event->disc_complete.reason);
        /* Only restart scan if we are not already connecting/connected */
        if (!connect_pending && !is_connected) {
            ble_hid_scan_start();
        }
        break;
    }

    case BLE_GAP_EVENT_CONNECT: {
        if (event->connect.status == 0) {
            conn_handle = event->connect.conn_handle;
            is_connected = true;
            discovery_started = false;
            connect_pending = false;
            ESP_LOGI(TAG, "Connected (handle=%d)", conn_handle);

            /* Strategy: initiate security/pairing first.
             * If ENC_CHANGE arrives, we start GATT discovery.
             * If no response within timeout, we try GATT discovery
             * without encryption as a fallback. */
            ESP_LOGI(TAG, "Requesting security/pairing...");
            int sec_rc = ble_gap_security_initiate(conn_handle);
            ESP_LOGI(TAG, "Security initiate rc=%d", sec_rc);

            /* Start timeout timer: if no ENC_CHANGE within 2s,
             * try GATT discovery anyway */
            if (connect_timeout_timer) {
                xTimerStart(connect_timeout_timer, 0);
            }
        } else {
            ESP_LOGE(TAG, "Connection failed: status=%d",
                     event->connect.status);
            conn_handle = BLE_HS_CONN_HANDLE_NONE;
            is_connected = false;
            connect_pending = false;
            ble_hid_scan_start();
        }
        break;
    }

    case BLE_GAP_EVENT_DISCONNECT: {
        static int disconnect_count = 0;
        disconnect_count++;
        ESP_LOGW(TAG, "Disconnected (reason=0x%03x, count=%d)",
                 event->disconnect.reason, disconnect_count);
        conn_handle = BLE_HS_CONN_HANDLE_NONE;
        is_connected = false;
        connect_pending = false;
        discovery_started = false;
        num_report_chars = 0;

        /* Stop all timers */
        polling_active = false;
        notifications_working = false;
        if (poll_timer) xTimerStop(poll_timer, 0);
        if (notification_watchdog_timer) xTimerStop(notification_watchdog_timer, 0);
        if (connect_timeout_timer) xTimerStop(connect_timeout_timer, 0);

        int delay_ms = (disconnect_count < 3) ? 500 : 2000;
        ESP_LOGI(TAG, "Waiting %d ms before re-scan...", delay_ms);
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
        ble_hid_scan_start();
        break;
    }

    case BLE_GAP_EVENT_NOTIFY_RX: {
        if (keyboard_callback && event->notify_rx.om) {
            uint8_t buf[16];
            uint16_t len = OS_MBUF_PKTLEN(event->notify_rx.om);
            if (len > sizeof(buf)) {
                len = sizeof(buf);
            }
            os_mbuf_copydata(event->notify_rx.om, 0, len, buf);

            /* Notifications are working, stop polling if active */
            if (!notifications_working) {
                notifications_working = true;
                if (polling_active) {
                    ESP_LOGI(TAG, "Notifications working, stopping poll fallback");
                    polling_active = false;
                    if (poll_timer) xTimerStop(poll_timer, 0);
                }
            }

            if (len >= sizeof(hid_keyboard_report_t)) {
                hid_keyboard_report_t *report = (hid_keyboard_report_t *)buf;
                keyboard_callback(report);
            }
        }
        break;
    }

    case BLE_GAP_EVENT_ENC_CHANGE: {
        /* Cancel the timeout timer since security completed */
        if (connect_timeout_timer) xTimerStop(connect_timeout_timer, 0);

        if (event->enc_change.status == 0) {
            ESP_LOGI(TAG, "Encryption enabled (status=0)");
        } else {
            ESP_LOGW(TAG, "Encryption change status=%d",
                     event->enc_change.status);
        }

        /* Only start discovery if not already in progress */
        if (!discovery_started) {
            ESP_LOGI(TAG, "Starting GATT discovery after encryption");
            start_service_discovery();
        } else {
            ESP_LOGI(TAG, "Discovery already in progress, ignoring ENC_CHANGE");
        }
        break;
    }

    case BLE_GAP_EVENT_REPEAT_PAIRING: {
        struct ble_gap_conn_desc desc;
        ble_gap_conn_find(event->repeat_pairing.conn_handle, &desc);
        ble_store_util_delete_peer(&desc.peer_id_addr);
        ESP_LOGI(TAG, "Repeat pairing - deleted old bond, retrying");
        return BLE_GAP_REPEAT_PAIRING_RETRY;
    }

    case BLE_GAP_EVENT_CONN_UPDATE: {
        ESP_LOGI(TAG, "Connection parameters updated (status=%d)",
                 event->conn_update.status);
        break;
    }

    case BLE_GAP_EVENT_MTU: {
        ESP_LOGI(TAG, "MTU updated: %d", event->mtu.value);
        break;
    }

    case BLE_GAP_EVENT_PASSKEY_ACTION: {
        ESP_LOGI(TAG, "Passkey action: %d", event->passkey.params.action);
        if (event->passkey.params.action == BLE_SM_IOACT_NUMCMP) {
            struct ble_sm_io pk = {0};
            pk.action = event->passkey.params.action;
            pk.numcmp_accept = 1;
            ble_sm_inject_io(event->passkey.conn_handle, &pk);
        } else if (event->passkey.params.action == BLE_SM_IOACT_NONE) {
            /* Just Works - nothing to do */
        }
        break;
    }

    default:
        ESP_LOGI(TAG, "Unhandled GAP event: %d", event->type);
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
        .itvl = 0x0060,
        .window = 0x0030,
        .filter_policy = 0,
        .limited = 0,
        .passive = 0,
        .filter_duplicates = 1,
    };

    uint8_t own_addr_type;
    int rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to infer own addr type: %d", rc);
        own_addr_type = BLE_OWN_ADDR_PUBLIC;
    }

    ESP_LOGI(TAG, "Starting BLE scan for HID keyboards...");

    rc = ble_gap_disc(own_addr_type, BLE_HS_FOREVER,
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
    int rc = ble_hs_util_ensure_addr(0);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to ensure address: %d", rc);
        return;
    }

    /* Log bond store status */
    ESP_LOGI(TAG, "BLE address configured");

    ble_hid_scan_start();
}

static void ble_hid_on_reset(int reason)
{
    ESP_LOGW(TAG, "NimBLE host reset, reason=%d", reason);
}

static void nimble_host_task(void *param)
{
    ESP_LOGI(TAG, "NimBLE host task started");
    nimble_port_run();
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

    /* Create polling fallback timer (50ms period, auto-reload) */
    poll_timer = xTimerCreate("hid_poll", pdMS_TO_TICKS(50),
                              pdTRUE, NULL, poll_timer_callback);

    /* Create notification watchdog timer (one-shot) */
    notification_watchdog_timer = xTimerCreate("hid_watchdog",
                                               pdMS_TO_TICKS(NOTIFICATION_WATCHDOG_MS),
                                               pdFALSE, NULL,
                                               notification_watchdog_callback);

    /* Create connect/security timeout timer (one-shot) */
    connect_timeout_timer = xTimerCreate("conn_timeout",
                                          pdMS_TO_TICKS(CONNECT_SECURITY_TIMEOUT_MS),
                                          pdFALSE, NULL,
                                          connect_timeout_callback);

    /* Initialize NimBLE */
    ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init NimBLE port: %d", ret);
        return ret;
    }

    /* Configure the NimBLE host */
    ble_hs_cfg.reset_cb = ble_hid_on_reset;
    ble_hs_cfg.sync_cb = ble_hid_on_sync;

    /* Security: prefer Secure Connections, fall back to legacy pairing.
     * NoIO capability (Just Works). Enable bonding. */
    ble_hs_cfg.sm_io_cap = BLE_SM_IO_CAP_NO_IO;
    ble_hs_cfg.sm_bonding = 1;
    ble_hs_cfg.sm_mitm = 0;
    ble_hs_cfg.sm_sc = 1;
    ble_hs_cfg.sm_our_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
    ble_hs_cfg.sm_their_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;

    /* Register the bond/key store backend callbacks.
     * WITHOUT these, all SM operations return BLE_HS_ENOTSUP
     * because NimBLE has no way to persist or read security keys. */
    ble_hs_cfg.store_read_cb = ble_store_config_read;
    ble_hs_cfg.store_write_cb = ble_store_config_write;
    ble_hs_cfg.store_delete_cb = ble_store_config_delete;

    /* Start the NimBLE host task */
    nimble_port_freertos_init(nimble_host_task);

    ESP_LOGI(TAG, "BLE HID Host initialized");
    return ESP_OK;
}

bool ble_hid_host_connected(void)
{
    return is_connected;
}
