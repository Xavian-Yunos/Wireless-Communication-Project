/* BLE presence detection module using NimBLE */

#include "ble_presence.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "host/ble_store.h"
#include "host/util/util.h"
#include "esp_bt.h"

static const char *TAG = "ble_presence";

#define RSSI_EMA_ALPHA          (0.25f)
#define METRIC_CB_INTERVAL_MS   (5000)  /* Report RSSI every 5 seconds */
#define TARGET_NAME_MAX_LEN     64

static bool s_enabled = true;
static bool s_auto_activate = true;
static uint32_t s_present_interval = 5;
static uint32_t s_absent_interval = 30;  /* Increased default to keep relay on longer */
static char s_target_name[TARGET_NAME_MAX_LEN] = {0};
static ble_presence_state_t s_state = {
    .present = false,
    .has_rssi = false,
    .rssi = -127,
};
static float s_rssi_ema;
static bool s_rssi_ema_valid;

static TickType_t s_last_seen_tick;
static TickType_t s_first_seen_tick;
static TickType_t s_last_metric_cb_tick;
static bool s_nimble_ready;
static bool s_initialized;

static ble_presence_state_cb_t s_state_cb;
static void *s_state_cb_priv;
static TaskHandle_t s_presence_task_handle;

void ble_store_config_init(void);

static int8_t filter_rssi(int8_t rssi)
{
    if (!s_rssi_ema_valid) {
        s_rssi_ema = (float)rssi;
        s_rssi_ema_valid = true;
    } else {
        s_rssi_ema = (RSSI_EMA_ALPHA * (float)rssi) + ((1.0f - RSSI_EMA_ALPHA) * s_rssi_ema);
    }
    if (s_rssi_ema < -127.0f) {
        s_rssi_ema = -127.0f;
    } else if (s_rssi_ema > 20.0f) {
        s_rssi_ema = 20.0f;
    }
    return (int8_t)lroundf(s_rssi_ema);
}

static void reset_runtime_metrics(void)
{
    s_first_seen_tick = 0;
    s_last_seen_tick = 0;
    s_rssi_ema_valid = false;
    s_state.has_rssi = false;
    s_state.rssi = -127;
}

/* Extract device name from BLE advertisement data.
 * Returns true if name was found, false otherwise. */
static bool extract_adv_name(const uint8_t *data, uint8_t data_len, char *out_name, size_t out_len)
{
    if (!data || data_len == 0 || !out_name || out_len == 0) {
        return false;
    }

    uint8_t i = 0;
    while (i < data_len) {
        uint8_t field_len = data[i];
        if (field_len == 0 || (i + 1 + field_len) > data_len) {
            break;
        }

        uint8_t field_type = data[i + 1];
        /* 0x09 = Complete Local Name, 0x08 = Shortened Local Name */
        if (field_type == 0x09 || field_type == 0x08) {
            uint8_t name_len = field_len - 1;
            if (name_len > 0) {
                size_t copy_len = (name_len < out_len - 1) ? name_len : (out_len - 1);
                memcpy(out_name, &data[i + 2], copy_len);
                out_name[copy_len] = '\0';
                return true;
            }
        }
        i += 1 + field_len;
    }
    return false;
}

static void emit_state_update(bool immediate)
{
    if (!s_state_cb) {
        return;
    }

    if (!immediate) {
        TickType_t now = xTaskGetTickCount();
        TickType_t min_interval = pdMS_TO_TICKS(METRIC_CB_INTERVAL_MS);
        if ((now - s_last_metric_cb_tick) < min_interval) {
            return;
        }
        s_last_metric_cb_tick = now;
    } else {
        s_last_metric_cb_tick = xTaskGetTickCount();
    }
    s_state_cb(&s_state, s_state_cb_priv);
}

static void maybe_report_presence_change(bool present)
{
    if (s_state.present == present) {
        if (!present) {
            bool had_metrics = s_state.has_rssi;
            reset_runtime_metrics();
            if (had_metrics) {
                emit_state_update(true);
            }
        }
        return;
    }

    s_state.present = present;
    if (!present) {
        reset_runtime_metrics();
    }
    emit_state_update(true);
}

static uint32_t s_debug_log_counter = 0;

static int gap_event_cb(struct ble_gap_event *event, void *arg)
{
    (void)arg;

    switch (event->type) {
    case BLE_GAP_EVENT_DISC: {
        if (!s_enabled) {
            return 0;
        }

        /* Extract device name from advertisement data */
        char device_name[TARGET_NAME_MAX_LEN] = {0};
        bool has_name = extract_adv_name(event->disc.data, event->disc.length_data, 
                                          device_name, sizeof(device_name));

        /* Debug: Log every 200th discovered device to show scanning is working */
        if ((s_debug_log_counter++ % 200) == 0) {
            ESP_LOGI(TAG, "BLE scan active: looking for '%s' (saw %lu devices)",
                     s_target_name, (unsigned long)s_debug_log_counter);
        }

        /* Skip if no target name configured or device has no name */
        if (s_target_name[0] == '\0' || !has_name) {
            return 0;
        }

        /* Match by device name (case-sensitive) */
        if (strcmp(device_name, s_target_name) == 0) {
            TickType_t now = xTaskGetTickCount();
            s_last_seen_tick = now;
            if (s_first_seen_tick == 0) {
                s_first_seen_tick = now;
            }

            int8_t filtered_rssi = filter_rssi((int8_t)event->disc.rssi);
            s_state.has_rssi = true;
            s_state.rssi = filtered_rssi;

            uint32_t seen_for = (uint32_t)((now - s_first_seen_tick) / configTICK_RATE_HZ);
            if (!s_state.present && seen_for >= s_present_interval) {
                ESP_LOGI(TAG, "Target present: '%s'", s_target_name);
                maybe_report_presence_change(true);
            } else {
                emit_state_update(false);
            }
        }
        return 0;
    }
    case BLE_GAP_EVENT_DISC_COMPLETE:
        ESP_LOGW(TAG, "Scan stopped (reason=%d), restarting", event->disc_complete.reason);
        if (s_enabled && s_nimble_ready) {
            uint8_t own_addr_type;
            if (ble_hs_id_infer_auto(0, &own_addr_type) == 0) {
                struct ble_gap_disc_params params = { 0 };
                params.passive = 0;
                params.filter_duplicates = 0;
                params.itvl = 0x50;
                params.window = 0x30;
                (void)ble_gap_disc(own_addr_type, BLE_HS_FOREVER, &params, gap_event_cb, NULL);
            }
        }
        return 0;
    default:
        return 0;
    }
}

static void start_scan(void)
{
    if (!s_nimble_ready || !s_enabled) {
        return;
    }

    uint8_t own_addr_type;
    int rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_hs_id_infer_auto failed: %d", rc);
        return;
    }

    struct ble_gap_disc_params params = { 0 };
    params.passive = 0;  /* Active scanning to get scan responses with device names */
    params.filter_duplicates = 0;  /* Allow duplicates to track RSSI changes */
    params.itvl = 0x50;  /* Scan interval (50ms in 0.625ms units = 80) */
    params.window = 0x30; /* Scan window (30ms in 0.625ms units = 48) */

    rc = ble_gap_disc(own_addr_type, BLE_HS_FOREVER, &params, gap_event_cb, NULL);
    if (rc != 0 && rc != BLE_HS_EALREADY) {
        ESP_LOGE(TAG, "ble_gap_disc failed: %d", rc);
    } else {
        ESP_LOGI(TAG, "BLE active scan started (looking for '%s')", s_target_name);
    }
}

static void on_reset(int reason)
{
    ESP_LOGW(TAG, "NimBLE reset reason=%d", reason);
}

static void on_sync(void)
{
    int rc = ble_hs_util_ensure_addr(0);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_hs_util_ensure_addr failed: %d", rc);
        return;
    }
    s_nimble_ready = true;
    start_scan();
}

static void host_task(void *param)
{
    (void)param;
    nimble_port_run();
    nimble_port_freertos_deinit();
}

static void presence_watchdog_task(void *arg)
{
    (void)arg;
    while (1) {
        if (s_enabled && s_state.present) {
            TickType_t now = xTaskGetTickCount();
            uint32_t absent_for = (uint32_t)((now - s_last_seen_tick) / configTICK_RATE_HZ);
            if (absent_for >= s_absent_interval) {
                ESP_LOGI(TAG, "Target absent: '%s'", s_target_name);
                maybe_report_presence_change(false);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

esp_err_t ble_presence_init(const char *target_name, ble_presence_state_cb_t cb, void *priv_data)
{
    if (s_initialized) {
        s_state_cb = cb;
        s_state_cb_priv = priv_data;
        s_enabled = true;
        start_scan();
        return ESP_OK;
    }

    if (target_name) {
        int rc = ble_presence_set_target_name(target_name);
        if (rc != 0) {
            return ESP_ERR_INVALID_ARG;
        }
    }

    s_state_cb = cb;
    s_state_cb_priv = priv_data;
    reset_runtime_metrics();
    s_last_metric_cb_tick = 0;
    s_nimble_ready = false;

    /* Check BT controller state and handle recovery from provisioning */
    esp_bt_controller_status_t bt_status = esp_bt_controller_get_status();
    ESP_LOGI(TAG, "BT controller status: %d (0=IDLE, 1=INITED, 2=ENABLED)", bt_status);

    if (bt_status == ESP_BT_CONTROLLER_STATUS_ENABLED) {
        /* Controller already enabled (e.g., still active from provisioning) - 
         * need to disable and deinit before nimble_port_init can work */
        ESP_LOGI(TAG, "Disabling active BT controller...");
        esp_bt_controller_disable();
        esp_bt_controller_deinit();
        bt_status = esp_bt_controller_get_status();
        ESP_LOGI(TAG, "After deinit, status: %d", bt_status);
    } else if (bt_status == ESP_BT_CONTROLLER_STATUS_INITED) {
        /* Controller initialized but not enabled - deinit it */
        ESP_LOGI(TAG, "Deinitializing BT controller...");
        esp_bt_controller_deinit();
        bt_status = esp_bt_controller_get_status();
    }

    /* Now nimble_port_init() should handle everything from clean state */
    esp_err_t err = nimble_port_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nimble_port_init failed: %d", err);
        
        /* If nimble_port_init failed, try manual controller init as fallback */
        if (bt_status == ESP_BT_CONTROLLER_STATUS_IDLE) {
            ESP_LOGI(TAG, "Attempting manual BT controller initialization...");
            esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
            err = esp_bt_controller_init(&bt_cfg);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "esp_bt_controller_init failed: %d", err);
                return err;
            }
            err = esp_bt_controller_enable(ESP_BT_MODE_BLE);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "esp_bt_controller_enable failed: %d", err);
                esp_bt_controller_deinit();
                return err;
            }
            
            /* Try nimble_port_init again */
            err = nimble_port_init();
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "nimble_port_init still failed after manual init: %d", err);
                esp_bt_controller_disable();
                esp_bt_controller_deinit();
                return err;
            }
        } else {
            return err;
        }
    }

    ble_hs_cfg.reset_cb = on_reset;
    ble_hs_cfg.sync_cb = on_sync;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;
    ble_store_config_init();

    nimble_port_freertos_init(host_task);
    s_initialized = true;

    if (!s_presence_task_handle) {
        xTaskCreate(presence_watchdog_task, "ble_presence_watchdog", 3072, NULL, 5, &s_presence_task_handle);
    }
    return ESP_OK;
}

void ble_presence_enable(bool enable)
{
    s_enabled = enable;
    if (!enable) {
        (void)ble_gap_disc_cancel();
        maybe_report_presence_change(false);
        return;
    }
    start_scan();
}

bool ble_presence_get_state(void)
{
    return s_state.present;
}

esp_err_t ble_presence_get_latest(ble_presence_state_t *out_state)
{
    if (!out_state) {
        return ESP_ERR_INVALID_ARG;
    }
    *out_state = s_state;
    return ESP_OK;
}

void ble_presence_set_auto_activate(bool enable)
{
    s_auto_activate = enable;
}

bool ble_presence_get_enabled(void)
{
    return s_enabled;
}

bool ble_presence_is_initialized(void)
{
    return s_initialized;
}

bool ble_presence_get_auto_activate(void)
{
    return s_auto_activate;
}

const char *ble_presence_get_target_name(void)
{
    return s_target_name;
}

void ble_presence_set_presence_interval(uint32_t present_secs)
{
    s_present_interval = present_secs;
}

void ble_presence_set_absence_interval(uint32_t absent_secs)
{
    s_absent_interval = absent_secs;
}

int ble_presence_set_target_name(const char *name)
{
    if (!name || strlen(name) == 0 || strlen(name) >= TARGET_NAME_MAX_LEN) {
        return -1;
    }
    strlcpy(s_target_name, name, sizeof(s_target_name));
    reset_runtime_metrics();
    maybe_report_presence_change(false);
    return 0;
}
