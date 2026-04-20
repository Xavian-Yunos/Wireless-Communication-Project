#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <esp_err.h>
#include <esp_rmaker_core.h>

typedef struct {
    bool present;
    bool has_rssi;
    int8_t rssi;
} ble_presence_state_t;

typedef void (*ble_presence_state_cb_t)(const ble_presence_state_t *state, void *priv_data);

esp_err_t ble_presence_init(const char *target_name, ble_presence_state_cb_t cb, void *priv_data);
void ble_presence_enable(bool enable);
bool ble_presence_get_state(void);
esp_err_t ble_presence_get_latest(ble_presence_state_t *out_state);
void ble_presence_set_auto_activate(bool enable);
bool ble_presence_get_enabled(void);
bool ble_presence_is_initialized(void);
bool ble_presence_get_auto_activate(void);
const char *ble_presence_get_target_name(void);

// Presence thresholds (in seconds)
void ble_presence_set_presence_interval(uint32_t present_secs);
void ble_presence_set_absence_interval(uint32_t absent_secs);

// Utility
int ble_presence_set_target_name(const char *name);
