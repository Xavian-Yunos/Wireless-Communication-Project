/* Bubbles Presence Detection Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <string.h>
#include <inttypes.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_event.h>
#include <nvs_flash.h>

#include <esp_rmaker_core.h>
#include <esp_rmaker_standard_types.h>
#include <esp_rmaker_standard_params.h>
#include <esp_rmaker_standard_devices.h>
#include <esp_rmaker_console.h>
#include <esp_rmaker_ota.h>

#include <esp_rmaker_common_events.h>

#include <app_network.h>
#include <app_insights.h>

#include "app_priv.h"
#include "ble_presence.h"

static const char *TAG = "app_main";

/* Restricted area state tracking */
static bool g_in_restricted_area_reported; // Tracks the last reported state of whether the target is in the restricted area or not
static bool g_in_restricted_area_reported_valid; // Tracks whether the reported state is valid

static int g_rssi_on_threshold = CONFIG_EXAMPLE_RSSI_ON_THRESHOLD; 
static int g_rssi_off_threshold = CONFIG_EXAMPLE_RSSI_OFF_THRESHOLD;

static bool g_ble_init_deferred = false; 

static void ble_presence_state_changed(const ble_presence_state_t *state, void *priv_data);
static void try_init_ble_presence(void);

static void check_restricted_area_if_needed(bool in_restricted_area) // Function to check if we need to send an alert for entering/leaving the restricted area based on the new state and the last reported state
{
    if (!g_in_restricted_area_reported_valid || g_in_restricted_area_reported != in_restricted_area) {
        g_in_restricted_area_reported = in_restricted_area;
        g_in_restricted_area_reported_valid = true;
        
        /* Send notification for entering/leaving restricted area */
        const char *target_name = ble_presence_get_target_name();
        if (!target_name || target_name[0] == '\0') {   // Not "Bubbles" OR no target
            target_name = CONFIG_EXAMPLE_TARGET_NAME;
        }
        
        char alert_msg[128];
        if (in_restricted_area) {   // Send "Bubbles entered the restricted area message"
            snprintf(alert_msg, sizeof(alert_msg), "%s entered the restricted area", target_name);
            esp_rmaker_raise_alert(alert_msg);  // Raise alert in RainMaker app
            ESP_LOGI(TAG, "Alert: %s", alert_msg);
        } else {    // Send "Bubbles left the restricted area message"
            snprintf(alert_msg, sizeof(alert_msg), "%s left the restricted area", target_name);
            esp_rmaker_raise_alert(alert_msg);  // Raise alert in RainMaker app
            ESP_LOGI(TAG, "Alert: %s", alert_msg);
        }
    }
}


static void try_init_ble_presence(void)     // init ble_presence
{
    if (ble_presence_is_initialized()) {    // If BLE presence is already initialized (e.g. from previous MQTT connection), just enable it again to start scanning
        ble_presence_enable(true);
        return;
    }

    /* Give a small delay to ensure BLE resources from provisioning are fully released */
    vTaskDelay(pdMS_TO_TICKS(500)); // Delay for 500 ms

    esp_err_t err = ble_presence_init(CONFIG_EXAMPLE_TARGET_NAME, ble_presence_state_changed, NULL);    // Initialize BLE presence detection with the target name and state change callback
    if (err != ESP_OK) {    // If initialization failed, log a warning but continue without BLE presence functionality
        ESP_LOGW(TAG, "BLE presence init failed: %d", err);
    } else {    // If initialization succeeded, enable BLE presence detection and check the initial state to send an alert if the target is already in the restricted area
        ESP_LOGI(TAG, "BLE presence initialized - scanning for %s", CONFIG_EXAMPLE_TARGET_NAME);
        ble_presence_state_t state = {0};   // Initialize state structure to hold the latest presence information
        if (ble_presence_get_latest(&state) == ESP_OK) {    // Get the latest presence state (if available) to determine if we need to send an alert immediately
            ble_presence_state_changed(&state, NULL);   // Call the state change handler with the latest state to check if the target is already in the restricted area and send an alert if needed
        }
    }
    g_ble_init_deferred = false;    // Clear the deferred initialization flag now that we've attempted to initialize BLE presence
}

static void ble_presence_state_changed(const ble_presence_state_t *state, void *priv_data)  // Callback function that gets called whenever there is a change in the BLE presence state (e.g. target detected, lost, or RSSI updated)
{
    (void)priv_data;    // Unused parameter
    if (!state) {   // If state is NULL, just return as we have no information to process
        return; // No state information available
    }

    /* Determine if target is in restricted area based on RSSI thresholds */
    bool in_restricted_area = false;
    
    if (state->present && state->has_rssi) {
        // Use hysteresis: enter at high RSSI, leave at low RSSI
        if (g_in_restricted_area_reported_valid && g_in_restricted_area_reported) {
            // Already in area - only leave if RSSI drops below off threshold
            in_restricted_area = (state->rssi > g_rssi_off_threshold);
        } else {
            // Not in area - only enter if RSSI exceeds on threshold
            in_restricted_area = (state->rssi >= g_rssi_on_threshold);
        }
        ESP_LOGD(TAG, "RSSI: %d dBm, in_area: %d", state->rssi, in_restricted_area);
    }
    
    check_restricted_area_if_needed(in_restricted_area); // Check if we need to send an alert for entering/leaving the restricted area based on the new state
}

/* Event handler for catching RainMaker events */
static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data)
{
    (void)arg;
    if (event_base == RMAKER_EVENT) { // Handle RainMaker-specific events related to provisioning and claim status
        switch (event_id) {
            case RMAKER_EVENT_INIT_DONE:
                ESP_LOGI(TAG, "RainMaker Initialised.");
                break;
            case RMAKER_EVENT_CLAIM_STARTED:
                ESP_LOGI(TAG, "RainMaker Claim Started.");
                break;
            case RMAKER_EVENT_CLAIM_SUCCESSFUL:
                ESP_LOGI(TAG, "RainMaker Claim Successful.");
                break;
            case RMAKER_EVENT_CLAIM_FAILED:
                ESP_LOGI(TAG, "RainMaker Claim Failed.");
                break;
            default:
                break;
        }
    } else if (event_base == RMAKER_COMMON_EVENT) { // Handle common events like reboot, Wi-Fi reset, factory reset, and MQTT connection status that may be relevant for the application
        switch (event_id) {
            case RMAKER_EVENT_REBOOT:
                ESP_LOGI(TAG, "Rebooting in %d seconds.", *((uint8_t *)event_data));
                break;
            case RMAKER_EVENT_WIFI_RESET:
                ESP_LOGI(TAG, "Wi-Fi credentials reset.");
                break;
            case RMAKER_EVENT_FACTORY_RESET:
                ESP_LOGI(TAG, "Node reset to factory defaults.");
                break;
            case RMAKER_MQTT_EVENT_CONNECTED:
                ESP_LOGI(TAG, "MQTT Connected.");
                if (g_ble_init_deferred) {
                    ESP_LOGI(TAG, "Starting BLE presence detection...");
                    try_init_ble_presence();
                }
                break;
            case RMAKER_MQTT_EVENT_DISCONNECTED:
                ESP_LOGI(TAG, "MQTT Disconnected.");
                break;
            default:
                break;
        }
    } else if (event_base == APP_NETWORK_EVENT) { // Handle network provisioning events such as QR code display, provisioning timeout, and provisioning restart that are relevant for the user experience during setup
        switch (event_id) {
            case APP_NETWORK_EVENT_QR_DISPLAY:
                ESP_LOGI(TAG, "Provisioning QR : %s", (char *)event_data);
                break;
            case APP_NETWORK_EVENT_PROV_TIMEOUT:
                ESP_LOGI(TAG, "Provisioning Timed Out. Please reboot.");
                break;
            case APP_NETWORK_EVENT_PROV_RESTART:
                ESP_LOGI(TAG, "Provisioning has restarted due to failures.");
                break;
            default:
                break;
        }
    } else if (event_base == RMAKER_OTA_EVENT) { // Handle OTA events related to firmware updates initiated from the RainMaker app, which can be useful for debugging OTA issues and providing feedback to the user
        switch (event_id) {
            case RMAKER_OTA_EVENT_STARTING:
                ESP_LOGI(TAG, "Starting OTA.");
                break;
            case RMAKER_OTA_EVENT_SUCCESSFUL:
                ESP_LOGI(TAG, "OTA successful.");
                break;
            case RMAKER_OTA_EVENT_FAILED:
                ESP_LOGI(TAG, "OTA Failed.");
                break;
            case RMAKER_OTA_EVENT_REQ_FOR_REBOOT:
                ESP_LOGI(TAG, "Firmware downloaded. Please reboot to apply.");
                break;
            default:
                break;
        }
    }
}

void app_main()
{
    esp_rmaker_console_init(); // Initialize console to print logs during provisioning
    app_driver_init(); // Initialize button driver for reset functions

    esp_err_t err = nvs_flash_init(); // Initialize NVS flash for storing Wi-Fi credentials and other data
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) { // NVS partition was truncated or new version found, erase it
        ESP_ERROR_CHECK(nvs_flash_erase()); // Erase NVS flash
        err = nvs_flash_init(); // Retry initializing NVS flash
    }
    ESP_ERROR_CHECK(err); // Check for errors in NVS flash initialization

    app_network_init(); // Initialize network provisioning and connection management

    ESP_ERROR_CHECK(esp_event_handler_register(RMAKER_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL)); // Register event handler for RainMaker events
    ESP_ERROR_CHECK(esp_event_handler_register(RMAKER_COMMON_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL)); // Register event handler for common events like reset and MQTT connection
    ESP_ERROR_CHECK(esp_event_handler_register(APP_NETWORK_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL)); // Register event handler for network provisioning events
    ESP_ERROR_CHECK(esp_event_handler_register(RMAKER_OTA_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL)); // Register event handler for OTA events

    esp_rmaker_config_t rainmaker_cfg = { // RainMaker configuration
        .enable_time_sync = false, // Disable time synchronization as it's not needed for this application
    };
    esp_rmaker_node_t *node = esp_rmaker_node_init(&rainmaker_cfg, "Bubbles Detector", "BLE Presence Sensor"); // Initialize RainMaker node with the name "Bubbles Detector" and type "BLE Presence Sensor"
    if (!node) { // Check if node initialization was successful
        ESP_LOGE(TAG, "Could not initialise node. Aborting!!!"); // Log error and abort if node initialization failed
        vTaskDelay(pdMS_TO_TICKS(5000)); // Delay to allow log to be sent before aborting
        abort(); // Abort the program
    }

    /* Create a simple device to show in RainMaker app */
    esp_rmaker_device_t *sensor_device = esp_rmaker_device_create("Bubbles Sensor", ESP_RMAKER_DEVICE_OTHER, NULL); // Create a device named "Bubbles Sensor" of type "Other"
    esp_rmaker_device_add_param(sensor_device, esp_rmaker_name_param_create("name", "Bubbles Sensor")); // Add a parameter to the device to show its name in the app
    esp_rmaker_node_add_device(node, sensor_device); // Add the device to the RainMaker node

    esp_rmaker_ota_enable_default(); // Enable default OTA handler to allow firmware updates from the RainMaker app
    app_insights_enable(); // Enable application insights for monitoring and diagnostics

    esp_rmaker_start(); // Start the RainMaker framework, which will handle provisioning, MQTT connection, and other functionalities

    err = app_network_start(POP_TYPE_RANDOM); // Start the network provisioning and connection process with random proof of possession (POP)
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Could not start Wifi. Aborting!!!"); // Log error and abort if network start failed
        vTaskDelay(pdMS_TO_TICKS(5000)); // Delay to allow log to be sent before aborting
        abort(); // Abort the program
    }

    ESP_LOGI(TAG, "Bubbles detector started - waiting for MQTT connection..."); // Log that the application has started and is waiting for MQTT connection to initialize BLE presence detection
    g_ble_init_deferred = true; // Defer BLE presence initialization until MQTT connection is established, as it may require network connectivity for certain operations (like fetching target name from cloud)
}