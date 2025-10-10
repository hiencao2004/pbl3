
/* Standard C/C++ Libraries */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <time.h>

/* FreeRTOS Libraries */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

/* ESP-IDF Core & System Libraries */
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_timer.h"
#include "rom/ets_sys.h"

/* ESP-IDF Driver Libraries */
#include "driver/gpio.h"

/* ESP-IDF Network Libraries */
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "mqtt_client.h"

/* Custom Component & Sensor Libraries */
#include "ds18b20.h"
#include "mq2_sensor.h"
#include "flame_sensor.h"
#include "RCSwitch.h"


// ============================
// --- CONFIGURATION ---
// ============================

// --- General ---
#define LED_PIN             GPIO_NUM_2
#define SENSOR_POLL_INTERVAL_MS 2000

// --- Wi-Fi & MQTT ---
#define WIFI_SSID           "OYE TRA SUA T2"
#define WIFI_PASS           "39393939"
#define MQTT_BROKER_URI     "mqtt://mqtt.pbl3.click"
#define MQTT_TOPIC_DATA     "sensor/TU_1_NHABEP/data"      // For periodic sensor data
#define MQTT_TOPIC_FIRE     "fire1"     // For immediate fire alerts

// --- Sensor Thresholds ---
#define FIRE_THRESHOLD_C        31.2f
#define GAS_THRESHOLD_LIGHT     0       // A value of 0 triggers on any reading above calibrated baseline.
#define GAS_THRESHOLD_STRONG    80

// --- Flame Sensor Array ---
static const gpio_num_t FLAME_SENSOR_PINS[] = {
    GPIO_NUM_13, GPIO_NUM_12, GPIO_NUM_14, GPIO_NUM_27, GPIO_NUM_26
};
#define NUM_FLAME_SENSORS (sizeof(FLAME_SENSOR_PINS) / sizeof(FLAME_SENSOR_PINS[0]))
#define FLAME_ALARM_THRESHOLD 2 // How many flame sensors must be active to trigger alarm

// --- RF Remote Control ---
#define RF_RECEIVER_PIN     GPIO_NUM_35 // RF receiver data pin
#define LEARN_BUTTON_PIN    GPIO_NUM_18 // Button to enter learning mode
#define DELETE_BUTTON_PIN   GPIO_NUM_5  // Button to delete all learned codes
#define NVS_NAMESPACE       "storage"   // NVS namespace for storing RF codes
#define MAX_RF_CODES        10          // Max number of RF codes that can be learned

// --- Manual Fire Control ---
#define MANUAL_ALARM_PIN    GPIO_NUM_25 // Pin for manual fire trigger button
#define MANUAL_RESET_PIN    GPIO_NUM_33 // Pin for manual alarm reset button


// ============================
// --- GLOBALS & TYPE DEFS ---
// ============================

static const char *TAG = "GATEWAY_FIRE_SYSTEM";
static const char *STATUS_TAG = "TRẠNG THÁI HỆ THỐNG";

// --- Network & ESP-NOW ---
static const uint8_t PEER_MAC[6] = {0xA0, 0xA3, 0xB3, 0xA9, 0xE9, 0x34};
static uint8_t s_local_mac[6] = {0x78, 0x1C, 0x3C, 0x2B, 0xC5, 0x64};
static esp_mqtt_client_handle_t mqtt_client = NULL;
static bool mqtt_connected = false;
static uint8_t last_cmd_sent_espnow = 0xFF;

// --- System State Variables ---
static bool alarm_on_state = false; // The final, global alarm state

// --- Individual Local Alarm Source States ---
static bool g_temp_gas_fire_state = false;
static bool g_flame_consensus_fire_state = false;
static bool g_rf_triggered_fire_state = false;
static bool g_manual_triggered_fire_state = false;

// --- Flame Sensor State Array ---
static bool g_flame_sensor_states[NUM_FLAME_SENSORS];

// --- RF Control Globals ---
RCSWITCH_t rf_receiver;
unsigned long learned_rf_codes[MAX_RF_CODES] = {0};
int num_learned_codes = 0;
bool is_learning_mode = false;

// --- Data Structures ---
typedef struct {
    float temperature;
    int gas_level;
    bool combined_local_fire; // Aggregate of ALL local triggers (temp, gas, flame, RF, manual)
    bool remote_fire;         // Fire alert received from peer device
} sensor_state_t;

typedef struct __attribute__((packed)) {
    uint8_t cmd; // 0 = Safe, 1 = Fire detected
} espnow_payload_t;

// --- Shared Resources ---
static sensor_state_t sensor_data;
static SemaphoreHandle_t data_mutex;

// --- Forward Declarations ---
static void update_and_propagate_alarm_state(void);


// ============================
// --- PERIPHERAL INITIALIZATION ---
// ============================
void init_nvs();
void init_rf_control_pins();
void init_manual_control_pins();

/**
 * @brief Initializes the NVS flash partition.
 */
void init_nvs() {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
}

/**
 * @brief Initializes GPIO pins for RF control buttons.
 */
void init_rf_control_pins() {
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << LEARN_BUTTON_PIN) | (1ULL << DELETE_BUTTON_PIN);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);
}

/**
 * @brief Initializes GPIO pins for manual control buttons.
 */
void init_manual_control_pins() {
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << MANUAL_ALARM_PIN) | (1ULL << MANUAL_RESET_PIN);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);
}


// ============================
// --- RF CONTROL ---
// ============================
void save_new_code(unsigned long new_code);
void load_codes_from_nvs();
void delete_all_codes_from_nvs();
bool is_code_already_learned(unsigned long code_to_check);

/**
 * @brief Checks if a given RF code is already in the learned list.
 */
bool is_code_already_learned(unsigned long code_to_check) {
    for (int i = 0; i < num_learned_codes; i++) {
        if (learned_rf_codes[i] == code_to_check) return true;
    }
    return false;
}

/**
 * @brief Saves a new RF code to NVS if not already learned and space is available.
 */
void save_new_code(unsigned long new_code) {
    if (num_learned_codes >= MAX_RF_CODES) {
        ESP_LOGE(TAG, "Cannot learn new code, storage is full!");
        return;
    }
    if (is_code_already_learned(new_code)) {
        ESP_LOGW(TAG, "Code %lu has already been learned.", new_code);
        return;
    }

    nvs_handle_t my_handle;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &my_handle) != ESP_OK) {
        ESP_LOGE(TAG, "Error opening NVS handle!");
        return;
    }

    char key[20];
    snprintf(key, sizeof(key), "code_%d", num_learned_codes);

    if (nvs_set_u32(my_handle, key, new_code) == ESP_OK) {
        num_learned_codes++;
        if (nvs_set_i32(my_handle, "code_count", num_learned_codes) == ESP_OK) {
            nvs_commit(my_handle);
            ESP_LOGI(TAG, "Successfully saved new code %lu. Total codes: %d", new_code, num_learned_codes);
            load_codes_from_nvs(); // Refresh RAM array
        }
    }
    nvs_close(my_handle);
}

/**
 * @brief Loads all learned RF codes from NVS into the RAM array.
 */
void load_codes_from_nvs() {
    nvs_handle_t my_handle;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &my_handle) != ESP_OK) return;

    int32_t count = 0;
    nvs_get_i32(my_handle, "code_count", &count);
    num_learned_codes = count;

    if (num_learned_codes > 0) {
        ESP_LOGI(TAG, "Found %d learned RF codes. Loading...", num_learned_codes);
        for (int i = 0; i < num_learned_codes; i++) {
            char key[20];
            snprintf(key, sizeof(key), "code_%d", i);
            uint32_t temp_code = 0;
            nvs_get_u32(my_handle, key, &temp_code);
            learned_rf_codes[i] = temp_code;
            ESP_LOGI(TAG, "  -> Code %d: %lu", i, learned_rf_codes[i]);
        }
    } else {
        ESP_LOGI(TAG, "No learned RF codes found in NVS.");
    }
    nvs_close(my_handle);
}

/**
 * @brief Erases all learned RF codes from NVS and RAM.
 */
void delete_all_codes_from_nvs() {
    nvs_handle_t my_handle;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &my_handle) != ESP_OK) return;

    nvs_erase_all(my_handle); // Simpler than deleting one by one
    nvs_commit(my_handle);
    nvs_close(my_handle);

    memset(learned_rf_codes, 0, sizeof(learned_rf_codes));
    num_learned_codes = 0;
    ESP_LOGW(TAG, "DELETED ALL LEARNED RF CODES!");
}


// ============================
// --- FLAME SENSOR ---
// ============================

/**
 * @brief Event handler called by the flame sensor library when a sensor changes state.
 */
void flame_sensor_event_handler(int sensor_index, bool is_flame_detected)
{
    g_flame_sensor_states[sensor_index] = is_flame_detected;

    int active_sensors = 0;
    for (int i = 0; i < NUM_FLAME_SENSORS; i++) {
        if (g_flame_sensor_states[i] == true) {
            active_sensors++;
        }
    }

    bool new_consensus_state = (active_sensors >= FLAME_ALARM_THRESHOLD);

    if (new_consensus_state != g_flame_consensus_fire_state) {
        g_flame_consensus_fire_state = new_consensus_state;
        if (g_flame_consensus_fire_state) {
            ESP_LOGE(TAG, "FLAME ALARM: ON (Consensus from %d sensors)", active_sensors);
        } else {
            ESP_LOGI(TAG, "FLAME ALARM: OFF (Below threshold)");
        }
        update_and_propagate_alarm_state();
    }
}


// ============================
// --- ALARM CONTROL LOGIC ---
// ============================

/**
 * @brief Sends a fire alert status to the peer device via ESP-NOW.
 */
static void send_fire_alert_espnow(uint8_t fire_flag) {
    if (fire_flag == last_cmd_sent_espnow) return;
    espnow_payload_t tx_payload = {.cmd = fire_flag};
    if (esp_now_send(PEER_MAC, (uint8_t*)&tx_payload, sizeof(tx_payload)) == ESP_OK) {
        ESP_LOGI(TAG, "Sent ESP-NOW message: {cmd: %d}", fire_flag);
        last_cmd_sent_espnow = fire_flag;
    } else {
        ESP_LOGE(TAG, "Failed to send ESP-NOW message.");
    }
}

/**
 * @brief Central function to update the entire system's alarm state.
 *
 * This function is the core of the alarm logic. It should be called WHENEVER
 * an alarm source changes its state. It recalculates the combined local and
 * global alarm states and propagates changes via ESP-NOW and MQTT.
 */
static void update_and_propagate_alarm_state(void)
{
    bool should_publish_on = false;
    bool should_publish_off = false;

    if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE)
    {
        // Step 1: Recalculate the COMBINED LOCAL fire state from all sources
        bool new_combined_local_state = g_temp_gas_fire_state || g_flame_consensus_fire_state || g_rf_triggered_fire_state || g_manual_triggered_fire_state;

        // Step 2: If the combined local state has changed, update it and send ESP-NOW
        if (new_combined_local_state != sensor_data.combined_local_fire) {
            sensor_data.combined_local_fire = new_combined_local_state;
            send_fire_alert_espnow(sensor_data.combined_local_fire ? 1 : 0);
        }

        // Step 3: Recalculate the GLOBAL alarm state (local OR remote)
        bool is_global_fire_active = sensor_data.combined_local_fire || sensor_data.remote_fire;
        bool previous_alarm_state = alarm_on_state;

        // Step 4: Detect transition in global state and flag MQTT for publishing
        if (is_global_fire_active && !previous_alarm_state) {
            alarm_on_state = true;
            should_publish_on = true;
            ESP_LOGW(TAG, "🔥 ALARM ACTIVATED! Global fire state is now ON.");
        } else if (!is_global_fire_active && previous_alarm_state) {
            alarm_on_state = false;
            should_publish_off = true;
            ESP_LOGI(TAG, "✅ ALARM DEACTIVATED. Global fire state is now OFF.");
        }
        
        xSemaphoreGive(data_mutex);
    }

    // Step 5: Publish to MQTT if the state changed and client is connected
    if ((should_publish_on || should_publish_off) && mqtt_connected) {
        char msg[64];
        snprintf(msg, sizeof(msg), "{\"alert\":%s}", should_publish_on ? "true" : "false");
        esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC_FIRE, msg, 0, 1, 0);
    }
}

// ============================
// --- ESP-NOW ---
// ============================

static void espnow_recv_cb(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    if (memcmp(info->src_addr, s_local_mac, 6) == 0) return;
    if (len < sizeof(espnow_payload_t)) return;

    const espnow_payload_t *rx_payload = (const espnow_payload_t*)data;
    bool new_remote_fire_state = (rx_payload->cmd == 1);
    ESP_LOGI(TAG, "ESP-NOW alert received from peer. Remote fire state: %s", new_remote_fire_state ? "ON" : "OFF");

    if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE) {
        sensor_data.remote_fire = new_remote_fire_state;
        xSemaphoreGive(data_mutex);
    }
    update_and_propagate_alarm_state();
}

static esp_err_t espnow_init_and_setup(void) {
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));

    esp_now_peer_info_t peer = {0};
    memcpy(peer.peer_addr, PEER_MAC, 6);
    peer.ifidx = ESP_IF_WIFI_STA;
    peer.encrypt = false;
    
    if (esp_now_add_peer(&peer) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add ESP-NOW peer");
        return ESP_FAIL;
    }
    return ESP_OK;
}

// ============================
// --- MQTT & WIFI ---
// ============================

static void mqtt_event_handler(void *args, esp_event_base_t base, int32_t event_id, void *event_data) {
    if ((esp_mqtt_event_id_t)event_id == MQTT_EVENT_CONNECTED) {
        mqtt_connected = true;
        ESP_LOGI(TAG, "MQTT client connected.");
    } else if ((esp_mqtt_event_id_t)event_id == MQTT_EVENT_DISCONNECTED) {
        mqtt_connected = false;
        ESP_LOGW(TAG, "MQTT client disconnected.");
    }
}

static void mqtt_app_init(void) {
    const esp_mqtt_client_config_t mqtt_cfg = { .broker.address.uri = MQTT_BROKER_URI };
    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
}

static void wifi_event_handler(void *arg, esp_event_base_t base, int32_t event_id, void *data) {
    if (event_id == WIFI_EVENT_STA_START || event_id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();
    } else if (event_id == IP_EVENT_STA_GOT_IP) {
        ESP_LOGI(TAG, "Wi-Fi connected. Starting MQTT client.");
        esp_mqtt_client_start(mqtt_client);
    }
}

static void wifi_init_sta(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));
    wifi_config_t wifi_cfg = { .sta = { .ssid = WIFI_SSID, .password = WIFI_PASS, .threshold.authmode = WIFI_AUTH_WPA2_PSK }};
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());
}

// ============================
// --- TASKS ---
// ============================

/**
 * @brief Task for temp/gas sensors.
 */
void temp_gas_sensor_task(void *pvParameters)
{
    while (1)
    {
        float temp = ds18b20_read_temp();
        int gas = mq2_read_value();
        
        bool current_temp_gas_state = (temp > FIRE_THRESHOLD_C) || (gas > GAS_THRESHOLD_LIGHT);

        if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE) {
            sensor_data.temperature = temp;
            sensor_data.gas_level = gas;
            xSemaphoreGive(data_mutex);
        }

        if (current_temp_gas_state != g_temp_gas_fire_state) {
            g_temp_gas_fire_state = current_temp_gas_state;
            ESP_LOGW(TAG, "Temp/Gas sensor state changed to: %s", g_temp_gas_fire_state ? "DETECTED" : "CLEARED");
            update_and_propagate_alarm_state();
        }
        
        vTaskDelay(pdMS_TO_TICKS(SENSOR_POLL_INTERVAL_MS));
    }
}

/**
 * @brief Task to handle RF remote control logic (learning, deleting, receiving).
 */
void rf_control_task(void *pvParameters) {
    load_codes_from_nvs();

    while (1) {
        if (gpio_get_level(LEARN_BUTTON_PIN) == 0) {
            is_learning_mode = true;
            vTaskDelay(pdMS_TO_TICKS(500)); // Debounce
        }

        if (gpio_get_level(DELETE_BUTTON_PIN) == 0) {
            delete_all_codes_from_nvs();
            if (g_rf_triggered_fire_state) { // If RF alarm was on, turn it off and update
                g_rf_triggered_fire_state = false;
                update_and_propagate_alarm_state();
            }
            vTaskDelay(pdMS_TO_TICKS(500)); // Debounce
        }

        if (available(&rf_receiver)) {
            unsigned long received_code = getReceivedValue(&rf_receiver);
            ESP_LOGI(TAG, "Received RF code: %lu", received_code);

            if (is_learning_mode) {
                save_new_code(received_code);
                is_learning_mode = false;
            } else {
                if (is_code_already_learned(received_code)) {
                    ESP_LOGI(TAG, "Matching RF code found! Toggling RF alarm state.");
                    g_rf_triggered_fire_state = !g_rf_triggered_fire_state;
                    update_and_propagate_alarm_state();
                }
            }
            resetAvailable(&rf_receiver);
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/**
 * @brief Task to handle manual alarm trigger and reset buttons.
 */
void manual_control_task(void *pvParameters)
{
    while (1)
    {
        // --- Check for manual alarm trigger ---
        if (gpio_get_level(MANUAL_ALARM_PIN) == 0) {
            vTaskDelay(pdMS_TO_TICKS(20)); // Debounce delay
            if (gpio_get_level(MANUAL_ALARM_PIN) == 0) { // Check again
                if (!g_manual_triggered_fire_state) {
                    ESP_LOGW(TAG, "MANUAL ALARM TRIGGERED!");
                    g_manual_triggered_fire_state = true;
                    update_and_propagate_alarm_state();
                }
                while(gpio_get_level(MANUAL_ALARM_PIN) == 0) { // Wait for button release
                    vTaskDelay(pdMS_TO_TICKS(50));
                }
            }
        }

        // --- Check for manual reset ---
        if (gpio_get_level(MANUAL_RESET_PIN) == 0) {
            vTaskDelay(pdMS_TO_TICKS(20)); // Debounce delay
            if (gpio_get_level(MANUAL_RESET_PIN) == 0) {
                ESP_LOGW(TAG, "MANUAL RESET ACTIVATED! Clearing all local and remote alarm states.");
                // Reset all individual fire source flags
                g_temp_gas_fire_state = false;
                g_flame_consensus_fire_state = false;
                g_rf_triggered_fire_state = false;
                g_manual_triggered_fire_state = false;

                // Also clear the remote fire state for an immediate local reset
                if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE) {
                    sensor_data.remote_fire = false;
                    xSemaphoreGive(data_mutex);
                }

                update_and_propagate_alarm_state(); // Propagate the "all-clear" signal
                
                while(gpio_get_level(MANUAL_RESET_PIN) == 0) { // Wait for button release
                    vTaskDelay(pdMS_TO_TICKS(50));
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100)); // Poll buttons every 100ms
    }
}


/**
 * @brief Task: Controls the onboard LED based on the global alarm state.
 */
void led_control_task(void *pvParameters)
{
    while (1) {
        bool is_alarm_on_local = false;
        if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE) {
            is_alarm_on_local = alarm_on_state;
            xSemaphoreGive(data_mutex);
        }
        gpio_set_level(LED_PIN, is_alarm_on_local ? 1 : 0);
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

/**
 * @brief Task: Periodically publishes detailed sensor data to MQTT and prints a status log.
 */
void data_publish_task(void *pv) {
    char msg[512];
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000)); // Wait for 5 seconds

        // FIX: Initialize variables with default values to prevent compiler warnings.
        float current_temp = 0.0f;
        int current_gas = 0;
        bool is_global_alert_active = false;
        bool is_gas_high = false;
        const char* device_id = "TU_1_NHABEP"; // Device ID for logging

        if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE) {
            current_temp = sensor_data.temperature;
            current_gas = sensor_data.gas_level;
            is_gas_high = (current_gas > GAS_THRESHOLD_LIGHT);
            is_global_alert_active = alarm_on_state;
            xSemaphoreGive(data_mutex);
        }

        // --- NEW: Build and print the consolidated status log ---
        char flame_status_str[50];
        int offset = snprintf(flame_status_str, sizeof(flame_status_str), "Flame:[");
        for (int i = 0; i < NUM_FLAME_SENSORS; i++) {
            offset += snprintf(flame_status_str + offset, sizeof(flame_status_str) - offset, " %d", g_flame_sensor_states[i] ? 1 : 0);
        }
        snprintf(flame_status_str + offset, sizeof(flame_status_str) - offset, " ]");

        ESP_LOGI(STATUS_TAG, "Nhiệt độ: %.1f°C | Gas: %d | %s | RF: %-10s | ==> BÁO ĐỘNG: %s",
             current_temp,
             current_gas,
             flame_status_str,
             is_learning_mode ? "Học Lệnh" : "Hoạt Động",
             is_global_alert_active ? "CÓ CHÁY!" : "AN TOÀN");

        // --- Publish detailed data to MQTT ---
        if (mqtt_connected) {
           snprintf(msg, sizeof(msg),
                     "{\"id_thiet_bi\":\"%s\",\"nhiet_do\":%.2f,\"khi_ga\":%s,\"lua\":%s}",
                     
                     // Biến 1: device_id (chuỗi) cho "id_thiet_bi"
                     device_id,
                     
                     // Biến 2: current_temp (float) cho "nhiet_do"
                     current_temp,
                     
                     // Biến 3: "cao" hoặc "thap" (chuỗi) cho "khi_ga"
                     is_gas_high ? "\"cao\"" : "\"thap\"",
                     
                     // Biến 4: true hoặc false (boolean) cho "lua"
                     is_global_alert_active ? "true" : "false");

            esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC_DATA, msg, 0, 1, 0);
        }
    }
}


// ============================
// --- APP MAIN ---
// ============================
void app_main(void) {
    // --- Initialize Core System Services ---
    init_nvs();
    data_mutex = xSemaphoreCreateMutex();
    
    // --- Initialize Peripherals ---
    gpio_reset_pin(LED_PIN);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_PIN, 0);
    init_rf_control_pins();
    init_manual_control_pins();

    // --- Initialize Network and Protocols ---
    wifi_init_sta();
    mqtt_app_init();
    espnow_init_and_setup();
    
    // --- Initialize and Calibrate Sensors ---
    // RF Receiver
    initSwich(&rf_receiver);
    enableReceive(&rf_receiver, RF_RECEIVER_PIN);
    // MQ2 Gas Sensor
    mq2_init();
    ESP_LOGW(TAG, "--- Calibrating MQ2 Sensor (ensure clean air)... ---");
    mq2_calibrate();
    ESP_LOGI(TAG, "--- MQ2 Calibration complete.");
    // Flame Sensor Array
    memset(g_flame_sensor_states, false, sizeof(g_flame_sensor_states));
    flame_sensor_init(FLAME_SENSOR_PINS, NUM_FLAME_SENSORS, &flame_sensor_event_handler);

    // --- Create Application Tasks ---
    xTaskCreate(temp_gas_sensor_task, "temp_gas_task", 4096, NULL, 5, NULL);
    xTaskCreate(rf_control_task, "rf_control_task", 4096, NULL, 6, NULL);
    xTaskCreate(manual_control_task, "manual_control_task", 2048, NULL, 7, NULL); // Task for manual buttons
    xTaskCreate(led_control_task, "led_control_task", 2048, NULL, 4, NULL);
    xTaskCreate(data_publish_task, "data_publish_task", 4096, NULL, 3, NULL);
    
    ESP_LOGI(TAG, "System initialization complete. Tasks are running.");
}

