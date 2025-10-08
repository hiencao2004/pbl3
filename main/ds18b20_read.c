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
// --- CONFIG ---
// ============================

#define LED_PIN     GPIO_NUM_2
#define WIFI_SSID          "Hieu Nguyen"
#define WIFI_PASS          "123456789@@"
#define MQTT_BROKER_URI    "mqtt://mqtt.pbl3.click"
#define MQTT_TOPIC_DATA    "Data"
#define MQTT_TOPIC_FIRE    "fire1"

#define FIRE_THRESHOLD_C   31.2f
#define GAS_THRESHOLD_LIGHT  0   // phát hiện khí gas nhẹ
#define GAS_THRESHOLD_STRONG 80   // rò rỉ khí gas mạnh

#define SENSOR_POLL_MS     2000

static const char *TAG = "GATEWAY_FIRE_SYSTEM";
static const uint8_t PEER_MAC[6] = {0xA0, 0xA3, 0xB3, 0xA9, 0xE9, 0x34};
static uint8_t s_local_mac[6] = {0x78, 0x1C, 0x3C, 0x2B, 0xC5, 0x64}; //local mac
static esp_mqtt_client_handle_t mqtt_client = NULL;
static bool mqtt_connected = false;
static bool alarm_on_state = false;
static uint8_t last_cmd_sent = 0xFF;

// ============================
// --- STRUCTS & GLOBALS ---
// ============================
typedef struct {
    float temperature;
    int gas_level;
    bool local_fire;
    bool remote_fire;
} sensor_state_t;

typedef struct __attribute__((packed)) {
    uint8_t cmd;        // 0 = Safe, 1 = Fire detected
} payload_t;

typedef enum {
    FIRE_EVENT_OFF = 0,   // Tắt cảnh báo
    FIRE_EVENT_ON         // Bật cảnh báo
} fire_event_t;

static sensor_state_t sensor_data;

static SemaphoreHandle_t fire_mutex;



// ============================
// --- ALARM CONTROL ---
// ============================
static void update_global_alarm_state(void) 
{
    bool current_global_fire = false;
    bool should_publish_on = false;
    bool should_publish_off = false;
    bool old_alarm_state = false;

    // 1. Kiểm tra trạng thái toàn cục (an toàn với Mutex)
    xSemaphoreTake(fire_mutex, portMAX_DELAY);
    old_alarm_state = alarm_on_state;
    current_global_fire = sensor_data.local_fire || sensor_data.remote_fire;

    // 2. Chuyển trạng thái
    if (current_global_fire && !old_alarm_state) {
        // Chuyển từ OFF -> ON
        alarm_on_state = true;
        should_publish_on = true;
        
        // Xác định nguồn kích hoạt

        ESP_LOGW(TAG, "🔥 ALARM ON (First alert )");

    } else if (!current_global_fire && old_alarm_state) {
        // Chuyển từ ON -> OFF
        alarm_on_state = false;
        should_publish_off = true;
        ESP_LOGI(TAG, "✅ ALARM OFF (GLOBAL clear)");
    }
    
    xSemaphoreGive(fire_mutex);
    // 3. Publish MQTT
    if ((should_publish_on || should_publish_off) && mqtt_connected) {
        char msg[64];
        if (should_publish_on) {
            // Gửi kèm nguồn kích hoạt
            snprintf(msg, sizeof(msg), "{\"alert\":true}");
            esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC_FIRE, msg, 0, 1, 0);
        } else {
            // Khi tắt toàn cục, chỉ cần báo trạng thái chung là OFF
            snprintf(msg, sizeof(msg), "{\"alert\":false}"); 
            esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC_FIRE, msg, 0, 1, 0);
        }
        
    }
}

// ============================
// --- ESP-NOW ---
// ============================
static void espnow_recv_cb(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
        // << NEW: LỌC GÓI TIN TỰ GỬI >>
    if (memcmp(info->src_addr, s_local_mac, 6) == 0) {
        ESP_LOGD(TAG, "Self-sent packet filtered.");
        return;
    }
    
    if (len < sizeof(payload_t)) return;

    const payload_t *rx = (const payload_t*)data;
    bool new_remote_fire = (rx->cmd == 1);

    xSemaphoreTake(fire_mutex, portMAX_DELAY);
    sensor_data.remote_fire = new_remote_fire;
    xSemaphoreGive(fire_mutex);

    update_global_alarm_state();
}


static void send_fire_alert_espnow(uint8_t flag) {
    static payload_t tx_data = {0};
    if (flag == last_cmd_sent) return;
    tx_data.cmd = flag;
    if (esp_now_send(PEER_MAC, (uint8_t*)&tx_data, sizeof(tx_data)) == ESP_OK)
        last_cmd_sent = flag;
}

static esp_err_t espnow_init_and_setup(void) {
    ESP_ERROR_CHECK(esp_now_init());
    esp_now_register_recv_cb(espnow_recv_cb);


    esp_now_peer_info_t peer = {0};
    memcpy(peer.peer_addr, PEER_MAC, 6);
    peer.ifidx = ESP_IF_WIFI_STA;
    peer.encrypt = false;
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_now_add_peer(&peer));
    return ESP_OK;
}

// ============================
// --- MQTT ---
// ============================
static void mqtt_event_handler(void *args, esp_event_base_t base, int32_t id, void *data) {
    // esp_mqtt_event_handle_t event = data;
    switch (id) {
        case MQTT_EVENT_CONNECTED:
            mqtt_connected = true;
            ESP_LOGI(TAG, "MQTT connected");
            break;
        case MQTT_EVENT_DISCONNECTED:
            mqtt_connected = false;
            ESP_LOGW(TAG, "MQTT disconnected");
            break;
        default: break;
    }
}
static void mqtt_app_init(void) {
    const esp_mqtt_client_config_t cfg = {
        .broker.address.uri = MQTT_BROKER_URI,
    };
    mqtt_client = esp_mqtt_client_init(&cfg);
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
}

// ============================
// --- WIFI ---
// ============================
static void wifi_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data) {
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ESP_LOGI(TAG, "Wi-Fi connected, starting MQTT");
        esp_mqtt_client_start(mqtt_client);
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();
    }
}
static void wifi_init_sta(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_cfg = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    espnow_init_and_setup();
}
// ============================
// --- TASK: Sensor read + event management ---
//  - read DS18B20 & MQ2
//  - set/clear event bits
//  - when change in alert state -> send espnow + publish fire msg
// ============================
void task_sensor_and_event(void *pvParameters)
{
    bool last_local_fire_state = false;
    while (1)
    {
        float temp = ds18b20_read_temp();
        int gas = mq2_read_value();
        // bool flame = flame_read();
        // bool rf = rf_sensor_read();

        bool over_temp = (temp > FIRE_THRESHOLD_C);
        bool gas_alarm = (gas > GAS_THRESHOLD_LIGHT);
        bool local_fire = over_temp || gas_alarm;  //|| flame || rf;

         ESP_LOGW("DEBUG_STEP", "------------------------------------");
        ESP_LOGI("DEBUG_STEP", "Gia tri Gas doc duoc (A): %d", gas);
        ESP_LOGI("DEBUG_STEP", "Nguong canh bao (B): 0");
         ESP_LOGI("DEBUG_STEP", "Ket qua so sanh (A > B): %s", gas_alarm ? "DUNG (TRUE)" : "SAI (FALSE)");
         ESP_LOGI("DEBUG_STEP", "Trang thai local_fire cuoi cung: %s", local_fire ? "CO CHAY (TRUE)" : "AN TOAN (FALSE)");
        ESP_LOGW("DEBUG_STEP", "------------------------------------");
        xSemaphoreTake(fire_mutex, portMAX_DELAY);
        sensor_data.temperature = temp;
        sensor_data.gas_level = gas;
        // fire_state.flame_detected = flame;
        // fire_state.rf_detected = rf;
        sensor_data.local_fire = local_fire;
        xSemaphoreGive(fire_mutex);

        if (local_fire != last_local_fire_state) {  
            // 1. Gửi ESP-NOW cho Peer
            send_fire_alert_espnow(local_fire ? 1 : 0);
            
            // 2. Cập nhật trạng thái toàn cục (LED/MQTT)
            update_global_alarm_state(); 
            last_local_fire_state = local_fire;
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}


void task_led_control(void *pvParameters)
{
    bool is_alarm_on;
    while (1) {
        xSemaphoreTake(fire_mutex, portMAX_DELAY);
        is_alarm_on = alarm_on_state; // Đọc được bảo vệ
        xSemaphoreGive(fire_mutex);
        if (is_alarm_on) {
            gpio_set_level(LED_PIN, 1);
        } else {
            // Tắt LED và chờ đợi
            gpio_set_level(LED_PIN, 0);
            vTaskDelay(pdMS_TO_TICKS(500)); // Delay để giảm tải CPU khi báo động OFF
        }
    }
}

void data_task(void *pv) {
    char msg[128];
    float last_temp = -999.0f;
    int last_gas = -999;
    bool last_fire = false;
    while (1) {
        xSemaphoreTake(fire_mutex, portMAX_DELAY);
        float temperature = sensor_data.temperature;
        int gas_level = sensor_data.gas_level;
        bool fire = sensor_data.local_fire || sensor_data.remote_fire;
        xSemaphoreGive(fire_mutex);

        if (fabs(temperature - last_temp) > 0.2 || abs(gas_level - last_gas) > 2) {
            time_t now = time(NULL);
            snprintf(msg, sizeof(msg),
                     "{\"timestamp\":%lld,\"temperature\":%.2f,\"gas\":%d,\"alert\":%s}",
                     (long long) now, temperature, gas_level,
                     fire ? "true" : "false");

            if (mqtt_connected)
                esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC_DATA, msg, 0, 1, 0);

            last_temp = temperature;
            last_gas = gas_level;
            last_fire = fire;
        }

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}


// ============================
// --- APP MAIN ---
// ============================
void app_main(void) {
    ESP_ERROR_CHECK(nvs_flash_init());
    fire_mutex = xSemaphoreCreateMutex();
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << DS18B20_GPIO_PIN,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = 1,
    };
    gpio_config(&io_conf);
        
    gpio_reset_pin(LED_PIN);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_PIN, 0);

    mqtt_app_init();     // init MQTT trước
    wifi_init_sta();     // rồi khởi động Wi-Fi
    mq2_init();
    ESP_LOGW(TAG, "--- BẮT ĐẦU HIỆU CHỈNH MQ2 (Giữ môi trường sạch) ---");
    mq2_calibrate(); // Hàm này sẽ block trong một thời gian ngắn để lấy giá trị nền
    ESP_LOGI(TAG, "--- HIỆU CHỈNH HOÀN TẤT. Ngưỡng cảnh báo: %d ---", GAS_THRESHOLD_LIGHT);
    xTaskCreate(task_sensor_and_event, "sensor_event", 4096, NULL, 6, NULL);
    xTaskCreate(data_task, "mqtt_pub", 4096, NULL, 5, NULL);
    xTaskCreate(task_led_control, "led_control", 2048, NULL, 4, NULL);
    ESP_LOGI(TAG, "System started");
}
