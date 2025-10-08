// flame_sensor.c

#include "flame_sensor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/gpio.h"
static const char *TAG = "FLAME_SENSOR_LIB";

// --- Các biến static chỉ được sử dụng nội bộ trong thư viện này ---
static QueueHandle_t gpio_evt_queue;
static flame_sensor_event_cb_t user_callback = NULL;
static const gpio_num_t* sensor_pins = NULL;
static int num_sensors = 0;
static bool *sensor_states = NULL; // Mảng trạng thái, sẽ được cấp phát động

/* ---------- ISR: chỉ đẩy pin vào queue ---------- */
static void IRAM_ATTR flame_isr_handler(void *arg) {
    uint32_t pin = (uint32_t)arg;
    xQueueSendFromISR(gpio_evt_queue, &pin, NULL);
}

/* ---------- Task xử lý sự kiện: debounce và gọi callback ---------- */
static void flame_sensor_task(void *arg) {
    const int DEBOUNCE_MS = 30;

    while (1) {
        uint32_t pin;
        if (xQueueReceive(gpio_evt_queue, &pin, portMAX_DELAY)) {
            // Tìm chỉ số (index) của pin
            int idx = -1;
            for (int i = 0; i < num_sensors; i++) {
                if (sensor_pins[i] == (gpio_num_t)pin) {
                    idx = i;
                    break;
                }
            }
            if (idx < 0) continue;

            // Debounce: chờ một chút rồi đọc lại
            vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_MS));
            bool fire = (gpio_get_level((gpio_num_t)pin) == 0); // 0 = có lửa

            // Nếu trạng thái thay đổi, cập nhật và gọi callback
            if (fire != sensor_states[idx]) {
                sensor_states[idx] = fire;
                if (user_callback != NULL) {
                    user_callback(idx, fire);
                }
            }
        }
    }
}

/* ---------- Hàm khởi tạo công khai ---------- */
esp_err_t flame_sensor_init(const gpio_num_t* pins, int count, flame_sensor_event_cb_t callback) {
    if (pins == NULL || count <= 0 || callback == NULL) {
        ESP_LOGE(TAG, "Invalid arguments");
        return ESP_ERR_INVALID_ARG;
    }

    sensor_pins = pins;
    num_sensors = count;
    user_callback = callback;

    // Cấp phát động cho mảng trạng thái
    sensor_states = (bool*)calloc(num_sensors, sizeof(bool));
    if (sensor_states == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for sensor states");
        return ESP_ERR_NO_MEM;
    }

    /* Config inputs */
    uint64_t pin_bit_mask = 0;
    for (int i = 0; i < num_sensors; i++) {
        pin_bit_mask |= (1ULL << sensor_pins[i]);
    }
    gpio_config_t in_cfg = {
        .pin_bit_mask = pin_bit_mask,
        .mode = GPIO_MODE_INPUT,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&in_cfg);

    /* Bật pull-up nội cho các chân có hỗ trợ */
    for (int i = 0; i < num_sensors; i++) {
        if (GPIO_IS_VALID_GPIO(sensor_pins[i]) && sensor_pins[i] < GPIO_NUM_34) {
             gpio_pullup_en(sensor_pins[i]);
        } else {
             ESP_LOGW(TAG, "GPIO %d does not have an internal pull-up. An external one is required.", sensor_pins[i]);
        }
    }

    /* Tạo queue và cài đặt ISR service */
    gpio_evt_queue = xQueueCreate(16, sizeof(uint32_t));
    gpio_install_isr_service(0);

    /* Thêm handler cho từng pin */
    for (int i = 0; i < num_sensors; i++) {
        gpio_set_intr_type(sensor_pins[i], GPIO_INTR_ANYEDGE);
        gpio_isr_handler_add(sensor_pins[i], flame_isr_handler, (void*)sensor_pins[i]);
    }

    /* Tạo task xử lý nền */
    xTaskCreate(flame_sensor_task, "flame_sensor_task", 2048, NULL, 10, NULL);

    ESP_LOGI(TAG, "Flame sensor library initialized for %d sensors.", num_sensors);
    return ESP_OK;
}