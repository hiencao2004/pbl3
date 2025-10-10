
#include "flame_sensor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include <stdlib.h> // For malloc, free
#include <string.h> // For memcpy

static const char *TAG = "FLAME_SENSOR_LIB";

// Debounce time in milliseconds
#define DEBOUNCE_TIME_MS 50

// --- Cấu trúc dữ liệu nội bộ ---
typedef struct {
    gpio_num_t pin;
    bool current_state; // Trạng thái đã được debounce
    uint32_t last_intr_time; // Thời điểm xảy ra ngắt cuối cùng (dùng cho debounce)
} flame_sensor_info_t;

// --- Các biến static chỉ được sử dụng nội bộ trong thư viện này ---
static flame_sensor_info_t *sensors_info = NULL; // Mảng thông tin các cảm biến
static int num_sensors = 0;
static flame_sensor_event_cb_t user_callback = NULL;
static QueueHandle_t sensor_evt_queue = NULL;
static TaskHandle_t sensor_task_handle = NULL;

/* ---------- ISR: chỉ đẩy INDEX của cảm biến vào queue ---------- */
// Rất nhanh và hiệu quả, không tìm kiếm gì trong task nữa
static void IRAM_ATTR flame_isr_handler(void *arg) {
    int sensor_index = (int)arg;
    xQueueSendFromISR(sensor_evt_queue, &sensor_index, NULL);
}

/* ---------- Task xử lý sự kiện: debounce không chặn và gọi callback ---------- */
static void flame_sensor_task(void *arg) {
    int sensor_index;
    while (1) {
        if (xQueueReceive(sensor_evt_queue, &sensor_index, portMAX_DELAY)) {
            // **Logic Debounce không chặn (non-blocking)**
            TickType_t now_ticks = xTaskGetTickCount();
            if ((now_ticks - sensors_info[sensor_index].last_intr_time) * portTICK_PERIOD_MS < DEBOUNCE_TIME_MS) {
                // Nhiễu, bỏ qua sự kiện này
                continue;
            }
            sensors_info[sensor_index].last_intr_time = now_ticks;

            // Đọc trạng thái thực tế của pin (0 = có lửa)
            bool is_flame_detected = (gpio_get_level(sensors_info[sensor_index].pin) == 0);

            // Nếu trạng thái thay đổi so với trạng thái đã lưu, cập nhật và gọi callback
            if (is_flame_detected != sensors_info[sensor_index].current_state) {
                sensors_info[sensor_index].current_state = is_flame_detected;
                ESP_LOGI(TAG, "Sensor index %d (GPIO %d) state changed: %s",
                         sensor_index, sensors_info[sensor_index].pin,
                         is_flame_detected ? "FLAME DETECTED" : "NO FLAME");

                if (user_callback != NULL) {
                    user_callback(sensor_index, is_flame_detected);
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

    // **[Cải tiến] Ngăn khởi tạo lại để tránh rò rỉ bộ nhớ**
    if (sensors_info != NULL) {
        ESP_LOGE(TAG, "Library already initialized.");
        return ESP_FAIL;
    }

    num_sensors = count;
    user_callback = callback;

    // **[Cải tiến] Cấp phát và sao chép mảng pin để đảm bảo an toàn bộ nhớ**
    sensors_info = (flame_sensor_info_t*)malloc(num_sensors * sizeof(flame_sensor_info_t));
    if (sensors_info == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for sensors info");
        return ESP_ERR_NO_MEM;
    }

    // Tạo queue và cài đặt ISR service
    sensor_evt_queue = xQueueCreate(10, sizeof(int)); // Queue chứa index (int)
    gpio_install_isr_service(0);

    // Cấu hình từng pin
    for (int i = 0; i < num_sensors; i++) {
        sensors_info[i].pin = pins[i];
        sensors_info[i].last_intr_time = 0;

        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << pins[i]),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_ANYEDGE // Ngắt cả 2 cạnh
        };
        gpio_config(&io_conf);
        
        // Đọc và lưu trạng thái ban đầu
        sensors_info[i].current_state = (gpio_get_level(pins[i]) == 0);

        // **[Cải tiến] Thêm ISR handler, truyền vào INDEX thay vì PIN**
        gpio_isr_handler_add(pins[i], flame_isr_handler, (void*)i);
    }

    // Tạo task xử lý nền
    xTaskCreate(flame_sensor_task, "flame_sensor_task", 2048, NULL, 10, &sensor_task_handle);

    ESP_LOGI(TAG, "Flame sensor library initialized for %d sensors.", num_sensors);
    return ESP_OK;
}