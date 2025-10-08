// flame_sensor.h

#ifndef FLAME_SENSOR_H
#define FLAME_SENSOR_H

#include "driver/gpio.h"
#include <stdbool.h>

/**
 * @brief Định nghĩa kiểu con trỏ hàm callback cho sự kiện cảm biến lửa.
 * * @param sensor_index Chỉ số của cảm biến trong mảng lúc khởi tạo (bắt đầu từ 0).
 * @param is_flame_detected Trạng thái mới của cảm biến (true: có lửa, false: không có lửa).
 */
typedef void (*flame_sensor_event_cb_t)(int sensor_index, bool is_flame_detected);

/**
 * @brief Khởi tạo thư viện cảm biến lửa.
 * * Hàm này sẽ cấu hình các chân GPIO, cài đặt ngắt, tạo task xử lý nền
 * để phát hiện và debounce tín hiệu từ các cảm biến.
 * * @param pins Mảng các chân GPIO kết nối với cảm biến.
 * @param num_sensors Số lượng cảm biến trong mảng.
 * @param callback Hàm sẽ được gọi khi có sự thay đổi trạng thái (đã debounce) của một cảm biến.
 * @return esp_err_t ESP_OK nếu thành công, ngược lại là mã lỗi.
 */
esp_err_t flame_sensor_init(const gpio_num_t* pins, int num_sensors, flame_sensor_event_cb_t callback);

#endif // FLAME_SENSOR_H