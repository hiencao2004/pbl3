// mq2_sensor.h
#ifndef MQ2_SENSOR_H
#define MQ2_SENSOR_H

/**
 * @brief Khởi tạo ADC cho cảm biến MQ2.
 */
void mq2_init(void);

/**
 * @brief Hiệu chỉnh giá trị baseline cho cảm biến trong môi trường không khí sạch.
 * Hàm này sẽ block trong vài giây.
 */
void mq2_calibrate(void);

/**
 * @brief Đọc và xử lý giá trị từ cảm biến MQ2.
 *
 * @return int Giá trị nồng độ khí gas đã được xử lý (đã trừ baseline).
 * Trả về 0 nếu không phát hiện khí gas.
 */
int mq2_read_value(void);

#endif // MQ2_SENSOR_H