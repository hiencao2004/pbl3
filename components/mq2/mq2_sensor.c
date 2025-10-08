// mq2_sensor.c
#include "mq2_sensor.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc/adc_oneshot.h"

// ADC channel for ESP32-C3: ADC_CHANNEL_0 corresponds to GPIO0
#define MQ2_CHANNEL ADC_CHANNEL_0

static const char *TAG = "MQ2_SENSOR";

// Các biến nội bộ, được giấu đi khỏi app_main.c
static adc_oneshot_unit_handle_t adc_handle = NULL;
static int baseline = 0;

void mq2_init(void)
{
    adc_oneshot_unit_init_cfg_t init_cfg = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg, &adc_handle));

    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = ADC_ATTEN_DB_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, MQ2_CHANNEL, &chan_cfg));

    ESP_LOGI(TAG, "MQ2 ADC initialized");
}

void mq2_calibrate(void)
{
    // BƯỚC 1: Thêm thời gian chờ làm nóng
    ESP_LOGW(TAG, "⚠️ Warming up MQ-2 heater for %d seconds...", 30000 / 1000);
    vTaskDelay(pdMS_TO_TICKS(30000)); // Chờ 30 giây
    ESP_LOGI(TAG, "Calibrating baseline... keep area clear of gas");
    long sum = 0;
    const int samples = 100;
    for (int i = 0; i < samples; ++i) {
        int raw = 0;
        ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, MQ2_CHANNEL, &raw));
        sum += raw;
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    baseline = sum / samples;
    ESP_LOGI(TAG, "Calibration complete. Baseline=%d", baseline);

}

int mq2_read_value(void)
{
    const int avg_samples = 20;
    const int margin = 20; // Ngưỡng sai số để xác định có khí hay chưa
    long sum = 0;

    for (int i = 0; i < avg_samples; ++i) {
        int raw = 0;
        ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, MQ2_CHANNEL, &raw));
        sum += raw;
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    int rawValue = sum / avg_samples;
    int gasValue = 0;

    // Nếu vượt baseline + margin thì coi là có khí
    if (rawValue > baseline + margin) {
        gasValue = rawValue - baseline;
    } else {
        gasValue = 0; // Không có khí
    }

    // In log để debug chính xác sự thay đổi
    ESP_LOGI(TAG, "Raw=%d, Baseline=%d, Diff=%d", rawValue, baseline, rawValue - baseline);

    return gasValue;
}