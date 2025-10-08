#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "ds18b20.h"
#include "esp_mac.h"

static const char *TAG = "DS18B20_SENSOR";

 void ow_output_low(void) { gpio_set_direction(DS18B20_GPIO_PIN, GPIO_MODE_OUTPUT); gpio_set_level(DS18B20_GPIO_PIN, 0); }
 void ow_input(void)      { gpio_set_direction(DS18B20_GPIO_PIN, GPIO_MODE_INPUT); }
 int  ow_read(void)       { return gpio_get_level(DS18B20_GPIO_PIN); }

 int ow_reset(void) {
    ow_output_low(); esp_rom_delay_us(480);
    ow_input(); esp_rom_delay_us(70);
    int presence = ow_read();
    esp_rom_delay_us(410);
    return (presence == 0) ? 0 : -1;
}
 void ow_write_bit(int bit) {
    ow_output_low(); esp_rom_delay_us(2);
    if (bit) { ow_input(); esp_rom_delay_us(60); }
    else     { esp_rom_delay_us(60); ow_input(); }
    esp_rom_delay_us(1); // Thời gian phục hồi tối thiểu 1us
}   
 int ow_read_bit(void) {
    ow_output_low(); esp_rom_delay_us(2);
    ow_input(); esp_rom_delay_us(10);
    int b = ow_read(); esp_rom_delay_us(50);
    return b;
}
 void ow_write_byte(uint8_t b) { for (int i=0;i<8;i++){ ow_write_bit(b&1); b>>=1; } }
 uint8_t ow_read_byte(void) { uint8_t b=0; for (int i=0;i<8;i++){ b|=(ow_read_bit()<<i);} return b; }
 float ds18b20_read_temp(void) {
    if (ow_reset() != 0) return -99.9f;
    ow_write_byte(0xCC); ow_write_byte(0x44); // Convert T
    vTaskDelay(pdMS_TO_TICKS(750));

    if (ow_reset() != 0) return -99.9f;
    ow_write_byte(0xCC); ow_write_byte(0xBE); // Read scratchpad

    uint8_t data[9];
    for (int i=0;i<9;i++) data[i] = ow_read_byte();
    int16_t raw = (data[1]<<8) | data[0];
    ESP_LOGI(TAG, "Temp=%d", raw/16.0);
    return raw / 16.0f;
}