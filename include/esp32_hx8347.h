#ifndef _esp32_hx8347_H_
#define _esp32_hx8347_H_

#include "esp_check.h"

typedef struct esp32_hx8347_t *esp32_hx8347_handle_t;


typedef struct esp32_hx8347_config_t
{
    uint8_t teste; // HX8347 server address
    uint32_t teste2; // HX8347 server port
} esp32_hx8347_config_t;

esp_err_t esp32_hx8347_init(esp32_hx8347_config_t *config, esp32_hx8347_handle_t *handle);

void hx8347_lcd_write_register(esp32_hx8347_handle_t dev, uint8_t addr, uint16_t data);

void lcd_write_comm_byte(esp32_hx8347_handle_t dev, uint8_t cmd);

void lcd_write_data_byte(esp32_hx8347_handle_t dev, uint8_t data);

void send_pixels_to_hx8347(esp32_hx8347_handle_t dev, uint8_t *pixels, size_t len);

void func(void);



#endif
