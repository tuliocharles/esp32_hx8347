#ifndef _esp32_hx8347_H_
#define _esp32_hx8347_H_

#include "esp_check.h"
#include "lvgl.h"

typedef struct esp32_hx8347_t *esp32_hx8347_handle_t;

typedef struct esp32_hx8347_config_t
{
    uint8_t teste; // HX8347 server address
    uint32_t teste2; // HX8347 server port
} esp32_hx8347_config_t;

esp_err_t esp32_hx8347_init(esp32_hx8347_config_t *config, esp32_hx8347_handle_t *handle);

lv_obj_t *esp32_hx8347_get_screen(esp32_hx8347_handle_t handle);

void esp32_hx8347_lvgl_lock(esp32_hx8347_handle_t handle);

void esp32_hx8347_lvgl_unlock(esp32_hx8347_handle_t handle);

#endif
