#include <stdio.h>
#include "esp32_hx8347.h"

#include "lvgl.h"
#include "freertos/FreeRTOS.h"

#include "esp32_hx8347.h"

esp32_hx8347_handle_t hx8347_handle = NULL;

void app_main(void)
{

    esp32_hx8347_config_t hx8347_config = {
        .teste = 0x00, // Example value for HX8347 server address
        .teste2 = 1234 // Example value for HX8347 server port
    };
    esp_err_t ret = esp32_hx8347_init(&hx8347_config, &hx8347_handle);

    ESP_ERROR_CHECK(ret);
    
    esp32_hx8347_lvgl_lock(hx8347_handle);

    lv_obj_set_style_bg_color(esp32_hx8347_get_screen(hx8347_handle), lv_color_hex(0x000000), 0);
    lv_obj_set_style_bg_opa(esp32_hx8347_get_screen(hx8347_handle), LV_OPA_COVER, 0);

    lv_obj_t *label1 = lv_label_create(esp32_hx8347_get_screen(hx8347_handle));
    lv_label_set_text(label1, "BLUE");
    lv_obj_set_style_text_color(label1, lv_color_hex(0x0000FF), LV_PART_MAIN);
    lv_obj_set_style_text_font(label1, &lv_font_montserrat_24, LV_PART_MAIN);
    lv_obj_align(label1, LV_ALIGN_BOTTOM_MID, 0, 0);


    lv_obj_t *label2 = lv_label_create(esp32_hx8347_get_screen(hx8347_handle));
    lv_label_set_text(label2, "GREEN");
    lv_obj_set_style_text_color(label2, lv_color_hex(0x00FF00), LV_PART_MAIN);
    lv_obj_set_style_text_font(label2, &lv_font_montserrat_24, LV_PART_MAIN);
    lv_obj_align(label2, LV_ALIGN_TOP_MID, 0, 0);


    lv_obj_t *label3 = lv_label_create(esp32_hx8347_get_screen(hx8347_handle));
    lv_label_set_text(label3, "RED");
    lv_obj_set_style_text_color(label3, lv_color_hex(0xFF0000), LV_PART_MAIN);
    lv_obj_set_style_text_font(label3, &lv_font_montserrat_32, LV_PART_MAIN);
    lv_obj_align(label3, LV_ALIGN_CENTER, 0, 0);

    
    esp32_hx8347_lvgl_unlock(hx8347_handle);

    while (1)
    {

    
        vTaskDelay(1000 / portTICK_PERIOD_MS);

    }

    
}
