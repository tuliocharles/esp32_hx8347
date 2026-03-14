#include <stdio.h>
#include "esp32_hx8347.h"

#include "lvgl.h"
#include "freertos/FreeRTOS.h"

esp32_hx8347_handle_t hx8347_handle = NULL;


static void lv_tick_timer_cb(void *arg)
{
    (void)arg;
    lv_tick_inc(1); // informa que passou 1 ms
}

void lvgl_task(void *arg)
{

    lv_obj_t *scr = lv_scr_act();

    lv_obj_t *label = lv_label_create(lv_scr_act());
    lv_obj_set_width(label, 150);
    lv_label_set_long_mode(label, LV_LABEL_LONG_SCROLL_CIRCULAR);
    lv_label_set_text(label, "texto --------------------------------------------------------------------");
    lv_obj_center(label);

    lv_obj_set_style_bg_color(scr,
                              lv_color_hex(0x000000),
                              LV_PART_MAIN);
    lv_obj_set_style_bg_opa(scr,
                            LV_OPA_COVER,
                            LV_PART_MAIN);

    int cont = 0;
    char string_valor_cont[20];
    while (1)
    {
        lv_tick_inc(10); // informa LVGL que se passaram 10 ms
        lv_timer_handler();
        vTaskDelay(pdMS_TO_TICKS(10));
        cont++;
        snprintf(string_valor_cont, sizeof(string_valor_cont), "%d", cont);
        lv_label_set_text(label, string_valor_cont);
    }
}

static void my_flush_cb(lv_display_t *display,
                        const lv_area_t *area,
                        uint8_t *color_map)
{
    int x1 = area->x1;
    int y1 = area->y1;
    int x2 = area->x2;
    int y2 = area->y2;

    // Define janela no HX8347
    hx8347_lcd_write_register(hx8347_handle, 0x02, x1);
    hx8347_lcd_write_register(hx8347_handle, 0x04, x2);
    hx8347_lcd_write_register(hx8347_handle, 0x06, y1);
    hx8347_lcd_write_register(hx8347_handle, 0x08, y2);

    // Diz que vai escrever GRAM
    lcd_write_comm_byte(hx8347_handle, 0x22);

    // 3Envia pixels (RGB565)
    size_t size = (x2 - x1 + 1) *
                  (y2 - y1 + 1) * 2;

    send_pixels_to_hx8347(hx8347_handle, color_map, size);

    

    // 4️ Informa LVGL que terminou
    lv_display_flush_ready(display);
}



void app_main(void)
{
    
    esp32_hx8347_config_t hx8347_config = {
        .teste = 0x00, // Example value for HX8347 server address
        .teste2 = 1234 // Example value for HX8347 server port
    };
    esp_err_t ret = esp32_hx8347_init(&hx8347_config, &hx8347_handle);   
    
       // 3) Inicializa LVGL
    lv_init();

    // 4) Cria display LVGL
    lv_display_t *display = lv_display_create(240, 320);

    // 5) Cria buffers LVGL
    static lv_color_t buf1[240 * 20];
    static lv_color_t buf2[240 * 20];

    lv_display_set_buffers(display,
                           buf1,
                           buf2,
                           sizeof(buf1),
                           LV_DISPLAY_RENDER_MODE_PARTIAL);

    lv_display_set_color_format(display, LV_COLOR_FORMAT_RGB565);

    // 6) Conecta o flush
    lv_display_set_flush_cb(display, my_flush_cb);

    // 7) Cria tarefa LVGL
    xTaskCreate(lvgl_task, "lvgl", 4096, NULL, 5, NULL);
    
}
