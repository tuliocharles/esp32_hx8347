#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <assert.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"

#include "driver/gpio.h"

#include "esp_lcd_io_i80.h"
#include "esp_lcd_panel_io.h"

#include "esp32_hx8347.h"

#include "lvgl.h"

/* =========================
 *  Pinagem do seu hardware
 * ========================= */
#define LCD_CS_PIN 37
#define LCD_RS_PIN 36 // DC / RS
#define LCD_WR_PIN 13
#define LCD_RD_PIN 11
#define LCD_D0_PIN 4
#define LCD_D1_PIN 5
#define LCD_D2_PIN 7
#define LCD_D3_PIN 15
#define LCD_D4_PIN 16
#define LCD_D5_PIN 17
#define LCD_D6_PIN 18
#define LCD_D7_PIN 10
#define LCD_RESET_PIN 38

#define LCD_H_RES 240
#define LCD_V_RES 320

#define LCD_PIXEL_CLOCK_HZ (20000000)
#define LCD_BUS_WIDTH (8)
#define LCD_CMD_BITS (8)
#define LCD_PARAM_BITS (8)
#define LCD_TRANS_QUEUE_DEPTH (10)

/* LVGL */
#define LVGL_TICK_PERIOD_MS (2)
#define LVGL_TASK_CORE (1)
#define LVGL_TASK_PRIORITY (2)
#define LVGL_TASK_DELAY_MIN_MS (10)
#define LVGL_TASK_DELAY_MAX_MS (20)
#define LVGL_DRAW_BUF_LINES (10)

/* HX8347G */
#define HX8347_RAMWR 0x22
#define TFTLCD_DELAY8 0x7F

static const char *TAG = "hx8347_lvgl";

static esp_lcd_panel_io_handle_t s_io = NULL;
static lv_display_t *s_disp = NULL;

static volatile int s_pending_chunks = 0;

typedef struct esp32_hx8347_t esp32_hx8347_t;

struct esp32_hx8347_t
{
    lv_obj_t *scr;
    SemaphoreHandle_t lvgl_mutex;
};

/* =========================
 *  Sequência de init HX8347G
 * ========================= */
static const uint8_t hx8347_init_seq[] = {
    0xEA, 2, 0x00, 0x20,
    0xEC, 2, 0x0C, 0xC4,
    0xE8, 1, 0x38,
    0xE9, 1, 0x10,
    0xF1, 1, 0x01,
    0xF2, 1, 0x10,

    0x40, 13, 0x01, 0x00, 0x00, 0x10, 0x0E, 0x24, 0x04, 0x50, 0x02, 0x13, 0x19, 0x19, 0x16,
    0x50, 14, 0x1B, 0x31, 0x2F, 0x3F, 0x3F, 0x3E, 0x2F, 0x7B, 0x09, 0x06, 0x06, 0x0C, 0x1D, 0xCC,

    0x1B, 1, 0x1B,
    0x1A, 1, 0x01,
    0x24, 1, 0x2F,
    0x25, 1, 0x57,
    0x23, 1, 0x88,

    0x18, 1, 0x34,
    0x19, 1, 0x01,
    0x01, 1, 0x00,
    0x1F, 1, 0x88,

    TFTLCD_DELAY8, 5,
    0x1F, 1, 0x80,

    TFTLCD_DELAY8, 3,
    0x1F, 1, 0x90,

    TFTLCD_DELAY8, 5,
    0x1F, 1, 0xD0,

    TFTLCD_DELAY8, 5,
    0x17, 1, 0x05,
    0x36, 1, 0x00,
    0x28, 1, 0x38,

    TFTLCD_DELAY8, 40,

    0x28, 1, 0x3C,
    0x16, 1, 0x80};

static void hx8347_gpio_reset(void)
{
    gpio_config_t out_cfg = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask =
            (1ULL << LCD_CS_PIN) |
            (1ULL << LCD_RS_PIN) |
            (1ULL << LCD_WR_PIN) |
            (1ULL << LCD_RD_PIN) |
            (1ULL << LCD_RESET_PIN)};
    ESP_ERROR_CHECK(gpio_config(&out_cfg));

    gpio_config_t data_cfg = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask =
            (1ULL << LCD_D0_PIN) |
            (1ULL << LCD_D1_PIN) |
            (1ULL << LCD_D2_PIN) |
            (1ULL << LCD_D3_PIN) |
            (1ULL << LCD_D4_PIN) |
            (1ULL << LCD_D5_PIN) |
            (1ULL << LCD_D6_PIN) |
            (1ULL << LCD_D7_PIN)};
    ESP_ERROR_CHECK(gpio_config(&data_cfg));

    gpio_set_level(LCD_CS_PIN, 1);
    gpio_set_level(LCD_RS_PIN, 1);
    gpio_set_level(LCD_WR_PIN, 1);
    gpio_set_level(LCD_RD_PIN, 1);
    gpio_set_level(LCD_RESET_PIN, 1);

    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(LCD_RESET_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(LCD_RESET_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(120));
}

static esp_err_t hx8347_write_param8(esp_lcd_panel_io_handle_t io, uint8_t cmd, uint8_t value)
{
    return esp_lcd_panel_io_tx_param(io, cmd, &value, 1);
}

static esp_err_t hx8347_write_table(esp_lcd_panel_io_handle_t io, const uint8_t *table, size_t size)
{
    size_t idx = 0;

    while (idx < size)
    {
        uint8_t cmd = table[idx++];
        uint8_t len = table[idx++];

        if (cmd == TFTLCD_DELAY8)
        {
            vTaskDelay(pdMS_TO_TICKS(len));
        }
        else
        {
            ESP_ERROR_CHECK(esp_lcd_panel_io_tx_param(io, cmd, &table[idx], len));
            idx += len;
        }
    }

    return ESP_OK;
}

static void hx8347_lcd_init(esp_lcd_panel_io_handle_t io)
{
    ESP_ERROR_CHECK(hx8347_write_table(io, hx8347_init_seq, sizeof(hx8347_init_seq)));
    ESP_LOGI(TAG, "Init HX8347G enviado");
}

static void hx8347_set_window(esp_lcd_panel_io_handle_t io,
                              uint16_t x0, uint16_t y0,
                              uint16_t x1, uint16_t y1)
{
    uint8_t v;

    v = (x0 >> 8) & 0xFF;
    ESP_ERROR_CHECK(hx8347_write_param8(io, 0x02, v));
    v = x0 & 0xFF;
    ESP_ERROR_CHECK(hx8347_write_param8(io, 0x03, v));

    v = (x1 >> 8) & 0xFF;
    ESP_ERROR_CHECK(hx8347_write_param8(io, 0x04, v));
    v = x1 & 0xFF;
    ESP_ERROR_CHECK(hx8347_write_param8(io, 0x05, v));

    v = (y0 >> 8) & 0xFF;
    ESP_ERROR_CHECK(hx8347_write_param8(io, 0x06, v));
    v = y0 & 0xFF;
    ESP_ERROR_CHECK(hx8347_write_param8(io, 0x07, v));

    v = (y1 >> 8) & 0xFF;
    ESP_ERROR_CHECK(hx8347_write_param8(io, 0x08, v));
    v = y1 & 0xFF;
    ESP_ERROR_CHECK(hx8347_write_param8(io, 0x09, v));
}

static bool hx8347_on_color_trans_done(esp_lcd_panel_io_handle_t panel_io,
                                       esp_lcd_panel_io_event_data_t *edata,
                                       void *user_ctx)
{
    (void)panel_io;
    (void)edata;

    lv_display_t *disp = (lv_display_t *)user_ctx;

    // if (--s_pending_chunks == 0)
    //{
    lv_display_flush_ready(disp);
    //}
    return false;
}

static void lvgl_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map)
{
    const uint16_t x1 = area->x1;
    const uint16_t y1 = area->y1;
    const uint16_t x2 = area->x2;
    const uint16_t y2 = area->y2;

    const uint16_t w = (x2 - x1 + 1);
    const uint16_t h = (y2 - y1 + 1);
    const uint16_t chunk_lines = 10;

    s_pending_chunks = (h + chunk_lines - 1) / chunk_lines;

    for (uint16_t row = 0; row < h;)
    {
        uint16_t ch = (h - row > chunk_lines) ? chunk_lines : (h - row);
        size_t chunk_bytes = (size_t)w * ch * 2;
        const uint8_t *chunk_ptr = px_map + ((size_t)row * w * 2);

        hx8347_set_window(s_io, x1, y1 + row, x2, y1 + row + ch - 1);

        ESP_ERROR_CHECK(esp_lcd_panel_io_tx_color(
            s_io,
            HX8347_RAMWR,
            chunk_ptr,
            chunk_bytes));

        row += ch;
    }

    // não chame lv_display_flush_ready(disp) aqui
}

static void lvgl_tick_timer_cb(void *arg)
{
    (void)arg;
    lv_tick_inc(LVGL_TICK_PERIOD_MS);
}

// static void lvgl_create_ui(lv_obj_t *scr)
//{
//     scr = lv_screen_active();

//    lv_obj_clean(lv_screen_active());

//  lv_obj_set_style_bg_color(scr, lv_color_hex(0x000000), LV_PART_MAIN);
//  lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, LV_PART_MAIN);

// lv_obj_t *label = lv_label_create(scr);
// lv_label_set_text(label, "VERMELHO ASDFGHJ");
// lv_obj_set_style_text_color(label, lv_color_hex(0xFF0000), LV_PART_MAIN);
// MUDAR TAMANHO DA FONTE
// lv_obj_set_style_text_font(label, &lv_font_montserrat_24, LV_PART_MAIN);

// lv_obj_center(label);

/* lv_obj_t *label2 = lv_label_create(scr);
 lv_label_set_text(label2, "VERDE ZXCVBNM");
 lv_obj_set_style_text_color(label2, lv_color_hex(0x00FF00), LV_PART_MAIN);
 lv_obj_set_style_text_font(label2, &lv_font_montserrat_24, LV_PART_MAIN);

 lv_obj_align(label2, LV_ALIGN_TOP_MID, 0, 0);
*/

/*      lv_obj_t *img2 = lv_image_create(scr);
     lv_image_set_src(img2, &logo);
    lv_obj_align(img2, LV_ALIGN_BOTTOM_MID, 0, 0);
*/

//     lv_obj_set_size(img, 100, 100);

//     lv_image_set_scale(img, 98); // 128 = 50%

//  }

static void lvgl_port_task(void *arg)
{
    esp32_hx8347_handle_t handle = (esp32_hx8347_handle_t)arg;

    while (1)

    {

        xSemaphoreTake(handle->lvgl_mutex, portMAX_DELAY);
        uint32_t delay_ms = lv_timer_handler();
        xSemaphoreGive(handle->lvgl_mutex);

        if (delay_ms < LVGL_TASK_DELAY_MIN_MS)
        {
            delay_ms = LVGL_TASK_DELAY_MIN_MS;
        }
        else if (delay_ms > LVGL_TASK_DELAY_MAX_MS)
        {
            delay_ms = LVGL_TASK_DELAY_MAX_MS;
        }

        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
}

static void lvgl_init_and_start(esp32_hx8347_handle_t handle)
{
    lv_init();

    handle->lvgl_mutex = xSemaphoreCreateMutex();
    assert(handle->lvgl_mutex);

    const esp_timer_create_args_t tick_args = {
        .callback = &lvgl_tick_timer_cb,
        .name = "lvgl_tick"};

    esp_timer_handle_t tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&tick_args, &tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(tick_timer, LVGL_TICK_PERIOD_MS * 1000));

    s_disp = lv_display_create(LCD_H_RES, LCD_V_RES);
    assert(s_disp);

    size_t draw_buf_size = LCD_H_RES * LVGL_DRAW_BUF_LINES * sizeof(lv_color_t);

    void *buf1 = heap_caps_malloc(draw_buf_size, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    void *buf2 = heap_caps_malloc(draw_buf_size, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    assert(buf1 && buf2);

    lv_display_set_buffers(s_disp, buf1, buf2, draw_buf_size, LV_DISPLAY_RENDER_MODE_PARTIAL);
    lv_display_set_flush_cb(s_disp, lvgl_flush_cb);
    lv_display_set_color_format(s_disp, LV_COLOR_FORMAT_RGB565);
    lv_display_set_default(s_disp);

    const esp_lcd_panel_io_callbacks_t cbs = {
        .on_color_trans_done = hx8347_on_color_trans_done,
    };
    ESP_ERROR_CHECK(esp_lcd_panel_io_register_event_callbacks(s_io, &cbs, s_disp));

    handle->scr = lv_screen_active();

    lv_obj_clean(lv_screen_active());

    xTaskCreatePinnedToCore(lvgl_port_task, "lvgl", 4096, handle, LVGL_TASK_PRIORITY, NULL, LVGL_TASK_CORE);
}

static esp_err_t hx8347_i80_init(esp_lcd_panel_io_handle_t *out_io)
{
    esp_lcd_i80_bus_handle_t i80_bus = NULL;

    esp_lcd_i80_bus_config_t bus_config = {
        .clk_src = LCD_CLK_SRC_DEFAULT,
        .dc_gpio_num = LCD_RS_PIN,
        .wr_gpio_num = LCD_WR_PIN,
        .data_gpio_nums = {
            LCD_D0_PIN, LCD_D1_PIN, LCD_D2_PIN, LCD_D3_PIN,
            LCD_D4_PIN, LCD_D5_PIN, LCD_D6_PIN, LCD_D7_PIN},
        .bus_width = LCD_BUS_WIDTH,
        .max_transfer_bytes = LCD_H_RES * LVGL_DRAW_BUF_LINES * 2,
        .dma_burst_size = 64,
    };

    ESP_ERROR_CHECK(esp_lcd_new_i80_bus(&bus_config, &i80_bus));

    esp_lcd_panel_io_i80_config_t io_config = {
        .cs_gpio_num = LCD_CS_PIN,
        .pclk_hz = LCD_PIXEL_CLOCK_HZ,
        .trans_queue_depth = LCD_TRANS_QUEUE_DEPTH,
        .dc_levels = {
            .dc_idle_level = 0,
            .dc_cmd_level = 0,
            .dc_dummy_level = 0,
            .dc_data_level = 1,
        },
        .lcd_cmd_bits = LCD_CMD_BITS,
        .lcd_param_bits = LCD_PARAM_BITS,
        .flags = {
            .reverse_color_bits = 1,
            .swap_color_bytes = 0,
        },
    };

    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i80(i80_bus, &io_config, out_io));
    return ESP_OK;
}

esp_err_t esp32_hx8347_init(esp32_hx8347_config_t *config, esp32_hx8347_handle_t *handle)
{
    esp_err_t ret = ESP_OK;
    esp32_hx8347_t *esp32_hx8347 = NULL;
    ESP_GOTO_ON_FALSE(config, ESP_ERR_INVALID_ARG, err, TAG, "Invalid arguments");
    esp32_hx8347 = calloc(1, sizeof(esp32_hx8347_t));

    hx8347_gpio_reset();

    ESP_ERROR_CHECK(hx8347_i80_init(&s_io));
    hx8347_lcd_init(s_io);

    lvgl_init_and_start(esp32_hx8347);

    if (esp32_hx8347 == NULL)
    {
        ESP_LOGE(TAG, "Erro ao alocar memória para esp32_hx8347_handle\n");
        return ESP_ERR_NO_MEM;
    }

    *handle = esp32_hx8347;
    ESP_LOGI(TAG, "Esp32-hx8347 initialized successfully");
    ret = ESP_OK;
    return ret;
err:
    ESP_LOGE(TAG, "Error to Configure");
    if (esp32_hx8347)
    {
        free(esp32_hx8347);
        esp32_hx8347 = NULL;
    }
    return ret;
}

// retorn the active screen object, so the user can create objects on it
lv_obj_t *esp32_hx8347_get_screen(esp32_hx8347_handle_t handle)
{
    if (handle == NULL)
    {
        ESP_LOGE(TAG, "Invalid handle");
        return NULL;
    }
    return handle->scr;
}

// disponibilize take and give of lvgl mutex, so the user can call lv_timer_handler() in their own task if they want
void esp32_hx8347_lvgl_lock(esp32_hx8347_handle_t handle)
{
    if (handle == NULL)
    {
        ESP_LOGE(TAG, "Invalid handle");
        return;
    }
    xSemaphoreTake(handle->lvgl_mutex, portMAX_DELAY);
}

void esp32_hx8347_lvgl_unlock(esp32_hx8347_handle_t handle)
{
    if (handle == NULL)
    {
        ESP_LOGE(TAG, "Invalid handle");
        return;
    }
    xSemaphoreGive(handle->lvgl_mutex);
}