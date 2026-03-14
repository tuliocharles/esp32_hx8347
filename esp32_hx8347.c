#include <stdio.h>
#include "esp32_hx8347.h"

#include <stdbool.h>
#include "sdkconfig.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_heap_caps.h"
#include "esp32s3/rom/lldesc.h"
#include "esp32s3/rom/gpio.h"
#include "driver/i2s.h"
#include "driver/adc.h"

#include "soc/dport_access.h"
#include "soc/dport_reg.h"
#include "soc/i2s_struct.h"
#include "hal/gpio_ll.h"
#include "esp_log.h"

#include "driver/gpio.h"
#include "soc/gpio_periph.h"
#include "soc/system_reg.h"
#include "soc/lcd_cam_struct.h"
#include "soc/lcd_cam_reg.h"
#include "soc/gdma_struct.h"
#include "soc/gdma_periph.h"
#include "soc/gdma_reg.h"

// #include "lvgl.h"

#define TFTLCD_DELAY8 0x7F

#define LCD_CS_PIN 37
#define LCD_RS_PIN 36
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

char *TAG = "LCD";

#define DRIVER "HX8347G"
#define CONFIG_WIDTH 240
#define CONFIG_HEIGHT 320
#define CONFIG_OFFSETX 0
#define CONFIG_OFFSETY 0
#define TFTLCD_DELAY8 0x7F

#define LCD_CMD_LEV (0)
#define LCD_DATA_LEV (1)

#define BOARD_LCD_I2S_BITWIDTH 8

#define LCD_CHECK(a, str, ret)                                                 \
    if (!(a))                                                                  \
    {                                                                          \
        ESP_LOGE(TAG, "%s:%d (%s):%s", __FILE__, __LINE__, __FUNCTION__, str); \
        return (ret);                                                          \
    }

#define LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE (4000)

#define ets_delay_us esp_rom_delay_us
#define portTICK_RATE_MS portTICK_PERIOD_MS


typedef struct
{
    uint32_t dma_buffer_size;
    uint32_t dma_half_buffer_size;
    uint32_t dma_node_buffer_size;
    uint32_t dma_node_cnt;
    uint32_t dma_half_node_cnt;
    lldesc_t *dma;
    uint8_t *dma_buffer;
    QueueHandle_t event_queue;
    uint8_t width;
    bool swap_data;
    uint8_t dma_num;
    intr_handle_t dma_out_intr_handle;
} lcd_cam_obj_t;

typedef struct
{
    int rs_io_num;
    lcd_cam_obj_t *i2s_lcd_obj;
    SemaphoreHandle_t mutex;
} i2s_lcd_driver1_t;


typedef struct
{
    int8_t data_width;       /*!< Parallel data width, 16bit or 8bit available */
    int8_t pin_data_num[16]; /*!< Parallel data output IO*/
    int8_t pin_num_cs;       /*!< CS io num */
    int8_t pin_num_wr;       /*!< Write clk io*/
    int8_t pin_num_rs;       /*!< RS io num */
    int clk_freq;            /*!< I2s clock frequency */
    i2s_port_t i2s_port;     /*!< I2S port number */
    bool swap_data;          /*!< Swap the 2 bytes of RGB565 color */
    uint32_t buffer_size;    /*!< DMA buffer size */
} i2s_lcd_config_t;

typedef void *i2s_lcd_handle_t; /** Handle of i2s lcd driver */

typedef struct
{
    uint32_t dma_buffer_size;
    uint32_t dma_half_buffer_size;
    uint32_t dma_node_buffer_size;
    uint32_t dma_node_cnt;
    uint32_t dma_half_node_cnt;
    // lldesc_t *dma;
    uint8_t *dma_buffer;
    QueueHandle_t event_queue;
    uint8_t width;
    bool swap_data;
    intr_handle_t lcd_cam_intr_handle;
    i2s_dev_t *i2s_dev;
} i2s_lcd_obj_t;

typedef struct
{
    void (*i2s_write_data_func)(i2s_lcd_obj_t *i2s_lcd_obj, uint8_t *data, size_t len);
    int rs_io_num;
    i2s_lcd_obj_t *i2s_lcd_obj;
    SemaphoreHandle_t mutex;
} i2s_lcd_driver_t;

// static esp32_hx8347_handle_t esp32_hx8347_handle = NULL;
static const char *tag_hx8347 = "hx8347";

typedef struct esp32_hx8347_t esp32_hx8347_t;

struct esp32_hx8347_t
{
    uint16_t _width;
    uint16_t _height;
    uint16_t _offsetx;
    uint16_t _offsety;
    uint16_t _font_direction;
    uint16_t _font_fill;
    uint16_t _font_fill_color;
    uint16_t _font_underline;
    uint16_t _font_underline_color;
    int16_t _rd;
    int16_t _wr;
    int16_t _rs;
    int16_t _cs;
    int16_t _d0;
    int16_t _d1;
    int16_t _d2;
    int16_t _d3;
    int16_t _d4;
    int16_t _d5;
    int16_t _d6;
    int16_t _d7;
    int16_t _delay;
    int16_t _interface;
    bool _debug;
    i2s_lcd_handle_t i2s_lcd_handle;
    adc1_channel_t _adc_yp;
    adc1_channel_t _adc_xm;
    int16_t _gpio_xp;
    int16_t _gpio_xm;
    int16_t _gpio_yp;
    int16_t _gpio_ym;
    int16_t _rawxp;
    int16_t _rawyp;
    bool _calibration;
    int16_t _min_xp; // Minimum xp calibration
    int16_t _min_yp; // Minimum yp calibration
    int16_t _max_xp; // Maximum xp calibration
    int16_t _max_yp; // Maximum yp calibration
    int16_t _min_xc; // Minimum x coordinate
    int16_t _min_yc; // Minimum y coordinate
    int16_t _max_xc; // Maximum x coordinate
    int16_t _max_yc; // Maximum y coordinate
};

// TFT initialization sequence for HX8347D
static const uint8_t regValues[] = {
    0xEA,
    2,
    0x00,
    0x20, // PTBA[15:0]
    0xEC,
    2,
    0x0C,
    0xC4, // STBA[15:0]
    0xE8,
    1,
    0x38, // OPON[7:0]
    0xE9,
    1,
    0x10, // OPON1[7:0]
    0xF1,
    1,
    0x01, // OTPS1B
    0xF2,
    1,
    0x10, // GEN
    // Gamma 2.2 Setting
    0x40,
    13,
    0x01,
    0x00,
    0x00,
    0x10,
    0x0E,
    0x24,
    0x04,
    0x50,
    0x02,
    0x13,
    0x19,
    0x19,
    0x16,

    0x50,
    14,
    0x1B,
    0x31,
    0x2F,
    0x3F,
    0x3F,
    0x3E,
    0x2F,
    0x7B,
    0x09,
    0x06,
    0x06,
    0x0C,
    0x1D,
    0xCC,
    // Power Voltage Setting
    0x1B,
    1,
    0x1B, // VRH=4.65V
    0x1A,
    1,
    0x01, // BT (VGH~15V,VGL~-10V,DDVDH~5V)
    0x24,
    1,
    0x2F, // VMH(VCOM High voltage ~3.2V)
    0x25,
    1,
    0x57, // VML(VCOM Low voltage -1.2V)
    // VCOM offset
    0x23,
    1,
    0x88, // for Flicker adjust //can reload from OTP
    // Power on Setting
    0x18,
    1,
    0x34, // I/P_RADJ,N/P_RADJ, Normal mode 60Hz
    0x19,
    1,
    0x01, // OSC_EN='1', start Osc
    0x01,
    1,
    0x00, // DP_STB='0', out deep sleep
    0x1F,
    1,
    0x88, // GAS=1, VOMG=00, PON=0, DK=1, XDK=0, DVDH_TRI=0, STB=0
    TFTLCD_DELAY8,
    5,
    0x1F,
    1,
    0x80, // GAS=1, VOMG=00, PON=0, DK=0, XDK=0, DVDH_TRI=0, STB=0
    TFTLCD_DELAY8,
    3,
    0x1F,
    1,
    0x90, // GAS=1, VOMG=00, PON=1, DK=0, XDK=0, DVDH_TRI=0, STB=0
    TFTLCD_DELAY8,
    5,
    0x1F,
    1,
    0xD0, // GAS=1, VOMG=10, PON=1, DK=0, XDK=0, DDVDH_TRI=0, STB=0
    TFTLCD_DELAY8,
    5, // 262k/65k color selection
    0x17,
    1,
    0x05, // default 0x06 262k color // 0x05 65k color
    // SET PANEL
    0x36,
    1,
    0x00, // SS_P, GS_P,REV_P,BGR_P
    // Display ON Setting
    0x28,
    1,
    0x38, // GON=1, DTE=1, D=1000
    TFTLCD_DELAY8,
    40, // 0x28, 1, 0x3F,	  //GON=1, DTE=1, D=1100
    0x28,
    1,
    0x3C, // GON=1, DTE=1, D=1100
    0x16,
    1,
    0x98, // MY=1, MX=0, MV=0, ML=1, BGR=1
          // 0x16, 1, 0x48,		//MY=0, MX=1, MV=0, ML=0, BGR=1
};

static void IRAM_ATTR dma_isr(void *arg)
{
    BaseType_t woken = pdFALSE;
    lcd_cam_obj_t *lcd_cam_obj = (lcd_cam_obj_t *)arg;
    uint32_t out_status = GDMA.channel[lcd_cam_obj->dma_num].out.int_st.val;
    if (out_status & GDMA_OUT_EOF_CH0_INT_ST)
    {
        GDMA.channel[lcd_cam_obj->dma_num].out.int_clr.val = GDMA_OUT_EOF_CH0_INT_ST;
        xQueueSendFromISR(lcd_cam_obj->event_queue, &out_status, &woken);
    }

    if (woken == pdTRUE)
    {
        portYIELD_FROM_ISR();
    }
}

static void lcd_dma_set_int(lcd_cam_obj_t *lcd_cam_obj)
{
    // Generate a data DMA linked list
    for (int x = 0; x < lcd_cam_obj->dma_node_cnt; x++)
    {
        lcd_cam_obj->dma[x].size = lcd_cam_obj->dma_node_buffer_size;
        lcd_cam_obj->dma[x].length = lcd_cam_obj->dma_node_buffer_size;
        lcd_cam_obj->dma[x].buf = (lcd_cam_obj->dma_buffer + lcd_cam_obj->dma_node_buffer_size * x);
        lcd_cam_obj->dma[x].eof = !((x + 1) % lcd_cam_obj->dma_half_node_cnt);
        lcd_cam_obj->dma[x].empty = (uint32_t)&lcd_cam_obj->dma[(x + 1) % lcd_cam_obj->dma_node_cnt];
    }
    lcd_cam_obj->dma[lcd_cam_obj->dma_half_node_cnt - 1].empty = (uint32_t)NULL;
    lcd_cam_obj->dma[lcd_cam_obj->dma_node_cnt - 1].empty = (uint32_t)NULL;
}

static void lcd_dma_set_left(lcd_cam_obj_t *lcd_cam_obj, int pos, size_t len)
{
    int end_pos = 0, size = 0;
    // Processing data length is an integer multiple of lcd_cam_obj->lcd.dma_node_buffer_size
    if (len % lcd_cam_obj->dma_node_buffer_size)
    {
        end_pos = (pos % 2) * lcd_cam_obj->dma_half_node_cnt + len / lcd_cam_obj->dma_node_buffer_size;
        size = len % lcd_cam_obj->dma_node_buffer_size;
    }
    else
    {
        end_pos = (pos % 2) * lcd_cam_obj->dma_half_node_cnt + len / lcd_cam_obj->dma_node_buffer_size - 1;
        size = lcd_cam_obj->dma_node_buffer_size;
    }
    // Process the tail node to make it a DMA tail
    lcd_cam_obj->dma[end_pos].size = size;
    lcd_cam_obj->dma[end_pos].length = size;
    lcd_cam_obj->dma[end_pos].eof = 1;
    lcd_cam_obj->dma[end_pos].empty = (uint32_t)NULL;
}

static void lcd_start(uint32_t dma_num, uint32_t addr, size_t len)
{
    while (LCD_CAM.lcd_user.lcd_start)
        ;
    LCD_CAM.lcd_user.lcd_reset = 1;
    LCD_CAM.lcd_user.lcd_reset = 0;
    LCD_CAM.lcd_misc.lcd_afifo_reset = 1;
    LCD_CAM.lcd_misc.lcd_afifo_reset = 0;
    while (GDMA.channel[dma_num].out.link.start)
        ;
    GDMA.channel[dma_num].out.conf0.val = 0;
    GDMA.channel[dma_num].out.conf1.val = 0;
    GDMA.channel[dma_num].out.int_clr.val = ~0;
    GDMA.channel[dma_num].out.int_ena.val = 0;
    GDMA.channel[dma_num].out.conf0.out_rst = 1;
    GDMA.channel[dma_num].out.conf0.out_rst = 0;
    GDMA.channel[dma_num].out.conf0.outdscr_burst_en = 1;
    GDMA.channel[dma_num].out.conf0.out_data_burst_en = 1;
    GDMA.channel[dma_num].out.peri_sel.sel = 5;
    GDMA.channel[dma_num].out.pri.tx_pri = 1;
    GDMA.channel[dma_num].out.int_ena.out_eof = 1;
    GDMA.channel[dma_num].out.link.addr = addr;
    GDMA.channel[dma_num].out.link.start = 1;
    esp_rom_delay_us(1); // ets_delay_us(1);
    LCD_CAM.lcd_user.lcd_update = 1;
    LCD_CAM.lcd_user.lcd_start = 1;
}

static void lcd_write_data(lcd_cam_obj_t *lcd_cam_obj, const uint8_t *data, size_t len)
{
    int event = 0;
    int x = 0, left = 0, cnt = 0;
    if (len <= 0)
    {
        ESP_LOGE(TAG, "wrong len!");
        return;
    }
    lcd_dma_set_int(lcd_cam_obj);
    uint32_t half_buffer_size = lcd_cam_obj->dma_half_buffer_size;
    cnt = len / half_buffer_size;
    // Start signal
    xQueueSend(lcd_cam_obj->event_queue, &event, 0);
    // Process a complete piece of data, ping-pong operation
    for (x = 0; x < cnt; x++)
    {
        uint8_t *out = (uint8_t *)lcd_cam_obj->dma[(x % 2) * lcd_cam_obj->dma_half_node_cnt].buf;
        const uint8_t *in = data;
        if (lcd_cam_obj->swap_data)
        {
            LCD_CAM.lcd_user.lcd_8bits_order = 1;
            memcpy(out, in, half_buffer_size);
        }
        else
        {
            LCD_CAM.lcd_user.lcd_8bits_order = 0;
            memcpy(out, in, half_buffer_size);
        }
        data += half_buffer_size;
        xQueueReceive(lcd_cam_obj->event_queue, (void *)&event, portMAX_DELAY);
        lcd_start(lcd_cam_obj->dma_num, ((uint32_t)&lcd_cam_obj->dma[(x % 2) * lcd_cam_obj->dma_half_node_cnt]) & 0xfffff, half_buffer_size);
    }
    left = len % half_buffer_size;
    // Process remaining incomplete segment data
    if (left)
    {
        uint8_t *out = (uint8_t *)lcd_cam_obj->dma[(x % 2) * lcd_cam_obj->dma_half_node_cnt].buf;
        const uint8_t *in = data;
        cnt = left - left % 2;
        if (cnt)
        {
            if (lcd_cam_obj->swap_data)
            {
                LCD_CAM.lcd_user.lcd_8bits_order = 1;
                memcpy(out, in, cnt);
            }
            else
            {
                LCD_CAM.lcd_user.lcd_8bits_order = 0;
                memcpy(out, in, cnt);
            }
        }
        if (left % 2)
        {
            LCD_CAM.lcd_user.lcd_8bits_order = 0;
            out[cnt] = in[cnt];
        }

        lcd_dma_set_left(lcd_cam_obj, x, left);
        xQueueReceive(lcd_cam_obj->event_queue, (void *)&event, portMAX_DELAY);
        lcd_start(lcd_cam_obj->dma_num, ((uint32_t)&lcd_cam_obj->dma[(x % 2) * lcd_cam_obj->dma_half_node_cnt]) & 0xfffff, left);
    }
    xQueueReceive(lcd_cam_obj->event_queue, (void *)&event, portMAX_DELAY);
}

static esp_err_t lcd_cam_config(const i2s_lcd_config_t *config, uint32_t dma_num)
{
    GDMA.channel[dma_num].out.conf0.val = 0;
    GDMA.channel[dma_num].out.conf1.val = 0;
    GDMA.channel[dma_num].in.conf0.val = 0;
    GDMA.channel[dma_num].in.conf1.val = 0;
    GDMA.channel[dma_num].out.int_clr.val = ~0;
    GDMA.channel[dma_num].out.int_ena.val = 0;
    GDMA.channel[dma_num].in.int_clr.val = ~0;
    GDMA.channel[dma_num].in.int_ena.val = 0;

    LCD_CAM.lcd_clock.val = 0;
    LCD_CAM.lcd_clock.clk_en = 1;
    LCD_CAM.lcd_clock.lcd_clk_sel = 3;
    LCD_CAM.lcd_clock.lcd_clkm_div_b = 0;
    LCD_CAM.lcd_clock.lcd_clkm_div_a = 10;
    LCD_CAM.lcd_clock.lcd_clkm_div_num = 2;
    LCD_CAM.lcd_clock.lcd_clkcnt_n = 80000000 / config->clk_freq - 1;
    LCD_CAM.lcd_clock.lcd_clk_equ_sysclk = 0;
    LCD_CAM.lcd_clock.lcd_ck_idle_edge = 1; // After lcd_clk_equ_sysclk is set to 1, this bit has no effect
    LCD_CAM.lcd_clock.lcd_ck_out_edge = 0;  // After lcd_clk_equ_sysclk is set to 1, this bit has no effect
    LCD_CAM.lcd_user.val = 0;
    LCD_CAM.lcd_user.lcd_2byte_en = (config->data_width == 16) ? 1 : 0;
    LCD_CAM.lcd_user.lcd_byte_order = 0;
    LCD_CAM.lcd_user.lcd_bit_order = 0;
    LCD_CAM.lcd_user.lcd_cmd = 0;            // FSM CMD phase
    LCD_CAM.lcd_user.lcd_cmd_2_cycle_en = 0; // 2 cycle command
    LCD_CAM.lcd_user.lcd_dout = 1;           // FSM DOUT phase
    LCD_CAM.lcd_user.lcd_dout_cyclelen = 2 - 1;
    LCD_CAM.lcd_user.lcd_8bits_order = 0;
    LCD_CAM.lcd_user.lcd_always_out_en = 1;
    LCD_CAM.lcd_misc.val = 0;
    LCD_CAM.lcd_misc.lcd_afifo_threshold_num = 11;
    LCD_CAM.lcd_misc.lcd_vfk_cyclelen = 3;
    LCD_CAM.lcd_misc.lcd_vbk_cyclelen = 0;
    LCD_CAM.lcd_misc.lcd_cd_idle_edge = 1; // idle edge of CD is set to 0
    LCD_CAM.lcd_misc.lcd_cd_cmd_set = 1;
    LCD_CAM.lcd_misc.lcd_cd_dummy_set = 0;
    LCD_CAM.lcd_misc.lcd_cd_data_set = 0; // change when DOUT start
    LCD_CAM.lcd_misc.lcd_bk_en = 1;
    LCD_CAM.lcd_misc.lcd_afifo_reset = 1;
    LCD_CAM.lcd_misc.lcd_afifo_reset = 0;
    LCD_CAM.lcd_ctrl.val = 0;
    LCD_CAM.lcd_ctrl.lcd_rgb_mode_en = 0;
    LCD_CAM.lcd_cmd_val.val = 0; // write command
    LCD_CAM.lcd_user.lcd_update = 1;

    GDMA.channel[dma_num].out.conf0.out_rst = 1;
    GDMA.channel[dma_num].out.conf0.out_rst = 0;
    GDMA.channel[dma_num].out.conf0.outdscr_burst_en = 1;
    GDMA.channel[dma_num].out.conf0.out_data_burst_en = 1;
    GDMA.channel[dma_num].out.peri_sel.sel = (config->data_width == 1) ? 1 : 5;
    GDMA.channel[dma_num].out.pri.tx_pri = 1;
    GDMA.channel[dma_num].out.int_ena.out_eof = 1;

    return ESP_OK;
}

static void lcd_set_pin(const i2s_lcd_config_t *config)
{
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[config->pin_num_wr], PIN_FUNC_GPIO);
    gpio_set_direction(config->pin_num_wr, GPIO_MODE_OUTPUT);
    gpio_set_pull_mode(config->pin_num_wr, GPIO_FLOATING);
    gpio_matrix_out(config->pin_num_wr, LCD_PCLK_IDX, false, false);

    for (int i = 0; i < config->data_width; i++)
    {
        PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[config->pin_data_num[i]], PIN_FUNC_GPIO);
        gpio_set_direction(config->pin_data_num[i], GPIO_MODE_OUTPUT);
        gpio_set_pull_mode(config->pin_data_num[i], GPIO_FLOATING);
        gpio_matrix_out(config->pin_data_num[i], LCD_DATA_OUT0_IDX + i, false, false);
    }
}

static esp_err_t lcd_dma_config(lcd_cam_obj_t *lcd_cam_obj, const i2s_lcd_config_t *config)
{
    int cnt = 0;
    if (LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE % 2 != 0)
    {
        ESP_LOGE(TAG, "need 2-byte aligned data length");
        return ESP_FAIL;
    }

    if (config->buffer_size >= LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE * 2)
    {
        lcd_cam_obj->dma_node_buffer_size = LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE;
        for (cnt = 0; cnt < config->buffer_size - 8; cnt++)
        { // Find a buffer size that can divide dma_size
            if ((config->buffer_size - cnt) % (lcd_cam_obj->dma_node_buffer_size * 2) == 0)
            {
                break;
            }
        }
        lcd_cam_obj->dma_buffer_size = config->buffer_size - cnt;
    }
    else
    {
        lcd_cam_obj->dma_node_buffer_size = config->buffer_size / 2;
        lcd_cam_obj->dma_buffer_size = lcd_cam_obj->dma_node_buffer_size * 2;
    }

    lcd_cam_obj->dma_half_buffer_size = lcd_cam_obj->dma_buffer_size / 2;

    lcd_cam_obj->dma_node_cnt = (lcd_cam_obj->dma_buffer_size) / lcd_cam_obj->dma_node_buffer_size; // Number of DMA nodes
    lcd_cam_obj->dma_half_node_cnt = lcd_cam_obj->dma_node_cnt / 2;

    // ESP_LOGI(TAG, "lcd_buffer_size: %d, lcd_dma_size: %d, lcd_dma_node_cnt: %d", lcd_cam_obj->dma_buffer_size, lcd_cam_obj->dma_node_buffer_size, lcd_cam_obj->dma_node_cnt);

    lcd_cam_obj->dma = (lldesc_t *)heap_caps_malloc(lcd_cam_obj->dma_node_cnt * sizeof(lldesc_t), MALLOC_CAP_DMA);
    lcd_cam_obj->dma_buffer = (uint8_t *)heap_caps_malloc(lcd_cam_obj->dma_buffer_size * sizeof(uint8_t), MALLOC_CAP_DMA);
    return ESP_OK;
}

esp_err_t lcd_cam_deinit(i2s_lcd_driver1_t *drv)
{
    if (!drv->i2s_lcd_obj)
    {
        return ESP_FAIL;
    }

    if (drv->i2s_lcd_obj->event_queue)
    {
        vQueueDelete(drv->i2s_lcd_obj->event_queue);
    }
    if (drv->i2s_lcd_obj->dma)
    {
        free(drv->i2s_lcd_obj->dma);
    }
    if (drv->i2s_lcd_obj->dma_buffer)
    {
        free(drv->i2s_lcd_obj->dma_buffer);
    }

    if (drv->i2s_lcd_obj->dma_out_intr_handle)
    {
        esp_intr_free(drv->i2s_lcd_obj->dma_out_intr_handle);
    }
    GDMA.channel[drv->i2s_lcd_obj->dma_num].out.link.start = 0x0;
    free(drv->i2s_lcd_obj);
    drv->i2s_lcd_obj = NULL;
    return ESP_OK;
}

static esp_err_t lcd_cam_init(i2s_lcd_driver1_t *drv, const i2s_lcd_config_t *config)
{
    esp_err_t ret = ESP_OK;
    lcd_cam_obj_t *lcd_cam_obj = (lcd_cam_obj_t *)heap_caps_calloc(1, sizeof(lcd_cam_obj_t), MALLOC_CAP_DMA);
    if (lcd_cam_obj == NULL)
    {
        ESP_LOGE(TAG, "lcd_cam object malloc error");
        return ESP_ERR_NO_MEM;
    }
    drv->i2s_lcd_obj = lcd_cam_obj;

    // Enable LCD_CAM periph
    if (REG_GET_BIT(SYSTEM_PERIP_CLK_EN1_REG, SYSTEM_LCD_CAM_CLK_EN) == 0)
    {
        REG_CLR_BIT(SYSTEM_PERIP_CLK_EN1_REG, SYSTEM_LCD_CAM_CLK_EN);
        REG_SET_BIT(SYSTEM_PERIP_CLK_EN1_REG, SYSTEM_LCD_CAM_CLK_EN);
        REG_SET_BIT(SYSTEM_PERIP_RST_EN1_REG, SYSTEM_LCD_CAM_RST);
        REG_CLR_BIT(SYSTEM_PERIP_RST_EN1_REG, SYSTEM_LCD_CAM_RST);
    }

    if (REG_GET_BIT(SYSTEM_PERIP_CLK_EN1_REG, SYSTEM_DMA_CLK_EN) == 0)
    {
        REG_CLR_BIT(SYSTEM_PERIP_CLK_EN1_REG, SYSTEM_DMA_CLK_EN);
        REG_SET_BIT(SYSTEM_PERIP_CLK_EN1_REG, SYSTEM_DMA_CLK_EN);
        REG_SET_BIT(SYSTEM_PERIP_RST_EN1_REG, SYSTEM_DMA_RST);
        REG_CLR_BIT(SYSTEM_PERIP_RST_EN1_REG, SYSTEM_DMA_RST);
    }

    for (int x = SOC_GDMA_PAIRS_PER_GROUP - 1; x >= 0; x--)
    {
        if (GDMA.channel[x].out.link.start == 0x0)
        {
            lcd_cam_obj->dma_num = x;
            break;
        }
        if (x == SOC_GDMA_PAIRS_PER_GROUP - 1)
        {
            ESP_LOGE(TAG, "DMA error");
            lcd_cam_deinit(drv);
            return ESP_FAIL;
        }
    }

    ret |= lcd_cam_config(config, lcd_cam_obj->dma_num);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "lcd_cam config fail!");
        lcd_cam_deinit(drv);
        return ESP_FAIL;
    }

    lcd_set_pin(config);
    ret |= lcd_dma_config(lcd_cam_obj, config);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "lcd config fail!");
        lcd_cam_deinit(drv);
        return ESP_FAIL;
    }

    lcd_cam_obj->event_queue = xQueueCreate(1, sizeof(int));
    lcd_cam_obj->width = config->data_width;
    lcd_cam_obj->swap_data = config->swap_data;
    if (lcd_cam_obj->event_queue == NULL)
    {
        ESP_LOGE(TAG, "lcd config fail!");
        lcd_cam_deinit(drv);
        return ESP_FAIL;
    }

    ret |= esp_intr_alloc_intrstatus(gdma_periph_signals.groups[0].pairs[lcd_cam_obj->dma_num].tx_irq_id,
                                     ESP_INTR_FLAG_LOWMED | ESP_INTR_FLAG_SHARED | ESP_INTR_FLAG_IRAM,
                                     (uint32_t)&GDMA.channel[lcd_cam_obj->dma_num].out.int_st, GDMA_OUT_EOF_CH0_INT_ST,
                                     dma_isr, lcd_cam_obj, &lcd_cam_obj->dma_out_intr_handle);

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "lcd_cam intr alloc fail!");
        lcd_cam_deinit(drv);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "lcd init ok");

    return ESP_OK;
}

i2s_lcd_handle_t i2s_lcd_driver_init(const i2s_lcd_config_t *config)
{
    LCD_CHECK(NULL != config, "config pointer invalid", NULL);
    LCD_CHECK(GPIO_IS_VALID_GPIO(config->pin_num_wr), "GPIO WR invalid", NULL);
    LCD_CHECK(GPIO_IS_VALID_GPIO(config->pin_num_rs), "GPIO RS invalid", NULL);
    LCD_CHECK(config->data_width > 0 && config->data_width <= 16, "Bit width out of range", NULL);
    LCD_CHECK(0 == (config->data_width % 8), "Bit width must be a multiple of 8", NULL);
    uint64_t pin_mask = 0;
    for (size_t i = 0; i < config->data_width; i++)
    {
        uint64_t mask = 1ULL << config->pin_data_num[i];
        LCD_CHECK(!(pin_mask & mask), "Data bus GPIO has a duplicate", NULL);
        LCD_CHECK(GPIO_IS_VALID_GPIO(config->pin_data_num[i]), "Data bus gpio invalid", NULL);
        pin_mask |= mask;
    }

    i2s_lcd_driver1_t *i2s_lcd_drv = (i2s_lcd_driver1_t *)heap_caps_malloc(sizeof(i2s_lcd_driver1_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    LCD_CHECK(NULL != i2s_lcd_drv, "Error malloc handle of i2s lcd driver", NULL);

    esp_err_t ret = lcd_cam_init(i2s_lcd_drv, config);
    if (ESP_OK != ret)
    {
        ESP_LOGE(TAG, "%s:%d (%s):%s", __FILE__, __LINE__, __FUNCTION__, "i2s lcd driver initialize failed");
        heap_caps_free(i2s_lcd_drv);
        return NULL;
    }

    i2s_lcd_drv->mutex = xSemaphoreCreateMutex();
    if (i2s_lcd_drv->mutex == NULL)
    {
        ESP_LOGE(TAG, "%s:%d (%s):%s", __FILE__, __LINE__, __FUNCTION__, "lcd create mutex failed");
        lcd_cam_deinit(i2s_lcd_drv);
        heap_caps_free(i2s_lcd_drv);
        return NULL;
    }

    if (config->pin_num_cs >= 0)
    {
        gpio_pad_select_gpio(config->pin_num_cs);
        gpio_set_direction(config->pin_num_cs, GPIO_MODE_OUTPUT);
        gpio_set_level(config->pin_num_cs, 0);
    }

    gpio_pad_select_gpio(config->pin_num_rs);
    gpio_set_direction(config->pin_num_rs, GPIO_MODE_OUTPUT);
    gpio_set_level(config->pin_num_rs, LCD_DATA_LEV);
    i2s_lcd_drv->rs_io_num = config->pin_num_rs;

    return (i2s_lcd_handle_t)i2s_lcd_drv;
}

esp_err_t i2s_lcd_driver_deinit(i2s_lcd_handle_t handle)
{
    i2s_lcd_driver1_t *i2s_lcd_drv = (i2s_lcd_driver1_t *)handle;
    LCD_CHECK(NULL != i2s_lcd_drv, "handle pointer invalid", ESP_ERR_INVALID_ARG);
    lcd_cam_deinit(i2s_lcd_drv);
    vSemaphoreDelete(i2s_lcd_drv->mutex);
    heap_caps_free(handle);
    return ESP_OK;
}

esp_err_t i2s_lcd_write_data(i2s_lcd_handle_t handle, uint16_t data)
{
    i2s_lcd_driver1_t *i2s_lcd_drv = (i2s_lcd_driver1_t *)handle;
    LCD_CHECK(NULL != i2s_lcd_drv, "handle pointer invalid", ESP_ERR_INVALID_ARG);
    lcd_write_data(i2s_lcd_drv->i2s_lcd_obj, (uint8_t *)&data, i2s_lcd_drv->i2s_lcd_obj->width == 16 ? 2 : 1);
    return ESP_OK;
}

esp_err_t i2s_lcd_write_cmd(i2s_lcd_handle_t handle, uint16_t cmd)
{
    i2s_lcd_driver1_t *i2s_lcd_drv = (i2s_lcd_driver1_t *)handle;
    LCD_CHECK(NULL != i2s_lcd_drv, "handle pointer invalid", ESP_ERR_INVALID_ARG);
    gpio_set_level(i2s_lcd_drv->rs_io_num, LCD_CMD_LEV);
    lcd_write_data(i2s_lcd_drv->i2s_lcd_obj, (uint8_t *)&cmd, i2s_lcd_drv->i2s_lcd_obj->width == 16 ? 2 : 1);
    gpio_set_level(i2s_lcd_drv->rs_io_num, LCD_DATA_LEV);
    return ESP_OK;
}

esp_err_t i2s_lcd_write_command(i2s_lcd_handle_t handle, const uint8_t *cmd, uint32_t length)
{
    i2s_lcd_driver1_t *i2s_lcd_drv = (i2s_lcd_driver1_t *)handle;
    LCD_CHECK(NULL != i2s_lcd_drv, "handle pointer invalid", ESP_ERR_INVALID_ARG);
    gpio_set_level(i2s_lcd_drv->rs_io_num, LCD_CMD_LEV);
    lcd_write_data(i2s_lcd_drv->i2s_lcd_obj, cmd, length);
    gpio_set_level(i2s_lcd_drv->rs_io_num, LCD_DATA_LEV);
    return ESP_OK;
}

esp_err_t i2s_lcd_write(i2s_lcd_handle_t handle, const uint8_t *data, uint32_t length)
{
    i2s_lcd_driver1_t *i2s_lcd_drv = (i2s_lcd_driver1_t *)handle;
    LCD_CHECK(NULL != i2s_lcd_drv, "handle pointer invalid", ESP_ERR_INVALID_ARG);
    lcd_write_data(i2s_lcd_drv->i2s_lcd_obj, data, length);

    return ESP_OK;
}

esp_err_t i2s_lcd_acquire(i2s_lcd_handle_t handle)
{
    i2s_lcd_driver1_t *i2s_lcd_drv = (i2s_lcd_driver1_t *)handle;
    LCD_CHECK(NULL != i2s_lcd_drv, "handle pointer invalid", ESP_ERR_INVALID_ARG);
    BaseType_t ret = xSemaphoreTake(i2s_lcd_drv->mutex, portMAX_DELAY);
    LCD_CHECK(pdTRUE == ret, "Take semaphore failed", ESP_FAIL);
    return ESP_OK;
}

esp_err_t i2s_lcd_release(i2s_lcd_handle_t handle)
{
    i2s_lcd_driver1_t *i2s_lcd_drv = (i2s_lcd_driver1_t *)handle;
    LCD_CHECK(NULL != i2s_lcd_drv, "handle pointer invalid", ESP_ERR_INVALID_ARG);
    BaseType_t ret = xSemaphoreGive(i2s_lcd_drv->mutex);
    LCD_CHECK(pdTRUE == ret, "Give semaphore failed", ESP_FAIL);
    return ESP_OK;
}

void lcd_delay_ms(int delay_time)
{
    vTaskDelay(delay_time / portTICK_PERIOD_MS);
}
#define INTERFACE_I2S 0

void lcd_write_comm_byte(esp32_hx8347_handle_t dev, uint8_t cmd)
{
    unsigned char c[1];
    c[0] = cmd;

    gpio_set_level(dev->_cs, 0);
    gpio_set_level(dev->_rs, 0);
    // gpio_set_level(dev->_rd, 1);
    i2s_lcd_write(dev->i2s_lcd_handle, c, 1);
    gpio_set_level(dev->_cs, 1);
    if (dev->_delay != 0)
        esp_rom_delay_us(dev->_delay);
}

void lcd_write_data_byte(esp32_hx8347_handle_t dev, uint8_t data);

void lcd_write_table(esp32_hx8347_t *dev, const void *table, int16_t size)
{
    int i;
    uint8_t *p = (uint8_t *)table;
    while (size > 0)
    {
        uint8_t cmd = *p++;
        uint8_t len = *p++;
        if (cmd == TFTLCD_DELAY8)
        {
            lcd_delay_ms(len);
            len = 0;
        }
        else
        {
            lcd_write_comm_byte(dev, cmd);
            for (i = 0; i < len; i++)
            {
                uint8_t data = *p++;
                lcd_write_data_byte(dev, data);
            }
        }
        size -= len + 2;
    }
}

void lcd_write_data_byte(esp32_hx8347_t *dev, uint8_t data)
{
    unsigned char d[1];
    d[0] = data;

    gpio_set_level(dev->_cs, 0);
    gpio_set_level(dev->_rs, 1);
    // gpio_set_level(dev->_rd, 1);
    i2s_lcd_write(dev->i2s_lcd_handle, d, 1);
    gpio_set_level(dev->_cs, 1);
    if (dev->_delay != 0)
        esp_rom_delay_us(dev->_delay);
}


void hx8347_lcd_write_register(esp32_hx8347_handle_t dev, uint8_t addr, uint16_t data)
{
    char d[2];
    d[0] = (data >> 8) & 0xFF;
    d[1] = data & 0xFF;
    lcd_write_comm_byte(dev, addr);
    lcd_write_data_byte(dev, d[0]);
    lcd_write_comm_byte(dev, addr + 1);
    lcd_write_data_byte(dev, d[1]);
}

esp_err_t esp32_hx8347_init(esp32_hx8347_config_t *config, esp32_hx8347_handle_t *handle)
{
    esp_err_t ret = ESP_OK;
    esp32_hx8347_t *esp32_hx8347 = NULL;
    ESP_GOTO_ON_FALSE(config, ESP_ERR_INVALID_ARG, err, tag_hx8347, "Invalid arguments");
    esp32_hx8347 = calloc(1, sizeof(esp32_hx8347_t));

    if (esp32_hx8347 == NULL)
    {
        ESP_LOGE(tag_hx8347, "Erro ao alocar memória para esp32_hx8347_handle\n");
        return ESP_ERR_NO_MEM;
    }

    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask =
            (1ULL << LCD_CS_PIN) |
            (1ULL << LCD_RS_PIN) |
            (1ULL << LCD_WR_PIN) |
            (1ULL << LCD_RD_PIN) |
            (1ULL << LCD_RESET_PIN)};

    gpio_config(&io_conf);

    gpio_set_level(LCD_CS_PIN, 1);
    gpio_set_level(LCD_RS_PIN, 1);
    gpio_set_level(LCD_WR_PIN, 1);
    gpio_set_level(LCD_RD_PIN, 1);
    gpio_set_level(LCD_RESET_PIN, 1);

    ESP_LOGI(TAG, "INTERFACE is I2S");

    i2s_lcd_config_t i2s_lcd_cfg = {
        .data_width = BOARD_LCD_I2S_BITWIDTH,
        .pin_data_num = {
            LCD_D0_PIN,
            LCD_D1_PIN,
            LCD_D2_PIN,
            LCD_D3_PIN,
            LCD_D4_PIN,
            LCD_D5_PIN,
            LCD_D6_PIN,
            LCD_D7_PIN,
        },
        .pin_num_cs = LCD_CS_PIN,
        .pin_num_wr = LCD_WR_PIN,
        .pin_num_rs = LCD_RS_PIN,

        .clk_freq = 20000000,
        .i2s_port = I2S_NUM_0,
        .buffer_size = 32000,
        .swap_data = false,
    };

    // i2s_lcd_handle_t i2s_lcd_handle;
    esp32_hx8347->i2s_lcd_handle = i2s_lcd_driver_init(&i2s_lcd_cfg);
    if (NULL == esp32_hx8347->i2s_lcd_handle)
    {
        ESP_LOGE(TAG, "%s:%d (%s):%s", __FILE__, __LINE__, __FUNCTION__, "screen 8080 interface create failed");
        return ESP_FAIL;
    }

    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(LCD_RESET_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(LCD_RESET_PIN, 1);

    esp32_hx8347->_rd = LCD_RD_PIN;
    esp32_hx8347->_wr = LCD_WR_PIN;
    esp32_hx8347->_rs = LCD_RS_PIN;
    esp32_hx8347->_cs = LCD_CS_PIN;
    esp32_hx8347->_width = CONFIG_WIDTH;
    esp32_hx8347->_height = CONFIG_HEIGHT;
    esp32_hx8347->_offsetx = CONFIG_OFFSETX;
    esp32_hx8347->_offsety = CONFIG_OFFSETY;
    esp32_hx8347->_font_direction = 0;
    esp32_hx8347->_font_fill = false;
    esp32_hx8347->_font_underline = false;
    esp32_hx8347->_debug = false;
    esp32_hx8347->_delay = 0;

    lcd_write_table(esp32_hx8347, regValues, sizeof(regValues));

    *handle = esp32_hx8347;
    ESP_LOGI(tag_hx8347, "Esp32-hx8347 initialized successfully");
    ret = ESP_OK;
    return ret;
err:
    ESP_LOGE(tag_hx8347, "Error to Configure");
    if (esp32_hx8347)
    {
        free(esp32_hx8347);
        esp32_hx8347 = NULL;
    }
    return ret;
}


void send_pixels_to_hx8347(esp32_hx8347_handle_t dev, uint8_t *pixels, size_t len){
    gpio_set_level(dev->_cs, 0);
    gpio_set_level(dev->_rs, 1);

    i2s_lcd_write(dev->i2s_lcd_handle,
                  pixels,
                  len);

    gpio_set_level(dev->_cs, 1);

}


void func(void)
{
}
