/*
该项目于2025/8/7启动
2026/01/01完成
功能：按键配网，获取网络上的“一言”并显示在屏幕(1.14寸彩屏)上，长按恢复配网状态，有一颗LED指示当前状态
By:LZY Language: C
*/
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "iot_button.h"
#include "button_gpio.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_http_client.h"
#include "esp_smartconfig.h"
#include "cJSON.h"
#include "esp_timer.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_lcd_ili9341.h"
#include "esp_partition.h"
#include "driver/ledc.h"
#include "esp_partition.h"
#include "esp_lvgl_port.h"

#define BOOT_BUTTON_NUM 1
#define BUTTON_ACTIVE_LEVEL 0

// 句子api选择
#define API_ONE

#if defined(API_ONE)
#define XYGENG_API_F "https://api.xygeng.cn/one"
#elif defined(API_TWO)
#define XYGENG_API_S "https://api.xygeng.cn/openapi/one"
#elif defined(API_THREE)
#define HITOKOTO_API "https://v1.hitokoto.cn/"
#endif

// 请求参数配置
#define HTTP_TIMEOUT_MS 8000
#define HTTP_REQ_TIMES 3
#define HTTP_REQ_DELAY_MS 10000
#define LONG_PRESS_TIME_MS 15000

#define WIFI_CONNECTED_BIT BIT0

// lcd配置
#define EXAMPLE_LCD_PIXEL_CLOCK_HZ (20 * 1000 * 1000)

#define EXAMPLE_LCD_CMD_BITS 8
#define EXAMPLE_LCD_PARAM_BITS 8
#define EXAMPLE_LCD_H_RES (135)        /* LCD Horizontal resolution */
#define EXAMPLE_LCD_V_RES (240)        /* LCD Vertical resolution */
#define EXAMPLE_LCD_BIT_PER_PIXEL (16) /* LCD bit per pixel */

#define EXAMPLE_LCD_BK_LIGHT_ON_LEVEL 1
#define EXAMPLE_LCD_BK_LIGHT_OFF_LEVEL !EXAMPLE_LCD_BK_LIGHT_ON_LEVEL

#define LCD_HOST SPI2_HOST
#define EXAMPLE_PIN_NUM_LCD_DC (GPIO_NUM_18)   /* GPIO number for SPI DC*/
#define EXAMPLE_PIN_NUM_LCD_CS (GPIO_NUM_10)   /* GPIO number for SPI CS*/
#define EXAMPLE_PIN_NUM_LCD_PCLK (GPIO_NUM_6)  /* GPIO number for SPI SCL*/
#define EXAMPLE_PIN_NUM_LCD_DATA0 (GPIO_NUM_7) /* GPIO number for SPI SDA*/
#define EXAMPLE_PIN_NUM_LCD_RST (GPIO_NUM_NC)  /* GPIO number for SPI RESET*/
#define EXAMPLE_PIN_NUM_BK_LIGHT (GPIO_NUM_5)  /* GPIO number for Backlight control */

// led配置
#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO (2) // Define the output GPIO
#define LEDC_CHANNEL LEDC_CHANNEL_0
#define LEDC_DUTY_RES LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY (4096)                // Set duty to 50%. (2 ** 13) * 50% = 4096
#define LEDC_FREQUENCY (4000)           // Frequency in Hertz. Set frequency at 4 kHz

#define EXAMPLE_LVGL_DRAW_BUF_LINES 20 // number of display lines in each draw buffer


// task分配
#define SMARTCONFIG_TASK_PRIO 1                          /* 任务优先级 */
#define SMARTCONFIG_TASK_SIZE 2 * 1024                   /* 任务堆栈大小 */
StackType_t SmartconfigTaskStack[SMARTCONFIG_TASK_SIZE]; /* 任务堆栈 */
StaticTask_t SmartconfigTaskTCB;                         /* 任务控制块 */
TaskHandle_t SmartconfigTask_Handler;                    /* 任务句柄 */

#define HTTP_TASK_PRIO 1
#define HTTP_TASK_STK_SIZE 4 * 1024
StackType_t HttpTaskStack[HTTP_TASK_STK_SIZE];
StaticTask_t HttpTaskTCB;
TaskHandle_t HttpTask_Handler;

#define LED_BREATHE_TASK_PRIO 1
#define LED_BREATHE_TASK_STK_SIZE 2 * 1024
StackType_t LedBreathTaskStack[LED_BREATHE_TASK_STK_SIZE];
StaticTask_t LedBreathTaskTCB;
TaskHandle_t LedBreathTask_Handler;

#define LED_BLINK_TASK_PRIO 1
#define LED_BLINK_TASK_STK_SIZE 2 * 1024
StackType_t LedBlinkTaskStack[LED_BLINK_TASK_STK_SIZE];
StaticTask_t LedBlinkTaskTCB;
TaskHandle_t LedBlinkTask_Handler;

// 字体缓冲区
#define FONT_BUF_SIZE 153
static uint8_t g_font_buf[FONT_BUF_SIZE] = {0};

#define TAG "app"

static EventGroupHandle_t s_wifi_event_group;
static bool wifi_ok = false;
static bool busy = false;
static bool senten = false;
static bool lvgl_ok = false;
static char *http_buf = NULL;
static size_t buf_len = 0;
static const esp_partition_t *partition_font = NULL;

static lv_display_t *lvgl_disp = NULL;
static esp_lcd_panel_handle_t panel_handle = NULL;
static esp_lcd_panel_io_handle_t io_handle = NULL;

typedef struct
{
    char content[256];
    char origin[128];
} sentence_t;

static sentence_t sen = {0};

typedef struct
{
    uint16_t min;
    uint16_t max;
    uint8_t bpp;
    uint8_t reserved[3];
} x_header_t;
typedef struct
{
    uint32_t pos;
} x_table_t;
typedef struct
{
    uint8_t adv_w;
    uint8_t box_w;
    uint8_t box_h;
    int8_t ofs_x;
    int8_t ofs_y;
    uint8_t r;
} glyph_dsc_t;

static x_header_t __g_xbf_hd = {
    .min = 0x0009,
    .max = 0xffe5,
    .bpp = 4,
};

static void http_request_task(void *arg);
static void smartconfig_task(void *arg);
static void led_breathe_task(void *arg);
static void led_blink_task(void *arg);
static bool parse_sentence(const char *json, sentence_t *sen);
static esp_err_t http_event_cb(esp_http_client_event_t *evt);
static const uint8_t *__user_font_getdata(int offset, int size);
static void lvgl_start();

static const ili9341_lcd_init_cmd_t lcd_init_cmds[] = {
    /* {cmd, { data }, data_size, delay_ms} */
    {0x11, (uint8_t[]){0x00}, 1, 120},
    {0x36, (uint8_t[]){0x70}, 1, 0},
    {0xB2, (uint8_t[]){0x0B, 0x0B, 0x00, 0x33, 0x33}, 5, 0},
    {0xB7, (uint8_t[]){0x11}, 1, 0},
    {0xBB, (uint8_t[]){0x2F}, 1, 0},
    {0xC0, (uint8_t[]){0x2C}, 1, 0},
    {0xC2, (uint8_t[]){0x01}, 1, 0},
    {0xC3, (uint8_t[]){0x0D}, 1, 0},
    {0xC4, (uint8_t[]){0x20}, 1, 0},
    {0xC6, (uint8_t[]){0x13}, 1, 0},
    {0xD0, (uint8_t[]){0xA4, 0xA1}, 2, 0},
    {0xD6, (uint8_t[]){0xA1}, 1, 0},
    {0xE0, (uint8_t[]){0xF0, 0x04, 0x07, 0x09, 0x07, 0x13, 0x25, 0x33, 0x3C, 0x34, 0x10, 0x10, 0x29, 0x32}, 14, 0},
    {0xE1, (uint8_t[]){0xF0, 0x05, 0x08, 0x0A, 0x09, 0x05, 0x25, 0x32, 0x3B, 0x3B, 0x17, 0x18, 0x2E, 0x37}, 14, 0},
    {0xE4, (uint8_t[]){0x25, 0x00, 0x00}, 3, 0},
    {0x21, (uint8_t[]){0x00}, 1, 0},
    {0x29, (uint8_t[]){0x00}, 1, 0},
    {0x2A, (uint8_t[]){0x00, 0x00, 0x00, 0xEF}, 4, 0},
    {0x2B, (uint8_t[]){0x00, 0x14, 0x01, 0x2B}, 4, 0},
    {0x2C, (uint8_t[]){0x00}, 1, 0},
};

// 字体加载相关
static const uint8_t *__user_font_getdata(int offset, int size)
{
    // 校验输入参数，防止缓冲区溢出
    if (size <= 0 || size > FONT_BUF_SIZE)
    {
        ESP_LOGE(TAG, "Invalid read size: %d (max: %d)", size, FONT_BUF_SIZE);
        return NULL;
    }

    // 安全的分区查找逻辑（替代assert）
    if (partition_font == NULL)
    {
        // 使用宏定义替代魔法数字，增强可读性
        partition_font = esp_partition_find_first(0x50, 0x32, "font");
        if (partition_font == NULL)
        {
            ESP_LOGE(TAG, "Font partition not found!");
            return NULL;
        }
    }

    // 清空缓冲区（防止残留数据）
    memset(g_font_buf, 0, FONT_BUF_SIZE);

    // 读取字库数据
    esp_err_t err = esp_partition_read(partition_font, offset, g_font_buf, size);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read font data (offset: %d, size: %d), err: %d",
                 offset, size, err);
        return NULL;
    }

    ESP_LOGD(TAG, "Read font data success (offset: %d, size: %d)", offset, size);
    return g_font_buf;
}

static const uint8_t *__user_font_get_bitmap(const lv_font_t *font, uint32_t unicode_letter)
{
    if (unicode_letter > __g_xbf_hd.max || unicode_letter < __g_xbf_hd.min)
    {
        ESP_LOGW(TAG, "Unicode 0x%04X out of range", unicode_letter);
        return NULL;
    }

    uint32_t unicode_offset = sizeof(x_header_t) + (unicode_letter - __g_xbf_hd.min) * 4;
    const uint32_t *p_pos = (const uint32_t *)__user_font_getdata(unicode_offset, 4);

    if (p_pos == NULL || p_pos[0] == 0)
    {
        ESP_LOGW(TAG, "No glyph for Unicode 0x%04X", unicode_letter);
        return NULL;
    }

    uint32_t pos = p_pos[0];
    glyph_dsc_t *gdsc = (glyph_dsc_t *)__user_font_getdata(pos, sizeof(glyph_dsc_t));
    if (gdsc == NULL)
    {
        return NULL;
    }

    return __user_font_getdata(pos + sizeof(glyph_dsc_t), gdsc->box_w * gdsc->box_h * __g_xbf_hd.bpp / 8);
}

static bool __user_font_get_glyph_dsc(const lv_font_t *font, lv_font_glyph_dsc_t *dsc_out, uint32_t unicode_letter, uint32_t unicode_letter_next)
{
    (void)unicode_letter_next; // 未使用参数

    if (unicode_letter > __g_xbf_hd.max || unicode_letter < __g_xbf_hd.min)
    {
        return false;
    }

    uint32_t unicode_offset = sizeof(x_header_t) + (unicode_letter - __g_xbf_hd.min) * 4;
    const uint32_t *p_pos = (const uint32_t *)__user_font_getdata(unicode_offset, 4);

    if (p_pos == NULL || p_pos[0] == 0)
    {
        return false;
    }

    glyph_dsc_t *gdsc = (glyph_dsc_t *)__user_font_getdata(p_pos[0], sizeof(glyph_dsc_t));
    if (gdsc == NULL)
    {
        return false;
    }

    dsc_out->adv_w = gdsc->adv_w;
    dsc_out->box_h = gdsc->box_h;
    dsc_out->box_w = gdsc->box_w;
    dsc_out->ofs_x = gdsc->ofs_x;
    dsc_out->ofs_y = gdsc->ofs_y;
    dsc_out->bpp = __g_xbf_hd.bpp;

    return true;
}

const lv_font_t myFont = {
    .get_glyph_bitmap = __user_font_get_bitmap,
    .get_glyph_dsc = __user_font_get_glyph_dsc,
    .line_height = 17,
    .base_line = 0,
};

// HTTP释放缓冲区
static void free_http_buffer(void)
{
    if (http_buf != NULL)
    {
        free(http_buf);
        http_buf = NULL;
    }
    buf_len = 0;
}

// 按钮事件回调
static void button_event_cb(void *arg, void *data)
{
    button_event_t evt = iot_button_get_event(arg);
    if (!wifi_ok)
    {
        esp_wifi_start();
        wifi_ok = true;
    }
    if (evt == BUTTON_SINGLE_CLICK && !busy && wifi_ok)
    {
        busy = true;
        // xTaskCreate(http_request_task, "http_request_task", 4096, NULL, 2, NULL);
        // xTaskCreate(led_breathe_task, "led_breathe_task", 256, NULL, 4, NULL);
        if (senten)
        {
            esp_wifi_stop();
            wifi_ok = false;
            lvgl_start();
        }
        else
        {

            LedBreathTask_Handler = xTaskCreateStaticPinnedToCore((TaskFunction_t)led_breathe_task,
                                                                  (const char *)"led_breathe_task",
                                                                  (uint32_t)LED_BREATHE_TASK_STK_SIZE,
                                                                  (void *)NULL,
                                                                  (UBaseType_t)LED_BREATHE_TASK_PRIO,
                                                                  (StackType_t *)LedBreathTaskStack,
                                                                  (StaticTask_t *)&LedBreathTaskTCB,
                                                                  (BaseType_t)0);

            vTaskDelay(pdMS_TO_TICKS(100));

            HttpTask_Handler = xTaskCreateStaticPinnedToCore((TaskFunction_t)http_request_task, /* 任务函数 */
                                                             (const char *)"http_task",         /* 任务名称 */
                                                             (uint32_t)HTTP_TASK_STK_SIZE,      /* 任务堆栈大小 */
                                                             (void *)NULL,                      /* 传递给任务函数的参数 */
                                                             (UBaseType_t)HTTP_TASK_PRIO,       /* 任务优先级 */
                                                             (StackType_t *)HttpTaskStack,      /* 任务堆栈 */
                                                             (StaticTask_t *)&HttpTaskTCB,      /* 任务控制块 */
                                                             (BaseType_t)0);                    /* 该任务哪个内核运行 */
        }
    }
    else if (evt == BUTTON_LONG_PRESS_UP)   //长按恢复WiFi配置状态
    {
        uint32_t press_time = iot_button_get_pressed_time(arg);
        if (press_time >= LONG_PRESS_TIME_MS)
        {
            esp_wifi_restore();
            wifi_ok = false;
        }
    }
}

// LED初始化
static void ledc_init(void)
{
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE,
        .duty_resolution = LEDC_DUTY_RES,
        .timer_num = LEDC_TIMER,
        .freq_hz = LEDC_FREQUENCY, // Set output frequency at 4 kHz
        .clk_cfg = LEDC_AUTO_CLK};
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL,
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = LEDC_OUTPUT_IO,
        .duty = 0, // Set duty to 0%
        .hpoint = 0};
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

// LCD初始化
static void lcd_st7789_init(void)
{
    ESP_LOGI(TAG, "Turn off LCD backlight");
    gpio_config_t bk_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << EXAMPLE_PIN_NUM_BK_LIGHT};
    ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));

    ESP_LOGI(TAG, "Initialize SPI bus");
    const spi_bus_config_t buscfg = ILI9341_PANEL_BUS_SPI_CONFIG(EXAMPLE_PIN_NUM_LCD_PCLK, EXAMPLE_PIN_NUM_LCD_DATA0,
                                                                 EXAMPLE_LCD_H_RES * EXAMPLE_LVGL_DRAW_BUF_LINES * sizeof(uint16_t));
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));

    ESP_LOGI(TAG, "Install panel IO");
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = EXAMPLE_PIN_NUM_LCD_DC,
        .cs_gpio_num = EXAMPLE_PIN_NUM_LCD_CS,
        .pclk_hz = EXAMPLE_LCD_PIXEL_CLOCK_HZ,
        .lcd_cmd_bits = EXAMPLE_LCD_CMD_BITS,
        .lcd_param_bits = EXAMPLE_LCD_PARAM_BITS,
        .spi_mode = 0,
        .trans_queue_depth = 10,
    };
    // Attach the LCD to the SPI bus
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &io_handle));

    ili9341_vendor_config_t vendor_config = {
        .init_cmds = lcd_init_cmds,
        .init_cmds_size = sizeof(lcd_init_cmds) / sizeof(ili9341_lcd_init_cmd_t),
    };
    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = EXAMPLE_PIN_NUM_LCD_RST,
        .rgb_endian = LCD_RGB_ENDIAN_RGB,
        .bits_per_pixel = EXAMPLE_LCD_BIT_PER_PIXEL,
        .vendor_config = &vendor_config,
    };
    ESP_LOGI(TAG, "Install ILI9341 panel driver");
    ESP_ERROR_CHECK(esp_lcd_new_panel_ili9341(io_handle, &panel_config, &panel_handle));

    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));

    // ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, false, false));

    // user can flush pre-defined pattern to the screen before we turn on the screen or backlight
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

    // ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel_handle, true));
    // ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, false, true));

    ESP_ERROR_CHECK(esp_lcd_panel_set_gap(panel_handle, 40, 52)); // 显示偏移校正
}

// LVGL初始化及文字显示
static void lvgl_start()
{
    if (!lvgl_ok)
    {
        /* Initialize LVGL */
        const lvgl_port_cfg_t lvgl_cfg = {
            .task_priority = 4,       /* LVGL task priority */
            .task_stack = 4 * 1024,   /* LVGL task stack size */
            .task_affinity = -1,      /* LVGL task pinned to core (-1 is no affinity) */
            .task_max_sleep_ms = 500, /* Maximum sleep in LVGL task */
            .timer_period_ms = 5      /* LVGL timer tick period in ms */
        };
        lvgl_port_init(&lvgl_cfg);

        /* Add LCD screen */
        ESP_LOGD(TAG, "Add LCD screen");
        const lvgl_port_display_cfg_t disp_cfg = {
            .io_handle = io_handle,
            .panel_handle = panel_handle,
            .buffer_size = EXAMPLE_LCD_H_RES * 50,
            .double_buffer = 0,
            .hres = EXAMPLE_LCD_V_RES,
            .vres = EXAMPLE_LCD_H_RES,
            .monochrome = false,
            .rotation = {
                .swap_xy = true,
                .mirror_x = false,
                .mirror_y = true,
            },
            .flags = {
                .buff_dma = false,
            }};
        lvgl_disp = lvgl_port_add_disp(&disp_cfg);
    }

    // ui创建
    if (senten)
    {
        lv_obj_t *origin = lv_label_create(lv_scr_act());
        lv_label_set_long_mode(origin, LV_LABEL_LONG_WRAP);
        lv_obj_align(origin, LV_ALIGN_BOTTOM_RIGHT, 0, 0);
        lv_obj_set_width(origin, EXAMPLE_LCD_V_RES);
        lv_obj_set_height(origin, EXAMPLE_LCD_H_RES - 110);
        lv_obj_set_style_text_align(origin, LV_TEXT_ALIGN_RIGHT, LV_PART_MAIN);
        lv_obj_set_style_pad_all(origin, 5, LV_PART_MAIN);
        lv_obj_set_style_text_font(origin, &myFont, 0);
        lv_label_set_text(origin, sen.origin);

        lv_obj_t *contents = lv_label_create(lv_scr_act());
        lv_label_set_long_mode(contents, LV_LABEL_LONG_WRAP);
        lv_obj_align(contents, LV_ALIGN_CENTER, 0, 0);
        lv_obj_set_width(contents, EXAMPLE_LCD_V_RES);
        lv_obj_set_height(contents, EXAMPLE_LCD_H_RES - 15);
        lv_obj_set_style_text_align(contents, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN);
        lv_obj_set_style_pad_all(contents, 7, LV_PART_MAIN);
        lv_obj_set_style_text_font(contents, &myFont, 0);
        lv_label_set_text(contents, sen.content);

        vTaskDelay(pdMS_TO_TICKS(200));

        ESP_ERROR_CHECK(gpio_set_level(EXAMPLE_PIN_NUM_BK_LIGHT, EXAMPLE_LCD_BK_LIGHT_ON_LEVEL));

        senten = false;
    }
}

static void button_init(uint32_t button_num)
{
    button_config_t btn_cfg = {0};
    btn_cfg.long_press_time = LONG_PRESS_TIME_MS;
    button_gpio_config_t gpio_cfg = {.gpio_num = button_num, .active_level = BUTTON_ACTIVE_LEVEL, .enable_power_save = true};
    button_handle_t btn;
    esp_err_t ret = iot_button_new_gpio_device(&btn_cfg, &gpio_cfg, &btn);
    assert(ret == ESP_OK);
    iot_button_register_cb(btn, BUTTON_SINGLE_CLICK, NULL, button_event_cb, NULL);
    iot_button_register_cb(btn, BUTTON_LONG_PRESS_UP, NULL, button_event_cb, NULL);
}

// Wi-Fi事件回调
static void wifi_event_cb(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        // xTaskCreate(smartconfig_task, "smartconfig_task", 2048, NULL, 3, NULL);
        SmartconfigTask_Handler = xTaskCreateStaticPinnedToCore((TaskFunction_t)smartconfig_task,    /* 任务函数 */
                                                                (const char *)"smartconfig_task",    /* 任务名称 */
                                                                (uint32_t)SMARTCONFIG_TASK_SIZE,     /* 任务堆栈大小 */
                                                                (void *)NULL,                        /* 传递给任务函数的参数 */
                                                                (UBaseType_t)SMARTCONFIG_TASK_PRIO,  /* 任务优先级 */
                                                                (StackType_t *)SmartconfigTaskStack, /* 任务堆栈 */
                                                                (StaticTask_t *)&SmartconfigTaskTCB, /* 任务控制块 */
                                                                (BaseType_t)0);                      /* 该任务哪个内核运行 */

        LedBlinkTask_Handler = xTaskCreateStaticPinnedToCore((TaskFunction_t)led_blink_task,
                                                             (const char *)"led_blink_task",
                                                             (uint32_t)LED_BLINK_TASK_STK_SIZE,
                                                             (void *)NULL,
                                                             (UBaseType_t)LED_BLINK_TASK_PRIO,
                                                             (StackType_t *)LedBlinkTaskStack,
                                                             (StaticTask_t *)&LedBlinkTaskTCB,
                                                             (BaseType_t)0);
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        wifi_ok = false;
        esp_wifi_connect();
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
        wifi_ok = true;
    else if (event_base == SC_EVENT && event_id == SC_EVENT_GOT_SSID_PSWD)
    {
        smartconfig_event_got_ssid_pswd_t *evt = event_data;
        wifi_config_t cfg = {0};
        memcpy(cfg.sta.ssid, evt->ssid, sizeof(cfg.sta.ssid));
        memcpy(cfg.sta.password, evt->password, sizeof(cfg.sta.password));
        esp_wifi_disconnect();
        esp_wifi_set_config(WIFI_IF_STA, &cfg);
        esp_wifi_connect();
        lvgl_start();
    }
    else if (event_base == SC_EVENT && event_id == SC_EVENT_SEND_ACK_DONE)
        esp_smartconfig_stop();
}

// SmartConfig任务
static void smartconfig_task(void *arg)
{
    wifi_config_t cfg = {0};
    esp_wifi_get_config(WIFI_IF_STA, &cfg);
    if (strlen((char *)cfg.sta.ssid))
        esp_wifi_connect();
    else
    {
        esp_smartconfig_set_type(SC_TYPE_ESPTOUCH_AIRKISS); // 支持ESP-TOUCH和AIRKISS两种配网协议
        smartconfig_start_config_t sc_cfg = SMARTCONFIG_START_CONFIG_DEFAULT();
        esp_smartconfig_start(&sc_cfg);
    }
    xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT, pdTRUE, pdFALSE, portMAX_DELAY);
    vTaskDelete(NULL);
}

// Wi-Fi初始化
static void wifi_init(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        nvs_flash_erase();
        nvs_flash_init();
    }
    esp_netif_init();
    s_wifi_event_group = xEventGroupCreate();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_cb, NULL, NULL);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_cb, NULL, NULL);
    esp_event_handler_instance_register(SC_EVENT, ESP_EVENT_ANY_ID, wifi_event_cb, NULL, NULL);
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_start();
}

// jsion解析
static bool parse_sentence(const char *json, sentence_t *sen)
{
    cJSON *root = cJSON_Parse(json);
    if (!root)
        return false;
    bool res = false;

#if defined(XYGENG_API_F)
    cJSON *code = cJSON_GetObjectItem(root, "code");
    cJSON *data = cJSON_GetObjectItem(root, "data");
    if (cJSON_IsNumber(code) && code->valueint == 200 && data)
    {
        cJSON *cont = cJSON_GetObjectItem(data, "content");
        cJSON *orig = cJSON_GetObjectItem(data, "origin");
        if (cJSON_IsString(cont) && cont->valuestring)
        {
            strlcpy(sen->content, cont->valuestring, sizeof(sen->content));
            strlcpy(sen->origin, cJSON_IsString(orig) && orig->valuestring ? orig->valuestring : "未知", sizeof(sen->origin));
            res = true;
        }
    }
#elif defined(XYGENG_API_S)
    cJSON *code = cJSON_GetObjectItem(root, "code");
    cJSON *data = cJSON_GetObjectItem(root, "data");
    if (cJSON_IsNumber(code) && code->valueint == 200 && data)
    {
        cJSON *cont = cJSON_GetObjectItem(data, "content");
        cJSON *orig = cJSON_GetObjectItem(data, "origin");
        if (cJSON_IsString(cont) && cont->valuestring)
        {
            strlcpy(sen->content, cont->valuestring, sizeof(sen->content));
            strlcpy(sen->origin, cJSON_IsString(orig) && orig->valuestring ? orig->valuestring : "未知", sizeof(sen->origin));
            res = true;
        }
    }
#elif defined(HITOKOTO_API)
    cJSON *cont = cJSON_GetObjectItem(root, "hitokoto");
    cJSON *orig = cJSON_GetObjectItem(root, "from");
    if (cJSON_IsString(cont) && cont->valuestring)
    {
        strlcpy(sen->content, cont->valuestring, sizeof(sen->content));
        strlcpy(sen->origin, cJSON_IsString(orig) && orig->valuestring ? orig->valuestring : "佚名", sizeof(sen->origin));
        res = true;
    }
#endif

    cJSON_Delete(root);
    return res;
}

// http回调
static esp_err_t http_event_cb(esp_http_client_event_t *evt)
{
    switch (evt->event_id)
    {
    case HTTP_EVENT_ON_CONNECTED:
        free_http_buffer();
        break;
    case HTTP_EVENT_ON_DATA:
        if (!esp_http_client_is_chunked_response(evt->client) && evt->data_len > 0)
        {
            char *tmp = realloc(http_buf, buf_len + evt->data_len + 1);
            if (tmp)
            {
                http_buf = tmp;
                memcpy(http_buf + buf_len, evt->data, evt->data_len);
                buf_len += evt->data_len;
                http_buf[buf_len] = '\0';
            }
        }
        break;
    case HTTP_EVENT_ON_FINISH:
    case HTTP_EVENT_ERROR:
        break;
    default:
        break;
    }
    return ESP_OK;
}

// http任务
static void http_request_task(void *arg)
{
    if (!wifi_ok)
        esp_wifi_start();
    // sentence_t sen = {0};
    bool req_success = false;
    for (int i = 1; i <= HTTP_REQ_TIMES && !req_success; i++)
    {
        ESP_LOGI(TAG, "req %d", i);
        free_http_buffer();
        esp_http_client_config_t cfg = {
#if defined(XYGENG_API_F)
            .url = XYGENG_API_F,
#elif defined(XYGENG_API_S)
            .url = XYGENG_API_S,
#elif defined(HITOKOTO_API)
            .url = HITOKOTO_API,
#endif
            .timeout_ms = HTTP_TIMEOUT_MS,
            .event_handler = http_event_cb,
            .transport_type = HTTP_TRANSPORT_OVER_SSL,
            .skip_cert_common_name_check = true};

        esp_http_client_handle_t cli = esp_http_client_init(&cfg);
        if (cli)
        {
            esp_err_t ret = esp_http_client_perform(cli);
            if (ret == ESP_OK && esp_http_client_get_status_code(cli) == 200 && http_buf)
            {
                if (parse_sentence(http_buf, &sen)) // 解析成功
                {
                    // ESP_LOGI(TAG, ">>%s", sen.content);
                    // ESP_LOGI(TAG, "  %s\n", sen.origin);

                    senten = true;
                    req_success = true;
                    if (LedBreathTask_Handler != NULL)
                    {
                        vTaskDelete(LedBreathTask_Handler);
                        LedBreathTask_Handler = NULL;
                        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0);
                        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
                    }

                    LedBlinkTask_Handler = xTaskCreateStaticPinnedToCore((TaskFunction_t)led_blink_task,
                                                                         (const char *)"led_blink_task",
                                                                         (uint32_t)LED_BLINK_TASK_STK_SIZE,
                                                                         (void *)NULL,
                                                                         (UBaseType_t)LED_BLINK_TASK_PRIO,
                                                                         (StackType_t *)LedBlinkTaskStack,
                                                                         (StaticTask_t *)&LedBlinkTaskTCB,
                                                                         (BaseType_t)0);
                    vTaskDelay(pdMS_TO_TICKS(3200));
                }
                else
                    ESP_LOGI(TAG, "parse err");
            }
            else
                ESP_LOGI(TAG, "req err");
            esp_http_client_cleanup(cli);
        }

        free_http_buffer();
        if (i < HTTP_REQ_TIMES && !req_success)
            vTaskDelay(pdMS_TO_TICKS(HTTP_REQ_DELAY_MS));
    }
    free_http_buffer();

    // led状态重置
    if (LedBreathTask_Handler != NULL)
    {
        vTaskDelete(LedBreathTask_Handler);
        LedBreathTask_Handler = NULL;
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
    }

    // if (!req_success)
    // {
    //     LedBlinkTask_Handler = xTaskCreateStaticPinnedToCore((TaskFunction_t)led_blink_task,
    //                                                          (const char *)"led_blink_task",
    //                                                          (uint32_t)LED_BLINK_TASK_STK_SIZE,
    //                                                          (void *)NULL,
    //                                                          (UBaseType_t)LED_BLINK_TASK_PRIO,
    //                                                          (StackType_t *)LedBlinkTaskStack,
    //                                                          (StaticTask_t *)&LedBlinkTaskTCB,
    //                                                          (BaseType_t)0);
    //     vTaskDelay(pdMS_TO_TICKS(3200));
    // }
    busy = false;
    vTaskDelete(NULL);
}

// LED呼吸任务
static void led_breathe_task(void *arg)
{
    while (1)
    {
        for (int i = 0; i < 4096; i += 40)
        {
            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, i);  // 状态更新
            ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
            vTaskDelay(pdMS_TO_TICKS(20));
        }
        for (int i = 4096; i > 0; i -= 40)
        {
            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, i);
            ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
            vTaskDelay(pdMS_TO_TICKS(20));
        }
        if (!busy)
        {
            vTaskDelete(NULL);
            LedBreathTask_Handler = NULL;
            break;
        }
    }
}

// LED闪烁任务
static void led_blink_task(void *arg)
{
    const int TOTAL_DURATION = 1500;
    // 单次闪烁周期（亮+灭，可调整速度，越小闪得越快）
    const int FLASH_CYCLE = 150; // 100ms/次 = 10次/秒
    // LED亮/灭的占空比（0=灭，800=最亮）
    const int LED_BRIGHTNESS = 1000;

    TickType_t start_time = xTaskGetTickCount();
    TickType_t current_time;

    while (1)
    {
        current_time = xTaskGetTickCount();

        int elapsed_time = pdTICKS_TO_MS(current_time - start_time);

        if (elapsed_time >= TOTAL_DURATION)
        {
            break;
        }

        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LED_BRIGHTNESS);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);

        vTaskDelay(pdMS_TO_TICKS(FLASH_CYCLE / 2));

        if (pdTICKS_TO_MS(xTaskGetTickCount() - start_time) >= TOTAL_DURATION)
        {
            break;
        }

        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);

        vTaskDelay(pdMS_TO_TICKS(FLASH_CYCLE / 2));

        if (!busy)
        {
            vTaskDelete(NULL);
            break;
        }
    }
    vTaskDelete(NULL);
    LedBlinkTask_Handler = NULL;
}

void app_main(void)
{
    // 初始化硬件
    ledc_init();
    button_init(BOOT_BUTTON_NUM);
    wifi_init();
    lcd_st7789_init();
}