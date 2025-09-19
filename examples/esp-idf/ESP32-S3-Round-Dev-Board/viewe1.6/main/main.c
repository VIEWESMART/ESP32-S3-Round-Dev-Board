#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "driver/gpio_filter.h"
#include "esp_heap_caps.h"
#include "esp_err.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_lcd_panel_io_interface.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_st77903_qspi.h"
#include "qmi8658.h"
#include "sd3078.h"
#include "driver/i2c_master.h"
#include "esp_lcd_touch_cst3530.h"
#include "esp_lcd_touch.h"
#include "driver/pulse_cnt.h"
#include "esp_timer.h"
#include "lvgl_port.h"
#include "lvgl.h"
#define LV_TICK_PERIOD_MS 1

#define LCD_HOST (SPI2_HOST)
#define LCD_H_RES (400) // 宽
#define LCD_V_RES (400) // 高
#define LCD_BIT_PER_PIXEL (16)
#define LCD_FB_BUF_NUM (1)
#define PIN_NUM_LCD_CS (GPIO_NUM_12)
#define PIN_NUM_LCD_PCLK (GPIO_NUM_48)
#define PIN_NUM_LCD_DATA0 (GPIO_NUM_13)
#define PIN_NUM_LCD_DATA1 (GPIO_NUM_47)
#define PIN_NUM_LCD_DATA2 (GPIO_NUM_21)
#define PIN_NUM_LCD_DATA3 (GPIO_NUM_14)
#define PIN_NUM_LCD_RST (GPIO_NUM_11)
#define PIN_NUM_LCD_DC (GPIO_NUM_NC)
#define LCD_BK_LIGHT_ON_LEVEL 1
#define LCD_BK_LIGHT_OFF_LEVEL !LCD_BK_LIGHT_ON_LEVEL
#define PIN_NUM_BK_LIGHT (GPIO_NUM_45)
// 触摸设置
#define TOUCH_I2C_NUM (0)
#define TOUCH_I2C_CLK_HZ (400000)

// 触摸引脚
#define TOUCH_I2C_SCL (GPIO_NUM_18)
#define TOUCH_I2C_SDA (GPIO_NUM_17)
#define TOUCH_I2C_RST (GPIO_NUM_10)
#define TOUCH_I2C_INT (GPIO_NUM_9)
// 按钮
#define ENCODER_KEY (GPIO_NUM_0)
static char *TAG = "屏幕测试";

extern void lvgl_demo_ui(lv_disp_t *disp);
/* LCD IO and panel */
static esp_lcd_panel_io_handle_t lcd_io = NULL;
static esp_lcd_panel_handle_t lcd_panel = NULL;
static esp_lcd_touch_handle_t touch_handle = NULL;

static esp_err_t app_lcd_init(void)
{
    esp_err_t ret = ESP_OK;

    /* LCD backlight */
    gpio_config_t bk_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << PIN_NUM_BK_LIGHT};
    ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));

    ESP_LOGI(TAG, "安装ST77903面板驱动");
    st77903_qspi_config_t qspi_config = ST77903_QSPI_CONFIG_DEFAULT(LCD_HOST,
                                                                    PIN_NUM_LCD_CS,
                                                                    PIN_NUM_LCD_PCLK,
                                                                    PIN_NUM_LCD_DATA0,
                                                                    PIN_NUM_LCD_DATA1,
                                                                    PIN_NUM_LCD_DATA2,
                                                                    PIN_NUM_LCD_DATA3,
                                                                    LCD_FB_BUF_NUM,
                                                                    LCD_H_RES,
                                                                    LCD_V_RES);
    st77903_vendor_config_t vendor_config = {
        .qspi_config = &qspi_config,
        // .init_cmds = lcd_init_cmds,      // Uncomment these line if use custom initialization commands
        // .init_cmds_size = sizeof(lcd_init_cmds) / sizeof(lcd_init_cmds[0]),
        .flags = {
            .mirror_by_cmd = 1,
        },
    };
    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = PIN_NUM_LCD_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR,
        .bits_per_pixel = LCD_BIT_PER_PIXEL,
        .vendor_config = &vendor_config,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_st77903_qspi(&panel_config, &lcd_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(lcd_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(lcd_panel, true, false));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(lcd_panel, true));
    ESP_ERROR_CHECK(esp_lcd_panel_init(lcd_panel));

    return ret;
}

static esp_err_t app_touch_init(void)
{
    /* 初始化 I2C */
    ESP_LOGI(TAG, "安装CST3530触摸驱动");
    gpio_config_t touch_rst_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << TOUCH_I2C_RST};
    ESP_ERROR_CHECK(gpio_config(&touch_rst_config));
    gpio_config_t touch_int_config = {
        .mode = GPIO_MODE_INPUT_OUTPUT,
        .pin_bit_mask = 1ULL << TOUCH_I2C_INT};
    ESP_ERROR_CHECK(gpio_config(&touch_int_config));
    gpio_set_level(TOUCH_I2C_INT, 0);
    gpio_set_level(TOUCH_I2C_RST, 0);
    vTaskDelay(200);
    gpio_set_level(TOUCH_I2C_RST, 1);
    vTaskDelay(200);

    const i2c_master_bus_config_t i2c_conf = {
        .i2c_port = 0,
        .sda_io_num = TOUCH_I2C_SDA,
        .scl_io_num = TOUCH_I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_conf, &i2c_bus));

    uint8_t address;
    printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\r\n");
    for (int i = 0; i < 128; i += 16)
    {
        printf("%02x: ", i);
        for (int j = 0; j < 16; j++)
        {
            fflush(stdout);
            address = i + j;
            esp_err_t ret = i2c_master_probe(i2c_bus, address, pdMS_TO_TICKS(500));
            if (ret == ESP_OK)
            {
                printf("%02x ", address);
            }
            else if (ret == ESP_ERR_TIMEOUT)
            {
                printf("UU ");
            }
            else
            {
                printf("-- ");
            }
        }
        printf("\r\n");
    }

    // 初始化触摸屏
    esp_lcd_panel_io_i2c_config_t tp_config = ESP_LCD_TOUCH_IO_I2C_CST3530_CONFIG();
    tp_config.scl_speed_hz = TOUCH_I2C_CLK_HZ;
    esp_lcd_touch_config_t tp_cfg = {
        .x_max = LCD_H_RES,
        .y_max = LCD_V_RES,
        .rst_gpio_num = TOUCH_I2C_RST,
        .int_gpio_num = TOUCH_I2C_INT,
        .levels = {
            .reset = 0,
            .interrupt = 0,
        },
        .flags = {
            .swap_xy = 1,
            .mirror_x = 0,
            .mirror_y = 0,
        },
    };
    if (tp_cfg.int_gpio_num != GPIO_NUM_NC) {
        init_touch_isr_mux();
        tp_cfg.interrupt_callback = lvgl_port_touch_isr_cb;
    }
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(i2c_bus, &tp_config, &lcd_io));
    ESP_ERROR_CHECK(esp_lcd_touch_new_i2c_cst3530(lcd_io, &tp_cfg, &touch_handle));
    return ESP_OK;
}

static void app_main_display(void)
{
    ESP_ERROR_CHECK(gpio_set_level(PIN_NUM_BK_LIGHT, LCD_BK_LIGHT_ON_LEVEL));
    if (lvgl_port_lock(-1)) 
    {

        lvgl_demo_ui(lv_disp_get_default());

        lvgl_port_unlock();
    }
}

void app_main()
{
    ESP_ERROR_CHECK(app_lcd_init());
    ESP_ERROR_CHECK(app_touch_init());
    ESP_ERROR_CHECK(lvgl_port_init(lcd_panel, touch_handle));
    app_main_display();
}

