/*
 * SPDX-FileCopyrightText: 2015-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_system.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_touch.h"

#define POINT_NUM_MAX (1)

#define DATA_START_REG (0xd000)
#define CHIP_ID_REG (0xA7)

static const char *TAG = "CST9217";

static esp_err_t read_data(esp_lcd_touch_handle_t tp);
static bool get_xy(esp_lcd_touch_handle_t tp, uint16_t *x, uint16_t *y, uint16_t *strength, uint8_t *point_num, uint8_t max_point_num);
static esp_err_t del(esp_lcd_touch_handle_t tp);

static esp_err_t i2c_read_bytes(esp_lcd_touch_handle_t tp, uint16_t reg, uint8_t *data, uint8_t len);
static esp_err_t reset(esp_lcd_touch_handle_t tp);
static esp_err_t read_res(esp_lcd_touch_handle_t tp);

esp_err_t esp_lcd_touch_new_i2c_cst9217(const esp_lcd_panel_io_handle_t io, const esp_lcd_touch_config_t *config, esp_lcd_touch_handle_t *tp)
{
    ESP_RETURN_ON_FALSE(io, ESP_ERR_INVALID_ARG, TAG, "无效的接口");
    ESP_RETURN_ON_FALSE(config, ESP_ERR_INVALID_ARG, TAG, "无效的配置");
    ESP_RETURN_ON_FALSE(tp, ESP_ERR_INVALID_ARG, TAG, "无效的触摸句柄");

    esp_err_t ret = ESP_OK;
    esp_lcd_touch_handle_t cst9217 = calloc(1, sizeof(esp_lcd_touch_t));
    ESP_GOTO_ON_FALSE(cst9217, ESP_ERR_NO_MEM, err, TAG, "触摸句柄分配内存失败");

    cst9217->io = io;
    cst9217->read_data = read_data;
    cst9217->get_xy = get_xy;
    cst9217->del = del;
    cst9217->data.lock.owner = portMUX_FREE_VAL;
    memcpy(&cst9217->config, config, sizeof(esp_lcd_touch_config_t));

    if (cst9217->config.int_gpio_num != GPIO_NUM_NC)
    {
        const gpio_config_t int_gpio_config = {
            .mode = GPIO_MODE_INPUT,
            .intr_type = (cst9217->config.levels.interrupt ? GPIO_INTR_POSEDGE : GPIO_INTR_NEGEDGE),
            .pin_bit_mask = BIT64(cst9217->config.int_gpio_num)};
        ESP_GOTO_ON_ERROR(gpio_config(&int_gpio_config), err, TAG, "GPIO配置失败");

        if (cst9217->config.interrupt_callback)
        {
            esp_lcd_touch_register_interrupt_callback(cst9217, cst9217->config.interrupt_callback);
        }
    }

    if (cst9217->config.rst_gpio_num != GPIO_NUM_NC)
    {
        const gpio_config_t rst_gpio_config = {
            .mode = GPIO_MODE_OUTPUT,
            .pin_bit_mask = BIT64(cst9217->config.rst_gpio_num)};
        ESP_GOTO_ON_ERROR(gpio_config(&rst_gpio_config), err, TAG, "GPIO复位配置失败");
    }
    ESP_GOTO_ON_ERROR(reset(cst9217), err, TAG, "复位失败");
    esp_lcd_panel_io_tx_param(cst9217->io, 0xd101, NULL, 0);
    ESP_GOTO_ON_ERROR(read_res(cst9217), err, TAG, "读取分辨率失败");
    *tp = cst9217;

    return ESP_OK;
err:
    if (cst9217)
    {
        del(cst9217);
    }
    ESP_LOGE(TAG, "初始化失败!");
    return ret;
}

static esp_err_t read_data(esp_lcd_touch_handle_t tp)
{
    typedef struct
    {
        uint8_t id;
        uint8_t x_h;
        uint8_t y_h;
        uint8_t xy_l;
        uint8_t press_value;
        uint8_t key_value;
        uint8_t flag;
    } data_t;

    data_t point;
    ESP_RETURN_ON_ERROR(i2c_read_bytes(tp, DATA_START_REG, (uint8_t *)&point, sizeof(data_t)), TAG, "I2C读取失败！");
    esp_lcd_panel_io_tx_param(tp->io, 0xd000ab, NULL, 0);
    point.key_value = point.key_value & 0x0f;
    portENTER_CRITICAL(&tp->data.lock);
    point.key_value = (point.key_value > POINT_NUM_MAX ? POINT_NUM_MAX : point.key_value);
    tp->data.points = point.key_value;
    for (int i = 0; i < point.key_value; i++)
    {
        tp->data.coords[i].x = (point.x_h << 4) | ((point.xy_l >> 4) & 0x0f);
        tp->data.coords[i].y = (point.y_h << 4) | (point.xy_l & 0x0f);
        tp->data.coords[i].strength = point.press_value;
    }
    portEXIT_CRITICAL(&tp->data.lock);

    return ESP_OK;
}

static bool get_xy(esp_lcd_touch_handle_t tp, uint16_t *x, uint16_t *y, uint16_t *strength, uint8_t *point_num, uint8_t max_point_num)
{
    portENTER_CRITICAL(&tp->data.lock);

    *point_num = (tp->data.points > max_point_num ? max_point_num : tp->data.points);
    for (size_t i = 0; i < *point_num; i++)
    {
        x[i] = tp->data.coords[i].x;
        y[i] = tp->data.coords[i].y;

        if (strength)
        {
            strength[i] = tp->data.coords[i].strength;
        }
    }

    tp->data.points = 0;
    portEXIT_CRITICAL(&tp->data.lock);

    return (*point_num > 0);
}

static esp_err_t del(esp_lcd_touch_handle_t tp)
{

    if (tp->config.int_gpio_num != GPIO_NUM_NC)
    {
        gpio_reset_pin(tp->config.int_gpio_num);
        if (tp->config.interrupt_callback)
        {
            gpio_isr_handler_remove(tp->config.int_gpio_num);
        }
    }
    if (tp->config.rst_gpio_num != GPIO_NUM_NC)
    {
        gpio_reset_pin(tp->config.rst_gpio_num);
    }

    free(tp);

    return ESP_OK;
}

static esp_err_t reset(esp_lcd_touch_handle_t tp)
{
    if (tp->config.rst_gpio_num != GPIO_NUM_NC)
    {
        ESP_RETURN_ON_ERROR(gpio_set_level(tp->config.rst_gpio_num, tp->config.levels.reset), TAG, "GPIO设置电平失败");
        vTaskDelay(pdMS_TO_TICKS(200));
        ESP_RETURN_ON_ERROR(gpio_set_level(tp->config.rst_gpio_num, !tp->config.levels.reset), TAG, "GPIO设置电平失败");
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    return ESP_OK;
}

static esp_err_t read_res(esp_lcd_touch_handle_t tp)
{
    uint8_t res[4] = {0};
    uint16_t x_res, y_res;
    ESP_RETURN_ON_ERROR(i2c_read_bytes(tp, 0XD1F8, res, 4), TAG, "I2C读取失败");
    x_res = (res[1] << 8) | res[0];
    y_res = (res[3] << 8) | res[2];
    ESP_LOGI(TAG, "触摸分辨率: X=%d,Y=%d", x_res, y_res);
    return ESP_OK;
}

static esp_err_t i2c_read_bytes(esp_lcd_touch_handle_t tp, uint16_t reg, uint8_t *data, uint8_t len)
{
    ESP_RETURN_ON_FALSE(data, ESP_ERR_INVALID_ARG, TAG, "无效的数据");

    return esp_lcd_panel_io_rx_param(tp->io, reg, data, len);
}