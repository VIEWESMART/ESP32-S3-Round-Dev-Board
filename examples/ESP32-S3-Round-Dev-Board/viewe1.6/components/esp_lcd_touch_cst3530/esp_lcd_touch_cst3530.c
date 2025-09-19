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

#define DATA_START_REG (0x00)
#define CHIP_ID_REG (0xA7)
#define NomalWorkModeRegister 0xD0070000
#define ReportCoordinates 0xD00002AB
static const char *TAG = "CST3530";

static esp_err_t read_data(esp_lcd_touch_handle_t tp);
static bool get_xy(esp_lcd_touch_handle_t tp, uint16_t *x, uint16_t *y, uint16_t *strength, uint8_t *point_num, uint8_t max_point_num);
static esp_err_t del(esp_lcd_touch_handle_t tp);

static esp_err_t i2c_read_bytes(esp_lcd_touch_handle_t tp, uint16_t reg, uint8_t *data, uint8_t len);
static esp_err_t reset(esp_lcd_touch_handle_t tp);
static esp_err_t read_reg(esp_lcd_touch_handle_t tp);

esp_err_t esp_lcd_touch_new_i2c_cst3530(const esp_lcd_panel_io_handle_t io, const esp_lcd_touch_config_t *config, esp_lcd_touch_handle_t *tp)
{
    ESP_RETURN_ON_FALSE(io, ESP_ERR_INVALID_ARG, TAG, "无效的接口");
    ESP_RETURN_ON_FALSE(config, ESP_ERR_INVALID_ARG, TAG, "无效的配置");
    ESP_RETURN_ON_FALSE(tp, ESP_ERR_INVALID_ARG, TAG, "无效的触摸句柄");

    esp_err_t ret = ESP_OK;
    esp_lcd_touch_handle_t cst3530 = calloc(1, sizeof(esp_lcd_touch_t));
    ESP_GOTO_ON_FALSE(cst3530, ESP_ERR_NO_MEM, err, TAG, "触摸句柄分配内存失败");

    cst3530->io = io;
    cst3530->read_data = read_data;
    cst3530->get_xy = get_xy;
    cst3530->del = del;
    cst3530->data.lock.owner = portMUX_FREE_VAL;
    memcpy(&cst3530->config, config, sizeof(esp_lcd_touch_config_t));

    if (cst3530->config.int_gpio_num != GPIO_NUM_NC)
    {
        const gpio_config_t int_gpio_config = {
            .mode = GPIO_MODE_INPUT,
            .intr_type = (cst3530->config.levels.interrupt ? GPIO_INTR_POSEDGE : GPIO_INTR_NEGEDGE),
            .pin_bit_mask = BIT64(cst3530->config.int_gpio_num)};
        ESP_GOTO_ON_ERROR(gpio_config(&int_gpio_config), err, TAG, "GPIO配置失败");

        if (cst3530->config.interrupt_callback)
        {
            esp_lcd_touch_register_interrupt_callback(cst3530, cst3530->config.interrupt_callback);
        }
    }

    if (cst3530->config.rst_gpio_num != GPIO_NUM_NC)
    {
        const gpio_config_t rst_gpio_config = {
            .mode = GPIO_MODE_OUTPUT,
            .pin_bit_mask = BIT64(cst3530->config.rst_gpio_num)};
        ESP_GOTO_ON_ERROR(gpio_config(&rst_gpio_config), err, TAG, "GPIO复位配置失败");
    }
    ESP_GOTO_ON_ERROR(reset(cst3530), err, TAG, "复位失败");

    // ESP_GOTO_ON_ERROR(read_reg(cst3530), err, TAG, "读取寄存器失败");
    *tp = cst3530;

    return ESP_OK;
err:
    if (cst3530)
    {
        del(cst3530);
    }
    ESP_LOGE(TAG, "初始化失败!");
    return ret;
}

static esp_err_t read_data(esp_lcd_touch_handle_t tp)
{
    typedef struct
    {
        uint8_t checksum_l;
        uint8_t checksum_h;
        uint8_t type;
        uint8_t num;
        uint8_t x_l;
        uint8_t y_l;
        uint8_t press_value;
        uint8_t xy_h;
        uint8_t id;
    } data_t;

    data_t point;
    // uint8_t buf[2]={0};
    // esp_lcd_panel_io_tx_param(tp->io, 0xd007, &buf, 2);
    uint8_t buf[3] = {0};
    buf[0] = (uint8_t)((NomalWorkModeRegister & 0x00ff0000) >> 16);
    buf[1] = (uint8_t)((NomalWorkModeRegister & 0x0000ff00) >> 8);
    buf[2] = (uint8_t)(NomalWorkModeRegister & 0x000000ff);
    //  i2c_read_bytes(tp, i, (uint8_t *)&buf, sizeof(buf));
    esp_lcd_panel_io_tx_param(tp->io, (uint8_t)(NomalWorkModeRegister >> 24), &buf, 3); // write stare
    uint8_t buf2[30] = {0};
    // ESP_RETURN_ON_ERROR(i2c_read_bytes(tp, DATA_START_REG, (uint8_t *)&point, sizeof(data_t)), TAG, "I2C读取失败！");
    ESP_RETURN_ON_ERROR(i2c_read_bytes(tp, 0x00, buf2, sizeof(buf2)), TAG, "I2C read failed");

    uint8_t buf3[3] = {0};
    buf3[0] = (uint8_t)((ReportCoordinates & 0x00ff0000) >> 16);
    buf3[1] = (uint8_t)((ReportCoordinates & 0x0000ff00) >> 8);
    buf3[2] = (uint8_t)(ReportCoordinates & 0x000000ff);
    //  i2c_read_bytes(tp, i, (uint8_t *)&buf, sizeof(buf));
    esp_lcd_panel_io_tx_param(tp->io, ReportCoordinates >> 24, &buf3, 3); // write  end

    point.num = buf2[3] & 0x0f;
    portENTER_CRITICAL(&tp->data.lock);
    point.num = (point.num > POINT_NUM_MAX ? POINT_NUM_MAX : point.num);
    tp->data.points = point.num;
    for (int i = 0; i < point.num; i++)
    {
        tp->data.coords[i].x = buf2[4] + ((uint16_t)(buf2[7]&0x0F) <<8);
        tp->data.coords[i].y = buf2[5] + ((uint16_t)(buf2[7]&0xf0) <<4);
        tp->data.coords[i].strength = buf2[6];
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
static esp_err_t read_reg(esp_lcd_touch_handle_t tp)
{
    uint8_t reg[26] = {0};
    // ESP_RETURN_ON_ERROR(i2c_read_bytes(tp, 0XD000, reg, 26), TAG, "I2C读取失败");
    esp_lcd_panel_io_rx_param(tp->io, 0xD000, reg, 26);
    // ESP_LOGI(TAG, "触摸分辨率: X=%x,Y=%x", reg[0], reg[1]);
    printf("REG1:%X\n", reg[0]);
    printf("REG2:%X\n", reg[1]);
    printf("REG3:%X\n", reg[2]);
    printf("REG4:%X\n", reg[3]);
    printf("REG5:%X\n", reg[4]);
    printf("REG6:%X\n", reg[5]);
    printf("REG7:%X\n", reg[6]);
    printf("REG8:%X\n", reg[7]);
    printf("REG9:%X\n", reg[8]);
    printf("REG10:%X\n", reg[9]);
    printf("REG11:%X\n", reg[10]);
    printf("REG12:%X\n", reg[11]);
    printf("REG13:%X\n", reg[12]);
    printf("REG14:%X\n", reg[13]);
    printf("REG15:%X\n", reg[14]);
    printf("REG16:%X\n", reg[15]);
    printf("REG17:%X\n", reg[16]);
    printf("REG18:%X\n", reg[17]);
    printf("REG19:%X\n", reg[18]);
    printf("REG20:%X\n", reg[19]);
    printf("REG21:%X\n", reg[20]);
    printf("REG22:%X\n", reg[21]);
    printf("REG23:%X\n", reg[22]);
    printf("REG24:%X\n", reg[23]);
    printf("REG25:%X\n", reg[24]);
    printf("REG26:%X\n", reg[25]);
    return ESP_OK;
}

static esp_err_t i2c_read_bytes(esp_lcd_touch_handle_t tp, uint16_t reg, uint8_t *data, uint8_t len)
{
    ESP_RETURN_ON_FALSE(data, ESP_ERR_INVALID_ARG, TAG, "无效的数据");

    return esp_lcd_panel_io_rx_param(tp->io, reg, data, len);
}