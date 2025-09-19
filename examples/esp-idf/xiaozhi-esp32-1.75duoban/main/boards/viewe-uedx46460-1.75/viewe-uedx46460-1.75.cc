#include "wifi_board.h"
#include "audio_codecs/no_audio_codec.h"
#include "display/lcd_display.h"
#include "application.h"
#include "config.h"
#include "iot/thing_manager.h"
#include "button.h"
#include <esp_log.h>
#include "i2c_device.h"
#include <driver/i2c.h>
#include <driver/ledc.h>
#include <wifi_station.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include "esp_lcd_co5300.h"
#include <esp_timer.h>
#include "esp_system.h"
#define TAG "viewe-uedx46460-1.75"

LV_FONT_DECLARE(font_puhui_30_4);
LV_FONT_DECLARE(font_awesome_30_1);

#define LCD_OPCODE_WRITE_CMD (0x02ULL)
#define LCD_OPCODE_READ_CMD (0x03ULL)
#define LCD_OPCODE_WRITE_COLOR (0x32ULL)

static const co5300_lcd_init_cmd_t lcd_init_cmds[] = {
    {0xFE, (uint8_t[]){0x00}, 0, 0},
    {0xC4, (uint8_t[]){0x80}, 1, 0},
    {0x3A, (uint8_t[]){0x55}, 1, 0},
    {0x35, (uint8_t[]){0x00}, 1, 10},
    {0x53, (uint8_t[]){0x20}, 1, 10}, // 亮度控制开关
    {0x51, (uint8_t[]){0xFF}, 1, 10}, // 亮度0-ff
    {0x63, (uint8_t[]){0xFF}, 1, 10},
    {0x2A, (uint8_t[]){0x00, 0x06, 0x01, 0xDD}, 4, 0}, // 列设置，从0-472
    {0x2B, (uint8_t[]){0x00, 0x00, 0x01, 0xD1}, 4, 0}, // 行设置，从0-465
    {0x11, (uint8_t[]){0x00}, 0, 60},
    {0x29, (uint8_t[]){0x00}, 0, 0},
};

class Cst816s : public I2cDevice
{
public:
    struct TouchPoint_t
    {
        int num = 0;
        int x = -1;
        int y = -1;
    };
    Cst816s(i2c_master_bus_handle_t i2c_bus, uint8_t addr) : I2cDevice(i2c_bus, addr)
    {
        uint8_t chip_id = ReadReg(TP_CHIP_ID_REG);
        ESP_LOGI(TAG, "触摸芯片ID: 0x%02X", chip_id);
        read_buffer_ = new uint8_t[6];
    }

    ~Cst816s()
    {
        delete[] read_buffer_;
    }

    void UpdateTouchPoint()
    {
        ReadRegs(TP_I2C_START_REG, read_buffer_, 6);
        tp_.num = read_buffer_[0] & 0x0F;
        tp_.x = ((read_buffer_[1] & 0x0F) << 8) | read_buffer_[2];
        tp_.y = ((read_buffer_[3] & 0x0F) << 8) | read_buffer_[4];
    }

    const TouchPoint_t &GetTouchPoint()
    {
        return tp_;
    }

private:
    uint8_t *read_buffer_ = nullptr;
    TouchPoint_t tp_;
};

class CustomLcdDisplay : public SpiLcdDisplay
{
public:
    static void rounder_event_cb(lv_event_t *e)
    {
        lv_area_t *area = (lv_area_t *)lv_event_get_param(e);
        uint16_t x1 = area->x1;
        uint16_t x2 = area->x2;
        uint16_t y1 = area->y1;
        uint16_t y2 = area->y2;

        // 将 Area 的 start 向下舍入到最接近的偶数
        area->x1 = (x1 >> 1) << 1;
        area->y1 = (y1 >> 1) << 1;

        // 将 End of Area 向上舍入到最接近的奇数
        area->x2 = ((x2 >> 1) << 1) + 1;
        area->y2 = ((y2 >> 1) << 1) + 1;
    }

    CustomLcdDisplay(esp_lcd_panel_io_handle_t io_handle,
                     esp_lcd_panel_handle_t panel_handle,
                     int width,
                     int height,
                     int offset_x,
                     int offset_y,
                     bool mirror_x,
                     bool mirror_y,
                     bool swap_xy)
        : SpiLcdDisplay(io_handle, panel_handle,
                        width, height, offset_x, offset_y, mirror_x, mirror_y, swap_xy,
                        {
                            .text_font = &font_puhui_30_4,
                            .icon_font = &font_awesome_30_1,
                            .emoji_font = font_emoji_64_init(),
                        })
    {
        DisplayLockGuard lock(this);
        lv_display_add_event_cb(display_, rounder_event_cb, LV_EVENT_INVALIDATE_AREA, NULL);
    }
};

class SPIBacklight : public Backlight
{
public:
    SPIBacklight(esp_lcd_panel_io_handle_t panel_io) : Backlight(), panel_io_(panel_io) {}

protected:
    esp_lcd_panel_io_handle_t panel_io_;

    virtual void SetBrightnessImpl(uint8_t brightness) override
    {
        auto display = Board::GetInstance().GetDisplay();
        DisplayLockGuard lock(display);
        uint8_t data[1] = {((uint8_t)((255 * brightness) / 100))};
        int lcd_cmd = 0x51;
        lcd_cmd &= 0xff;
        lcd_cmd <<= 8;
        lcd_cmd |= LCD_OPCODE_WRITE_CMD << 24;
        esp_lcd_panel_io_tx_param(panel_io_, lcd_cmd, &data, sizeof(data));
    }
};

class ViewecircleS3 : public WifiBoard
{
private:
    i2c_master_bus_handle_t i2c_bus_;
    LcdDisplay *display_;
    Cst816s *cst816s_;
    Button boot_button_;
    SPIBacklight *backlight_;
    esp_timer_handle_t touchpad_timer_;
    void InitializeI2c()
    {
        const i2c_master_bus_config_t i2c_bus_config = {
            .i2c_port = (i2c_port_t)0,
            .sda_io_num = TP_PIN_NUM_SDA,
            .scl_io_num = TP_PIN_NUM_SCL,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags = {
                .enable_internal_pullup = true,
            }};
        ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, &i2c_bus_));
    }
    static void touchpad_timer_callback(void *arg)
    {
        auto &board = (ViewecircleS3 &)Board::GetInstance();
        auto touchpad = board.GetTouchpad();
        static bool was_touched = false;
        static int64_t touch_start_time = 0;
        const int64_t TOUCH_THRESHOLD_MS = 500; // 触摸时长阈值，超过500ms视为长按

        touchpad->UpdateTouchPoint();
        auto touch_point = touchpad->GetTouchPoint();

        // 检测触摸开始
        if (touch_point.num > 0 && !was_touched)
        {
            was_touched = true;
            touch_start_time = esp_timer_get_time() / 1000; // 转换为毫秒
        }
        // 检测触摸释放
        else if (touch_point.num == 0 && was_touched)
        {
            was_touched = false;
            int64_t touch_duration = (esp_timer_get_time() / 1000) - touch_start_time;

            // 只有短触才触发
            if (touch_duration < TOUCH_THRESHOLD_MS)
            {
                auto &app = Application::GetInstance();
                if (app.GetDeviceState() == kDeviceStateStarting &&
                    !WifiStation::GetInstance().IsConnected())
                {
                    board.ResetWifiConfiguration();
                }
                app.ToggleChatState();
            }
        }
    }

    void InitializeCst816sTouchPad()
    {
        ESP_LOGI(TAG, "开始初始化 Cst816s");
        cst816s_ = new Cst816s(i2c_bus_, TP_I2C_ADDRESS);
        ESP_LOGI(TAG, "初始化 Cst816s成功");
        // 创建定时器，10ms 间隔
        esp_timer_create_args_t timer_args = {
            .callback = touchpad_timer_callback,
            .arg = NULL,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "touchpad_timer",
            .skip_unhandled_events = true,
        };

        ESP_ERROR_CHECK(esp_timer_create(&timer_args, &touchpad_timer_));
        ESP_ERROR_CHECK(esp_timer_start_periodic(touchpad_timer_, 10 * 1000)); // 10ms = 10000us
    }

    void I2cDetect()
    {
        uint8_t address;
        printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\r\n");
        for (int i = 0; i < 128; i += 16)
        {
            printf("%02x: ", i);
            for (int j = 0; j < 16; j++)
            {
                fflush(stdout);
                address = i + j;
                esp_err_t ret = i2c_master_probe(i2c_bus_, address, pdMS_TO_TICKS(500));
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
    }

    void InitializeSpi()
    {
        ESP_LOGI(TAG, "初始化 QSPI 总线");
        spi_bus_config_t bus_config = {};
        bus_config.sclk_io_num = QSPI_PIN_NUM_LCD_PCLK;
        bus_config.data0_io_num = QSPI_PIN_NUM_LCD_DATA0;
        bus_config.data1_io_num = QSPI_PIN_NUM_LCD_DATA1;
        bus_config.data2_io_num = QSPI_PIN_NUM_LCD_DATA2;
        bus_config.data3_io_num = QSPI_PIN_NUM_LCD_DATA3;
        bus_config.max_transfer_sz = QSPI_LCD_H_RES * QSPI_LCD_V_RES * QSPI_LCD_BIT_PER_PIXEL / 8;
        bus_config.flags = SPICOMMON_BUSFLAG_QUAD;

        ESP_ERROR_CHECK(spi_bus_initialize(QSPI_LCD_HOST, &bus_config, SPI_DMA_CH_AUTO));
    }

    void Initializeco5300Display()
    {
        esp_lcd_panel_io_handle_t panel_io = nullptr;
        esp_lcd_panel_handle_t panel = nullptr;

        ESP_LOGI(TAG, "安装屏幕接口");

        const esp_lcd_panel_io_spi_config_t io_config = CO5300_PANEL_IO_QSPI_CONFIG(QSPI_PIN_NUM_LCD_CS, NULL, NULL);
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)QSPI_LCD_HOST, &io_config, &panel_io));

        ESP_LOGI(TAG, "安装 CO5300 屏幕驱动");

        co5300_vendor_config_t vendor_config = {
            .init_cmds = lcd_init_cmds,
            .init_cmds_size = sizeof(lcd_init_cmds) / sizeof(lcd_init_cmds[0]),
            .flags = {
                .use_qspi_interface = 1,
            },
        };
        const esp_lcd_panel_dev_config_t panel_config = {
            .reset_gpio_num = QSPI_PIN_NUM_LCD_RST,
            .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
            .bits_per_pixel = QSPI_LCD_BIT_PER_PIXEL,
            .vendor_config = (void *)&vendor_config,
        };
        ESP_ERROR_CHECK(esp_lcd_new_panel_co5300(panel_io, &panel_config, &panel));
        esp_lcd_panel_reset(panel);
        esp_lcd_panel_init(panel);
        esp_lcd_panel_disp_on_off(panel, true);
        esp_lcd_panel_swap_xy(panel, DISPLAY_SWAP_XY);
        esp_lcd_panel_mirror(panel, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);
        display_ = new CustomLcdDisplay(panel_io, panel,
                                        DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY);
        backlight_ = new SPIBacklight(panel_io);
    }

    void InitializeButtons()
    {
        boot_button_.OnClick([this]()
                             {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting && !WifiStation::GetInstance().IsConnected()) {
                ResetWifiConfiguration();
            }
            app.ToggleChatState(); });
    }

    // 物联网初始化，添加对 AI 可见设备
    void InitializeIot()
    {
        auto &thing_manager = iot::ThingManager::GetInstance();
        thing_manager.AddThing(iot::CreateThing("Speaker"));
        thing_manager.AddThing(iot::CreateThing("Screen"));
        thing_manager.AddThing(iot::CreateThing("SDCard"));
    }

    void InitializeMute()
    {
        gpio_reset_pin(AUDIO_MUTE_PIN);
        gpio_set_direction(AUDIO_MUTE_PIN, GPIO_MODE_OUTPUT);
        gpio_set_level(AUDIO_MUTE_PIN, 1);
    }
    void TouchRst()
    {
        gpio_reset_pin(TP_I2C_RST_IO);
        gpio_set_direction(TP_I2C_RST_IO, GPIO_MODE_OUTPUT);
        gpio_set_level(TP_I2C_RST_IO, 0);
        vTaskDelay(pdMS_TO_TICKS(200));
        gpio_set_level(TP_I2C_RST_IO, 1);
        vTaskDelay(pdMS_TO_TICKS(200));
    }

public:
    ViewecircleS3() : boot_button_(BOOT_BUTTON_GPIO)
    {
        TouchRst();
        InitializeI2c();
        I2cDetect();
        InitializeCst816sTouchPad();
        InitializeSpi();
        Initializeco5300Display();
        InitializeButtons();
        InitializeIot();
        InitializeMute();
        GetBacklight()->RestoreBrightness();
    }

    virtual AudioCodec *GetAudioCodec() override
    {
        static NoAudioCodecSimplexPdm audio_codec(AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
                                                  AUDIO_I2S_SPK_GPIO_BCLK, AUDIO_I2S_SPK_GPIO_LRCK, AUDIO_I2S_SPK_GPIO_DOUT,
                                                  AUDIO_PDM_MIC_GPIO_SCK, AUDIO_PDM_MIC_GPIO_DIN);

        return &audio_codec;
    }

    virtual Display *GetDisplay() override
    {
        return display_;
    }

    virtual Backlight *GetBacklight() override
    {
        return backlight_;
    }

    Cst816s *GetTouchpad()
    {
        return cst816s_;
    }
};

DECLARE_BOARD(ViewecircleS3);
