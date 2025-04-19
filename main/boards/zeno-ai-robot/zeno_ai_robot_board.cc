#include "wifi_board.h"
#include "audio_codecs/box_audio_codec.h"
#include "display/lcd_display.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "i2c_device.h"
#include "iot/thing_manager.h"
#include "drivers/pcf8574.h"
#include "iot/things/dog_control.h"

#include <esp_log.h>
#include <esp_lcd_panel_vendor.h>
#include <driver/i2c_master.h>
#include <driver/spi_common.h>
#include <wifi_station.h>

#define TAG "ZenoAiRobotBoard"

LV_FONT_DECLARE(font_puhui_20_4);
LV_FONT_DECLARE(font_awesome_20_4);

class Pca9557 : public I2cDevice {
public:
    Pca9557(i2c_master_bus_handle_t i2c_bus, uint8_t addr) : I2cDevice(i2c_bus, addr) {
        try {
            ESP_LOGD(TAG, "初始化PCA9557，地址:0x%02x", addr);
            // 配置寄存器01(输出端口配置)
            WriteReg(0x01, 0x03);
            // 配置寄存器03(极性反转配置)
            WriteReg(0x03, 0xf8);
            ESP_LOGI(TAG, "PCA9557初始化成功");
        } catch (const std::exception& e) {
            ESP_LOGE(TAG, "PCA9557初始化失败: %s", e.what());
        }
    }

    bool SetOutputState(uint8_t bit, uint8_t level) {
        try {
            uint8_t data = ReadReg(0x01);
            data = (data & ~(1 << bit)) | (level << bit);
            WriteReg(0x01, data);
            return true;
        } catch (const std::exception& e) {
            ESP_LOGE(TAG, "设置PCA9557输出失败(bit=%d, level=%d): %s", bit, level, e.what());
            return false;
        }
    }
};

class ZenoAiRobotBoard : public WifiBoard {
private:
    i2c_master_bus_handle_t i2c_bus_;
    i2c_master_bus_handle_t io_expander_i2c_bus_;
    Button boot_button_;
    LcdDisplay* display_;
    Pca9557* pca9557_;
    PCF8574* pcf8574_;
    bool pca9557_initialized_ = false;
    bool pcf8574_initialized_ = false;

    void InitializeI2c() {
        // 初始化音频编解码器I2C总线
        i2c_master_bus_config_t i2c_bus_cfg = {
            .i2c_port = (i2c_port_t)1,
            .sda_io_num = AUDIO_CODEC_I2C_SDA_PIN,
            .scl_io_num = AUDIO_CODEC_I2C_SCL_PIN,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags = {
                .enable_internal_pullup = 1,
            },
        };
        
        ESP_LOGI(TAG, "初始化音频I2C总线 SDA:%d, SCL:%d", AUDIO_CODEC_I2C_SDA_PIN, AUDIO_CODEC_I2C_SCL_PIN);
        ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg, &i2c_bus_));

        // 初始化PCA9557
        try {
            pca9557_ = new Pca9557(i2c_bus_, 0x19);
            pca9557_initialized_ = true;
        } catch (const std::exception& e) {
            ESP_LOGE(TAG, "PCA9557初始化异常: %s", e.what());
            pca9557_initialized_ = false;
        }
        
        // 初始化IO扩展器I2C总线 - 使用硬件工程师指定的引脚
        i2c_master_bus_config_t io_i2c_cfg = {
            .i2c_port = (i2c_port_t)0,
            .sda_io_num = IO_EXPANDER_I2C_SDA_PIN,  // 使用GPIO48
            .scl_io_num = IO_EXPANDER_I2C_SCL_PIN,  // 使用GPIO38
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags = {
                .enable_internal_pullup = 1,
            },
        };
        
        ESP_LOGI(TAG, "初始化IO扩展I2C总线 SDA:%d, SCL:%d", IO_EXPANDER_I2C_SDA_PIN, IO_EXPANDER_I2C_SCL_PIN);
        ESP_ERROR_CHECK(i2c_new_master_bus(&io_i2c_cfg, &io_expander_i2c_bus_));
        
        // 初始化HT8574ARSZ IO扩展器 (使用PCF8574驱动，因为它们兼容)
        try {
            pcf8574_ = new PCF8574(io_expander_i2c_bus_, IO_EXPANDER_I2C_ADDR);
            // 设置所有IO为低电平
            pcf8574_->Write(0x00);
            pcf8574_initialized_ = true;
            ESP_LOGI(TAG, "IO扩展器HT8574ARSZ初始化成功，地址:0x%02x", IO_EXPANDER_I2C_ADDR);
        } catch (const std::exception& e) {
            ESP_LOGE(TAG, "IO扩展器初始化失败: %s", e.what());
            pcf8574_initialized_ = false;
        }
    }

    void InitializeSpi() {
        spi_bus_config_t buscfg = {};
        buscfg.mosi_io_num = GPIO_NUM_40;
        buscfg.miso_io_num = GPIO_NUM_NC;
        buscfg.sclk_io_num = GPIO_NUM_41;
        buscfg.quadwp_io_num = GPIO_NUM_NC;
        buscfg.quadhd_io_num = GPIO_NUM_NC;
        buscfg.max_transfer_sz = DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t);
        ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO));
        ESP_LOGI(TAG, "SPI总线初始化成功 MOSI:%d, SCK:%d", GPIO_NUM_40, GPIO_NUM_41);
    }

    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting && !WifiStation::GetInstance().IsConnected()) {
                ResetWifiConfiguration();
            }
            app.ToggleChatState();
        });
        ESP_LOGI(TAG, "按钮初始化完成，使用引脚: %d", BOOT_BUTTON_GPIO);
    }

    void InitializeSt7789Display() {
        esp_lcd_panel_io_handle_t panel_io = nullptr;
        esp_lcd_panel_handle_t panel = nullptr;
        // 液晶屏控制IO初始化
        ESP_LOGD(TAG, "正在初始化显示屏控制器");
        esp_lcd_panel_io_spi_config_t io_config = {};
        io_config.cs_gpio_num = GPIO_NUM_NC;
        io_config.dc_gpio_num = GPIO_NUM_39;
        io_config.spi_mode = 2;
        io_config.pclk_hz = 80 * 1000 * 1000;
        io_config.trans_queue_depth = 10;
        io_config.lcd_cmd_bits = 8;
        io_config.lcd_param_bits = 8;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(SPI3_HOST, &io_config, &panel_io));

        // 初始化液晶屏驱动芯片ST7789
        ESP_LOGD(TAG, "正在初始化ST7789驱动");
        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = GPIO_NUM_NC;
        panel_config.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB;
        panel_config.bits_per_pixel = 16;
        ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(panel_io, &panel_config, &panel));
        
        esp_lcd_panel_reset(panel);
        
        // 只有当PCA9557初始化成功时才调用SetOutputState
        if (pca9557_initialized_) {
            pca9557_->SetOutputState(0, 0);
        } else {
            ESP_LOGW(TAG, "PCA9557未初始化，跳过显示屏复位信号");
        }

        esp_lcd_panel_init(panel);
        esp_lcd_panel_invert_color(panel, true);
        esp_lcd_panel_swap_xy(panel, DISPLAY_SWAP_XY);
        esp_lcd_panel_mirror(panel, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);
        display_ = new SpiLcdDisplay(panel_io, panel,
                                     DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY,
                                     {
                                         .text_font = &font_puhui_20_4,
                                         .icon_font = &font_awesome_20_4,
#if CONFIG_USE_WECHAT_MESSAGE_STYLE
                                         .emoji_font = font_emoji_32_init(),
#else
                                         .emoji_font = font_emoji_64_init(),
#endif
                                     });
        ESP_LOGI(TAG, "显示屏初始化完成 %dx%d", DISPLAY_WIDTH, DISPLAY_HEIGHT);
    }

    // 物联网初始化，添加对 AI 可见设备
    void InitializeIot() {
        auto& thing_manager = iot::ThingManager::GetInstance();
        thing_manager.AddThing(iot::CreateThing("Speaker"));
        thing_manager.AddThing(iot::CreateThing("Screen"));
        
        // 只有当PCF8574初始化成功时才添加小狗控制器
        if (pcf8574_initialized_) {
            // 添加小狗控制器并设置IO扩展器
            auto dog_control = new iot::DogControl();
            // 设置PCF8574 IO扩展器，实现舵机和灯光控制
            dog_control->SetIoExpander(pcf8574_);
            thing_manager.AddThing(dog_control);
            
            // 初始化时，闪烁流水灯以显示板子已就绪
            pcf8574_->SetPin(FLOW_LIGHT_1, true);
            pcf8574_->SetPin(FLOW_LIGHT_2, true);
            ESP_LOGI(TAG, "流水灯初始化完成");
        } else {
            ESP_LOGW(TAG, "IO扩展器未初始化，跳过小狗控制器配置");
        }
        
        ESP_LOGI(TAG, "IoT组件初始化完成");
    }

public:
    ZenoAiRobotBoard() : boot_button_(BOOT_BUTTON_GPIO) {
        ESP_LOGI(TAG, "开始初始化ZenoAiRobot开发板");
        InitializeI2c();
        InitializeSpi();
        InitializeButtons();
        
        // 如果PCA9557初始化成功，再初始化显示屏
        if (pca9557_initialized_) {
            InitializeSt7789Display();
        } else {
            ESP_LOGE(TAG, "PCA9557初始化失败，跳过显示屏初始化");
        }
        
        InitializeIot();
        GetBacklight()->RestoreBrightness();
        ESP_LOGI(TAG, "ZenoAiRobot开发板初始化完成");
    }

    virtual AudioCodec* GetAudioCodec() override {
        static BoxAudioCodec* audio_codec = nullptr;
        // 如果编解码器已经创建，直接返回
        if (audio_codec != nullptr) {
            return audio_codec;
        }
        
        // 尝试创建编解码器，如果失败则返回静默实现
        try {
            audio_codec = new BoxAudioCodec(
                i2c_bus_, 
                AUDIO_INPUT_SAMPLE_RATE, 
                AUDIO_OUTPUT_SAMPLE_RATE,
                AUDIO_I2S_GPIO_MCLK, 
                AUDIO_I2S_GPIO_BCLK, 
                AUDIO_I2S_GPIO_WS, 
                AUDIO_I2S_GPIO_DOUT, 
                AUDIO_I2S_GPIO_DIN,
                GPIO_NUM_NC, 
                AUDIO_CODEC_ES8311_ADDR, 
                AUDIO_CODEC_ES7210_ADDR, 
                AUDIO_INPUT_REFERENCE);
            
            ESP_LOGI(TAG, "音频编解码器初始化成功");
        } catch (const std::exception& e) {
            ESP_LOGE(TAG, "音频编解码器初始化失败: %s", e.what());
            // 失败情况下返回空音频编解码器实现（仅用于满足接口需求）
            static class NullAudioCodec : public AudioCodec {
            private:
                virtual int Read(int16_t* dest, int samples) override { return 0; }
                virtual int Write(const int16_t* data, int samples) override { return samples; }
            public:
                NullAudioCodec() {
                    ESP_LOGW(TAG, "使用空音频编解码器实现");
                }
            } null_codec;
            
            return &null_codec;
        }
        
        return audio_codec;
    }

    virtual Display* GetDisplay() override {
        return display_;
    }
    
    virtual Backlight* GetBacklight() override {
        static PwmBacklight backlight(DISPLAY_BACKLIGHT_PIN, DISPLAY_BACKLIGHT_OUTPUT_INVERT);
        return &backlight;
    }
    
    PCF8574* GetIoExpander() {
        return pcf8574_;
    }
};

DECLARE_BOARD(ZenoAiRobotBoard); 