/*
    zeno-ai-dog 的LED灯光控制
    通过串口发送命令控制流水灯
*/

#include "sdkconfig.h"
#include "iot/thing.h"
#include "board.h"

#include <driver/gpio.h>
#include <driver/uart.h>
#include <esp_log.h>
#include <cstring>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <cmath>

#include "boards/zeno-ai-dog/config.h"

#define TAG "DogLED"

// 定义固定的UART参数
#define UART_NUM        UART_NUM_2
#define TXD_PIN         GPIO_NUM_48
#define RXD_PIN         GPIO_NUM_38
#define UART_BAUD_RATE  19200
#define BUF_SIZE        256

namespace iot {

// LED动画状态枚举
enum LedAnimationState {
    LED_IDLE = 0,
    LED_STARTUP_FLASH,
    LED_LIGHT_SHOW,
    LED_BREATHING
};

class DogLED : public Thing {
private:
    TaskHandle_t animation_task_handle_;
    SemaphoreHandle_t uart_mutex_;
    volatile LedAnimationState current_state_;
    volatile bool stop_animation_;

    // 原始LED命令发送（不含互斥锁，内部使用）
    void SendLedCommandRaw(uint8_t cmd, uint8_t r, uint8_t g, uint8_t b) {
        ESP_LOGI(TAG, "发送原始LED命令 - 命令:0x%02X RGB(%d,%d,%d)", cmd, r, g, b);

        uint8_t frame[19];
        frame[0] = 0xAA;  // 帧头
        frame[1] = 17;    // 数据长度
        frame[2] = cmd;   // 命令
        
        // 5个LED设置相同颜色 - 协议顺序是BRG（蓝红绿）
        for (int i = 0; i < 5; i++) {
            frame[3 + i*3] = b;     // B (蓝色)
            frame[4 + i*3] = r;     // R (红色)  
            frame[5 + i*3] = g;     // G (绿色)
        }
        
        // 计算校验：数据长度 + 命令 + 数据字节的异或
        uint8_t checksum = frame[1] ^ frame[2];
        for (int i = 3; i < 18; i++) {
            checksum ^= frame[i];
        }
        frame[18] = checksum;
        
        // 打印完整的发送数据帧（十六进制）
        ESP_LOGI(TAG, "帧数据:");
        char hex_str[128] = {0};
        for (int i = 0; i < 19; i++) {
            char temp[4];
            sprintf(temp, "%02X ", frame[i]);
            strcat(hex_str, temp);
        }
        ESP_LOGI(TAG, "%s", hex_str);
        
        // 清空UART接收缓冲区，避免干扰
        uart_flush_input(UART_NUM);
        
        // 发送命令
        int bytes_written = uart_write_bytes(UART_NUM, frame, sizeof(frame));
        ESP_LOGI(TAG, "UART写入字节数: %d", bytes_written);
        
        // 等待发送完成
        esp_err_t wait_result = uart_wait_tx_done(UART_NUM, pdMS_TO_TICKS(2000));
        if (wait_result != ESP_OK) {
            ESP_LOGE(TAG, "UART发送超时，错误码: %d", wait_result);
        } else {
            ESP_LOGI(TAG, "UART发送完成");
        }
        
        // 发送完成后再延时，确保硬件处理完成
        vTaskDelay(pdMS_TO_TICKS(10));
        
        ESP_LOGI(TAG, "原始LED命令发送完毕 - 命令:0x%02X", cmd);
    }

    // 发送LED命令（带互斥锁保护）
    void SendLedCommand(uint8_t cmd, uint8_t r, uint8_t g, uint8_t b) {
        ESP_LOGI(TAG, "准备发送LED命令 - 命令:0x%02X RGB(%d,%d,%d)", cmd, r, g, b);
        
        // 获取UART互斥锁，确保线程安全
        if (xSemaphoreTake(uart_mutex_, pdMS_TO_TICKS(1000)) != pdTRUE) {
            ESP_LOGW(TAG, "获取UART互斥锁超时");
            return;
        }

        uint8_t frame[19];
        frame[0] = 0xAA;  // 帧头
        frame[1] = 17;    // 数据长度
        frame[2] = cmd;   // 命令
        
        // 5个LED设置相同颜色 - 协议顺序是BRG（蓝红绿）
        for (int i = 0; i < 5; i++) {
            frame[3 + i*3] = b;     // B (蓝色)
            frame[4 + i*3] = r;     // R (红色)  
            frame[5 + i*3] = g;     // G (绿色)
        }
        
        // 计算校验：数据长度 + 命令 + 数据字节的异或
        uint8_t checksum = frame[1] ^ frame[2];
        for (int i = 3; i < 18; i++) {
            checksum ^= frame[i];
        }
        frame[18] = checksum;
        
        // 打印完整的发送数据帧（十六进制）
        ESP_LOGI(TAG, "发送帧数据:");
        char hex_str[128] = {0};
        for (int i = 0; i < 19; i++) {
            char temp[4];
            sprintf(temp, "%02X ", frame[i]);
            strcat(hex_str, temp);
        }
        ESP_LOGI(TAG, "%s", hex_str);
        
        // 清空UART接收缓冲区，避免干扰
        uart_flush_input(UART_NUM);
        
        // 发送命令
        int bytes_written = uart_write_bytes(UART_NUM, frame, sizeof(frame));
        ESP_LOGI(TAG, "UART写入字节数: %d", bytes_written);
        
        // 等待发送完成，增加超时时间
        esp_err_t wait_result = uart_wait_tx_done(UART_NUM, pdMS_TO_TICKS(2000));
        if (wait_result != ESP_OK) {
            ESP_LOGE(TAG, "UART发送超时，错误码: %d", wait_result);
        } else {
            ESP_LOGI(TAG, "UART发送完成");
        }
        
        // 发送完成后再延时，确保硬件处理完成
        vTaskDelay(pdMS_TO_TICKS(10));
        
        xSemaphoreGive(uart_mutex_);
        
        ESP_LOGI(TAG, "LED命令发送完毕 - 命令:0x%02X RGB(%d,%d,%d)", cmd, r, g, b);
    }

    // 同时控制两组LED - 优化互斥锁使用
    void SetBothLeds(uint8_t r, uint8_t g, uint8_t b) {
        ESP_LOGI(TAG, "=== 开始设置两组LED颜色 - RGB(%d,%d,%d) ===", r, g, b);
        
        // 一次获取锁，发送两个命令，避免锁竞争
        if (xSemaphoreTake(uart_mutex_, pdMS_TO_TICKS(2000)) != pdTRUE) {
            ESP_LOGE(TAG, "获取UART互斥锁超时 - 两组LED设置失败");
            return;
        }
        
        // 发送第一组LED命令 (不使用SendLedCommand，避免重复获取锁)
        ESP_LOGI(TAG, ">>> 发送第一组LED命令 (0x02)");
        SendLedCommandRaw(0x02, r, g, b);
        vTaskDelay(pdMS_TO_TICKS(50)); // 硬件处理时间
        
        // 发送第二组LED命令  
        ESP_LOGI(TAG, ">>> 发送第二组LED命令 (0x03)");
        SendLedCommandRaw(0x03, r, g, b);
        vTaskDelay(pdMS_TO_TICKS(50)); // 硬件处理时间
        
        // 释放锁
        xSemaphoreGive(uart_mutex_);
        
        ESP_LOGI(TAG, "=== 两组LED命令全部发送完成 ===");
    }

    // 分别控制两组LED（用于跳舞效果）- 优化互斥锁使用
    void SetLedGroup(uint8_t group1_r, uint8_t group1_g, uint8_t group1_b, 
                     uint8_t group2_r, uint8_t group2_g, uint8_t group2_b) {
        ESP_LOGI(TAG, "设置两组LED不同颜色 - 组1:RGB(%d,%d,%d) 组2:RGB(%d,%d,%d)", 
                group1_r, group1_g, group1_b, group2_r, group2_g, group2_b);
        
        // 一次获取锁，发送两个命令，避免锁竞争
        if (xSemaphoreTake(uart_mutex_, pdMS_TO_TICKS(2000)) != pdTRUE) {
            ESP_LOGE(TAG, "获取UART互斥锁超时 - 分组LED设置失败");
            return;
        }
        
        // 第一组LED
        SendLedCommandRaw(0x02, group1_r, group1_g, group1_b);
        vTaskDelay(pdMS_TO_TICKS(50));
        
        // 第二组LED  
        SendLedCommandRaw(0x03, group2_r, group2_g, group2_b);
        vTaskDelay(pdMS_TO_TICKS(50));
        
        // 释放锁
        xSemaphoreGive(uart_mutex_);
        
        ESP_LOGI(TAG, "分组LED命令发送完成");
    }

    // 发送舵机命令
    void SendServoCommand(uint8_t front_left, uint8_t front_right, uint8_t rear_left, uint8_t rear_right) {
        // 获取UART互斥锁
        if (xSemaphoreTake(uart_mutex_, pdMS_TO_TICKS(1000)) != pdTRUE) {
            ESP_LOGW(TAG, "获取UART互斥锁超时");
            return;
        }

        uint8_t frame[8];
        frame[0] = 0xAA;        // 帧头
        frame[1] = 0x06;        // 数据长度 (1+4+1=6)
        frame[2] = 0x01;        // 舵机命令
        frame[3] = front_left;  // 前左舵机角度
        frame[4] = front_right; // 前右舵机角度
        frame[5] = rear_left;   // 后左舵机角度
        frame[6] = rear_right;  // 后右舵机角度
        
        // 计算校验：数据长度 + 命令 + 数据字节的异或
        uint8_t checksum = frame[1] ^ frame[2] ^ frame[3] ^ frame[4] ^ frame[5] ^ frame[6];
        frame[7] = checksum;
        
        // 发送命令
        uart_write_bytes(UART_NUM, frame, sizeof(frame));
        ESP_ERROR_CHECK(uart_wait_tx_done(UART_NUM, pdMS_TO_TICKS(1000)));
        
        xSemaphoreGive(uart_mutex_);
        
        ESP_LOGI(TAG, "舵机命令发送完成 - 前左:%d° 前右:%d° 后左:%d° 后右:%d°", 
                front_left, front_right, rear_left, rear_right);
    }

    // 开机闪烁效果
    void StartupFlash() {
        ESP_LOGI(TAG, "开始开机闪烁效果");
        
        // 1秒内闪2次，总持续5秒，共10次闪烁
        for (int i = 0; i < 10; i++) {
            if (stop_animation_) break;
            
            // 亮起 - 白光
            SetBothLeds(255, 255, 255);
            vTaskDelay(pdMS_TO_TICKS(250));  // 亮250ms
            
            if (stop_animation_) break;
            
            // 熄灭
            SetBothLeds(0, 0, 0);
            vTaskDelay(pdMS_TO_TICKS(250));  // 暗250ms
        }
        
        ESP_LOGI(TAG, "开机闪烁效果完成");
    }

    // 炫酷跳舞灯光秀效果
    void LightShow() {
        ESP_LOGI(TAG, "开始炫酷跳舞灯光秀效果");
        
        // 基础色彩库
        const int colors[][3] = {
            {255, 0, 0},     // 红色
            {255, 127, 0},   // 橙色
            {255, 255, 0},   // 黄色
            {127, 255, 0},   // 黄绿色
            {0, 255, 0},     // 绿色
            {0, 255, 127},   // 青绿色
            {0, 255, 255},   // 青色
            {0, 127, 255},   // 天蓝色
            {0, 0, 255},     // 蓝色
            {127, 0, 255},   // 蓝紫色
            {255, 0, 255},   // 紫色
            {255, 0, 127}    // 粉红色
        };
        const int num_colors = sizeof(colors) / sizeof(colors[0]);
        
        // 跳舞灯光秀总时长约15秒，包含多种效果
        
        // 效果1：快速闪烁彩虹 (3秒)
        ESP_LOGI(TAG, "彩虹快闪效果");
        for (int i = 0; i < 15 && !stop_animation_; i++) {
            int color_idx = i % num_colors;
            SetBothLeds(colors[color_idx][0], colors[color_idx][1], colors[color_idx][2]);
            vTaskDelay(pdMS_TO_TICKS(100)); // 快闪100ms
            SetBothLeds(0, 0, 0);
            vTaskDelay(pdMS_TO_TICKS(100)); // 暗100ms
        }
        
        if (stop_animation_) return;
        
        // 效果2：双组LED交替跳跃 (4秒)
        ESP_LOGI(TAG, "双组交替跳跃效果");
        for (int i = 0; i < 20 && !stop_animation_; i++) {
            int color1_idx = i % num_colors;
            int color2_idx = (i + 6) % num_colors; // 错开6种颜色
            
            // 第一组亮，第二组暗
            SetLedGroup(colors[color1_idx][0], colors[color1_idx][1], colors[color1_idx][2],
                       0, 0, 0);
            vTaskDelay(pdMS_TO_TICKS(100));
            
            // 第一组暗，第二组亮
            SetLedGroup(0, 0, 0,
                       colors[color2_idx][0], colors[color2_idx][1], colors[color2_idx][2]);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        
        if (stop_animation_) return;
        
        // 效果3：爆闪效果 (3秒)
        ESP_LOGI(TAG, "爆闪效果");
        for (int i = 0; i < 30 && !stop_animation_; i++) {
            // 随机选择颜色
            int color_idx = (i * 7) % num_colors; // 伪随机
            SetBothLeds(colors[color_idx][0], colors[color_idx][1], colors[color_idx][2]);
            vTaskDelay(pdMS_TO_TICKS(50)); // 极快闪烁50ms
            SetBothLeds(0, 0, 0);
            vTaskDelay(pdMS_TO_TICKS(50));
        }
        
        if (stop_animation_) return;
        
        // 效果4：双色波浪 (3秒)
        ESP_LOGI(TAG, "双色波浪效果");
        for (int i = 0; i < 15 && !stop_animation_; i++) {
            int color1_idx = i % num_colors;
            int color2_idx = (i + 6) % num_colors;
            
            // 两组LED不同颜色同时闪烁
            SetLedGroup(colors[color1_idx][0], colors[color1_idx][1], colors[color1_idx][2],
                       colors[color2_idx][0], colors[color2_idx][1], colors[color2_idx][2]);
            vTaskDelay(pdMS_TO_TICKS(200));
        }
        
        if (stop_animation_) return;
        
        // 效果5：彩虹渐变收尾 (2秒)
        ESP_LOGI(TAG, "彩虹渐变收尾");
        for (int i = 0; i < num_colors && !stop_animation_; i++) {
            SetBothLeds(colors[i][0], colors[i][1], colors[i][2]);
            vTaskDelay(pdMS_TO_TICKS(150));
        }
        
        // 灯光秀结束，关闭LED
        SetBothLeds(0, 0, 0);
        ESP_LOGI(TAG, "炫酷跳舞灯光秀完成！");
    }

    // 呼吸灯效果
    void BreathingLight() {
        ESP_LOGI(TAG, "开始呼吸灯效果");
        
        // 使用温暖的白光进行呼吸效果
        const uint8_t base_r = 255;
        const uint8_t base_g = 200;
        const uint8_t base_b = 150;
        
        // 快节奏呼吸效果，持续24秒
        for (int cycle = 0; cycle < 8 && !stop_animation_; cycle++) {
            // 渐亮过程 (1.5秒)
            for (int brightness = 0; brightness <= 255 && !stop_animation_; brightness += 10) {
                uint8_t r = (base_r * brightness) / 255;
                uint8_t g = (base_g * brightness) / 255;
                uint8_t b = (base_b * brightness) / 255;
                
                SetBothLeds(r, g, b);
                vTaskDelay(pdMS_TO_TICKS(60)); // 1.5秒渐亮，更有节奏感
            }
            
            if (stop_animation_) break;
            
            // 渐暗过程 (1.5秒)
            for (int brightness = 255; brightness >= 0 && !stop_animation_; brightness -= 10) {
                uint8_t r = (base_r * brightness) / 255;
                uint8_t g = (base_g * brightness) / 255;
                uint8_t b = (base_b * brightness) / 255;
                
                SetBothLeds(r, g, b);
                vTaskDelay(pdMS_TO_TICKS(60)); // 1.5秒渐暗，快节奏呼吸
            }
        }
        
        // 呼吸灯结束，关闭LED
        SetBothLeds(0, 0, 0);
        ESP_LOGI(TAG, "呼吸灯效果完成");
    }

    // 动画任务函数
    static void AnimationTask(void* parameter) {
        DogLED* dog_led = static_cast<DogLED*>(parameter);
        
        while (true) {
            switch (dog_led->current_state_) {
                case LED_STARTUP_FLASH:
                    dog_led->StartupFlash();
                    dog_led->current_state_ = LED_IDLE;
                    break;
                    
                case LED_LIGHT_SHOW:
                    dog_led->LightShow();
                    dog_led->current_state_ = LED_IDLE;
                    break;
                    
                case LED_BREATHING:
                    dog_led->BreathingLight();
                    dog_led->current_state_ = LED_IDLE;
                    break;
                    
                case LED_IDLE:
                default:
                    vTaskDelay(pdMS_TO_TICKS(100));
                    break;
            }
        }
    }

    // 停止当前动画并设置新状态
    void SetAnimationState(LedAnimationState new_state) {
        stop_animation_ = true;
        vTaskDelay(pdMS_TO_TICKS(100)); // 等待当前动画停止
        stop_animation_ = false;
        current_state_ = new_state;
    }

    void InitializeLedUart() {
        uart_driver_delete(UART_NUM);
        
        uart_config_t uart_config = {
            .baud_rate = UART_BAUD_RATE,
            .data_bits = UART_DATA_8_BITS,
            .parity    = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .source_clk = UART_SCLK_DEFAULT,
        };

        ESP_ERROR_CHECK(uart_driver_install(UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0));
        ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
        ESP_ERROR_CHECK(uart_set_pin(UART_NUM, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

        ESP_LOGI(TAG, "LED串口初始化成功");
    }

public:
    DogLED() : Thing("DogLED", "AI小狗的LED灯光控制"), 
               animation_task_handle_(nullptr),
               current_state_(LED_IDLE),
               stop_animation_(false) {
        
        // 创建UART互斥锁
        uart_mutex_ = xSemaphoreCreateMutex();
        if (uart_mutex_ == NULL) {
            ESP_LOGE(TAG, "创建UART互斥锁失败");
            return;
        }
        
        InitializeLedUart();

        // 创建动画任务
        xTaskCreate(AnimationTask, "led_animation", 4096, this, 5, &animation_task_handle_);

        // 开机后延时1秒再开始闪烁效果
        vTaskDelay(pdMS_TO_TICKS(1000));
        SetAnimationState(LED_STARTUP_FLASH);

        // 基本灯光控制语音命令
        methods_.AddMethod("打开灯光", "打开LED灯带", ParameterList(), [this](const ParameterList& parameters) {
            SetAnimationState(LED_IDLE);
            vTaskDelay(pdMS_TO_TICKS(200));
            SetBothLeds(255, 255, 255);
        });

        methods_.AddMethod("关闭灯光", "关闭LED灯带", ParameterList(), [this](const ParameterList& parameters) {
            SetAnimationState(LED_IDLE);
            vTaskDelay(pdMS_TO_TICKS(200));
            SetBothLeds(0, 0, 0);
        });

        // 动画效果语音命令
        methods_.AddMethod("灯光秀", "播放炫酷跳舞闪烁的LED灯光秀效果，包含快闪、交替、爆闪等动感效果", ParameterList(), [this](const ParameterList& parameters) {
            SetAnimationState(LED_LIGHT_SHOW);
        });

        methods_.AddMethod("呼吸灯", "播放由暗到明再由明到暗的呼吸灯效果", ParameterList(), [this](const ParameterList& parameters) {
            SetAnimationState(LED_BREATHING);
        });

        // LED测试命令（用于调试）
        methods_.AddMethod("测试第一组灯", "单独测试第一组LED（0x02命令）", ParameterList(), [this](const ParameterList& parameters) {
            SetAnimationState(LED_IDLE);
            vTaskDelay(pdMS_TO_TICKS(200));
            ESP_LOGI(TAG, "=== 开始测试第一组LED ===");
            SendLedCommand(0x02, 255, 0, 0); // 红色
            vTaskDelay(pdMS_TO_TICKS(1000));
            SendLedCommand(0x02, 0, 255, 0); // 绿色
            vTaskDelay(pdMS_TO_TICKS(1000));
            SendLedCommand(0x02, 0, 0, 255); // 蓝色
            vTaskDelay(pdMS_TO_TICKS(1000));
            SendLedCommand(0x02, 0, 0, 0);   // 关闭
            ESP_LOGI(TAG, "=== 第一组LED测试完成 ===");
        });

        methods_.AddMethod("测试第二组灯", "单独测试第二组LED（0x03命令）", ParameterList(), [this](const ParameterList& parameters) {
            SetAnimationState(LED_IDLE);
            vTaskDelay(pdMS_TO_TICKS(200));
            ESP_LOGI(TAG, "=== 开始测试第二组LED ===");
            SendLedCommand(0x03, 255, 0, 0); // 红色
            vTaskDelay(pdMS_TO_TICKS(1000));
            SendLedCommand(0x03, 0, 255, 0); // 绿色
            vTaskDelay(pdMS_TO_TICKS(1000));
            SendLedCommand(0x03, 0, 0, 255); // 蓝色
            vTaskDelay(pdMS_TO_TICKS(1000));
            SendLedCommand(0x03, 0, 0, 0);   // 关闭
            ESP_LOGI(TAG, "=== 第二组LED测试完成 ===");
        });

        // 舵机控制语音命令
        methods_.AddMethod("休息", "让AI狗进入休息状态，所有舵机归位", ParameterList(), [this](const ParameterList& parameters) {
            SendServoCommand(0, 0, 0, 0);  // 所有舵机设为0度（初始化状态）
        });

        methods_.AddMethod("站起来", "让AI狗站立起来", ParameterList(), [this](const ParameterList& parameters) {
            SendServoCommand(90, 90, 90, 90);  // 所有舵机设为90度（垂直状态）
        });
    }

    ~DogLED() {
        if (animation_task_handle_) {
            vTaskDelete(animation_task_handle_);
        }
        if (uart_mutex_) {
            vSemaphoreDelete(uart_mutex_);
        }
    }
};

// 注册Thing类型
static iot::Thing* CreateDogLED() {
    return new iot::DogLED();
}

static bool RegisterDogLEDHelper = []() {
    RegisterThing("DogLED", CreateDogLED);
    return true;
}();

} // namespace iot 