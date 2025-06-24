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

#include "boards/zeno-ai-dog/config.h"

#define TAG "DogLED"

// 定义固定的UART参数
#define UART_NUM        UART_NUM_2
#define TXD_PIN         GPIO_NUM_48
#define RXD_PIN         GPIO_NUM_38
#define UART_BAUD_RATE  19200
#define BUF_SIZE        256

namespace iot {

class DogLED : public Thing {
private:
    // 发送LED命令
    void SendLedCommand(uint8_t cmd, uint8_t r, uint8_t g, uint8_t b) {
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
        
        // 发送命令
        uart_write_bytes(UART_NUM, frame, sizeof(frame));
        ESP_ERROR_CHECK(uart_wait_tx_done(UART_NUM, pdMS_TO_TICKS(1000)));
        
        // 打印发送的串口数据（十六进制格式）
        ESP_LOGI(TAG, "发送串口数据 (十六进制):");
        char hex_str[256] = {0};
        for (int i = 0; i < sizeof(frame); i++) {
            char temp[4];
            sprintf(temp, "%02X ", frame[i]);
            strcat(hex_str, temp);
        }
        ESP_LOGI(TAG, "%s", hex_str);
        
        const char* led_name = (cmd == 0x02) ? "1号灯" : "2号灯";
        ESP_LOGI(TAG, "%s命令发送完成 - BRG(%d,%d,%d)", led_name, b, r, g);
    }

    // 发送舵机命令
    void SendServoCommand(uint8_t front_left, uint8_t front_right, uint8_t rear_left, uint8_t rear_right) {
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
        
        // 打印发送的串口数据（十六进制格式）
        ESP_LOGI(TAG, "发送舵机数据 (十六进制):");
        char hex_str[64] = {0};
        for (int i = 0; i < sizeof(frame); i++) {
            char temp[4];
            sprintf(temp, "%02X ", frame[i]);
            strcat(hex_str, temp);
        }
        ESP_LOGI(TAG, "%s", hex_str);
        
        ESP_LOGI(TAG, "舵机命令发送完成 - 前左:%d° 前右:%d° 后左:%d° 后右:%d°", 
                front_left, front_right, rear_left, rear_right);
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
    DogLED() : Thing("DogLED", "AI小狗的LED灯光控制") {
        InitializeLedUart();

        // 语音命令
        methods_.AddMethod("打开灯光", "打开1号LED灯带", ParameterList(), [this](const ParameterList& parameters) {
            SendLedCommand(0x02, 255, 255, 255);
            SendLedCommand(0x03, 255, 255, 255);
        });

        methods_.AddMethod("关闭灯光", "关闭1号LED灯带", ParameterList(), [this](const ParameterList& parameters) {
            SendLedCommand(0x02, 0, 0, 0);
            SendLedCommand(0x03, 0, 0, 0);
        });

        // 舵机控制语音命令
        methods_.AddMethod("休息", "让AI狗进入休息状态，所有舵机归位", ParameterList(), [this](const ParameterList& parameters) {
            SendServoCommand(0, 0, 0, 0);  // 所有舵机设为0度（初始化状态）
        });

        methods_.AddMethod("站起来", "让AI狗站立起来", ParameterList(), [this](const ParameterList& parameters) {
            SendServoCommand(90, 90, 90, 90);  // 所有舵机设为90度（垂直状态）
        });
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