/*
    zeno-ai-dog 的舵机控制
    通过串口发送命令控制四个足部舵机
*/

#include "sdkconfig.h"
#include "iot/thing.h"
#include "board.h"

#include <driver/gpio.h>
#include <driver/uart.h>
#include <esp_log.h>
#include <cstring>

#include "boards/zeno-ai-dog/config.h"

#define TAG "DogServo"

// 定义固定的UART参数，与用户hello world代码保持一致
#define UART_NUM        UART_NUM_1
#define TXD_PIN         GPIO_NUM_48
#define RXD_PIN         GPIO_NUM_38
#define UART_BAUD_RATE  19200
#define BUF_SIZE        256

// 协议常量
#define FRAME_HEADER    0xAA
#define CMD_SERVO       0x01
#define CMD_LED1        0x02
#define CMD_LED2        0x03
#define SERVO_DATA_LEN  6    // cmd(1) + data(4) + checksum(1)
#define LED_DATA_LEN    16   // cmd(1) + data(15) + checksum(1)

namespace iot {

class DogServo : public Thing {
private:
    // 存储四个舵机的当前角度
    uint8_t servo_angles_[4] = {90, 90, 90, 90}; // LF, RF, LR, RR

    // 发送流水灯控制命令
    void SendLedCommand(uint8_t cmd, uint8_t r1, uint8_t g1, uint8_t b1, 
                       uint8_t r2, uint8_t g2, uint8_t b2,
                       uint8_t r3, uint8_t g3, uint8_t b3,
                       uint8_t r4, uint8_t g4, uint8_t b4,
                       uint8_t r5, uint8_t g5, uint8_t b5) {
        // 构建协议帧
        uint8_t frame[18];
        frame[0] = FRAME_HEADER;        // 帧头 0xAA
        frame[1] = LED_DATA_LEN;        // 数据长度 16
        frame[2] = cmd;                 // 命令 0x02 或 0x03
        frame[3] = r1; frame[4] = g1; frame[5] = b1;   // LED1 RGB
        frame[6] = r2; frame[7] = g2; frame[8] = b2;   // LED2 RGB
        frame[9] = r3; frame[10] = g3; frame[11] = b3; // LED3 RGB
        frame[12] = r4; frame[13] = g4; frame[14] = b4; // LED4 RGB
        frame[15] = r5; frame[16] = g5; frame[17] = b5; // LED5 RGB
        
        // 计算校验：数据长度 + 命令 + 数据字节的异或
        uint8_t checksum = frame[1] ^ frame[2];
        for (int i = 3; i < 18; i++) {
            checksum ^= frame[i];
        }
        
        // 发送帧头到数据部分
        uart_write_bytes(UART_NUM, frame, 18);
        // 发送校验码
        uart_write_bytes(UART_NUM, &checksum, 1);
        
        ESP_LOGI(TAG, "发送灯光命令: CMD=0x%02X, 校验=0x%02X", cmd, checksum);
    }

    // 发送二进制协议命令
    void SendServoCommand(uint8_t lf_angle, uint8_t rf_angle, uint8_t lr_angle, uint8_t rr_angle) {
        // 构建协议帧
        uint8_t frame[7];
        frame[0] = FRAME_HEADER;        // 帧头 0xAA
        frame[1] = SERVO_DATA_LEN;      // 数据长度 6
        frame[2] = CMD_SERVO;           // 命令 0x01
        frame[3] = lf_angle;            // 前左舵机角度
        frame[4] = rf_angle;            // 前右舵机角度
        frame[5] = lr_angle;            // 后左舵机角度
        frame[6] = rr_angle;            // 后右舵机角度
        
        // 计算校验：数据长度 + 命令 + 数据字节的异或
        uint8_t checksum = frame[1] ^ frame[2] ^ frame[3] ^ frame[4] ^ frame[5] ^ frame[6];
        
        // 发送帧头到数据部分
        uart_write_bytes(UART_NUM, frame, 7);
        // 发送校验码
        uart_write_bytes(UART_NUM, &checksum, 1);
        
        // 更新存储的角度值
        servo_angles_[0] = lf_angle;
        servo_angles_[1] = rf_angle;
        servo_angles_[2] = lr_angle;
        servo_angles_[3] = rr_angle;
        
        ESP_LOGI(TAG, "发送舵机命令: LF=%d, RF=%d, LR=%d, RR=%d, 校验=0x%02X", 
                 lf_angle, rf_angle, lr_angle, rr_angle, checksum);
    }

    void InitializeServoUart() {
        uart_config_t uart_config = {
            .baud_rate = UART_BAUD_RATE,
            .data_bits = UART_DATA_8_BITS,
            .parity    = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .source_clk = UART_SCLK_DEFAULT,
        };
        int intr_alloc_flags = 0;

        ESP_ERROR_CHECK(uart_driver_install(UART_NUM, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
        ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
        ESP_ERROR_CHECK(uart_set_pin(UART_NUM, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

        ESP_LOGI(TAG, "串口初始化完成：TX=GPIO%d, RX=GPIO%d", TXD_PIN, RXD_PIN);
    }

public:
    DogServo() : Thing("DogServo", "AI小狗的舵机控制") {
        InitializeServoUart();

        // 定义设备可以被远程执行的指令
        methods_.AddMethod("站立", "让小狗站立", ParameterList(), [this](const ParameterList& parameters) {
            // 左前45度，左后45度，右前135度，右后135度
            SendServoCommand(45, 135, 45, 135);
        });

        methods_.AddMethod("坐下", "让小狗坐下", ParameterList(), [this](const ParameterList& parameters) {
            // 所有舵机90度
            SendServoCommand(90, 90, 90, 90);
        });
        
        methods_.AddMethod("摇尾", "让小狗摇尾巴", ParameterList(), [this](const ParameterList& parameters) {
            // 后腿左右摆动
            SendServoCommand(servo_angles_[0], servo_angles_[1], 70, 110);
            vTaskDelay(pdMS_TO_TICKS(500));
            SendServoCommand(servo_angles_[0], servo_angles_[1], 110, 70);
            vTaskDelay(pdMS_TO_TICKS(500));
            SendServoCommand(servo_angles_[0], servo_angles_[1], 90, 90);
        });

        methods_.AddMethod("前进", "向前走", ParameterList(), [this](const ParameterList& parameters) {
            // 前进动作序列
            SendServoCommand(60, 120, servo_angles_[2], servo_angles_[3]);
            vTaskDelay(pdMS_TO_TICKS(200));
            SendServoCommand(servo_angles_[0], servo_angles_[1], 60, 120);
            vTaskDelay(pdMS_TO_TICKS(200));
            SendServoCommand(90, 90, servo_angles_[2], servo_angles_[3]);
            vTaskDelay(pdMS_TO_TICKS(200));
            SendServoCommand(servo_angles_[0], servo_angles_[1], 90, 90);
        });

        methods_.AddMethod("后退", "向后退", ParameterList(), [this](const ParameterList& parameters) {
            // 后退动作序列
            SendServoCommand(servo_angles_[0], servo_angles_[1], 120, 60);
            vTaskDelay(pdMS_TO_TICKS(200));
            SendServoCommand(120, 60, servo_angles_[2], servo_angles_[3]);
            vTaskDelay(pdMS_TO_TICKS(200));
            SendServoCommand(servo_angles_[0], servo_angles_[1], 90, 90);
            vTaskDelay(pdMS_TO_TICKS(200));
            SendServoCommand(90, 90, servo_angles_[2], servo_angles_[3]);
        });

        methods_.AddMethod("左转", "向左转", ParameterList(), [this](const ParameterList& parameters) {
            // 左转动作序列
            SendServoCommand(120, servo_angles_[1], 120, servo_angles_[3]);
            vTaskDelay(pdMS_TO_TICKS(300));
            SendServoCommand(servo_angles_[0], 120, servo_angles_[2], 120);
            vTaskDelay(pdMS_TO_TICKS(300));
            SendServoCommand(90, 90, 90, 90);
        });

        methods_.AddMethod("右转", "向右转", ParameterList(), [this](const ParameterList& parameters) {
            // 右转动作序列
            SendServoCommand(servo_angles_[0], 60, servo_angles_[2], 60);
            vTaskDelay(pdMS_TO_TICKS(300));
            SendServoCommand(60, servo_angles_[1], 60, servo_angles_[3]);
            vTaskDelay(pdMS_TO_TICKS(300));
            SendServoCommand(90, 90, 90, 90);
        });

        methods_.AddMethod("休息", "让小狗休息，所有舵机恢复默认角度", ParameterList(), [this](const ParameterList& parameters) {
            // 所有舵机恢复90度默认位置
            SendServoCommand(90, 90, 90, 90);
            ESP_LOGI(TAG, "小狗进入休息状态，所有舵机已恢复默认角度");
        });

        methods_.AddMethod("打开灯光", "打开所有LED灯，显示白色", ParameterList(), [this](const ParameterList& parameters) {
            // 流水灯1和流水灯2都设置为白色（255,255,255）
            SendLedCommand(CMD_LED1, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255);
            SendLedCommand(CMD_LED2, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255);
            ESP_LOGI(TAG, "所有灯光已打开");
        });

        methods_.AddMethod("关闭灯光", "关闭所有LED灯", ParameterList(), [this](const ParameterList& parameters) {
            // 流水灯1和流水灯2都设置为黑色（0,0,0）
            SendLedCommand(CMD_LED1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
            SendLedCommand(CMD_LED2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
            ESP_LOGI(TAG, "所有灯光已关闭");
        });

        methods_.AddMethod("流水灯", "播放彩虹流水灯效果", ParameterList(), [this](const ParameterList& parameters) {
            ESP_LOGI(TAG, "开始播放流水灯效果");
            
            // 彩虹色序列：红、橙、黄、绿、蓝、靛、紫
            uint8_t colors[7][3] = {
                {255, 0, 0},    // 红
                {255, 127, 0},  // 橙
                {255, 255, 0},  // 黄
                {0, 255, 0},    // 绿
                {0, 0, 255},    // 蓝
                {75, 0, 130},   // 靛
                {148, 0, 211}   // 紫
            };
            
            // 播放10个循环的流水灯效果
            for (int cycle = 0; cycle < 10; cycle++) {
                for (int offset = 0; offset < 7; offset++) {
                    // 流水灯1
                    SendLedCommand(CMD_LED1, 
                        colors[(offset + 0) % 7][0], colors[(offset + 0) % 7][1], colors[(offset + 0) % 7][2],
                        colors[(offset + 1) % 7][0], colors[(offset + 1) % 7][1], colors[(offset + 1) % 7][2],
                        colors[(offset + 2) % 7][0], colors[(offset + 2) % 7][1], colors[(offset + 2) % 7][2],
                        colors[(offset + 3) % 7][0], colors[(offset + 3) % 7][1], colors[(offset + 3) % 7][2],
                        colors[(offset + 4) % 7][0], colors[(offset + 4) % 7][1], colors[(offset + 4) % 7][2]);
                    
                    // 流水灯2
                    SendLedCommand(CMD_LED2, 
                        colors[(offset + 2) % 7][0], colors[(offset + 2) % 7][1], colors[(offset + 2) % 7][2],
                        colors[(offset + 3) % 7][0], colors[(offset + 3) % 7][1], colors[(offset + 3) % 7][2],
                        colors[(offset + 4) % 7][0], colors[(offset + 4) % 7][1], colors[(offset + 4) % 7][2],
                        colors[(offset + 5) % 7][0], colors[(offset + 5) % 7][1], colors[(offset + 5) % 7][2],
                        colors[(offset + 6) % 7][0], colors[(offset + 6) % 7][1], colors[(offset + 6) % 7][2]);
                    
                    vTaskDelay(pdMS_TO_TICKS(150)); // 每150ms切换一次
                }
            }
            
            // 流水灯效果结束后关闭所有灯
            SendLedCommand(CMD_LED1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
            SendLedCommand(CMD_LED2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
            ESP_LOGI(TAG, "流水灯效果播放完成");
        });
    }
};

// 确保DogServo类的正确注册
static iot::Thing* CreateDogServo() {
    return new iot::DogServo();
}

// 注册Thing类型，确保名称与ThingManager::AddThing中的参数匹配
static bool RegisterDogServoHelper = []() {
    RegisterThing("DogServo", CreateDogServo);
    return true;
}();

} // namespace iot 