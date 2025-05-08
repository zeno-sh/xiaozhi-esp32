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
#define UART_BAUD_RATE  115200
#define BUF_SIZE        256

namespace iot {

class DogServo : public Thing {
private:
    void SendUartMessage(const char * command_str) {
        uint8_t len = strlen(command_str);
        uart_write_bytes(UART_NUM, command_str, len);
        ESP_LOGI(TAG, "发送命令: %s", command_str);
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
            SendUartMessage("SG90+LF:45\r\n");
            SendUartMessage("SG90+LR:45\r\n");
            SendUartMessage("SG90+RF:135\r\n");
            SendUartMessage("SG90+RR:135\r\n");
        });

        methods_.AddMethod("坐下", "让小狗坐下", ParameterList(), [this](const ParameterList& parameters) {
            // 所有舵机90度
            SendUartMessage("SG90+LF:90\r\n");
            SendUartMessage("SG90+LR:90\r\n");
            SendUartMessage("SG90+RF:90\r\n");
            SendUartMessage("SG90+RR:90\r\n");
        });
        
        methods_.AddMethod("摇尾", "让小狗摇尾巴", ParameterList(), [this](const ParameterList& parameters) {
            // 后腿左右摆动
            SendUartMessage("SG90+LR:70\r\n");
            SendUartMessage("SG90+RR:110\r\n");
            vTaskDelay(pdMS_TO_TICKS(500));
            SendUartMessage("SG90+LR:110\r\n");
            SendUartMessage("SG90+RR:70\r\n");
            vTaskDelay(pdMS_TO_TICKS(500));
            SendUartMessage("SG90+LR:90\r\n");
            SendUartMessage("SG90+RR:90\r\n");
        });

        methods_.AddMethod("前进", "向前走", ParameterList(), [this](const ParameterList& parameters) {
            // 前进动作序列
            SendUartMessage("SG90+LF:60\r\n");
            SendUartMessage("SG90+RF:120\r\n");
            vTaskDelay(pdMS_TO_TICKS(200));
            SendUartMessage("SG90+LR:60\r\n");
            SendUartMessage("SG90+RR:120\r\n");
            vTaskDelay(pdMS_TO_TICKS(200));
            SendUartMessage("SG90+LF:90\r\n");
            SendUartMessage("SG90+RF:90\r\n");
            vTaskDelay(pdMS_TO_TICKS(200));
            SendUartMessage("SG90+LR:90\r\n");
            SendUartMessage("SG90+RR:90\r\n");
        });

        methods_.AddMethod("后退", "向后退", ParameterList(), [this](const ParameterList& parameters) {
            // 后退动作序列
            SendUartMessage("SG90+LR:120\r\n");
            SendUartMessage("SG90+RR:60\r\n");
            vTaskDelay(pdMS_TO_TICKS(200));
            SendUartMessage("SG90+LF:120\r\n");
            SendUartMessage("SG90+RF:60\r\n");
            vTaskDelay(pdMS_TO_TICKS(200));
            SendUartMessage("SG90+LR:90\r\n");
            SendUartMessage("SG90+RR:90\r\n");
            vTaskDelay(pdMS_TO_TICKS(200));
            SendUartMessage("SG90+LF:90\r\n");
            SendUartMessage("SG90+RF:90\r\n");
        });

        methods_.AddMethod("左转", "向左转", ParameterList(), [this](const ParameterList& parameters) {
            // 左转动作序列
            SendUartMessage("SG90+LF:120\r\n");
            SendUartMessage("SG90+LR:120\r\n");
            vTaskDelay(pdMS_TO_TICKS(300));
            SendUartMessage("SG90+RF:120\r\n");
            SendUartMessage("SG90+RR:120\r\n");
            vTaskDelay(pdMS_TO_TICKS(300));
            SendUartMessage("SG90+LF:90\r\n");
            SendUartMessage("SG90+LR:90\r\n");
            SendUartMessage("SG90+RF:90\r\n");
            SendUartMessage("SG90+RR:90\r\n");
        });

        methods_.AddMethod("右转", "向右转", ParameterList(), [this](const ParameterList& parameters) {
            // 右转动作序列
            SendUartMessage("SG90+RF:60\r\n");
            SendUartMessage("SG90+RR:60\r\n");
            vTaskDelay(pdMS_TO_TICKS(300));
            SendUartMessage("SG90+LF:60\r\n");
            SendUartMessage("SG90+LR:60\r\n");
            vTaskDelay(pdMS_TO_TICKS(300));
            SendUartMessage("SG90+LF:90\r\n");
            SendUartMessage("SG90+LR:90\r\n");
            SendUartMessage("SG90+RF:90\r\n");
            SendUartMessage("SG90+RR:90\r\n");
        });

        methods_.AddMethod("LF", "设置左前腿角度", ParameterList({
            Parameter("角度", "角度值", kValueTypeNumber, true)
        }), [this](const ParameterList& parameters) {
            int angle = static_cast<int>(parameters["角度"].number());
            if (angle < SERVO_ANGLE_MIN) angle = SERVO_ANGLE_MIN;
            if (angle > SERVO_ANGLE_MAX) angle = SERVO_ANGLE_MAX;
            
            char command[20] = {0};
            snprintf(command, sizeof(command), "SG90+LF:%d\r\n", angle);
            SendUartMessage(command);
        });

        methods_.AddMethod("LR", "设置左后腿角度", ParameterList({
            Parameter("角度", "角度值", kValueTypeNumber, true)
        }), [this](const ParameterList& parameters) {
            int angle = static_cast<int>(parameters["角度"].number());
            if (angle < SERVO_ANGLE_MIN) angle = SERVO_ANGLE_MIN;
            if (angle > SERVO_ANGLE_MAX) angle = SERVO_ANGLE_MAX;
            
            char command[20] = {0};
            snprintf(command, sizeof(command), "SG90+LR:%d\r\n", angle);
            SendUartMessage(command);
        });

        methods_.AddMethod("RF", "设置右前腿角度", ParameterList({
            Parameter("角度", "角度值", kValueTypeNumber, true)
        }), [this](const ParameterList& parameters) {
            int angle = static_cast<int>(parameters["角度"].number());
            if (angle < SERVO_ANGLE_MIN) angle = SERVO_ANGLE_MIN;
            if (angle > SERVO_ANGLE_MAX) angle = SERVO_ANGLE_MAX;
            
            char command[20] = {0};
            snprintf(command, sizeof(command), "SG90+RF:%d\r\n", angle);
            SendUartMessage(command);
        });

        methods_.AddMethod("RR", "设置右后腿角度", ParameterList({
            Parameter("角度", "角度值", kValueTypeNumber, true)
        }), [this](const ParameterList& parameters) {
            int angle = static_cast<int>(parameters["角度"].number());
            if (angle < SERVO_ANGLE_MIN) angle = SERVO_ANGLE_MIN;
            if (angle > SERVO_ANGLE_MAX) angle = SERVO_ANGLE_MAX;
            
            char command[20] = {0};
            snprintf(command, sizeof(command), "SG90+RR:%d\r\n", angle);
            SendUartMessage(command);
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