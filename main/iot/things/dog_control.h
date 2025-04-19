#ifndef DOG_CONTROL_H
#define DOG_CONTROL_H

#include "iot/thing.h"
#include "drivers/pcf8574.h"
#include <memory>
#include <esp_timer.h>
#include <vector>

// 舵机控制引脚定义
#define SERVO_BACK_LEFT    0  // P0
#define SERVO_FRONT_LEFT   1  // P1
#define FLOW_LIGHT_1       2  // P2
#define FLOW_LIGHT_2       3  // P3
#define SERVO_BACK_RIGHT   4  // P4
#define SERVO_FRONT_RIGHT  5  // P5

// PWM参数定义 - 标准舵机
#define SERVO_PWM_FREQ     50  // 舵机要求50Hz PWM信号
#define SERVO_MIN_PULSE_US 500  // 最小脉宽 (0度)
#define SERVO_MAX_PULSE_US 2500 // 最大脉宽 (180度)
#define SERVO_PERIOD_US    20000 // 周期 (1000000/SERVO_PWM_FREQ)

namespace iot {

// 流水灯模式
enum FlowLightMode {
    FLOW_LIGHT_OFF = 0,    // 关闭
    FLOW_LIGHT_STATIC = 1, // 常亮
    FLOW_LIGHT_BLINK = 2,  // 闪烁
    FLOW_LIGHT_FLOW = 3    // 流水
};

// 舵机状态结构
struct ServoState {
    int pin;              // 舵机对应的引脚
    int angle;            // 舵机当前角度 (0-180)
    int pulse_width_us;   // 当前脉冲宽度(微秒)
    bool is_active;       // 舵机是否处于活动状态
    unsigned long next_toggle_time; // 下一次电平切换时间
    bool output_level;    // 当前输出电平
};

class DogControl : public Thing {
public:
    DogControl();
    ~DogControl();
    
    // 设置IO扩展器
    void SetIoExpander(PCF8574* io_expander);

private:
    // PCF8574 I2C IO扩展芯片
    std::unique_ptr<PCF8574> io_expander_;
    
    // 状态变量
    FlowLightMode light_mode_;
    bool is_moving_;
    int action_state_;
    
    // 定时器
    esp_timer_handle_t timer_handle_;
    esp_timer_handle_t pwm_timer_handle_; // PWM定时器
    
    // 舵机状态
    std::vector<ServoState> servo_states_;
    
    // 初始化I2C总线和设备
    bool InitI2C();
    
    // 定时器回调
    static void TimerCallback(void* arg);
    static void PwmTimerCallback(void* arg); // PWM定时器回调
    
    // 流水灯控制
    void SetFlowLightMode(FlowLightMode mode);
    void UpdateFlowLights();
    
    // 舵机控制 - 基础控制
    void SetServo(int servo, bool value);
    void SetAllServos(bool value);
    
    // 舵机控制 - PWM控制
    void InitServos();
    void SetServoAngle(int servo, int angle);
    void UpdateServoPwm();
    int AngleToPulseWidth(int angle);
    
    // 动作控制
    void UpdateMovement();
    void StopMovement();
    
    // 方法注册
    void RegisterMethods();
};

} // namespace iot

#endif // DOG_CONTROL_H