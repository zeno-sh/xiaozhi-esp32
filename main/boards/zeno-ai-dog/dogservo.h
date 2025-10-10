#pragma once

#include "iot/thing.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <vector>
#include <memory>

// 前向声明
class IGaitPattern;
class GaitExecutor;

// LED动画状态枚举
enum LedAnimationState {
    LED_IDLE = 0,
    LED_STARTUP_FLASH,
    LED_LIGHT_SHOW,
    LED_BREATHING
};

/**
 * DogLED类 - AI狗的LED灯光和舵机控制类
 * 实现了SOLID原则，具备良好的扩展性和复用性
 */
class DogLED : public iot::Thing {
private:
    // UART通信相关
    SemaphoreHandle_t uart_mutex_;
    
    // LED动画相关
    TaskHandle_t animation_task_handle_;
    LedAnimationState current_state_;
    
    // 步态执行器
    std::unique_ptr<GaitExecutor> gait_executor_;
    
    // 私有方法
    void InitializeLedUart();
    
    // LED控制方法
    void SendLedCommandRaw(uint8_t command, uint8_t b, uint8_t r, uint8_t g);
    void SendLedCommand(uint8_t command, uint8_t b, uint8_t r, uint8_t g);
    void SetBothLeds(uint8_t b1, uint8_t r1, uint8_t g1, uint8_t b2, uint8_t r2, uint8_t g2);
    void SetBothLeds(uint8_t b, uint8_t r, uint8_t g);
    void SetLedGroup(uint8_t group, uint8_t b, uint8_t r, uint8_t g);
    
    // 舵机控制方法
    void SendServoCommand(uint8_t left_angle, uint8_t right_angle);
    void SendServoCommandRaw(uint8_t front_left, uint8_t front_right, uint8_t rear_left, uint8_t rear_right);
    void SendServoCommandGradual(uint8_t left_angle, uint8_t right_angle);
    
    // LED动画效果
    void StartupFlash();
    void LightShow();
    void BreathingLight();
    
    // 动画状态管理
    void SetAnimationState(LedAnimationState new_state);
    static void AnimationTask(void* parameter);
    
public:
    DogLED();
    ~DogLED();
    
    // 禁用拷贝构造和赋值操作符
    DogLED(const DogLED&) = delete;
    DogLED& operator=(const DogLED&) = delete;
};