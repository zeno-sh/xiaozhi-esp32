#include "dog_control.h"
#include "drivers/pcf8574.h"
#include <esp_log.h>
#include <string>
#include <cstring>  // 添加对memset的支持
#include <driver/i2c_master.h>

#define TAG "DogControl"

// I2C配置 - 使用正确的硬件引脚
#define I2C_MASTER_NUM       0
#define I2C_MASTER_SDA_IO    GPIO_NUM_48  // 硬件工程师指定SDA为GPIO48
#define I2C_MASTER_SCL_IO    GPIO_NUM_38  // 硬件工程师指定SCL为GPIO38
#define PCF8574_I2C_ADDR     0x40         // HT8574ARSZ地址(7位)
#define I2C_MASTER_FREQ_HZ   100000

// 定时器配置
#define TIMER_PERIOD_MS      100  // 100ms定时器
#define PWM_TIMER_PERIOD_US  100  // 100微秒检查一次PWM状态

namespace iot {

DogControl::DogControl() 
    : Thing("DogControl", "小狗控制器-可控制舵机和流水灯模拟小狗运动"),
      light_mode_(FLOW_LIGHT_OFF),
      is_moving_(false),
      action_state_(0),
      timer_handle_(nullptr),
      pwm_timer_handle_(nullptr) {
    
    ESP_LOGI(TAG, "正在初始化小狗控制器...");
    
    // 初始化IO扩展芯片
    if (!InitI2C()) {
        ESP_LOGE(TAG, "IO扩展芯片初始化失败！");
        return;
    }
    
    // 初始化舵机状态
    InitServos();
    
    // 初始化100ms定时器(用于流水灯和动作控制)
    esp_timer_create_args_t timer_args = {
        .callback = &DogControl::TimerCallback,
        .arg = this,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "dog_timer",
        .skip_unhandled_events = true
    };
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &timer_handle_));
    ESP_ERROR_CHECK(esp_timer_start_periodic(timer_handle_, TIMER_PERIOD_MS * 1000));
    
    // 初始化PWM高精度定时器(用于舵机控制)
    esp_timer_create_args_t pwm_timer_args = {
        .callback = &DogControl::PwmTimerCallback,
        .arg = this,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "pwm_timer",
        .skip_unhandled_events = true
    };
    ESP_ERROR_CHECK(esp_timer_create(&pwm_timer_args, &pwm_timer_handle_));
    ESP_ERROR_CHECK(esp_timer_start_periodic(pwm_timer_handle_, PWM_TIMER_PERIOD_US));

    // 初始状态 - 所有舵机设为默认角度90度(中间位置)
    SetServoAngle(SERVO_FRONT_LEFT, 90);
    SetServoAngle(SERVO_FRONT_RIGHT, 90);
    SetServoAngle(SERVO_BACK_LEFT, 90);
    SetServoAngle(SERVO_BACK_RIGHT, 90);
    
    // 添加属性和方法
    RegisterMethods();
    
    ESP_LOGI(TAG, "小狗控制器初始化完成");
}

DogControl::~DogControl() {
    if (timer_handle_ != nullptr) {
        esp_timer_stop(timer_handle_);
        esp_timer_delete(timer_handle_);
    }
    
    if (pwm_timer_handle_ != nullptr) {
        esp_timer_stop(pwm_timer_handle_);
        esp_timer_delete(pwm_timer_handle_);
    }
}

void DogControl::TimerCallback(void* arg) {
    DogControl* dog = static_cast<DogControl*>(arg);
    dog->UpdateFlowLights();
    
    if (dog->is_moving_) {
        dog->UpdateMovement();
    }
}

void DogControl::PwmTimerCallback(void* arg) {
    DogControl* dog = static_cast<DogControl*>(arg);
    dog->UpdateServoPwm();
}

bool DogControl::InitI2C() {
    // 使用C风格零初始化避免字段顺序问题
    i2c_master_bus_config_t i2c_bus_config;
    memset(&i2c_bus_config, 0, sizeof(i2c_bus_config));
    
    // 按照ESP-IDF文档中展示的顺序设置字段
    i2c_bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
    i2c_bus_config.i2c_port = I2C_MASTER_NUM;
    i2c_bus_config.scl_io_num = I2C_MASTER_SCL_IO;
    i2c_bus_config.sda_io_num = I2C_MASTER_SDA_IO;
    i2c_bus_config.glitch_ignore_cnt = 7;
    i2c_bus_config.flags.enable_internal_pullup = true;
    
    // 创建I2C总线
    i2c_master_bus_handle_t i2c_bus = nullptr;
    esp_err_t ret = i2c_new_master_bus(&i2c_bus_config, &i2c_bus);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C总线初始化失败: %s", esp_err_to_name(ret));
        return false;
    }
    
    // 创建PCF8574设备 - 注意这里使用的是PCF8574兼容芯片HT8574ARSZ
    i2c_device_config_t dev_cfg = {};
    dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_cfg.device_address = PCF8574_I2C_ADDR;
    dev_cfg.scl_speed_hz = I2C_MASTER_FREQ_HZ;
    
    i2c_master_dev_handle_t i2c_dev = nullptr;
    ret = i2c_master_bus_add_device(i2c_bus, &dev_cfg, &i2c_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C设备创建失败: %s", esp_err_to_name(ret));
        return false;
    }
    
    // 创建IO扩展器
    io_expander_ = std::make_unique<PCF8574>(i2c_bus, PCF8574_I2C_ADDR);
    
    // 初始化IO引脚全部为输出低电平
    io_expander_->Write(0x00);
    ESP_LOGI(TAG, "IO扩展器初始化成功");
    
    return true;
}

void DogControl::InitServos() {
    // 初始化舵机状态数组
    servo_states_.resize(6);  // 支持6个通道
    
    // 设置每个舵机的初始状态
    for (int i = 0; i < servo_states_.size(); i++) {
        servo_states_[i].pin = i;
        servo_states_[i].angle = 90;  // 默认中间位置
        servo_states_[i].pulse_width_us = AngleToPulseWidth(90);
        servo_states_[i].is_active = false;
        servo_states_[i].output_level = false;
        servo_states_[i].next_toggle_time = 0;
    }
    
    ESP_LOGI(TAG, "舵机状态初始化完成");
}

int DogControl::AngleToPulseWidth(int angle) {
    // 将角度(0-180)转换为脉冲宽度(us)
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;
    
    // 线性映射: 角度0-180 -> 脉宽500-2500us
    return SERVO_MIN_PULSE_US + (angle * (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US) / 180);
}

void DogControl::UpdateServoPwm() {
    // 获取当前时间
    static unsigned long time_us = 0;
    time_us += PWM_TIMER_PERIOD_US;
    
    // 每个周期开始时重置计时器
    if (time_us >= SERVO_PERIOD_US) {
        time_us = 0;
        
        // 周期开始时，所有活动舵机设为高电平
        for (auto& servo : servo_states_) {
            if (servo.is_active) {
                servo.output_level = true;
                servo.next_toggle_time = servo.pulse_width_us;
                
                // 更新IO扩展芯片输出
                if (io_expander_) {
                    io_expander_->SetPin(servo.pin, true);
                }
            }
        }
    }
    
    // 检查每个舵机是否需要切换电平
    for (auto& servo : servo_states_) {
        if (servo.is_active && servo.output_level && time_us >= servo.next_toggle_time) {
            servo.output_level = false;
            
            // 更新IO扩展芯片输出
            if (io_expander_) {
                io_expander_->SetPin(servo.pin, false);
            }
        }
    }
}

void DogControl::SetServoAngle(int servo, int angle) {
    if (servo < 0 || servo >= servo_states_.size()) {
        ESP_LOGE(TAG, "无效的舵机编号: %d", servo);
        return;
    }
    
    // 更新舵机状态
    servo_states_[servo].angle = angle;
    servo_states_[servo].pulse_width_us = AngleToPulseWidth(angle);
    servo_states_[servo].is_active = true;
    
    ESP_LOGI(TAG, "舵机 %d 设置角度为 %d 度, 脉宽为 %d us", 
        servo, angle, servo_states_[servo].pulse_width_us);
}

void DogControl::SetFlowLightMode(FlowLightMode mode) {
    light_mode_ = mode;
    
    // 立即更新灯光状态
    if (mode == FLOW_LIGHT_OFF) {
        if (io_expander_) {
            io_expander_->SetPin(FLOW_LIGHT_1, false);
            io_expander_->SetPin(FLOW_LIGHT_2, false);
        }
    } else if (mode == FLOW_LIGHT_STATIC) {
        if (io_expander_) {
            io_expander_->SetPin(FLOW_LIGHT_1, true);
            io_expander_->SetPin(FLOW_LIGHT_2, true);
        }
    }
}

void DogControl::UpdateFlowLights() {
    static int flow_light_state = 0;
    static bool blink_state = false;
    
    if (!io_expander_) {
        return;
    }
    
    switch (light_mode_) {
        case FLOW_LIGHT_OFF:
            // 关闭所有灯
            io_expander_->SetPin(FLOW_LIGHT_1, false);
            io_expander_->SetPin(FLOW_LIGHT_2, false);
            break;
            
        case FLOW_LIGHT_STATIC:
            // 常亮模式，不需要更新
            break;
            
        case FLOW_LIGHT_BLINK:
            // 闪烁模式
            blink_state = !blink_state;
            io_expander_->SetPin(FLOW_LIGHT_1, blink_state);
            io_expander_->SetPin(FLOW_LIGHT_2, blink_state);
            break;
            
        case FLOW_LIGHT_FLOW:
            // 流水模式
            flow_light_state = (flow_light_state + 1) % 4;
            switch (flow_light_state) {
                case 0:
                    io_expander_->SetPin(FLOW_LIGHT_1, true);
                    io_expander_->SetPin(FLOW_LIGHT_2, false);
                    break;
                case 1:
                    io_expander_->SetPin(FLOW_LIGHT_1, true);
                    io_expander_->SetPin(FLOW_LIGHT_2, true);
                    break;
                case 2:
                    io_expander_->SetPin(FLOW_LIGHT_1, false);
                    io_expander_->SetPin(FLOW_LIGHT_2, true);
                    break;
                case 3:
                    io_expander_->SetPin(FLOW_LIGHT_1, false);
                    io_expander_->SetPin(FLOW_LIGHT_2, false);
                    break;
            }
            break;
    }
}

// 保留原来的简单舵机控制方法，但内部实现修改为角度控制
void DogControl::SetServo(int servo, bool value) {
    // 高电平(value=true)时舵机转到最大角度180度
    // 低电平(value=false)时舵机转到最小角度0度
    SetServoAngle(servo, value ? 180 : 0);
}

void DogControl::SetAllServos(bool value) {
    SetServo(SERVO_FRONT_LEFT, value);
    SetServo(SERVO_FRONT_RIGHT, value);
    SetServo(SERVO_BACK_LEFT, value);
    SetServo(SERVO_BACK_RIGHT, value);
}

void DogControl::UpdateMovement() {
    // 根据不同的动作更新舵机状态
    action_state_ = (action_state_ + 1) % 4;
    
    if (!io_expander_) {
        return;
    }
    
    if (light_mode_ == FLOW_LIGHT_FLOW) {
        // 行走模式 - 对角线腿交替抬起放下
        switch (action_state_) {
            case 0:
                SetServoAngle(SERVO_FRONT_LEFT, 150);
                SetServoAngle(SERVO_BACK_RIGHT, 150);
                SetServoAngle(SERVO_FRONT_RIGHT, 30);
                SetServoAngle(SERVO_BACK_LEFT, 30);
                break;
            case 2:
                SetServoAngle(SERVO_FRONT_LEFT, 30);
                SetServoAngle(SERVO_BACK_RIGHT, 30);
                SetServoAngle(SERVO_FRONT_RIGHT, 150);
                SetServoAngle(SERVO_BACK_LEFT, 150);
                break;
        }
    } else if (light_mode_ == FLOW_LIGHT_BLINK) {
        if (is_moving_) {
            // 奔跑模式 - 前后腿交替抬起放下
            switch (action_state_) {
                case 0:
                    SetServoAngle(SERVO_FRONT_LEFT, 150);
                    SetServoAngle(SERVO_FRONT_RIGHT, 150);
                    SetServoAngle(SERVO_BACK_LEFT, 30);
                    SetServoAngle(SERVO_BACK_RIGHT, 30);
                    break;
                case 2:
                    SetServoAngle(SERVO_FRONT_LEFT, 30);
                    SetServoAngle(SERVO_FRONT_RIGHT, 30);
                    SetServoAngle(SERVO_BACK_LEFT, 150);
                    SetServoAngle(SERVO_BACK_RIGHT, 150);
                    break;
            }
        } else {
            // 摇头模式 - 仅控制前腿
            switch (action_state_) {
                case 0:
                    SetServoAngle(SERVO_FRONT_LEFT, 150);
                    SetServoAngle(SERVO_FRONT_RIGHT, 30);
                    break;
                case 2:
                    SetServoAngle(SERVO_FRONT_LEFT, 30);
                    SetServoAngle(SERVO_FRONT_RIGHT, 150);
                    break;
            }
        }
    }
}

void DogControl::StopMovement() {
    is_moving_ = false;
    
    // 停止所有动作，所有舵机回到中间位置
    SetServoAngle(SERVO_FRONT_LEFT, 90);
    SetServoAngle(SERVO_FRONT_RIGHT, 90);
    SetServoAngle(SERVO_BACK_LEFT, 90);
    SetServoAngle(SERVO_BACK_RIGHT, 90);
    
    // 关闭流水灯
    SetFlowLightMode(FLOW_LIGHT_OFF);
}

void DogControl::SetIoExpander(PCF8574* io_expander) {
    ESP_LOGI(TAG, "设置外部IO扩展器");
    if (io_expander) {
        io_expander_ = std::unique_ptr<PCF8574>(io_expander);
        
        // 初始化时设置所有IO为高电平
        io_expander_->Write(0xFF);
        ESP_LOGI(TAG, "使用外部PCF8574设置成功");
        
        // 重新初始化舵机
        InitServos();
    } else {
        ESP_LOGE(TAG, "设置IO扩展器失败：传入的扩展器为空");
    }
}

void DogControl::RegisterMethods() {
    // 添加属性 - 用于状态查询和监控
    properties_.AddNumberProperty("light_mode", "流水灯模式(0-关闭,1-常亮,2-闪烁,3-流水)", [this]() -> int {
        return static_cast<int>(light_mode_);
    });
    
    properties_.AddBooleanProperty("is_moving", "小狗是否在运动中", [this]() -> bool {
        return is_moving_;
    });
    
    // 添加方法 - 灯光控制
    methods_.AddMethod("OpenLight", "打开流水灯", ParameterList(), [this](const ParameterList& parameters) {
        ESP_LOGI(TAG, "打开流水灯");
        SetFlowLightMode(FLOW_LIGHT_STATIC);
    });
    
    methods_.AddMethod("CloseLight", "关闭流水灯", ParameterList(), [this](const ParameterList& parameters) {
        ESP_LOGI(TAG, "关闭流水灯");
        SetFlowLightMode(FLOW_LIGHT_OFF);
    });
    
    methods_.AddMethod("LightShow", "开启灯光秀", ParameterList(), [this](const ParameterList& parameters) {
        ESP_LOGI(TAG, "开启灯光秀");
        SetFlowLightMode(FLOW_LIGHT_FLOW);
    });
    
    methods_.AddMethod("FlashLight", "闪烁灯光", ParameterList(), [this](const ParameterList& parameters) {
        ESP_LOGI(TAG, "闪烁灯光");
        SetFlowLightMode(FLOW_LIGHT_BLINK);
    });
    
    methods_.AddMethod("SetLightMode", "设置流水灯模式", ParameterList({
        Parameter("mode", "流水灯模式(0-关闭,1-常亮,2-闪烁,3-流水)", kValueTypeNumber, true)
    }), [this](const ParameterList& parameters) {
        int mode = parameters["mode"].number();
        if (mode >= FLOW_LIGHT_OFF && mode <= FLOW_LIGHT_FLOW) {
            SetFlowLightMode(static_cast<FlowLightMode>(mode));
        } else {
            ESP_LOGE(TAG, "无效的流水灯模式: %d", mode);
        }
    });
    
    // 添加方法 - 舵机角度精确控制
    methods_.AddMethod("SetServoAngle", "设置舵机角度", ParameterList({
        Parameter("servo", "舵机编号(0-5)", kValueTypeNumber, true),
        Parameter("angle", "角度(0-180)", kValueTypeNumber, true)
    }), [this](const ParameterList& parameters) {
        int servo = parameters["servo"].number();
        int angle = parameters["angle"].number();
        
        ESP_LOGI(TAG, "设置舵机 %d 角度为 %d", servo, angle);
        SetServoAngle(servo, angle);
    });
    
    // 添加方法 - 小狗基本动作
    methods_.AddMethod("SitDown", "小狗坐下", ParameterList(), [this](const ParameterList& parameters) {
        ESP_LOGI(TAG, "小狗坐下");
        is_moving_ = false;
        action_state_ = 0;
        
        // 前腿抬起，后腿放下
        SetServoAngle(SERVO_FRONT_LEFT, 150);
        SetServoAngle(SERVO_FRONT_RIGHT, 150);
        SetServoAngle(SERVO_BACK_LEFT, 30);
        SetServoAngle(SERVO_BACK_RIGHT, 30);
        
        // 设置流水灯为常亮
        SetFlowLightMode(FLOW_LIGHT_STATIC);
    });
    
    methods_.AddMethod("StandUp", "小狗站立", ParameterList(), [this](const ParameterList& parameters) {
        ESP_LOGI(TAG, "小狗站立");
        is_moving_ = false;
        action_state_ = 0;
        
        // 所有腿都抬起
        SetServoAngle(SERVO_FRONT_LEFT, 150);
        SetServoAngle(SERVO_FRONT_RIGHT, 150);
        SetServoAngle(SERVO_BACK_LEFT, 150);
        SetServoAngle(SERVO_BACK_RIGHT, 150);
        
        // 设置流水灯为常亮
        SetFlowLightMode(FLOW_LIGHT_STATIC);
    });
    
    methods_.AddMethod("Walk", "小狗行走", ParameterList(), [this](const ParameterList& parameters) {
        ESP_LOGI(TAG, "小狗行走");
        is_moving_ = true;
        action_state_ = 0;
        
        // 设置流水灯为流水模式
        SetFlowLightMode(FLOW_LIGHT_FLOW);
    });
    
    methods_.AddMethod("Run", "小狗奔跑", ParameterList(), [this](const ParameterList& parameters) {
        ESP_LOGI(TAG, "小狗奔跑");
        is_moving_ = true;
        action_state_ = 0;
        
        // 设置流水灯为闪烁模式
        SetFlowLightMode(FLOW_LIGHT_BLINK);
    });
    
    methods_.AddMethod("Stop", "小狗停止", ParameterList(), [this](const ParameterList& parameters) {
        ESP_LOGI(TAG, "小狗停止");
        StopMovement();
    });
    
    methods_.AddMethod("ShakeHead", "小狗摇头", ParameterList(), [this](const ParameterList& parameters) {
        ESP_LOGI(TAG, "小狗摇头 - 控制前腿交替抬起放下");
        is_moving_ = false;
        action_state_ = 0;
        
        // 设置流水灯为闪烁模式
        SetFlowLightMode(FLOW_LIGHT_BLINK);
        
        // 控制前腿交替抬起放下
        SetServoAngle(SERVO_FRONT_LEFT, 150);
        SetServoAngle(SERVO_FRONT_RIGHT, 30);
        SetServoAngle(SERVO_BACK_LEFT, 30);
        SetServoAngle(SERVO_BACK_RIGHT, 30);
    });
    
    // 添加方法 - 小狗复合动作
    methods_.AddMethod("Happy", "小狗开心", ParameterList(), [this](const ParameterList& parameters) {
        ESP_LOGI(TAG, "小狗开心 - 摇头并闪烁灯光");
        is_moving_ = true;
        action_state_ = 0;
        
        // 设置流水灯为闪烁模式
        SetFlowLightMode(FLOW_LIGHT_BLINK);
    });
    
    methods_.AddMethod("Dance", "小狗跳舞", ParameterList(), [this](const ParameterList& parameters) {
        ESP_LOGI(TAG, "小狗跳舞 - 交替抬腿并流水灯");
        is_moving_ = true;
        action_state_ = 0;
        
        // 设置流水灯为流水模式
        SetFlowLightMode(FLOW_LIGHT_FLOW);
    });
    
    methods_.AddMethod("LeftPawUp", "左前爪抬起", ParameterList(), [this](const ParameterList& parameters) {
        ESP_LOGI(TAG, "左前爪抬起");
        SetServoAngle(SERVO_FRONT_LEFT, 150);
        SetServoAngle(SERVO_FRONT_RIGHT, 30);
        SetServoAngle(SERVO_BACK_LEFT, 30);
        SetServoAngle(SERVO_BACK_RIGHT, 30);
    });
    
    methods_.AddMethod("RightPawUp", "右前爪抬起", ParameterList(), [this](const ParameterList& parameters) {
        ESP_LOGI(TAG, "右前爪抬起");
        SetServoAngle(SERVO_FRONT_LEFT, 30);
        SetServoAngle(SERVO_FRONT_RIGHT, 150);
        SetServoAngle(SERVO_BACK_LEFT, 30);
        SetServoAngle(SERVO_BACK_RIGHT, 30);
    });
    
    methods_.AddMethod("BothPawsUp", "双爪抬起", ParameterList(), [this](const ParameterList& parameters) {
        ESP_LOGI(TAG, "双爪抬起");
        SetServoAngle(SERVO_FRONT_LEFT, 150);
        SetServoAngle(SERVO_FRONT_RIGHT, 150);
        SetServoAngle(SERVO_BACK_LEFT, 30);
        SetServoAngle(SERVO_BACK_RIGHT, 30);
    });
    
    methods_.AddMethod("StretchLegs", "伸展腿部", ParameterList(), [this](const ParameterList& parameters) {
        ESP_LOGI(TAG, "伸展腿部 - 交替伸展前后腿");
        is_moving_ = true;
        action_state_ = 0;
        
        // 前腿放下，后腿抬起
        SetServoAngle(SERVO_FRONT_LEFT, 30);
        SetServoAngle(SERVO_FRONT_RIGHT, 30);
        SetServoAngle(SERVO_BACK_LEFT, 150);
        SetServoAngle(SERVO_BACK_RIGHT, 150);
    });
}

} // namespace iot

// 注册Thing
DECLARE_THING(DogControl); 