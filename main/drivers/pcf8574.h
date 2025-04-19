#ifndef PCF8574_H
#define PCF8574_H

#include "boards/common/i2c_device.h"
#include <stdint.h>

class PCF8574 : public I2cDevice {
public:
    PCF8574(i2c_master_bus_handle_t i2c_bus, uint8_t addr);
    
    // 写入所有8位IO状态
    void Write(uint8_t value);
    
    // 读取所有8位IO状态
    uint8_t Read();
    
    // 设置单个IO口状态
    void SetPin(uint8_t pin, bool value);
    
    // 读取单个IO口状态
    bool GetPin(uint8_t pin);
    
private:
    uint8_t pin_states_; // 当前IO状态缓存
};

#endif // PCF8574_H 