#include "drivers/pcf8574.h"
#include <esp_log.h>

#define TAG "PCF8574"

PCF8574::PCF8574(i2c_master_bus_handle_t i2c_bus, uint8_t addr)
    : I2cDevice(i2c_bus, addr), pin_states_(0xFF) {
    ESP_LOGI(TAG, "PCF8574 initialized at address 0x%02X", addr);
}

void PCF8574::Write(uint8_t value) {
    pin_states_ = value;
    
    // PCF8574直接写入值，不需要寄存器地址
    esp_err_t err = i2c_master_transmit(i2c_device_, &pin_states_, 1, 100);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Write failed: 0x%02X, error: %s", pin_states_, esp_err_to_name(err));
    } else {
        ESP_LOGD(TAG, "Write value: 0x%02X", pin_states_);
    }
}

uint8_t PCF8574::Read() {
    uint8_t data = 0;
    
    // PCF8574直接读取值，不需要寄存器地址
    esp_err_t err = i2c_master_receive(i2c_device_, &data, 1, 100);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Read failed: error: %s", esp_err_to_name(err));
    } else {
        pin_states_ = data;
        ESP_LOGD(TAG, "Read value: 0x%02X", pin_states_);
    }
    
    return pin_states_;
}

void PCF8574::SetPin(uint8_t pin, bool value) {
    if (pin > 7) {
        ESP_LOGE(TAG, "Invalid pin number: %d", pin);
        return;
    }
    
    if (value) {
        pin_states_ |= (1 << pin);  // 对PCF8574，高电平为1
    } else {
        pin_states_ &= ~(1 << pin); // 对PCF8574，低电平为0
    }
    
    Write(pin_states_);
}

bool PCF8574::GetPin(uint8_t pin) {
    if (pin > 7) {
        ESP_LOGE(TAG, "Invalid pin number: %d", pin);
        return false;
    }
    
    uint8_t data = Read();
    return (data & (1 << pin)) != 0;
} 