# Zeno AI Robot 开发板

这是一个为Zeno AI机器人设计的ESP32S3开发板，支持语音交互、显示屏及机器狗控制功能。

## 硬件规格

- **MCU**: ESP32-S3 (双核处理器，支持WiFi和蓝牙)
- **显示屏**: ST7789 SPI显示屏 (320×240分辨率)
- **音频**: 集成ES8311音频编解码器和ES7210麦克风阵列芯片
- **IO扩展**:
  - HT8574ARSZ (用于机器狗舵机和流水灯控制)
  - PCA9557 (用于显示屏控制)
- **其他接口**:
  - I2S音频接口
  - SPI显示屏接口
  - 两组I2C总线

## 引脚定义

### 音频配置
- MCLK: GPIO39
- BCLK: GPIO14
- WS: GPIO13
- DIN: GPIO12
- DOUT: GPIO45

### I2C配置
- 音频编解码器I2C: SDA=GPIO1, SCL=GPIO2
- IO扩展I2C: SDA=GPIO48, SCL=GPIO38

### 显示屏配置
- SPI MOSI: GPIO40
- SPI CLK: GPIO41
- DC: GPIO39
- 背光控制: GPIO42

### 按键配置
- BOOT按钮: GPIO0

## 使用方法

1. 使用官方的编译脚本编译固件:
   ```
   python scripts/release.py zeno-ai-robot
   ```

2. 连接USB-TTL，使用esptool烧录固件:
   ```
   python -m esptool.py --chip esp32s3 --port /dev/ttyUSB0 --baud 921600 write_flash 0x0 build/zeno-ai-robot.bin
   ```

3. 首次启动后，按下BOOT按钮进入配网模式，连接到"xiaozhi-xxxx"WiFi热点进行配置

## 故障排除

1. 如果I2C设备初始化失败，检查硬件连接以及相关设备地址设置
2. 如果显示屏无法正常工作，可能是PCA9557芯片未正确初始化
3. 如果机器狗控制失效，检查HT8574ARSZ芯片连接和地址设置

## 开发说明

本开发板采用模块化设计，将I2C设备、显示屏等组件初始化放在不同函数中，使代码更易于维护。同时添加了错误处理机制，确保即使某个组件初始化失败，也不会导致整个系统崩溃。 