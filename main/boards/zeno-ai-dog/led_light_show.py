#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
AIDog LED流水灯炫酷控制演示
控制流水灯2(命令0x03)实现传递、闪烁等炫酷效果
"""

import serial
import time
import glob
import sys
import math
import random

def find_available_ports():
    """查找可用的串口设备"""
    patterns = [
        '/dev/tty.usbmodem*',
        '/dev/tty.usbserial*', 
        '/dev/tty.USB*',
        '/dev/ttyUSB*',
        '/dev/ttyACM*'
    ]
    
    ports = []
    for pattern in patterns:
        ports.extend(glob.glob(pattern))
    
    return sorted(ports)

def send_led_command(ser, led_colors, show_output=True):
    """
    发送LED流水灯2控制命令
    
    Args:
        ser: 串口对象
        led_colors: 5个LED的颜色列表，每个元素为(B, R, G)元组
        show_output: 是否显示输出
    """
    try:
        # 构建数据字节 (15字节，每个LED 3字节：B, R, G)
        data_bytes = []
        for b, r, g in led_colors:
            data_bytes.extend([b, r, g])
        
        # 构建完整命令（不带校验位）
        command = bytearray([0xAA, 0x11, 0x03] + data_bytes)
        
        # 发送命令
        ser.write(command)
        ser.flush()
        
        if show_output:
            # 显示当前LED状态
            led_status = []
            for i, (b, r, g) in enumerate(led_colors if led_colors else [(0,0,0)]*5):
                if b > 0 or r > 0 or g > 0:
                    color_desc = get_color_description(b, r, g)
                    led_status.append(f"LED{i+1}:{color_desc}")
            
            if led_status:
                print(f"\r{'|'.join(led_status)}", end='', flush=True)
            else:
                print(f"\r所有LED关闭", end='', flush=True)
        
        return command
        
    except Exception as e:
        print(f"❌ 发送LED命令失败: {e}")
        return None

def get_color_description(b, r, g):
    """根据BGR值返回颜色描述"""
    if r > 200 and g < 50 and b < 50:
        return "红"
    elif r < 50 and g > 200 and b < 50:
        return "绿"
    elif r < 50 and g < 50 and b > 200:
        return "蓝"
    elif r > 200 and g > 200 and b < 50:
        return "黄"
    elif r > 200 and g < 50 and b > 200:
        return "紫"
    elif r < 50 and g > 200 and b > 200:
        return "青"
    elif r > 200 and g > 200 and b > 200:
        return "白"
    elif r > 100 or g > 100 or b > 100:
        return "彩"
    else:
        return "暗"

def get_rainbow_color(index, total, brightness=255):
    """获取彩虹色谱中的颜色，返回BGR格式"""
    # 使用HSV色彩空间生成彩虹色
    hue = (index / total) * 360  # 色相：0-360度
    
    # 简化的HSV到RGB转换
    c = brightness
    x = c * (1 - abs((hue / 60) % 2 - 1))
    m = 0
    
    if 0 <= hue < 60:
        r, g, b = c, x, 0
    elif 60 <= hue < 120:
        r, g, b = x, c, 0
    elif 120 <= hue < 180:
        r, g, b = 0, c, x
    elif 180 <= hue < 240:
        r, g, b = 0, x, c
    elif 240 <= hue < 300:
        r, g, b = x, 0, c
    else:
        r, g, b = c, 0, x
    
    return (int(b), int(r), int(g))  # 返回BGR格式

def get_fire_color(intensity):
    """获取火焰效果颜色，返回BGR格式"""
    # 火焰色：红→橙→黄→白
    if intensity < 0.3:
        # 深红 (B=0, R=强度值, G=0)
        return (0, int(255 * intensity / 0.3), 0)
    elif intensity < 0.6:
        # 红→橙 (B=0, R=255, G=渐增)
        progress = (intensity - 0.3) / 0.3
        return (0, 255, int(128 * progress))
    elif intensity < 0.9:
        # 橙→黄 (B=0, R=255, G=继续增)
        progress = (intensity - 0.6) / 0.3
        return (0, 255, int(128 + 127 * progress))
    else:
        # 黄→白 (B=渐增, R=255, G=255)
        progress = (intensity - 0.9) / 0.1
        blue_val = int(255 * progress)
        return (blue_val, 255, 255)

def select_port():
    """选择串口设备"""
    ports = find_available_ports()
    
    if not ports:
        print("未找到可用的串口设备")
        print("请确保设备已连接，或手动指定串口路径")
        manual_port = input("手动输入串口路径 (按Enter跳过): ").strip()
        return manual_port if manual_port else None
    
    print("找到以下可用串口设备:")
    for i, port in enumerate(ports):
        print(f"{i+1}. {port}")
    
    while True:
        try:
            choice = input(f"请选择串口 (1-{len(ports)}) 或手动输入路径: ").strip()
            
            if choice.isdigit():
                idx = int(choice) - 1
                if 0 <= idx < len(ports):
                    return ports[idx]
                else:
                    print("无效选择，请重试")
            else:
                return choice if choice else None
                
        except KeyboardInterrupt:
            print("\n用户取消")
            return None

def test_connection(port, baudrate=115200):
    """测试串口连接"""
    try:
        ser = serial.Serial(port, baudrate, timeout=1)
        print(f"✓ 串口连接成功: {port}")
        return ser
    except Exception as e:
        print(f"✗ 串口连接失败: {e}")
        return None

def cascade_effect(ser, delay=0.2):
    """
    传递效果：从1号灯逐步亮到5号，然后逐步递减
    """
    print("\n🌈 开始传递效果...")
    print("-" * 50)
    
    try:
        # 阶段1：逐步增加 1→2→3→4→5
        print("阶段1: 逐步点亮 (1→2→3→4→5)")
        for stage in range(1, 6):  # 1到5个灯
            colors = []
            for i in range(stage):
                # 使用彩虹色，每个灯不同颜色
                color = get_rainbow_color(i, 5, 255)
                colors.append(color)
            
            # 剩余灯光关闭
            while len(colors) < 5:
                colors.append((0, 0, 0))
            
            send_led_command(ser, colors)
            print(f"\n  点亮{stage}个LED")
            time.sleep(delay)
        
        time.sleep(0.5)
        
        # 阶段2：逐步递减 5→4→3→2→1
        print("\n阶段2: 逐步递减 (5→4→3→2→1)")
        for stage in range(4, 0, -1):  # 4到1个灯
            colors = []
            for i in range(stage):
                # 使用火焰色效果
                intensity = (i + 1) / stage
                color = get_fire_color(intensity)
                colors.append(color)
            
            # 剩余灯光关闭
            while len(colors) < 5:
                colors.append((0, 0, 0))
            
            send_led_command(ser, colors)
            print(f"\n  点亮{stage}个LED")
            time.sleep(delay)
        
        time.sleep(0.5)
        
        # 阶段3：全部点亮
        print("\n阶段3: 全部点亮")
        all_colors = []
        for i in range(5):
            color = get_rainbow_color(i, 5, 255)
            all_colors.append(color)
        
        send_led_command(ser, all_colors)
        print("\n  🌟 全部LED点亮！")
        time.sleep(1.0)
        
    except KeyboardInterrupt:
        print("\n\n⏹️ 用户中断传递效果")
    finally:
        # 关闭所有LED
        turn_off_all_leds(ser)

def blink_effect(ser, blink_count=6, blink_delay=0.3):
    """
    闪烁效果
    """
    print(f"\n⚡ 开始闪烁效果 ({blink_count}次)...")
    print("-" * 50)
    
    try:
        # 准备闪烁的颜色
        bright_colors = []
        for i in range(5):
            color = get_rainbow_color(i, 5, 255)
            bright_colors.append(color)
        
        off_colors = [(0, 0, 0)] * 5
        
        for i in range(blink_count):
            # 亮
            send_led_command(ser, bright_colors)
            print(f"\r  闪烁 {i+1}/{blink_count} - 亮", end='', flush=True)
            time.sleep(blink_delay)
            
            # 灭
            send_led_command(ser, off_colors)
            print(f"\r  闪烁 {i+1}/{blink_count} - 灭", end='', flush=True)
            time.sleep(blink_delay)
        
        print("\n  ✨ 闪烁效果完成！")
        
    except KeyboardInterrupt:
        print("\n\n⏹️ 用户中断闪烁效果")
    finally:
        # 关闭所有LED
        turn_off_all_leds(ser)

def breathing_effect(ser, cycles=3, steps=20):
    """
    呼吸灯效果
    """
    print(f"\n💨 开始呼吸灯效果 ({cycles}轮)...")
    print("-" * 50)
    
    try:
        for cycle in range(cycles):
            print(f"\n第{cycle+1}轮呼吸:")
            
            # 渐亮
            for step in range(steps + 1):
                intensity = step / steps
                colors = []
                for i in range(5):
                    base_color = get_rainbow_color(i, 5, 255)
                    color = (
                        int(base_color[0] * intensity),
                        int(base_color[1] * intensity),
                        int(base_color[2] * intensity)
                    )
                    colors.append(color)
                
                send_led_command(ser, colors)
                print(f"\r  渐亮 {int(intensity*100)}%", end='', flush=True)
                time.sleep(0.05)
            
            time.sleep(0.2)
            
            # 渐暗
            for step in range(steps, -1, -1):
                intensity = step / steps
                colors = []
                for i in range(5):
                    base_color = get_rainbow_color(i, 5, 255)
                    color = (
                        int(base_color[0] * intensity),
                        int(base_color[1] * intensity),
                        int(base_color[2] * intensity)
                    )
                    colors.append(color)
                
                send_led_command(ser, colors)
                print(f"\r  渐暗 {int(intensity*100)}%", end='', flush=True)
                time.sleep(0.05)
            
            time.sleep(0.2)
        
        print("\n  🌙 呼吸灯效果完成！")
        
    except KeyboardInterrupt:
        print("\n\n⏹️ 用户中断呼吸灯效果")
    finally:
        # 关闭所有LED
        turn_off_all_leds(ser)

def wave_effect(ser, cycles=3, delay=0.1):
    """
    波浪效果
    """
    print(f"\n🌊 开始波浪效果 ({cycles}轮)...")
    print("-" * 50)
    
    try:
        for cycle in range(cycles):
            print(f"\n第{cycle+1}轮波浪:")
            
            # 正向波浪
            for pos in range(8):  # 波浪位置
                colors = []
                for i in range(5):
                    # 计算每个LED的亮度（基于与波浪中心的距离）
                    distance = abs(i - pos)
                    if distance <= 1:
                        intensity = 1.0 - distance * 0.3
                    elif distance <= 2:
                        intensity = 0.4 - (distance - 2) * 0.2
                    else:
                        intensity = 0
                    
                    base_color = get_rainbow_color(i, 5, 255)
                    color = (
                        int(base_color[0] * intensity),
                        int(base_color[1] * intensity),
                        int(base_color[2] * intensity)
                    )
                    colors.append(color)
                
                send_led_command(ser, colors)
                print(f"\r  波浪位置: {pos}", end='', flush=True)
                time.sleep(delay)
            
            # 反向波浪
            for pos in range(6, -2, -1):
                colors = []
                for i in range(5):
                    distance = abs(i - pos)
                    if distance <= 1:
                        intensity = 1.0 - distance * 0.3
                    elif distance <= 2:
                        intensity = 0.4 - (distance - 2) * 0.2
                    else:
                        intensity = 0
                    
                    base_color = get_rainbow_color(i, 5, 255)
                    color = (
                        int(base_color[0] * intensity),
                        int(base_color[1] * intensity),
                        int(base_color[2] * intensity)
                    )
                    colors.append(color)
                
                send_led_command(ser, colors)
                print(f"\r  波浪位置: {pos}", end='', flush=True)
                time.sleep(delay)
        
        print("\n  🌊 波浪效果完成！")
        
    except KeyboardInterrupt:
        print("\n\n⏹️ 用户中断波浪效果")
    finally:
        # 关闭所有LED
        turn_off_all_leds(ser)

def turn_off_all_leds(ser):
    """关闭所有LED"""
    print("\n🔌 关闭所有LED...")
    off_colors = [(0, 0, 0)] * 5
    send_led_command(ser, off_colors)
    print("  ✓ 所有LED已关闭")
    time.sleep(0.5)

def complete_light_show(ser):
    """完整的灯光秀"""
    print("\n🎭 开始完整灯光秀...")
    print("=" * 60)
    
    try:
        # 1. 传递效果
        cascade_effect(ser, delay=0.3)
        time.sleep(1)
        
        # 2. 闪烁效果
        blink_effect(ser, blink_count=5, blink_delay=0.25)
        time.sleep(1)
        
        # 3. 呼吸灯效果
        breathing_effect(ser, cycles=2)
        time.sleep(1)
        
        # 4. 波浪效果
        wave_effect(ser, cycles=2)
        time.sleep(1)
        
        # 5. 最终关闭
        turn_off_all_leds(ser)
        
        print("\n🎉 完整灯光秀结束！")
        
    except KeyboardInterrupt:
        print("\n\n⏹️ 用户中断灯光秀")
        turn_off_all_leds(ser)

def simple_test_all_on(ser):
    """简单测试：点亮所有LED为白色"""
    print("\n🔆 测试：点亮所有LED为白色...")
    
    # 白色：B=255, R=255, G=255
    white_colors = [(255, 255, 255)] * 5
    send_led_command(ser, white_colors)
    
    print("  ✓ 所有LED应该为白色")
    print("  如果看到LED亮了，说明通信正常")
    time.sleep(2)

def simple_test_all_off(ser):
    """简单测试：关闭所有LED"""
    print("\n🔌 测试：关闭所有LED...")
    
    # 关闭：B=0, R=0, G=0  
    off_colors = [(0, 0, 0)] * 5
    send_led_command(ser, off_colors)
    
    print("  ✓ 所有LED应该关闭")
    time.sleep(1)

def simple_test_red_only(ser):
    """简单测试：只点亮红色"""
    print("\n🔴 测试：点亮所有LED为红色...")
    
    # 红色：B=0, R=255, G=0
    red_colors = [(0, 255, 0)] * 5
    send_led_command(ser, red_colors)
    
    print("  ✓ 所有LED应该为红色")
    time.sleep(2)

def simple_test_green_only(ser):
    """简单测试：只点亮绿色"""
    print("\n🟢 测试：点亮所有LED为绿色...")
    
    # 绿色：B=0, R=0, G=255
    green_colors = [(0, 0, 255)] * 5
    send_led_command(ser, green_colors)
    
    print("  ✓ 所有LED应该为绿色")
    time.sleep(2)

def simple_test_blue_only(ser):
    """简单测试：只点亮蓝色"""
    print("\n🔵 测试：点亮所有LED为蓝色...")
    
    # 蓝色：B=255, R=0, G=0
    blue_colors = [(255, 0, 0)] * 5
    send_led_command(ser, blue_colors)
    
    print("  ✓ 所有LED应该为蓝色")
    time.sleep(2)

def debug_protocol(ser):
    """调试协议：显示发送的具体字节"""
    print("\n🔍 协议调试模式...")
    print("发送关闭所有LED的命令，显示具体字节：")
    
    # 手动构建协议包（不带校验位）
    frame_header = 0xAA
    data_length = 0x11
    command = 0x03
    data_bytes = [0] * 15  # 15个0
    
    packet = [frame_header, data_length, command] + data_bytes
    
    # 显示字节
    hex_str = " ".join([f"{b:02X}" for b in packet])
    print(f"发送字节: {hex_str}")
    print(f"总长度: {len(packet)}字节")
    print("校验位: 已去除")
    
    # 发送
    ser.write(bytearray(packet))
    print("  ✓ 命令已发送")
    time.sleep(1)

def box_stacking_effect(ser, delay=0.5):
    """
    码箱子效果：从前到后逐个"放置"LED，每个都保持亮着，像叠箱子一样
    每个新的"箱子"都有不同的颜色，表示不同类型的箱子
    """
    print("\n📦 开始码箱子效果...")
    print("-" * 50)
    print("就像在仓库里码箱子一样，每放一个箱子都会留在原地...")
    
    try:
        # 定义每个"箱子"的颜色 - 模拟不同类型的货物
        box_colors = [
            (0, 255, 0),      # 绿色箱子 - 电子产品
            (255, 100, 0),    # 橙色箱子 - 服装
            (0, 0, 255),      # 蓝色箱子 - 食品
            (255, 255, 0),    # 黄色箱子 - 书籍  
            (255, 0, 255)     # 紫色箱子 - 工具
        ]
        
        box_names = ["电子产品", "服装", "食品", "书籍", "工具"]
        
        # 初始状态：所有灯都关闭
        current_colors = [(0, 0, 0)] * 5
        send_led_command(ser, current_colors)
        print("\n📍 仓库货架准备就绪...")
        time.sleep(1.0)
        
        # 逐个码箱子的过程
        for i in range(5):
            print(f"\n📦 正在放置第{i+1}个箱子: {box_names[i]}")
            
            # 先显示箱子要放置的位置（闪烁效果）
            for blink in range(3):
                temp_colors = current_colors.copy()
                temp_colors[i] = box_colors[i]
                send_led_command(ser, temp_colors)
                time.sleep(0.1)
                
                send_led_command(ser, current_colors)
                time.sleep(0.1)
            
            # 正式放置箱子（渐亮效果）
            print(f"   🔽 放置中...")
            for intensity in range(0, 101, 10):
                temp_colors = current_colors.copy()
                color = box_colors[i]
                scaled_color = (
                    int(color[0] * intensity / 100),
                    int(color[1] * intensity / 100),
                    int(color[2] * intensity / 100)
                )
                temp_colors[i] = scaled_color
                send_led_command(ser, temp_colors)
                time.sleep(0.03)
            
            # 箱子放置完成，添加到当前状态
            current_colors[i] = box_colors[i]
            send_led_command(ser, current_colors)
            print(f"   ✅ {box_names[i]}箱子已就位！")
            
            # 显示当前已码好的箱子状态
            stacked_count = i + 1
            print(f"   📊 货架状态: 已码放 {stacked_count}/5 个箱子")
            time.sleep(delay)
        
        time.sleep(0.5)
        
        # 最终检查效果 - 所有箱子闪烁一次表示码放完成
        print(f"\n🎉 所有箱子码放完成！进行最终检查...")
        for check in range(2):
            # 全部熄灭
            send_led_command(ser, [(0, 0, 0)] * 5)
            time.sleep(0.3)
            # 全部点亮
            send_led_command(ser, current_colors)
            time.sleep(0.3)
        
        print(f"\n✅ 码箱子作业完成！共码放5个不同类型的箱子")
        print(f"📋 最终货架配置: {' | '.join(box_names)}")
        
    except KeyboardInterrupt:
        print("\n\n⏹️ 码箱子作业被中断")
    finally:
        # 关闭所有LED
        turn_off_all_leds(ser)

def main():
    """主函数"""
    print("=== AIDog LED流水灯炫酷控制演示 ===")
    print("控制流水灯2实现传递、闪烁等炫酷效果")
    
    # 选择并连接串口
    port = select_port()
    if not port:
        print("未选择串口，程序退出")
        return
    
    ser = test_connection(port)
    if not ser:
        print("串口连接失败，程序退出")
        return
    
    try:
        while True:
            print("\n" + "=" * 60)
            print("🎮 请选择LED控制模式:")
            print("【基础效果】")
            print("  1. 传递效果 (1→5→1)")
            print("  2. 闪烁效果")
            print("  3. 呼吸灯效果")
            print("  4. 波浪效果")
            print("【组合效果】")
            print("  5. 完整灯光秀")
            print("【其他选项】")
            print("  6. 关闭所有LED")
            print("  7. 码箱子效果")
            print("  8. 退出程序")
            
            try:
                choice = input("请选择 (1-8): ").strip()
                
                if choice == '1':
                    cascade_effect(ser)
                elif choice == '2':
                    blink_effect(ser)
                elif choice == '3':
                    breathing_effect(ser)
                elif choice == '4':
                    wave_effect(ser)
                elif choice == '5':
                    complete_light_show(ser)
                elif choice == '6':
                    turn_off_all_leds(ser)
                elif choice == '7':
                    box_stacking_effect(ser)
                elif choice == '8':
                    turn_off_all_leds(ser)
                    break
                else:
                    print("无效选择，请重试")
                    
            except KeyboardInterrupt:
                print("\n检测到 Ctrl+C，返回主菜单...")
                simple_test_all_off(ser)
                continue
                
    except KeyboardInterrupt:
        print("\n\n程序被用户中断")
    finally:
        simple_test_all_off(ser)
        ser.close()
        print("串口已关闭")
        print("程序结束，谢谢使用！")

if __name__ == "__main__":
    main()