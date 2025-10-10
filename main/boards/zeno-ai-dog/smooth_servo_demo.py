#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
丝滑舵机控制演示 - 模仿Arduino效果
控制舵机1实现丝滑的0°↔180°循环
"""

import serial
import serial.serialutil
import time
import glob
import sys
import math

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

def check_serial_connection(ser):
    """
    检查串口连接状态
    
    Args:
        ser: 串口对象
        
    Returns:
        bool: 连接状态
    """
    try:
        return ser.is_open and ser.in_waiting is not None
    except:
        return False

def send_servo_command(ser, front_right=90, rear_right=90, front_left=90, rear_left=90, show_output=True):
    """
    发送舵机控制命令
    
    Args:
        ser: 串口对象
        front_right: 前右舵机角度A (0-180)
        rear_right: 后右舵机角度B (0-180)
        front_left: 前左舵机角度C (0-180)
        rear_left: 后左舵机角度D (0-180)
        show_output: 是否显示输出
        
    Returns:
        bool: 发送成功状态
    """
    # 检查串口连接
    if not check_serial_connection(ser):
        print("\n❌ 串口连接已断开")
        return False
    
    try:
        # 协议参数
        frame_header = 0xAA
        data_length = 0x06
        command = 0x01
        
        # 限制角度范围
        front_right = max(0, min(180, front_right))
        rear_right = max(0, min(180, rear_right))
        front_left = max(0, min(180, front_left))
        rear_left = max(0, min(180, rear_left))
        
        # 所有舵机直接使用原始角度（已恢复角度反转处理）
        front_right_actual = front_right
        rear_right_actual = rear_right
        front_left_actual = front_left
        rear_left_actual = rear_left
        
        # 计算校验位
        checksum = data_length ^ command ^ front_left_actual ^ front_right_actual ^ rear_left_actual ^ rear_right_actual
        
        # 构建数据包
        packet = [frame_header, data_length, command, front_left_actual, front_right_actual, rear_left_actual, rear_right_actual, checksum]
        
        # 发送数据
        ser.write(bytearray(packet))
        ser.flush()  # 确保数据发送完成
        
        # 显示输出
        if show_output:
            # 找出正在变化的舵机
            default_angle = 90
            active_servos = []
            if front_right != default_angle:
                active_servos.append(f"前右A:{front_right:3d}°")
            if rear_right != default_angle:
                active_servos.append(f"后右B:{rear_right:3d}°")
            if front_left != default_angle:
                active_servos.append(f"前左C:{front_left:3d}°")
            if rear_left != default_angle:
                active_servos.append(f"后左D:{rear_left:3d}°")
            
            if len(active_servos) == 1:
                # 单舵机控制时，显示详细进度
                print(f"\r{active_servos[0]} ", end='', flush=True)
            else:
                # 多舵机控制时，显示所有舵机
                print(f"\r前右A:{front_right:3d}° 后右B:{rear_right:3d}° 前左C:{front_left:3d}° 后左D:{rear_left:3d}°", end='', flush=True)
        
        return True
        
    except serial.SerialException as e:
        if show_output:
            print(f"\n❌ 串口发送失败: {e}")
        return False
    except Exception as e:
        if show_output:
            print(f"\n❌ 未知错误: {e}")
        return False

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

def reconnect_serial(current_port, baudrate=115200):
    """
    重连串口
    
    Args:
        current_port: 当前串口路径
        baudrate: 波特率
        
    Returns:
        Serial对象或None
    """
    print("\n🔄 尝试重新连接串口...")
    return test_connection(current_port, baudrate)

def single_servo_loop(ser, servo_id, step=1, delay=0.02, cycles=3):
    """
    单个舵机循环控制
    
    Args:
        ser: 串口对象
        servo_id: 舵机ID (1-4, 对应ABCD)
        step: 角度步进（度）
        delay: 每步延时（秒）
        cycles: 循环次数
    """
    servo_names = {1: "前右A", 2: "后右B", 3: "前左C", 4: "后左D"}
    servo_name = servo_names.get(servo_id, "未知")
    
    print(f"🎯 {servo_name}舵机控制参数:")
    print(f"   角度步进: {step}°")
    print(f"   延时间隔: {delay*1000:.0f}ms")
    print(f"   循环次数: {cycles}次")
    print(f"   预计时间: {(360*delay*cycles/step):.1f}秒")
    print(f"\n开始{servo_name}舵机循环，按 Ctrl+C 停止...")
    print("-" * 50)
    
    cycle_count = 0
    
    try:
        while cycle_count < cycles:
            cycle_count += 1
            print(f"\n第{cycle_count}轮循环:")
            
            # 阶段1：0° → 180°
            print(f"  阶段1: 0° → 180° (步进{step}°)")
            angle_count = 0
            for angle in range(0, 181, step):
                kwargs = {
                    1: {'front_right': angle},
                    2: {'rear_right': angle},
                    3: {'front_left': angle},
                    4: {'rear_left': angle}
                }.get(servo_id, {})
                
                if not send_servo_command(ser, **kwargs):
                    print(f"\n❌ 串口发送失败，停止循环")
                    return
                
                angle_count += 1
                time.sleep(delay)
            print(f" ✓ 完成，共{angle_count}步，最大角度{min(180, (angle_count-1)*step)}°")
            
            # 阶段2：180° → 0°
            print(f"  阶段2: 180° → 0° (步进{step}°)")
            angle_count = 0
            for angle in range(180, -1, -step):
                kwargs = {
                    1: {'front_right': angle},
                    2: {'rear_right': angle},
                    3: {'front_left': angle},
                    4: {'rear_left': angle}
                }.get(servo_id, {})
                
                if not send_servo_command(ser, **kwargs):
                    print(f"\n❌ 串口发送失败，停止循环")
                    return
                
                angle_count += 1
                time.sleep(delay)
            print(f" ✓ 完成，共{angle_count}步，最小角度{max(0, 180-(angle_count-1)*step)}°")
            
            print(f"\n  第{cycle_count}轮完成！")
            
            # 循环间隔
            if cycle_count < cycles:
                time.sleep(0.5)
        
        print(f"\n🎉 所有循环完成！{servo_name}舵机共执行了{cycles}轮循环")
        
    except KeyboardInterrupt:
        print(f"\n\n⏹️  用户中断，已完成{cycle_count-1}轮循环")

def all_servos_sync(ser, step=1, delay=0.02, cycles=3):
    """
    四个舵机同步控制
    """
    print(f"🎯 四舵机同步控制参数:")
    print(f"   角度步进: {step}°")
    print(f"   延时间隔: {delay*1000:.0f}ms")
    print(f"   循环次数: {cycles}次")
    print(f"   预计时间: {(360*delay*cycles/step):.1f}秒")
    print("\n开始四舵机同步循环，按 Ctrl+C 停止...")
    print("-" * 50)
    
    cycle_count = 0
    
    try:
        while cycle_count < cycles:
            cycle_count += 1
            print(f"\n第{cycle_count}轮同步循环:")
            
            # 阶段1：0° → 180°
            print(f"  同步 0° → 180° ")
            for angle in range(0, 181, step):
                if not send_servo_command(ser, angle, angle, angle, angle):
                    print(f"\n❌ 串口发送失败，停止同步控制")
                    return
                time.sleep(delay)
            
            # 阶段2：180° → 0°
            print(f"\n  同步 180° → 0° ")
            for angle in range(180, -1, -step):
                if not send_servo_command(ser, angle, angle, angle, angle):
                    print(f"\n❌ 串口发送失败，停止同步控制")
                    return
                time.sleep(delay)
            
            print(f"\n  第{cycle_count}轮完成！")
            
            if cycle_count < cycles:
                time.sleep(0.5)
        
        print(f"\n🎉 四舵机同步循环完成！共执行了{cycles}轮")
        
    except KeyboardInterrupt:
        print(f"\n\n⏹️  用户中断，已完成{cycle_count-1}轮循环")

def all_servos_wave(ser, step=2, delay=0.03, cycles=2):
    """
    四个舵机波浪式控制
    """
    print(f"🌊 四舵机波浪控制参数:")
    print(f"   角度步进: {step}°")
    print(f"   延时间隔: {delay*1000:.0f}ms")
    print(f"   循环次数: {cycles}次")
    print("\n开始四舵机波浪运动，按 Ctrl+C 停止...")
    print("-" * 50)
    
    cycle_count = 0
    
    try:
        while cycle_count < cycles:
            cycle_count += 1
            print(f"\n第{cycle_count}轮波浪运动:")
            
            # 波浪运动：每个舵机有不同的相位
            for i in range(0, 361, step):
                # 计算每个舵机的角度（添加相位差）
                angle1 = int(90 + 50 * math.sin(math.radians(i)))
                angle2 = int(90 + 50 * math.sin(math.radians(i + 90)))
                angle3 = int(90 + 50 * math.sin(math.radians(i + 180)))
                angle4 = int(90 + 50 * math.sin(math.radians(i + 270)))
                
                if not send_servo_command(ser, angle1, angle2, angle3, angle4):
                    print(f"\n❌ 串口发送失败，停止波浪控制")
                    return
                time.sleep(delay)
            
            print(f"\n  第{cycle_count}轮波浪完成！")
            
            if cycle_count < cycles:
                time.sleep(0.5)
        
        print(f"\n🌊 四舵机波浪运动完成！共执行了{cycles}轮")
        
    except KeyboardInterrupt:
        print(f"\n\n⏹️  用户中断，已完成{cycle_count-1}轮循环")

def all_servos_diagonal(ser, step=2, delay=0.03, cycles=2):
    """
    四个舵机对角控制
    """
    print(f"🔄 四舵机对角控制参数:")
    print(f"   角度步进: {step}°")
    print(f"   延时间隔: {delay*1000:.0f}ms")
    print(f"   循环次数: {cycles}次")
    print("\n开始四舵机对角运动，按 Ctrl+C 停止...")
    print("-" * 50)
    
    cycle_count = 0
    
    try:
        while cycle_count < cycles:
            cycle_count += 1
            print(f"\n第{cycle_count}轮对角运动:")
            
            # 对角运动：前左+后右 vs 前右+后左
            for angle in range(0, 181, step):
                # 前左和后右一组，前右和后左一组
                if not send_servo_command(ser, angle, 180-angle, 180-angle, angle):
                    print(f"\n❌ 串口发送失败，停止对角控制")
                    return
                time.sleep(delay)
            
            for angle in range(180, -1, -step):
                if not send_servo_command(ser, angle, 180-angle, 180-angle, angle):
                    print(f"\n❌ 串口发送失败，停止对角控制")
                    return
                time.sleep(delay)
            
            print(f"\n  第{cycle_count}轮对角完成！")
            
            if cycle_count < cycles:
                time.sleep(0.5)
        
        print(f"\n🔄 四舵机对角运动完成！共执行了{cycles}轮")
        
    except KeyboardInterrupt:
        print(f"\n\n⏹️  用户中断，已完成{cycle_count-1}轮循环")

def stand_pose(ser):
    """
    站立姿态 - 四个舵机全部90度
    """
    print("🧍 执行站立姿态...")
    if send_servo_command(ser, 90, 90, 90, 90, show_output=False):
        print("✅ 已设置为站立姿态 (90°, 90°, 90°, 90°)")
    else:
        print("❌ 站立姿态设置失败")
    time.sleep(1)

def walk_forward(ser, steps=3, step_delay=0.12):
    """
    前进步态 - 基于图示8个动作实现
    
    Args:
        ser: 串口对象
        steps: 步数（循环次数）
        step_delay: 每步延时（秒）
    """
    print(f"🚶 开始前进步态 (共{steps}步)")
    print("步态序列基于图示8个动作实现")
    print("按 Ctrl+C 停止前进...")
    print("-" * 50)
    
    # 前进步态序列
    # 顺序为：前右(A), 后右(B), 前左(C), 后左(D)
    gait_sequence = [
        (130, 90, 90, 50),   # 步骤1
        (130, 50, 130, 50),  # 步骤2
        (90, 50, 130, 90),   # 步骤3
        (90, 90, 90, 90),    # 步骤4 - 站立
        (90, 130, 50, 90),   # 步骤5
        (50, 130, 50, 130),  # 步骤6
        (50, 90, 90, 130),   # 步骤7
        (90, 90, 90, 90)     # 步骤8 - 站立
    ]
    
    try:
        for step_count in range(steps):
            print(f"\n第{step_count + 1}步循环:")
            
            for i, (rf, rb, lf, lb) in enumerate(gait_sequence):
                print(f"  步态{i+1}: 前右A:{rf}° 后右B:{rb}° 前左C:{lf}° 后左D:{lb}°")
                if not send_servo_command(ser, rf, rb, lf, lb, show_output=False):
                    print(f"\n❌ 串口发送失败，停止前进步态")
                    return
                time.sleep(step_delay)
            
            print(f"  第{step_count + 1}步完成")
        
        print(f"\n🎉 前进步态完成！共执行{steps}步")
        print("🧍 回到站立姿态...")
        stand_pose(ser)
        
    except KeyboardInterrupt:
        print(f"\n\n⏹️  用户中断前进步态")
        print("🧍 回到站立姿态...")
        stand_pose(ser)

def walk_backward(ser, steps=3, step_delay=0.2):
    """
    后退步态 - 基于前进步态反向执行
    
    Args:
        ser: 串口对象
        steps: 步数（循环次数）
        step_delay: 每步延时（秒）
    """
    print(f"🚶 开始后退步态 (共{steps}步)")
    print("步态序列基于前进步态反向执行")
    print("按 Ctrl+C 停止后退...")
    print("-" * 50)
    
    # 后退步态序列
    # 顺序为：前右(A), 后右(B), 前左(C), 后左(D)
    gait_sequence = [
        (50, 90, 90, 130),   # 步骤1
        (50, 130, 50, 130),  # 步骤2
        (90, 130, 50, 90),   # 步骤3
        (90, 90, 90, 90),    # 步骤4 - 站立
        (130, 50, 130, 50),  # 步骤5
        (130, 90, 90, 50),   # 步骤6
        (90, 50, 130, 90),   # 步骤7
        (90, 90, 90, 90)     # 步骤8 - 站立
    ]
    
    try:
        for step_count in range(steps):
            print(f"\n第{step_count + 1}步循环:")
            
            for i, (rf, rb, lf, lb) in enumerate(gait_sequence):
                print(f"  步态{i+1}: 前右A:{rf}° 后右B:{rb}° 前左C:{lf}° 后左D:{lb}°")
                if not send_servo_command(ser, rf, rb, lf, lb, show_output=False):
                    print(f"\n❌ 串口发送失败，停止后退步态")
                    return
                time.sleep(step_delay)
            
            print(f"  第{step_count + 1}步完成")
        
        print(f"\n🎉 后退步态完成！共执行{steps}步")
        print("🧍 回到站立姿态...")
        stand_pose(ser)
        
    except KeyboardInterrupt:
        print(f"\n\n⏹️  用户中断后退步态")
        print("🧍 回到站立姿态...")
        stand_pose(ser)

def test_left_right_symmetry(ser):
    """
    测试左右侧舵机对称性
    """
    print("🔧 测试左右侧舵机对称性...")
    print("设置所有舵机为90度（应该呈现对称姿态）")
    
    # 设置所有舵机为90度
    if not send_servo_command(ser, 90, 90, 90, 90, show_output=False):
        print("❌ 无法设置舵机角度，测试终止")
        return
    print("✅ 已设置所有舵机为90度")
    
    input("请观察机器狗姿态，左右两侧应该对称。按Enter继续...")
    
    print("\n测试不同角度的对称性:")
    test_angles = [0, 45, 90, 135, 180]
    
    for angle in test_angles:
        print(f"\n设置所有舵机为{angle}度...")
        if not send_servo_command(ser, angle, angle, angle, angle, show_output=False):
            print(f"❌ 无法设置舵机角度为{angle}度，测试终止")
            return
        print(f"前左C:{angle}° 前右A:{angle}° 后左D:{angle}° 后右B:{angle}°")
        time.sleep(2)
    
    print("\n🎉 对称性测试完成！")
    time.sleep(1)

def turn_right(ser, steps=3, step_delay=0.12):
    """
    右转步态
    
    Args:
        ser: 串口对象
        steps: 步数（循环次数）
        step_delay: 每步延时（秒）
    """
    print(f"🔄 开始右转步态 (共{steps}步)")
    print("按 Ctrl+C 停止右转...")
    print("-" * 50)
    
    # 右转步态序列
    # 顺序为：前右(A), 后右(B), 前左(C), 后左(D)
    gait_sequence = [
        (50, 90, 90, 50),    # 步骤1：右前抬起
        (50, 130, 130, 50),   # 步骤2：右前右后着地
        (90, 130, 130, 90),    # 步骤3：右后放下
        (90, 90, 90, 90),     # 步骤4：站立
    ]
    
    try:
        for step_count in range(steps):
            print(f"\n第{step_count + 1}步循环:")
            
            for i, (rf, rb, lf, lb) in enumerate(gait_sequence):
                print(f"  步态{i+1}: 前右A:{rf}° 后右B:{rb}° 前左C:{lf}° 后左D:{lb}°")
                if not send_servo_command(ser, rf, rb, lf, lb, show_output=False):
                    print(f"\n❌ 串口发送失败，停止右转步态")
                    return
                time.sleep(step_delay)
            
            print(f"  第{step_count + 1}步完成")
        
        print(f"\n🎉 右转步态完成！共执行{steps}步")
        print("🧍 回到站立姿态...")
        stand_pose(ser)
        
    except KeyboardInterrupt:
        print(f"\n\n⏹️  用户中断右转步态")
        print("🧍 回到站立姿态...")
        stand_pose(ser)

def turn_left(ser, steps=3, step_delay=0.12):
    """
    左转步态
    
    Args:
        ser: 串口对象
        steps: 步数（循环次数）
        step_delay: 每步延时（秒）
    """
    print(f"🔄 开始左转步态 (共{steps}步)")
    print("按 Ctrl+C 停止左转...")
    print("-" * 50)
    
    # 左转步态序列
    # 顺序为：前右(A), 后右(B), 前左(C), 后左(D)
    gait_sequence = [
        (90, 130, 130, 90),    # 步骤1：左前抬起
        (50, 130, 130, 50),   # 步骤2：左前左后着地
        (50, 90, 90, 50),    # 步骤3：左后放下
        (90, 90, 90, 90),     # 步骤4：站立
    ]
    
    try:
        for step_count in range(steps):
            print(f"\n第{step_count + 1}步循环:")
            
            for i, (rf, rb, lf, lb) in enumerate(gait_sequence):
                print(f"  步态{i+1}: 前右A:{rf}° 后右B:{rb}° 前左C:{lf}° 后左D:{lb}°")
                if not send_servo_command(ser, rf, rb, lf, lb, show_output=False):
                    print(f"\n❌ 串口发送失败，停止左转步态")
                    return
                time.sleep(step_delay)
            
            print(f"  第{step_count + 1}步完成")
        
        print(f"\n🎉 左转步态完成！共执行{steps}步")
        print("🧍 回到站立姿态...")
        stand_pose(ser)
        
    except KeyboardInterrupt:
        print(f"\n\n⏹️  用户中断左转步态")
        print("🧍 回到站立姿态...")
        stand_pose(ser)

def wave_hand(ser, cycles=3, step_delay=0.19):
    """
    招手动作 - 右前腿摆动
    
    Args:
        ser: 串口对象
        cycles: 循环次数
        step_delay: 每步延时（秒）
    """
    print(f"👋 开始招手动作 (共{cycles}次)")
    print("按 Ctrl+C 停止招手...")
    print("-" * 50)
    
    # 招手动作序列
    # 顺序为：前右(A), 后右(B), 前左(C), 后左(D)
    wave_sequence = [
        (110, 130, 90, 50),  # 步骤1：抬起一点
        (145, 130, 90, 50),  # 步骤2：抬起更高
        (180, 130, 90, 50),  # 步骤3：最高点
        (145, 130, 90, 50),  # 步骤4：放下一点
    ]
    
    try:
        for cycle in range(cycles):
            print(f"\n第{cycle + 1}次招手:")
            
            for i, (rf, rb, lf, lb) in enumerate(wave_sequence):
                print(f"  动作{i+1}: 前右A:{rf}° 后右B:{rb}° 前左C:{lf}° 后左D:{lb}°")
                if not send_servo_command(ser, rf, rb, lf, lb, show_output=False):
                    print(f"\n❌ 串口发送失败，停止招手")
                    return
                time.sleep(step_delay)
            
            print(f"  第{cycle + 1}次完成")
        
        print(f"\n🎉 招手动作完成！共执行{cycles}次")
        print("🧍 回到站立姿态...")
        stand_pose(ser)
        
    except KeyboardInterrupt:
        print(f"\n\n⏹️  用户中断招手动作")
        print("🧍 回到站立姿态...")
        stand_pose(ser)

def swing_dance(ser, cycles=3, step_delay=0.19):
    """
    摇摆动作 - 四条腿同时向外展开再收回
    
    Args:
        ser: 串口对象
        cycles: 循环次数
        step_delay: 每步延时（秒）
    """
    print(f"💃 开始摇摆动作 (共{cycles}次)")
    print("按 Ctrl+C 停止摇摆...")
    print("-" * 50)
    
    # 摇摆动作序列
    # 顺序为：前右(A), 后右(B), 前左(C), 后左(D)
    swing_sequence = [
        (150, 150, 30, 30),   # 步骤1：四腿向外展开
        (90, 90, 90, 90),     # 步骤2：回到中间
        (30, 30, 150, 150),   # 步骤3：四腿向内收拢
        (90, 90, 90, 90),     # 步骤4：回到中间
    ]
    
    try:
        for cycle in range(cycles):
            print(f"\n第{cycle + 1}次摇摆:")
            
            for i, (rf, rb, lf, lb) in enumerate(swing_sequence):
                print(f"  动作{i+1}: 前右A:{rf}° 后右B:{rb}° 前左C:{lf}° 后左D:{lb}°")
                if not send_servo_command(ser, rf, rb, lf, lb, show_output=False):
                    print(f"\n❌ 串口发送失败，停止摇摆")
                    return
                time.sleep(step_delay)
            
            print(f"  第{cycle + 1}次完成")
        
        print(f"\n🎉 摇摆动作完成！共执行{cycles}次")
        print("🧍 回到站立姿态...")
        stand_pose(ser)
        
    except KeyboardInterrupt:
        print(f"\n\n⏹️  用户中断摇摆动作")
        print("🧍 回到站立姿态...")
        stand_pose(ser)

def debug_servos(ser):
    """
    调试模式 - 直接设置4个舵机的角度
    
    Args:
        ser: 串口对象
    """
    print("\n🔧 舵机角度调试模式")
    print("说明：输入4个舵机的目标角度，范围0-180")
    print("顺序：前右A, 后右B, 前左C, 后左D")
    print("示例：90 90 90 90")
    print("输入 'q' 退出调试模式")
    print("-" * 50)
    
    while True:
        try:
            user_input = input("\n请输入4个角度（空格分隔）: ").strip().lower()
            
            if user_input == 'q':
                print("退出调试模式")
                break
            
            # 解析输入的角度
            try:
                angles = [int(x) for x in user_input.split()]
                if len(angles) != 4:
                    print("❌ 请输入4个角度值")
                    continue
                
                # 检查角度范围
                if not all(0 <= angle <= 180 for angle in angles):
                    print("❌ 角度必须在0-180之间")
                    continue
                
                # 设置舵机角度
                rf, rb, lf, lb = angles
                print(f"设置角度: 前右A:{rf}° 后右B:{rb}° 前左C:{lf}° 后左D:{lb}°")
                if not send_servo_command(ser, rf, rb, lf, lb, show_output=False):
                    print("❌ 串口发送失败")
                else:
                    print("✅ 角度设置成功")
                
            except ValueError:
                print("❌ 输入格式错误，请输入数字")
                continue
                
        except KeyboardInterrupt:
            print("\n用户中断，退出调试模式")
            break

def main():
    print("=== AIDog 全功能舵机控制演示 ===")
    print("支持单舵机控制和四舵机组合控制（使用原始角度，已恢复角度反转处理）")
    print("✨ 新增功能：自动检测串口断开 + 智能重连机制")
    print("💡 提示：如果遇到串口断开，程序会自动尝试重连，或选择 'R' 手动重连")
    
    # 选择串口
    port = select_port()
    if not port:
        print("未选择串口，程序退出")
        return
    
    # 测试连接
    ser = test_connection(port)
    if not ser:
        print("无法连接串口，程序退出")
        return
    
    current_port = port  # 记录当前串口路径
    
    try:
        while True:
            # 检查串口连接状态
            if not check_serial_connection(ser):
                print("\n⚠️  检测到串口连接断开")
                ser = reconnect_serial(current_port)
                if not ser:
                    print("❌ 无法重新连接串口，程序退出")
                    break
                else:
                    print("✅ 串口重连成功")
            
            print("\n" + "="*50)
            print("🎮 请选择控制模式:")
            print("【单舵机控制】")
            print("1. 前右舵机(A)")
            print("2. 后右舵机(B)")
            print("3. 前左舵机(C)")
            print("4. 后左舵机(D)")
            print("\n【步态控制】")
            print("8. 站立姿态")
            print("9. 前进步态")
            print("10. 后退步态")
            print("11. 右转步态")
            print("12. 左转步态")
            print("13. 招手动作")
            print("14. 摇摆动作")
            print("\n【调试功能】")
            print("99. 调试模式")
            print("0. 退出程序")
            
            choice = input("\n请选择 (0-4, 8-14, 99): ").strip()
            
            if choice in ['1', '2', '3', '4']:
                # 单舵机控制
                servo_id = int(choice)
                servo_names = {1: "前右A", 2: "后右B", 3: "前左C", 4: "后左D"}
                
                print(f"\n选择{servo_names[servo_id]}舵机控制模式:")
                print("1. 超丝滑模式 (1°步进, 20ms延时)")
                print("2. 丝滑模式   (2°步进, 30ms延时)")
                print("3. 快速模式   (5°步进, 50ms延时)")
                print("4. 自定义模式")
                
                mode = input("请选择模式 (1-4): ").strip()
                
                if mode == '1':
                    step, delay, cycles = 1, 0.02, 2
                elif mode == '2':
                    step, delay, cycles = 2, 0.03, 2
                elif mode == '3':
                    step, delay, cycles = 5, 0.05, 2
                elif mode == '4':
                    step = int(input("角度步进 (度): "))
                    delay = float(input("延时间隔 (秒): "))
                    cycles = int(input("循环次数: "))
                else:
                    step, delay, cycles = 2, 0.03, 2
                
                single_servo_loop(ser, servo_id, step, delay, cycles)
                
            elif choice == '8':
                # 站立姿态
                stand_pose(ser)
                
            elif choice == '9':
                # 前进步态
                print("\n前进步态参数:")
                print("1. 标准步态 (3步)")
                print("2. 长距离步态 (5步)")
                print("3. 自定义步数")
                
                gait_mode = input("请选择 (1-3): ").strip()
                
                if gait_mode == '1':
                    steps = 3
                elif gait_mode == '2':
                    steps = 5
                elif gait_mode == '3':
                    try:
                        steps = int(input("步数: "))
                        if steps < 1:
                            print("❌ 步数无效，使用默认值")
                            steps = 3
                    except ValueError:
                        print("❌ 输入无效，使用默认值")
                        steps = 3
                else:
                    steps = 3
                
                walk_forward(ser, steps)
                
            elif choice == '10':
                # 后退步态
                print("\n后退步态参数:")
                print("1. 标准步态 (3步)")
                print("2. 长距离步态 (5步)")
                print("3. 自定义步数")
                
                gait_mode = input("请选择 (1-3): ").strip()
                
                if gait_mode == '1':
                    steps = 3
                elif gait_mode == '2':
                    steps = 5
                elif gait_mode == '3':
                    try:
                        steps = int(input("步数: "))
                        if steps < 1:
                            print("❌ 步数无效，使用默认值")
                            steps = 3
                    except ValueError:
                        print("❌ 输入无效，使用默认值")
                        steps = 3
                else:
                    steps = 3
                
                walk_backward(ser, steps)
                
            elif choice == '11':
                # 右转步态
                print("\n右转步态参数:")
                print("1. 标准步态 (3步)")
                print("2. 大转弯 (5步)")
                print("3. 自定义步数")
                
                gait_mode = input("请选择 (1-3): ").strip()
                
                if gait_mode == '1':
                    steps = 3
                elif gait_mode == '2':
                    steps = 5
                elif gait_mode == '3':
                    try:
                        steps = int(input("步数: "))
                        if steps < 1:
                            print("❌ 步数无效，使用默认值")
                            steps = 3
                    except ValueError:
                        print("❌ 输入无效，使用默认值")
                        steps = 3
                else:
                    steps = 3
                
                turn_right(ser, steps)
                
            elif choice == '12':
                # 左转步态
                print("\n左转步态参数:")
                print("1. 标准步态 (3步)")
                print("2. 大转弯 (5步)")
                print("3. 自定义步数")
                
                gait_mode = input("请选择 (1-3): ").strip()
                
                if gait_mode == '1':
                    steps = 3
                elif gait_mode == '2':
                    steps = 5
                elif gait_mode == '3':
                    try:
                        steps = int(input("步数: "))
                        if steps < 1:
                            print("❌ 步数无效，使用默认值")
                            steps = 3
                    except ValueError:
                        print("❌ 输入无效，使用默认值")
                        steps = 3
                else:
                    steps = 3
                
                turn_left(ser, steps)
                
            elif choice == '13':
                # 招手动作
                print("\n招手动作参数:")
                print("1. 标准招手 (3次)")
                print("2. 热情招手 (5次)")
                print("3. 自定义次数")
                
                wave_mode = input("请选择 (1-3): ").strip()
                
                if wave_mode == '1':
                    cycles = 3
                elif wave_mode == '2':
                    cycles = 5
                elif wave_mode == '3':
                    try:
                        cycles = int(input("招手次数: "))
                        if cycles < 1:
                            print("❌ 次数无效，使用默认值")
                            cycles = 3
                    except ValueError:
                        print("❌ 输入无效，使用默认值")
                        cycles = 3
                else:
                    cycles = 3
                
                wave_hand(ser, cycles)
                
            elif choice == '14':
                # 摇摆动作
                print("\n摇摆动作参数:")
                print("1. 标准摇摆 (3次)")
                print("2. 热情摇摆 (5次)")
                print("3. 自定义次数")
                
                swing_mode = input("请选择 (1-3): ").strip()
                
                if swing_mode == '1':
                    cycles = 3
                elif swing_mode == '2':
                    cycles = 5
                elif swing_mode == '3':
                    try:
                        cycles = int(input("摇摆次数: "))
                        if cycles < 1:
                            print("❌ 次数无效，使用默认值")
                            cycles = 3
                    except ValueError:
                        print("❌ 输入无效，使用默认值")
                        cycles = 3
                else:
                    cycles = 3
                
                swing_dance(ser, cycles)
                
            elif choice == '99':
                # 调试模式
                debug_servos(ser)
            elif choice == '0':
                break
            else:
                print("❌ 无效选择，请重新输入")
        
    except (ValueError, KeyboardInterrupt):
        print("\n输入无效或用户取消")
    finally:
        if ser:
            ser.close()
            print("串口已关闭")
        print("程序结束，谢谢使用！")

if __name__ == "__main__":
    main() 