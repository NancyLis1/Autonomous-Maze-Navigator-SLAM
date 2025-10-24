"""
===============================================================================
Robot Serial Interface (Text Protocol) - 文本协议版本
用于 goalseeking.py 与 STM32 硬件通信(文本格式)
===============================================================================

适配 STM32 发送的文本格式数据:
- IMU数据: "angles":[yaw,roll,pitch],"accel":[x,y,z],"gyro":[x,y,z]\r\n
- 激光雷达: Lidar: A=<angle>, D=<distance>mm, Q=<quality>\n
- PID调试: PID: Tgt=..., RPM_A=..., RPM_B=..., PWM_L=..., PWM_R=...\n

使用方法:
```python
import robot_serial_interface_text as robot_serial

# 初始化
robot_serial.init(port='COM5', baudrate=115200)

# 获取数据
pose = robot_serial.get_robot_pose()  # 返回 Pose(x, y, theta)
scan = robot_serial.get_lidar_scan()  # 返回 Scan(angles, ranges)

# 关闭
robot_serial.close()
```
===============================================================================
"""

import serial
import time
import threading
import re
import math
from collections import deque
from dataclasses import dataclass
from typing import Optional, List
import numpy as np

# ==============================================================================
# 数据结构
# ==============================================================================
@dataclass
class Pose:
    """机器人位姿"""
    x: float      # 位置 x (米)
    y: float      # 位置 y (米)
    theta: float  # 航向角 (弧度)
    vx: float = 0.0     # 线速度 (m/s)
    omega: float = 0.0  # 角速度 (rad/s)

@dataclass
class Scan:
    """激光雷达扫描数据"""
    angles: np.ndarray   # 角度数组 (弧度)
    ranges: np.ndarray   # 距离数组 (米)

@dataclass
class PoseSetpoint:
    """目标位姿"""
    x: float
    y: float
    theta: float
    v: float = 0.3      # 期望线速度
    omega: float = 0.0  # 期望角速度

# ==============================================================================
# 全局变量
# ==============================================================================
_serial_port: Optional[serial.Serial] = None
_receiver_thread: Optional[threading.Thread] = None
_running = False

# 数据缓存
_latest_imu_data = {
    'yaw': 0.0,     # 偏航角(度)
    'roll': 0.0,    # 横滚角(度)
    'pitch': 0.0,   # 俯仰角(度)
    'accel': [0, 0, 0],
    'gyro': [0, 0, 0],
    'timestamp': 0.0
}

_lidar_buffer = deque(maxlen=500)  # 存储最近的激光点 (angle_deg, distance_mm, quality)

# 里程计数据(从编码器估算)
_odometry = {
    'x': 0.0,       # 累积x位移(米)
    'y': 0.0,       # 累积y位移(米)
    'theta': 0.0,   # 累积航向角(弧度)
    'vx': 0.0,      # 线速度
    'omega': 0.0    # 角速度
}

_data_lock = threading.Lock()

# ==============================================================================
# 初始化与关闭
# ==============================================================================
def init(port: str = 'COM5', baudrate: int = 115200, timeout: float = 0.1) -> None:
    """
    初始化串口并启动接收线程

    Args:
        port: 串口名称 (Windows: 'COM3', Linux: '/dev/rfcomm0')
        baudrate: 波特率 (默认115200,匹配STM32固件)
        timeout: 读超时 (秒)
    """
    global _serial_port, _receiver_thread, _running

    if _serial_port is not None:
        print("[RobotSerial] 警告: 串口已打开,先关闭旧连接")
        close()

    try:
        _serial_port = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=timeout
        )
        print(f"[RobotSerial] 成功打开串口: {port} @ {baudrate} bps")
        print(f"!!!VERSION_2025_10_21_MODIFIED!!!", flush=True)

        # 【调试】不清空缓冲区,保留已有数据
        # _serial_port.reset_input_buffer()
        # _serial_port.reset_output_buffer()
        print(f"[DEBUG] Buffer in_waiting={_serial_port.in_waiting}", flush=True)

        # 启动接收线程
        _running = True
        _receiver_thread = threading.Thread(target=_receive_loop, daemon=True)
        _receiver_thread.start()
        print("[RobotSerial] 接收线程已启动")

        # 等待线程真正开始
        import time
        time.sleep(0.1)
        print(f"[DEBUG] 线程活动状态: {_receiver_thread.is_alive()}", flush=True)

    except serial.SerialException as e:
        print(f"[RobotSerial] 打开串口失败: {e}")
        raise


def close() -> None:
    """关闭串口并停止接收线程"""
    global _serial_port, _running, _receiver_thread

    _running = False

    if _receiver_thread is not None:
        _receiver_thread.join(timeout=2.0)
        _receiver_thread = None

    if _serial_port is not None:
        _serial_port.close()
        _serial_port = None
        print("[RobotSerial] 串口已关闭")


# ==============================================================================
# 数据接收线程
# ==============================================================================
def _receive_loop() -> None:
    """后台线程: 持续接收和解析串口数据(支持文本+二进制混合)"""
    # ===【最基本的生存标记】===
    try:
        with open("d:\\硬件\\thread_alive.txt", "w") as f:
            f.write("THREAD ENTERED\n")
    except:
        pass

    global _serial_port, _running

    import sys
    sys.stdout.flush()
    sys.stderr.flush()

    try:
        print("="*50, flush=True)
        print("[DEBUG] === 接收循环已启动 ===", flush=True)
        print("="*50, flush=True)
    except Exception as e:
        import traceback
        traceback.print_exc()
        return

    line_buffer = ""
    unified_buffer = bytearray()  # 统一缓冲区
    sync_loss_count = 0  # 【修复】同步丢失计数器

    while _running and _serial_port is not None:
        try:
            # 读取可用数据到统一缓冲区
            if _serial_port.in_waiting > 0:
                # print(f"[DEBUG] 检测到 {_serial_port.in_waiting} 字节待读取", flush=True)
                data = _serial_port.read(_serial_port.in_waiting)
                unified_buffer.extend(data)

                # === 第1步: 先提取所有二进制包 ===
                i = 0
                packets_found = 0
                while i < len(unified_buffer):
                    # 查找二进制包头 0xAA 0x55
                    if (i + 1 < len(unified_buffer) and
                        unified_buffer[i] == 0xAA and
                        unified_buffer[i+1] == 0x55):

                        # print(f"[DEBUG] 在位置{i}找到包头AA 55, 缓冲区长度={len(unified_buffer)}")

                        # 检查是否有完整包头(至少5字节)
                        if i + 5 <= len(unified_buffer):
                            msg_type = unified_buffer[i+2]
                            length = unified_buffer[i+3] | (unified_buffer[i+4] << 8)
                            packet_len = 7 + length  # header(2)+type(1)+len(2)+payload+crc(2)

                            # print(f"[DEBUG] type=0x{msg_type:02X}, length={length}, packet_len={packet_len}")

                            # 【修复】检查包长度合理性（防止错误的包头导致缓冲区溢出）
                            if packet_len > 256:  # 最大合理包长度
                                # 可能是错误的包头，跳过
                                i += 1
                                continue

                            # 检查完整包
                            if i + packet_len <= len(unified_buffer):
                                # 提取并解析包
                                packet = bytes(unified_buffer[i:i+packet_len])
                                packets_found += 1

                                if msg_type == 0x11 and length == 24:  # MSG_TYPE_ODOMETRY
                                    # print(f"[DEBUG] ✓ 里程计包#{packets_found}: len={packet_len}")
                                    _parse_odometry_packet(packet)
                                    sync_loss_count = 0  # 重置同步丢失计数
                                elif msg_type == 0x12 and length == 28:  # MSG_TYPE_IMU_DATA (28 bytes)
                                    # print(f"[DEBUG] ✓ IMU包#{packets_found}: len={packet_len}")
                                    _parse_imu_packet(packet)
                                    sync_loss_count = 0  # 重置同步丢失计数
                                else:
                                    pass
                                    # print(f"[DEBUG] 跳过包: type=0x{msg_type:02X}, len={length}")

                                # 移除已处理的包
                                unified_buffer = unified_buffer[:i] + unified_buffer[i+packet_len:]
                                continue  # 重新从当前位置开始
                            else:
                                pass
                                # print(f"[DEBUG] 包不完整,等待更多数据 (需要{packet_len}, 有{len(unified_buffer)-i})")

                        # 包不完整,等待更多数据
                        break

                    i += 1

                # 【修复】如果缓冲区过大且没有找到有效包头，可能是同步丢失
                if len(unified_buffer) > 512 and packets_found == 0:
                    sync_loss_count += 1
                    if sync_loss_count > 3:
                        print(f"[WARNING] 通信同步丢失！缓冲区大小={len(unified_buffer)}, 清空缓冲区", flush=True)
                        unified_buffer.clear()
                        sync_loss_count = 0

                # === 第2步: 处理剩余数据为文本 ===
                if len(unified_buffer) > 0:
                    # 查找是否有完整文本行(以\n结尾)
                    try:
                        # 只处理不包含二进制包头的部分
                        text_end = len(unified_buffer)
                        for j in range(len(unified_buffer) - 1):
                            if unified_buffer[j] == 0xAA and unified_buffer[j+1] == 0x55:
                                text_end = j
                                break

                        if text_end > 0:
                            text_data = bytes(unified_buffer[:text_end])
                            text = text_data.decode('utf-8', errors='ignore')
                            line_buffer += text
                            unified_buffer = unified_buffer[text_end:]

                            # 处理完整行
                            while '\n' in line_buffer:
                                line, line_buffer = line_buffer.split('\n', 1)
                                line = line.strip()
                                if line:
                                    _parse_line(line)
                    except:
                        pass

                # 防止缓冲区无限增长
                if len(unified_buffer) > 2048:
                    # print("[DEBUG] 缓冲区溢出,清空前1024字节")
                    unified_buffer = unified_buffer[1024:]

            else:
                time.sleep(0.01)  # 避免空转

        except Exception as e:
            if _running:
                print(f"[RobotSerial] 接收错误: {e}")
            time.sleep(0.1)


def _parse_odometry_packet(packet: bytes) -> None:
    """解析里程计二进制数据包"""
    import struct

    try:
        # 包格式: AA 55 11 [len_lo len_hi] [payload 24字节] [crc_lo crc_hi]
        # Payload: uint32 timestamp, float x, y, theta, v_linear, v_angular
        if len(packet) < 31:  # 7 + 24
            return

        payload = packet[5:29]  # 提取payload (24字节)
        timestamp, x, y, theta, v_linear, v_angular = struct.unpack('<I5f', payload)

        # 更新里程计数据
        with _data_lock:
            _odometry['x'] = x
            _odometry['y'] = y
            _odometry['theta'] = theta
            _odometry['vx'] = v_linear
            _odometry['omega'] = v_angular
            _latest_imu_data['timestamp'] = timestamp  # 标记有数据

        # print(f"[DEBUG] 里程计: t={timestamp}, x={x:.3f}, y={y:.3f}, θ={theta:.3f}")

    except Exception as e:
        print(f"[RobotSerial] 里程计包解析错误: {e}")


def _parse_imu_packet(packet: bytes) -> None:
    """解析IMU二进制数据包"""
    import struct

    try:
        # 包格式: AA 55 12 [len_lo len_hi] [payload 28字节] [crc_lo crc_hi]
        # Payload: uint32 timestamp, float yaw, roll, pitch, int16_t accel[3], int16_t gyro[3]
        if len(packet) < 35:  # 7 + 28
            return

        payload = packet[5:33]  # 提取payload (28字节)
        # 解包: 1个uint32, 3个float, 6个int16
        timestamp, yaw, roll, pitch, ax, ay, az, gx, gy, gz = struct.unpack('<I3f6h', payload)

        # 更新IMU数据
        with _data_lock:
            _latest_imu_data['yaw'] = yaw
            _latest_imu_data['roll'] = roll
            _latest_imu_data['pitch'] = pitch
            _latest_imu_data['accel'] = [ax, ay, az]
            _latest_imu_data['gyro'] = [gx, gy, gz]
            _latest_imu_data['timestamp'] = time.time()  # 使用当前时间

        # print(f"[DEBUG] IMU: t={timestamp}, yaw={yaw:.2f}°, accel=[{ax},{ay},{az}]")

    except Exception as e:
        print(f"[RobotSerial] IMU包解析错误: {e}")


def _parse_line(line: str) -> None:
    """
    解析一行文本数据

    支持格式:
    1. IMU数据: "angles":[yaw,roll,pitch],"accel":[x,y,z],"gyro":[x,y,z]
    2. 激光雷达: Lidar: A=123.45, D=1234.56mm, Q=47
    3. PID调试: PID: Tgt=60.0, RPM_A=58.3, RPM_B=59.1, PWM_L=320, PWM_R=330
    """
    global _latest_imu_data, _lidar_buffer, _odometry, _data_lock

    # 解析IMU数据
    if '"angles":' in line:
        try:
            # 使用正则提取数值
            angles_match = re.search(r'"angles":\[([-\d.]+),([-\d.]+),([-\d.]+)\]', line)
            accel_match = re.search(r'"accel":\[([-\d]+),([-\d]+),([-\d]+)\]', line)
            gyro_match = re.search(r'"gyro":\[([-\d]+),([-\d]+),([-\d]+)\]', line)

            if angles_match:
                with _data_lock:
                    _latest_imu_data['yaw'] = float(angles_match.group(1))
                    _latest_imu_data['roll'] = float(angles_match.group(2))
                    _latest_imu_data['pitch'] = float(angles_match.group(3))
                    _latest_imu_data['timestamp'] = time.time()

                    # 更新里程计的航向角(转为弧度)
                    _odometry['theta'] = np.deg2rad(_latest_imu_data['yaw'])

            if accel_match:
                _latest_imu_data['accel'] = [int(accel_match.group(1)),
                                              int(accel_match.group(2)),
                                              int(accel_match.group(3))]

            if gyro_match:
                _latest_imu_data['gyro'] = [int(gyro_match.group(1)),
                                             int(gyro_match.group(2)),
                                             int(gyro_match.group(3))]
        except Exception as e:
            print(f"[RobotSerial] IMU数据解析失败: {line} | {e}")

    # 解析激光雷达数据
    elif line.startswith('Lidar:'):
        try:
            # Lidar: A=123.45, D=1234.56mm, Q=47
            match = re.search(r'A=([-\d.]+),\s*D=([-\d.]+)mm,\s*Q=(\d+)', line)
            if match:
                angle_deg = float(match.group(1))
                distance_mm = float(match.group(2))
                quality = int(match.group(3))

                with _data_lock:
                    _lidar_buffer.append((angle_deg, distance_mm, quality))
        except Exception as e:
            print(f"[RobotSerial] 激光雷达数据解析失败: {line} | {e}")

    # 解析PID调试信息(可选,用于监控)
    elif line.startswith('PID:'):
        # 仅打印,不做处理
        print(f"[RobotSerial-DEBUG] {line}")


# ==============================================================================
# 数据获取接口
# ==============================================================================
def get_robot_pose(timeout: float = 1.0) -> Pose:
    """
    获取机器人当前位姿

    从STM32接收的里程计数据包中获取位姿信息
    数据来源: MSG_TYPE_ODOMETRY (0x11) 二进制包

    Args:
        timeout: 超时时间(秒)

    Returns:
        Pose对象

    Raises:
        TimeoutError: 超时未收到数据
    """
    start_time = time.time()

    while time.time() - start_time < timeout:
        with _data_lock:
            if _latest_imu_data['timestamp'] > 0:
                # 归一化角度到[-π, π]
                import math
                theta_normalized = math.atan2(math.sin(_odometry['theta']), math.cos(_odometry['theta']))

                pose = Pose(
                    x=_odometry['x'],
                    y=_odometry['y'],
                    theta=theta_normalized,  # 归一化后的角度
                    vx=_odometry['vx'],
                    omega=_odometry['omega']
                )

                # 【调试日志】每10次打印一次，避免刷屏
                if not hasattr(get_robot_pose, '_call_count'):
                    get_robot_pose._call_count = 0
                get_robot_pose._call_count += 1
                if get_robot_pose._call_count % 10 == 0:
                    print(f"[POSE] x={pose.x:.3f}m y={pose.y:.3f}m θ={math.degrees(pose.theta):.1f}° "
                          f"v={pose.vx:.2f}m/s ω={pose.omega:.2f}rad/s")

                return pose
        time.sleep(0.01)

    # DEBUG: 输出当前数据状态
    with _data_lock:
        print(f"[ERROR-TIMEOUT] 未收到位姿数据！", flush=True)
        print(f"  IMU timestamp: {_latest_imu_data['timestamp']}", flush=True)
        print(f"  Odometry: x={_odometry['x']:.3f}, y={_odometry['y']:.3f}, θ={_odometry['theta']:.3f}", flush=True)
        print(f"  提示: 检查STM32是否正常发送里程计数据包(MSG_TYPE_ODOMETRY=0x11)", flush=True)
    raise TimeoutError(f"未在 {timeout}s 内接收到位姿数据")


def get_lidar_scan(timeout: float = 2.0, min_points: int = 20) -> Scan:
    """
    获取激光雷达扫描数据

    Args:
        timeout: 超时时间(秒)
        min_points: 最少点数要求

    Returns:
        Scan对象,包含angles(弧度)和ranges(米)

    Raises:
        TimeoutError: 超时未收到足够数据
    """
    start_time = time.time()

    while time.time() - start_time < timeout:
        with _data_lock:
            if len(_lidar_buffer) >= min_points:
                # 复制数据
                points = list(_lidar_buffer)

                # 【修复】清空缓冲区，避免重复使用旧数据
                _lidar_buffer.clear()

                # 转换格式
                angles_deg = np.array([p[0] for p in points])
                distances_mm = np.array([p[1] for p in points])

                # 转换单位
                angles_rad = np.deg2rad(angles_deg)
                ranges_m = distances_mm / 1000.0

                # 【调试日志】每20次打印一次
                if not hasattr(get_lidar_scan, '_call_count'):
                    get_lidar_scan._call_count = 0
                get_lidar_scan._call_count += 1
                if get_lidar_scan._call_count % 20 == 0:
                    print(f"[LIDAR] 获取扫描: {len(points)}个点, "
                          f"角度范围[{angles_deg.min():.1f}°, {angles_deg.max():.1f}°], "
                          f"距离范围[{ranges_m.min():.2f}m, {ranges_m.max():.2f}m]")

                return Scan(angles=angles_rad, ranges=ranges_m)

        time.sleep(0.05)

    # 超时错误提示
    with _data_lock:
        current_points = len(_lidar_buffer)
    print(f"[ERROR-TIMEOUT] 激光雷达数据不足！", flush=True)
    print(f"  需要: {min_points}个点, 当前: {current_points}个点", flush=True)
    print(f"  提示: 检查激光雷达是否正常工作，串口是否接收到Lidar数据", flush=True)
    raise TimeoutError(f"未在 {timeout}s 内接收到足够的激光雷达数据(当前点数: {current_points})")


def send_setpoint(sp: PoseSetpoint, retry: int = 3) -> bool:
    """
    发送目标位姿到STM32 (使用二进制协议)

    数据包格式: [0xAA 0x55] [0x01] [len_lo len_hi] [22字节PoseSetpoint] [CRC16]
    PoseSetpoint: uint16 seq + float x,y,theta,v,omega

    Args:
        sp: 目标位姿
        retry: 失败重试次数

    Returns:
        bool: 发送成功返回True，失败返回False
    """
    global _serial_port
    import struct

    if _serial_port is None:
        print("[RobotSerial] 错误: 串口未打开")
        return False

    for attempt in range(retry):
        try:
            # 构建PoseSetpoint载荷 (22字节)
            # 格式: <H (uint16 seq) + 5f (5个float: x,y,theta,v,omega)
            seq = getattr(send_setpoint, '_seq_counter', 0)
            send_setpoint._seq_counter = (seq + 1) % 65536

            payload = struct.pack('<H5f',
                seq,           # 序列号
                sp.x,          # 目标X (米)
                sp.y,          # 目标Y (米)
                sp.theta,      # 目标航向 (弧度)
                sp.v,          # 目标线速度 (m/s)
                sp.omega       # 目标角速度 (rad/s)
            )

            # 构建完整数据包
            header = bytes([0xAA, 0x55])
            msg_type = 0x01  # MSG_TYPE_POSE_SETPOINT
            length = len(payload)  # 22
            length_bytes = struct.pack('<H', length)  # 小端序

            # 计算CRC16 (从msg_type到payload末尾)
            crc_data = bytes([msg_type]) + length_bytes + payload
            crc16 = _calc_crc16(crc_data)
            crc_bytes = struct.pack('<H', crc16)

            # 组装完整包
            packet = header + bytes([msg_type]) + length_bytes + payload + crc_bytes

            # 发送
            _serial_port.write(packet)
            _serial_port.flush()  # 确保数据发送

            # 【调试输出】每10次打印一次
            if not hasattr(send_setpoint, '_send_count'):
                send_setpoint._send_count = 0
            send_setpoint._send_count += 1
            if send_setpoint._send_count % 10 == 0:
                print(f"[CMD-BIN] Setpoint seq={seq} | Target:({sp.x:.2f},{sp.y:.2f},{math.degrees(sp.theta):.1f}°) "
                      f"v={sp.v:.2f}m/s ω={sp.omega:.2f}rad/s | Sent {len(packet)} bytes")

            return True

        except Exception as e:
            if attempt < retry - 1:
                print(f"[RobotSerial] 发送失败(尝试{attempt+1}/{retry}): {e}, 重试中...")
                time.sleep(0.05)
            else:
                print(f"[RobotSerial] 发送位姿设定点失败(已重试{retry}次): {e}")
                import traceback
                traceback.print_exc()

                # 【修复】通信失败时尝试重新连接
                print(f"[RobotSerial] 尝试重新连接串口...")
                try:
                    close()
                    time.sleep(0.5)
                    # 注意：这里需要用户重新调用open()来重新连接
                except:
                    pass

                return False

    return False


def _calc_crc16(data: bytes) -> int:
    """
    CRC16-CCITT计算 (多项式0x1021)
    匹配STM32固件的Comm_CRC16实现
    """
    crc = 0xFFFF
    for byte in data:
        crc ^= (byte << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc <<= 1
            crc &= 0xFFFF  # 保持16位
    return crc


def set_odometry_pose(x: float, y: float, theta: float) -> bool:
    """
    【新增】设置STM32里程计的初始位置，用于坐标系对齐

    这个函数用于将STM32的里程计坐标系与Python的世界坐标系对齐。
    例如：如果entrance在(1.05, 1.05)，需要调用此函数设置STM32的初始位置。

    数据包格式: [0xAA 0x55] [0x02] [0x00 0x0C] [12字节SetOdometryData] [CRC16]
    SetOdometryData: 3个float (x, y, theta)

    Args:
        x: 初始X坐标 (米)
        y: 初始Y坐标 (米)
        theta: 初始航向角 (弧度)

    Returns:
        bool: 发送成功返回True，失败返回False
    """
    global _serial_port
    import struct

    if _serial_port is None:
        print("[RobotSerial] 错误: 串口未打开")
        return False

    try:
        # 构建SetOdometryData载荷 (12字节)
        # 格式: 3f (3个float: x, y, theta)
        payload = struct.pack('<3f', x, y, theta)

        # 构建完整数据包
        header = bytes([0xAA, 0x55])
        msg_type = 0x02  # MSG_TYPE_SET_ODOMETRY
        length = len(payload)  # 12
        length_bytes = struct.pack('<H', length)  # 小端序

        # 计算CRC16 (从msg_type到payload末尾)
        crc_data = bytes([msg_type]) + length_bytes + payload
        crc16 = _calc_crc16(crc_data)
        crc_bytes = struct.pack('<H', crc16)

        # 组装完整包
        packet = header + bytes([msg_type]) + length_bytes + payload + crc_bytes

        # 发送
        _serial_port.write(packet)
        _serial_port.flush()

        print(f"[ODOM-SET] 设置里程计初始位置: x={x:.3f}m y={y:.3f}m θ={math.degrees(theta):.1f}° | Sent {len(packet)} bytes")
        return True

    except Exception as e:
        print(f"[RobotSerial] 设置里程计初始位置失败: {e}")
        import traceback
        traceback.print_exc()
        return False


# ==============================================================================
# 调试工具
# ==============================================================================
def get_latest_imu_data() -> dict:
    """获取最新IMU数据(调试用)"""
    with _data_lock:
        return _latest_imu_data.copy()


def get_lidar_buffer_size() -> int:
    """获取激光雷达缓冲区大小(调试用)"""
    with _data_lock:
        return len(_lidar_buffer)


def print_status() -> None:
    """打印当前状态(调试用)"""
    imu = get_latest_imu_data()
    lidar_size = get_lidar_buffer_size()

    print("\n========== Robot Serial Status ==========")
    print(f"串口状态: {'已连接' if _serial_port and _serial_port.is_open else '未连接'}")
    print(f"接收线程: {'运行中' if _running else '已停止'}")
    print(f"\nIMU数据:")
    print(f"  Yaw:   {imu['yaw']:.2f}°")
    print(f"  Roll:  {imu['roll']:.2f}°")
    print(f"  Pitch: {imu['pitch']:.2f}°")
    print(f"  Accel: {imu['accel']}")
    print(f"  Gyro:  {imu['gyro']}")
    print(f"  更新时间: {time.time() - imu['timestamp']:.2f}s前")
    print(f"\n激光雷达缓冲区: {lidar_size} 个点")
    print(f"\n里程计估计:")
    print(f"  位置: ({_odometry['x']:.3f}, {_odometry['y']:.3f}) m")
    print(f"  航向: {np.rad2deg(_odometry['theta']):.2f}°")
    print("=========================================\n")


# ==============================================================================
# 测试代码
# ==============================================================================
if __name__ == '__main__':
    import sys

    # 示例: python robot_serial_interface_text.py COM5
    port = sys.argv[1] if len(sys.argv) > 1 else 'COM5'

    print(f"连接到 {port}...")
    init(port=port, baudrate=115200)

    try:
        print("等待数据...(按 Ctrl+C 退出)\n")

        while True:
            time.sleep(3)
            print_status()

            # 测试获取位姿
            try:
                pose = get_robot_pose(timeout=0.5)
                print(f"✓ 获取位姿成功: x={pose.x:.3f}, y={pose.y:.3f}, theta={np.rad2deg(pose.theta):.1f}°")
            except TimeoutError as e:
                print(f"✗ 获取位姿失败: {e}")

            # 测试获取激光扫描
            try:
                scan = get_lidar_scan(timeout=1.0, min_points=10)
                print(f"✓ 获取激光扫描成功: {len(scan.ranges)} 个点")
            except TimeoutError as e:
                print(f"✗ 获取激光扫描失败: {e}")

    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        close()
        print("程序结束")
