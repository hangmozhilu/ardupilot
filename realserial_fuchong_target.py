#!/usr/bin/env python3
"""
realserial_fuchong_target.py — 真实AI识别模块串口 → SITL 飞控桥接脚本

功能：
  从真实硬件串口（如 /dev/ttyUSB0 或 COM3）读取 AI 识别模块发送的
  FUCHONGCESHI 目标帧（23字节二进制协议），校验后转发到 SITL 飞控的
  SERIAL5 端口（TCP）。

用法：
  python3 realserial_fuchong_target.py COM3              # Windows
  python3 realserial_fuchong_target.py /dev/ttyUSB0      # Linux
  python3 realserial_fuchong_target.py COM3 -v            # verbose 输出

协议帧格式（23字节）：
  [0xFA][0xAA][cx:4][cy:4][yaw:4][pitch:4][conf:1][active:1][crc:1][0xAF][0x55]
"""

import argparse
import struct
import socket
import sys
import time
import datetime

# -------------------------------
# 配置
# -------------------------------
SITL_SERIAL5_HOST = '127.0.0.1'
SITL_SERIAL5_PORT = 5765
SERIAL_BAUD = 115200

FRAME_LEN = 23
FRAME_HEADER = b'\xFA\xAA'
FRAME_TAIL = b'\xAF\x55'

# 重连间隔（秒）
RECONNECT_DELAY_SERIAL = 2.0
RECONNECT_DELAY_SITL = 1.0


def hex_str(data: bytes) -> str:
    """将字节串转为十六进制字符串，用于调试打印"""
    return ' '.join(f'{b:02X}' for b in data)


def xor_checksum(data: bytes) -> int:
    """计算 18 字节载荷的 XOR 校验值"""
    c = 0
    for b in data:
        c ^= b
    return c


def parse_frame(frame: bytes) -> dict:
    """
    解析完整帧（23字节），返回结构化字典。
    帧格式：
      [0] 0xFA, [1] 0xAA
      [2-5]  cx (int32 LE)
      [6-9]  cy (int32 LE)
      [10-13] yaw_cdeg (int32 LE)
      [14-17] pitch_cdeg (int32 LE)
      [18] conf (uint8)
      [19] active (uint8)
      [20] crc (uint8, XOR of bytes [2-19])
      [21] 0xAF, [22] 0x55
    """
    if len(frame) != FRAME_LEN:
        return None
    if frame[0:2] != FRAME_HEADER:
        return None
    if frame[21:23] != FRAME_TAIL:
        return None

    payload = frame[2:20]  # 18 bytes payload
    crc_received = frame[20]
    if xor_checksum(payload) != crc_received:
        return None

    cx, cy, yaw, pitch, conf, active = struct.unpack('<iiiiBB', payload)
    return {
        'cx': cx, 'cy': cy,
        'yaw_cdeg': yaw, 'pitch_cdeg': pitch,
        'conf': conf, 'active': active,
    }


def open_serial(port: str):
    """打开真实串口，失败返回 None"""
    try:
        import serial
        ser = serial.Serial(port, SERIAL_BAUD, timeout=0.5)
        print(f"[串口] 已连接 {port} @ {SERIAL_BAUD} baud")
        return ser
    except ImportError:
        print("[错误] 需要安装 pyserial: pip install pyserial")
        sys.exit(1)
    except Exception as e:
        print(f"[错误] 无法打开串口 {port}: {e}")
        return None


def open_sitl():
    """连接 SITL SERIAL5 TCP 端口，失败返回 None"""
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(3.0)
        sock.connect((SITL_SERIAL5_HOST, SITL_SERIAL5_PORT))
        sock.settimeout(None)
        print(f"[SITL] 已连接 {SITL_SERIAL5_HOST}:{SITL_SERIAL5_PORT}")
        return sock
    except Exception as e:
        print(f"[错误] 无法连接 SITL SERIAL5: {e}")
        return None


def main():
    parser = argparse.ArgumentParser(
        description='真实AI识别模块串口 → SITL 飞控桥接')
    parser.add_argument('serial_port', help='串口设备名，如 COM3 或 /dev/ttyUSB0')
    parser.add_argument('-v', '--verbose', action='store_true',
                        help='打印每帧解析结果')
    parser.add_argument('--log', default='realserial_fuchong.csv',
                        help='CSV 日志文件路径（默认 realserial_fuchong.csv）')
    args = parser.parse_args()

    # 打开 CSV 日志
    csv = open(args.log, 'w')
    csv.write('timestamp,rel_t,cx,cy,yaw_cdeg,pitch_cdeg,conf,active,valid\n')
    csv.flush()
    print(f"[日志] 写入 {args.log}")

    # 统计
    total_frames = 0
    valid_frames = 0
    bad_frames = 0
    last_status = 0.0
    start_time = time.time()

    # 帧缓冲区
    buf = bytearray()

    while True:
        # ---- 连接串口 ----
        ser = None
        while ser is None:
            ser = open_serial(args.serial_port)
            if ser is None:
                time.sleep(RECONNECT_DELAY_SERIAL)
        ser.reset_input_buffer()

        # ---- 连接 SITL ----
        sitl = None
        while sitl is None:
            sitl = open_sitl()
            if sitl is None:
                time.sleep(RECONNECT_DELAY_SITL)

        print(f"[运行] 开始转发数据，Ctrl+C 停止")
        buf.clear()

        try:
            while True:
                # ---- 从串口读取数据 ----
                try:
                    data = ser.read(ser.in_waiting or 1)
                except Exception:
                    print("[串口] 读取错误，尝试重连...")
                    break

                if not data:
                    continue

                buf.extend(data)

                # 在缓冲区中寻找完整帧
                while True:
                    # 寻找帧头 0xFA 0xAA
                    idx = buf.find(FRAME_HEADER)
                    if idx < 0:
                        # 只保留最后一个字节（可能是 0xFA）
                        if len(buf) > 0 and buf[-1] == 0xFA:
                            buf = bytearray([0xFA])
                        else:
                            buf.clear()
                        break

                    # 丢弃帧头之前的无效数据
                    if idx > 0:
                        buf = buf[idx:]
                        idx = 0

                    # 帧头在位置0，检查是否有足够的数据
                    if len(buf) < FRAME_LEN:
                        break

                    # 提取候选帧
                    candidate = bytes(buf[:FRAME_LEN])
                    buf = buf[FRAME_LEN:]

                    total_frames += 1
                    parsed = parse_frame(candidate)

                    rel_t = time.time() - start_time

                    if parsed is not None:
                        valid_frames += 1
                        # 转发到 SITL
                        try:
                            sitl.sendall(candidate)
                        except Exception:
                            print("[SITL] 发送失败，尝试重连...")
                            sitl.close()
                            sitl = None
                            # 放回缓冲区，等待重连后重发
                            buf = candidate + buf
                            break

                        # 写入 CSV
                        csv.write(
                            f"{datetime.datetime.now().isoformat()},{rel_t:.3f},"
                            f"{parsed['cx']},{parsed['cy']},"
                            f"{parsed['yaw_cdeg']},{parsed['pitch_cdeg']},"
                            f"{parsed['conf']},{parsed['active']},1\n"
                        )

                        if args.verbose:
                            print(f"[帧] cx={parsed['cx']:5d} cy={parsed['cy']:5d} "
                                  f"yaw={parsed['yaw_cdeg']:+6d}° "
                                  f"pitch={parsed['pitch_cdeg']:+6d}° "
                                  f"conf={parsed['conf']}% active=0x{parsed['active']:02X}")
                    else:
                        bad_frames += 1
                        csv.write(
                            f"{datetime.datetime.now().isoformat()},{rel_t:.3f},"
                            f"0,0,0,0,0,0,0\n"
                        )
                        if args.verbose:
                            print(f"[帧] 无效帧: {hex_str(candidate[:8])}...")

                    # 定期刷新状态
                    now = time.time()
                    if now - last_status >= 5.0:
                        rate = total_frames / max(now - start_time, 1)
                        print(f"[状态] 总数={total_frames} 有效={valid_frames} "
                              f"无效={bad_frames} 速率={rate:.1f} fps")
                        last_status = now
                        csv.flush()

        except KeyboardInterrupt:
            print("\n[退出] 用户中断")
            break
        finally:
            if ser:
                ser.close()
            if sitl:
                sitl.close()

        # 重连前等待
        print(f"[重连] {RECONNECT_DELAY_SERIAL}s 后重试...")
        time.sleep(RECONNECT_DELAY_SERIAL)

    csv.close()
    print(f"\n[统计] 总帧数={total_frames} 有效={valid_frames} 无效={bad_frames}")


if __name__ == '__main__':
    main()