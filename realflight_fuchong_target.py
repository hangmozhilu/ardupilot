#!/usr/bin/env python3
import atexit
import datetime
import math
import socket
import struct
import time

from pymavlink import mavutil


# -------------------------------
# 用户设置：目标 GPS 坐标
# 改成 RealFlight 场景里你想撞击的位置
# -------------------------------
TARGET_LAT = 29.8070610      # 目标纬度
TARGET_LON = 119.6731281    # 目标经度
TARGET_ALT = 25           # 目标海拔高度（m，AMSL）

# 相机参数
CAM_W = 1920
CAM_H = 1080
HFOV_DEG = 120.0
VFOV_DEG = 90.0

# 连接参数
MAVLINK_URL = 'tcp:127.0.0.1:5762'
SITL_SERIAL5_HOST = '127.0.0.1'
SITL_SERIAL5_PORT = 5765
SEND_HZ = 50

# 脱靶量记录文件
LOG_FILE = 'fuchong_miss_distance.csv'


# -------------------------------
# 坐标转换
# -------------------------------
def latlon_to_ned(plane_lat, plane_lon, plane_alt,
                  tgt_lat, tgt_lon, tgt_alt):
    """目标相对飞机的 NED 位置（m）"""
    lat_rad = math.radians(plane_lat)
    m_per_deg_lat = 111320.0
    m_per_deg_lon = 111320.0 * math.cos(lat_rad)

    north = (tgt_lat - plane_lat) * m_per_deg_lat
    east  = (tgt_lon - plane_lon) * m_per_deg_lon
    down  = plane_alt - tgt_alt      # NED 下正
    return north, east, down


def ned_to_pixel(n, e, d, roll, pitch, yaw):
    """
    将目标相对飞机的 NED 向量转换为机体固定相机的像素坐标。
    与飞控端处理一致：先去掉航向/俯仰得到“无滚转、带俯仰”框架下的视线角，
    再按机体 roll 预旋转，模拟相机图像随飞机滚转。
    """
    cy, sy = math.cos(yaw), math.sin(yaw)
    cp, sp = math.cos(pitch), math.sin(pitch)

    # 1. NED -> 无滚转、带俯仰的机体框架（与飞控去滚转后的框架一致）
    x1 = cy * n + sy * e
    y1 = -sy * n + cy * e
    z1 = d

    # Ry(pitch)^T：正确变换应使"机头轴线上的目标"映射为 bearing=elev=0
    # 验证：目标 NED = (cos p, 0, -sin p) -> x=1, z=0
    x = x1 * cp - z1 * sp
    y = y1
    z = x1 * sp + z1 * cp

    # 2. 该框架下的方位角（右正）和俯仰角（上正）
    bearing = math.atan2(y, x)
    elev = math.atan2(-z, math.hypot(x, y))

    # 3. 使用 tan 映射到归一化像素坐标，与飞控 FOV 模型保持一致
    # 飞控端：pixel_yaw = atan(nx_level * tan(hfov/2))
    # 因此逆变换：nx_level = tan(bearing) / tan(hfov/2)
    hfov = math.radians(HFOV_DEG)
    vfov = math.radians(VFOV_DEG)
    nx_level = math.tan(bearing) / math.tan(hfov * 0.5)
    ny_level = math.tan(elev) / math.tan(vfov * 0.5)

    # 4. 按机体滚转预旋转，模拟机体固定相机
    cr, sr = math.cos(roll), math.sin(roll)
    nx_rolled = nx_level * cr + ny_level * sr
    ny_rolled = -nx_level * sr + ny_level * cr

    # 5. 转成像素坐标
    px = int((nx_rolled + 1.0) * 0.5 * CAM_W)
    py = int((1.0 - ny_rolled) * 0.5 * CAM_H)

    # 限制在画面边界内，但 active 以归一化坐标是否越界为准
    px = max(0, min(CAM_W - 1, px))
    py = max(0, min(CAM_H - 1, py))

    return px, py, bearing, elev, nx_rolled, ny_rolled


def build_frame(cx, cy, yaw_cdeg, pitch_cdeg, conf, active):
    payload = struct.pack('<iiiiBB', cx, cy, yaw_cdeg, pitch_cdeg, conf, active)
    crc = 0
    for b in payload:
        crc ^= b
    return b'\xFA\xAA' + payload + bytes([crc]) + b'\xAF\x55'


# -------------------------------
# 主循环
# -------------------------------
def main():
    print(f"Connecting MAVLink at {MAVLINK_URL} ...")
    mav = mavutil.mavlink_connection(MAVLINK_URL)
    mav.wait_heartbeat()
    print("MAVLink OK")

    # 请求姿态和位置数据
    mav.mav.request_data_stream_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_POSITION, 10, 1)
    mav.mav.request_data_stream_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 20, 1)

    print(f"Connecting SITL SERIAL5 {SITL_SERIAL5_HOST}:{SITL_SERIAL5_PORT} ...")
    ser = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    ser.connect((SITL_SERIAL5_HOST, SITL_SERIAL5_PORT))
    print("SERIAL5 OK")

    period = 1.0 / SEND_HZ
    last_send = 0
    last_debug = 0.0

    plane_lat = plane_lon = plane_alt = None
    roll = pitch = yaw = None
    v_north = v_east = v_down = 0.0

    # 脱靶量统计
    min_h_miss = float('inf')
    min_3d_miss = float('inf')
    start_time = time.time()

    # CSV 记录
    csv = open(LOG_FILE, 'w')
    csv.write('timestamp,rel_t_s,h_dist_m,slant_range_m,bearing_deg,elev_deg,'
              'vx_north_m_s,vy_east_m_s,vz_down_m_s,closing_speed_m_s,'
              'cross_track_h_m,px,py,active\n')
    csv.flush()

    def print_summary():
        print("\n===== FUCHONGCESHI 制导结果统计 =====")
        print(f"最小水平脱靶量: {min_h_miss:.2f} m")
        print(f"最小三维脱靶量: {min_3d_miss:.2f} m")
        print(f"总运行时间:     {time.time() - start_time:.1f} s")
        print(f"数据已保存到:   {LOG_FILE}")
        csv.close()

    atexit.register(print_summary)

    while True:
        msg = mav.recv_match(blocking=False)
        if msg is not None:
            t = msg.get_type()
            if t == 'GLOBAL_POSITION_INT':
                plane_lat = msg.lat / 1e7
                plane_lon = msg.lon / 1e7
                plane_alt = msg.alt / 1e3
                v_north = msg.vx / 100.0
                v_east = msg.vy / 100.0
                v_down = msg.vz / 100.0
            elif t == 'ATTITUDE':
                roll = msg.roll
                pitch = msg.pitch
                yaw = msg.yaw

        now = time.time()
        if now - last_send < period:
            time.sleep(0.001)
            continue

        if None in (plane_lat, plane_lon, plane_alt, roll, pitch, yaw):
            continue

        # 1. 目标相对飞机的 NED 向量
        n, e, d = latlon_to_ned(plane_lat, plane_lon, plane_alt,
                                TARGET_LAT, TARGET_LON, TARGET_ALT)

        # 2. 计算脱靶量
        h_dist = math.hypot(n, e)
        slant_range = math.hypot(h_dist, d)
        min_h_miss = min(min_h_miss, h_dist)
        min_3d_miss = min(min_3d_miss, slant_range)

        # 3. 计算“无滚转、带俯仰”框架下的视线角并模拟机体滚转生成像素
        px, py, bearing, elev, nx_rolled, ny_rolled = ned_to_pixel(
            n, e, d, roll, pitch, yaw)

        # 目标在视场范围内才标记为有效
        active = 1 if (-1.0 <= nx_rolled <= 1.0 and -1.0 <= ny_rolled <= 1.0) else 0
        conf = 95

        # 云台角度固定为 0（模拟机体固定摄像头）
        yaw_cdeg = 0
        pitch_cdeg = 0

        frame = build_frame(px, py, yaw_cdeg, pitch_cdeg, conf, active)
        ser.sendall(frame)
        last_send = now

        # 6. 计算接近速度和水平脱靶量（用于评估制导效果）
        if slant_range > 0.1:
            closing_speed = (n * v_north + e * v_east + d * v_down) / slant_range
        else:
            closing_speed = 0.0

        v_h = math.hypot(v_north, v_east)
        if v_h > 0.5:
            cross_track_h = abs(n * v_east - e * v_north) / v_h
        else:
            cross_track_h = h_dist

        # 7. 写入 CSV，每秒刷新一次磁盘
        rel_t = now - start_time
        csv.write(f"{datetime.datetime.now().isoformat()},{rel_t:.3f},"
                  f"{h_dist:.3f},{slant_range:.3f},"
                  f"{math.degrees(bearing):.3f},{math.degrees(elev):.3f},"
                  f"{v_north:.3f},{v_east:.3f},{v_down:.3f},"
                  f"{closing_speed:.3f},{cross_track_h:.3f},"
                  f"{px},{py},{active}\n")
        if int(rel_t) > int(rel_t - period):
            csv.flush()

        # 每秒在终端打印一次目标方位和脱靶量，便于无图像辅助时瞄准
        if now - last_debug >= 1.0:
            print(f"[DEBUG] slant={slant_range:7.1f}m  hdist={h_dist:7.1f}m  "
                  f"min_h={min_h_miss:7.2f}m  min_3d={min_3d_miss:7.2f}m  "
                  f"v_close={closing_speed:6.1f}m/s  "
                  f"bearing={math.degrees(bearing):+7.1f}deg  "
                  f"elev={math.degrees(elev):+6.1f}deg  "
                  f"px={px:5d} py={py:5d}  active={active}")
            last_debug = now


if __name__ == '__main__':
    main()