#!/usr/bin/env python3
import atexit
import datetime
import math
import socket
import struct
import time

from pymavlink import mavutil


# -------------------------------
# 用户设置：空中目标 GPS 坐标（3D空间点）
# 设置目标在空中的经纬度和海拔高度
# -------------------------------
TARGET_LAT = 29.8020152      # 目标纬度
TARGET_LON = 119.6791363    # 目标经度
TARGET_ALT = 300           # 目标海拔高度（m，AMSL）—— 空中目标，设为100m以上

# 相机参数
CAM_W = 640
CAM_H = 480
HFOV_DEG = 120.0
VFOV_DEG = 90.0

# 连接参数
MAVLINK_URL = 'tcp:127.0.0.1:5762'
SITL_SERIAL5_HOST = '127.0.0.1'
SITL_SERIAL5_PORT = 5765
SEND_HZ = 50

# 制导参数前缀（当前固件为 GA_；若后续改为 ImgGuide_，只需改这里）
PARAM_PREFIX = 'GA_'

# 目标仍被视为“在视场内”的归一化边界（硬边界）
ACTIVE_FOV_LIMIT = 1.0

# 脱靶量记录文件
LOG_FILE = 'img_guide_miss_distance.csv'


# -------------------------------
# 坐标转换
# -------------------------------
def latlon_to_ned(plane_lat, plane_lon, plane_alt,
                  tgt_lat, tgt_lon, tgt_alt):
    """目标相对飞机的 NED 位置（m）"""
    lat_rad = math.radians(plane_lat)
    # WGS84 米/度近似，比固定 111320 更精确
    m_per_deg_lat = 111132.92 - 559.82 * math.cos(2.0 * lat_rad) + 1.175 * math.cos(4.0 * lat_rad)
    m_per_deg_lon = 111412.84 * math.cos(lat_rad) - 93.5 * math.cos(3.0 * lat_rad)

    north = (tgt_lat - plane_lat) * m_per_deg_lat
    east  = (tgt_lon - plane_lon) * m_per_deg_lon
    down  = plane_alt - tgt_alt      # NED 下正
    return north, east, down


def ned_to_pixel(n, e, d, roll, pitch, yaw):
    """
    将目标相对飞机的 NED 向量转换为机体固定相机的像素坐标。
    与飞控端处理一致：先去掉航向/俯仰得到"无滚转、带俯仰"框架下的视线角，
    再按机体 roll 预旋转，模拟相机图像随飞机滚转。
    返回 (cx, cy, bearing, elev, nx_rolled, ny_rolled, nx_level, ny_level, visible)
    其中 cx, cy 是以图像中心为原点的坐标（与 ImgGuide 飞控协议一致）
    visible 表示目标是否在相机前方可视半空间内
    """
    cy_s, sy = math.cos(yaw), math.sin(yaw)
    cp, sp = math.cos(pitch), math.sin(pitch)

    # 1. NED -> 无滚转、带俯仰的机体框架（与飞控去滚转后的框架一致）
    x1 = cy_s * n + sy * e
    y1 = -sy * n + cy_s * e
    z1 = d

    # Ry(pitch)^T：正确变换应使"机头轴线上的目标"映射为 bearing=elev=0
    x = x1 * cp - z1 * sp
    y = y1
    z = x1 * sp + z1 * cp

    # 2. 判断目标是否在相机前方（x > 0）。相机是前视的，后方目标不可见。
    visible = (x > 0.0)

    # 该框架下的方位角（右正）和俯仰角（上正）
    bearing = math.atan2(y, x)
    elev = math.atan2(-z, math.hypot(x, y))

    # 3. 使用 tan 映射到归一化像素坐标，与飞控 FOV 模型保持一致
    hfov = math.radians(HFOV_DEG)
    vfov = math.radians(VFOV_DEG)
    nx_level = math.tan(bearing) / math.tan(hfov * 0.5)
    ny_level = math.tan(elev) / math.tan(vfov * 0.5)

    # 4. 按机体滚转预旋转，模拟机体固定相机
    cr, sr = math.cos(roll), math.sin(roll)
    nx_rolled = nx_level * cr + ny_level * sr
    ny_rolled = -nx_level * sr + ny_level * cr

    # 5. 转成以图像中心为原点的像素坐标（与 ImgGuide 飞控协议一致）
    # 飞控端：nx = camera_x / (CAM_W*0.5), ny = camera_y / (CAM_H*0.5)
    # 因此：camera_x = nx_rolled * CAM_W * 0.5, camera_y = ny_rolled * CAM_H * 0.5
    cx = int(nx_rolled * CAM_W * 0.5)
    cy = int(ny_rolled * CAM_H * 0.5)

    return cx, cy, bearing, elev, nx_rolled, ny_rolled, nx_level, ny_level, visible


def build_frame(cx, cy, yaw_cdeg, pitch_cdeg, conf, active):
    payload = struct.pack('<iiiiBB', cx, cy, yaw_cdeg, pitch_cdeg, conf, active)
    crc = 0
    for b in payload:
        crc ^= b
    return b'\xFA\xAA' + payload + bytes([crc]) + b'\xAF\x55'


def setup_guidance_params(mav):
    """
    将飞控中与 ImgGuide 仿真相关的关键参数设为与脚本一致。
    重点：GA_HFOV/GA_VFOV 必须与下方 HFOV_DEG/VFOV_DEG 一致，
    否则飞控的像素→LOS 角度映射会出现偏差。
    """
    params = {
        f'{PARAM_PREFIX}HFOV': HFOV_DEG,
        f'{PARAM_PREFIX}VFOV': VFOV_DEG,
        # 新纵向航迹角跟踪参数，可按场景微调
        f'{PARAM_PREFIX}PTCH_KP': 1.2,
        f'{PARAM_PREFIX}PTCH_KD': 0.3,
        # FOV 保持控制器，便于测试边缘拉回
        f'{PARAM_PREFIX}FOV_MAR': 0.85,
        f'{PARAM_PREFIX}FOV_GAIN': 0.3,
    }
    print(f"Configuring {len(params)} guidance parameters (prefix={PARAM_PREFIX}) ...")
    for name, value in params.items():
        print(f"  {name} = {value}")
        mav.param_set_send(name, float(value),
                           parm_type=mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    print("Parameter setup done")


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

    setup_guidance_params(mav)

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
    csv.write('timestamp,rel_t_s,plane_lat,plane_lon,plane_alt_m,'
              'roll_deg,pitch_deg,yaw_deg,'
              'h_dist_m,slant_range_m,bearing_deg,elev_deg,'
              'vx_north_m_s,vy_east_m_s,vz_down_m_s,closing_speed_m_s,'
              'cross_track_h_m,gamma_desired_deg,gamma_current_deg,'
              'nx_level,ny_level,nx_rolled,ny_rolled,cx,cy,active\n')
    csv.flush()

    def print_summary():
        print("\n===== ImgGuide 制导结果统计 =====")
        print(f"目标坐标: ({TARGET_LAT:.7f}, {TARGET_LON:.7f}, {TARGET_ALT:.0f}m)")
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

        # 3. 计算"无滚转、带俯仰"框架下的视线角并模拟机体滚转生成像素
        # 输出以图像中心为原点的坐标 (cx, cy)，与 ImgGuide 飞控协议一致
        cx, cy, bearing, elev, nx_rolled, ny_rolled, nx_level, ny_level, visible = ned_to_pixel(
            n, e, d, roll, pitch, yaw)

        # 4. 目标在视场范围内且在前方可视半空间时才标记为追踪
        # In FOV & in front: active=0x22 triggers ImgGuide auto-switch; otherwise 0x00
        active = 0x22 if (visible and
                          abs(nx_rolled) <= ACTIVE_FOV_LIMIT and
                          abs(ny_rolled) <= ACTIVE_FOV_LIMIT) else 0x00
        in_fov = 1 if active == 0x22 else 0  # 打印用标志位：1=在视场内，0=不在
        conf = 100  # 置信度100%，满足 >=70% 的自动切换阈值

        # 5. 云台角度固定为 0（模拟机体固定摄像头）
        yaw_cdeg = 0
        pitch_cdeg = 0

        # 只有目标在视场内且位于相机前方时，才向飞控输出目标数据
        if in_fov:
            frame = build_frame(cx, cy, yaw_cdeg, pitch_cdeg, conf, active)
            ser.sendall(frame)
        last_send = now

        # 6. 计算接近速度
        if slant_range > 0.1:
            closing_speed = (n * v_north + e * v_east + d * v_down) / slant_range
        else:
            closing_speed = 0.0

        # 7. 计算水平脱靶量
        v_h = math.hypot(v_north, v_east)
        if v_h > 0.5:
            cross_track_h = abs(n * v_east - e * v_north) / v_h
        else:
            cross_track_h = h_dist

        # 8. 航迹角（用于评估新纵向制导通道）
        gamma_desired = math.degrees(math.atan2(-d, h_dist)) if h_dist > 0.1 else 0.0
        gamma_current = math.degrees(math.atan2(-v_down, v_h)) if v_h > 0.5 else 0.0

        # 9. 写入 CSV，每秒刷新一次磁盘
        rel_t = now - start_time
        csv.write(f"{datetime.datetime.now().isoformat()},{rel_t:.3f},"
              f"{plane_lat:.7f},{plane_lon:.7f},{plane_alt:.3f},"
              f"{math.degrees(roll):.3f},{math.degrees(pitch):.3f},{math.degrees(yaw):.3f},"
              f"{h_dist:.3f},{slant_range:.3f},"
              f"{math.degrees(bearing):.3f},{math.degrees(elev):.3f},"
              f"{v_north:.3f},{v_east:.3f},{v_down:.3f},"
              f"{closing_speed:.3f},{cross_track_h:.3f},"
              f"{gamma_desired:.3f},{gamma_current:.3f},"
              f"{nx_level:.4f},{ny_level:.4f},{nx_rolled:.4f},{ny_rolled:.4f},"
              f"{cx},{cy},{active:#04x}\n")
        if int(rel_t) > int(rel_t - period):
            csv.flush()

        # 每秒在终端打印一次目标方位和脱靶量，便于无图像辅助时瞄准
        if now - last_debug >= 1.0:
            print(f"[DEBUG] slant={slant_range:7.1f}m  hdist={h_dist:7.1f}m  "
                  f"min_h={min_h_miss:7.2f}m  min_3d={min_3d_miss:7.2f}m  "
                  f"v_close={closing_speed:6.1f}m/s  "
                  f"bearing={math.degrees(bearing):+7.1f}deg  "
                  f"elev={math.degrees(elev):+6.1f}deg  "
                  f"gamma_des={gamma_desired:+6.1f}deg  gamma_cur={gamma_current:+6.1f}deg  "
                  f"nx={nx_level:+.3f} ny={ny_level:+.3f}  "
                  f"cx={cx:5d} cy={cy:5d}  in_fov={in_fov}  active=0x{active:02X}")
            last_debug = now


if __name__ == '__main__':
    main()