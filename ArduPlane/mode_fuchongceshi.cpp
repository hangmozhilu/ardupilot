#include "mode.h"
#include "Plane.h"

#if HAL_QUADPLANE_ENABLED

#include <AP_SerialManager/AP_SerialManager.h>
#include <string.h>
#include <cmath>

/*
  FUCHONGCESHI flight mode for precision image-guided collision.精准图像引导碰撞模式

  Gimbal binary protocol (configure the gimbal/camera UART to
  SERIALn_PROTOCOL = 50 (SerialProtocol_FuchongTarget)):

    [0xFA][0xAA][cx][cy][yaw][pitch][conf][active][XX][0xAF][0x55]

  Frame length: 23 bytes
    header (2)  : 0xFA, 0xAA   帧头
    cx (4)      : int32 little-endian, 目标x像素坐标, 0..1920
    cy (4)      : int32 little-endian, 目标y像素坐标, 0..1080
    yaw (4)     : int32 little-endian, 云台相对飞机的偏航角度, 0.01度, 0.01 deg (+ = right)
    pitch (4)   : int32 little-endian, 云台相对飞机的俯仰角度, 0.01度, 0.01 deg (+ = up)
    conf (1)    : uint8, 目标置信度, 0..100 (0%..100%)
    active (1)  : uint8, 目标是否有效, 1 if target valid
    XX (1)      : XOR of all bytes between header and tail (payload bytes)
    tail (2)    : 0xAF, 0x55   帧尾

  Guidance:
    - bearing (azimuth) error -> roll command (bank-to-turn)  方位误差通过横滚修正
    - elevation error + programmed dive bias -> pitch command  俯仰误差通过俯仰修正
    - proportional navigation rate term improves tracking of moving targets  比例导航率项改善移动目标的跟踪效果
    - image coordinates are deskewed using aircraft roll  图像坐标通过飞机横滚修正为直角
*/

// little-endian int32 from byte buffer 字节序转换
static inline int32_t int32_from_le(const uint8_t *b)
{
    return (int32_t)((uint32_t)b[0] |
                     ((uint32_t)b[1] << 8) |
                     ((uint32_t)b[2] << 16) |
                     ((uint32_t)b[3] << 24));
}

ModeFuchongceshi::ModeFuchongceshi()
    : frame_idx(0),
      parse_state(0),
      loss_action_triggered(false),
      uart(nullptr),
      uart_initialised(false),
      target{},
      guidance{}
{
    memset(frame_buffer, 0, sizeof(frame_buffer));
}

bool ModeFuchongceshi::_enter()
{
    init_uart();

    frame_idx = 0;
    parse_state = 0;
    loss_action_triggered = false;
    memset(frame_buffer, 0, sizeof(frame_buffer));

    target.camera_x = 0;
    target.camera_y = 0;
    target.gimbal_yaw_cdeg = 0;
    target.gimbal_pitch_cdeg = 0;
    target.confidence = 0;
    target.active = false;
    target.last_update_ms = 0;

    guidance.bearing_error_rad = 0;
    guidance.bearing_rate_rad_s = 0;
    guidance.elevation_error_rad = 0;
    guidance.last_bearing_error_rad = 0;
    guidance.last_update_ms = 0;
    guidance.target_valid = false;

    plane.nav_roll_cd = 0;
    plane.nav_pitch_cd = 0;

    plane.gcs().send_text(MAV_SEVERITY_INFO, "FUCHONGCESHI: entered");
    return true;
}

void ModeFuchongceshi::_exit()
{
    plane.gcs().send_text(MAV_SEVERITY_INFO, "FUCHONGCESHI: exited");
}

void ModeFuchongceshi::init_uart()
{
    if (uart_initialised) {
        return;
    }
    uart = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_FuchongTarget, 0);
    if (uart != nullptr) {
        uint32_t baud = AP::serialmanager().find_baudrate(AP_SerialManager::SerialProtocol_FuchongTarget, 0);
        if (baud == 0) {
            baud = 115200;
        }
        uart->begin(baud);
        plane.gcs().send_text(MAV_SEVERITY_INFO, "FUCHONGCESHI: gimbal UART @ %lu baud", (unsigned long)baud);
    } else {
        plane.gcs().send_text(MAV_SEVERITY_WARNING, "FUCHONGCESHI: no gimbal UART found");
    }
    uart_initialised = true;
}

bool ModeFuchongceshi::validate_frame(const uint8_t *frame) const
{
    // verify tail
    if (frame[19] != 0xAF || frame[20] != 0x55) {
        return false;
    }

    // XOR over the 18 payload bytes (everything between header and tail except checksum itself)
    uint8_t crc = 0;
    for (uint8_t i = 0; i < 18; i++) {
        crc ^= frame[i];
    }
    return crc == frame[18];
}

// 解析 gimbal 发送的图像目标数据帧
bool ModeFuchongceshi::parse_frame(const uint8_t *frame)
{
    const int32_t cx   = int32_from_le(frame + 0);  // 目标x像素坐标, 0..1920
    const int32_t cy   = int32_from_le(frame + 4);  // 目标y像素坐标, 0..1080
    const int32_t gy   = int32_from_le(frame + 8);  // 云台相对飞机的偏航角度, 0.01度, 0.01 deg (+ = right)
    const int32_t gp   = int32_from_le(frame + 12); // 云台相对飞机的俯仰角度, 0.01度, 0.01 deg (+ = up)
    const uint8_t conf = frame[16];  // 目标置信度, 0..100 (0%..100%)
    const uint8_t active = frame[17]; // 目标是否有效, 1 if target valid

    plane.gcs().send_text(MAV_SEVERITY_WARNING, "FUCHONGCESHI: Camera frame:cx=%d, cy=%d, conf=%d, active=%d", (int)cx, (int)cy, (int)conf, (int)active); // 打印识别结果

    target.camera_x = cx;
    target.camera_y = cy;
    target.gimbal_yaw_cdeg   = gy;
    target.gimbal_pitch_cdeg = gp;
    target.confidence = conf * 0.01f;
    target.active = (active != 0);
    return true;
}

// 从串口读取数据并处理状态机
void ModeFuchongceshi::read_serial()
{
    if (uart == nullptr) {
        return;
    }

    const int16_t nbytes = uart->available();
    for (int16_t i = 0; i < nbytes; i++) {
        const int16_t c = uart->read();
        if (c < 0) {
            continue;
        }

        // 状态机逻辑:
        // 0: 等待帧头0xFA
        // 1: 等待帧头0xAA
        // 2: 接收数据帧主体
        switch (parse_state) {
        case 0:
            if (c == 0xFA) {
                parse_state = 1;
            }
            break;

        case 1:
            if (c == 0xAA) {
                parse_state = 2;
                frame_idx = 0;
            } else if (c != 0xFA) {
                // not a valid header, restart scan
                parse_state = 0;
            }
            break;

        case 2:
            frame_buffer[frame_idx++] = uint8_t(c);
            if (frame_idx >= PAYLOAD_TAIL_LEN) {
                // full frame body received, validate and parse
                if (validate_frame(frame_buffer) && parse_frame(frame_buffer)) {
                    target.last_update_ms = AP_HAL::millis();
                }
                parse_state = 0;
                frame_idx = 0;
            }
            break;

        default:
            parse_state = 0;
            frame_idx = 0;
            break;
        }
    }
}

// 检查目标是否有效
bool ModeFuchongceshi::target_valid() const
{
    if (!target.active) {
        return false;
    }
    if (target.confidence < 0.3f) {
        return false;
    }
    if (AP_HAL::millis() - target.last_update_ms > uint32_t(plane.ga_guidance.timeout_ms.get())) {
        return false;
    }
    return true;
}

void ModeFuchongceshi::update_guidance()
{
    const uint32_t now = AP_HAL::millis();

    if (!target_valid()) {
        handle_target_loss();
        return;
    }

    loss_action_triggered = false;
    guidance.target_valid = true;

    // normalize pixel coordinates to [-1, 1], y positive up
    const float nx = 2.0f * (target.camera_x - CAMERA_WIDTH_PX * 0.5f) / CAMERA_WIDTH_PX;
    const float ny = -2.0f * (target.camera_y - CAMERA_HEIGHT_PX * 0.5f) / CAMERA_HEIGHT_PX;

    // deskew image rotation due to aircraft roll
    const float roll = ahrs.get_roll();
    const float cr = cosf(roll);
    const float sr = sinf(roll);
    const float nx_level = nx * cr - ny * sr;
    const float ny_level = nx * sr + ny * cr;

    // convert pixel error to angle using camera FOV
    const float hfov_rad = radians(plane.ga_guidance.hfov.get());
    const float vfov_rad = radians(plane.ga_guidance.vfov.get());
    const float pixel_yaw = nx_level * hfov_rad * 0.5f;
    const float pixel_pitch = ny_level * vfov_rad * 0.5f;

    // total line-of-sight angles in body frame (gimbal + pixel)
    // gimbal angles are in centi-degrees, convert to radians
    const float los_yaw = radians(target.gimbal_yaw_cdeg * 0.01f) + pixel_yaw;
    const float los_pitch = radians(target.gimbal_pitch_cdeg * 0.01f) + pixel_pitch;

    guidance.bearing_error_rad = los_yaw;
    guidance.elevation_error_rad = los_pitch;

    // bearing rate for proportional navigation / damping
    float dt = 0;
    if (guidance.last_update_ms != 0) {
        dt = (now - guidance.last_update_ms) * 0.001f;
    }
    if (dt <= 0.0f || dt > 0.5f) {
        guidance.bearing_rate_rad_s = 0.0f;
    } else {
        guidance.bearing_rate_rad_s = (guidance.bearing_error_rad - guidance.last_bearing_error_rad) / dt;
    }
    guidance.last_bearing_error_rad = guidance.bearing_error_rad;
    guidance.last_update_ms = now;

    // bearing guidance -> roll (bank-to-turn) command
    // proportional + rate damping + PNG rate term
    const float kp_roll = plane.ga_guidance.kp_roll.get();
    const float kd_roll = plane.ga_guidance.kd_roll.get();
    const float png_n   = plane.ga_guidance.png_n.get();
    float roll_cmd = kp_roll * guidance.bearing_error_rad
                   + kd_roll * guidance.bearing_rate_rad_s
                   + png_n * guidance.bearing_rate_rad_s;

    // elevation guidance -> pitch command (program dive + correction)
    const float kp_pitch = plane.ga_guidance.kp_pitch.get();
    const float dive_pitch_rad = radians(plane.ga_guidance.dive_pitch.get());
    float pitch_cmd = dive_pitch_rad + kp_pitch * guidance.elevation_error_rad;

    // attitude limits
    const float roll_lim_rad = radians(plane.ga_guidance.roll_lim.get());
    const float pitch_min_rad = radians(plane.ga_guidance.pitch_min.get());
    const float pitch_max_rad = radians(plane.ga_guidance.pitch_max.get());
    roll_cmd = constrain_float(roll_cmd, -roll_lim_rad, roll_lim_rad);
    pitch_cmd = constrain_float(pitch_cmd, pitch_min_rad, pitch_max_rad);

    // altitude floor safety: limit dive angle if too low
    if (plane.relative_altitude < plane.ga_guidance.min_alt.get() && is_positive(plane.relative_altitude)) {
        if (pitch_cmd < radians(-5.0f)) {
            pitch_cmd = radians(-5.0f);
        }
    }

    // manual RC override for safety / abort
    const float roll_stick = plane.channel_roll->norm_input_dz();
    const float pitch_stick = plane.channel_pitch->norm_input_dz();
    if (fabsf(roll_stick) > RC_OVERRIDE_DEADZONE || fabsf(pitch_stick) > RC_OVERRIDE_DEADZONE) {
        plane.nav_roll_cd = int32_t(roll_stick * plane.roll_limit_cd);
        const int32_t pitch_max_cd = int32_t(plane.aparm.pitch_limit_max.get() * 100.0f);
        const int32_t pitch_min_cd = int32_t(plane.pitch_limit_min * 100.0f);
        if (pitch_stick > 0.0f) {
            plane.nav_pitch_cd = int32_t(pitch_stick * pitch_max_cd);
        } else {
            plane.nav_pitch_cd = int32_t(pitch_stick * pitch_min_cd);
        }
        plane.nav_pitch_cd = constrain_int32(plane.nav_pitch_cd, pitch_min_cd, pitch_max_cd);
        return;
    }

    plane.nav_roll_cd = int32_t(degrees(roll_cmd) * 100.0f);
    plane.nav_pitch_cd = int32_t(degrees(pitch_cmd) * 100.0f);
}

void ModeFuchongceshi::handle_target_loss()
{
    // target lost: wings level, zero pitch
    plane.nav_roll_cd = 0;
    plane.nav_pitch_cd = 0;
    guidance.target_valid = false;
    guidance.last_update_ms = 0;

    if (loss_action_triggered) {
        return;
    }
    loss_action_triggered = true;

    const uint8_t action = plane.ga_guidance.loss_action.get();
    if (action == 1) {
        plane.gcs().send_text(MAV_SEVERITY_WARNING, "FUCHONGCESHI: target lost, switch to Loiter");
        plane.set_mode(plane.mode_loiter, ModeReason::GCS_COMMAND);
    } else if (action == 2) {
        plane.gcs().send_text(MAV_SEVERITY_WARNING, "FUCHONGCESHI: target lost, switch to RTL");
        plane.set_mode(plane.mode_rtl, ModeReason::GCS_COMMAND);
    }
    // action == 0: stay in FUCHONGCESHI with wings level
}

void ModeFuchongceshi::update()
{
    read_serial();
    update_guidance();
}

void ModeFuchongceshi::run()
{
    // run fixed-wing attitude controllers using nav_roll_cd / nav_pitch_cd
    Mode::run();

    // maximum throttle for high-speed collision pass
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, plane.aparm.throttle_max.get());
}

#endif  // HAL_QUADPLANE_ENABLED
