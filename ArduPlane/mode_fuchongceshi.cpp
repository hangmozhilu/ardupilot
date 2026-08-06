#include "mode.h"
#include "Plane.h"

#if HAL_QUADPLANE_ENABLED

#include <AP_SerialManager/AP_SerialManager.h>
#include <string.h>
#include <math.h>

/*
  FUCHONGCESHI flight mode for precision image-guided collision.
  精准图像引导碰撞模式。

  ======================== 总体设计说明 ========================

  本模式通过云台相机获取目标的像素坐标和云台框架角，由飞控
  计算目标相对于飞机的视线角（LOS），经过Alpha-Beta滤波器
  平滑和预测后，使用比例导航（PNG）和纯追踪（Pure Pursuit）
  混合制导律，生成滚转和俯仰指令，控制飞机以最大速度撞击目标。

  制导策略分层：
    [远距离] 比例导航 + 增益调度（增益大，快速对准）
    [中距离] 比例导航 + 增益调度（增益适中，稳定跟踪）
    [近距离] 纯追踪（限制滚转，防止过冲，保证命中）

  关键优化点（8项）：
    1. 斜距估计：利用高度差+俯视角几何估算，支撑剖面规划
    2. 增益调度：按斜距动态缩放kp_roll/kp_pitch/png_n
    3. 目标运动预测：Alpha-Beta滤波器平滑+预测，滤除噪声
    4. 分段制导：距离<TERM_RNG时切换到纯追踪+限制滚转
    5. 俯冲角自适应：atan(高度差/水平距离)，替代固定dive_pitch
    6. 滚转-偏航耦合补偿：方向舵混合，减小侧滑
    7. 云台杆臂补偿：抵消飞机姿态变化引入的虚假LOS运动
    8. 安全保护：目标丢失超时、置信度滤波、遥控器超控

  ======================== 协议说明 ========================

  Gimbal/Camera binary protocol (configure the gimbal/camera UART to
  SERIALn_PROTOCOL = 50 (SerialProtocol_FuchongTarget)):

    [0xFA][0xAA][cx][cy][yaw][pitch][conf][active][XX][0xAF][0x55]

  Frame length: 23 bytes
    header (2)  : 0xFA, 0xAA   帧头
    cx (4)      : int32 little-endian, 目标x像素坐标(0~640), Camera_x
    cy (4)      : int32 little-endian, 目标y像素坐标(0~320), Camera_y
    yaw (4)     : int32 little-endian, 云台偏航角, 0.01度, +=右, Gimbal_y
    pitch (4)   : int32 little-endian, 云台俯仰角, 0.01度, +=上, Gimbal_x
    conf (1)    : uint8, 目标置信度, 0~100, Object_confidence
    active (1)  : uint8, 追踪模式: 0x00=无目标, 0x11=不追踪, 其他非零=追踪, Object_active
    XX (1)      : XOR of all bytes between header and tail (payload bytes)
    tail (2)    : 0xAF, 0x55   帧尾

  自动切换：Object_active为非零且非0x11 && Object_confidence>=70% 时，
  飞控自动从任意模式切换到 FUCHONGCESHI 模式。

  ======================== 坐标系说明 ========================

  像素坐标系  : x向右(0→640), y向下(0→480)
  归一化像素  : x∈[-1,1]右正, y∈[-1,1]上正
  机体坐标系  : X向前, Y向右, Z向下
  云台框架角  : 相对机体, 偏航右正, 俯仰上正
  LOS角(体轴) : 云台角 + 像素角 → 目标在机体坐标系下的方位/俯仰
  LOS角(地轴) : LOS角(体轴) + 飞机姿态角
*/

// ============================================================
// 工具函数：小端序int32解析
// 从字节缓冲区读取4字节小端序整数，用于解析云台协议数据
// ============================================================
static inline int32_t int32_from_le(const uint8_t *b)
{
    return (int32_t)((uint32_t)b[0] |
                     ((uint32_t)b[1] << 8) |
                     ((uint32_t)b[2] << 16) |
                     ((uint32_t)b[3] << 24));
}

// ============================================================
// 构造函数
// 初始化所有成员变量为默认值，确保首次进入模式时状态干净
// ============================================================
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

// ============================================================
// 模式进入函数
// 初始化串口、重置所有状态、清空滤波器和制导数据
// ============================================================
bool ModeFuchongceshi::_enter()
{
    // 初始化串口：根据SERIALn_PROTOCOL=50查找目标云台UART
    // Initialize UART: find the gimbal UART configured with SERIALn_PROTOCOL=50
    init_uart();

    // 重置串口帧解析状态机
    // Reset frame parser state machine
    frame_idx = 0;
    parse_state = 0;
    loss_action_triggered = false;
    memset(frame_buffer, 0, sizeof(frame_buffer));

    // 重置原始目标数据
    // Reset raw target data
    target.camera_x = 0;
    target.camera_y = 0;
    target.gimbal_yaw_cdeg = 0;
    target.gimbal_pitch_cdeg = 0;
    target.confidence = 0;
    target.object_active = 0;
    target.last_update_ms = 0;

    // 重置制导状态（滤波后的数据）
    // Reset guidance state (filtered data)
    guidance.bearing_error_rad = 0;
    guidance.bearing_rate_rad_s = 0;
    guidance.elevation_error_rad = 0;
    guidance.elevation_rate_rad_s = 0;
    guidance.slant_range_m = 0;
    guidance.last_update_ms = 0;
    guidance.target_valid = false;
    guidance.in_terminal_phase = false;

    // 重置三个Alpha-Beta滤波器
    // Reset all three Alpha-Beta filters
    filt_bearing.reset();
    filt_elevation.reset();
    filt_range.reset();

    // 初始化姿态指令为水平
    // Initialize attitude commands to level
    plane.nav_roll_cd = 0;
    plane.nav_pitch_cd = 0;

    // 复位滚转/俯仰/偏航控制器积分器，避免从其他模式切入时积分饱和
    // Reset controllers to avoid integrator windup when switching from other modes
    reset_controllers();

    plane.gcs().send_text(MAV_SEVERITY_INFO, "FUCHONGCESHI: entered");
    return true;
}

// ============================================================
// 模式退出函数
// ============================================================
void ModeFuchongceshi::_exit()
{
    plane.gcs().send_text(MAV_SEVERITY_INFO, "FUCHONGCESHI: exited");
}

// ============================================================
// 初始化云台目标串口
// 根据SERIALn_PROTOCOL=50查找对应的UART设备并配置波特率
// ============================================================
void ModeFuchongceshi::init_uart()
{
    if (uart_initialised) {
        return;  // 已初始化，跳过 Already initialized
    }
    // 查找配置为SerialProtocol_FuchongTarget协议的串口
    // Find UART configured with SerialProtocol_FuchongTarget
    uart = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_FuchongTarget, 0);
    if (uart != nullptr) {
        // 获取对应串口的波特率配置，默认115200
        // Get baud rate, default to 115200
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

// ============================================================
// 校验接收到的帧数据
// 验证帧尾是否正确（0xAF 0x55），以及XOR校验和是否匹配
// ============================================================
bool ModeFuchongceshi::validate_frame(const uint8_t *frame) const
{
    // 验证帧尾：最后两个字节必须是 0xAF 0x55
    // Verify tail bytes
    if (frame[19] != 0xAF || frame[20] != 0x55) {
        return false;
    }

    // 对18字节载荷（帧头+数据+标志位，不含校验字节和帧尾）做XOR校验
    // XOR checksum over the 18 payload bytes (headers + data + flags)
    uint8_t crc = 0;
    for (uint8_t i = 0; i < 18; i++) {
        crc ^= frame[i];
    }
    return crc == frame[18];
}

// ============================================================
// 解析云台发送的图像目标数据帧
// 将字节流解析为结构化的目标信息（像素坐标、云台角度、置信度等）
// ============================================================
bool ModeFuchongceshi::parse_frame(const uint8_t *frame)
{
    // 解析各字段（小端序）
    // Parse all fields (little-endian)
    const int32_t cx   = int32_from_le(frame + 0);  // 目标x像素坐标 Camera_x, 0..640
    const int32_t cy   = int32_from_le(frame + 4);  // 目标y像素坐标 Camera_y, 0..320
    const int32_t gy   = int32_from_le(frame + 8);  // 云台偏航角 Gimbal_y, 0.01度, +右
    const int32_t gp   = int32_from_le(frame + 12); // 云台俯仰角 Gimbal_x, 0.01度, +上
    const uint8_t conf = frame[16];  // 目标置信度 Object_confidence, 0..100
    const uint8_t active = frame[17]; // 追踪模式 Object_active: 0x11=不追踪, 0x22=追踪

    // 同步打印像素偏差与飞机姿态角，方便在Mission Planner中调试制导效果
    // Print pixel deviation and aircraft attitude together for debugging in Mission Planner
    static uint32_t last_print_ms = 0;
    const uint32_t now = AP_HAL::millis();
    if (now - last_print_ms > 200) {
        last_print_ms = now;
        const float roll_deg  = degrees(ahrs.get_roll());
        const float pitch_deg = degrees(ahrs.get_pitch());
        const float yaw_deg   = degrees(ahrs.get_yaw());
        plane.gcs().send_text(MAV_SEVERITY_INFO,
                              "FUCHONGCESHI: cx=%d cy=%d gym=%d gpm=%d conf=%d act=0x%02X "
                              "roll=%.1f pitch=%.1f yaw=%.1f",
                              (int)cx, (int)cy, (int)gy, (int)gp, (int)conf, (int)active,
                              (double)roll_deg, (double)pitch_deg, (double)yaw_deg);
    }

    // 存储原始目标数据
    // Store raw target data
    target.camera_x = cx;
    target.camera_y = cy;
    target.gimbal_yaw_cdeg   = gy;
    target.gimbal_pitch_cdeg = gp;
    target.confidence = conf * 0.01f;  // 0..100 → 0.0..1.0
    target.object_active = active;
    return true;
}

// ============================================================
// 从串口读取数据并运行帧解析状态机
// 状态机：0→等待帧头0xFA, 1→等待帧头0xAA, 2→接收帧体21字节
// ============================================================
void ModeFuchongceshi::read_serial()
{
    if (uart == nullptr) {
        return;  // 串口未初始化，直接返回 No UART available
    }

    const uint32_t nbytes = uart->available();
    for (uint32_t i = 0; i < nbytes; i++) {
        const int16_t c = uart->read();
        if (c < 0) {
            continue;  // 读取失败，跳过 Read error, skip
        }

        // 帧解析状态机
        // Frame parsing state machine
        switch (parse_state) {
        case 0:
            // 状态0：等待第一个帧头字节 0xFA
            // State 0: waiting for first header byte 0xFA
            if (c == 0xFA) {
                parse_state = 1;
            }
            break;

        case 1:
            // 状态1：等待第二个帧头字节 0xAA
            // State 1: waiting for second header byte 0xAA
            if (c == 0xAA) {
                parse_state = 2;
                frame_idx = 0;  // 开始接收帧体 Start receiving frame body
            } else if (c != 0xFA) {
                // 不是有效帧头，重新扫描
                // Not a valid header, restart scan
                parse_state = 0;
            }
            // 如果是0xFA，保持在状态1（可能是连续的帧头）
            // If 0xFA, stay in state 1 (could be back-to-back headers)
            break;

        case 2:
            // 状态2：接收帧体，共21字节（18字节载荷+1字节校验+2字节帧尾）
            // State 2: receiving frame body, 21 bytes total
            frame_buffer[frame_idx++] = uint8_t(c);
            if (frame_idx >= FRAME_BODY_LEN) {
                // 帧体接收完毕，进行校验和解析
                // Frame body complete, validate and parse
                if (validate_frame(frame_buffer) && parse_frame(frame_buffer)) {
                    target.last_update_ms = AP_HAL::millis();  // 记录接收时间戳
                }
                parse_state = 0;  // 回到初始状态等待下一帧
                frame_idx = 0;
            }
            break;

        default:
            // 异常状态，重置
            // Unknown state, reset
            parse_state = 0;
            frame_idx = 0;
            break;
        }
    }
}

// ============================================================
// 在任意模式中持续检查串口，满足条件时自动切换到此模式
// 由 Plane::update_control_mode() 每帧调用。
// 条件：Object_active==0x22 && 置信度>=70%
// ============================================================
void ModeFuchongceshi::check_auto_switch()
{
    // 已在FUCHONGCESHI模式中，无需切换
    // Already in FUCHONGCESHI mode, no need to switch
    if (plane.control_mode == &plane.mode_fuchongceshi) {
        return;
    }

    // 懒初始化串口：首次调用时初始化UART
    // Lazy init UART on first call
    init_uart();

    if (uart == nullptr) {
        return;  // 无可用串口 No UART available
    }

    // 读取串口数据并解析帧
    // Read serial data and parse frames
    read_serial();

    // 检查是否满足自动切换条件
    // Check if auto-switch conditions are met
    if (should_auto_switch()) {
        plane.gcs().send_text(MAV_SEVERITY_INFO,
                              "FUCHONGCESHI: auto-switch triggered (conf=%.0f%%, active=0x%02X)",
                              (double)(target.confidence * 100.0f), target.object_active);
        plane.set_mode(plane.mode_fuchongceshi, ModeReason::GCS_COMMAND);
    }
}

// ============================================================
// 检查原始目标是否有效（用于模式内制导更新）
// 条件：Object_active==0x22、置信度≥30%、未超时
// 注意：与should_auto_switch()的区别在于置信度阈值更低(30% vs 70%)，
// 因为一旦进入模式后，即使置信度下降也应继续追踪。
// ============================================================
bool ModeFuchongceshi::target_valid() const
{
    // 追踪模式标志必须为非零（AI模块检测到目标）
    // object_active==0x00 表示无目标，==0x11 表示不追踪，其他非零值均视为有效
    // Object_active must be non-zero (AI module detected a target)
    // 0x00=no target, 0x11=detected but not tracking, others=active tracking
    if (target.object_active == 0x00 || target.object_active == 0x11) {
        return false;
    }
    // 置信度低于最小阈值（30%），认为检测不可靠
    // Confidence too low (< 30%), detection unreliable
    if (target.confidence < CONFIDENCE_MIN_VALID) {
        return false;
    }
    // 数据超时：超过GA_TOUT_MS未收到新数据
    // Data timeout: no new data for longer than GA_TOUT_MS
    if (AP_HAL::millis() - target.last_update_ms > uint32_t(plane.ga_guidance.timeout_ms.get())) {
        return false;
    }
    return true;
}

// ============================================================
// 检查是否满足自动切换到此模式的条件
// 条件：Object_active==0x22 && 置信度>=70%
// 在任意飞行模式中，read_serial()持续检查此条件。
// ============================================================
bool ModeFuchongceshi::should_auto_switch() const
{
    // 追踪模式标志必须为非零且非0x11（AI模块主动请求追踪）
    // object_active==0x00=无目标, 0x11=识别但明确不追踪
    // 0x01/0x22等其他非零值均视为追踪请求
    // Object_active must be non-zero and not 0x11 (AI module actively tracking)
    if (target.object_active == 0x00 || target.object_active == 0x11) {
        return false;
    }
    // 置信度必须≥70%，减少误触发
    // Confidence must be ≥70% to reduce false triggers
    if (target.confidence < CONFIDENCE_AUTO_SWITCH) {
        return false;
    }
    return true;
}

// ============================================================
// 目标丢失处理
// 根据GA_LOSS_ACT执行：0=保持平飞, 1=切Loiter, 2=切RTL
// ============================================================
void ModeFuchongceshi::handle_target_loss()
{
    // 目标丢失：平飞姿态
    // Target lost: level flight attitude
    plane.nav_roll_cd = 0;
    plane.nav_pitch_cd = 0;
    guidance.target_valid = false;
    guidance.last_update_ms = 0;
    guidance.in_terminal_phase = false;

    // 重置滤波器，避免收到新目标后使用过期估计值
    // Reset filters to avoid using stale estimates when target returns
    filt_bearing.reset();
    filt_elevation.reset();
    filt_range.reset();

    // 目标丢失动作只触发一次，避免反复切换模式
    // Loss action triggers only once to avoid repeated mode switching
    if (loss_action_triggered) {
        return;
    }
    loss_action_triggered = true;

    const uint8_t action = plane.ga_guidance.loss_action.get();
    if (action == 1) {
        // 切换到Loiter定点盘旋
        // Switch to Loiter
        plane.gcs().send_text(MAV_SEVERITY_WARNING, "FUCHONGCESHI: target lost, switch to Loiter");
        plane.set_mode(plane.mode_loiter, ModeReason::GCS_COMMAND);
    } else if (action == 2) {
        // 切换到RTL返航
        // Switch to RTL
        plane.gcs().send_text(MAV_SEVERITY_WARNING, "FUCHONGCESHI: target lost, switch to RTL");
        plane.set_mode(plane.mode_rtl, ModeReason::GCS_COMMAND);
    }
    // action == 0: 保持在FUCHONGCESHI模式，平飞等待目标恢复
    // action == 0: stay in FUCHONGCESHI mode, level flight while waiting
}

// ============================================================
// 计算当前飞机高于目标的高度差 (m)
// 使用当前绝对海拔高度减去目标海拔高度。
// 如果GA_TGT_ALT未设置(=0)，则默认使用home点海拔（假设目标在地面）。
// ============================================================
float ModeFuchongceshi::compute_height_above_target() const
{
    // 获取当前绝对海拔高度 (cm)
    // Get current absolute altitude (cm)
    int32_t current_alt_cm = 0;
    if (!plane.current_loc.get_alt_cm(Location::AltFrame::ABSOLUTE, current_alt_cm)) {
        // 获取失败，退回使用相对高度
        // Failed to get absolute altitude, fall back to relative altitude
        return plane.relative_altitude;
    }

    // 获取目标海拔高度 (m)
    // Get target altitude (m)
    float tgt_alt_m = plane.ga_guidance.tgt_alt.get();
    if (tgt_alt_m <= 0.0f) {
        // 未设置目标高度，使用home点海拔作为默认值
        // Target altitude not set, use home altitude as default
        int32_t home_alt_cm = 0;
        if (plane.home.get_alt_cm(Location::AltFrame::ABSOLUTE, home_alt_cm)) {
            tgt_alt_m = home_alt_cm * 0.01f;
        } else {
            // 无法获取home高度，假设目标在地面（相对高度=0）
            // Cannot get home altitude, assume target on ground
            tgt_alt_m = 0.0f;
        }
    }

    // 高度差 = 当前海拔 - 目标海拔 (m)
    // Height above target = current AMSL - target AMSL (m)
    const float height_above_target_m = current_alt_cm * 0.01f - tgt_alt_m;

    // 返回原始高度差（允许负值），以便目标高于飞机时生成爬升指令
    // Return raw signed height, allowing climb command when target is above aircraft
    return height_above_target_m;
}

// ============================================================
// 估计斜距（飞机到目标的直线距离）
// 利用高度差和地轴俯视角进行几何估算。
//
// 原理：在地轴坐标系中，目标视线与水平面的夹角为俯视角
//   depression_angle = -los_pitch_earth
//   slant_range = height_above_target / sin(depression_angle)
//
// 当俯视角很小时（几乎水平），sin≈0会导致数值不稳定，
// 此时使用水平距离代替斜距。
// ============================================================
float ModeFuchongceshi::estimate_slant_range(float los_pitch_earth_rad) const
{
    // 获取高于目标的高度差
    // Get height above target
    const float height_above_target_m = compute_height_above_target();

    // 地轴俯视角：正值=向上看，负值=向下看
    // Earth-frame pitch: positive = looking up, negative = looking down
    const float depression_angle_rad = -los_pitch_earth_rad;

    // 飞机低于目标或俯视角过小（< 2度），几何法不可靠，返回保守默认值
    // Below target or depression angle too small: geometry unreliable, use conservative fallback
    if (height_above_target_m <= 0.0f || depression_angle_rad < radians(2.0f)) {
        // 使用较大固定值，避免近距离增益误判
        // Use large fixed value to avoid close-range gain mis-scheduling
        return 5000.0f;
    }

    // 几何估算：斜距 = 高度差 / sin(俯视角)
    // Geometric estimation: slant_range = height / sin(depression_angle)
    float slant_range = height_above_target_m / sinf(depression_angle_rad);

    // 限幅：斜距不小于高度差，不大于合理上限（防止数值异常）
    // Clamp: slant range not less than height, not more than reasonable max
    slant_range = MAX(slant_range, height_above_target_m);
    slant_range = MIN(slant_range, 5000.0f);  // 最大5km

    return slant_range;
}

// ============================================================
// 计算增益缩放系数
// 根据斜距动态调整制导增益，实现"远距大增益→近距小增益"的调度。
//
// 缩放策略：
//   range >= GSC_RNG  → scale = 1.0  (基准增益)
//   range <  GSC_RNG  → scale = range / GSC_RNG (线性缩小，最小0.2)
//   range <  TERM_RNG → scale = 0.0  (纯追踪，无PNG)
// ============================================================
float ModeFuchongceshi::compute_gain_scale(float slant_range_m) const
{
    // 防止参数被设置为0导致除零，最小保护为1m
    const float gsc_range = MAX(plane.ga_guidance.gain_sched_range.get(), 1.0f);
    const float term_range = plane.ga_guidance.terminal_range.get();

    // 终端制导阶段：增益为0（纯追踪，不使用PNG）
    // Terminal phase: zero gain (pure pursuit, no PNG)
    if (slant_range_m < term_range) {
        return 0.0f;
    }

    // 距离大于调度参考值：使用基准增益
    // Range larger than scheduling reference: use base gains
    if (slant_range_m >= gsc_range) {
        return 1.0f;
    }

    // 线性缩放：scale = range / GSC_RNG，下限0.2
    // Linear scaling: scale = range / GSC_RNG, minimum 0.2
    float scale = slant_range_m / gsc_range;
    scale = MAX(scale, 0.2f);  // 避免增益过小导致无法跟踪
    return scale;
}

// ============================================================
// 计算云台杆臂补偿角速率
// 当云台安装位置不在飞机重心时，飞机姿态角速度会在云台处
// 产生额外的线速度，在LOS中表现为虚假的角运动。
//
// 补偿原理：
//   v_gimbal = omega(机体角速度) × r_offset(云台偏移量)
//   LOS角速率补偿 = v_gimbal在LOS垂直方向的分量 / 斜距
//
// 机体坐标系：X向前，Y向右，Z向下
// 角速度: p(滚转), q(俯仰), r(偏航)
// ============================================================
void ModeFuchongceshi::compute_gimbal_compensation(
    float &comp_bearing_rad_s,
    float &comp_elevation_rad_s) const
{
    comp_bearing_rad_s = 0.0f;
    comp_elevation_rad_s = 0.0f;

    // 获取云台安装偏移量 (m)
    // Get gimbal mounting offset (m)
    const float dx = plane.ga_guidance.gimbal_offset_x.get();  // 前向
    const float dy = plane.ga_guidance.gimbal_offset_y.get();  // 右侧
    const float dz = plane.ga_guidance.gimbal_offset_z.get();  // 下方

    // 如果所有偏移量都为0，说明云台在重心，无需补偿
    // If all offsets are zero, gimbal is at CG, no compensation needed
    if (is_zero(dx) && is_zero(dy) && is_zero(dz)) {
        return;
    }

    // 获取机体角速度 (rad/s): p=滚转, q=俯仰, r=偏航
    // Get body angular rates (rad/s): p=roll, q=pitch, r=yaw
    Vector3f gyro = ahrs.get_gyro();  // 机体角速度 rad/s

    // 计算云台处的诱导速度 (m/s): v = omega × r_offset
    // Compute induced velocity at gimbal position: v = omega × r_offset
    // vx = q*dz - r*dy
    // vy = r*dx - p*dz
    // vz = p*dy - q*dx
    const float vx = gyro.y * dz - gyro.z * dy;  // 前向速度分量
    const float vy = gyro.z * dx - gyro.x * dz;  // 右侧速度分量

    // 获取斜距用于将线速度转换为角速度
    // Get slant range to convert linear velocity to angular rate
    const float range_m = MAX(guidance.slant_range_m, 10.0f);  // 最小10m防止除零

    // 方位角速率补偿：Y方向（右侧）速度分量引起
    // 注意符号：云台向右移动时，目标在图像中向左运动，补偿应为负
    // Note sign: when gimbal moves right, target appears left in image
    comp_bearing_rad_s = -vy / range_m;

    // 俯仰角速率补偿：主要是X方向（前向）速度分量引起
    // Elevation rate compensation: mainly from X (forward) velocity component
    // 注意：向下看时，前向速度产生的角速率符号为负
    // Note: when looking down, forward velocity produces negative angular rate
    comp_elevation_rad_s = -vx / range_m;
}

// ============================================================
// 计算终端制导（纯追踪）的滚转和俯仰指令
// 适用条件：斜距 < GA_TERM_RNG
//
// 纯追踪策略：
//   - 滚转：方位角误差直接映射为滚转指令，增益较小
//   - 滚转限制在±15°以内，防止末端过冲
//   - 俯仰：直接对准目标俯角（自适应俯冲角），不做额外修正
//   - 不使用PNG速率项
// ============================================================
void ModeFuchongceshi::compute_terminal_guidance(
    float filtered_bearing_rad,
    float filtered_elevation_rad,
    float filtered_range_m,
    float &roll_cmd_rad,
    float &pitch_cmd_rad) const
{
    // 终端阶段滚转使用较小的P增益
    // Terminal phase uses smaller roll P gain
    const float term_kp_roll = 0.5f;  // 纯追踪滚转增益 pure pursuit roll gain

    // 滚转指令 = 增益 × 方位角误差
    // Roll command = gain × bearing error
    roll_cmd_rad = term_kp_roll * filtered_bearing_rad;
    const float term_roll_lim_rad = radians(TERMINAL_ROLL_LIM_DEG);
    roll_cmd_rad = constrain_float(roll_cmd_rad, -term_roll_lim_rad, term_roll_lim_rad);

    // 俯仰指令：使用自适应俯冲角，不做额外修正
    // Pitch command: use adaptive dive pitch, no additional correction
    // 传入地轴LOS俯仰角（体轴LOS + 飞机俯仰角），确保几何计算正确
    // Pass earth-frame LOS elevation (body-frame + aircraft pitch)
    const float los_pitch_earth_rad = filtered_elevation_rad + ahrs.get_pitch();
    const float adaptive_dive_rad = compute_adaptive_dive_pitch(
        filtered_range_m, los_pitch_earth_rad);

    pitch_cmd_rad = adaptive_dive_rad;
}

// ============================================================
// 计算自适应俯冲角
// 根据高度差和水平距离动态计算最优俯冲角，替代固定dive_pitch。
//
// 参数：
//   filtered_range_m: 滤波后的斜距 (m)
//   los_pitch_earth_rad: 地轴LOS俯仰角 (rad)，体轴LOS + 飞机俯仰角
//                        正值=目标在地平线上方，负值=目标在地平线下方
//
// 计算方法：
//   俯视角 = -los_pitch_earth_rad（地轴，正值=向下看）
//   水平距离 = 斜距 × cos(俯视角)
//   自适应俯冲角 = atan2(-高度差, 水平距离)
//
// 当水平距离很小时（几乎正上方），使用固定dive_pitch作为后备。
// ============================================================
float ModeFuchongceshi::compute_adaptive_dive_pitch(
    float filtered_range_m,
    float los_pitch_earth_rad) const
{
    // 获取高于目标的高度差
    // Get height above target
    const float height_above_target_m = compute_height_above_target();

    // 俯视角（地轴）：正值=向下看，负值=向上看
    // Depression angle (earth frame): positive = looking down, negative = looking up
    const float depression_angle_rad = -los_pitch_earth_rad;

    // 水平距离 = 斜距 × cos(俯视角)
    // Horizontal distance = slant_range × cos(depression_angle)
    const float horizontal_dist_m = filtered_range_m * cosf(depression_angle_rad);

    // 水平距离过小（< 10m），接近正上方，使用固定俯冲角
    // Horizontal distance too small, almost directly above, use fixed dive pitch
    if (horizontal_dist_m < 10.0f) {
        return radians(plane.ga_guidance.dive_pitch.get());
    }

    // 自适应俯冲角 = atan2(-高度差, 水平距离)
    // 结果为负值（向下俯冲）
    // Adaptive dive pitch = atan2(-height, horizontal_dist)
    // Result is negative (diving down)
    float adaptive_dive_rad = atan2f(-height_above_target_m, horizontal_dist_m);

    // 限制在参数设定的俯仰范围内
    // Clamp to parameter-specified pitch range
    const float pitch_min_rad = radians(plane.ga_guidance.pitch_min.get());
    const float pitch_max_rad = radians(plane.ga_guidance.pitch_max.get());
    adaptive_dive_rad = constrain_float(adaptive_dive_rad, pitch_min_rad, pitch_max_rad);

    return adaptive_dive_rad;
}

// ============================================================
// 计算正常制导（比例导航+增益调度）的滚转和俯仰指令
// 适用条件：斜距 >= GA_TERM_RNG
//
// 滚转通道（方位制导）：
//   roll_cmd = kp_roll * bearing_error   (比例项)
//            + kd_roll * bearing_rate    (阻尼项)
//            + png_n  * bearing_rate    (PNG超前项)
//   所有增益乘以缩放系数
//
// 俯仰通道（俯仰制导）：
//   pitch_cmd = adaptive_dive_pitch  (自适应俯冲基线)
//             + kp_pitch * elevation_error  (误差修正)
//   所有增益乘以缩放系数
// ============================================================
void ModeFuchongceshi::compute_normal_guidance(
    float filtered_bearing_rad,
    float filtered_bearing_rate_rad_s,
    float filtered_elevation_rad,
    float filtered_elevation_rate_rad_s,
    float filtered_range_m,
    float &roll_cmd_rad,
    float &pitch_cmd_rad) const
{
    // 获取基准增益
    // Get base gains
    const float kp_roll  = plane.ga_guidance.kp_roll.get();
    const float kd_roll  = plane.ga_guidance.kd_roll.get();
    const float png_n    = plane.ga_guidance.png_n.get();

    // 计算增益缩放系数（根据斜距动态调整）
    // Compute gain scaling factor (dynamic based on slant range)
    const float gain_scale = compute_gain_scale(filtered_range_m);

    // ---- 滚转通道（方位制导） ----
    // Roll channel (bearing guidance)
    // 比例项：角度误差越大，滚转越大
    // Proportional term: larger angle error → larger roll
    // 阻尼项：抑制滚转振荡
    // Damping term: suppress roll oscillation
    // PNG项：比例导航，超前跟踪目标运动（仅使用真实目标角速率）
    // PNG term: proportional navigation, leads target motion (uses true target rate only)

    // 从bearing_rate中减去飞机自身偏航角速率，避免飞机自身旋转污染PNG项
    // 飞机右转时gyro.z>0，目标在画面中左移bearing_rate<0
    // 关系：true_bearing_rate = bearing_rate + gyro.z
    // Subtract aircraft yaw rate from bearing_rate to avoid self-rotation
    // contaminating the PNG term. Aircraft right turn (gyro.z>0) causes target
    // to move left in image (bearing_rate<0), so true = bearing_rate + gyro.z
    const float true_bearing_rate = filtered_bearing_rate_rad_s + ahrs.get_gyro().z;

    // 滚转指令（正常PNG制导）
    // 比例项：方位角误差越大，滚转越大
    // 阻尼+PNG项：使用补偿后的真实角速率（已减去飞机自身旋转）
    // Roll command (normal PNG guidance)
    roll_cmd_rad = gain_scale * (kp_roll * filtered_bearing_rad
                               + kd_roll * true_bearing_rate
                               + png_n   * true_bearing_rate);

    // ---- 俯仰通道（俯仰制导） ----
    // Pitch channel (elevation guidance)
    // 自适应俯冲基线：根据几何关系计算最优俯冲角
    // Adaptive dive baseline: compute optimal dive angle from geometry
    // 地轴LOS俯仰角 = 机体LOS俯仰角 + 飞机俯仰角
    // Earth-frame LOS elevation = body-frame LOS + aircraft pitch
    const float los_pitch_earth_rad = filtered_elevation_rad + ahrs.get_pitch();
    const float adaptive_dive_rad = compute_adaptive_dive_pitch(
        filtered_range_m, los_pitch_earth_rad);

    // 俯仰指令 = 自适应俯冲基线（几何基准，直接使用）
    // 不再叠加 filtered_elevation_rad，避免与 adaptive_dive 双重计数导致抬头
    // Pitch command = adaptive dive baseline only (geometric reference)
    // No longer adds kp_pitch * filtered_elevation_rad to avoid double-counting
    pitch_cmd_rad = adaptive_dive_rad;
}

// ============================================================
// 主制导更新函数（核心）
// 完整流程：
//   1. 目标有效性检查
//   2. 像素坐标 → 归一化坐标 → 去旋转 → 角度
//   3. 云台角 + 像素角 → 机体LOS角
//   4. 斜距估计（高度差+俯视角几何法）
//   5. Alpha-Beta滤波器平滑+预测
//   6. 杆臂补偿修正
//   7. 分段制导（正常PNG 或 终端纯追踪）
//   8. 姿态限幅
//   9. 遥控器超控检查
//   10. 方向舵耦合补偿
// ============================================================
void ModeFuchongceshi::update_guidance()
{
    const uint32_t now = AP_HAL::millis();

    // ---- 第1步：检查目标是否有效 ----
    // Step 1: check if target is valid
    if (!target_valid()) {
        handle_target_loss();
        return;
    }

    // 目标恢复，清除丢失动作触发标志
    // Target recovered, clear loss action trigger flag
    loss_action_triggered = false;
    guidance.target_valid = true;

    // ---- 第2步：像素坐标归一化到 [-1, 1] ----
    // Step 2: normalize pixel coordinates to [-1, 1]
    // x: 0→640 映射到 -1→1（中心为0）
    // y: 0→320 映射到 1→-1（像素y向下，归一化y向上为正）
    const float nx = target.camera_x / (CAMERA_WIDTH_PX * 0.5f);
    const float ny = target.camera_y / (CAMERA_HEIGHT_PX * 0.5f);

    // ---- 第3步：图像去旋转（消除飞机滚转对像素坐标的影响） ----
    // Step 3: deskew image rotation due to aircraft roll
    // 飞机滚转时图像也会旋转，需要将像素坐标旋转回水平参考系
    // When aircraft rolls, the image rotates. Rotate pixels back to level frame.
    const float roll = ahrs.get_roll();
    const float cr = cosf(roll);  // cos(roll)
    const float sr = sinf(roll);  // sin(roll)
    // 旋转矩阵： [nx_level] = [cr  -sr] [nx]
    //            [ny_level]   [sr   cr] [ny]
    const float nx_level = nx * cr - ny * sr;
    const float ny_level = nx * sr + ny * cr;

    // ---- 第4步：像素偏差 → 角度偏差（利用相机FOV） ----
    // Step 4: pixel error → angle error (using camera FOV)
    // 使用 atan(nx * tan(hfov/2)) 精确映射，避免大FOV边缘线性近似误差
    const float hfov_rad = radians(plane.ga_guidance.hfov.get());
    const float vfov_rad = radians(plane.ga_guidance.vfov.get());
    const float pixel_yaw   = atanf(nx_level * tanf(hfov_rad * 0.5f));   // 水平像素→偏航角
    const float pixel_pitch = atanf(ny_level * tanf(vfov_rad * 0.5f));   // 垂直像素→俯仰角

    // ---- 第5步：云台框架角 + 像素角 → 机体LOS角 ----
    // Step 5: gimbal angles + pixel angles → body-frame LOS angles
    // 云台角（相对机体） + 像素角（相对光轴） = 目标在机体坐标系下的视线角
    // Gimbal angle (relative to body) + pixel angle (relative to optical axis)
    // = target LOS angle in body frame
    const float los_yaw_body   = radians(target.gimbal_yaw_cdeg * 0.01f) + pixel_yaw;
    const float los_pitch_body = radians(target.gimbal_pitch_cdeg * 0.01f) + pixel_pitch;

    // 计算时间步长：用于滤波器更新和角速率计算
    // Compute time step: for filter update and angular rate calculation
    float dt = 0.0f;
    if (guidance.last_update_ms != 0) {
        dt = (now - guidance.last_update_ms) * 0.001f;
    }
    guidance.last_update_ms = now;

    // ---- 第6步：斜距估计（高度差 + 俯视角几何法） ----
    // Step 6: slant range estimation (height + depression angle geometry)
    // 机体LOS角 + 飞机俯仰角 = 地轴LOS俯仰角
    // Body LOS pitch + aircraft pitch = earth-frame LOS pitch
    const float los_pitch_earth = los_pitch_body + ahrs.get_pitch();
    // 利用地轴俯视角和高度差估算斜距
    // Estimate slant range from earth-frame depression angle and height above target
    const float raw_slant_range_m = estimate_slant_range(los_pitch_earth);

    // ---- 第7步：Alpha-Beta滤波器平滑和预测 ----
    // Step 7: Alpha-Beta filter smoothing and prediction
    // 获取滤波器增益参数
    // Get filter gain parameters
    const float kf_alpha = plane.ga_guidance.kf_alpha.get();
    const float kf_beta  = plane.ga_guidance.kf_beta.get();

    // 对方位角误差、俯仰角误差、斜距分别进行滤波
    // 方位角和俯仰角是循环角度，启用角度环绕处理
    // Filter bearing error, elevation error, and slant range independently
    // Bearing and elevation are circular angles, enable wrapping
    const float filtered_bearing   = filt_bearing.update(los_yaw_body, kf_alpha, kf_beta, dt, true);
    const float filtered_elevation = filt_elevation.update(los_pitch_body, kf_alpha, kf_beta, dt, true);
    const float filtered_range     = filt_range.update(raw_slant_range_m, kf_alpha, kf_beta, dt);

    // 从滤波器获取速度估计值（角速率、距离速率）
    // Get velocity estimates from filters (angular rates, range rate)
    // 预测t_go秒后的LOS角（用于前馈补偿目标运动）
    // Predict LOS angles after t_go seconds (feedforward for target motion)
    const float predicted_bearing   = filt_bearing.predict(T_GO_PREDICT_S);
    const float predicted_elevation = filt_elevation.predict(T_GO_PREDICT_S);
    const float filtered_bearing_rate   = filt_bearing.v_est;
    const float filtered_elevation_rate = filt_elevation.v_est;

    // 存储滤波后的制导状态
    // Store filtered guidance state
    guidance.bearing_error_rad   = filtered_bearing;
    guidance.bearing_rate_rad_s  = filtered_bearing_rate;
    guidance.elevation_error_rad = filtered_elevation;
    guidance.elevation_rate_rad_s = filtered_elevation_rate;
    guidance.slant_range_m       = filtered_range;

    // ---- 第8步：云台杆臂补偿 ----
    // Step 8: gimbal lever-arm compensation
    // 计算由飞机姿态角速度+云台偏移引起的虚假LOS角速率
    // Compute spurious LOS angular rates caused by aircraft rotation + gimbal offset
    float comp_bearing_rad_s = 0.0f;
    float comp_elevation_rad_s = 0.0f;
    compute_gimbal_compensation(comp_bearing_rad_s, comp_elevation_rad_s);

    // 从滤波后的角速率中减去杆臂补偿值，得到真实的目标LOS角速率
    // Subtract compensation from filtered rates to get true target LOS rates
    const float true_bearing_rate   = filtered_bearing_rate - comp_bearing_rad_s;
    const float true_elevation_rate = filtered_elevation_rate - comp_elevation_rad_s;

    // ---- 第9步：判断终端制导阶段 ----
    // Step 9: determine terminal guidance phase
    const float term_range = plane.ga_guidance.terminal_range.get();
    guidance.in_terminal_phase = (filtered_range < term_range);

    // ---- 第10步：分段制导计算 ----
    // Step 10: segmented guidance computation
    float roll_cmd_rad = 0.0f;
    float pitch_cmd_rad = 0.0f;

    if (guidance.in_terminal_phase) {
        // 【终端制导】斜距 < GA_TERM_RNG：使用纯追踪
        // [Terminal guidance] range < GA_TERM_RNG: pure pursuit
        compute_terminal_guidance(
            predicted_bearing,   // 使用预测值（含运动补偿）
            predicted_elevation, // 使用预测值（含运动补偿）
            filtered_range,      // 使用滤波后的斜距
            roll_cmd_rad,
            pitch_cmd_rad);
    } else {
        // 【正常制导】斜距 >= GA_TERM_RNG：使用比例导航+增益调度
        // [Normal guidance] range >= GA_TERM_RNG: PNG + gain scheduling
        compute_normal_guidance(
            predicted_bearing,
            true_bearing_rate,   // 使用杆臂补偿后的真实角速率
            predicted_elevation,
            true_elevation_rate, // 使用杆臂补偿后的真实角速率
            filtered_range,
            roll_cmd_rad,
            pitch_cmd_rad);
    }

    // ---- 第11步：姿态限幅 ----
    // Step 11: attitude limiting
    // 滚转限制
    const float roll_lim_rad = radians(plane.ga_guidance.roll_lim.get());
    roll_cmd_rad = constrain_float(roll_cmd_rad, -roll_lim_rad, roll_lim_rad);

    // 俯仰限制
    const float pitch_min_rad = radians(plane.ga_guidance.pitch_min.get());
    const float pitch_max_rad = radians(plane.ga_guidance.pitch_max.get());
    pitch_cmd_rad = constrain_float(pitch_cmd_rad, pitch_min_rad, pitch_max_rad);

    // ---- 第12步：遥控器手动超控检测 ----
    // Step 12: manual RC override detection
    // 当飞行员操作摇杆超过死区时，切换为手动控制，用于紧急避险
    // When pilot moves sticks beyond deadzone, switch to manual control for emergency
    const float roll_stick = plane.channel_roll->norm_input_dz();
    const float pitch_stick = plane.channel_pitch->norm_input_dz();
    if (fabsf(roll_stick) > RC_OVERRIDE_DEADZONE || fabsf(pitch_stick) > RC_OVERRIDE_DEADZONE) {
        // 遥控器滚转指令
        // RC roll command
        plane.nav_roll_cd = int32_t(roll_stick * plane.roll_limit_cd);

        // 遥控器俯仰指令
        // RC pitch command
        const int32_t pitch_max_cd = int32_t(plane.aparm.pitch_limit_max.get() * 100.0f);
        const int32_t pitch_min_cd = int32_t(plane.pitch_limit_min * 100.0f);
        if (pitch_stick > 0.0f) {
            plane.nav_pitch_cd = int32_t(pitch_stick * pitch_max_cd);
        } else {
            plane.nav_pitch_cd = int32_t(pitch_stick * pitch_min_cd);
        }
        plane.nav_pitch_cd = constrain_int32(plane.nav_pitch_cd, pitch_min_cd, pitch_max_cd);
        return;  // 遥控器超控时，不执行自动制导指令
    }

    // ---- 第13步：输出最终姿态指令 ----
    // Step 13: output final attitude commands
    // 转换为厘度（centidegrees）格式，ArduPilot控制回路使用此单位
    // Convert to centidegrees format used by ArduPilot control loops
    plane.nav_roll_cd = int32_t(degrees(roll_cmd_rad) * 100.0f);
    plane.nav_pitch_cd = int32_t(degrees(pitch_cmd_rad) * 100.0f);
}

// ============================================================
// 主更新函数
// 每帧调用：读取串口数据 → 更新制导
// ============================================================
void ModeFuchongceshi::update()
{
    // 读取串口中的云台目标数据帧
    // Read gimbal target data frames from serial
    read_serial();

    // 执行制导计算（目标有效性检查 + LOS计算 + 滤波 + 控制律）
    // Execute guidance computation (validity check + LOS + filtering + control law)
    update_guidance();
}

// ============================================================
// 执行函数（姿态控制）
// 使用固定翼姿态控制器执行nav_roll_cd/nav_pitch_cd指令。
// 最大油门用于高速撞击。
// 输出方向舵混合以补偿滚转-偏航耦合。
// ============================================================
void ModeFuchongceshi::run()
{
    // 执行固定翼姿态控制器：使用nav_roll_cd和nav_pitch_cd作为目标姿态
    // Run fixed-wing attitude controllers using nav_roll_cd and nav_pitch_cd
    Mode::run();

    // 最大油门：确保以最大速度撞击目标
    // Full throttle: ensure maximum speed at impact
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, plane.aparm.throttle_max.get());

    // ---- 滚转-偏航耦合补偿（方向舵混合） ----
    // Roll-yaw coupling compensation (rudder mixing)
    // 滚转时飞机产生侧滑，降低航向跟踪精度。
    // 通过方向舵混合（Aileron→Rudder Interconnect, ARI）补偿：
    // 方向舵偏转 = 滚转指令 × GA_RUD_MIX
    // 正值=右滚转→右方向舵，帮助协调转弯，减小侧滑。
    //
    // During roll, aircraft sideslips, reducing heading tracking accuracy.
    // Compensate via rudder mixing (Aileron-Rudder Interconnect):
    // rudder_deflection = roll_cmd × GA_RUD_MIX
    // Positive = right roll → right rudder, helps coordinate turn, reduces sideslip.
    const float rudder_mix = plane.ga_guidance.rudder_mix.get();
    if (rudder_mix > 0.0f) {
        // 获取当前滚转指令（归一化到[-1, 1]）
        // Get current roll command (normalized to [-1, 1])
        const float roll_lim = plane.ga_guidance.roll_lim.get();
        float roll_norm = 0.0f;
        if (roll_lim > 0.0f) {
            roll_norm = constrain_float(plane.nav_roll_cd * 0.01f / roll_lim, -1.0f, 1.0f);
        }
        // output_rudder_and_steering 期望归一化输入 [-1, 1]
        const float rudder_output = roll_norm * rudder_mix;
        output_rudder_and_steering(rudder_output);
    }
}

#endif  // HAL_QUADPLANE_ENABLED