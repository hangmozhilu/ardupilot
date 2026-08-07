#include "mode.h"
#include "Plane.h"

#if HAL_QUADPLANE_ENABLED

#include <AP_SerialManager/AP_SerialManager.h>
#include <string.h>
#include <math.h>

/*
  ImgGuide flight mode for image-guided aerial target intercept.
  图像引导空中目标拦截模式。

  ======================== 总体设计说明 ========================

  本模式通过云台相机获取目标的像素坐标和云台框架角，由飞控
  计算目标相对于飞机的视线角（LOS），经过Alpha-Beta滤波器
  平滑和预测后，使用连续增益调度的比例导航（PNG）制导律，
  生成滚转和俯仰指令，控制飞机以最大速度穿过空中目标中心。

  核心原则：纯视觉伺服，不依赖GPS/高度/距离信息。
  所有控制决策基于LOS角度和角速率。

  连续增益调度策略：
    gain_scale = err_gain × rate_damp
    - err_gain: |bearing_error|大 → 增益大（快速对准）
    - rate_damp: |bearing_rate|大 → 增益小（目标接近，避免振荡）
    - 两者都小时 → 进入终端制导，限制滚转

  关键特性：
    1. 纯视觉伺服：像素→LOS→体轴姿态指令，闭环控制
    2. 连续增益调度：基于LOS误差和角速率，无高度/距离依赖
    3. 目标运动预测：Alpha-Beta滤波器平滑+预测，滤除噪声
    4. 图像去旋转：消除飞机滚转对像素坐标的影响
    5. 云台杆臂补偿：抵消飞机姿态变化引入的虚假LOS运动
    6. 滚转-偏航耦合补偿：方向舵混合，减小侧滑
    7. 安全保护：目标丢失超时、置信度滤波、遥控器超控

  ======================== 协议说明 ========================

  Gimbal/Camera binary protocol (configure the gimbal/camera UART to
  SERIALn_PROTOCOL = 50 (SerialProtocol_FuchongTarget)):

    [0xFA][0xAA][cx][cy][yaw][pitch][conf][active][XX][0xAF][0x55]

  Frame length: 23 bytes
    header (2)  : 0xFA, 0xAA   帧头
    cx (4)      : int32 little-endian, 目标x像素坐标(以图像中心为原点), Camera_x
    cy (4)      : int32 little-endian, 目标y像素坐标(以图像中心为原点), Camera_y
    yaw (4)     : int32 little-endian, 云台偏航角, 0.01度, +=右, Gimbal_y
    pitch (4)   : int32 little-endian, 云台俯仰角, 0.01度, +=上, Gimbal_x
    conf (1)    : uint8, 目标置信度, 0~100, Object_confidence
    active (1)  : uint8, 追踪模式: 0x00=无目标, 0x11=不追踪, 其他非零=追踪, Object_active
    XX (1)      : XOR of all bytes between header and tail (payload bytes)
    tail (2)    : 0xAF, 0x55   帧尾

  自动切换：Object_active为非零且非0x11 && Object_confidence>=70% 时，
  飞控自动从任意模式切换到 ImgGuide 模式。

  ======================== 坐标系说明 ========================

  像素坐标系  : x向右, y向下, 原点在图像中心
  归一化像素  : x∈[-1,1]右正, y∈[-1,1]上正
  机体坐标系  : X向前, Y向右, Z向下
  云台框架角  : 相对机体, 偏航右正, 俯仰上正
  LOS角(体轴) : 云台角 + 像素角 → 目标在机体坐标系下的方位/俯仰
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
ModeImgGuide::ModeImgGuide()
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
bool ModeImgGuide::_enter()
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
    guidance.last_update_ms = 0;
    guidance.target_valid = false;
    guidance.in_terminal_phase = false;

    // 重置两个Alpha-Beta滤波器
    // Reset both Alpha-Beta filters
    filt_bearing.reset();
    filt_elevation.reset();

    // 初始化姿态指令为水平
    // Initialize attitude commands to level
    plane.nav_roll_cd = 0;
    plane.nav_pitch_cd = 0;

    // 复位滚转/俯仰/偏航控制器积分器，避免从其他模式切入时积分饱和
    // Reset controllers to avoid integrator windup when switching from other modes
    reset_controllers();

    plane.gcs().send_text(MAV_SEVERITY_INFO, "ImgGuide: entered");
    return true;
}

// ============================================================
// 模式退出函数
// ============================================================
void ModeImgGuide::_exit()
{
    plane.gcs().send_text(MAV_SEVERITY_INFO, "ImgGuide: exited");
}

// ============================================================
// 初始化云台目标串口
// 根据SERIALn_PROTOCOL=50查找对应的UART设备并配置波特率
// ============================================================
void ModeImgGuide::init_uart()
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
        plane.gcs().send_text(MAV_SEVERITY_INFO, "ImgGuide: gimbal UART @ %lu baud", (unsigned long)baud);
    } else {
        plane.gcs().send_text(MAV_SEVERITY_WARNING, "ImgGuide: no gimbal UART found");
    }
    uart_initialised = true;
}

// ============================================================
// 校验接收到的帧数据
// 验证帧尾是否正确（0xAF 0x55），以及XOR校验和是否匹配
// ============================================================
bool ModeImgGuide::validate_frame(const uint8_t *frame) const
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
bool ModeImgGuide::parse_frame(const uint8_t *frame)
{
    // 解析各字段（小端序）
    // Parse all fields (little-endian)
    const int32_t cx   = int32_from_le(frame + 0);  // 目标x像素坐标(图像中心为原点) Camera_x
    const int32_t cy   = int32_from_le(frame + 4);  // 目标y像素坐标(图像中心为原点) Camera_y
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
                              "ImgGuide: cx=%d cy=%d gym=%d gpm=%d conf=%d act=0x%02X "
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
void ModeImgGuide::read_serial()
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
void ModeImgGuide::check_auto_switch()
{
    // 已在ImgGuide模式中，无需切换
    // Already in ImgGuide mode, no need to switch
    if (plane.control_mode == &plane.mode_img_guide) {
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
                              "ImgGuide: auto-switch triggered (conf=%.0f%%, active=0x%02X)",
                              (double)(target.confidence * 100.0f), target.object_active);
        plane.set_mode(plane.mode_img_guide, ModeReason::GCS_COMMAND);
    }
}

// ============================================================
// 检查原始目标是否有效（用于模式内制导更新）
// 条件：Object_active==0x22、置信度≥30%、未超时
// 注意：与should_auto_switch()的区别在于置信度阈值更低(30% vs 70%)，
// 因为一旦进入模式后，即使置信度下降也应继续追踪。
// ============================================================
bool ModeImgGuide::target_valid() const
{
    // 追踪模式标志必须为非零（AI模块检测到目标）
    // object_active==0x00 表示无目标，==0x11 表示不追踪，其他非零值均视为有效
    // Object_active must be non-zero (AI module detected a target)
    // 0x00=no target, 0x11=detected but not tracking, others=active tracking
    if (target.object_active == 0x00 || target.object_active == 0x11) {
        static uint32_t last_tv_print_ms = 0;
        const uint32_t now = AP_HAL::millis();
        if (now - last_tv_print_ms > 1000) {
            last_tv_print_ms = now;
            plane.gcs().send_text(MAV_SEVERITY_WARNING,
                "ImgGuide: TGT_INVALID active=0x%02X conf=%.0f%%",
                target.object_active, (double)(target.confidence * 100.0f));
        }
        return false;
    }
    // 置信度低于最小阈值（30%），认为检测不可靠
    // Confidence too low (< 30%), detection unreliable
    if (target.confidence < CONFIDENCE_MIN_VALID) {
        static uint32_t last_tv_print_ms = 0;
        const uint32_t now = AP_HAL::millis();
        if (now - last_tv_print_ms > 1000) {
            last_tv_print_ms = now;
            plane.gcs().send_text(MAV_SEVERITY_WARNING,
                "ImgGuide: TGT_INVALID low_conf=%.0f%% thresh=%.0f%%",
                (double)(target.confidence * 100.0f), (double)(CONFIDENCE_MIN_VALID * 100.0f));
        }
        return false;
    }
    // 数据超时：超过GA_TOUT_MS未收到新数据
    // Data timeout: no new data for longer than GA_TOUT_MS
    if (AP_HAL::millis() - target.last_update_ms > uint32_t(plane.ga_guidance.timeout_ms.get())) {
        static uint32_t last_tv_print_ms = 0;
        const uint32_t now = AP_HAL::millis();
        if (now - last_tv_print_ms > 1000) {
            last_tv_print_ms = now;
            const uint32_t elapsed = AP_HAL::millis() - target.last_update_ms;
            plane.gcs().send_text(MAV_SEVERITY_WARNING,
                "ImgGuide: TGT_INVALID timeout=%lums limit=%dms",
                (unsigned long)elapsed, (int)plane.ga_guidance.timeout_ms.get());
        }
        return false;
    }
    return true;
}

// ============================================================
// 检查是否满足自动切换到此模式的条件
// 条件：Object_active==0x22 && 置信度>=70%
// 在任意飞行模式中，read_serial()持续检查此条件。
// ============================================================
bool ModeImgGuide::should_auto_switch() const
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
void ModeImgGuide::handle_target_loss()
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
        plane.gcs().send_text(MAV_SEVERITY_WARNING, "ImgGuide: target lost, switch to Loiter");
        plane.set_mode(plane.mode_loiter, ModeReason::GCS_COMMAND);
    } else if (action == 2) {
        // 切换到RTL返航
        // Switch to RTL
        plane.gcs().send_text(MAV_SEVERITY_WARNING, "ImgGuide: target lost, switch to RTL");
        plane.set_mode(plane.mode_rtl, ModeReason::GCS_COMMAND);
    } else {
        // action == 0: 保持在ImgGuide模式，平飞等待目标恢复
        // action == 0: stay in ImgGuide mode, level flight while waiting
        plane.gcs().send_text(MAV_SEVERITY_INFO,
            "ImgGuide: target lost, holding level (loss_act=%d)", (int)action);
    }
}

// ============================================================
// 计算连续增益缩放系数
// 基于LOS方位误差和方位角速率，不依赖任何高度/距离信息。
//
// 连续增益调度公式：
//   gain_scale = err_gain × rate_damp
//
// err_gain (误差增益)：
//   当目标偏离画面中心较远时，需要快速、大幅度的滚转来对准。
//   err_gain = |bearing_error| / err_ref_rad，限制在 [0.3, 1.0]
//   误差 > err_ref → 增益=1.0（最大，快速对准）
//   误差 < err_ref → 增益线性缩小（精细跟踪）
//
// rate_damp (角速率阻尼)：
//   当LOS角速率大时，说明目标接近（相对运动快），
//   需要减小增益避免振荡和过冲。
//   rate_damp = 1.0 / (1.0 + |bearing_rate| / rate_ref_rad_s)
//   角速率=0 → rate_damp=1.0（无衰减）
//   角速率=rate_ref → rate_damp=0.5（减半）
//   角速率→∞ → rate_damp→0（大幅衰减）
//
// 最终增益 = 两者乘积，既保证远距快速对准，又保证近距稳定。
// ============================================================
float ModeImgGuide::compute_continuous_gain_scale(
    float bearing_error_rad,
    float bearing_rate_rad_s) const
{
    // 获取参数（转换为弧度）
    // Get parameters (convert to radians)
    const float err_ref_rad = radians(plane.ga_guidance.err_ref.get());
    const float rate_ref_rad_s = radians(plane.ga_guidance.rate_ref.get());

    // 防止参数为0导致除零，最小保护
    // Prevent division by zero, minimum protection
    const float err_ref_safe = MAX(err_ref_rad, radians(1.0f));
    const float rate_ref_safe = MAX(rate_ref_rad_s, radians(1.0f));

    // 误差增益：误差大→增益大，线性缩放，下限0.3
    // Error gain: larger error → larger gain, linear scaling, floor 0.3
    const float error_mag = fabsf(bearing_error_rad);
    float err_gain = error_mag / err_ref_safe;
    err_gain = constrain_float(err_gain, 0.3f, 1.0f);

    // 角速率阻尼：角速率大→增益小，防止末端振荡
    // Rate damping: larger rate → smaller gain, prevent terminal oscillation
    const float rate_mag = fabsf(bearing_rate_rad_s);
    const float rate_damp = 1.0f / (1.0f + rate_mag / rate_ref_safe);

    // 合成增益 = 误差增益 × 角速率阻尼
    // Combined gain = error gain × rate damping
    const float gain_scale = err_gain * rate_damp;

    // 每隔1秒打印增益调度细节，便于调参
    // Print gain scheduling details every 1s for tuning
    static uint32_t last_gs_print_ms = 0;
    const uint32_t now = AP_HAL::millis();
    if (now - last_gs_print_ms > 1000) {
        last_gs_print_ms = now;
        /*plane.gcs().send_text(MAV_SEVERITY_INFO,
            "ImgGuide: GAIN err=%.1fdeg rate=%.1fdeg/s err_gain=%.2f rate_damp=%.2f scale=%.2f",
            (double)degrees(error_mag), (double)degrees(rate_mag),
            (double)err_gain, (double)rate_damp, (double)gain_scale);*/
    }

    return gain_scale;
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
void ModeImgGuide::compute_gimbal_compensation(
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

    // 获取斜距估计值用于将线速度转换为角速度
    // 由于空中目标无法获取真实斜距，使用固定默认值
    // Get slant range estimate for converting linear velocity to angular rate
    // Since air target range is unknown, use a fixed default value
    const float range_m_default = 500.0f;  // 默认斜距500m作为近似

    // 方位角速率补偿：Y方向（右侧）速度分量引起
    // 注意符号：云台向右移动时，目标在图像中向左运动，补偿应为负
    // Note sign: when gimbal moves right, target appears left in image
    comp_bearing_rad_s = -vy / range_m_default;

    // 俯仰角速率补偿：主要是X方向（前向）速度分量引起
    // Elevation rate compensation: mainly from X (forward) velocity component
    // 注意：当前符号假设相机向前看。如果云台大幅俯仰，符号可能需要调整。
    // Note: current sign assumes camera looking forward. If gimbal has large pitch, sign may need adjustment.
    comp_elevation_rad_s = -vx / range_m_default;
}

// ============================================================
// 计算控制指令（混合PNG/TPN + 航迹角跟踪 + FOV保持 + 能量管理）
//
// 横向通道（方位制导）：
//   远距离/中距离：真比例导航 TPN
//     a_lat = N * V_c * sigma_dot_bearing
//     roll_cmd = atan2(a_lat, g)
//   终端阶段：纯P跟踪，限制±15°
//
// 纵向通道（俯仰制导）：
//   使用期望航迹角跟踪替代纯LOS俯仰误差映射：
//     gamma_desired = atan2(-alt_error, h_dist)
//     gamma_current = atan2(-v_down, v_ground)
//     pitch_cmd = kp * (gamma_desired - gamma_current) - kd * gamma_rate
//   并叠加LOS俯仰误差前馈，提高响应速度。
//
// FOV保持：目标接近视场边缘时主动拉回中心。
// 能量管理：限制组合负载，禁止同时大滚转+大俯仰导致失速。
// ============================================================
void ModeImgGuide::compute_guidance_commands(
    float filtered_bearing_rad,
    float filtered_bearing_rate_rad_s,
    float filtered_elevation_rad,
    float filtered_elevation_rate_rad_s,
    float gain_scale,
    bool in_terminal,
    float v_north, float v_east, float v_down,
    float nx, float ny,
    float &roll_cmd_rad,
    float &pitch_cmd_rad) const
{
    // 通用物理常量
    const float g = GRAVITY_MSS;

    // 计算地速和接近速度（用于TPN和航迹角）
    const float v_ground = sqrtf(v_north * v_north + v_east * v_east);
    const float v_squared = v_ground * v_ground + v_down * v_down;
    const float v_total = sqrtf(MAX(v_squared, 0.01f));

    // 当前航迹角：水平面内速度方向与bearing的关系
    // 这里用 LOS 角速率近似表示横向运动，TPN 直接产生横向加速度命令
    // Closing velocity 用沿 LOS 的速度分量近似（取负号因为 Vc 是距离缩短率）
    const float closing_speed = v_total;  // 简化处理：假设速度大致指向目标

    if (in_terminal) {
        // ---- 终端阶段：纯追踪，限制滚转，减半增益 ----
        // Terminal phase: pure pursuit, limited roll, halved gain

        // 滚转指令：纯P，限制±15°
        // Roll command: pure P, limited to ±15°
        const float term_kp_roll = 0.5f;
        roll_cmd_rad = term_kp_roll * filtered_bearing_rad;
        const float term_roll_lim_rad = radians(TERMINAL_ROLL_LIM_DEG);
        roll_cmd_rad = constrain_float(roll_cmd_rad, -term_roll_lim_rad, term_roll_lim_rad);

        // 俯仰指令：纯视觉伺服，减半增益
        // Pitch command: pure visual servoing, halved gain
        pitch_cmd_rad = 0.5f * filtered_elevation_rad;
    } else {
        // ---- 正常阶段：横向 TPN，纵向航迹角跟踪 ----
        // Normal phase: lateral TPN, longitudinal flight-path-angle tracking

        const float kp_roll = plane.ga_guidance.kp_roll.get();
        const float kd_roll = plane.ga_guidance.kd_roll.get();
        const float png_n   = plane.ga_guidance.png_n.get();
        const float kp_pitch_los = plane.ga_guidance.kp_pitch.get();
        const float kd_pitch_los = plane.ga_guidance.kd_pitch.get();
        const float ptch_kp = plane.ga_guidance.pitch_track_kp.get();
        const float ptch_kd = plane.ga_guidance.pitch_track_kd.get();

        // 从bearing_rate中减去飞机自身偏航角速率，避免飞机自身旋转污染PNG项
        // 飞机右转时gyro.z>0，目标在画面中左移bearing_rate<0
        // 关系：true_bearing_rate = bearing_rate + gyro.z
        // Subtract aircraft yaw rate from bearing_rate to avoid self-rotation
        // contaminating the PNG term. Aircraft right turn (gyro.z>0) causes target
        // to move left in image (bearing_rate<0), so true = bearing_rate + gyro.z
        const float true_bearing_rate = filtered_bearing_rate_rad_s + ahrs.get_gyro().z;

        // ---- 横向通道：TPN 产生横向加速度，再映射为滚转角 ----
        // Lateral channel: TPN generates lateral acceleration, mapped to roll angle
        // a_lat = N * V_c * sigma_dot, 方向垂直于 LOS，使 LOS 角速率归零
        // 由于 tailsitter 滚转直接控制横向加速度，用 atan(a_lat / g) 得到滚转命令
        const float a_lat_cmd = png_n * closing_speed * true_bearing_rate;

        // 保留原始 P+D 项作为方位粗对准，TPN 提供精确跟踪
        const float roll_p = kp_roll * filtered_bearing_rad;
        const float roll_d = kd_roll * true_bearing_rate;
        roll_cmd_rad = gain_scale * (roll_p + roll_d) + atan2f(a_lat_cmd, g);

        // ---- 滚转-俯仰解耦：目标显著高于/低于机头时，优先爬升/俯冲 ----
        // Roll-pitch decoupling: when target is significantly above/below,
        // reduce roll to prioritize climb/dive. A heavily rolled aircraft
        // cannot pitch effectively due to tilted lift vector.
        const float pitch_priority = constrain_float(
            fabsf(filtered_elevation_rad) / radians(PITCH_PRIORITY_REF_DEG), 0.0f, 1.0f);
        // 俯仰优先级越高，滚转衰减越多（最多衰减70%）
        roll_cmd_rad *= (1.0f - pitch_priority * PITCH_PRIORITY_ROLL_DERATE);

        // ---- 纵向通道：航迹角跟踪 + LOS俯仰误差前馈 ----
        // Longitudinal channel: flight path angle tracking + LOS elevation feedforward
        // 通过 GPS 速度计算当前航迹角 gamma_current
        // gamma 定义：速度矢量与水平面的夹角，向上为正
        float gamma_current = 0.0f;
        if (v_ground > 1.0f) {
            gamma_current = atan2f(-v_down, v_ground);
        }
        const float gamma_rate = -filtered_elevation_rate_rad_s;  // 近似航迹角变化率

        // 期望航迹角：从当前位置指向目标的方向
        // 这里使用体轴LOS俯仰角作为期望航迹角的近似（相机向前看）
        // 更精确的方案需要知道斜距，但纯视觉模式下不可用
        const float gamma_desired = filtered_elevation_rad;

        // 航迹角跟踪命令
        const float pitch_fpa_cmd = ptch_kp * (gamma_desired - gamma_current)
                                  - ptch_kd * gamma_rate;

        // LOS 俯仰误差前馈：直接对体轴LOS俯仰角响应，提高近距机动性
        const float pitch_los_cmd = kp_pitch_los * filtered_elevation_rad
                                  - kd_pitch_los * filtered_elevation_rate_rad_s;

        // 合成俯仰命令：以航迹角跟踪为主，LOS前馈为辅
        pitch_cmd_rad = gain_scale * pitch_fpa_cmd + 0.3f * pitch_los_cmd;
    }

    // ---- FOV保持控制器：目标接近视场边缘时主动拉回中心 ----
    // FOV keep controller: pull target back when near edge
    const float fov_margin = plane.ga_guidance.fov_margin.get();
    const float fov_gain = plane.ga_guidance.fov_gain.get();
    if (fov_margin > 0.0f && fov_gain > 0.0f) {
        const float margin_abs = fabsf(fov_margin);
        if (fabsf(nx) > margin_abs) {
            const float excess = fabsf(nx) - margin_abs;
            // 目标在右侧(nx>0)时，需要左滚转将其拉回，所以加负号
            roll_cmd_rad += -copysignf(fov_gain * excess, nx);
        }
        if (fabsf(ny) > margin_abs) {
            const float excess = fabsf(ny) - margin_abs;
            // 目标在下方(ny>0)时，需要抬头将其拉回，所以加负号
            pitch_cmd_rad += -copysignf(fov_gain * excess, ny);
        }
    }

    // ---- 能量管理：限制组合负载，避免同时大滚转+大俯仰导致失速 ----
    // Energy management: limit combined load factor
    const float tan_roll = tanf(fabsf(roll_cmd_rad));
    const float tan_pitch = tanf(fabsf(pitch_cmd_rad));
    const float combined_load = sqrtf(tan_roll * tan_roll + tan_pitch * tan_pitch);
    const float max_load = 1.5f;  // 最大等效过载，可参数化
    if (combined_load > max_load && combined_load > 0.001f) {
        const float scale = max_load / combined_load;
        roll_cmd_rad *= scale;
        pitch_cmd_rad *= scale;
    }
}

// ============================================================
// 主制导更新函数（核心）
// 完整流程：
//   1. 目标有效性检查
//   2. 像素坐标 → 归一化坐标 → 去旋转 → FOV角度 → 机体LOS角
//   3. Alpha-Beta滤波器平滑+预测
//   4. 杆臂补偿修正
//   5. 连续增益调度计算
//   6. 终端阶段判断
//   7. 控制指令计算
//   8. 姿态限幅
//   9. 遥控器超控检查
// ============================================================
void ModeImgGuide::update_guidance()
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
    // 坐标原点在图像中心，nx∈[-1,1]右正, ny∈[-1,1]上正
    const float nx = target.camera_x / (CAMERA_WIDTH_PX * 0.5f);
    const float ny = target.camera_y / (CAMERA_HEIGHT_PX * 0.5f);

    // ---- 第3步：图像去旋转（消除飞机滚转对像素坐标的影响） ----
    // Step 3: deskew image rotation due to aircraft roll
    // 飞机滚转时图像也会旋转，需要将像素坐标旋转回水平参考系。
    // 注意：这里要将"图像坐标系"旋转回"水平坐标系"，因此使用逆旋转矩阵。
    // 当前飞机 roll>0（右翼下沉）时，图像相对水平参考系顺时针旋转，
    // 需要将图像中的点逆时针旋转 -roll 才能回到水平参考系。
    // When aircraft rolls, the image rotates. Rotate pixels back to level frame.
    // We rotate the image frame by -roll to get back to the level frame.
    const float roll = ahrs.get_roll();
    const float cr = cosf(roll);  // cos(roll)
    const float sr = sinf(roll);  // sin(roll)
    // 逆旋转矩阵（图像坐标系 → 水平坐标系）：
    //  [nx_level] = [ cr   sr] [nx]
    //  [ny_level]   [-sr   cr] [ny]
    const float nx_level =  nx * cr + ny * sr;
    const float ny_level = -nx * sr + ny * cr;

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

    // ---- 第6步：Alpha-Beta滤波器平滑和预测 ----
    // Step 6: Alpha-Beta filter smoothing and prediction
    // 获取滤波器增益参数
    // Get filter gain parameters
    const float kf_alpha = plane.ga_guidance.kf_alpha.get();
    const float kf_beta  = plane.ga_guidance.kf_beta.get();

    // 对方位角误差和俯仰角误差分别进行滤波
    // 方位角和俯仰角是循环角度，启用角度环绕处理
    // Filter bearing error and elevation error independently
    // Bearing and elevation are circular angles, enable wrapping
    const float filtered_bearing   = filt_bearing.update(los_yaw_body, kf_alpha, kf_beta, dt, true);
    const float filtered_elevation = filt_elevation.update(los_pitch_body, kf_alpha, kf_beta, dt, true);

    // 从滤波器获取速度估计值（角速率）
    // Get velocity estimates from filters (angular rates)
    // 预测t_go秒后的LOS角（用于前馈补偿目标运动）
    // Predict LOS angles after t_go seconds (feedforward for target motion)
    // 角度预测启用环绕处理，避免预测值超出[-π, π]范围
    const float predicted_bearing   = filt_bearing.predict(T_GO_PREDICT_S, true);
    const float predicted_elevation = filt_elevation.predict(T_GO_PREDICT_S, true);
    const float filtered_bearing_rate   = filt_bearing.v_est;
    const float filtered_elevation_rate = filt_elevation.v_est;

    // 存储滤波后的制导状态
    // Store filtered guidance state
    guidance.bearing_error_rad   = filtered_bearing;
    guidance.bearing_rate_rad_s  = filtered_bearing_rate;
    guidance.elevation_error_rad = filtered_elevation;
    guidance.elevation_rate_rad_s = filtered_elevation_rate;

    // ---- 第7步：云台杆臂补偿 ----
    // Step 7: gimbal lever-arm compensation
    // 计算由飞机姿态角速度+云台偏移引起的虚假LOS角速率
    // Compute spurious LOS angular rates caused by aircraft rotation + gimbal offset
    float comp_bearing_rad_s = 0.0f;
    float comp_elevation_rad_s = 0.0f;
    compute_gimbal_compensation(comp_bearing_rad_s, comp_elevation_rad_s);

    // 从滤波后的角速率中减去杆臂补偿值，得到真实的目标LOS角速率
    // Subtract compensation from filtered rates to get true target LOS rates
    const float true_bearing_rate = filtered_bearing_rate - comp_bearing_rad_s;

    // ---- 第8步：连续增益调度 ----
    // Step 8: continuous gain scheduling
    // 基于LOS方位误差和方位角速率计算增益缩放系数
    // Compute gain scale from LOS bearing error and rate
    const float gain_scale = compute_continuous_gain_scale(
        filtered_bearing, true_bearing_rate);

    // ---- 第9步：终端制导阶段判断 ----
    // Step 9: determine terminal guidance phase
    // 当方位误差和角速率都小于阈值时，认为已接近目标，进入终端制导
    // Terminal when both bearing error and rate are below thresholds
    const float term_err_rad = radians(plane.ga_guidance.term_err.get());
    const float term_rate_rad_s = radians(plane.ga_guidance.term_rate.get());
    const bool was_terminal = guidance.in_terminal_phase;
    guidance.in_terminal_phase = (fabsf(filtered_bearing) < term_err_rad)
                               && (fabsf(true_bearing_rate) < term_rate_rad_s);

    // 终端阶段状态变化时打印提示
    // Print notification when terminal phase state changes
    if (guidance.in_terminal_phase && !was_terminal) {
        plane.gcs().send_text(MAV_SEVERITY_INFO,
            "ImgGuide: ->TERMINAL err=%.1fdeg rate=%.1fdeg/s",
            (double)degrees(filtered_bearing), (double)degrees(true_bearing_rate));
    } else if (!guidance.in_terminal_phase && was_terminal) {
        plane.gcs().send_text(MAV_SEVERITY_INFO,
            "ImgGuide: ->NORMAL err=%.1fdeg rate=%.1fdeg/s",
            (double)degrees(filtered_bearing), (double)degrees(true_bearing_rate));
    }

    // ---- 第10步：计算控制指令 ----
    // Step 10: compute control commands
    float roll_cmd_rad = 0.0f;
    float pitch_cmd_rad = 0.0f;

    // 获取 NED 速度，用于航迹角跟踪和 TPN 的接近速度计算
    // Get NED velocity for flight-path-angle tracking and closing velocity
    Vector3f vel_ned;
    bool have_vel = ahrs.get_velocity_NED(vel_ned);
    if (!have_vel) {
        vel_ned.zero();
    }

    compute_guidance_commands(
        predicted_bearing,          // 使用预测值（含运动补偿）
        true_bearing_rate,          // 使用杆臂补偿后的真实角速率
        predicted_elevation,        // 使用预测值（含运动补偿）
        filtered_elevation_rate,    // 俯仰角速率（用于阻尼）
        gain_scale,
        guidance.in_terminal_phase,
        vel_ned.x, vel_ned.y, vel_ned.z,  // NED 速度
        nx_level, ny_level,         // 去旋转后的归一化像素坐标，用于FOV保持
        roll_cmd_rad,
        pitch_cmd_rad);

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
        // 遥控器超控时打印提示
        // Print notification when RC override is active
        static uint32_t last_rcov_print_ms = 0;
        if (now - last_rcov_print_ms > 1000) {
            last_rcov_print_ms = now;
            plane.gcs().send_text(MAV_SEVERITY_WARNING,
                "ImgGuide: RC_OVERRIDE roll=%.2f pitch=%.2f", (double)roll_stick, (double)pitch_stick);
        }
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

    // ---- 综合调试打印：每200ms打印一次核心制导变量的中间值 ----
    // Comprehensive debug print: output key guidance variables every 200ms
    static uint32_t last_guid_print_ms = 0;
    if (now - last_guid_print_ms > 200) {
        last_guid_print_ms = now;
        // 格式：LOS(原始/滤波/预测) | 增益(误差/速率/合成) | 指令(滚转/俯仰) | 阶段
        // Format: LOS(raw/filtered/predicted) | gain(err/rate/combined) | cmd(roll/pitch) | phase
        plane.gcs().send_text(MAV_SEVERITY_INFO,
            "ImgGuide: LOS_y=%.1f/%.1f/%.1f LOS_p=%.1f/%.1f/%.1f "
            "gain_scale=%.2f term=%d "
            "cmd_roll=%.1f cmd_pitch=%.1f "
            "br=%.1f tbr=%.1f er=%.1f",
            // LOS_yaw: raw / filtered / predicted (deg)
            (double)degrees(los_yaw_body), (double)degrees(filtered_bearing), (double)degrees(predicted_bearing),
            // LOS_pitch: raw / filtered / predicted (deg)
            (double)degrees(los_pitch_body), (double)degrees(filtered_elevation), (double)degrees(predicted_elevation),
            // gain_scale, terminal_flag
            (double)gain_scale, (int)guidance.in_terminal_phase,
            // roll_cmd, pitch_cmd (deg)
            (double)degrees(roll_cmd_rad), (double)degrees(pitch_cmd_rad),
            // bearing_rate, true_bearing_rate, elevation_rate (deg/s)
            (double)degrees(filtered_bearing_rate), (double)degrees(true_bearing_rate),
            (double)degrees(filtered_elevation_rate));
    }
}

// ============================================================
// 主更新函数
// 每帧调用：读取串口数据 → 更新制导
// ============================================================
void ModeImgGuide::update()
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
void ModeImgGuide::run()
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
    } else {
        // 方向舵混合关闭时，显式将方向舵归零，避免残留上一模式的值
        // When rudder mix is off, explicitly zero rudder to avoid residual from previous mode
        output_rudder_and_steering(0.0f);
    }
}

#endif  // HAL_QUADPLANE_ENABLED