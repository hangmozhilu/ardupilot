#include "mode_satguid.h"
#include "Plane.h"

#if HAL_QUADPLANE_ENABLED

#include <AP_SerialManager/AP_SerialManager.h>
#include <GCS_MAVLink/GCS.h>
#include <cmath>

// ============================================================
// 一、串口目标帧解析辅助函数
// ============================================================

/**
 * @brief 从小端字节序缓冲区中解析 int32 值
 *
 * Fuchong 串口帧采用小端格式存储经纬高及速度分量，
 * 本函数将 4 个字节还原为有符号 32 位整数。
 */
static inline int32_t int32_from_le(const uint8_t *b)
{
    return (int32_t)((uint32_t)b[0] |
                     ((uint32_t)b[1] << 8) |
                     ((uint32_t)b[2] << 16) |
                     ((uint32_t)b[3] << 24));
}

// ============================================================
// 二、可选调试输出开关
// ============================================================

// 将下面的 #if 0 改为 #if 1 即可在 GCS 上看到详细调试信息。
// 建议仅在仿真或地面排故时开启，正式飞行关闭以避免占用遥测带宽。
#if 0
#define SATGUID_DBG(fmt, ...) plane.gcs().send_text(MAV_SEVERITY_DEBUG, "SATGUID: " fmt, ##__VA_ARGS__)
#else
#define SATGUID_DBG(fmt, ...) ((void)0)
#endif

// ============================================================
// 三、构造函数与成员初始化
// ============================================================

/**
 * @brief SATGUID 模式构造函数
 *
 * 初始状态设为 TAKEOFF，各类标志位清零。
 * 注意： target / guidance / repos 为非平凡结构体，
 * 必须使用值初始化 {} 而不是 memset，否则某些编译器会报错。
 */
ModeSatGuid::ModeSatGuid()
    : state(State::TAKEOFF),
      corridor_phase(0.0f),
      takeoff_start_alt_cm(0),
      climb_target_rel_m(0.0f),
      target_loss_loiter_init(false),
      uart(nullptr),
      uart_initialised(false),
      frame_idx(0),
      parse_state(0)
{
    // 帧缓冲可以安全清零
    memset(frame_buffer, 0, sizeof(frame_buffer));
    // 值初始化其余结构体成员
    target = {};
    guidance = {};
    repos = {};
}

// ============================================================
// 四、模式进入与退出
// ============================================================

/**
 * @brief 进入 SATGUID 模式时的初始化
 *
 * 主要工作：
 * 1. 根据 SGUID_TGT_SRC 初始化串口或参数目标；
 * 2. 记录起飞/解锁点绝对高度（takeoff_start_alt_cm）；
 * 3. 硬性检查：若目标水平距离 < 100m 则拒绝进入，避免在目标正上方进入；
 * 4. 根据当前是 VTOL、过渡态还是固定翼，选择初始阶段。
 */
bool ModeSatGuid::_enter()
{
    // 仅在串口目标源时才需要初始化 UART
    if (plane.sat_guid_guidance.tgt_src.get() == 1) {
        init_uart();
    }

    // 重置内部状态，确保每次进入都是干净的起始状态
    corridor_phase = 0.0f;
    climb_target_rel_m = 0.0f;
    target = {};
    guidance = {};
    repos = {};
    target_loss_loiter_init = false;

    // 若目标源为参数，立即加载参数坐标，供后续阶段决策使用
    if (plane.sat_guid_guidance.tgt_src.get() == 0) {
        if (load_param_target()) {
            target.last_update_ms = AP_HAL::millis();
        } else {
            plane.gcs().send_text(MAV_SEVERITY_WARNING, "SATGUID: target coordinates not set or invalid");
        }
    }

    // 起飞高度基准取 home 点绝对高度（即解锁/起飞点海拔）。
    // 若 home 尚未就绪，则退而求其次使用当前绝对高度。
    takeoff_start_alt_cm = plane.home.alt;
    if (takeoff_start_alt_cm == 0) {
        int32_t current_alt_cm = 0;
        if (plane.current_loc.get_alt_cm(Location::AltFrame::ABSOLUTE, current_alt_cm)) {
            takeoff_start_alt_cm = current_alt_cm;
        }
    }

    // 硬性保护：若有效目标水平距离小于 100m，拒绝进入模式。
    // 这是因为在目标正上方或极近距离，固定翼制导无法建立有效俯冲航线。
    if (target_valid()) {
        compute_guidance();
        if (guidance.distance_m < 100.0f) {
            plane.gcs().send_text(MAV_SEVERITY_WARNING,
                                  "SATGUID: target too close (%.1f m), cannot enter",
                                  (double)guidance.distance_m);
            return false;
        }
    }

    // 根据当前飞行构型决定初始阶段：
    // - VTOL 模式：从垂直起飞开始；
    // - 前飞过渡态：等待过渡完成；
    // - 已固定翼：跳过 VTOL 起飞，根据距离决定爬升或重定位。
    if (plane.quadplane.in_vtol_mode()) {
        state = State::TAKEOFF;
    } else if (plane.quadplane.in_frwd_transition()) {
        state = State::TRANSITION;
    } else {
        if (target_valid()) {
            if (guidance.distance_m < MIN_DIVE_DISTANCE_M) {
                // 水平距离太近，无法直接建立俯冲，先重定位
                state = State::REPOSITION;
                plane.gcs().send_text(MAV_SEVERITY_INFO,
                                      "SATGUID: too close to target (%.1f m), reposition first",
                                      (double)guidance.distance_m);
            } else {
                state = State::CLIMB;
                select_climb_target();
                plane.gcs().send_text(MAV_SEVERITY_INFO, "SATGUID: entered in fixed-wing, skip takeoff");
            }
        } else {
            state = State::CLIMB;
            climb_target_rel_m = float(plane.sat_guid_guidance.cruise_alt.get());
            plane.gcs().send_text(MAV_SEVERITY_INFO, "SATGUID: entered in fixed-wing, skip takeoff");
        }
    }

    // 初始导航命令清零，避免上一切模式的残余指令造成瞬态
    plane.nav_roll_cd = 0;
    plane.nav_pitch_cd = 0;

    // SATGUID 是完全自主模式，不需要等待飞行员油门
    quadplane.throttle_wait = false;

    plane.gcs().send_text(MAV_SEVERITY_INFO, "SATGUID: entered");
    return true;
}

/**
 * @brief 退出 SATGUID 模式
 */
void ModeSatGuid::_exit()
{
    plane.gcs().send_text(MAV_SEVERITY_INFO, "SATGUID: exited");
}

// ============================================================
// 五、串口目标接收与解析
// ============================================================

/**
 * @brief 初始化 Fuchong 目标串口
 *
 * 通过 AP_SerialManager 查找 SerialProtocol_FuchongTarget，
 * 若未配置波特率则默认使用 115200。
 */
void ModeSatGuid::init_uart()
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
        plane.gcs().send_text(MAV_SEVERITY_INFO, "SATGUID: target UART @ %lu baud", (unsigned long)baud);
    } else {
        plane.gcs().send_text(MAV_SEVERITY_WARNING, "SATGUID: no target UART found");
    }
    uart_initialised = true;
}

/**
 * @brief 校验一帧 Fuchong 数据是否完整
 *
 * 校验规则：
 * - 帧长至少 21 字节；
 * - 末尾两字节为固定尾码 FRAME_TAIL_1 / FRAME_TAIL_2；
 * - 帧头到校验位之前所有字节异或值应等于倒数第三字节的校验字节。
 */
bool ModeSatGuid::validate_frame(const uint8_t *frame, uint8_t len) const
{
    if (len < 21) {
        return false;
    }
    // 验证帧尾
    if (frame[len - 2] != FRAME_TAIL_1 || frame[len - 1] != FRAME_TAIL_2) {
        return false;
    }
    // 计算校验字节：除校验位本身和尾码外所有字节异或
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len - 3; i++) {
        crc ^= frame[i];
    }
    return crc == frame[len - 3];
}

/**
 * @brief 解析 GPS 类型目标帧（帧类型 0x01）
 *
 * 帧格式：类型(1B) + 纬度(4B) + 经度(4B) + 高度(4B) + vx(4B) + vy(4B) + 校验(1B) + 尾码(2B)。
 * 静态目标只使用经纬高，vx/vy 忽略。
 */
bool ModeSatGuid::parse_gps_frame(const uint8_t *frame)
{
    if (frame[0] != FRAME_TYPE_GPS) {
        return false;
    }
    const int32_t lat_e7 = int32_from_le(frame + 1);
    const int32_t lon_e7 = int32_from_le(frame + 5);
    const int32_t alt_cm = int32_from_le(frame + 9);

    Location new_loc(lat_e7, lon_e7, alt_cm, Location::AltFrame::ABSOLUTE);
    if (!new_loc.initialised()) {
        return false;
    }

    target.loc = new_loc;
    target.active_serial = true;
    target.last_serial_ms = AP_HAL::millis();
    target.last_update_ms = target.last_serial_ms;

    return true;
}

/**
 * @brief 从串口读取字节并解析目标帧
 *
 * 采用状态机方式寻找帧头、接收定长帧体、校验并解析。
 */
void ModeSatGuid::read_serial()
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

        switch (parse_state) {
        case 0:
            // 等待第一个帧头字节
            if (c == FRAME_HEADER_1) {
                parse_state = 1;
            }
            break;

        case 1:
            // 等待第二个帧头字节
            if (c == FRAME_HEADER_2) {
                parse_state = 2;
                frame_idx = 0;
            } else if (c != FRAME_HEADER_1) {
                // 若不是连续帧头则重置
                parse_state = 0;
            }
            break;

        case 2:
            // 接收帧体：载荷 18B + 校验 1B + 尾码 2B = 21B
            frame_buffer[frame_idx++] = uint8_t(c);
            if (frame_idx >= 21) {
                if (validate_frame(frame_buffer, 21) && parse_gps_frame(frame_buffer)) {
                    // 目标状态已在 parse_gps_frame 中更新
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

// ============================================================
// 六、参数目标与目标有效性判断
// ============================================================

/**
 * @brief 从 SGUID_TGT_LAT/LON/ALT 参数加载目标坐标
 *
 * 仅当经纬度均不为 0 时才认为参数目标有效。
 */
bool ModeSatGuid::load_param_target()
{
    target.loc = Location(int32_t(plane.sat_guid_guidance.tgt_lat.get() * 1.0e7f),
                          int32_t(plane.sat_guid_guidance.tgt_lon.get() * 1.0e7f),
                          int32_t(plane.sat_guid_guidance.tgt_alt.get() * 100.0f),
                          Location::AltFrame::ABSOLUTE);
    target.active_param = (!is_zero(plane.sat_guid_guidance.tgt_lat.get()) &&
                           !is_zero(plane.sat_guid_guidance.tgt_lon.get()));
    return target.active_param;
}

/**
 * @brief 每周期更新目标状态
 *
 * 参数目标：每周期重新加载，确保飞行中修改参数立即生效，且永不过时；
 * 串口目标：以最新收到一帧的时间戳为准。
 */
void ModeSatGuid::update_target_state()
{
    const uint32_t now = AP_HAL::millis();

    if (plane.sat_guid_guidance.tgt_src.get() == 0 && load_param_target()) {
        target.last_update_ms = now;
    }

    if (target.active_serial) {
        target.last_update_ms = target.last_serial_ms;
    }
}

/**
 * @brief 判断当前目标是否有效
 *
 * 条件：坐标已初始化，且在超时窗口内有过更新。
 */
bool ModeSatGuid::target_valid() const
{
    const uint32_t now = AP_HAL::millis();
    if (now - target.last_update_ms > uint32_t(plane.sat_guid_guidance.tout_ms.get())) {
        return false;
    }
    if (!target.loc.initialised()) {
        return false;
    }
    return true;
}

/**
 * @brief 处理目标丢失事件
 *
 * 根据 SGUID_LOSS_ACT 参数：
 * - 0：留在 SATGUID 内，由 TARGET_LOSS 状态保持位置/盘旋等待；
 * - 1：切换到 LOITER 模式；
 * - 2：切换到 RTL 模式。
 */
void ModeSatGuid::handle_target_loss()
{
    if (state == State::TARGET_LOSS) {
        return;
    }
    state = State::TARGET_LOSS;
    plane.gcs().send_text(MAV_SEVERITY_WARNING, "SATGUID: target lost");

    const uint8_t action = plane.sat_guid_guidance.loss_action.get();
    if (action == 1) {
        plane.set_mode(plane.mode_loiter, ModeReason::GCS_COMMAND);
    } else if (action == 2) {
        plane.set_mode(plane.mode_rtl, ModeReason::GCS_COMMAND);
    }
    // action == 0 在本函数中不切换模式，由 run_target_loss 执行等待逻辑
}

// ============================================================
// 七、制导计算工具函数
// ============================================================

/**
 * @brief 将角度规范化到 [-pi, pi] 区间
 */
float ModeSatGuid::wrap_pi(float angle) const
{
    while (angle > M_PI) {
        angle -= 2.0f * M_PI;
    }
    while (angle < -M_PI) {
        angle += 2.0f * M_PI;
    }
    return angle;
}

/**
 * @brief 计算到目标的制导量
 *
 * 输出：
 * - distance_m：水平距离；
 * - bearing_rad：目标方位角（北东地坐标系，正北为 0，顺时针为正）；
 * - elevation_rad：目标俯仰角（目标在飞机下方为负）；
 * - bearing_error_rad：方位误差；
 * - bearing_rate_rad_s / elevation_rate_rad_s：视线角速率，用于阻尼。
 */
void ModeSatGuid::compute_guidance()
{
    if (!target.loc.initialised()) {
        return;
    }

    const uint32_t now = AP_HAL::millis();

    // 水平距离与方位
    guidance.distance_m = plane.current_loc.get_distance(target.loc);
    guidance.bearing_rad = plane.current_loc.get_bearing(target.loc);

    // 计算目标相对飞机的俯仰角
    float alt_diff_m = 0.0f;
    int32_t target_alt_cm, current_alt_cm;
    if (target.loc.get_alt_cm(Location::AltFrame::ABSOLUTE, target_alt_cm) &&
        plane.current_loc.get_alt_cm(Location::AltFrame::ABSOLUTE, current_alt_cm)) {
        alt_diff_m = (target_alt_cm - current_alt_cm) * 0.01f;
    }
    guidance.elevation_rad = atan2f(alt_diff_m, MAX(guidance.distance_m, 1.0f));

    // 方位误差（飞机当前航向与目标方位之差）
    const float yaw = ahrs.get_yaw();
    guidance.bearing_error_rad = wrap_pi(guidance.bearing_rad - yaw);

    // 计算视线角速率
    float dt = 0.0f;
    if (guidance.last_update_ms != 0) {
        dt = (now - guidance.last_update_ms) * 0.001f;
    }
    if (dt <= 0.0f || dt > 0.5f) {
        // 时间间隔异常时重置角速率，避免使用不可靠的差分
        guidance.bearing_rate_rad_s = 0.0f;
        guidance.elevation_rate_rad_s = 0.0f;
    } else {
        guidance.bearing_rate_rad_s = wrap_pi(guidance.bearing_rad - guidance.last_bearing_rad) / dt;
        guidance.elevation_rate_rad_s = (guidance.elevation_rad - guidance.last_elevation_rad) / dt;
    }
    guidance.last_bearing_rad = guidance.bearing_rad;
    guidance.last_elevation_rad = guidance.elevation_rad;
    guidance.last_update_ms = now;
}

/**
 * @brief 获取当前飞机相对目标的高度差（飞机高于目标为正，单位：米）
 *
 * 该值在俯冲起始判断、俯冲可行性判断、重定位距离计算中都会用到。
 */
float ModeSatGuid::get_alt_above_target_m() const
{
    int32_t current_alt_cm = 0;
    int32_t target_alt_cm = 0;
    if (plane.current_loc.get_alt_cm(Location::AltFrame::ABSOLUTE, current_alt_cm) &&
        target.loc.get_alt_cm(Location::AltFrame::ABSOLUTE, target_alt_cm)) {
        return (current_alt_cm - target_alt_cm) * 0.01f;
    }
    return 0.0f;
}

// ============================================================
// 八、姿态/航路点限制辅助函数
// ============================================================

/**
 * @brief 将横滚指令限制在 SGUID_ROLL_LIM 范围内
 */
void ModeSatGuid::apply_roll_limit()
{
    const int32_t roll_lim_cd = int32_t(plane.sat_guid_guidance.roll_lim.get() * 100.0f);
    plane.nav_roll_cd = constrain_int32(plane.nav_roll_cd, -roll_lim_cd, roll_lim_cd);
}

/**
 * @brief 将俯仰指令限制在 SGUID_PITCH_MIN/MAX 范围内
 *
 * 这是 SATGUID 模式使用独立俯仰限制的关键，避免受到
 * PTCH_LIM_MIN_DEG / PTCH_LIM_MAX_DEG 的约束。
 */
void ModeSatGuid::apply_pitch_limit()
{
    const int32_t pitch_min_cd = int32_t(plane.sat_guid_guidance.pitch_min.get() * 100.0f);
    const int32_t pitch_max_cd = int32_t(plane.sat_guid_guidance.pitch_max.get() * 100.0f);
    plane.nav_pitch_cd = constrain_int32(plane.nav_pitch_cd, pitch_min_cd, pitch_max_cd);
}

/**
 * @brief 设置固定翼航点并更新 L1 导航控制器
 */
void ModeSatGuid::set_fw_waypoint(const Location &wp)
{
    plane.prev_WP_loc = plane.current_loc;
    plane.next_WP_loc = wp;
    plane.set_target_altitude_location(wp);
    plane.nav_controller->update_waypoint(plane.prev_WP_loc, plane.next_WP_loc);
}

// ============================================================
// 九、爬升目标选择
// ============================================================

/**
 * @brief 选择固定翼爬升阶段的目标高度
 *
 * 远距离：直接爬到 SGUID_CRALT；
 * 近距离：为保证后续有足够俯冲距离，爬得更高。
 * 爬升目标计算基于当前高度 + 几何所需高度 + 安全余量。
 */
void ModeSatGuid::select_climb_target()
{
    if (guidance.distance_m < plane.sat_guid_guidance.close_dist.get()) {
        // 近距离场景：根据水平距离计算最小俯冲高度。
        // 即使目标几乎在正上方，也使用 MIN_DIVE_DISTANCE_M 保证有意义的高度。
        const float horizontal_m = MAX(guidance.distance_m, MIN_DIVE_DISTANCE_M);
        // 假设以 45 度角俯冲，所需高度 = 水平距离 * tan(45°)
        const float min_dive_ht_m = horizontal_m * tanf(fabsf(radians(45.0f)));
        climb_target_rel_m = MAX(float(plane.sat_guid_guidance.cruise_alt.get()),
                                 plane.relative_altitude + min_dive_ht_m + 20.0f);
    } else {
        climb_target_rel_m = float(plane.sat_guid_guidance.cruise_alt.get());
    }
}

// ============================================================
// 十、主更新循环与状态机
// ============================================================

/**
 * @brief 每周期顶层更新函数
 *
 * 流程：
 * 1. 读取串口目标；
 * 2. 更新目标状态（参数/串口）；
 * 3. 目标有效性判断（巡航/俯冲/重定位期间不因暂时丢失而退出）；
 * 4. 计算制导量；
 * 5. 执行状态机转换。
 */
void ModeSatGuid::update()
{
    read_serial();
    update_target_state();

    if (!target_valid()) {
        // 一旦进入攻击阶段（巡航/俯冲/重定位），短暂的串口超时不应导致任务终止，
        // 而是沿用最后一次已知目标位置继续执行。仅在起飞、爬升、过渡等初期阶段退出。
        if (state == State::TAKEOFF || state == State::CLIMB || state == State::TRANSITION) {
            handle_target_loss();
            return;
        }
    } else {
        if (state == State::TARGET_LOSS) {
            // 目标恢复后重新回到巡航阶段
            state = State::CRUISE;
            target_loss_loiter_init = false;
        }
    }

    compute_guidance();
    update_state_machine();
}

/**
 * @brief SATGUID 状态机转换逻辑
 *
 * 这是本模式的核心调度器，负责在各个阶段之间切换。
 * 关键点：
 * - CRUISE 到 DIVE 的转换由俯冲几何决定，而不是固定距离；
 * - DIVE 阶段持续检查俯冲可行性，不可行时提前重定位；
 * - CLIMB 阶段若已接近目标且高度足够，可直接进入俯冲。
 */
void ModeSatGuid::update_state_machine()
{
    int32_t current_alt_cm = 0;
    if (!plane.current_loc.get_alt_cm(Location::AltFrame::ABSOLUTE, current_alt_cm)) {
        current_alt_cm = plane.current_loc.alt;
    }

    switch (state) {
    case State::TAKEOFF:
    {
        // VTOL 起飞到 SGUID_TKOF_H（相对于解锁点高度）
        const int32_t target_takeoff_alt_cm = takeoff_start_alt_cm + int32_t(plane.sat_guid_guidance.takeoff_h.get()) * 100;
        if (current_alt_cm >= target_takeoff_alt_cm) {
            plane.gcs().send_text(MAV_SEVERITY_INFO, "SATGUID: takeoff complete, starting transition");
            plane.quadplane.transition->restart();
            state = State::TRANSITION;
        }
        break;
    }

    case State::TRANSITION:
    {
        // 尾座式前飞过渡由 QuadPlane::update 中的 transition->update 处理
        if (plane.quadplane.transition->complete()) {
            plane.gcs().send_text(MAV_SEVERITY_INFO, "SATGUID: transition complete");
            select_climb_target();
            if (guidance.distance_m < plane.sat_guid_guidance.close_dist.get()) {
                plane.gcs().send_text(MAV_SEVERITY_INFO, "SATGUID: close range, climb to %.1f m", (double)climb_target_rel_m);
            }
            state = State::CLIMB;
        }
        break;
    }

    case State::CLIMB:
    {
        // 重新评估爬升目标，以应对目标在爬升中途才有效的情况
        select_climb_target();

        const float alt_above_target_m = get_alt_above_target_m();
        const float pitch_min_rad = radians(plane.sat_guid_guidance.pitch_min.get());
        const float overshoot_sf = MAX(plane.sat_guid_guidance.overshoot_sf.get(), 1.0f);
        const float max_dive_angle_rad = fabsf(pitch_min_rad) / overshoot_sf;

        // 计算以最大俯冲角完成俯冲所需的水平距离，并预留安全余量
        float required_dive_distance_m = 0.0f;
        if (alt_above_target_m > 0.0f && is_positive(max_dive_angle_rad)) {
            required_dive_distance_m = alt_above_target_m / tanf(max_dive_angle_rad);
        }
        const float dive_start_distance_m = required_dive_distance_m + 150.0f;

        // 若已接近俯冲起始距离且当前高度安全（至少 50m），即使未达巡航高度也直接俯冲，
        // 避免因为继续爬升而飞过目标。
        if (guidance.distance_m < dive_start_distance_m && plane.relative_altitude >= 50.0f) {
            plane.gcs().send_text(MAV_SEVERITY_INFO,
                                  "SATGUID: close enough and high enough, start dive (dist=%.0f, req=%.0f)",
                                  (double)guidance.distance_m,
                                  (double)dive_start_distance_m);
            state = State::DIVE;
            repos.active = false;
            break;
        }

        if (guidance.distance_m < MIN_DIVE_DISTANCE_M) {
            // 水平距离太近，无法建立固定翼俯冲，改为重定位
            plane.gcs().send_text(MAV_SEVERITY_INFO,
                                  "SATGUID: climb too close (%.1f m), reposition",
                                  (double)guidance.distance_m);
            repos.active = false;
            repos.turn_started = false;
            repos.climb_complete = false;
            state = State::REPOSITION;
            break;
        }

        if (plane.relative_altitude >= climb_target_rel_m) {
            if (guidance.distance_m < plane.sat_guid_guidance.close_dist.get()) {
                plane.gcs().send_text(MAV_SEVERITY_INFO, "SATGUID: close range climb complete, dive");
                state = State::DIVE;
            } else {
                plane.gcs().send_text(MAV_SEVERITY_INFO, "SATGUID: climb complete, cruise");
                state = State::CRUISE;
            }
        }
        break;
    }

    case State::CRUISE:
    {
        const float alt_above_target_m = get_alt_above_target_m();
        const float pitch_min_rad = radians(plane.sat_guid_guidance.pitch_min.get());
        const float overshoot_sf = MAX(plane.sat_guid_guidance.overshoot_sf.get(), 1.0f);
        const float max_dive_angle_rad = fabsf(pitch_min_rad) / overshoot_sf;

        // 关键改进：巡航阶段根据当前高度和目标几何，动态计算俯冲起始距离。
        // 这样即使从很高处飞向地面目标，也能在足够远的距离开始俯冲，
        // 避免飞到目标正上方还无法建立有效俯冲航迹。
        float required_dive_distance_m = 0.0f;
        if (alt_above_target_m > 0.0f && is_positive(max_dive_angle_rad)) {
            required_dive_distance_m = alt_above_target_m / tanf(max_dive_angle_rad);
        }
        // 加入 150m 安全余量，给姿态建立和制导响应留出时间
        const float dive_start_distance_m = MAX(required_dive_distance_m + 150.0f,
                                                plane.sat_guid_guidance.dive_dist.get());

        if (guidance.distance_m < dive_start_distance_m) {
            plane.gcs().send_text(MAV_SEVERITY_INFO,
                                  "SATGUID: start dive (dist=%.0f, req=%.0f)",
                                  (double)guidance.distance_m,
                                  (double)dive_start_distance_m);
            state = State::DIVE;
            repos.active = false;
        }
        break;
    }

    case State::DIVE:
    {
        // 俯冲可行性检查：
        // 如果当前几何需要的俯冲角已经超过最大允许俯冲角（加 5% 裕量），
        // 说明即使以最大俯冲角也无法命中，必须提前重定位，
        // 而不是等到飞越目标正上方后才被迫重定位。
        const float alt_above_target_m = get_alt_above_target_m();
        const float pitch_min_rad = radians(plane.sat_guid_guidance.pitch_min.get());
        const float overshoot_sf = MAX(plane.sat_guid_guidance.overshoot_sf.get(), 1.0f);
        const float max_dive_angle_rad = fabsf(pitch_min_rad) / overshoot_sf;

        // 当前几何所需的俯冲角（目标在下方时为正值）
        const float required_dive_angle_rad = atan2f(alt_above_target_m, MAX(guidance.distance_m, 1.0f));

        // 动态撞击承诺距离：
        // 高度越高、速度越快，越需要提前承诺撞击，避免临近目标时因为瞬态误差触发重定位。
        const float airspeed_mps = get_air_speed();
        float commit_distance_m = MAX(IMPACT_COMMIT_DISTANCE_M, alt_above_target_m * 0.3f);
        commit_distance_m = MAX(commit_distance_m, airspeed_mps * 2.0f);

        if (guidance.distance_m > commit_distance_m &&
            required_dive_angle_rad > max_dive_angle_rad * 1.05f) {
            plane.gcs().send_text(MAV_SEVERITY_WARNING,
                                  "SATGUID: dive not feasible (el_req=%.1f, max=%.1f), reposition",
                                  (double)degrees(required_dive_angle_rad),
                                  (double)degrees(max_dive_angle_rad));
            state = State::REPOSITION;
            repos.active = false;
            repos.turn_started = false;
            repos.climb_complete = false;
        }
        break;
    }

    case State::REPOSITION:
    {
        // 重定位阶段的退出逻辑由 run_reposition 自己管理
        break;
    }

    case State::TARGET_LOSS:
    {
        // 目标丢失的恢复逻辑在 update() 中处理
        break;
    }
    }
}

// ============================================================
// 十一、空速估计辅助
// ============================================================

/**
 * @brief 获取最佳可用空速估计
 *
 * 优先使用空速传感器，其次使用 AHRS 估计，最后使用地速作为兜底。
 */
float ModeSatGuid::get_air_speed() const
{
#if AP_AIRSPEED_ENABLED
    if (plane.airspeed.enabled() && plane.airspeed.healthy()) {
        return plane.airspeed.get_airspeed();
    }
#endif
    float aspeed;
    if (AP::ahrs().airspeed_estimate(aspeed)) {
        return aspeed;
    }
    return plane.ahrs.groundspeed();
}

// ============================================================
// 十二、run() 总调度
// ============================================================

/**
 * @brief 根据当前阶段调用对应的执行函数
 */
void ModeSatGuid::run()
{
    switch (state) {
    case State::TAKEOFF:
        run_takeoff();
        break;
    case State::TRANSITION:
        run_transition();
        break;
    case State::CLIMB:
        run_climb();
        break;
    case State::CRUISE:
        run_cruise();
        break;
    case State::DIVE:
        run_dive();
        break;
    case State::REPOSITION:
        run_reposition();
        break;
    case State::TARGET_LOSS:
        run_target_loss();
        break;
    }
}

// ============================================================
// 十三、各阶段执行函数
// ============================================================

/**
 * @brief 垂直起飞阶段
 *
 * 使用四旋翼位置控制器垂直爬升到 SGUID_TKOF_H，
 * 水平位置保持，航向保持。
 */
void ModeSatGuid::run_takeoff()
{
    quadplane.assist.check_VTOL_recovery();

    if (quadplane.throttle_wait) {
        quadplane.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        attitude_control->set_throttle_out(0, true, 0);
        quadplane.relax_attitude_control();
        pos_control->relax_z_controller(0);
    } else {
        // 设置垂直速度/加速度限制
        pos_control->set_max_speed_accel_z(-quadplane.get_pilot_velocity_z_max_dn(),
                                           quadplane.pilot_speed_z_max_up * 100,
                                           quadplane.pilot_accel_z * 100);
        pos_control->set_correction_speed_accel_z(-quadplane.get_pilot_velocity_z_max_dn(),
                                                  quadplane.pilot_speed_z_max_up * 100,
                                                  quadplane.pilot_accel_z * 100);
        plane.quadplane.assign_tilt_to_fwd_thr();

        // 以默认上升速度爬升
        quadplane.set_climb_rate_cms(quadplane.wp_nav->get_default_speed_up());
        quadplane.run_z_controller();
    }

    // 水平速度、加速度设为零，保持当前水平位置
    Vector2f vel, accel;
    pos_control->input_vel_accel_xy(vel, accel);
    quadplane.run_xy_controller();

    // 从位置控制器获取期望滚转/俯仰（尾座式 VTOL 视角）
    plane.nav_roll_cd = pos_control->get_roll_cd();
    plane.nav_pitch_cd = pos_control->get_pitch_cd();

    // 稳定固定翼舵面并居中方向舵
    plane.stabilize_roll();
    plane.stabilize_pitch();
    output_rudder_and_steering(0.0f);

    quadplane.assist.output_spin_recovery();

    SATGUID_DBG("TKOF cur_alt=%.1f tkof_h=%d rel_alt=%.1f",
                (double)(plane.current_loc.alt * 0.01),
                (int)plane.sat_guid_guidance.takeoff_h.get(),
                (double)plane.relative_altitude);
}

/**
 * @brief 过渡阶段
 *
 * 尾座式前飞过渡主要由 QuadPlane::update 中的 transition->update 处理，
 * 本阶段仅保持姿态中立并运行固定翼舵面稳定。
 */
void ModeSatGuid::run_transition()
{
    plane.nav_roll_cd = 0;
    plane.nav_pitch_cd = 0;

    // 运行固定翼舵面稳定
    Mode::run();

    // 过渡期间方向舵归零
    output_rudder_and_steering(0.0f);
}

/**
 * @brief 固定翼爬升阶段
 *
 * 使用 L1 横向导航飞向目标，纵向使用固定爬升角 SGUID_CLMB_ANG。
 */
void ModeSatGuid::run_climb()
{
    // 计算 home 点绝对高度（米）
    float home_amsl_m = 0.0f;
    int32_t current_alt_cm;
    if (plane.current_loc.get_alt_cm(Location::AltFrame::ABSOLUTE, current_alt_cm)) {
        home_amsl_m = current_alt_cm * 0.01f - plane.relative_altitude;
    }

    // 爬升航点使用目标经纬度，高度为 home + climb_target_rel_m
    Location wp = target.loc;
    wp.set_alt_cm(int32_t((home_amsl_m + climb_target_rel_m) * 100.0f),
                  Location::AltFrame::ABSOLUTE);

    set_fw_waypoint(wp);
    plane.calc_nav_roll();
    apply_roll_limit();

    // 纵向直接使用爬升角命令
    const float pitch_cmd_rad = radians(plane.sat_guid_guidance.climb_ang.get());
    plane.nav_pitch_cd = int32_t(degrees(pitch_cmd_rad) * 100.0f);
    apply_pitch_limit();

    Mode::run();

    // 全油门以获得最佳爬升率
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, plane.aparm.throttle_max.get());

    SATGUID_DBG("CLIMB dist=%.1f rel_alt=%.1f tgt=%.1f pitch=%d roll=%d",
                (double)guidance.distance_m,
                (double)plane.relative_altitude,
                (double)climb_target_rel_m,
                (int)plane.nav_pitch_cd,
                (int)plane.nav_roll_cd);
}

/**
 * @brief 固定翼巡航阶段
 *
 * 使用 L1 控制器飞向目标，同时叠加随机横向/高度/速度走廊扰动，
 * 模拟非直线接近轨迹。
 */
void ModeSatGuid::run_cruise()
{
    // 推进走廊相位（假设 run 以约 50Hz 调用）
    const float freq = MAX(plane.sat_guid_guidance.course_noise_f.get(), 0.001f);
    corridor_phase += 2.0f * M_PI * freq * 0.02f;
    if (corridor_phase > 2.0f * M_PI) {
        corridor_phase -= 2.0f * M_PI;
    }

    const float lateral_bias_m = plane.sat_guid_guidance.cruise_lat_bias.get() * sinf(corridor_phase);
    const float alt_bias_m = plane.sat_guid_guidance.cruise_alt_bias.get() * sinf(corridor_phase + M_PI_2);
    const float spd_bias_mps = plane.sat_guid_guidance.cruise_spd_bias.get() * sinf(corridor_phase + M_PI);

    // 计算 home 点绝对高度
    float home_amsl_m = 0.0f;
    int32_t current_alt_cm;
    if (plane.current_loc.get_alt_cm(Location::AltFrame::ABSOLUTE, current_alt_cm)) {
        home_amsl_m = current_alt_cm * 0.01f - plane.relative_altitude;
    }

    // 构建带走廊偏移的航点
    Location wp = target.loc;
    wp.offset_bearing(90.0f + degrees(guidance.bearing_rad), lateral_bias_m);
    wp.set_alt_cm(int32_t((home_amsl_m + plane.sat_guid_guidance.cruise_alt.get() + alt_bias_m) * 100.0f),
                  Location::AltFrame::ABSOLUTE);

    set_fw_waypoint(wp);
    plane.calc_nav_roll();
    apply_roll_limit();
    plane.calc_nav_pitch();
    apply_pitch_limit();  // 使用 SATGUID 独立俯仰限制

    // 速度走廊
    const float cruise_spd = plane.sat_guid_guidance.cruise_spd.get();
    const float target_spd = MAX(cruise_spd + spd_bias_mps, 5.0f);
    plane.target_airspeed_cm = int32_t(target_spd * 100.0f);

    Mode::run();
    plane.calc_throttle();

    SATGUID_DBG("CRUISE dist=%.1f spd=%.1f yaw_err=%.1f roll=%d",
                (double)guidance.distance_m,
                (double)target_spd,
                (double)degrees(guidance.bearing_error_rad),
                (int)plane.nav_roll_cd);
}

/**
 * @brief 俯冲/末制导阶段
 *
 * 这是命中最关键的阶段。主要逻辑：
 * 1. 远距离（> TERMINAL_DISTANCE_M）使用 L1 航点导航；
 * 2. 近距离切换为纯方位追踪，避免 L1 在航点附近不稳定；
 * 3. 俯仰角直接跟踪目标视线俯仰角，并加入角速率阻尼；
 * 4. 全油门保持能量；
 * 5. 撞击承诺距离内不再触发重定位。
 */
void ModeSatGuid::run_dive()
{
    const float yaw = ahrs.get_yaw();
    const float pitch_min_rad = radians(plane.sat_guid_guidance.pitch_min.get());
    const float pitch_max_rad = radians(plane.sat_guid_guidance.pitch_max.get());
    const float roll_lim_rad = radians(plane.sat_guid_guidance.roll_lim.get());

    // --------------------------
    // 横向控制：远距离 L1，近距离纯方位追踪
    // --------------------------
    if (guidance.distance_m < TERMINAL_DISTANCE_M) {
        // 当飞机非常接近目标时，L1 航点导航会把目标当作下一个航点，
        // 由于飞机与航点几乎重合，L1 会产生剧烈振荡甚至方向反转。
        // 因此切换为直接追踪目标方位。
        const float bearing_error = wrap_pi(guidance.bearing_rad - yaw);

        // 比例-微分方位追踪：
        // KP_BEARING 把方位误差映射为横滚角；
        // KD_BEARING 用方位角速率阻尼，抑制高速接近时的振荡。
        constexpr float KP_BEARING = 3.0f;
        constexpr float KD_BEARING = 0.5f;
        float roll_cmd_rad = KP_BEARING * bearing_error - KD_BEARING * guidance.bearing_rate_rad_s;
        roll_cmd_rad = constrain_float(roll_cmd_rad, -roll_lim_rad, roll_lim_rad);
        plane.nav_roll_cd = int32_t(degrees(roll_cmd_rad) * 100.0f);
    } else {
        // 远距离仍使用 L1 控制器，平滑地飞向目标
        set_fw_waypoint(target.loc);
        plane.calc_nav_roll();
        apply_roll_limit();
    }

    // --------------------------
    // 纵向控制：直接视线俯仰跟踪
    // --------------------------
    // 期望俯仰角直接取目标视线俯仰角，使机头始终指向目标。
    // 这比 "dive_ang + kp*(elevation - dive_ang)" 更直接，
    // 在目标在正下方等极端几何下响应更快。
    float pitch_cmd_rad = guidance.elevation_rad;

    // 加入俯仰角速率阻尼，抑制末段由于距离快速减小导致的俯仰角剧烈变化
    pitch_cmd_rad += 0.15f * guidance.elevation_rate_rad_s;

    // 将俯仰指令限制在 [SGUID_PITCH_MIN, SGUID_PITCH_MAX]
    pitch_cmd_rad = constrain_float(pitch_cmd_rad, pitch_min_rad, pitch_max_rad);
    plane.nav_pitch_cd = int32_t(degrees(pitch_cmd_rad) * 100.0f);
    apply_pitch_limit();

    // --------------------------
    // 运行固定翼控制环与油门
    // --------------------------
    Mode::run();

    // 俯冲阶段保持全油门，确保速度能量充足，便于控制响应
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, plane.aparm.throttle_max.get());

    SATGUID_DBG("DIVE dist=%.1f el=%.1f pitch=%d roll=%d",
                (double)guidance.distance_m,
                (double)degrees(guidance.elevation_rad),
                (int)plane.nav_pitch_cd,
                (int)plane.nav_roll_cd);
}

/**
 * @brief 重定位/过顶恢复阶段
 *
 * 当飞机无法在当前几何下完成俯冲（如从目标正上方飞过）时进入本阶段。
 * 目标：快速脱离目标区域，建立足够的水平距离和高度，然后重新对准目标俯冲。
 *
 * 改进点：
 * 1. 第一阶段不再无差别直飞，而是优先背向目标飞行，尽快离开危险区域；
 * 2. 第二阶段不直接飞向目标，而是飞向目标后方的一个"俯冲进入点"，
 *    从而以大半径平滑转弯重新建立俯冲航线，避免在目标正上方盘旋。
 */
void ModeSatGuid::run_reposition()
{
    const float yaw = ahrs.get_yaw();

    // --------------------------
    // 初始化重定位状态
    // --------------------------
    if (!repos.active) {
        repos.active = true;
        repos.turn_started = false;
        repos.climb_complete = false;
        repos.start_loc = plane.current_loc;
        // 记录进入重定位时的航向，作为后续重新建立俯冲航线的参考方向
        repos.entry_heading_rad = yaw;
        plane.gcs().send_text(MAV_SEVERITY_INFO, "SATGUID: reposition start");
    }

    const float alt_above_target_m = get_alt_above_target_m();

    // --------------------------
    // 低高度安全爬升
    // --------------------------
    // 若飞机在目标上方 100m 以内，先改平飞并爬升到 200m 以上，
    // 避免在过低高度做剧烈机动撞地。
    if (alt_above_target_m < 100.0f && !repos.climb_complete) {
        plane.nav_roll_cd = 0;
        const float pitch_cmd_rad = radians(plane.sat_guid_guidance.climb_ang.get());
        plane.nav_pitch_cd = int32_t(degrees(pitch_cmd_rad) * 100.0f);
        apply_pitch_limit();
        plane.target_airspeed_cm = int32_t(plane.sat_guid_guidance.cruise_spd.get() * 100.0f);

        Mode::run();
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, plane.aparm.throttle_max.get());

        if (alt_above_target_m > 200.0f) {
            repos.climb_complete = true;
            plane.gcs().send_text(MAV_SEVERITY_INFO, "SATGUID: reposition climb complete, alt=%.1f", (double)alt_above_target_m);
        }

        SATGUID_DBG("REPOS_CLIMB dist=%.1f alt_above=%.1f", (double)guidance.distance_m, (double)alt_above_target_m);
        return;
    }

    // --------------------------
    // 计算重定位所需几何参数
    // --------------------------
    // 盘旋半径作为最小直线段长度单位
    const float loiter_radius_m = MAX(fabsf(float(plane.aparm.loiter_radius)), 30.0f);
    const float min_straight_m = plane.sat_guid_guidance.repos_mul.get() * loiter_radius_m;

    // 根据当前相对高度和最大俯冲角，计算完成俯冲所需的最小水平距离
    const float pitch_min_rad = radians(plane.sat_guid_guidance.pitch_min.get());
    const float overshoot_sf = MAX(plane.sat_guid_guidance.overshoot_sf.get(), 1.0f);
    const float max_dive_angle_rad = fabsf(pitch_min_rad) / overshoot_sf;

    float required_horizontal_m = 0.0f;
    if (alt_above_target_m > 0.0f && is_positive(max_dive_angle_rad)) {
        required_horizontal_m = alt_above_target_m / tanf(max_dive_angle_rad);
    }
    // 取几何所需距离与最小直线段的较大值，并再加 50m 安全余量
    required_horizontal_m = MAX(required_horizontal_m, min_straight_m) + 50.0f;

    const float flown_m = plane.current_loc.get_distance(repos.start_loc);

    // --------------------------
    // 第一阶段：背向目标直飞，创造水平距离
    // --------------------------
    if (!repos.turn_started) {
        // 选择期望航向：
        // - 若已经在目标非常近处（< 50m），直接背向目标飞离；
        // - 否则保持进入重定位时的航向，沿原航迹继续飞一段，便于后续顺滑转弯。
        float desired_yaw;
        if (guidance.distance_m < 50.0f) {
            desired_yaw = wrap_pi(guidance.bearing_rad + M_PI);
        } else {
            desired_yaw = repos.entry_heading_rad;
        }

        // 使用航向跟踪生成横滚指令（比例控制），实现期望航向
        const float yaw_error = wrap_pi(desired_yaw - yaw);
        constexpr float KP_YAW_TO_ROLL = 1.5f;
        float roll_cmd_rad = KP_YAW_TO_ROLL * yaw_error;
        roll_cmd_rad = constrain_float(roll_cmd_rad, -radians(plane.sat_guid_guidance.roll_lim.get()), radians(plane.sat_guid_guidance.roll_lim.get()));
        plane.nav_roll_cd = int32_t(degrees(roll_cmd_rad) * 100.0f);

        // 平飞，保持巡航速度
        plane.nav_pitch_cd = 0;
        plane.target_airspeed_cm = int32_t(plane.sat_guid_guidance.cruise_spd.get() * 100.0f);

        Mode::run();
        plane.calc_throttle();

        // 退出第一阶段的条件：
        // 已飞出最小直线段，并且当前到目标的水平距离 >= 俯冲所需水平距离。
        const bool min_leg_complete = flown_m >= min_straight_m;
        const bool separation_ok = guidance.distance_m >= required_horizontal_m;
        if (min_leg_complete && separation_ok) {
            repos.turn_started = true;
            plane.gcs().send_text(MAV_SEVERITY_INFO,
                                  "SATGUID: reposition turn back (sep=%.0f m)",
                                  (double)required_horizontal_m);
        }

        SATGUID_DBG("REPOS_S dist=%.1f flown=%.1f req=%.1f yaw_err=%.1f",
                    (double)guidance.distance_m,
                    (double)flown_m,
                    (double)required_horizontal_m,
                    (double)degrees(yaw_error));
        return;
    }

    // --------------------------
    // 第二阶段：飞向俯冲进入点，而不是直接飞向目标
    // --------------------------
    // 俯冲进入点设在目标逆着原进入航向的后方，距离为 required_horizontal_m。
    // 这样飞机从目标前方以大半径转弯重新进入俯冲，不会在目标正上方盘旋。
    Location dive_entry_point = target.loc;
    dive_entry_point.offset_bearing(degrees(wrap_pi(repos.entry_heading_rad + M_PI)), required_horizontal_m);

    // 俯冲进入点高度与当前高度相同，平飞接近
    int32_t current_alt_cm = 0;
    if (plane.current_loc.get_alt_cm(Location::AltFrame::ABSOLUTE, current_alt_cm)) {
        dive_entry_point.set_alt_cm(current_alt_cm, Location::AltFrame::ABSOLUTE);
    }

    set_fw_waypoint(dive_entry_point);
    plane.calc_nav_roll();
    apply_roll_limit();
    plane.calc_nav_pitch();
    apply_pitch_limit();
    plane.target_airspeed_cm = int32_t(plane.sat_guid_guidance.cruise_spd.get() * 100.0f);

    Mode::run();
    plane.calc_throttle();

    // --------------------------
    // 退出条件：接近俯冲进入点且航向大致对准目标
    // --------------------------
    const float dist_to_entry_m = plane.current_loc.get_distance(dive_entry_point);
    const float bearing_to_target = plane.current_loc.get_bearing(target.loc);
    const bool aligned = fabsf(wrap_pi(bearing_to_target - yaw)) < radians(20.0f);
    const bool near_entry = dist_to_entry_m < MAX(required_horizontal_m * 0.3f, 50.0f);
    const bool elevation_ok = guidance.elevation_rad > pitch_min_rad / (overshoot_sf * 1.1f);

    if (near_entry && aligned && elevation_ok) {
        plane.gcs().send_text(MAV_SEVERITY_INFO, "SATGUID: reposition complete, resume dive");
        repos.active = false;
        state = State::DIVE;
    }

    SATGUID_DBG("REPOS_T dist=%.1f entry_dist=%.1f bear_err=%.1f el=%.1f req=%.1f",
                (double)guidance.distance_m,
                (double)dist_to_entry_m,
                (double)degrees(wrap_pi(bearing_to_target - yaw)),
                (double)degrees(guidance.elevation_rad),
                (double)required_horizontal_m);
}

/**
 * @brief 目标丢失等待阶段
 *
 * 当 SGUID_LOSS_ACT = 0 时进入本状态：
 * - VTOL：使用 loiter_nav 主动保持当前位置/高度悬停；
 * - 固定翼：在当前位置盘旋等待。
 */
void ModeSatGuid::run_target_loss()
{
#if HAL_QUADPLANE_ENABLED
    if (quadplane.in_vtol_mode()) {
        // VTOL：主动保持当前位置（类似 QLOITER）
        const uint32_t now = AP_HAL::millis();
        if (!target_loss_loiter_init || now - quadplane.last_loiter_ms > 500) {
            loiter_nav->clear_pilot_desired_acceleration();
            loiter_nav->init_target();
            if (!target_loss_loiter_init) {
                pos_control->init_z_controller();
                target_loss_loiter_init = true;
            }
        }
        quadplane.last_loiter_ms = now;

        quadplane.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        // 水平定点
        loiter_nav->update();
        plane.nav_roll_cd = loiter_nav->get_roll();
        plane.nav_pitch_cd = loiter_nav->get_pitch();
        quadplane.assign_tilt_to_fwd_thr();

        // 高度保持
        pos_control->set_max_speed_accel_z(
            -quadplane.get_pilot_velocity_z_max_dn(),
            quadplane.pilot_speed_z_max_up * 100,
            quadplane.pilot_accel_z * 100);
        quadplane.set_climb_rate_cms(0);
        quadplane.run_z_controller();

        // 多旋翼姿态控制器
        Vector3f target_att {
            plane.nav_roll_cd * 0.01f,
            plane.nav_pitch_cd * 0.01f,
            0.0f
        };
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(
            target_att.x * 100.0f,
            target_att.y * 100.0f,
            target_att.z * 100.0f);

        // 同时驱动固定翼舵面
        plane.stabilize_roll();
        plane.stabilize_pitch();
        output_rudder_and_steering(0.0f);
        return;
    }
#endif

    // 固定翼：在当前位置盘旋等待目标恢复
    plane.do_loiter_at_location();
    plane.update_loiter(0);
    plane.calc_nav_roll();
    apply_roll_limit();
    plane.calc_nav_pitch();
    apply_pitch_limit();
    plane.target_airspeed_cm = int32_t(plane.sat_guid_guidance.cruise_spd.get() * 100.0f);
    Mode::run();
    plane.calc_throttle();
}

#endif  // HAL_QUADPLANE_ENABLED
