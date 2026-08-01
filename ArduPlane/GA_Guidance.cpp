#include "GA_Guidance.h"

const AP_Param::GroupInfo GA_Guidance::var_info[] = {
    // @Param: HFOV
    // @DisplayName: Camera horizontal FOV
    // @Description: Horizontal field of view of the gimbal camera
    // @Units: deg
    // @Range: 10 180
    // @User: Standard
    AP_GROUPINFO("HFOV", 0, GA_Guidance, hfov, 60.0f),

    // @Param: VFOV
    // @DisplayName: Camera vertical FOV
    // @Description: Vertical field of view of the gimbal camera
    // @Units: deg
    // @Range: 10 180
    // @User: Standard
    AP_GROUPINFO("VFOV", 1, GA_Guidance, vfov, 45.0f),

    // @Param: DIVE_PITCH
    // @DisplayName: Programmed dive pitch
    // @Description: Base pitch command used to dive toward the target
    // @Units: deg
    // @Range: -60 0
    // @User: Standard
    AP_GROUPINFO("DIVE_PITCH", 2, GA_Guidance, dive_pitch, -18.0f),

    // @Param: MIN_ALT
    // @DisplayName: Minimum altitude for dive safety
    // @Description: Below this altitude the dive pitch is limited to -5 deg
    // @Units: m
    // @Range: 0 500
    // @User: Standard
    AP_GROUPINFO("MIN_ALT", 3, GA_Guidance, min_alt, 30),

    // @Param: TOUT_MS
    // @DisplayName: Target loss timeout
    // @Description: Time without valid target before triggering GA_LOSS_ACT
    // @Units: ms
    // @Range: 100 5000
    // @User: Standard
    AP_GROUPINFO("TOUT_MS", 4, GA_Guidance, timeout_ms, 500),

    // @Param: ROLL_LIM
    // @DisplayName: Roll limit
    // @Description: Maximum roll command during guidance
    // @Units: deg
    // @Range: 5 80
    // @User: Standard
    AP_GROUPINFO("ROLL_LIM", 5, GA_Guidance, roll_lim, 35.0f),

    // @Param: PITCH_MIN
    // @DisplayName: Minimum pitch
    // @Description: Minimum allowed pitch command
    // @Units: deg
    // @Range: -90 0
    // @User: Standard
    AP_GROUPINFO("PITCH_MIN", 6, GA_Guidance, pitch_min, -45.0f),

    // @Param: PITCH_MAX
    // @DisplayName: Maximum pitch
    // @Description: Maximum allowed pitch command
    // @Units: deg
    // @Range: 0 45
    // @User: Standard
    AP_GROUPINFO("PITCH_MAX", 7, GA_Guidance, pitch_max, 10.0f),

    // @Param: KP_ROLL
    // @DisplayName: Roll P gain
    // @Description: Roll proportional gain on bearing error
    // @Range: 0 5
    // @User: Standard
    AP_GROUPINFO("KP_ROLL", 8, GA_Guidance, kp_roll, 0.8f),

    // @Param: KD_ROLL
    // @DisplayName: Roll D gain
    // @Description: Roll damping gain on bearing rate
    // @Range: 0 5
    // @User: Standard
    AP_GROUPINFO("KD_ROLL", 9, GA_Guidance, kd_roll, 0.3f),

    // @Param: PNG_N
    // @DisplayName: PNG gain
    // @Description: Proportional navigation gain on bearing rate
    // @Range: 0 5
    // @User: Standard
    AP_GROUPINFO("PNG_N", 10, GA_Guidance, png_n, 2.0f),

    // @Param: KP_PITCH
    // @DisplayName: Pitch P gain
    // @Description: Pitch proportional gain on elevation error
    // @Range: 0 5
    // @User: Standard
    AP_GROUPINFO("KP_PITCH", 11, GA_Guidance, kp_pitch, 0.2f),

    // @Param: LOSS_ACT
    // @DisplayName: Target loss action
    // @Description: Action when target is lost for longer than GA_TOUT_MS
    // @Values: 0:LevelHold,1:Loiter,2:RTL
    // @User: Standard
    AP_GROUPINFO("LOSS_ACT", 12, GA_Guidance, loss_action, 0),

    // ===== 新增参数：高精度图像制导优化 =====

    // @Param: TGT_ALT
    // @DisplayName: Target altitude AMSL
    // @Description: Target altitude above mean sea level. Used with current altitude
    //   to compute height-above-target for slant range estimation. Set to 0 to use
    //   home altitude as default (assumes target is on the ground at takeoff point).
    //   目标海拔高度。与当前高度一起用于计算飞机高于目标的高度差，进而估计斜距。
    //   设为0时默认使用home海拔（假设目标位于起飞点地面）。
    // @Units: m
    // @Range: -100 10000
    // @User: Standard
    AP_GROUPINFO("TGT_ALT", 13, GA_Guidance, tgt_alt, 0.0f),

    // @Param: GSC_RNG
    // @DisplayName: Gain scheduling range
    // @Description: Reference distance for gain scheduling. When slant range > this
    //   value, base gains are used. When closer, gains are scaled down to avoid
    //   terminal oscillation. 增益调度参考距离。斜距大于此值时使用基准增益，
    //   小于此值时增益按比例缩小，避免末端振荡。
    // @Units: m
    // @Range: 20 1000
    // @User: Standard
    AP_GROUPINFO("GSC_RNG", 14, GA_Guidance, gain_sched_range, 200.0f),

    // @Param: TERM_RNG
    // @DisplayName: Terminal phase range
    // @Description: Distance threshold for terminal guidance. When range is shorter
    //   than this, the mode switches to pure pursuit with reduced roll limit to
    //   prevent overshoot. 终端制导距离阈值。小于此距离时切换为纯追踪，
    //   限制滚转角以避免过冲。
    // @Units: m
    // @Range: 10 300
    // @User: Standard
    AP_GROUPINFO("TERM_RNG", 15, GA_Guidance, terminal_range, 50.0f),

    // @Param: RUD_MIX
    // @DisplayName: Rudder mixing gain
    // @Description: Rudder mixing gain for roll-yaw coupling compensation.
    //   During roll, rudder is deflected to reduce sideslip and improve
    //   heading tracking accuracy. 0=off, 0.3=light, 0.6=strong.
    //   方向舵混合增益。滚转时方向舵偏转以减小侧滑，提高航向跟踪精度。
    // @Range: 0 1
    // @User: Standard
    AP_GROUPINFO("RUD_MIX", 16, GA_Guidance, rudder_mix, 0.3f),

    // @Param: GMB_OFS_X
    // @DisplayName: Gimbal offset X (forward)
    // @Description: Gimbal mounting offset from CG in body X axis (forward).
    //   云台相对于重心的前向安装偏移。
    // @Units: m
    // @Range: -2 2
    // @User: Standard
    AP_GROUPINFO("GMB_OFS_X", 17, GA_Guidance, gimbal_offset_x, 0.0f),

    // @Param: GMB_OFS_Y
    // @DisplayName: Gimbal offset Y (right)
    // @Description: Gimbal mounting offset from CG in body Y axis (right).
    //   云台相对于重心的右侧安装偏移。
    // @Units: m
    // @Range: -2 2
    // @User: Standard
    AP_GROUPINFO("GMB_OFS_Y", 18, GA_Guidance, gimbal_offset_y, 0.0f),

    // @Param: GMB_OFS_Z
    // @DisplayName: Gimbal offset Z (down)
    // @Description: Gimbal mounting offset from CG in body Z axis (down).
    //   云台相对于重心的下方安装偏移。
    // @Units: m
    // @Range: -2 2
    // @User: Standard
    AP_GROUPINFO("GMB_OFS_Z", 19, GA_Guidance, gimbal_offset_z, 0.0f),

    // @Param: KF_ALPHA
    // @DisplayName: Kalman filter position gain
    // @Description: Alpha gain of the alpha-beta filter for LOS angles and range.
    //   Higher = faster response but weaker filtering. Recommended 0.1~0.3.
    //   Alpha-beta滤波器的位置增益。越大响应越快，滤波效果越弱。
    // @Range: 0.01 0.5
    // @User: Standard
    AP_GROUPINFO("KF_ALPHA", 20, GA_Guidance, kf_alpha, 0.15f),

    // @Param: KF_BETA
    // @DisplayName: Kalman filter velocity gain
    // @Description: Beta gain of the alpha-beta filter for LOS angles and range.
    //   Higher = more sensitive velocity estimate but noisier. Recommended 0.01~0.05.
    //   Alpha-beta滤波器的速度增益。越大速度估计越灵敏，噪声越大。
    // @Range: 0.001 0.1
    // @User: Standard
    AP_GROUPINFO("KF_BETA", 21, GA_Guidance, kf_beta, 0.02f),

    AP_GROUPEND
};

GA_Guidance::GA_Guidance()
{
    AP_Param::setup_object_defaults(this, var_info);
}