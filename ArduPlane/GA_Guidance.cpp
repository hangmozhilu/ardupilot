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

    // @Param: TOUT_MS
    // @DisplayName: Target loss timeout
    // @Description: Time without valid target before triggering GA_LOSS_ACT
    // @Units: ms
    // @Range: 100 5000
    // @User: Standard
    AP_GROUPINFO("TOUT_MS", 2, GA_Guidance, timeout_ms, 500),

    // @Param: ROLL_LIM
    // @DisplayName: Roll limit
    // @Description: Maximum roll command during guidance
    // @Units: deg
    // @Range: 5 80
    // @User: Standard
    AP_GROUPINFO("ROLL_LIM", 3, GA_Guidance, roll_lim, 35.0f),

    // @Param: PITCH_MIN
    // @DisplayName: Minimum pitch
    // @Description: Minimum allowed pitch command
    // @Units: deg
    // @Range: -90 0
    // @User: Standard
    AP_GROUPINFO("PITCH_MIN", 4, GA_Guidance, pitch_min, -45.0f),

    // @Param: PITCH_MAX
    // @DisplayName: Maximum pitch
    // @Description: Maximum allowed pitch command
    // @Units: deg
    // @Range: 0 45
    // @User: Standard
    AP_GROUPINFO("PITCH_MAX", 5, GA_Guidance, pitch_max, 10.0f),

    // @Param: KP_ROLL
    // @DisplayName: Roll P gain
    // @Description: Roll proportional gain on bearing error
    // @Range: 0 5
    // @User: Standard
    AP_GROUPINFO("KP_ROLL", 6, GA_Guidance, kp_roll, 0.8f),

    // @Param: KD_ROLL
    // @DisplayName: Roll D gain
    // @Description: Roll damping gain on bearing rate
    // @Range: 0 5
    // @User: Standard
    AP_GROUPINFO("KD_ROLL", 7, GA_Guidance, kd_roll, 0.3f),

    // @Param: PNG_N
    // @DisplayName: PNG gain
    // @Description: Proportional navigation gain on bearing rate
    // @Range: 0 5
    // @User: Standard
    AP_GROUPINFO("PNG_N", 8, GA_Guidance, png_n, 2.0f),

    // @Param: KP_PITCH
    // @DisplayName: Pitch P gain
    // @Description: Pitch proportional gain on elevation error.
    //   与滚转通道增益匹配，确保俯仰响应与滚转同等强度。
    //   Matches roll channel aggressiveness for balanced response.
    // @Range: 0 5
    // @User: Standard
    AP_GROUPINFO("KP_PITCH", 9, GA_Guidance, kp_pitch, 1.0f),

    // @Param: KD_PITCH
    // @DisplayName: Pitch D gain
    // @Description: Pitch damping gain on elevation rate.
    //   抑制俯仰通道振荡，防止大增益下的过冲。
    //   Dampens pitch oscillations, prevents overshoot with high P gain.
    // @Range: 0 5
    // @User: Standard
    AP_GROUPINFO("KD_PITCH", 10, GA_Guidance, kd_pitch, 0.3f),

    // @Param: PTCH_KP
    // @DisplayName: Flight path angle tracking P gain
    // @Description: Proportional gain for tracking desired flight path angle
    //   in the longitudinal guidance channel. Replaces pure LOS elevation mapping.
    //   航迹角跟踪P增益，用于纵向制导通道替代纯LOS俯仰误差映射。
    // @Range: 0 5
    // @User: Standard
    AP_GROUPINFO("PTCH_KP", 22, GA_Guidance, pitch_track_kp, 1.2f),

    // @Param: PTCH_KD
    // @DisplayName: Flight path angle tracking D gain
    // @Description: Damping gain for flight path angle rate in longitudinal guidance.
    //   航迹角跟踪D增益，用于阻尼航迹角速率。
    // @Range: 0 5
    // @User: Standard
    AP_GROUPINFO("PTCH_KD", 23, GA_Guidance, pitch_track_kd, 0.3f),

    // @Param: FOV_MAR
    // @DisplayName: FOV keep margin
    // @Description: Soft boundary for FOV keep controller. When normalized target
    //   coordinate exceeds this value, guidance pulls target back to image center.
    //   FOV保持控制器软边界。归一化目标坐标超过此值时，制导律将目标拉回图像中心。
    // @Range: 0.5 0.99
    // @User: Standard
    AP_GROUPINFO("FOV_MAR", 24, GA_Guidance, fov_margin, 0.85f),

    // @Param: FOV_GAIN
    // @DisplayName: FOV keep gain
    // @Description: Gain for pulling target back when near FOV edge.
    //   当目标接近视场边缘时的拉回增益。
    // @Range: 0 1
    // @User: Standard
    AP_GROUPINFO("FOV_GAIN", 25, GA_Guidance, fov_gain, 0.3f),

    // @Param: LOSS_ACT
    // @DisplayName: Target loss action
    // @Description: Action when target is lost for longer than GA_TOUT_MS
    // @Values: 0:LevelHold,1:Loiter,2:RTL
    // @User: Standard
    AP_GROUPINFO("LOSS_ACT", 11, GA_Guidance, loss_action, 0),

    // ===== 连续增益调度参数（LOS角度驱动，无高度/距离依赖） =====

    // @Param: ERR_REF
    // @DisplayName: Gain scheduling reference bearing error
    // @Description: Reference bearing error angle for gain scheduling.
    //   When bearing error > this value, gain is maximum (fast alignment).
    //   When bearing error < this value, gain scales down proportionally.
    //   增益调度参考方位误差角。误差大于此值时增益最大，小于此值时按比例缩小。
    // @Units: deg
    // @Range: 3 60
    // @User: Standard
    AP_GROUPINFO("ERR_REF", 12, GA_Guidance, err_ref, 15.0f),

    // @Param: RATE_REF
    // @DisplayName: Gain scheduling reference bearing rate
    // @Description: Reference bearing rate for gain damping.
    //   When bearing rate > this value, the target is considered close and
    //   gain is damped to avoid oscillation. Higher rate ≈ closer target.
    //   增益调度参考方位角速率。角速率大于此值时认为目标接近，增益衰减。
    // @Units: deg/s
    // @Range: 5 100
    // @User: Standard
    AP_GROUPINFO("RATE_REF", 13, GA_Guidance, rate_ref, 30.0f),

    // @Param: TERM_ERR
    // @DisplayName: Terminal phase bearing error threshold
    // @Description: When bearing error < this value AND bearing rate < term_rate,
    //   the mode enters terminal guidance with reduced roll limit.
    //   终端制导方位误差阈值。误差小于此值且角速率够小时进入终端制导。
    // @Units: deg
    // @Range: 1 15
    // @User: Standard
    AP_GROUPINFO("TERM_ERR", 14, GA_Guidance, term_err, 5.0f),

    // @Param: TERM_RATE
    // @DisplayName: Terminal phase bearing rate threshold
    // @Description: When bearing rate < this value AND bearing error < term_err,
    //   the mode enters terminal guidance with reduced roll limit.
    //   终端制导方位角速率阈值。角速率小于此值且误差够小时进入终端制导。
    // @Units: deg/s
    // @Range: 2 30
    // @User: Standard
    AP_GROUPINFO("TERM_RATE", 15, GA_Guidance, term_rate, 10.0f),

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
    // @Description: Alpha gain of the alpha-beta filter for LOS angles.
    //   Higher = faster response but weaker filtering. Recommended 0.1~0.3.
    //   Alpha-beta滤波器的位置增益。越大响应越快，滤波效果越弱。
    // @Range: 0.01 0.5
    // @User: Standard
    AP_GROUPINFO("KF_ALPHA", 20, GA_Guidance, kf_alpha, 0.15f),

    // @Param: KF_BETA
    // @DisplayName: Kalman filter velocity gain
    // @Description: Beta gain of the alpha-beta filter for LOS angles.
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