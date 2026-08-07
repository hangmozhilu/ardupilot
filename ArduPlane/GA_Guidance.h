#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>

/*
  Guidance parameters for ImgGuide image-guided intercept mode.
  These parameters are exposed as GA_* in Mission Planner / GCS.
  
  ImgGuide图像引导拦截模式的制导参数表。
  这些参数在Mission Planner/GCS中以GA_*前缀暴露。
*/
class GA_Guidance {
public:
    GA_Guidance();

    static const struct AP_Param::GroupInfo var_info[];

    // ---- 相机/云台基本参数 ----
    AP_Float hfov;          // 相机水平视场角 (deg) camera horizontal FOV
    AP_Float vfov;          // 相机垂直视场角 (deg) camera vertical FOV
    AP_Int16 timeout_ms;    // 目标丢失超时 (ms) target loss timeout
    
    // ---- 姿态限制 ----
    AP_Float roll_lim;      // 滚转限制 (deg) roll limit
    AP_Float pitch_min;     // 俯仰下限 (deg) pitch lower limit
    AP_Float pitch_max;     // 俯仰上限 (deg) pitch upper limit
    
    // ---- 制导增益 ----
    AP_Float kp_roll;       // 滚转P增益（方位误差→滚转指令）roll P gain on bearing error
    AP_Float kd_roll;       // 滚转D增益（方位角速率阻尼）roll D gain on bearing rate
    AP_Float png_n;         // 比例导航增益 proportional navigation gain
    AP_Float kp_pitch;      // 俯仰P增益（俯仰误差→俯仰指令）pitch P gain on elevation error
    AP_Float kd_pitch;      // 俯仰D增益（俯仰角速率阻尼）pitch D gain on elevation rate

    // ---- 航迹角跟踪制导参数 ----
    // 纵向通道使用期望航迹角跟踪替代纯LOS俯仰误差映射，
    // 更直接地控制能量最优的爬升/俯冲轨迹。
    AP_Float pitch_track_kp;    // 航迹角跟踪P增益 (pitch command / gamma error)
    AP_Float pitch_track_kd;    // 航迹角跟踪D增益 (pitch command / gamma rate)

    // ---- FOV保持控制器参数 ----
    // 当目标接近视场边缘时，主动将目标拉回图像中心，防止目标丢失。
    AP_Float fov_margin;        // FOV软边界 (0~1, 建议0.80~0.90)
    AP_Float fov_gain;          // FOV拉回增益 (rad per unit overshoot)
    
    // ---- 目标丢失处理 ----
    AP_Int8 loss_action;    // 目标丢失动作：0=保持平飞, 1=Loiter, 2=RTL loss action

    // ===== 连续增益调度参数（LOS角度驱动，无高度/距离依赖） =====

    // ---- 增益调度 ----
    // LOS方位误差参考角 (deg)。
    // 当方位误差 > 此值时，增益最大（快速对准）；
    // 当方位误差 < 此值时，增益按比例缩小。
    // Reference bearing error angle for gain scheduling.
    AP_Float err_ref;
    
    // LOS方位角速率参考值 (deg/s)。
    // 当方位角速率 > 此值时，认为目标接近，增益开始衰减。
    // 角速率越大说明目标越近，增益越小以避免振荡。
    // Reference bearing rate for gain damping. Higher rate ≈ closer target.
    AP_Float rate_ref;
    
    // ---- 终端制导 ----
    // 终端阶段方位误差阈值 (deg)。
    // 当方位误差 < 此值 且 方位角速率 < term_rate 时，进入终端制导。
    // Terminal phase bearing error threshold.
    AP_Float term_err;
    
    // 终端阶段方位角速率阈值 (deg/s)。
    // 当方位角速率 < 此值 且 方位误差 < term_err 时，进入终端制导。
    // Terminal phase bearing rate threshold.
    AP_Float term_rate;
    
    // ---- 滚转-偏航耦合补偿 ----
    // 方向舵混合增益 (0~1)。
    // 滚转时产生方向舵补偿，减小侧滑，提高航向跟踪精度。
    // 0=关闭，0.3=轻度补偿，0.6=强力补偿。
    // Rudder mixing gain for roll-yaw coupling compensation.
    AP_Float rudder_mix;
    
    // ---- 云台安装杆臂补偿 ----
    // 云台相对于飞机重心的安装偏移量 (m)。
    // 机体坐标系：X向前，Y向右，Z向下。
    // 当云台不在重心时，飞机姿态变化会在LOS中引入虚假运动，
    // 这些参数用于补偿该效应。
    // Gimbal mounting offset relative to CG (body frame: X forward, Y right, Z down).
    AP_Float gimbal_offset_x;   // 云台前向偏移 (m)
    AP_Float gimbal_offset_y;   // 云台右侧偏移 (m)
    AP_Float gimbal_offset_z;   // 云台下方偏移 (m)
    
    // ---- Alpha-Beta滤波器参数 ----
    // Alpha-beta滤波器位置增益 (0~1)。
    // 越大响应越快，但滤波效果越弱。推荐0.1~0.3。
    // Alpha-beta filter position gain (alpha).
    AP_Float kf_alpha;
    
    // Alpha-beta滤波器速度增益 (0~1)。
    // 越大速度估计越灵敏，但噪声越大。推荐0.01~0.05。
    // Alpha-beta filter velocity gain (beta).
    AP_Float kf_beta;
};