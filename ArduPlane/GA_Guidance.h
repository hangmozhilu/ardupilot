#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>

/*
  Guidance parameters for FUCHONGCESHI image-guided collision mode.
  These parameters are exposed as GA_* in Mission Planner / GCS.
  
  FUCHONGCESHI图像引导碰撞模式的制导参数表。
  这些参数在Mission Planner/GCS中以GA_*前缀暴露。
*/
class GA_Guidance {
public:
    GA_Guidance();

    static const struct AP_Param::GroupInfo var_info[];

    // ---- 相机/云台基本参数 ----
    AP_Float hfov;          // 相机水平视场角 (deg) camera horizontal FOV
    AP_Float vfov;          // 相机垂直视场角 (deg) camera vertical FOV
    AP_Float dive_pitch;    // 程序俯冲角 (deg)，当自适应俯冲角不可用时作为后备 programmed dive pitch
    AP_Int16 min_alt;       // 最低安全高度 (m)，低于此高度限制俯冲角 minimum altitude for dive safety
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
    
    // ---- 目标丢失处理 ----
    AP_Int8 loss_action;    // 目标丢失动作：0=保持平飞, 1=Loiter, 2=RTL loss action

    // ===== 新增参数：高精度图像制导优化 =====

    // ---- 目标高度（用于斜距估计） ----
    // 目标海拔高度 (m, AMSL)，
    // 用于计算"飞机高于目标的高度差"以估计斜距。
    // 设为0时默认使用起飞点（home）海拔。
    // Target altitude AMSL for computing height-above-target and slant range.
    AP_Float tgt_alt;
    
    // ---- 增益调度 ----
    // 增益调度参考距离 (m)。
    // 当斜距 > 此值时，使用基准增益；
    // 当斜距 < 此值时，增益按比例缩小，避免末端振荡。
    // Gain scheduling reference range. Gains are scaled down when closer.
    AP_Float gain_sched_range;
    
    // ---- 终端制导 ----
    // 终端制导距离 (m)。
    // 小于此距离时切换为"纯追踪"制导，限制滚转以避免过冲。
    // Terminal phase range. Shorter than this switches to pure pursuit.
    AP_Float terminal_range;
    
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
    
    // ---- 卡尔曼滤波器参数 ----
    // Alpha-beta滤波器位置增益 (0~1)。
    // 越大响应越快，但滤波效果越弱。推荐0.1~0.3。
    // Alpha-beta filter position gain (alpha).
    AP_Float kf_alpha;
    
    // Alpha-beta滤波器速度增益 (0~1)。
    // 越大速度估计越灵敏，但噪声越大。推荐0.01~0.05。
    // Alpha-beta filter velocity gain (beta).
    AP_Float kf_beta;
};