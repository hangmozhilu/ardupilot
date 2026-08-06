#pragma once

#include <AP_Param/AP_Param.h>
#include <AP_Common/Location.h>
#include <stdint.h>
#include <AP_Soaring/AP_Soaring.h>
#include <AP_ADSB/AP_ADSB.h>
#include <AP_Vehicle/ModeReason.h>
#include <AP_HAL/AP_HAL.h>
#include "quadplane.h"
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Mission/AP_Mission.h>
#include "pullup.h"
#include "systemid.h"

#ifndef AP_QUICKTUNE_ENABLED
#define AP_QUICKTUNE_ENABLED HAL_QUADPLANE_ENABLED
#endif

#include <AP_Quicktune/AP_Quicktune.h>

class AC_PosControl;
class AC_AttitudeControl_Multi;
class AC_Loiter;
class Mode
{
public:

    /* Do not allow copies */
    CLASS_NO_COPY(Mode);

    // Auto Pilot modes
    // ----------------
    enum Number : uint8_t {
        MANUAL        = 0,
        CIRCLE        = 1,
        STABILIZE     = 2,
        TRAINING      = 3,
        ACRO          = 4,
        FLY_BY_WIRE_A = 5,
        FLY_BY_WIRE_B = 6,
        CRUISE        = 7,
        AUTOTUNE      = 8,
        AUTO          = 10,
        RTL           = 11,
        LOITER        = 12,
        TAKEOFF       = 13,
        AVOID_ADSB    = 14,
        GUIDED        = 15,
        INITIALISING  = 16,
#if HAL_QUADPLANE_ENABLED
        QSTABILIZE    = 17,
        QHOVER        = 18,
        QLOITER       = 19,
        QLAND         = 20,
        QRTL          = 21,
#if QAUTOTUNE_ENABLED
        QAUTOTUNE     = 22,
#endif
        QACRO         = 23,
#endif
        THERMAL       = 24,
#if HAL_QUADPLANE_ENABLED
        LOITER_ALT_QLAND = 25,
        FUCHONGCESHI     = 26,
        SATGUID          = 27,
#endif
    };

    // Constructor
    Mode();

    // enter this mode, always returns true/success
    bool enter();

    // perform any cleanups required:
    void exit();

    // run controllers specific to this mode
    virtual void run();

    // returns a unique number specific to this mode
    virtual Number mode_number() const = 0;

    // returns full text name
    virtual const char *name() const = 0;

    // returns a string for this flightmode, exactly 4 bytes
    virtual const char *name4() const = 0;

    // returns true if the vehicle can be armed in this mode
    bool pre_arm_checks(size_t buflen, char *buffer) const;

    // Reset rate and steering and TECS controllers
    void reset_controllers();

    //
    // methods that sub classes should override to affect movement of the vehicle in this mode
    //

    // convert user input to targets, implement high level control for this mode
    virtual void update() = 0;

    // true for all q modes
    virtual bool is_vtol_mode() const { return false; }
    virtual bool is_vtol_man_throttle() const;
    virtual bool is_vtol_man_mode() const { return false; }

    // guided or adsb mode
    virtual bool is_guided_mode() const { return false; }

    // true if mode can have terrain following disabled by switch
    virtual bool allows_terrain_disable() const { return false; }

    // true if automatic switch to thermal mode is supported.
    virtual bool does_automatic_thermal_switch() const {return false; }

    // subclasses override this if they require navigation.
    virtual void navigate() { return; }

    // this allows certain flight modes to mix RC input with throttle
    // depending on airspeed_nudge_cm
    virtual bool allows_throttle_nudging() const { return false; }

    // true if the mode sets the vehicle destination, which controls
    // whether control input is ignored with STICK_MIXING=0
    virtual bool does_auto_navigation() const { return false; }

    // true if the mode sets the vehicle destination, which controls
    // whether control input is ignored with STICK_MIXING=0
    virtual bool does_auto_throttle() const { return false; }
    
    // true if the mode supports autotuning (via switch for modes other
    // that AUTOTUNE itself
    virtual bool mode_allows_autotuning() const { return false; }

    // method for mode specific target altitude profiles
    virtual void update_target_altitude();

    // handle a guided target request from GCS
    virtual bool handle_guided_request(Location target_loc) { return false; }

    // true if is landing 
    virtual bool is_landing() const { return false; }

    // true if is taking 
    virtual bool is_taking_off() const;

    // true if throttle min/max limits should be applied
    virtual bool use_throttle_limits() const;

    // true if voltage correction should be applied to throttle
    virtual bool use_battery_compensation() const;

#if AP_QUICKTUNE_ENABLED
    // does this mode support VTOL quicktune?
    virtual bool supports_quicktune() const { return false; }
#endif

#if AP_PLANE_SYSTEMID_ENABLED
    // does this mode support systemid?
    virtual bool supports_systemid() const { return false; }
#endif
    
protected:

    // subclasses override this to perform checks before entering the mode
    virtual bool _enter() { return true; }

    // subclasses override this to perform any required cleanup when exiting the mode
    virtual void _exit() { return; }

    // mode specific pre-arm checks
    virtual bool _pre_arm_checks(size_t buflen, char *buffer) const;

    // Helper to output to both k_rudder and k_steering servo functions
    void output_rudder_and_steering(float val);

    // Output pilot throttle, this is used in stabilized modes without auto throttle control
    void output_pilot_throttle();

#if HAL_QUADPLANE_ENABLED
    // References for convenience, used by QModes
    AC_PosControl*& pos_control;
    AC_AttitudeControl_Multi*& attitude_control;
    AC_Loiter*& loiter_nav;
    QuadPlane& quadplane;
    QuadPlane::PosControlState &poscontrol;
#endif
    AP_AHRS& ahrs;
};


class ModeAcro : public Mode
{
friend class ModeQAcro;
public:

    Mode::Number mode_number() const override { return Mode::Number::ACRO; }
    const char *name() const override { return "ACRO"; }
    const char *name4() const override { return "ACRO"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void run() override;

    void stabilize();

    void stabilize_quaternion();

protected:

    // ACRO controller state
    struct {
        bool locked_roll;
        bool locked_pitch;
        float locked_roll_err;
        int32_t locked_pitch_cd;
        Quaternion q;
        bool roll_active_last;
        bool pitch_active_last;
        bool yaw_active_last;
    } acro_state;

    bool _enter() override;
};

class ModeAuto : public Mode
{
public:
    friend class Plane;

    Number mode_number() const override { return Number::AUTO; }
    const char *name() const override { return "AUTO"; }
    const char *name4() const override { return "AUTO"; }

    bool does_automatic_thermal_switch() const override { return true; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void navigate() override;

    bool allows_throttle_nudging() const override { return true; }

    bool does_auto_navigation() const override;

    bool does_auto_throttle() const override;
    
    bool mode_allows_autotuning() const override { return true; }

    bool is_landing() const override;

    void do_nav_delay(const AP_Mission::Mission_Command& cmd);
    bool verify_nav_delay(const AP_Mission::Mission_Command& cmd);

    bool verify_altitude_wait(const AP_Mission::Mission_Command& cmd);

    void run() override;

#if AP_PLANE_GLIDER_PULLUP_ENABLED
    bool in_pullup() const { return pullup.in_pullup(); }
#endif

protected:

    bool _enter() override;
    void _exit() override;
    bool _pre_arm_checks(size_t buflen, char *buffer) const override;

private:

    // Delay the next navigation command
    struct {
        uint32_t time_max_ms;
        uint32_t time_start_ms;
    } nav_delay;

    // wiggle state and timer for NAV_ALTITUDE_WAIT
    void wiggle_servos();
    struct {
        uint8_t stage;
        uint32_t last_ms;
    } wiggle;

#if AP_PLANE_GLIDER_PULLUP_ENABLED
    GliderPullup pullup;
#endif // AP_PLANE_GLIDER_PULLUP_ENABLED
};


class ModeAutoTune : public Mode
{
public:

    Number mode_number() const override { return Number::AUTOTUNE; }
    const char *name() const override { return "AUTOTUNE"; }
    const char *name4() const override { return "ATUN"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;
    
    bool mode_allows_autotuning() const override { return true; }

    void run() override;

protected:

    bool _enter() override;
};

class ModeGuided : public Mode
{
public:

    Number mode_number() const override { return Number::GUIDED; }
    const char *name() const override { return "GUIDED"; }
    const char *name4() const override { return "GUID"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void navigate() override;

    virtual bool is_guided_mode() const override { return true; }

    bool allows_throttle_nudging() const override { return true; }

    bool does_auto_navigation() const override { return true; }

    bool does_auto_throttle() const override { return true; }

    // handle a guided target request from GCS
    bool handle_guided_request(Location target_loc) override;

    void set_radius_and_direction(const float radius, const bool direction_is_ccw);

    void update_target_altitude() override;

protected:

    bool _enter() override;
    bool _pre_arm_checks(size_t buflen, char *buffer) const override { return true; }
#if AP_QUICKTUNE_ENABLED
    bool supports_quicktune() const override { return true; }
#endif

private:
    float active_radius_m;
};

class ModeCircle: public Mode
{
public:

    Number mode_number() const override { return Number::CIRCLE; }
    const char *name() const override { return "CIRCLE"; }
    const char *name4() const override { return "CIRC"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    bool does_auto_navigation() const override { return true; }

    bool does_auto_throttle() const override { return true; }

protected:

    bool _enter() override;
};

class ModeLoiter : public Mode
{
public:

    Number mode_number() const override { return Number::LOITER; }
    const char *name() const override { return "LOITER"; }
    const char *name4() const override { return "LOIT"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void navigate() override;

    bool isHeadingLinedUp(const Location loiterCenterLoc, const Location targetLoc);
    bool isHeadingLinedUp_cd(const int32_t bearing_cd, const int32_t heading_cd);
    bool isHeadingLinedUp_cd(const int32_t bearing_cd);

    bool allows_throttle_nudging() const override { return true; }

    bool does_auto_navigation() const override { return true; }

    bool does_auto_throttle() const override { return true; }

    bool allows_terrain_disable() const override { return true; }

    void update_target_altitude() override;
    
    bool mode_allows_autotuning() const override { return true; }

protected:

    bool _enter() override;
};

#if HAL_QUADPLANE_ENABLED
class ModeLoiterAltQLand : public ModeLoiter
{
public:

    Number mode_number() const override { return Number::LOITER_ALT_QLAND; }
    const char *name() const override { return "Loiter to QLAND"; }
    const char *name4() const override { return "L2QL"; }

    // handle a guided target request from GCS
    bool handle_guided_request(Location target_loc) override;

protected:
    bool _enter() override;

    void navigate() override;

private:
    void switch_qland();

};
#endif // HAL_QUADPLANE_ENABLED

class ModeManual : public Mode
{
public:

    Number mode_number() const override { return Number::MANUAL; }
    const char *name() const override { return "MANUAL"; }
    const char *name4() const override { return "MANU"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void run() override;

    // true if throttle min/max limits should be applied
    bool use_throttle_limits() const override;

    // true if voltage correction should be applied to throttle
    bool use_battery_compensation() const override { return false; }

};


class ModeRTL : public Mode
{
public:

    Number mode_number() const override { return Number::RTL; }
    const char *name() const override { return "RTL"; }
    const char *name4() const override { return "RTL "; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void navigate() override;

    bool allows_throttle_nudging() const override { return true; }

    bool does_auto_navigation() const override { return true; }

    bool does_auto_throttle() const override { return true; }

protected:

    bool _enter() override;
    bool _pre_arm_checks(size_t buflen, char *buffer) const override { return false; }

private:

    // Switch to QRTL if enabled and within radius
    bool switch_QRTL();
};

class ModeStabilize : public Mode
{
public:

    Number mode_number() const override { return Number::STABILIZE; }
    const char *name() const override { return "STABILIZE"; }
    const char *name4() const override { return "STAB"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void run() override;

private:
    void stabilize_stick_mixing_direct();

};

class ModeTraining : public Mode
{
public:

    Number mode_number() const override { return Number::TRAINING; }
    const char *name() const override { return "TRAINING"; }
    const char *name4() const override { return "TRAN"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void run() override;

};

class ModeInitializing : public Mode
{
public:

    Number mode_number() const override { return Number::INITIALISING; }
    const char *name() const override { return "INITIALISING"; }
    const char *name4() const override { return "INIT"; }

    bool _enter() override { return false; }

    // methods that affect movement of the vehicle in this mode
    void update() override { }

    bool allows_throttle_nudging() const override { return true; }

    bool does_auto_throttle() const override { return true; }

protected:
    bool _pre_arm_checks(size_t buflen, char *buffer) const override { return false; }

};

class ModeFBWA : public Mode
{
public:

    Number mode_number() const override { return Number::FLY_BY_WIRE_A; }
    const char *name() const override { return "FLY_BY_WIRE_A"; }
    const char *name4() const override { return "FBWA"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;
    
    bool mode_allows_autotuning() const override { return true; }

    void run() override;

};

class ModeFBWB : public Mode
{
public:

    Number mode_number() const override { return Number::FLY_BY_WIRE_B; }
    const char *name() const override { return "FLY_BY_WIRE_B"; }
    const char *name4() const override { return "FBWB"; }

    bool allows_terrain_disable() const override { return true; }

    bool does_automatic_thermal_switch() const override { return true; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    bool does_auto_throttle() const override { return true; }
    
    bool mode_allows_autotuning() const override { return true; }

    void update_target_altitude() override {};

protected:

    bool _enter() override;
};

class ModeCruise : public Mode
{
public:

    Number mode_number() const override { return Number::CRUISE; }
    const char *name() const override { return "CRUISE"; }
    const char *name4() const override { return "CRUS"; }

    bool allows_terrain_disable() const override { return true; }

    bool does_automatic_thermal_switch() const override { return true; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void navigate() override;

    bool get_target_heading_cd(int32_t &target_heading) const;

    bool does_auto_throttle() const override { return true; }

    void update_target_altitude() override {};

protected:

    bool _enter() override;

    bool locked_heading;
    int32_t locked_heading_cd;
    uint32_t lock_timer_ms;
};

#if HAL_ADSB_ENABLED
class ModeAvoidADSB : public Mode
{
public:

    Number mode_number() const override { return Number::AVOID_ADSB; }
    const char *name() const override { return "AVOID_ADSB"; }
    const char *name4() const override { return "AVOI"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void navigate() override;

    virtual bool is_guided_mode() const override { return true; }

    bool does_auto_throttle() const override { return true; }

protected:

    bool _enter() override;
};
#endif

#if HAL_QUADPLANE_ENABLED
class ModeQStabilize : public Mode
{
public:

    Number mode_number() const override { return Number::QSTABILIZE; }
    const char *name() const override { return "QSTABILIZE"; }
    const char *name4() const override { return "QSTB"; }

    bool is_vtol_mode() const override { return true; }
    bool is_vtol_man_throttle() const override { return true; }
    virtual bool is_vtol_man_mode() const override { return true; }
    bool allows_throttle_nudging() const override { return true; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    // used as a base class for all Q modes
    bool _enter() override;

    void run() override;

#if AP_PLANE_SYSTEMID_ENABLED
    // does this mode support systemid?
    bool supports_systemid() const override { return true; }
#endif
    
protected:
private:

    void set_tailsitter_roll_pitch(const float roll_input, const float pitch_input);
    void set_limited_roll_pitch(const float roll_input, const float pitch_input);

};

class ModeQHover : public Mode
{
public:

    Number mode_number() const override { return Number::QHOVER; }
    const char *name() const override { return "QHOVER"; }
    const char *name4() const override { return "QHOV"; }

    bool is_vtol_mode() const override { return true; }
    virtual bool is_vtol_man_mode() const override { return true; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void run() override;

#if AP_PLANE_SYSTEMID_ENABLED
    // does this mode support systemid?
    bool supports_systemid() const override { return true; }
#endif
    
protected:

    bool _enter() override;
#if AP_QUICKTUNE_ENABLED
    bool supports_quicktune() const override { return true; }
#endif
};

class ModeQLoiter : public Mode
{
friend class QuadPlane;
friend class ModeQLand;
friend class Plane;

public:

    Number mode_number() const override { return Number::QLOITER; }
    const char *name() const override { return "QLOITER"; }
    const char *name4() const override { return "QLOT"; }

    bool is_vtol_mode() const override { return true; }
    virtual bool is_vtol_man_mode() const override { return true; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void run() override;

#if AP_PLANE_SYSTEMID_ENABLED
    // does this mode support systemid?
    bool supports_systemid() const override { return true; }
#endif
    
protected:

    bool _enter() override;
    uint32_t last_target_loc_set_ms;

#if AP_QUICKTUNE_ENABLED
    bool supports_quicktune() const override { return true; }
#endif
};

class ModeQLand : public Mode
{
public:
    Number mode_number() const override { return Number::QLAND; }
    const char *name() const override { return "QLAND"; }
    const char *name4() const override { return "QLND"; }

    bool is_vtol_mode() const override { return true; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void run() override;

protected:

    bool _enter() override;
    bool _pre_arm_checks(size_t buflen, char *buffer) const override { return false; }
};

class ModeQRTL : public Mode
{
public:

    Number mode_number() const override { return Number::QRTL; }
    const char *name() const override { return "QRTL"; }
    const char *name4() const override { return "QRTL"; }

    bool is_vtol_mode() const override { return true; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void run() override;

    bool does_auto_throttle() const override { return true; }

    void update_target_altitude() override;

    bool allows_throttle_nudging() const override;

    float get_VTOL_return_radius() const;

protected:

    bool _enter() override;
    bool _pre_arm_checks(size_t buflen, char *buffer) const override { return false; }

private:

    enum class SubMode {
        climb,
        RTL,
    } submode;
};

class ModeQAcro : public Mode
{
public:

    Number mode_number() const override { return Number::QACRO; }
    const char *name() const override { return "QACRO"; }
    const char *name4() const override { return "QACO"; }

    bool is_vtol_mode() const override { return true; }
    bool is_vtol_man_throttle() const override { return true; }
    virtual bool is_vtol_man_mode() const override { return true; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void run() override;

protected:

    bool _enter() override;
};

#if QAUTOTUNE_ENABLED
class ModeQAutotune : public Mode
{
public:

    Number mode_number() const override { return Number::QAUTOTUNE; }
    const char *name() const override { return "QAUTOTUNE"; }
    const char *name4() const override { return "QATN"; }

    bool is_vtol_mode() const override { return true; }
    virtual bool is_vtol_man_mode() const override { return true; }

    void run() override;

    // methods that affect movement of the vehicle in this mode
    void update() override;

protected:

    bool _enter() override;
    void _exit() override;
};
#endif  // QAUTOTUNE_ENABLED

#endif  // HAL_QUADPLANE_ENABLED

class ModeTakeoff: public Mode
{
public:
    ModeTakeoff();

    Number mode_number() const override { return Number::TAKEOFF; }
    const char *name() const override { return "TAKEOFF"; }
    const char *name4() const override { return "TKOF"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void navigate() override;

    bool allows_throttle_nudging() const override { return true; }

    bool does_auto_navigation() const override { return true; }

    bool does_auto_throttle() const override { return true; }

    // var_info for holding parameter information
    static const struct AP_Param::GroupInfo var_info[];

    AP_Int16 target_alt;
    AP_Int16 level_alt;
    AP_Float ground_pitch;

protected:
    AP_Int16 target_dist;
    AP_Int8 level_pitch;

    bool takeoff_mode_setup;
    Location start_loc;

    bool _enter() override;

private:

    // flag that we have already called autoenable fences once in MODE TAKEOFF
    bool have_autoenabled_fences;

};

#if HAL_SOARING_ENABLED

class ModeThermal: public Mode
{
public:

    Number mode_number() const override { return Number::THERMAL; }
    const char *name() const override { return "THERMAL"; }
    const char *name4() const override { return "THML"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    // Update thermal tracking and exiting logic.
    void update_soaring();

    void navigate() override;

    bool allows_throttle_nudging() const override { return true; }

    bool does_auto_navigation() const override { return true; }

    // true if we are in an auto-throttle mode, which means
    // we need to run the speed/height controller
    bool does_auto_throttle() const override { return true; }

protected:

    bool exit_heading_aligned() const;
    void restore_mode(const char *reason, ModeReason modereason);

    bool _enter() override;
};

#endif

#if HAL_QUADPLANE_ENABLED
/*
  FUCHONGCESHI flight mode for precision image-guided collision.
  Tailless tailsitter fixed-wing mode that tracks a ground target from a
  gimbal camera and dives through the target center at maximum speed.
*/
class ModeFuchongceshi : public Mode
{
public:

    ModeFuchongceshi();

    Number mode_number() const override { return Number::FUCHONGCESHI; }
    const char *name() const override { return "FUCHONGCESHI"; }
    const char *name4() const override { return "FUCH"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;
    void run() override;

    // true if we are doing automatic navigation
    bool does_auto_navigation() const override { return true; }

    // true if mode sets throttle automatically
    bool does_auto_throttle() const override { return true; }

    // 在任意模式中持续检查串口，满足条件时自动切换到此模式
    // Called from Plane::update_control_mode() every iteration
    void check_auto_switch();

protected:

    bool _enter() override;
    void _exit() override;

private:

    // ============================================================
    // 数据结构定义
    // ============================================================

    // 目标信息：从云台/相机接收的原始数据
    // Target information received from gimbal/camera
    struct {
        int32_t camera_x;         // 目标x像素坐标 target x pixel coordinate
        int32_t camera_y;         // 目标y像素坐标 target y pixel coordinate
        int32_t gimbal_yaw_cdeg;  // 云台偏航角（相对机体，0.01度，+右）gimbal yaw relative to body
        int32_t gimbal_pitch_cdeg;// 云台俯仰角（相对机体，0.01度，+上）gimbal pitch relative to body
        float confidence;         // 目标置信度 0..1 target confidence
        uint8_t object_active;    // 追踪模式标志：0x00=无目标, 0x11=不追踪, 其他非零=追踪
        uint32_t last_update_ms;  // 最后一次收到目标的时间戳
    } target;

    // 制导状态：滤波后的制导量和中间计算结果
    // Guidance state: filtered guidance quantities and intermediate results
    struct {
        float bearing_error_rad;         // 方位角误差（滤波后）filtered bearing error
        float bearing_rate_rad_s;        // 方位角速率（滤波后）filtered bearing rate
        float elevation_error_rad;       // 俯仰角误差（滤波后）filtered elevation error
        float elevation_rate_rad_s;      // 俯仰角速率（滤波后）filtered elevation rate
        float slant_range_m;             // 斜距估计值（滤波后）filtered slant range
        uint32_t last_update_ms;         // 最后一次制导更新的时间戳
        bool target_valid;               // 目标是否有效（滤波后）
        bool in_terminal_phase;          // 是否处于终端制导阶段
    } guidance;

    // ============================================================
    // Alpha-Beta滤波器（稳态卡尔曼滤波器）
    // 用于平滑和预测目标视线的角度和距离
    // Alpha-Beta filter (steady-state Kalman filter) for smoothing
    // and predicting target line-of-sight angles and range.
    // ============================================================
    struct AlphaBetaFilter {
        float x_est;    // 状态估计值（位置/角度）position estimate
        float v_est;    // 速度估计值（角速率/距离速率）velocity estimate
        bool init;      // 是否已初始化 filter initialized flag

        // 将角度归一化到 [-PI, PI]
        // Wrap angle to [-PI, PI]
        static float wrap_pi(float x) {
            while (x > M_PI) { x -= 2.0f * M_PI; }
            while (x < -M_PI) { x += 2.0f * M_PI; }
            return x;
        }

        // 重置滤波器状态
        // Reset filter state
        void reset() {
            init = false;
            x_est = 0.0f;
            v_est = 0.0f;
        }

        // 更新滤波器，返回滤波后的位置估计值
        // Update filter, returns filtered position estimate
        // meas: 测量值 measurement
        // alpha: 位置增益 position gain
        // beta: 速度增益 velocity gain
        // dt: 时间步长 (s) time step
        // circular: 是否对测量值进行角度环绕处理（用于方位角/俯仰角）
        // circular: true for circular angles (bearing/elevation)
        float update(float meas, float alpha, float beta, float dt, bool circular = false) {
            // 未初始化或时间步长异常，直接初始化
            // Not initialized or abnormal dt, initialize directly
            if (!init || dt <= 0.0f || dt > 0.5f) {
                x_est = meas;
                v_est = 0.0f;
                init = true;
                return x_est;
            }
            // 预测步骤：根据上一时刻速度外推当前时刻位置
            // Prediction: extrapolate position from previous velocity
            const float x_pred = x_est + v_est * dt;
            // 计算残差（测量值 - 预测值）
            // Compute residual (measurement - prediction)
            float residual = meas - x_pred;
            // 对循环角度（如方位角）进行环绕处理，避免±180°跳变
            // Wrap residual for circular angles to avoid ±180° jumps
            if (circular) {
                residual = wrap_pi(residual);
            }
            // 更新步骤：用alpha修正位置，用beta修正速度
            // Update: correct position with alpha, velocity with beta
            x_est = x_pred + alpha * residual;
            if (circular) {
                x_est = wrap_pi(x_est);
            }
            v_est = v_est + (beta / dt) * residual;
            return x_est;
        }

        // 预测t_go秒后的位置（用于前馈补偿）
        // Predict position after t_go seconds (for feedforward compensation)
        float predict(float t_go) const {
            return x_est + v_est * t_go;
        }
    };

    // 三个独立的滤波器：方位角、俯仰角、斜距
    // Three independent filters: bearing, elevation, slant range
    AlphaBetaFilter filt_bearing;     // 方位角误差滤波器 bearing error filter
    AlphaBetaFilter filt_elevation;   // 俯仰角误差滤波器 elevation error filter
    AlphaBetaFilter filt_range;       // 斜距滤波器 slant range filter

    // ============================================================
    // 常量定义
    // ============================================================

    // 相机/云台配置（硬编码原型默认值，后续可改为参数）
    // Camera/gimbal configuration (hardcoded defaults for prototype)
    static constexpr float CAMERA_WIDTH_PX = 640.0f;    // 相机水平像素
    static constexpr float CAMERA_HEIGHT_PX = 480.0f;   // 相机垂直像素
    static constexpr float RC_OVERRIDE_DEADZONE = 0.15f; // 遥控器超控死区

    // 终端制导阶段的滚转限制（度），近距离时限制滚转以防过冲
    // Roll limit during terminal phase (deg), to prevent overshoot
    static constexpr float TERMINAL_ROLL_LIM_DEG = 15.0f;

    // 预测时间常数 (s)，用于提前修正目标运动
    // Prediction time constant for feedforward correction of target motion
    static constexpr float T_GO_PREDICT_S = 0.3f;

    // 置信度阈值：>=0.7 且 Object_active==0x22 时自动切换到此模式
    // Confidence threshold for auto-switch: >=0.7 and Object_active==0x22
    static constexpr float CONFIDENCE_AUTO_SWITCH = 0.7f;
    // 置信度阈值：<0.3 时认为目标无效，不更新制导
    // Confidence threshold for target validity: <0.3 means target invalid
    static constexpr float CONFIDENCE_MIN_VALID = 0.3f;

    // ============================================================
    // 串口帧协议处理
    // ============================================================

    // 二进制帧缓冲区（不含帧头，仅payload+校验+帧尾）
    // Binary frame buffer (without header, payload+checksum+tail only)
    static constexpr uint8_t FRAME_BODY_LEN = 21;   // 18 payload + 1 checksum + 2 tail = 21 bytes
    uint8_t frame_buffer[FRAME_BODY_LEN];
    uint8_t frame_idx;          // 当前帧体内字节索引
    uint8_t parse_state;        // 解析状态：0=等待帧头1, 1=等待帧头2, 2=接收帧体

    bool loss_action_triggered; // 目标丢失动作是否已触发

    AP_HAL::UARTDriver *uart;   // 串口设备指针
    bool uart_initialised;      // 串口是否已初始化

    // ============================================================
    // 方法声明
    // ============================================================

    // 串口与协议
    void init_uart();
    void read_serial();
    bool validate_frame(const uint8_t *frame) const;
    bool parse_frame(const uint8_t *frame);

    // 目标有效性检查
    bool target_valid() const;
    // 自动模式切换条件检查：Object_active==0x22 && confidence>0.7
    bool should_auto_switch() const;

    // 目标丢失处理
    void handle_target_loss();

    // ---- 核心制导函数（优化后） ----

    // 主制导更新函数：像素角→机体角→LOS→滤波→控制指令
    // Main guidance update: pixel angles → body angles → LOS → filter → control commands
    void update_guidance();

    // 计算当前飞机高于目标的高度差 (m)
    // Compute height above target (m)
    float compute_height_above_target() const;

    // 估计斜距：利用高度差和俯视角进行几何估算
    // Estimate slant range using height above target and depression angle
    float estimate_slant_range(float los_pitch_earth_rad) const;

    // 计算增益缩放系数：根据斜距动态调整制导增益
    // Compute gain scaling factor based on slant range
    float compute_gain_scale(float slant_range_m) const;

    // 计算云台杆臂补偿角速率：飞机姿态角速度在杆臂上的投影
    // Compute gimbal lever-arm compensation angular rate
    void compute_gimbal_compensation(float &comp_bearing_rad_s, float &comp_elevation_rad_s) const;

    // 计算终端制导（纯追踪）的滚转和俯仰指令
    // Compute terminal guidance (pure pursuit) roll and pitch commands
    void compute_terminal_guidance(float filtered_bearing_rad,
                                   float filtered_elevation_rad,
                                   float filtered_range_m,
                                   float &roll_cmd_rad,
                                   float &pitch_cmd_rad) const;

    // 计算正常制导（比例导航+增益调度）的滚转和俯仰指令
    // Compute normal guidance (PNG + gain scheduling) roll and pitch commands
    void compute_normal_guidance(float filtered_bearing_rad,
                                 float filtered_bearing_rate_rad_s,
                                 float filtered_elevation_rad,
                                 float filtered_elevation_rate_rad_s,
                                 float filtered_range_m,
                                 float &roll_cmd_rad,
                                 float &pitch_cmd_rad) const;

    // 计算自适应俯冲角：根据高度差和水平距离动态计算
    // Compute adaptive dive pitch from height above target and horizontal distance
    float compute_adaptive_dive_pitch(float filtered_range_m,
                                          float los_pitch_earth_rad) const;
};

#include "mode_satguid.h"

#endif  // HAL_QUADPLANE_ENABLED
