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

    AP_GROUPEND
};

GA_Guidance::GA_Guidance()
{
    AP_Param::setup_object_defaults(this, var_info);
}
