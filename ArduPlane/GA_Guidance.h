#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>

/*
  Guidance parameters for FUCHONGCESHI image-guided collision mode.
  These parameters are exposed as GA_* in Mission Planner / GCS.
*/
class GA_Guidance {
public:
    GA_Guidance();

    static const struct AP_Param::GroupInfo var_info[];

    AP_Float hfov;          // camera horizontal FOV (deg)
    AP_Float vfov;          // camera vertical FOV (deg)
    AP_Float dive_pitch;    // programmed dive pitch (deg)
    AP_Int16 min_alt;       // minimum altitude for dive safety (m)
    AP_Int16 timeout_ms;    // target loss timeout (ms)
    AP_Float roll_lim;      // roll limit (deg)
    AP_Float pitch_min;     // pitch lower limit (deg)
    AP_Float pitch_max;     // pitch upper limit (deg)
    AP_Float kp_roll;       // roll P gain on bearing error
    AP_Float kd_roll;       // roll D gain on bearing rate
    AP_Float png_n;         // proportional navigation gain
    AP_Float kp_pitch;      // pitch P gain on elevation error
    AP_Int8 loss_action;    // 0=level hold, 1=loiter, 2=RTL
};
