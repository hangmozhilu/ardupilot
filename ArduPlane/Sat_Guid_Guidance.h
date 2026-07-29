#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>

/*
  Guidance parameters for SatGuid GPS-guided collision mode.
  These parameters are exposed as SGUID_* in Mission Planner / GCS.
*/
class Sat_Guid_Guidance {
public:
    Sat_Guid_Guidance();

    static const struct AP_Param::GroupInfo var_info[];

    // Phase 1: VTOL takeoff
    AP_Int16 takeoff_h;          // vertical takeoff target height (m, relative)

    // Phase 2: cruise
    AP_Float cruise_spd;         // cruise speed (m/s)
    AP_Int16 cruise_alt;         // cruise relative altitude (m)
    AP_Float cruise_lat_bias;    // random corridor lateral bias amplitude (m)
    AP_Float cruise_alt_bias;    // random altitude fluctuation amplitude (m)
    AP_Float course_noise_f;     // random corridor / altitude frequency (Hz)

    // Phase 3: popup and dive
    AP_Int8  popup_ena;          // enable popup before dive
    AP_Int16 popup_dist;         // distance to target to trigger popup (m)
    AP_Int16 popup_h;            // popup height above target (m)
    AP_Int16 dive_dist;          // distance to target to start dive (m)
    AP_Float dive_ang;           // programmed dive angle (deg, negative)
    AP_Int16 dive_min_alt;       // minimum altitude during dive (m)

    // Close range handling
    AP_Int16 close_dist;         // close range threshold (m)
    AP_Int8  imm_ena;            // enable Immelmann maneuver
    AP_Float imm_aspd_min;       // minimum entry airspeed for Immelmann (m/s)
    AP_Float imm_pit_rate;       // pitch rate during Immelmann pull-up (deg/s)
    AP_Float imm_rol_rate;       // roll rate at top of Immelmann (deg/s)

    // Target input
    AP_Float tgt_lat;            // target latitude (deg)
    AP_Float tgt_lon;            // target longitude (deg)
    AP_Float tgt_alt;            // target altitude (m, AMSL)
    AP_Int8  tgt_src;            // 0=parameter, 1=GCS command, 2=serial
    AP_Float tgt_filt;           // target velocity low-pass filter coefficient
    AP_Float tgt_static_thr;     // speed below this is treated as stationary (m/s)

    // Safety
    AP_Int16 tout_ms;            // target loss timeout (ms)
    AP_Int8  loss_action;        // 0=level, 1=loiter, 2=RTL

    // Guidance gains
    AP_Float kp_roll;            // roll P gain on bearing error
    AP_Float kd_roll;            // roll D gain on bearing rate
    AP_Float png_n;              // proportional navigation gain
    AP_Float kp_pitch;           // pitch P gain on elevation error

    // Limits
    AP_Float roll_lim;           // roll limit (deg)
    AP_Float pitch_min;          // minimum pitch (deg)
    AP_Float pitch_max;          // maximum pitch (deg)
};
