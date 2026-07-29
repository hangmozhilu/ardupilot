#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>

/*
  Guidance parameters for SatGuid GPS-guided collision mode.
  These parameters are exposed as SGUID_* in Mission Planner / GCS.

  SatGuid is independent from FUCHONGCESHI; it uses its own parameter
  namespace (SGUID_) and target source configuration.
*/
class Sat_Guid_Guidance {
public:
    Sat_Guid_Guidance();

    static const struct AP_Param::GroupInfo var_info[];

    // Phase 1: VTOL takeoff
    AP_Int16 takeoff_h;          // vertical takeoff target height (m, relative to arming / takeoff point)

    // Phase 2: fixed-wing climb / cruise
    AP_Float cruise_spd;         // cruise target airspeed (m/s)
    AP_Int16 cruise_alt;         // cruise / climb-out altitude relative to home (m)
    AP_Float climb_ang;          // fixed-wing climb pitch angle (deg)

    // Phase 3: random corridor during cruise (lateral + altitude + speed)
    AP_Float cruise_lat_bias;    // lateral sinusoidal corridor amplitude (m)
    AP_Float cruise_alt_bias;    // altitude sinusoidal corridor amplitude (m)
    AP_Float cruise_spd_bias;    // airspeed sinusoidal corridor amplitude (m/s)
    AP_Float course_noise_f;     // corridor frequency (Hz)

    // Phase 4: terminal dive
    AP_Int16 dive_dist;          // distance to target to start final dive (m)
    AP_Float dive_ang;           // programmed dive angle (deg, negative)
    AP_Int16 close_dist;         // close-range threshold: inside this, skip cruise and climb high (m)
    AP_Float overshoot_sf;       // overshoot detection safety factor (>1.0)

    // Reposition after overshoot
    AP_Float repos_mul;          // straight-ahead distance multiplier in loiter radii

    // Target input
    AP_Float tgt_lat;            // target latitude (deg)
    AP_Float tgt_lon;            // target longitude (deg)
    AP_Float tgt_alt;            // target altitude (m, AMSL)
    AP_Int8  tgt_src;            // 0=parameter, 1=serial

    // Safety
    AP_Int16 tout_ms;            // target loss timeout (ms)
    AP_Int8  loss_action;        // 0=level, 1=loiter, 2=RTL

    // Guidance gains / limits
    AP_Float kp_pitch;           // pitch P gain on elevation error
    AP_Float roll_lim;           // roll limit (deg)
    AP_Float pitch_min;          // minimum pitch, also steepest dive limit (deg)
    AP_Float pitch_max;          // maximum pitch (deg)
};
