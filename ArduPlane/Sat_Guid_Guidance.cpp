#include "Sat_Guid_Guidance.h"

const AP_Param::GroupInfo Sat_Guid_Guidance::var_info[] = {
    // @Param: TKOF_H
    // @DisplayName: SatGuid takeoff height
    // @Description: Relative altitude to climb in VTOL mode before transitioning to fixed wing
    // @Units: m
    // @Range: 10 500
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("TKOF_H", 0, Sat_Guid_Guidance, takeoff_h, 50),

    // @Param: CRSPD
    // @DisplayName: SatGuid cruise speed
    // @Description: Target airspeed during cruise phase
    // @Units: m/s
    // @Range: 5 100
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("CRSPD", 1, Sat_Guid_Guidance, cruise_spd, 22.0f),

    // @Param: CRALT
    // @DisplayName: SatGuid cruise altitude
    // @Description: Relative altitude for cruise phase
    // @Units: m
    // @Range: 10 1000
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("CRALT", 2, Sat_Guid_Guidance, cruise_alt, 150),

    // @Param: CRLATB
    // @DisplayName: SatGuid corridor lateral bias amplitude
    // @Description: Lateral sinusoidal corridor amplitude during cruise to avoid interception
    // @Units: m
    // @Range: 0 500
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("CRLATB", 3, Sat_Guid_Guidance, cruise_lat_bias, 100.0f),

    // @Param: CRALTB
    // @DisplayName: SatGuid cruise altitude bias amplitude
    // @Description: Altitude fluctuation amplitude during cruise to avoid interception
    // @Units: m
    // @Range: 0 100
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("CRALTB", 4, Sat_Guid_Guidance, cruise_alt_bias, 30.0f),

    // @Param: C_NOISE_F
    // @DisplayName: SatGuid random corridor frequency
    // @Description: Frequency of lateral/altitude sinusoidal variations during cruise
    // @Units: Hz
    // @Range: 0.001 0.5
    // @Increment: 0.001
    // @User: Standard
    AP_GROUPINFO("C_NOISE_F", 5, Sat_Guid_Guidance, course_noise_f, 0.02f),

    // @Param: POP_ENA
    // @DisplayName: SatGuid popup enable
    // @Description: Enable pop-up climb before final dive
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("POP_ENA", 6, Sat_Guid_Guidance, popup_ena, 1),

    // @Param: POP_DIST
    // @DisplayName: SatGuid popup trigger distance
    // @Description: Distance to target at which popup climb is initiated
    // @Units: m
    // @Range: 100 3000
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("POP_DIST", 7, Sat_Guid_Guidance, popup_dist, 800),

    // @Param: POP_H
    // @DisplayName: SatGuid popup height
    // @Description: Height above target altitude to climb during popup
    // @Units: m
    // @Range: 10 500
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("POP_H", 8, Sat_Guid_Guidance, popup_h, 120),

    // @Param: DIV_DST
    // @DisplayName: SatGuid dive start distance
    // @Description: Distance to target at which final dive is started
    // @Units: m
    // @Range: 50 2000
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("DIV_DST", 9, Sat_Guid_Guidance, dive_dist, 400),

    // @Param: DIV_ANG
    // @DisplayName: SatGuid dive angle
    // @Description: Programmed dive angle, negative for descent
    // @Units: deg
    // @Range: -80 -5
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("DIV_ANG", 10, Sat_Guid_Guidance, dive_ang, -35.0f),

    // @Param: DIV_MALT
    // @DisplayName: SatGuid minimum dive altitude
    // @Description: Minimum altitude above target during dive for safety
    // @Units: m
    // @Range: 0 500
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("DIV_MALT", 11, Sat_Guid_Guidance, dive_min_alt, 30),

    // @Param: CLO_DST
    // @DisplayName: SatGuid close range threshold
    // @Description: Distance below which close range Immelmann logic is considered
    // @Units: m
    // @Range: 100 1000
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("CLO_DST", 12, Sat_Guid_Guidance, close_dist, 300),

    // @Param: IMM_ENA
    // @DisplayName: SatGuid Immelmann enable
    // @Description: Enable Immelmann turn for close range repositioning
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("IMM_ENA", 13, Sat_Guid_Guidance, imm_ena, 1),

    // @Param: IMM_ASPMIN
    // @DisplayName: SatGuid Immelmann minimum airspeed
    // @Description: Minimum entry airspeed for Immelmann maneuver
    // @Units: m/s
    // @Range: 5 100
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("IMM_ASPMIN", 14, Sat_Guid_Guidance, imm_aspd_min, 25.0f),

    // @Param: IMM_PRATE
    // @DisplayName: SatGuid Immelmann pitch rate
    // @Description: Pitch rate during Immelmann pull-up
    // @Units: deg/s
    // @Range: 10 100
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("IMM_PRATE", 15, Sat_Guid_Guidance, imm_pit_rate, 30.0f),

    // @Param: IMM_RRATE
    // @DisplayName: SatGuid Immelmann roll rate
    // @Description: Roll rate at top of Immelmann to flip heading
    // @Units: deg/s
    // @Range: 10 200
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("IMM_RRATE", 16, Sat_Guid_Guidance, imm_rol_rate, 60.0f),

    // @Param: TGT_LAT
    // @DisplayName: SatGuid target latitude
    // @Description: Target latitude when SGUID_TGT_SRC = 0
    // @Units: deg
    // @Increment: 0.000001
    // @User: Standard
    AP_GROUPINFO("TGT_LAT", 17, Sat_Guid_Guidance, tgt_lat, 0.0f),

    // @Param: TGT_LON
    // @DisplayName: SatGuid target longitude
    // @Description: Target longitude when SGUID_TGT_SRC = 0
    // @Units: deg
    // @Increment: 0.000001
    // @User: Standard
    AP_GROUPINFO("TGT_LON", 18, Sat_Guid_Guidance, tgt_lon, 0.0f),

    // @Param: TGT_ALT
    // @DisplayName: SatGuid target altitude
    // @Description: Target AMSL altitude when SGUID_TGT_SRC = 0
    // @Units: m
    // @Range: -100 10000
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("TGT_ALT", 19, Sat_Guid_Guidance, tgt_alt, 0.0f),

    // @Param: TGT_SRC
    // @DisplayName: SatGuid target source
    // @Description: Source of target GPS coordinates
    // @Values: 0:Parameter,1:GCS command,2:Serial device
    // @User: Standard
    AP_GROUPINFO("TGT_SRC", 20, Sat_Guid_Guidance, tgt_src, 0),

    // @Param: TGT_FILT
    // @DisplayName: SatGuid target velocity filter
    // @Description: Low pass filter coefficient for target velocity estimation (0=use measured directly)
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("TGT_FILT", 21, Sat_Guid_Guidance, tgt_filt, 0.3f),

    // @Param: TGT_STATHR
    // @DisplayName: SatGuid target static speed threshold
    // @Description: Target speed below which is treated as stationary
    // @Units: m/s
    // @Range: 0 10
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("TGT_STATHR", 22, Sat_Guid_Guidance, tgt_static_thr, 0.5f),

    // @Param: TOUT_MS
    // @DisplayName: SatGuid target loss timeout
    // @Description: Time without valid target before triggering SGUID_LOSS_ACT
    // @Units: ms
    // @Range: 100 10000
    // @Increment: 100
    // @User: Standard
    AP_GROUPINFO("TOUT_MS", 23, Sat_Guid_Guidance, tout_ms, 2000),

    // @Param: LOSS_ACT
    // @DisplayName: SatGuid target loss action
    // @Description: Action when target is lost for longer than SGUID_TOUT_MS
    // @Values: 0:LevelHold,1:Loiter,2:RTL
    // @User: Standard
    AP_GROUPINFO("LOSS_ACT", 24, Sat_Guid_Guidance, loss_action, 1),

    // @Param: KP_ROLL
    // @DisplayName: SatGuid roll P gain
    // @Description: Roll proportional gain on bearing error
    // @Range: 0 5
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("KP_ROLL", 25, Sat_Guid_Guidance, kp_roll, 0.8f),

    // @Param: KD_ROLL
    // @DisplayName: SatGuid roll D gain
    // @Description: Roll damping gain on bearing rate
    // @Range: 0 5
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("KD_ROLL", 26, Sat_Guid_Guidance, kd_roll, 0.3f),

    // @Param: PNG_N
    // @DisplayName: SatGuid PNG gain
    // @Description: Proportional navigation gain on bearing rate
    // @Range: 0 5
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("PNG_N", 27, Sat_Guid_Guidance, png_n, 2.0f),

    // @Param: KP_PITCH
    // @DisplayName: SatGuid pitch P gain
    // @Description: Pitch proportional gain on elevation error
    // @Range: 0 5
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("KP_PITCH", 28, Sat_Guid_Guidance, kp_pitch, 0.3f),

    // @Param: ROLL_LIM
    // @DisplayName: SatGuid roll limit
    // @Description: Maximum roll command during guidance
    // @Units: deg
    // @Range: 5 80
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ROLL_LIM", 29, Sat_Guid_Guidance, roll_lim, 45.0f),

    // @Param: PITCH_MIN
    // @DisplayName: SatGuid minimum pitch
    // @Description: Minimum allowed pitch command
    // @Units: deg
    // @Range: -90 0
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("PITCH_MIN", 30, Sat_Guid_Guidance, pitch_min, -60.0f),

    // @Param: PITCH_MAX
    // @DisplayName: SatGuid maximum pitch
    // @Description: Maximum allowed pitch command
    // @Units: deg
    // @Range: 0 90
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("PITCH_MAX", 31, Sat_Guid_Guidance, pitch_max, 40.0f),

    AP_GROUPEND
};

Sat_Guid_Guidance::Sat_Guid_Guidance()
{
    AP_Param::setup_object_defaults(this, var_info);
}
