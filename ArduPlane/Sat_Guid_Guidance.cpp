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
    // @Description: Target airspeed during cruise phase and climb-out
    // @Units: m/s
    // @Range: 5 100
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("CRSPD", 1, Sat_Guid_Guidance, cruise_spd, 22.0f),

    // @Param: CRALT
    // @DisplayName: SatGuid cruise altitude
    // @Description: Relative altitude for cruise phase and fixed-wing climb-out
    // @Units: m
    // @Range: 10 1000
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("CRALT", 2, Sat_Guid_Guidance, cruise_alt, 150),

    // @Param: CLMB_ANG
    // @DisplayName: SatGuid climb angle
    // @Description: Fixed-wing climb pitch angle during climb-out phase
    // @Units: deg
    // @Range: 5 45
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("CLMB_ANG", 3, Sat_Guid_Guidance, climb_ang, 15.0f),

    // @Param: CRLATB
    // @DisplayName: SatGuid corridor lateral bias amplitude
    // @Description: Lateral sinusoidal corridor amplitude during cruise to avoid interception
    // @Units: m
    // @Range: 0 500
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("CRLATB", 4, Sat_Guid_Guidance, cruise_lat_bias, 100.0f),

    // @Param: CRALTB
    // @DisplayName: SatGuid corridor altitude bias amplitude
    // @Description: Altitude sinusoidal corridor amplitude during cruise to avoid interception
    // @Units: m
    // @Range: 0 100
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("CRALTB", 5, Sat_Guid_Guidance, cruise_alt_bias, 30.0f),

    // @Param: CRSPDB
    // @DisplayName: SatGuid corridor speed bias amplitude
    // @Description: Airspeed sinusoidal corridor amplitude during cruise to avoid interception
    // @Units: m/s
    // @Range: 0 20
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("CRSPDB", 6, Sat_Guid_Guidance, cruise_spd_bias, 5.0f),

    // @Param: C_NOISE_F
    // @DisplayName: SatGuid random corridor frequency
    // @Description: Frequency of lateral/altitude/speed sinusoidal variations during cruise
    // @Units: Hz
    // @Range: 0.001 0.5
    // @Increment: 0.001
    // @User: Standard
    AP_GROUPINFO("C_NOISE_F", 7, Sat_Guid_Guidance, course_noise_f, 0.02f),

    // @Param: DIV_DST
    // @DisplayName: SatGuid dive start distance
    // @Description: Distance to target at which final dive is started
    // @Units: m
    // @Range: 50 2000
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("DIV_DST", 8, Sat_Guid_Guidance, dive_dist, 400),

    // @Param: DIV_ANG
    // @DisplayName: SatGuid dive angle
    // @Description: Programmed dive angle, negative for descent. Used as a bias when KP_PITCH is less than 1.0; with KP_PITCH=1.0 the pitch follows the elevation to the target.
    // @Units: deg
    // @Range: -90 0
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("DIV_ANG", 9, Sat_Guid_Guidance, dive_ang, -60.0f),

    // @Param: CLO_DST
    // @DisplayName: SatGuid close range threshold
    // @Description: Distance below which cruise is skipped and the aircraft climbs high before diving
    // @Units: m
    // @Range: 100 3000
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("CLO_DST", 10, Sat_Guid_Guidance, close_dist, 300),

    // @Param: OVRSHT_SF
    // @DisplayName: SatGuid overshoot safety factor
    // @Description: Safety factor for overshoot detection. The aircraft repositions if required dive angle exceeds PITCH_MIN by this factor.
    // @Range: 1.0 2.0
    // @Increment: 0.05
    // @User: Standard
    AP_GROUPINFO("OVRSHT_SF", 11, Sat_Guid_Guidance, overshoot_sf, 1.2f),

    // @Param: REPOS_MUL
    // @DisplayName: SatGuid reposition distance multiplier
    // @Description: After overshoot, fly straight ahead for this many loiter radii before turning back
    // @Range: 1.0 10.0
    // @Increment: 0.5
    // @User: Standard
    AP_GROUPINFO("REPOS_MUL", 12, Sat_Guid_Guidance, repos_mul, 3.0f),

    // @Param: TGT_LAT
    // @DisplayName: SatGuid target latitude
    // @Description: Target latitude when SGUID_TGT_SRC = 0
    // @Units: deg
    // @Increment: 0.000001
    // @User: Standard
    AP_GROUPINFO("TGT_LAT", 13, Sat_Guid_Guidance, tgt_lat, 0.0f),

    // @Param: TGT_LON
    // @DisplayName: SatGuid target longitude
    // @Description: Target longitude when SGUID_TGT_SRC = 0
    // @Units: deg
    // @Increment: 0.000001
    // @User: Standard
    AP_GROUPINFO("TGT_LON", 14, Sat_Guid_Guidance, tgt_lon, 0.0f),

    // @Param: TGT_ALT
    // @DisplayName: SatGuid target altitude
    // @Description: Target AMSL altitude when SGUID_TGT_SRC = 0
    // @Units: m
    // @Range: -100 10000
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("TGT_ALT", 15, Sat_Guid_Guidance, tgt_alt, 0.0f),

    // @Param: TGT_SRC
    // @DisplayName: SatGuid target source
    // @Description: Source of target GPS coordinates
    // @Values: 0:Parameter,1:Serial device
    // @User: Standard
    AP_GROUPINFO("TGT_SRC", 16, Sat_Guid_Guidance, tgt_src, 0),

    // @Param: TOUT_MS
    // @DisplayName: SatGuid target loss timeout
    // @Description: Time without valid target before triggering SGUID_LOSS_ACT
    // @Units: ms
    // @Range: 100 10000
    // @Increment: 100
    // @User: Standard
    AP_GROUPINFO("TOUT_MS", 17, Sat_Guid_Guidance, tout_ms, 2000),

    // @Param: LOSS_ACT
    // @DisplayName: SatGuid target loss action
    // @Description: Action when target is lost for longer than SGUID_TOUT_MS
    // @Values: 0:LevelHold,1:Loiter,2:RTL
    // @User: Standard
    AP_GROUPINFO("LOSS_ACT", 18, Sat_Guid_Guidance, loss_action, 1),

    // @Param: KP_PITCH
    // @DisplayName: SatGuid pitch P gain
    // @Description: Pitch proportional gain on elevation error during dive. 1.0 points the nose directly at the predicted target.
    // @Range: 0 5
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("KP_PITCH", 19, Sat_Guid_Guidance, kp_pitch, 1.0f),

    // @Param: ROLL_LIM
    // @DisplayName: SatGuid roll limit
    // @Description: Maximum roll command during guidance
    // @Units: deg
    // @Range: 5 80
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ROLL_LIM", 20, Sat_Guid_Guidance, roll_lim, 45.0f),

    // @Param: PITCH_MIN
    // @DisplayName: SatGuid minimum pitch
    // @Description: Minimum allowed pitch command, also the steepest dive angle the aircraft is allowed to use
    // @Units: deg
    // @Range: -90 -25
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("PITCH_MIN", 21, Sat_Guid_Guidance, pitch_min, -80.0f),

    // @Param: PITCH_MAX
    // @DisplayName: SatGuid maximum pitch
    // @Description: Maximum allowed pitch command
    // @Units: deg
    // @Range: 0 90
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("PITCH_MAX", 22, Sat_Guid_Guidance, pitch_max, 40.0f),

    AP_GROUPEND
};

Sat_Guid_Guidance::Sat_Guid_Guidance()
{
    AP_Param::setup_object_defaults(this, var_info);
}
