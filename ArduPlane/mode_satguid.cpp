#include "mode_satguid.h"
#include "Plane.h"

#if HAL_QUADPLANE_ENABLED

#include <AP_SerialManager/AP_SerialManager.h>
#include <GCS_MAVLink/GCS.h>
#include <cmath>

// little-endian int32 from byte buffer
static inline int32_t int32_from_le(const uint8_t *b)
{
    return (int32_t)((uint32_t)b[0] |
                     ((uint32_t)b[1] << 8) |
                     ((uint32_t)b[2] << 16) |
                     ((uint32_t)b[3] << 24));
}

// Optional SatGuid debug prints. Change #if 0 to #if 1 to enable.
#if 0
#define SATGUID_DBG(fmt, ...) plane.gcs().send_text(MAV_SEVERITY_DEBUG, "SATGUID: " fmt, ##__VA_ARGS__)
#else
#define SATGUID_DBG(fmt, ...) ((void)0)
#endif

ModeSatGuid::ModeSatGuid()
    : state(State::TAKEOFF),
      corridor_phase(0.0f),
      takeoff_start_alt_cm(0),
      climb_target_rel_m(0.0f),
      uart(nullptr),
      uart_initialised(false),
      frame_idx(0),
      parse_state(0)
{
    memset(frame_buffer, 0, sizeof(frame_buffer));
    target = {};
    guidance = {};
    repos = {};
}

bool ModeSatGuid::_enter()
{
    // only the serial target source needs the target UART
    if (plane.sat_guid_guidance.tgt_src.get() == 1) {
        init_uart();
    }

    // reset internal state
    corridor_phase = 0.0f;
    climb_target_rel_m = 0.0f;
    target = {};
    guidance = {};
    repos = {};

    // load parameter target immediately so the initial phase decision can plan
    if (plane.sat_guid_guidance.tgt_src.get() == 0) {
        if (load_param_target()) {
            target.last_update_ms = AP_HAL::millis();
        } else {
            plane.gcs().send_text(MAV_SEVERITY_WARNING, "SATGUID: target coordinates not set or invalid");
        }
    }

    // takeoff height is relative to the arming / takeoff point (home altitude)
    takeoff_start_alt_cm = plane.home.alt;
    if (takeoff_start_alt_cm == 0) {
        // home altitude not available yet, fall back to the current absolute altitude
        int32_t current_alt_cm = 0;
        if (plane.current_loc.get_alt_cm(Location::AltFrame::ABSOLUTE, current_alt_cm)) {
            takeoff_start_alt_cm = current_alt_cm;
        }
    }

    // Determine initial phase based on current flight configuration so that
    // SATGUID can be entered from VTOL modes (QLOITER/QHOVER) or fixed-wing
    // modes (FBWA/FBWB/etc.) without requiring a fresh VTOL takeoff.
    if (plane.quadplane.in_vtol_mode()) {
        state = State::TAKEOFF;
    } else if (plane.quadplane.in_frwd_transition()) {
        state = State::TRANSITION;
    } else {
        // Already in fixed-wing flight: skip VTOL takeoff and transition.
        state = State::CLIMB;
        if (target_valid()) {
            compute_guidance();
            select_climb_target();
        } else {
            climb_target_rel_m = float(plane.sat_guid_guidance.cruise_alt.get());
        }
        plane.gcs().send_text(MAV_SEVERITY_INFO, "SATGUID: entered in fixed-wing, skip takeoff");
    }

    plane.nav_roll_cd = 0;
    plane.nav_pitch_cd = 0;

    // SATGUID is fully autonomous; do not wait for pilot throttle
    quadplane.throttle_wait = false;

    plane.gcs().send_text(MAV_SEVERITY_INFO, "SATGUID: entered");
    return true;
}

void ModeSatGuid::_exit()
{
    plane.gcs().send_text(MAV_SEVERITY_INFO, "SATGUID: exited");
}

void ModeSatGuid::init_uart()
{
    if (uart_initialised) {
        return;
    }
    uart = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_FuchongTarget, 0);
    if (uart != nullptr) {
        uint32_t baud = AP::serialmanager().find_baudrate(AP_SerialManager::SerialProtocol_FuchongTarget, 0);
        if (baud == 0) {
            baud = 115200;
        }
        uart->begin(baud);
        plane.gcs().send_text(MAV_SEVERITY_INFO, "SATGUID: target UART @ %lu baud", (unsigned long)baud);
    } else {
        plane.gcs().send_text(MAV_SEVERITY_WARNING, "SATGUID: no target UART found");
    }
    uart_initialised = true;
}

bool ModeSatGuid::validate_frame(const uint8_t *frame, uint8_t len) const
{
    if (len < 21) {
        return false;
    }
    // verify tail
    if (frame[len - 2] != FRAME_TAIL_1 || frame[len - 1] != FRAME_TAIL_2) {
        return false;
    }
    // XOR over the payload bytes (everything between header and tail except checksum itself)
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len - 3; i++) {
        crc ^= frame[i];
    }
    return crc == frame[len - 3];
}

bool ModeSatGuid::parse_gps_frame(const uint8_t *frame)
{
    if (frame[0] != FRAME_TYPE_GPS) {
        return false;
    }
    const int32_t lat_e7 = int32_from_le(frame + 1);
    const int32_t lon_e7 = int32_from_le(frame + 5);
    const int32_t alt_cm = int32_from_le(frame + 9);
    // vx/vy are included in the frame but ignored for static targets

    Location new_loc(lat_e7, lon_e7, alt_cm, Location::AltFrame::ABSOLUTE);
    if (!new_loc.initialised()) {
        return false;
    }

    target.loc = new_loc;
    target.active_serial = true;
    target.last_serial_ms = AP_HAL::millis();
    target.last_update_ms = target.last_serial_ms;

    return true;
}

void ModeSatGuid::read_serial()
{
    if (uart == nullptr) {
        return;
    }

    const int16_t nbytes = uart->available();
    for (int16_t i = 0; i < nbytes; i++) {
        const int16_t c = uart->read();
        if (c < 0) {
            continue;
        }

        switch (parse_state) {
        case 0:
            if (c == FRAME_HEADER_1) {
                parse_state = 1;
            }
            break;

        case 1:
            if (c == FRAME_HEADER_2) {
                parse_state = 2;
                frame_idx = 0;
            } else if (c != FRAME_HEADER_1) {
                parse_state = 0;
            }
            break;

        case 2:
            frame_buffer[frame_idx++] = uint8_t(c);
            if (frame_idx >= 21) {
                // full frame body received: payload(18) + checksum(1) + tail(2)
                if (validate_frame(frame_buffer, 21) && parse_gps_frame(frame_buffer)) {
                    // target state updated in parse_gps_frame
                }
                parse_state = 0;
                frame_idx = 0;
            }
            break;

        default:
            parse_state = 0;
            frame_idx = 0;
            break;
        }
    }
}

// load target location from SGUID_TGT_* parameters; valid only when both
// latitude and longitude are non-zero
bool ModeSatGuid::load_param_target()
{
    target.loc = Location(int32_t(plane.sat_guid_guidance.tgt_lat.get() * 1.0e7f),
                          int32_t(plane.sat_guid_guidance.tgt_lon.get() * 1.0e7f),
                          int32_t(plane.sat_guid_guidance.tgt_alt.get() * 100.0f),
                          Location::AltFrame::ABSOLUTE);
    target.active_param = (!is_zero(plane.sat_guid_guidance.tgt_lat.get()) &&
                           !is_zero(plane.sat_guid_guidance.tgt_lon.get()));
    return target.active_param;
}

void ModeSatGuid::update_target_state()
{
    const uint32_t now = AP_HAL::millis();

    // parameter target is persistent: reload from parameters every cycle so it
    // never times out and in-flight edits to SGUID_TGT_LAT/LON/ALT take effect
    if (plane.sat_guid_guidance.tgt_src.get() == 0 && load_param_target()) {
        target.last_update_ms = now;
    }

    // serial target: a fresh frame refreshes the timestamp
    if (target.active_serial) {
        target.last_update_ms = target.last_serial_ms;
    }
}

bool ModeSatGuid::target_valid() const
{
    const uint32_t now = AP_HAL::millis();
    if (now - target.last_update_ms > uint32_t(plane.sat_guid_guidance.tout_ms.get())) {
        return false;
    }
    if (!target.loc.initialised()) {
        return false;
    }
    return true;
}

void ModeSatGuid::handle_target_loss()
{
    if (state == State::TARGET_LOSS) {
        return;
    }
    state = State::TARGET_LOSS;
    plane.gcs().send_text(MAV_SEVERITY_WARNING, "SATGUID: target lost");

    const uint8_t action = plane.sat_guid_guidance.loss_action.get();
    if (action == 1) {
        plane.set_mode(plane.mode_loiter, ModeReason::GCS_COMMAND);
    } else if (action == 2) {
        plane.set_mode(plane.mode_rtl, ModeReason::GCS_COMMAND);
    }
    // action == 0: stay in SATGUID with last commands (handled by state)
}

float ModeSatGuid::wrap_pi(float angle) const
{
    while (angle > M_PI) {
        angle -= 2.0f * M_PI;
    }
    while (angle < -M_PI) {
        angle += 2.0f * M_PI;
    }
    return angle;
}

void ModeSatGuid::compute_guidance()
{
    if (!target.loc.initialised()) {
        return;
    }

    const uint32_t now = AP_HAL::millis();

    // distance and bearing to target (horizontal)
    guidance.distance_m = plane.current_loc.get_distance(target.loc);
    guidance.bearing_rad = plane.current_loc.get_bearing(target.loc);

    // elevation to target
    float alt_diff_m = 0.0f;
    int32_t target_alt_cm, current_alt_cm;
    if (target.loc.get_alt_cm(Location::AltFrame::ABSOLUTE, target_alt_cm) &&
        plane.current_loc.get_alt_cm(Location::AltFrame::ABSOLUTE, current_alt_cm)) {
        alt_diff_m = (target_alt_cm - current_alt_cm) * 0.01f;
    }
    guidance.elevation_rad = atan2f(alt_diff_m, MAX(guidance.distance_m, 1.0f));

    // bearing / elevation error and rates
    const float yaw = ahrs.get_yaw();
    guidance.bearing_error_rad = wrap_pi(guidance.bearing_rad - yaw);

    float dt = 0.0f;
    if (guidance.last_update_ms != 0) {
        dt = (now - guidance.last_update_ms) * 0.001f;
    }
    if (dt <= 0.0f || dt > 0.5f) {
        guidance.bearing_rate_rad_s = 0.0f;
        guidance.elevation_rate_rad_s = 0.0f;
    } else {
        guidance.bearing_rate_rad_s = wrap_pi(guidance.bearing_rad - guidance.last_bearing_rad) / dt;
        guidance.elevation_rate_rad_s = (guidance.elevation_rad - guidance.last_elevation_rad) / dt;
    }
    guidance.last_bearing_rad = guidance.bearing_rad;
    guidance.last_elevation_rad = guidance.elevation_rad;
    guidance.last_update_ms = now;
}

void ModeSatGuid::apply_roll_limit()
{
    const int32_t roll_lim_cd = int32_t(plane.sat_guid_guidance.roll_lim.get() * 100.0f);
    plane.nav_roll_cd = constrain_int32(plane.nav_roll_cd, -roll_lim_cd, roll_lim_cd);
}

void ModeSatGuid::apply_pitch_limit()
{
    const int32_t pitch_min_cd = int32_t(plane.sat_guid_guidance.pitch_min.get() * 100.0f);
    const int32_t pitch_max_cd = int32_t(plane.sat_guid_guidance.pitch_max.get() * 100.0f);
    plane.nav_pitch_cd = constrain_int32(plane.nav_pitch_cd, pitch_min_cd, pitch_max_cd);
}

void ModeSatGuid::set_fw_waypoint(const Location &wp)
{
    plane.prev_WP_loc = plane.current_loc;
    plane.next_WP_loc = wp;
    plane.set_target_altitude_location(wp);
    plane.nav_controller->update_waypoint(plane.prev_WP_loc, plane.next_WP_loc);
}

void ModeSatGuid::select_climb_target()
{
    // Choose climb-out altitude. For close-range engagements, climb higher than
    // cruise altitude so the aircraft can execute a steep dive.
    if (guidance.distance_m < plane.sat_guid_guidance.close_dist.get()) {
        const float min_dive_ht_m = guidance.distance_m * tanf(fabsf(radians(45.0f)));
        climb_target_rel_m = MAX(float(plane.sat_guid_guidance.cruise_alt.get()),
                                 plane.relative_altitude + min_dive_ht_m + 20.0f);
    } else {
        climb_target_rel_m = float(plane.sat_guid_guidance.cruise_alt.get());
    }
}

void ModeSatGuid::update()
{
    read_serial();
    update_target_state();

    if (!target_valid()) {
        handle_target_loss();
        return;
    }

    if (state == State::TARGET_LOSS) {
        // target recovered, resume from cruise
        state = State::CRUISE;
    }

    compute_guidance();
    update_state_machine();
}

void ModeSatGuid::update_state_machine()
{
    int32_t current_alt_cm = 0;
    if (!plane.current_loc.get_alt_cm(Location::AltFrame::ABSOLUTE, current_alt_cm)) {
        current_alt_cm = plane.current_loc.alt;
    }

    switch (state) {
    case State::TAKEOFF:
    {
        const int32_t target_takeoff_alt_cm = takeoff_start_alt_cm + int32_t(plane.sat_guid_guidance.takeoff_h.get()) * 100;
        if (current_alt_cm >= target_takeoff_alt_cm) {
            plane.gcs().send_text(MAV_SEVERITY_INFO, "SATGUID: takeoff complete, starting transition");
            plane.quadplane.transition->restart();
            state = State::TRANSITION;
        }
        break;
    }

    case State::TRANSITION:
    {
        if (plane.quadplane.transition->complete()) {
            plane.gcs().send_text(MAV_SEVERITY_INFO, "SATGUID: transition complete");
            select_climb_target();
            if (guidance.distance_m < plane.sat_guid_guidance.close_dist.get()) {
                plane.gcs().send_text(MAV_SEVERITY_INFO, "SATGUID: close range, climb to %.1f m", (double)climb_target_rel_m);
            }
            state = State::CLIMB;
        }
        break;
    }

    case State::CLIMB:
    {
        // re-evaluate climb target in case target became valid after fixed-wing entry
        select_climb_target();
        if (plane.relative_altitude >= climb_target_rel_m) {
            if (guidance.distance_m < plane.sat_guid_guidance.close_dist.get()) {
                plane.gcs().send_text(MAV_SEVERITY_INFO, "SATGUID: close range climb complete, dive");
                state = State::DIVE;
            } else {
                plane.gcs().send_text(MAV_SEVERITY_INFO, "SATGUID: climb complete, cruise");
                state = State::CRUISE;
            }
        }
        break;
    }

    case State::CRUISE:
    {
        if (guidance.distance_m < plane.sat_guid_guidance.dive_dist.get()) {
            plane.gcs().send_text(MAV_SEVERITY_INFO, "SATGUID: start dive");
            state = State::DIVE;
            repos.active = false;
        }
        break;
    }

    case State::DIVE:
    {
        // overshoot detection: if the line-of-sight angle to the target is
        // steeper than the aircraft is allowed to dive, we cannot reach the
        // target before passing above it.
        const float pitch_min_rad = radians(plane.sat_guid_guidance.pitch_min.get());
        const float overshoot_sf = MAX(plane.sat_guid_guidance.overshoot_sf.get(), 1.0f);
        if (guidance.elevation_rad < pitch_min_rad / overshoot_sf) {
            plane.gcs().send_text(MAV_SEVERITY_WARNING,
                                  "SATGUID: overshoot detected (el=%.1f, lim=%.1f), reposition",
                                  (double)degrees(guidance.elevation_rad),
                                  (double)degrees(pitch_min_rad));
            state = State::REPOSITION;
            repos.active = false;
            repos.turn_started = false;
            repos.climb_complete = false;
        }
        break;
    }

    case State::REPOSITION:
    {
        // run_reposition manages its own exit back to DIVE
        break;
    }

    case State::TARGET_LOSS:
    {
        // handled in update()
        break;
    }
    }
}

float ModeSatGuid::get_air_speed() const
{
#if AP_AIRSPEED_ENABLED
    if (plane.airspeed.enabled() && plane.airspeed.healthy()) {
        return plane.airspeed.get_airspeed();
    }
#endif
    float aspeed;
    if (AP::ahrs().airspeed_estimate(aspeed)) {
        return aspeed;
    }
    return plane.ahrs.groundspeed();
}

void ModeSatGuid::run()
{
    switch (state) {
    case State::TAKEOFF:
        run_takeoff();
        break;
    case State::TRANSITION:
        run_transition();
        break;
    case State::CLIMB:
        run_climb();
        break;
    case State::CRUISE:
        run_cruise();
        break;
    case State::DIVE:
        run_dive();
        break;
    case State::REPOSITION:
        run_reposition();
        break;
    case State::TARGET_LOSS:
        run_target_loss();
        break;
    }
}

void ModeSatGuid::run_takeoff()
{
    // VTOL takeoff using quadplane position controller, similar to QHOVER
    quadplane.assist.check_VTOL_recovery();

    if (quadplane.throttle_wait) {
        quadplane.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        attitude_control->set_throttle_out(0, true, 0);
        quadplane.relax_attitude_control();
        pos_control->relax_z_controller(0);
    } else {
        // set vertical speed/accel limits
        pos_control->set_max_speed_accel_z(-quadplane.get_pilot_velocity_z_max_dn(),
                                           quadplane.pilot_speed_z_max_up * 100,
                                           quadplane.pilot_accel_z * 100);
        pos_control->set_correction_speed_accel_z(-quadplane.get_pilot_velocity_z_max_dn(),
                                                  quadplane.pilot_speed_z_max_up * 100,
                                                  quadplane.pilot_accel_z * 100);
        plane.quadplane.assign_tilt_to_fwd_thr();

        // climb at default up speed
        quadplane.set_climb_rate_cms(quadplane.wp_nav->get_default_speed_up());
        quadplane.run_z_controller();
    }

    // hold current yaw/position horizontally
    Vector2f vel, accel;
    pos_control->input_vel_accel_xy(vel, accel);
    quadplane.run_xy_controller();

    // nav roll/pitch from position controller (in tailsitter VTOL view)
    plane.nav_roll_cd = pos_control->get_roll_cd();
    plane.nav_pitch_cd = pos_control->get_pitch_cd();

    // stabilize surfaces and center rudder
    plane.stabilize_roll();
    plane.stabilize_pitch();
    output_rudder_and_steering(0.0f);

    quadplane.assist.output_spin_recovery();

    SATGUID_DBG("TKOF cur_alt=%.1f tkof_h=%d rel_alt=%.1f",
                (double)(plane.current_loc.alt * 0.01),
                (int)plane.sat_guid_guidance.takeoff_h.get(),
                (double)plane.relative_altitude);
}

void ModeSatGuid::run_transition()
{
    // tailsitter transition is handled by quadplane.transition->update() in QuadPlane::update()
    // keep attitude neutral and let transition logic run
    plane.nav_roll_cd = 0;
    plane.nav_pitch_cd = 0;

    // run fixed-wing stabilisation surfaces
    Mode::run();

    // zero rudder during transition
    output_rudder_and_steering(0.0f);
}

void ModeSatGuid::run_climb()
{
    // Fixed-wing climb toward target while gaining altitude.
    // L1 controller provides lateral guidance; pitch is held at CLMB_ANG.

    float home_amsl_m = 0.0f;
    int32_t current_alt_cm;
    if (plane.current_loc.get_alt_cm(Location::AltFrame::ABSOLUTE, current_alt_cm)) {
        home_amsl_m = current_alt_cm * 0.01f - plane.relative_altitude;
    }

    Location wp = target.loc;
    wp.set_alt_cm(int32_t((home_amsl_m + climb_target_rel_m) * 100.0f),
                  Location::AltFrame::ABSOLUTE);

    set_fw_waypoint(wp);
    plane.calc_nav_roll();
    apply_roll_limit();

    // direct pitch command for steady climb angle
    const float pitch_cmd_rad = radians(plane.sat_guid_guidance.climb_ang.get());
    plane.nav_pitch_cd = int32_t(degrees(pitch_cmd_rad) * 100.0f);
    apply_pitch_limit();

    Mode::run();

    // full throttle for best climb
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, plane.aparm.throttle_max.get());

    SATGUID_DBG("CLIMB dist=%.1f rel_alt=%.1f tgt=%.1f pitch=%d roll=%d",
                (double)guidance.distance_m,
                (double)plane.relative_altitude,
                (double)climb_target_rel_m,
                (int)plane.nav_pitch_cd,
                (int)plane.nav_roll_cd);
}

void ModeSatGuid::run_cruise()
{
    // Use L1 controller to fly toward target with random lateral/altitude/speed corridor.

    // advance corridor phase (assumes run() called at ~50 Hz)
    const float freq = MAX(plane.sat_guid_guidance.course_noise_f.get(), 0.001f);
    corridor_phase += 2.0f * M_PI * freq * 0.02f;
    if (corridor_phase > 2.0f * M_PI) {
        corridor_phase -= 2.0f * M_PI;
    }

    const float lateral_bias_m = plane.sat_guid_guidance.cruise_lat_bias.get() * sinf(corridor_phase);
    const float alt_bias_m = plane.sat_guid_guidance.cruise_alt_bias.get() * sinf(corridor_phase + M_PI_2);
    const float spd_bias_mps = plane.sat_guid_guidance.cruise_spd_bias.get() * sinf(corridor_phase + M_PI);

    // base cruise altitude above home (AMSL)
    float home_amsl_m = 0.0f;
    int32_t current_alt_cm;
    if (plane.current_loc.get_alt_cm(Location::AltFrame::ABSOLUTE, current_alt_cm)) {
        home_amsl_m = current_alt_cm * 0.01f - plane.relative_altitude;
    }

    Location wp = target.loc;
    wp.offset_bearing(90.0f + degrees(guidance.bearing_rad), lateral_bias_m);
    wp.set_alt_cm(int32_t((home_amsl_m + plane.sat_guid_guidance.cruise_alt.get() + alt_bias_m) * 100.0f),
                  Location::AltFrame::ABSOLUTE);

    set_fw_waypoint(wp);
    plane.calc_nav_roll();
    apply_roll_limit();
    plane.calc_nav_pitch();
    apply_pitch_limit();  // use SatGuid pitch limits, not PTCH_LIM_MIN/MAX

    // speed corridor
    const float cruise_spd = plane.sat_guid_guidance.cruise_spd.get();
    const float target_spd = MAX(cruise_spd + spd_bias_mps, 5.0f);
    plane.target_airspeed_cm = int32_t(target_spd * 100.0f);

    Mode::run();
    plane.calc_throttle();

    SATGUID_DBG("CRUISE dist=%.1f spd=%.1f yaw_err=%.1f roll=%d",
                (double)guidance.distance_m,
                (double)target_spd,
                (double)degrees(guidance.bearing_error_rad),
                (int)plane.nav_roll_cd);
}

void ModeSatGuid::run_dive()
{
    // Terminal guidance: L1 waypoint steering laterally, direct pitch command.
    // Pitch follows the elevation to the target (proportional navigation) so
    // the aircraft can hit the target precisely. It is constrained to the
    // configured [PITCH_MIN, PITCH_MAX] range; the user should tune DIV_DST
    // and CRALT to obtain the desired dive angle (>=25 deg when possible).

    set_fw_waypoint(target.loc);
    plane.calc_nav_roll();
    apply_roll_limit();

    const float kp_pitch = plane.sat_guid_guidance.kp_pitch.get();
    const float dive_pitch_rad = radians(plane.sat_guid_guidance.dive_ang.get());

    // blend between programmed dive angle and direct target pointing
    float pitch_cmd_rad = dive_pitch_rad + kp_pitch * (guidance.elevation_rad - dive_pitch_rad);

    // elevation rate damping
    pitch_cmd_rad += 0.1f * guidance.elevation_rate_rad_s;

    plane.nav_pitch_cd = int32_t(degrees(pitch_cmd_rad) * 100.0f);
    apply_pitch_limit();  // use SatGuid pitch limits, not PTCH_LIM_MIN/MAX

    Mode::run();

    // full throttle for terminal pass
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, plane.aparm.throttle_max.get());

    SATGUID_DBG("DIVE dist=%.1f el=%.1f pitch=%d roll=%d",
                (double)guidance.distance_m,
                (double)degrees(guidance.elevation_rad),
                (int)plane.nav_pitch_cd,
                (int)plane.nav_roll_cd);
}

void ModeSatGuid::run_reposition()
{
    const float yaw = ahrs.get_yaw();

    if (!repos.active) {
        repos.active = true;
        repos.turn_started = false;
        repos.climb_complete = false;
        repos.start_loc = plane.current_loc;
        repos.entry_heading_rad = yaw;
        plane.gcs().send_text(MAV_SEVERITY_INFO, "SATGUID: reposition start");
    }

    // Altitude above target (m). If we are below 100 m, climb to 200 m first
    // before attempting the reposition manoeuvre, to avoid hitting obstacles.
    float alt_above_target_m = 0.0f;
    int32_t current_alt_cm = 0;
    int32_t target_alt_cm = 0;
    if (plane.current_loc.get_alt_cm(Location::AltFrame::ABSOLUTE, current_alt_cm) &&
        target.loc.get_alt_cm(Location::AltFrame::ABSOLUTE, target_alt_cm)) {
        alt_above_target_m = (current_alt_cm - target_alt_cm) * 0.01f;
    }
    if (alt_above_target_m < 100.0f && !repos.climb_complete) {
        // low-altitude safety climb: wings level, pitched up at CLMB_ANG
        plane.nav_roll_cd = 0;
        const float pitch_cmd_rad = radians(plane.sat_guid_guidance.climb_ang.get());
        plane.nav_pitch_cd = int32_t(degrees(pitch_cmd_rad) * 100.0f);
        apply_pitch_limit();
        plane.target_airspeed_cm = int32_t(plane.sat_guid_guidance.cruise_spd.get() * 100.0f);

        Mode::run();
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, plane.aparm.throttle_max.get());

        if (alt_above_target_m > 200.0f) {
            repos.climb_complete = true;
            plane.gcs().send_text(MAV_SEVERITY_INFO, "SATGUID: reposition climb complete, alt=%.1f", (double)alt_above_target_m);
        }

        SATGUID_DBG("REPOS_CLIMB dist=%.1f alt_above=%.1f", (double)guidance.distance_m, (double)alt_above_target_m);
        return;
    }

    // loiter radius as the distance unit for the straight-ahead leg
    const float loiter_radius_m = MAX(fabsf(float(plane.aparm.loiter_radius)), 30.0f);
    const float straight_dist_m = plane.sat_guid_guidance.repos_mul.get() * loiter_radius_m;
    const float flown_m = plane.current_loc.get_distance(repos.start_loc);

    if (!repos.turn_started) {
        // Phase 1: level flight straight ahead
        plane.nav_roll_cd = 0;
        plane.nav_pitch_cd = 0;
        plane.target_airspeed_cm = int32_t(plane.sat_guid_guidance.cruise_spd.get() * 100.0f);

        Mode::run();
        plane.calc_throttle();

        if (flown_m >= straight_dist_m) {
            repos.turn_started = true;
            plane.gcs().send_text(MAV_SEVERITY_INFO, "SATGUID: reposition turn back");
        }

        SATGUID_DBG("REPOS_S dist=%.1f flown=%.1f", (double)guidance.distance_m, (double)flown_m);
        return;
    }

    // Phase 2: turn back toward target using L1; let TECS hold altitude
    set_fw_waypoint(target.loc);
    plane.calc_nav_roll();
    apply_roll_limit();
    plane.calc_nav_pitch();
    apply_pitch_limit();
    plane.target_airspeed_cm = int32_t(plane.sat_guid_guidance.cruise_spd.get() * 100.0f);

    Mode::run();
    plane.calc_throttle();

    // exit when roughly aligned with the target and the required dive angle is
    // comfortably within the aircraft limit (hysteresis below the entry threshold)
    const bool aligned = fabsf(wrap_pi(guidance.bearing_rad - yaw)) < radians(15.0f);
    const float pitch_min_rad = radians(plane.sat_guid_guidance.pitch_min.get());
    const float overshoot_sf = MAX(plane.sat_guid_guidance.overshoot_sf.get(), 1.0f);
    if (aligned && guidance.elevation_rad > pitch_min_rad / (overshoot_sf * 1.3f)) {
        plane.gcs().send_text(MAV_SEVERITY_INFO, "SATGUID: reposition complete, resume dive");
        repos.active = false;
        state = State::DIVE;
    }

    SATGUID_DBG("REPOS_T dist=%.1f bear_err=%.1f el=%.1f",
                (double)guidance.distance_m,
                (double)degrees(wrap_pi(guidance.bearing_rad - yaw)),
                (double)degrees(guidance.elevation_rad));
}

void ModeSatGuid::run_target_loss()
{
    // wings level, maintain altitude by TECS while loss action is handled
    plane.nav_roll_cd = 0;
    plane.nav_pitch_cd = 0;
    plane.target_airspeed_cm = int32_t(plane.sat_guid_guidance.cruise_spd.get() * 100.0f);
    Mode::run();
    plane.calc_throttle();
}

#endif  // HAL_QUADPLANE_ENABLED
