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

// little-endian int16 from byte buffer
static inline int16_t int16_from_le(const uint8_t *b)
{
    return (int16_t)((uint16_t)b[0] | ((uint16_t)b[1] << 8));
}

ModeSatGuid::ModeSatGuid()
    : state(State::TAKEOFF),
      corridor_phase(0.0f),
      takeoff_start_alt_cm(0),
      popup_top_amsl_m(0.0f),
      uart(nullptr),
      uart_initialised(false),
      frame_idx(0),
      parse_state(0)
{
    memset(frame_buffer, 0, sizeof(frame_buffer));
    target = {};
    guidance = {};
    imm = {};
}

bool ModeSatGuid::_enter()
{
    // only the serial target source needs the FuchongTarget UART
    if (plane.sat_guid_guidance.tgt_src.get() == 2) {
        init_uart();
    }

    state = State::TAKEOFF;
    corridor_phase = 0.0f;
    popup_top_amsl_m = 0.0f;
    takeoff_start_alt_cm = 0;

    target.last_update_ms = 0;
    target.last_serial_ms = 0;
    target.last_loc_update_ms = 0;
    target.active_param = false;
    target.active_gcs = false;
    target.active_serial = false;
    target.vel_ned.zero();
    target.raw_vel_ned.zero();

    guidance.bearing_rad = 0.0f;
    guidance.bearing_error_rad = 0.0f;
    guidance.bearing_rate_rad_s = 0.0f;
    guidance.elevation_rad = 0.0f;
    guidance.elevation_error_rad = 0.0f;
    guidance.distance_m = 0.0f;
    guidance.last_bearing_rad = 0.0f;
    guidance.last_update_ms = 0;
    guidance.target_valid = false;

    imm.start_ms = 0;
    imm.entry_heading_rad = 0.0f;
    imm.entry_airspeed = 0.0f;
    imm.top_reached = false;

    plane.nav_roll_cd = 0;
    plane.nav_pitch_cd = 0;

    // SATGUID is fully autonomous; do not wait for pilot throttle
    quadplane.throttle_wait = false;

    // load parameter target if selected
    if (plane.sat_guid_guidance.tgt_src.get() == 0) {
        if (load_param_target()) {
            target.last_update_ms = AP_HAL::millis();
        } else {
            plane.gcs().send_text(MAV_SEVERITY_WARNING, "SATGUID: target coordinates not set or invalid");
        }
    }

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
    const int16_t vx_cms = int16_from_le(frame + 13);
    const int16_t vy_cms = int16_from_le(frame + 15);

    Location new_loc(lat_e7, lon_e7, alt_cm, Location::AltFrame::ABSOLUTE);
    if (!new_loc.initialised()) {
        return false;
    }

    target.loc = new_loc;
    target.raw_vel_ned.x = vx_cms * 0.01f;
    target.raw_vel_ned.y = vy_cms * 0.01f;
    target.raw_vel_ned.z = 0.0f; // estimate z from altitude filter if needed
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
                    // target state update handled in parse_gps_frame
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

bool ModeSatGuid::handle_guided_request(Location target_loc)
{
    if (!target_loc.initialised()) {
        return false;
    }
    // store as AMSL absolute
    target_loc.change_alt_frame(Location::AltFrame::ABSOLUTE);
    target.loc = target_loc;
    target.active_gcs = true;
    target.last_update_ms = AP_HAL::millis();
    plane.gcs().send_text(MAV_SEVERITY_INFO, "SATGUID: GCS target set");
    return true;
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
    const float alpha = plane.sat_guid_guidance.tgt_filt.get();

    // parameter target is persistent: reload from parameters every cycle so it
    // never times out and in-flight edits to SGUID_TGT_LAT/LON/ALT take effect
    if (plane.sat_guid_guidance.tgt_src.get() == 0 && load_param_target()) {
        target.last_update_ms = now;
    }

    // serial frame provides direct velocity; use it when valid and fast
    if (target.active_serial && target.raw_vel_ned.length() > plane.sat_guid_guidance.tgt_static_thr.get()) {
        if (alpha > 0.0f) {
            target.vel_ned = target.vel_ned * (1.0f - alpha) + target.raw_vel_ned * alpha;
        } else {
            target.vel_ned = target.raw_vel_ned;
        }
    } else if (target.last_loc_update_ms != 0 && target.last_loc.initialised() && target.loc.initialised()) {
        // differentiate successive target positions for GCS/parameter moving targets
        const float dt = (now - target.last_loc_update_ms) * 0.001f;
        if (dt > 0.0f && dt <= 1.0f) {
            const float dist_m = target.last_loc.get_distance(target.loc);
            const float bearing_rad = target.last_loc.get_bearing(target.loc);
            Vector3f new_vel_ned;
            new_vel_ned.x = (dist_m / dt) * cosf(bearing_rad);
            new_vel_ned.y = (dist_m / dt) * sinf(bearing_rad);
            int32_t alt_now_cm, alt_last_cm;
            if (target.loc.get_alt_cm(Location::AltFrame::ABSOLUTE, alt_now_cm) &&
                target.last_loc.get_alt_cm(Location::AltFrame::ABSOLUTE, alt_last_cm)) {
                new_vel_ned.z = -((alt_now_cm - alt_last_cm) * 0.01f) / dt; // NED down is positive
            } else {
                new_vel_ned.z = 0.0f;
            }
            if (new_vel_ned.length() > plane.sat_guid_guidance.tgt_static_thr.get()) {
                if (alpha > 0.0f) {
                    target.vel_ned = target.vel_ned * (1.0f - alpha) + new_vel_ned * alpha;
                } else {
                    target.vel_ned = new_vel_ned;
                }
            } else {
                target.vel_ned.zero();
            }
        }
    }

    target.last_loc = target.loc;
    target.last_loc_update_ms = now;
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

Location ModeSatGuid::predict_target(float dt) const
{
    Location pred = target.loc;
    if (dt > 0.0f && target.vel_ned.length() > plane.sat_guid_guidance.tgt_static_thr.get()) {
        // extrapolate by velocity * dt in NED (Location::offset takes metres)
        const float d_north = target.vel_ned.x * dt;
        const float d_east = target.vel_ned.y * dt;
        pred.offset(d_north, d_east);
        // altitude extrapolation
        int32_t alt_cm;
        if (pred.get_alt_cm(Location::AltFrame::ABSOLUTE, alt_cm)) {
            pred.set_alt_cm(alt_cm + int32_t(-target.vel_ned.z * dt * 100.0f), Location::AltFrame::ABSOLUTE);
        }
    }
    return pred;
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

void ModeSatGuid::compute_guidance(const Location &pred_target)
{
    const uint32_t now = AP_HAL::millis();

    // distance and bearing to predicted target (horizontal)
    guidance.distance_m = plane.current_loc.get_distance(pred_target);
    guidance.bearing_rad = plane.current_loc.get_bearing(pred_target);

    // elevation to predicted target
    float alt_diff_m = 0.0f;
    int32_t target_alt_cm, current_alt_cm;
    if (pred_target.get_alt_cm(Location::AltFrame::ABSOLUTE, target_alt_cm) &&
        plane.current_loc.get_alt_cm(Location::AltFrame::ABSOLUTE, current_alt_cm)) {
        alt_diff_m = (target_alt_cm - current_alt_cm) * 0.01f;
    }
    guidance.elevation_rad = atan2f(alt_diff_m, MAX(guidance.distance_m, 1.0f));
    guidance.elevation_error_rad = guidance.elevation_rad - radians(plane.sat_guid_guidance.dive_ang.get());

    // bearing error relative to current yaw
    const float yaw = ahrs.get_yaw();
    guidance.bearing_error_rad = wrap_pi(guidance.bearing_rad - yaw);

    // bearing rate for PNG and damping
    float dt = 0.0f;
    if (guidance.last_update_ms != 0) {
        dt = (now - guidance.last_update_ms) * 0.001f;
    }
    if (dt <= 0.0f || dt > 0.5f) {
        guidance.bearing_rate_rad_s = 0.0f;
    } else {
        guidance.bearing_rate_rad_s = wrap_pi(guidance.bearing_rad - guidance.last_bearing_rad) / dt;
    }
    guidance.last_bearing_rad = guidance.bearing_rad;
    guidance.last_update_ms = now;
}

void ModeSatGuid::set_fw_attitude(float roll_cmd_rad, float pitch_cmd_rad, float throttle_pct)
{
    const float roll_lim_rad = radians(plane.sat_guid_guidance.roll_lim.get());
    const float pitch_min_rad = radians(plane.sat_guid_guidance.pitch_min.get());
    const float pitch_max_rad = radians(plane.sat_guid_guidance.pitch_max.get());

    roll_cmd_rad = constrain_float(roll_cmd_rad, -roll_lim_rad, roll_lim_rad);
    pitch_cmd_rad = constrain_float(pitch_cmd_rad, pitch_min_rad, pitch_max_rad);

    plane.nav_roll_cd = int32_t(degrees(roll_cmd_rad) * 100.0f);
    plane.nav_pitch_cd = int32_t(degrees(pitch_cmd_rad) * 100.0f);

    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, throttle_pct * 100.0f);
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
        // target recovered, resume from cruise/dive
        state = State::CRUISE;
    }

    update_state_machine();
}

void ModeSatGuid::update_state_machine()
{
    const uint32_t now = AP_HAL::millis();

    // predict target position to account for latency
    const float latency_s = (now - target.last_update_ms) * 0.001f;
    const Location pred_target = predict_target(latency_s);
    compute_guidance(pred_target);
    guidance.target_valid = true;

    switch (state) {
    case State::TAKEOFF:
    {
        // hand over to run_takeoff for control; switch to transition on height
        int32_t current_alt_cm;
        if (!plane.current_loc.get_alt_cm(Location::AltFrame::ABSOLUTE, current_alt_cm)) {
            current_alt_cm = plane.current_loc.alt;
        }
        if (takeoff_start_alt_cm == 0) {
            takeoff_start_alt_cm = current_alt_cm;
        }
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
            state = State::CRUISE;
        }
        break;
    }

    case State::CRUISE:
    {
        const float close_dist = plane.sat_guid_guidance.close_dist.get();
        if (guidance.distance_m < close_dist) {
            if (plane.sat_guid_guidance.imm_ena.get() != 0 &&
                plane.sat_guid_guidance.imm_aspd_min.get() < get_air_speed()) {
                plane.gcs().send_text(MAV_SEVERITY_INFO, "SATGUID: close range, Immelmann");
                state = State::IMMELMANN;
                imm.start_ms = now;
                imm.entry_heading_rad = ahrs.get_yaw();
                imm.entry_airspeed = get_air_speed();
                imm.top_reached = false;
                break;
            }
        }

        if (guidance.distance_m < plane.sat_guid_guidance.dive_dist.get()) {
            plane.gcs().send_text(MAV_SEVERITY_INFO, "SATGUID: dive");
            state = State::DIVE;
            break;
        }

        if (plane.sat_guid_guidance.popup_ena.get() != 0 &&
            guidance.distance_m < plane.sat_guid_guidance.popup_dist.get()) {
            plane.gcs().send_text(MAV_SEVERITY_INFO, "SATGUID: popup");
            state = State::POPUP;
            int32_t target_alt_cm;
            if (target.loc.get_alt_cm(Location::AltFrame::ABSOLUTE, target_alt_cm)) {
                // popup top as AMSL = target AMSL + popup height above target
                popup_top_amsl_m = target_alt_cm * 0.01f + plane.sat_guid_guidance.popup_h.get();
            } else {
                popup_top_amsl_m = 0.0f;
            }
            break;
        }

        break;
    }

    case State::POPUP:
    {
        // compare current AMSL to popup top AMSL
        int32_t current_alt_cm;
        if (plane.current_loc.get_alt_cm(Location::AltFrame::ABSOLUTE, current_alt_cm) &&
            current_alt_cm * 0.01f >= popup_top_amsl_m) {
            plane.gcs().send_text(MAV_SEVERITY_INFO, "SATGUID: popup complete, dive");
            state = State::DIVE;
        }
        break;
    }

    case State::DIVE:
    {
        // terminal guidance until impact / pass-through
        break;
    }

    case State::IMMELMANN:
    {
        // run_immelmann will manage exit back to DIVE/CRUISE
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
    case State::CRUISE:
        run_cruise();
        break;
    case State::POPUP:
        run_popup();
        break;
    case State::DIVE:
        run_dive();
        break;
    case State::IMMELMANN:
        run_immelmann();
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

void ModeSatGuid::run_cruise()
{
    // Use L1 controller to fly toward predicted target with random corridor offsets

    // advance corridor phase (assumes run() called at ~50 Hz)
    const float freq = MAX(plane.sat_guid_guidance.course_noise_f.get(), 0.001f);
    corridor_phase += 2.0f * M_PI * freq * 0.05f;
    if (corridor_phase > 2.0f * M_PI) {
        corridor_phase -= 2.0f * M_PI;
    }

    const float lateral_bias_m = plane.sat_guid_guidance.cruise_lat_bias.get() * sinf(corridor_phase);
    const float alt_bias_m = plane.sat_guid_guidance.cruise_alt_bias.get() * sinf(corridor_phase + M_PI_2);

    // base cruise altitude above home (AMSL)
    float home_amsl_m = 0.0f;
    int32_t current_alt_cm;
    if (plane.current_loc.get_alt_cm(Location::AltFrame::ABSOLUTE, current_alt_cm)) {
        home_amsl_m = current_alt_cm * 0.01f - plane.relative_altitude;
    }

    Location wp = predict_target(0.0f);
    wp.offset_bearing(90.0f + degrees(guidance.bearing_rad), lateral_bias_m);
    wp.set_alt_cm(int32_t((home_amsl_m + plane.sat_guid_guidance.cruise_alt.get() + alt_bias_m) * 100.0f),
                  Location::AltFrame::ABSOLUTE);

    plane.prev_WP_loc = plane.current_loc;
    plane.next_WP_loc = wp;
    plane.set_target_altitude_location(wp);

    plane.nav_controller->update_waypoint(plane.prev_WP_loc, plane.next_WP_loc);
    plane.calc_nav_roll();
    plane.calc_nav_pitch();

    // target cruise speed
    plane.target_airspeed_cm = int32_t(plane.sat_guid_guidance.cruise_spd.get() * 100.0f);

    Mode::run();
    plane.calc_throttle();
}

void ModeSatGuid::run_popup()
{
    // climb at max pitch toward popup top altitude while continuing toward target
    Location wp = predict_target(0.0f);
    if (popup_top_amsl_m > 0.0f) {
        wp.set_alt_cm(int32_t(popup_top_amsl_m * 100.0f), Location::AltFrame::ABSOLUTE);
    }

    plane.prev_WP_loc = plane.current_loc;
    plane.next_WP_loc = wp;
    plane.set_target_altitude_location(wp);

    plane.nav_controller->update_waypoint(plane.prev_WP_loc, plane.next_WP_loc);
    plane.calc_nav_roll();
    plane.calc_nav_pitch();

    // request climb speed near TECS max climb
    plane.target_airspeed_cm = int32_t(plane.sat_guid_guidance.cruise_spd.get() * 100.0f);

    Mode::run();
    plane.calc_throttle();
}

void ModeSatGuid::run_dive()
{
    // terminal guidance: bank-to-turn + programmed dive angle + elevation correction
    const float kp_roll = plane.sat_guid_guidance.kp_roll.get();
    const float kd_roll = plane.sat_guid_guidance.kd_roll.get();
    const float png_n = plane.sat_guid_guidance.png_n.get();
    const float kp_pitch = plane.sat_guid_guidance.kp_pitch.get();

    float roll_cmd = kp_roll * guidance.bearing_error_rad
                   + kd_roll * guidance.bearing_rate_rad_s
                   + png_n * guidance.bearing_rate_rad_s;

    const float dive_pitch_rad = radians(plane.sat_guid_guidance.dive_ang.get());
    float pitch_cmd = dive_pitch_rad + kp_pitch * guidance.elevation_error_rad;

    // altitude floor safety
    if (plane.relative_altitude < plane.sat_guid_guidance.dive_min_alt.get() && is_positive(plane.relative_altitude)) {
        if (pitch_cmd < radians(-5.0f)) {
            pitch_cmd = radians(-5.0f);
        }
    }

    // full throttle for terminal pass
    set_fw_attitude(roll_cmd, pitch_cmd, plane.aparm.throttle_max.get() * 0.01f);

    Mode::run();
}

void ModeSatGuid::run_immelmann()
{
    const uint32_t now = AP_HAL::millis();
    const float dt = (now - imm.start_ms) * 0.001f;
    const float pit_rate_rads = radians(plane.sat_guid_guidance.imm_pit_rate.get());
    const float rol_rate_rads = radians(plane.sat_guid_guidance.imm_rol_rate.get());

    if (!imm.top_reached) {
        // Phase 1: pull up at max pitch rate, wings level
        float pitch_cmd = dt * pit_rate_rads;
        pitch_cmd = constrain_float(pitch_cmd, 0.0f, radians(plane.sat_guid_guidance.pitch_max.get()));
        set_fw_attitude(0.0f, pitch_cmd, 1.0f);

        // detect top (approx 90 deg inverted) or when pitch stops increasing due to limit
        if (pitch_cmd >= radians(plane.sat_guid_guidance.pitch_max.get()) * 0.95f) {
            imm.top_reached = true;
            imm.start_ms = now;  // reset timer for phase 2 roll
            imm.entry_heading_rad = ahrs.get_yaw(); // reuse as top heading
        }
    } else {
        // Phase 2: roll 180 to return upright and point back toward target
        const float roll_cmd = dt * rol_rate_rads;
        if (roll_cmd >= M_PI) {
            // Immelmann complete, resume dive toward target
            state = State::DIVE;
            imm.start_ms = 0;
        } else {
            // hold near-zero pitch while rolling
            set_fw_attitude(roll_cmd, 0.0f, 1.0f);
        }
    }

    Mode::run();
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
