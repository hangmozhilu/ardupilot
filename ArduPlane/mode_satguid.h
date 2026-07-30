#pragma once

#include "mode.h"

#if HAL_QUADPLANE_ENABLED

/*
  SatGuid flight mode for GPS-guided precision collision with static targets.

  Supported platforms: tailsitter VTOL (e.g. Q_FRAME_CLASS=1, Q_FRAME_TYPE=1,
  Q_TAILSIT_ENABLE=2).

  Mission profile:
    1. TAKEOFF   - VTOL climb to SGUID_TKOF_H above the arming / takeoff point.
    2. TRANSITION- tailsitter forward transition to fixed-wing flight.
    3. CLIMB     - fixed-wing climb to SGUID_CRALT (or higher for close-range).
    4. CRUISE    - fixed-wing cruise toward target with random lateral/altitude/speed corridor.
    5. DIVE      - waypoint-style terminal guidance with steep dive angle (>=25 deg).
    6. REPOSITION- if too high to complete the dive, level out, fly ahead,
                   turn back and re-aim.

  Close-range handling (< SGUID_CLO_DST):
    - Skip cruise, climb high, then dive directly on the target.

  Target input sources (SGUID_TGT_SRC):
    0 = parameters SGUID_TGT_LAT/LON/ALT
    1 = serial device (Fuchong GPS frame type 0x01)

  SatGuid is independent from FUCHONGCESHI; it uses the SGUID_ parameter
  namespace and has its own guidance logic.
*/
class ModeSatGuid : public Mode
{
public:

    ModeSatGuid();

    Number mode_number() const override { return Number::SATGUID; }
    const char *name() const override { return "SATGUID"; }
    const char *name4() const override { return "SGUI"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;
    void run() override;

    bool does_auto_navigation() const override { return state != State::TAKEOFF; }
    bool does_auto_throttle() const override { return state != State::TAKEOFF && state != State::TRANSITION; }
    bool is_guided_mode() const override { return true; }

protected:

    bool _enter() override;
    void _exit() override;

private:

    // mission phase state machine
    enum class State {
        TAKEOFF,       // VTOL climb to takeoff height
        TRANSITION,    // tailsitter forward transition
        CLIMB,         // fixed-wing climb to cruise / dive-start altitude
        CRUISE,        // fixed-wing cruise toward target with corridor offsets
        DIVE,          // terminal guidance dive
        REPOSITION,    // overshoot recovery: level, turn back, re-aim
        TARGET_LOSS    // target lost, execute loss_action
    } state;

    // target state (static target, updated from parameter or serial)
    struct {
        Location loc;             // target location (AMSL)
        uint32_t last_update_ms;
        uint32_t last_serial_ms;
        bool active_param;        // true if parameter source selected and valid
        bool active_serial;       // true if serial frame received
    } target;

    // guidance internal state
    struct {
        float bearing_rad;        // bearing to target
        float bearing_error_rad;  // bearing relative to current yaw
        float bearing_rate_rad_s; // bearing rate for damping
        float elevation_rad;      // elevation to target
        float elevation_rate_rad_s; // elevation rate for damping
        float distance_m;
        float last_bearing_rad;
        float last_elevation_rad;
        uint32_t last_update_ms;
        bool target_valid;
    } guidance;

    // random corridor phase (accumulated for sinusoid)
    float corridor_phase;

    // VTOL takeoff origin (AMSL, cm)
    int32_t takeoff_start_alt_cm;

    // climb / cruise target altitude (relative to home, m)
    float climb_target_rel_m;

    // reposition state
    struct {
        bool active;
        Location start_loc;       // location where reposition started
        float entry_heading_rad;  // heading when reposition started
        bool turn_started;        // true after straight-ahead leg is complete
        bool climb_complete;      // true after low-altitude safety climb is done
    } repos;

    // true once the VTOL loiter controller has been initialised for target-loss hold
    bool target_loss_loiter_init;

    // serial protocol state (Fuchong GPS frame)
    AP_HAL::UARTDriver *uart;
    bool uart_initialised;
    uint8_t frame_buffer[32];
    uint8_t frame_idx;
    uint8_t parse_state;

    static constexpr uint8_t FRAME_HEADER_1 = 0xFA;
    static constexpr uint8_t FRAME_HEADER_2 = 0xAA;
    static constexpr uint8_t FRAME_TAIL_1 = 0xAF;
    static constexpr uint8_t FRAME_TAIL_2 = 0x55;
    static constexpr uint8_t FRAME_TYPE_GPS = 0x01;

    // Minimum horizontal distance required before a fixed-wing dive can be
    // executed. If the aircraft is closer than this (e.g. directly above the
    // target), it must reposition first to create separation. This needs to be
    // larger than a typical fixed-wing turn radius at cruise speed.
    static constexpr float MIN_DIVE_DISTANCE_M = 80.0f;

    // methods
    void init_uart();
    void read_serial();
    bool validate_frame(const uint8_t *frame, uint8_t len) const;
    bool parse_gps_frame(const uint8_t *frame);

    bool load_param_target();
    void update_target_state();
    bool target_valid() const;
    void handle_target_loss();

    void update_state_machine();
    void run_takeoff();
    void run_transition();
    void run_climb();
    void run_cruise();
    void run_dive();
    void run_reposition();
    void run_target_loss();

    void compute_guidance();
    void set_fw_waypoint(const Location &wp);
    void select_climb_target();
    void apply_roll_limit();
    void apply_pitch_limit();
    float wrap_pi(float angle) const;

    // helper to get best available airspeed estimate
    float get_air_speed() const;
};

#endif  // HAL_QUADPLANE_ENABLED
