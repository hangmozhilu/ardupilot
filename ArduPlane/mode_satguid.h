#pragma once

#include "mode.h"

#if HAL_QUADPLANE_ENABLED

/*
  SatGuid flight mode for GPS-guided collision / pass-through.

  Three-phase mission profile:
    1. VTOL takeoff to SGUID_TKOF_H relative altitude.
    2. Forward transition to fixed-wing, then cruise with random
       lateral/altitude corridor to avoid interception.
    3. Pop-up (optional) and high-speed dive through target GPS coordinate.

  Close range handling (< SGUID_CLO_DST):
    - Optional Immelmann turn to rapidly reverse heading toward target.

  Target input sources (SGUID_TGT_SRC):
    0 = parameters SGUID_TGT_LAT/LON/ALT
    1 = GCS guided target command
    2 = serial device (Fuchong binary frame type 0x01)
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

    // handle a guided target request from GCS
    bool handle_guided_request(Location target_loc) override;

protected:

    bool _enter() override;
    void _exit() override;

private:

    // mission phase state machine
    enum class State {
        TAKEOFF,       // VTOL climb to takeoff height
        TRANSITION,    // tailsitter forward transition
        CRUISE,        // fixed-wing cruise toward target
        POPUP,         // pre-dive popup climb
        DIVE,          // terminal guidance dive
        IMMELMANN,     // close range half-loop + roll reposition
        TARGET_LOSS    // target lost, execute loss_action
    } state;

    // target state estimator
    struct {
        Location loc;             // estimated target location (AMSL)
        Location last_loc;        // previous target location for differentiation
        Vector3f vel_ned;         // estimated target velocity NED (m/s)
        Vector3f raw_vel_ned;     // measured velocity from serial
        uint32_t last_update_ms;
        uint32_t last_serial_ms;
        uint32_t last_loc_update_ms;
        bool active_param;        // true if parameter source selected and valid
        bool active_gcs;          // true if GCS target received
        bool active_serial;       // true if serial frame received
    } target;

    // guidance internal state
    struct {
        float bearing_rad;        // bearing to predicted target
        float bearing_error_rad;
        float bearing_rate_rad_s;
        float elevation_rad;      // elevation to predicted target
        float elevation_error_rad;
        float distance_m;
        float last_bearing_rad;
        uint32_t last_update_ms;
        bool target_valid;
    } guidance;

    // Immelmann maneuver state
    struct {
        uint32_t start_ms;
        float entry_heading_rad;
        float entry_airspeed;
        bool top_reached;
    } imm;

    // random corridor phase (accumulated for sinusoid)
    float corridor_phase;

    // VTOL takeoff origin
    int32_t takeoff_start_alt_cm;

    // popup/dive tracking (AMSL)
    float popup_top_amsl_m;

    // serial protocol state (same UART as FUCHONGCESHI)
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
    void run_cruise();
    void run_popup();
    void run_dive();
    void run_immelmann();
    void run_target_loss();

    void compute_guidance(const Location &pred_target);
    void set_fw_attitude(float roll_cmd_rad, float pitch_cmd_rad, float throttle_pct);
    float wrap_pi(float angle) const;
    Location predict_target(float dt) const;

    // helper to get best available airspeed estimate
    float get_air_speed() const;
};

#endif  // HAL_QUADPLANE_ENABLED
