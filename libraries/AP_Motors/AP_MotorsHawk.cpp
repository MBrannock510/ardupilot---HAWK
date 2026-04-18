#include "AP_MotorsHawk.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Math/AP_Math.h>
#include <AP_RSSI/AP_RSSI.h>
#include <GCS_MAVLink/GCS.h>
#include <stdarg.h>
#include <stdio.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_MotorsHawk::var_info[] = {
    AP_NESTEDGROUPINFO(AP_MotorsMulticopter, 0),

    // @Param: H_CGAIN
    // @DisplayName: HAWK collective gain
    // @Description: Scales the collective throttle command before cyclic modulation
    // @Range: 0.0 2.0
    // @User: Advanced
    AP_GROUPINFO("H_CGAIN", 1, AP_MotorsHawk, _collective_gain, 1.0f),

    // @Param: H_RGAIN
    // @DisplayName: HAWK roll cyclic gain
    // @Description: Gain applied to roll harmonic modulation
    // @Range: 0.0 1.0
    // @User: Advanced
    AP_GROUPINFO("H_RGAIN", 2, AP_MotorsHawk, _cyclic_roll_gain, 0.10f),

    // @Param: H_PGAIN
    // @DisplayName: HAWK pitch cyclic gain
    // @Description: Gain applied to pitch harmonic modulation
    // @Range: 0.0 1.0
    // @User: Advanced
    AP_GROUPINFO("H_PGAIN", 3, AP_MotorsHawk, _cyclic_pitch_gain, 0.10f),

    // @Param: H_YGAIN
    // @DisplayName: HAWK yaw gain
    // @Description: Gain applied to yaw bias term
    // @Range: 0.0 1.0
    // @User: Advanced
    AP_GROUPINFO("H_YGAIN", 4, AP_MotorsHawk, _yaw_gain, 0.05f),

    // @Param: H_CMAX
    // @DisplayName: HAWK cyclic limit
    // @Description: Maximum magnitude of cyclic contribution per motor
    // @Range: 0.0 0.5
    // @User: Advanced
    AP_GROUPINFO("H_CMAX", 5, AP_MotorsHawk, _cyclic_max, 0.20f),

    // @Param: H_RP1
    // @DisplayName: HAWK motor1 roll phase
    // @Description: Roll phase offset for motor1
    // @Range: -180 180
    // @Units: deg
    // @User: Advanced
    AP_GROUPINFO("H_RP1", 6, AP_MotorsHawk, _phase_roll_deg[0], 0.0f),

    // @Param: H_RP2
    // @DisplayName: HAWK motor2 roll phase
    // @Description: Roll phase offset for motor2
    // @Range: -180 180
    // @Units: deg
    // @User: Advanced
    AP_GROUPINFO("H_RP2", 7, AP_MotorsHawk, _phase_roll_deg[1], 120.0f),

    // @Param: H_RP3
    // @DisplayName: HAWK motor3 roll phase
    // @Description: Roll phase offset for motor3
    // @Range: -180 180
    // @Units: deg
    // @User: Advanced
    AP_GROUPINFO("H_RP3", 8, AP_MotorsHawk, _phase_roll_deg[2], -120.0f),

    // @Param: H_PP1
    // @DisplayName: HAWK motor1 pitch phase
    // @Description: Pitch phase offset for motor1
    // @Range: -180 180
    // @Units: deg
    // @User: Advanced
    AP_GROUPINFO("H_PP1", 9, AP_MotorsHawk, _phase_pitch_deg[0], 90.0f),

    // @Param: H_PP2
    // @DisplayName: HAWK motor2 pitch phase
    // @Description: Pitch phase offset for motor2
    // @Range: -180 180
    // @Units: deg
    // @User: Advanced
    AP_GROUPINFO("H_PP2", 10, AP_MotorsHawk, _phase_pitch_deg[1], -30.0f),

    // @Param: H_PP3
    // @DisplayName: HAWK motor3 pitch phase
    // @Description: Pitch phase offset for motor3
    // @Range: -180 180
    // @Units: deg
    // @User: Advanced
    AP_GROUPINFO("H_PP3", 11, AP_MotorsHawk, _phase_pitch_deg[2], -150.0f),

    // @Param: H_YB1
    // @DisplayName: HAWK motor1 yaw bias
    // @Description: Relative yaw bias for motor1
    // @Range: -1.0 1.0
    // @User: Advanced
    AP_GROUPINFO("H_YB1", 12, AP_MotorsHawk, _yaw_bias[0], 1.0f),

    // @Param: H_YB2
    // @DisplayName: HAWK motor2 yaw bias
    // @Description: Relative yaw bias for motor2
    // @Range: -1.0 1.0
    // @User: Advanced
    AP_GROUPINFO("H_YB2", 13, AP_MotorsHawk, _yaw_bias[1], -0.5f),

    // @Param: H_YB3
    // @DisplayName: HAWK motor3 yaw bias
    // @Description: Relative yaw bias for motor3
    // @Range: -1.0 1.0
    // @User: Advanced
    AP_GROUPINFO("H_YB3", 14, AP_MotorsHawk, _yaw_bias[2], -0.5f),

    AP_GROUPEND
};

AP_MotorsHawk::AP_MotorsHawk(uint16_t speed_hz) :
    AP_MotorsMulticopter(speed_hz),
    _encoders_initialized(false),
    _frame_configured(false),
    _last_measured_theta_rad(0.0f),
    _avg_rotation_period_us(0.0f),
    _last_rotation_tick_us(0),
    _rotation_history_count(0),
    _rotation_history_index(0),
    _rotation_tick_valid(false),
    _theta_sample_valid(false),
    _last_debug_ms(0),
    _last_fault_ms(0),
    _last_status_ms(0),
    _last_command_log_ms(0),
    _sent_init_msg(false),
    _had_encoder_fault(false),
    _last_armed_state(false),
    _last_spool_state(SpoolState::SHUT_DOWN),
    _last_pivot_target_rad(0.0f),
    _last_pivot_pwm(HAWK_PIVOT_PWM_TRIM),
    _last_logged_roll_in(0.0f),
    _last_logged_pitch_in(0.0f),
    _last_logged_yaw_in(0.0f),
    _last_logged_throttle_in(0.0f)
{
    for (uint8_t i = 0; i < HAWK_NUM_MOTORS; i++) {
        _theta_rad[i] = 0.0f;
        _hawk_out[i] = 0.0f;
    }
    _encoder_healthy = false;
    for (uint8_t i = 0; i < ROTATION_HISTORY_LEN; i++) {
        _rotation_period_history_us[i] = 0.0f;
    }
}

const char* AP_MotorsHawk::_get_frame_string() const
{
    return "HAWK";
}

void AP_MotorsHawk::send_debug_text(MAV_SEVERITY severity, const char *fmt, ...) const
{
    char msg[96];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(msg, sizeof(msg), fmt, ap);
    va_end(ap);

    gcs().send_text(severity, "%s", msg);
}

void AP_MotorsHawk::init(motor_frame_class frame_class,
                         motor_frame_type frame_type)
{
    set_frame_class_and_type(frame_class, frame_type);

    const bool ok = (frame_class == MOTOR_FRAME_HAWK) &&
                    _encoders_initialized &&
                    SRV_Channels::function_assigned(HAWK_PIVOT_FUNCTION);
    set_initialised_ok(ok);

    if (!_sent_init_msg) {
        send_debug_text(MAV_SEVERITY_INFO,
                        "HAWK init frame=%u ok=%u",
                        (unsigned)frame_class,
                        (unsigned)ok);
        _sent_init_msg = true;
    }
}

void AP_MotorsHawk::set_frame_class_and_type(motor_frame_class frame_class,
                                             motor_frame_type frame_type)
{
    (void)frame_type;

    if (frame_class != MOTOR_FRAME_HAWK) {
        if (_frame_configured) {
            send_debug_text(MAV_SEVERITY_WARNING,
                            "HAWK wrong frame class=%u",
                            (unsigned)frame_class);
        }
        _frame_configured = false;
        _encoders_initialized = false;
        return;
    }

    if (_frame_configured && _encoders_initialized) {
        return;
    }

    _encoders_initialized = false;

    for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        motor_enabled[i] = false;
    }

    add_motor_num(AP_MOTORS_MOT_1);
    add_motor_num(AP_MOTORS_MOT_2);
    add_motor_num(AP_MOTORS_MOT_3);
    SRV_Channels::set_aux_channel_default(HAWK_PIVOT_FUNCTION, HAWK_PIVOT_CH);

    motor_enabled[MOTOR_HAWK_1] = true;
    motor_enabled[MOTOR_HAWK_2] = true;
    motor_enabled[MOTOR_HAWK_3] = true;

    configure_hardcoded_defaults();
    _encoders.init();
    _encoders_initialized = _encoders.initialized();
    _frame_configured = true;
    _rotation_history_count = 0;
    _rotation_history_index = 0;
    _avg_rotation_period_us = 0.0f;
    _rotation_tick_valid = false;
    _theta_sample_valid = false;
    _encoder_healthy = false;
    _last_pivot_target_rad = 0.0f;
    _last_pivot_pwm = HAWK_PIVOT_PWM_TRIM;
    _last_armed_state = armed();
    _last_spool_state = _spool_state;

    rc_set_freq((1U << AP_MOTORS_MOT_1) |
                (1U << AP_MOTORS_MOT_2) |
                (1U << AP_MOTORS_MOT_3),
                _speed_hz);
    SRV_Channels::set_rc_frequency(HAWK_PIVOT_FUNCTION, 50);
    SRV_Channels::set_angle(HAWK_PIVOT_FUNCTION, 1);

    send_debug_text(MAV_SEVERITY_INFO, "HAWK frame configured");
    if (_encoders_initialized) {
        send_debug_text(MAV_SEVERITY_INFO, "HAWK encoder ready bus=0 addr=0x36");
    } else {
        send_debug_text(MAV_SEVERITY_ERROR, "HAWK encoder init failed bus=0 addr=0x36");
    }
}

bool AP_MotorsHawk::arming_checks(size_t buflen, char *buffer) const
{
    if (!AP_MotorsMulticopter::arming_checks(buflen, buffer)) {
        return false;
    }

    if (!_encoders_initialized) {
        hal.util->snprintf(buffer, buflen, "HAWK encoders not initialized");
        return false;
    }

    if (!SRV_Channels::function_assigned(HAWK_PIVOT_FUNCTION)) {
        hal.util->snprintf(buffer, buflen, "HAWK pivot servo missing on S4");
        return false;
    }

    if (!_encoder_healthy) {
        hal.util->snprintf(buffer, buflen, "HAWK rotor encoder invalid");
        return false;
    }

    if (!rotor_timing_ready()) {
        hal.util->snprintf(buffer, buflen, "HAWK rotor timing not ready");
        return false;
    }

    return true;
}

bool AP_MotorsHawk::motor_test_checks(size_t buflen, char *buffer) const
{
    if (!AP_MotorsMulticopter::motor_test_checks(buflen, buffer)) {
        return false;
    }
    return true;
}

uint32_t AP_MotorsHawk::get_motor_mask()
{
    return (1U << AP_MOTORS_MOT_1) |
           (1U << AP_MOTORS_MOT_2) |
           (1U << AP_MOTORS_MOT_3) |
           SRV_Channels::get_output_channel_mask(HAWK_PIVOT_FUNCTION);
}

void AP_MotorsHawk::update_encoder_state()
{
    if (!_encoders_initialized) {
        _encoder_healthy = false;
        _theta_sample_valid = false;
        return;
    }

    _encoders.update();
    const uint32_t now_us = AP_HAL::micros();

    const bool good = _encoders.healthy(0) &&
                      !_encoders.stale(0, ENCODER_TIMEOUT_US);
    _encoder_healthy = good;

    if (good) {
        const float theta = _encoders.get_angle_rad(0);
        update_rotation_period(theta, now_us);
        const float theta_to_use = rotor_timing_ready(now_us) ? compute_predicted_theta(now_us) : theta;
        for (uint8_t i = 0; i < HAWK_NUM_MOTORS; i++) {
            _theta_rad[i] = wrap_2PI(theta_to_use + radians(120.0f * i));
        }
    }
}

bool AP_MotorsHawk::encoders_healthy() const
{
    return _encoder_healthy;
}

bool AP_MotorsHawk::rotor_timing_ready() const
{
    return rotor_timing_ready(AP_HAL::micros());
}

bool AP_MotorsHawk::rotor_timing_ready(uint32_t now_us) const
{
    if ((_rotation_history_count < ROTATION_HISTORY_LEN) ||
        !is_positive(_avg_rotation_period_us) ||
        !_rotation_tick_valid) {
        return false;
    }

    const uint32_t max_age_us = (uint32_t)(_avg_rotation_period_us * 2.5f);
    return (now_us - _last_rotation_tick_us) <= max_age_us;
}

void AP_MotorsHawk::update_rotation_period(float theta_rad, uint32_t sample_time_us)
{
    if (!_theta_sample_valid) {
        _last_measured_theta_rad = theta_rad;
        _theta_sample_valid = true;
        return;
    }

    const bool wrapped = (_last_measured_theta_rad > radians(300.0f)) &&
                         (theta_rad < radians(60.0f));
    _last_measured_theta_rad = theta_rad;

    if (!wrapped) {
        return;
    }

    if (!_rotation_tick_valid) {
        _last_rotation_tick_us = sample_time_us;
        _rotation_tick_valid = true;
        return;
    }

    const uint32_t dt_us = sample_time_us - _last_rotation_tick_us;
    _last_rotation_tick_us = sample_time_us;

    if ((dt_us < 1000U) || (dt_us > 2000000U)) {
        return;
    }

    _rotation_period_history_us[_rotation_history_index] = dt_us;
    _rotation_history_index = (_rotation_history_index + 1U) % ROTATION_HISTORY_LEN;
    if (_rotation_history_count < ROTATION_HISTORY_LEN) {
        _rotation_history_count++;
    }

    float sum = 0.0f;
    for (uint8_t i = 0; i < _rotation_history_count; i++) {
        sum += _rotation_period_history_us[i];
    }
    _avg_rotation_period_us = sum / (float)_rotation_history_count;
}

float AP_MotorsHawk::compute_predicted_theta(uint32_t now_us) const
{
    if (!_rotation_tick_valid || !is_positive(_avg_rotation_period_us)) {
        return _last_measured_theta_rad;
    }

    const float phase_rotations = (float)(now_us - _last_rotation_tick_us) / _avg_rotation_period_us;
    return wrap_2PI(phase_rotations * (2.0f * M_PI));
}

float AP_MotorsHawk::compute_collective_thrust(float throttle_in) const
{
    return constrain_float(throttle_in * _collective_gain, 0.0f, 1.0f);
}

float AP_MotorsHawk::compute_cyclic_term(uint8_t motor_idx,
                                         float theta_rad,
                                         float roll_in,
                                         float pitch_in,
                                         float yaw_in) const
{
    const float roll_phase_rad  = radians((float)_phase_roll_deg[motor_idx]);
    const float pitch_phase_rad = radians((float)_phase_pitch_deg[motor_idx]);

    const float roll_term =
        constrain_float(roll_in, -1.0f, 1.0f) *
        _cyclic_roll_gain *
        cosf(theta_rad - roll_phase_rad);

    const float pitch_term =
        constrain_float(pitch_in, -1.0f, 1.0f) *
        _cyclic_pitch_gain *
        cosf(theta_rad - pitch_phase_rad);

    const float yaw_term =
        constrain_float(yaw_in, -1.0f, 1.0f) *
        _yaw_gain *
        _yaw_bias[motor_idx];

    const float total = roll_term + pitch_term + yaw_term;
    return constrain_float(total, -(float)_cyclic_max, (float)_cyclic_max);
}

float AP_MotorsHawk::apply_output_limits(float in) const
{
    return constrain_float(in, 0.0f, 1.0f);
}

void AP_MotorsHawk::set_actuator_safe()
{
    for (uint8_t i = 0; i < HAWK_NUM_MOTORS; i++) {
        _hawk_out[i] = 0.0f;
    }
}

void AP_MotorsHawk::set_pivot_servo_safe()
{
    _last_pivot_pwm = HAWK_PIVOT_PWM_TRIM;
    SRV_Channels::set_output_pwm_trimmed(HAWK_PIVOT_FUNCTION, HAWK_PIVOT_PWM_TRIM);
}

void AP_MotorsHawk::configure_hardcoded_defaults()
{
    AP_Param::set_default_by_name("SERVO1_FUNCTION", SRV_Channel::k_motor1);
    AP_Param::set_default_by_name("SERVO2_FUNCTION", SRV_Channel::k_motor2);
    AP_Param::set_default_by_name("SERVO3_FUNCTION", SRV_Channel::k_motor3);
    AP_Param::set_default_by_name("SERVO4_FUNCTION", HAWK_PIVOT_FUNCTION);
    AP_Param::set_default_by_name("SERVO4_MIN", HAWK_PIVOT_PWM_MIN);
    AP_Param::set_default_by_name("SERVO4_TRIM", HAWK_PIVOT_PWM_TRIM);
    AP_Param::set_default_by_name("SERVO4_MAX", HAWK_PIVOT_PWM_MAX);
    AP_Param::set_default_by_name("MOT_THST_HOVER", 0.35f);
    AP_Param::set_default_by_name("MOT_THST_EXPO", 0.65f);
}

void AP_MotorsHawk::update_pivot_servo(float theta_rad,
                                       float roll_in,
                                       float pitch_in)
{
    if (!SRV_Channels::function_assigned(HAWK_PIVOT_FUNCTION)) {
        return;
    }

    if (!is_zero(roll_in) || !is_zero(pitch_in)) {
        _last_pivot_target_rad = wrap_2PI(atan2f(pitch_in, roll_in));
    }

    const float phase_error = wrap_PI(_last_pivot_target_rad - theta_rad);
    const float drive = constrain_float(phase_error * HAWK_PIVOT_PHASE_GAIN, -1.0f, 1.0f);
    const int16_t pwm = HAWK_PIVOT_PWM_TRIM + (int16_t)(drive * HAWK_PIVOT_PWM_SPAN);
    _last_pivot_pwm = pwm;
    SRV_Channels::set_output_pwm_trimmed(HAWK_PIVOT_FUNCTION, pwm);
}

void AP_MotorsHawk::send_state_change_debug_if_needed()
{
    const bool armed_now = armed();
    if (armed_now != _last_armed_state) {
        send_debug_text(MAV_SEVERITY_INFO, "HAWK armed=%u", (unsigned)armed_now);
        _last_armed_state = armed_now;
    }

    if (_spool_state != _last_spool_state) {
        send_debug_text(MAV_SEVERITY_INFO,
                        "HAWK spool=%u desired=%u",
                        (unsigned)_spool_state,
                        (unsigned)_spool_desired);
        _last_spool_state = _spool_state;
    }
}

void AP_MotorsHawk::send_status_debug_if_due()
{
    const uint32_t now = AP_HAL::millis();
    if (now - _last_debug_ms < 30000) {
        return;
    }
    _last_debug_ms = now;

    AP_BattMonitor &battery = AP::battery();
    float current_amps = 0.0f;
    const bool have_current = battery.current_amps(current_amps);
    const AP_GPS &gps = AP::gps();
    AP_RSSI *rssi_backend = AP::rssi();
    const float receiver_rssi = (rssi_backend != nullptr) ? rssi_backend->read_receiver_rssi() : -1.0f;

    send_debug_text(MAV_SEVERITY_INFO,
                    "HAWK st arm=%u sp=%u enc=%u tim=%u gps=%u sat=%u V=%.2f I=%.1f rssi=%.2f",
                    (unsigned)armed(),
                    (unsigned)_spool_state,
                    (unsigned)_encoder_healthy,
                    (unsigned)rotor_timing_ready(),
                    (unsigned)gps.status(),
                    (unsigned)gps.num_sats(),
                    (double)battery.voltage(),
                    (double)(have_current ? current_amps : -1.0f),
                    (double)receiver_rssi);
}

void AP_MotorsHawk::send_encoder_fault_if_needed()
{
    if (!armed()) {
        _had_encoder_fault = false;
        return;
    }

    const bool bad = !encoders_healthy() || !rotor_timing_ready();
    const uint32_t now = AP_HAL::millis();

    if (bad) {
        if (!_had_encoder_fault || (now - _last_fault_ms) > 2000) {
            send_debug_text(MAV_SEVERITY_WARNING,
                            "HAWK encoder/timing fault h=%u t=%u",
                            (unsigned)_encoder_healthy,
                            (unsigned)rotor_timing_ready());
            _last_fault_ms = now;
            _had_encoder_fault = true;
        }
    } else if (_had_encoder_fault) {
        send_debug_text(MAV_SEVERITY_INFO, "HAWK encoder fault cleared");
        _had_encoder_fault = false;
    }
}

void AP_MotorsHawk::log_status_if_due()
{
#if HAL_LOGGING_ENABLED
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - _last_status_ms < 30000) {
        return;
    }

    AP_BattMonitor &battery = AP::battery();
    float current_amps = 0.0f;
    const bool have_current = battery.current_amps(current_amps);
    const AP_GPS &gps = AP::gps();
    AP_RSSI *rssi_backend = AP::rssi();
    const float receiver_rssi = (rssi_backend != nullptr) ? rssi_backend->read_receiver_rssi() : -1.0f;

    AP::logger().WriteStreaming("HWKS",
                                "TimeUS,Arm,Spool,EncOk,TimOk,GPS,Sats,BatV,BatI,RSSI,Tms,Theta,PPWM",
                                "QBBBBBBfffffh",
                                AP_HAL::micros64(),
                                (uint8_t)armed(),
                                (uint8_t)_spool_state,
                                (uint8_t)_encoder_healthy,
                                (uint8_t)rotor_timing_ready(),
                                (uint8_t)gps.status(),
                                (uint8_t)gps.num_sats(),
                                battery.voltage(),
                                have_current ? current_amps : -1.0f,
                                receiver_rssi,
                                _avg_rotation_period_us * 0.001f,
                                degrees(_theta_rad[0]),
                                _last_pivot_pwm);

    _last_status_ms = now_ms;
#endif
}

void AP_MotorsHawk::log_command_if_needed()
{
#if HAL_LOGGING_ENABLED
    const uint32_t now_ms = AP_HAL::millis();
    const bool due = (now_ms - _last_command_log_ms) > 5000U;
    const bool changed =
        (fabsf(_roll_in - _last_logged_roll_in) > 0.05f) ||
        (fabsf(_pitch_in - _last_logged_pitch_in) > 0.05f) ||
        (fabsf(_yaw_in - _last_logged_yaw_in) > 0.05f) ||
        (fabsf(_throttle_in - _last_logged_throttle_in) > 0.05f);

    if (!due && !changed) {
        return;
    }

    AP::logger().WriteStreaming("HWKC",
                                "TimeUS,Roll,Pitch,Yaw,Thr,Col,Out1,Out2,Out3,PTgt,PPWM",
                                "Qfffffffffh",
                                AP_HAL::micros64(),
                                _roll_in,
                                _pitch_in,
                                _yaw_in,
                                _throttle_in,
                                compute_collective_thrust(_throttle_in),
                                _hawk_out[0],
                                _hawk_out[1],
                                _hawk_out[2],
                                degrees(_last_pivot_target_rad),
                                _last_pivot_pwm);

    _last_command_log_ms = now_ms;
    _last_logged_roll_in = _roll_in;
    _last_logged_pitch_in = _pitch_in;
    _last_logged_yaw_in = _yaw_in;
    _last_logged_throttle_in = _throttle_in;
#endif
}

void AP_MotorsHawk::output_armed_stabilizing()
{
    update_encoder_state();
    send_state_change_debug_if_needed();
    send_encoder_fault_if_needed();
    send_status_debug_if_due();
    log_status_if_due();

    if (!encoders_healthy() || !rotor_timing_ready()) {
        set_actuator_safe();
        set_pivot_servo_safe();

        limit.roll = true;
        limit.pitch = true;
        limit.yaw = true;
        limit.throttle_lower = true;
        limit.throttle_upper = false;
        return;
    }

    const float roll_in = _roll_in;
    const float pitch_in = _pitch_in;
    const float yaw_in = _yaw_in;
    const float throttle_in = _throttle_in;

    const float collective = compute_collective_thrust(throttle_in);

    bool hit_upper = false;
    bool hit_lower = false;

    for (uint8_t i = 0; i < HAWK_NUM_MOTORS; i++) {
        const float cyclic = compute_cyclic_term(i,
                                                 _theta_rad[i],
                                                 roll_in,
                                                 pitch_in,
                                                 yaw_in);

        const float out = apply_output_limits(collective + cyclic);
        _hawk_out[i] = out;

        if (out >= 0.999f) {
            hit_upper = true;
        }
        if (out <= 0.001f) {
            hit_lower = true;
        }
    }

    update_pivot_servo(_theta_rad[0], roll_in, pitch_in);
    log_command_if_needed();

    limit.roll = hit_upper || hit_lower;
    limit.pitch = hit_upper || hit_lower;
    limit.yaw = hit_upper || hit_lower;
    limit.throttle_lower = hit_lower;
    limit.throttle_upper = hit_upper;
}

void AP_MotorsHawk::output_to_motors()
{
    send_state_change_debug_if_needed();
    send_status_debug_if_due();
    log_status_if_due();

    switch (_spool_state) {
    case SpoolState::SHUT_DOWN:
        set_actuator_safe();
        set_pivot_servo_safe();
        break;

    case SpoolState::GROUND_IDLE: {
        const float idle = actuator_spin_up_to_ground_idle();
        for (uint8_t i = 0; i < HAWK_NUM_MOTORS; i++) {
            _hawk_out[i] = idle;
        }
        set_pivot_servo_safe();
        break;
    }

    case SpoolState::SPOOLING_UP:
    case SpoolState::THROTTLE_UNLIMITED:
    case SpoolState::SPOOLING_DOWN:
        output_armed_stabilizing();
        break;
    }

    for (uint8_t i = 0; i < HAWK_NUM_MOTORS; i++) {
        set_actuator_with_slew(_actuator[i], _hawk_out[i]);
        rc_write(i, output_to_pwm(_actuator[i]));
    }
}

void AP_MotorsHawk::_output_test_seq(uint8_t motor_seq, int16_t pwm)
{
    if (motor_seq < 1 || motor_seq > (HAWK_NUM_MOTORS + 1U)) {
        return;
    }

    const uint8_t motor_idx = motor_seq - 1;
    if (motor_idx < HAWK_NUM_MOTORS) {
        rc_write(motor_idx, pwm);
        return;
    }
    if (motor_seq == (HAWK_NUM_MOTORS + 1U)) {
        SRV_Channels::set_output_pwm_trimmed(HAWK_PIVOT_FUNCTION, pwm);
    }
    send_debug_text(MAV_SEVERITY_INFO, "HAWK test seq=%u pwm=%d", (unsigned)motor_seq, (int)pwm);
}
