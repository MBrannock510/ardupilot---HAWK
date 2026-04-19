#pragma once

#include "AP_MotorsMulticopter.h"

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_HawkEncoder/AP_HawkEncoder.h>
#include <SRV_Channel/SRV_Channel.h>

class AP_MotorsHawk : public AP_MotorsMulticopter
{
public:
    friend class AP_Motors;

    AP_MotorsHawk(uint16_t speed_hz = AP_MOTORS_SPEED_DEFAULT);

    void init(motor_frame_class frame_class,
              motor_frame_type frame_type) override;

    void set_frame_class_and_type(motor_frame_class frame_class,
                                  motor_frame_type frame_type) override;

    bool arming_checks(size_t buflen, char *buffer) const override;
    bool motor_test_checks(size_t buflen, char *buffer) const override;

    uint32_t get_motor_mask() override;

    static const struct AP_Param::GroupInfo var_info[];

protected:
    void output_to_motors() override;
    void output_armed_stabilizing() override;
    const char* _get_frame_string() const override;
    void _output_test_seq(uint8_t motor_seq, int16_t pwm) override;

    void update_encoder_state();
    bool encoders_healthy() const;
    void set_actuator_safe();
    void set_pivot_servo_safe();
    float compute_collective_thrust(float throttle_in) const;
    float compute_cyclic_term(uint8_t motor_idx,
                              float theta_rad,
                              float roll_in,
                              float pitch_in,
                              float yaw_in) const;
    float apply_output_limits(float in) const;
    void configure_hardcoded_defaults();
    void update_pivot_servo(float theta_rad,
                            float roll_in,
                            float pitch_in);
    void send_state_change_debug_if_needed();
    void send_status_debug_if_due();
    void log_status_if_due();
    void log_command_if_needed();

    // debug helpers
    void send_debug_text(MAV_SEVERITY severity, const char *fmt, ...) const;
    void send_encoder_fault_if_needed();
    bool rotor_timing_ready() const;
    bool rotor_timing_ready(uint32_t now_us) const;
    void update_rotation_period(float theta_rad, uint32_t sample_time_us);
    float compute_predicted_theta(uint32_t now_us) const;

private:
    static constexpr const char *HAWK_FW_VERSION = "HAWK v0.1.0";
    static constexpr uint8_t HAWK_NUM_MOTORS = 3;
    static constexpr uint32_t ENCODER_TIMEOUT_US = 20000U; // 20 ms
    static constexpr uint8_t ROTATION_HISTORY_LEN = 10;
    static constexpr uint8_t HAWK_PIVOT_CH = AP_MOTORS_MOT_4;
    static constexpr SRV_Channel::Function HAWK_PIVOT_FUNCTION = SRV_Channel::k_motor_tilt;
    static constexpr uint16_t HAWK_PIVOT_PWM_MIN = 1000;
    static constexpr uint16_t HAWK_PIVOT_PWM_MAX = 2000;
    static constexpr uint16_t HAWK_PIVOT_PWM_TRIM = 1500;
    static constexpr int16_t HAWK_PIVOT_PWM_SPAN = 300;
    static constexpr float HAWK_PIVOT_PHASE_GAIN = 1.5f;

    enum MotorIndex : uint8_t {
        MOTOR_HAWK_1 = 0,
        MOTOR_HAWK_2 = 1,
        MOTOR_HAWK_3 = 2
    };

    AP_HawkEncoder _encoders;

    float _theta_rad[HAWK_NUM_MOTORS];
    bool _encoder_healthy;
    float _hawk_out[HAWK_NUM_MOTORS];

    AP_Float _cyclic_roll_gain;
    AP_Float _cyclic_pitch_gain;
    AP_Float _yaw_gain;
    AP_Float _collective_gain;
    AP_Float _cyclic_max;

    AP_Float _phase_roll_deg[HAWK_NUM_MOTORS];
    AP_Float _phase_pitch_deg[HAWK_NUM_MOTORS];
    AP_Float _yaw_bias[HAWK_NUM_MOTORS];

    bool _encoders_initialized;
    bool _frame_configured;
    float _last_measured_theta_rad;
    float _avg_rotation_period_us;
    float _rotation_period_history_us[ROTATION_HISTORY_LEN];
    uint32_t _last_rotation_tick_us;
    uint8_t _rotation_history_count;
    uint8_t _rotation_history_index;
    bool _rotation_tick_valid;
    bool _theta_sample_valid;

    // debug state
    uint32_t _last_debug_ms;
    uint32_t _last_fault_ms;
    uint32_t _last_status_ms;
    uint32_t _last_command_log_ms;
    bool _sent_init_msg;
    bool _had_encoder_fault;
    bool _last_armed_state;
    SpoolState _last_spool_state;
    float _last_pivot_target_rad;
    int16_t _last_pivot_pwm;
    float _last_logged_roll_in;
    float _last_logged_pitch_in;
    float _last_logged_yaw_in;
    float _last_logged_throttle_in;
};
