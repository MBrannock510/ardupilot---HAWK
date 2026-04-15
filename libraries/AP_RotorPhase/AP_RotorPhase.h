#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <AP_Param/AP_Param.h>

class AP_RotorPhase
{
public:
    AP_RotorPhase();

    CLASS_NO_COPY(AP_RotorPhase);

    static AP_RotorPhase *get_singleton();

    // update the estimator state; call from the flight loop
    void update();

    // return true when azimuth and rate estimates are valid
    bool get_estimate(float &theta_rad, float &rate_rps) const;
    bool healthy() const { return _healthy; }
    bool enabled() const { return _enable > 0; }

    static const struct AP_Param::GroupInfo var_info[];

private:
    enum class HallSector : int8_t {
        INVALID = -1,
        S0 = 0,
        S1 = 1,
        S2 = 2
    };

    struct IRQState {
        uint8_t hall_state;
        HallSector sector;
        uint32_t last_transition_us;
        uint32_t prev_transition_us;
        uint32_t last_event_us;
        uint32_t sector_period_us;
        bool valid;
    };

    void refresh_pin_state();
    bool setup_pins();
    void detach_pins();
    HallSector hall_to_sector(uint8_t hall_state) const;
    float sector_to_theta(HallSector sector, float frac) const;
    void irq_update(uint32_t timestamp_us);
    void irq_handler(uint8_t pin, bool pin_value, uint32_t timestamp_us);

    static AP_RotorPhase *_singleton;

    AP_Int8 _enable;
    AP_Int16 _pin1;
    AP_Int16 _pin2;
    AP_Int16 _pin3;
    AP_Int8 _invert_dir;
    AP_Float _timeout_ms;

    mutable float _theta_rad;
    mutable float _rate_rps;
    bool _healthy;
    bool _pins_active;
    uint8_t _last_pin1;
    uint8_t _last_pin2;
    uint8_t _last_pin3;
    IRQState _irq_state;
};

namespace AP {
AP_RotorPhase *rotor_phase();
}
