#include "AP_RotorPhase.h"

#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

AP_RotorPhase *AP_RotorPhase::_singleton;

const AP_Param::GroupInfo AP_RotorPhase::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: Rotor phase estimator enable
    // @Description: Enable hall-sensor-based rotor azimuth estimation for tip-jet modulation.
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("ENABLE", 1, AP_RotorPhase, _enable, 0),

    // @Param: PIN1
    // @DisplayName: Rotor phase hall sensor pin 1
    // @Description: GPIO pin for hall sensor sector 1.
    // @User: Advanced
    AP_GROUPINFO("PIN1", 2, AP_RotorPhase, _pin1, -1),

    // @Param: PIN2
    // @DisplayName: Rotor phase hall sensor pin 2
    // @Description: GPIO pin for hall sensor sector 2.
    // @User: Advanced
    AP_GROUPINFO("PIN2", 3, AP_RotorPhase, _pin2, -1),

    // @Param: PIN3
    // @DisplayName: Rotor phase hall sensor pin 3
    // @Description: GPIO pin for hall sensor sector 3.
    // @User: Advanced
    AP_GROUPINFO("PIN3", 4, AP_RotorPhase, _pin3, -1),

    // @Param: INVERT
    // @DisplayName: Rotor phase direction invert
    // @Description: Reverse hall sensor rotational direction interpretation.
    // @Values: 0:Normal,1:Inverted
    // @User: Advanced
    AP_GROUPINFO("INVERT", 5, AP_RotorPhase, _invert_dir, 0),

    // @Param: TIMEOUT
    // @DisplayName: Rotor phase timeout
    // @Description: Maximum time since last hall event before phase becomes invalid.
    // @Range: 5 1000
    // @Units: ms
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("TIMEOUT", 6, AP_RotorPhase, _timeout_ms, 120.0f),

    AP_GROUPEND
};

AP_RotorPhase::AP_RotorPhase()
{
    AP_Param::setup_object_defaults(this, var_info);

    if (_singleton != nullptr) {
        AP_HAL::panic("AP_RotorPhase must be singleton");
    }
    _singleton = this;

    _irq_state.hall_state = 0;
    _irq_state.sector = HallSector::INVALID;
    _irq_state.last_transition_us = 0;
    _irq_state.prev_transition_us = 0;
    _irq_state.last_event_us = 0;
    _irq_state.sector_period_us = 0;
    _irq_state.valid = false;
}

AP_RotorPhase *AP_RotorPhase::get_singleton()
{
    return _singleton;
}

bool AP_RotorPhase::setup_pins()
{
    if ((_pin1 < 0) || (_pin2 < 0) || (_pin3 < 0)) {
        return false;
    }

    const uint8_t p1 = uint8_t(_pin1.get());
    const uint8_t p2 = uint8_t(_pin2.get());
    const uint8_t p3 = uint8_t(_pin3.get());

    if (!hal.gpio->valid_pin(p1) || !hal.gpio->valid_pin(p2) || !hal.gpio->valid_pin(p3)) {
        return false;
    }

    if (_pins_active && p1 == _last_pin1 && p2 == _last_pin2 && p3 == _last_pin3) {
        return true;
    }

    detach_pins();

    hal.gpio->pinMode(p1, HAL_GPIO_INPUT);
    hal.gpio->pinMode(p2, HAL_GPIO_INPUT);
    hal.gpio->pinMode(p3, HAL_GPIO_INPUT);

    if (!hal.gpio->attach_interrupt(p1, FUNCTOR_BIND_MEMBER(&AP_RotorPhase::irq_handler, void, uint8_t, bool, uint32_t), AP_HAL::GPIO::INTERRUPT_BOTH) ||
        !hal.gpio->attach_interrupt(p2, FUNCTOR_BIND_MEMBER(&AP_RotorPhase::irq_handler, void, uint8_t, bool, uint32_t), AP_HAL::GPIO::INTERRUPT_BOTH) ||
        !hal.gpio->attach_interrupt(p3, FUNCTOR_BIND_MEMBER(&AP_RotorPhase::irq_handler, void, uint8_t, bool, uint32_t), AP_HAL::GPIO::INTERRUPT_BOTH)) {
        detach_pins();
        return false;
    }

    _pins_active = true;
    _last_pin1 = p1;
    _last_pin2 = p2;
    _last_pin3 = p3;
    refresh_pin_state();
    return true;
}

void AP_RotorPhase::detach_pins()
{
    if (!_pins_active) {
        return;
    }
    hal.gpio->detach_interrupt(_last_pin1);
    hal.gpio->detach_interrupt(_last_pin2);
    hal.gpio->detach_interrupt(_last_pin3);
    _pins_active = false;
}

void AP_RotorPhase::refresh_pin_state()
{
    const uint8_t s1 = (uint8_t)hal.gpio->read(uint8_t(_pin1.get()));
    const uint8_t s2 = (uint8_t)hal.gpio->read(uint8_t(_pin2.get()));
    const uint8_t s3 = (uint8_t)hal.gpio->read(uint8_t(_pin3.get()));
    _irq_state.hall_state = (s1 << 0) | (s2 << 1) | (s3 << 2);
    _irq_state.sector = hall_to_sector(_irq_state.hall_state);
    _irq_state.valid = _irq_state.sector != HallSector::INVALID;
}

AP_RotorPhase::HallSector AP_RotorPhase::hall_to_sector(uint8_t hall_state) const
{
    switch (hall_state) {
    case 0x01:
        return HallSector::S0;
    case 0x02:
        return HallSector::S1;
    case 0x04:
        return HallSector::S2;
    default:
        return HallSector::INVALID;
    }
}

float AP_RotorPhase::sector_to_theta(HallSector sector, float frac) const
{
    if (sector == HallSector::INVALID) {
        return 0.0f;
    }
    const float sector_size = (2.0f * M_PI) / 3.0f;
    const float base = int8_t(sector) * sector_size;
    float theta = base + sector_size * constrain_float(frac, 0.0f, 1.0f);
    if (_invert_dir > 0) {
        theta = wrap_2PI(2.0f * M_PI - theta);
    }
    return wrap_2PI(theta);
}

void AP_RotorPhase::irq_update(uint32_t timestamp_us)
{
    const uint8_t s1 = (uint8_t)hal.gpio->read(uint8_t(_pin1.get()));
    const uint8_t s2 = (uint8_t)hal.gpio->read(uint8_t(_pin2.get()));
    const uint8_t s3 = (uint8_t)hal.gpio->read(uint8_t(_pin3.get()));
    const uint8_t hall_state = (s1 << 0) | (s2 << 1) | (s3 << 2);
    const HallSector new_sector = hall_to_sector(hall_state);

    _irq_state.hall_state = hall_state;
    _irq_state.last_event_us = timestamp_us;

    if (new_sector == HallSector::INVALID) {
        _irq_state.valid = false;
        _irq_state.sector = HallSector::INVALID;
        return;
    }

    if (new_sector != _irq_state.sector) {
        _irq_state.prev_transition_us = _irq_state.last_transition_us;
        _irq_state.last_transition_us = timestamp_us;
        if ((_irq_state.prev_transition_us > 0) && (_irq_state.last_transition_us > _irq_state.prev_transition_us)) {
            _irq_state.sector_period_us = _irq_state.last_transition_us - _irq_state.prev_transition_us;
        }
        _irq_state.sector = new_sector;
    }

    _irq_state.valid = true;
}

void AP_RotorPhase::irq_handler(uint8_t pin, bool pin_value, uint32_t timestamp_us)
{
    (void)pin;
    (void)pin_value;
    irq_update(timestamp_us);
}

void AP_RotorPhase::update()
{
    _healthy = false;
    _theta_rad = 0.0f;
    _rate_rps = 0.0f;

    if (!enabled()) {
        detach_pins();
        return;
    }

    if (!setup_pins()) {
        return;
    }

    IRQState state;
    {
        void *irqstate = hal.scheduler->disable_interrupts_save();
        state = _irq_state;
        hal.scheduler->restore_interrupts(irqstate);
    }

    if (!state.valid || state.sector == HallSector::INVALID) {
        return;
    }

    const uint32_t now_us = AP_HAL::micros();
    const uint32_t timeout_us = (uint32_t)constrain_float(_timeout_ms, 5.0f, 1000.0f) * 1000U;
    if ((state.last_event_us == 0) || (now_us - state.last_event_us > timeout_us)) {
        return;
    }

    float frac = 0.0f;
    if (state.sector_period_us > 0) {
        const float dt_us = float(now_us - state.last_transition_us);
        frac = dt_us / float(state.sector_period_us);
    }

    _theta_rad = sector_to_theta(state.sector, frac);
    if (state.sector_period_us > 0) {
        _rate_rps = 1.0e6f / (float(state.sector_period_us) * 3.0f);
    }
    _healthy = true;
}

bool AP_RotorPhase::get_estimate(float &theta_rad, float &rate_rps) const
{
    if (!_healthy) {
        return false;
    }
    theta_rad = _theta_rad;
    rate_rps = _rate_rps;
    return true;
}

namespace AP {
AP_RotorPhase *rotor_phase()
{
    static AP_RotorPhase rotor_phase_singleton;
    return &rotor_phase_singleton;
}
}
