#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_HAL/Semaphores.h>
#include <AP_Math/AP_Math.h>

class AP_HawkEncoder
{
public:
    static constexpr uint8_t NUM_ENCODERS = 1;

    AP_HawkEncoder() :
        _initialized(false)
    {
        for (uint8_t i = 0; i < NUM_ENCODERS; i++) {
            _state[i].raw_angle = 0;
            _state[i].angle_rad = 0.0f;
            _state[i].last_update_us = 0;
            _state[i].is_healthy = false;
        }
    }

    void init()
    {
        constexpr uint8_t I2C_BUS = 0;

        _dev = AP_HAL::get_HAL().i2c_mgr->get_device(I2C_BUS, AS5600_ADDR);
        if ((bool)_dev) {
            _dev->set_retries(3);
        }
        _initialized = (bool)_dev;

        if (!_initialized) {
            for (uint8_t i = 0; i < NUM_ENCODERS; i++) {
                _state[i].is_healthy = false;
            }
        }
    }

    void update()
    {
        if (!_initialized) {
            for (uint8_t i = 0; i < NUM_ENCODERS; i++) {
                _state[i].is_healthy = false;
            }
            return;
        }
        if (!read_one(0)) {
            mark_unhealthy(0);
        }
    }

    bool initialized() const
    {
        return _initialized;
    }

    bool healthy(uint8_t idx) const
    {
        if (idx >= NUM_ENCODERS) {
            return false;
        }
        return _state[idx].is_healthy;
    }

    bool stale(uint8_t idx, uint32_t timeout_us) const
    {
        if (idx >= NUM_ENCODERS) {
            return true;
        }
        if (!_state[idx].is_healthy) {
            return true;
        }

        const uint32_t now = AP_HAL::micros();
        return (now - _state[idx].last_update_us) > timeout_us;
    }

    float get_angle_rad(uint8_t idx) const
    {
        if (idx >= NUM_ENCODERS) {
            return 0.0f;
        }
        return _state[idx].angle_rad;
    }

    uint16_t get_raw_angle(uint8_t idx) const
    {
        if (idx >= NUM_ENCODERS) {
            return 0;
        }
        return _state[idx].raw_angle;
    }

    uint32_t get_last_update_us(uint8_t idx) const
    {
        if (idx >= NUM_ENCODERS) {
            return 0;
        }
        return _state[idx].last_update_us;
    }

private:
    static constexpr uint8_t AS5600_ADDR = 0x36;
    static constexpr uint8_t AS5600_RAW_ANGLE = 0x0C;

    struct EncoderState {
        uint16_t raw_angle;
        float angle_rad;
        uint32_t last_update_us;
        bool is_healthy;
    };

    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;

    EncoderState _state[NUM_ENCODERS];
    bool _initialized;

    bool read_one(uint8_t idx)
    {
        if (idx >= NUM_ENCODERS || !(bool)_dev) {
            return false;
        }

        uint8_t reg = AS5600_RAW_ANGLE;
        uint8_t buf[2] = {0, 0};

        // Take ownership of the encoder device bus before any transfer.
        // This prevents the "I2C: not owner ..." error from ArduPilot.
        WITH_SEMAPHORE(_dev->get_semaphore());

        if (!_dev->transfer(&reg, 1, buf, 2)) {
            return false;
        }

        const uint16_t raw = ((((uint16_t)buf[0]) << 8) | buf[1]) & 0x0FFF;
        const float angle_rad = raw * (2.0f * M_PI / 4096.0f);

        _state[idx].raw_angle = raw;
        _state[idx].angle_rad = angle_rad;
        _state[idx].last_update_us = AP_HAL::micros();
        _state[idx].is_healthy = true;

        return true;
    }

    void mark_unhealthy(uint8_t idx)
    {
        if (idx >= NUM_ENCODERS) {
            return;
        }
        _state[idx].is_healthy = false;
    }
};
