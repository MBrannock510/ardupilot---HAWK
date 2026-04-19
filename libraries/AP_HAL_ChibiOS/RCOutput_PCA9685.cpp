#include "RCOutput_PCA9685.h"

#if HAL_USE_I2C == TRUE && defined(HAL_PCA9685_RCOUT_ENABLED) && HAL_PCA9685_RCOUT_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Device.h>
#include <AP_Math/AP_Math.h>
#include <math.h>

using namespace ChibiOS;

extern const AP_HAL::HAL& hal;

void RCOutput_PCA9685::init(uint8_t bus, uint8_t address)
{
    _dev = hal.i2c_mgr->get_device(bus, address);
    if (!_dev) {
        return;
    }
    _dev->set_retries(3);
    _hw_initialised = false;
}

bool RCOutput_PCA9685::ensure_initialised()
{
    if (!_dev) {
        return false;
    }
    if (_hw_initialised) {
        return true;
    }

    reset_all_channels();
    set_freq(_frequency);
    force_safety_off();
    _hw_initialised = true;
    return true;
}

void RCOutput_PCA9685::reset_all_channels()
{
    if (!_dev) {
        return;
    }
    WITH_SEMAPHORE(_dev->get_semaphore());
    const uint8_t data[] = {PCA9685_RA_LED0_ON_L, 0, 0, 0, 0};
    _dev->transfer(data, sizeof(data), nullptr, 0);
    hal.scheduler->delay_microseconds(2000);
}

void RCOutput_PCA9685::set_freq(uint16_t freq_hz)
{
    if (!_dev) {
        return;
    }

    freq_hz = constrain_int16(freq_hz, 24, 400);
    _frequency = freq_hz;

    if (!_hw_initialised) {
        return;
    }

    WITH_SEMAPHORE(_dev->get_semaphore());

    _dev->write_register(PCA9685_RA_ALL_LED_OFF_H, PCA9685_ALL_LED_OFF_H_SHUT);
    _dev->write_register(PCA9685_RA_MODE1, PCA9685_MODE1_SLEEP_BIT);

    const uint8_t prescale = uint8_t(MAX(3.0f, ceilf(PCA9685_INTERNAL_CLOCK / (4096.0f * freq_hz)) - 1.0f));
    _frequency = uint16_t(PCA9685_INTERNAL_CLOCK / (4096.0f * (prescale + 1U)));

    _dev->write_register(PCA9685_RA_PRE_SCALE, prescale);
    _dev->write_register(PCA9685_RA_MODE1, PCA9685_MODE1_RESTART_BIT | PCA9685_MODE1_AI_BIT);
}

void RCOutput_PCA9685::write(uint8_t ch, uint16_t period_us)
{
    if (ch >= CHANNEL_COUNT) {
        return;
    }
    _pulse_buffer[ch] = period_us;
    _pending_write_mask |= (1U << ch);
}

void RCOutput_PCA9685::push()
{
    if (!_dev || _pending_write_mask == 0 || !ensure_initialised()) {
        return;
    }

    const uint8_t max_ch = (sizeof(unsigned) * 8) - __builtin_clz(_pending_write_mask);
    const uint8_t min_ch = __builtin_ctz(_pending_write_mask);

    struct PACKED pwm_values {
        uint8_t reg;
        uint8_t data[CHANNEL_COUNT * 4];
    } pwm_values {};

    for (uint8_t ch = min_ch; ch < max_ch; ch++) {
        const uint16_t period_us = _pulse_buffer[ch];
        uint16_t length = 0;
        if (period_us != 0) {
            length = roundf((period_us * 4096.0f) / (1000000.0f / _frequency));
            length = constrain_int16(length, 1, 4095);
        }

        uint8_t *d = &pwm_values.data[(ch - min_ch) * 4];
        *d++ = 0;
        *d++ = 0;
        *d++ = length & 0xFF;
        *d++ = length >> 8;
    }

    WITH_SEMAPHORE(_dev->get_semaphore());
    pwm_values.reg = PCA9685_RA_LED0_ON_L + 4U * min_ch;
    const size_t payload_size = 1U + (max_ch - min_ch) * 4U;
    _dev->transfer((const uint8_t *)&pwm_values, payload_size, nullptr, 0);
    _pending_write_mask = 0;
}

bool RCOutput_PCA9685::force_safety_on()
{
    if (!_dev || !_hw_initialised) {
        return false;
    }
    WITH_SEMAPHORE(_dev->get_semaphore());
    _dev->write_register(PCA9685_RA_ALL_LED_OFF_H, PCA9685_ALL_LED_OFF_H_SHUT);
    return true;
}

void RCOutput_PCA9685::force_safety_off()
{
    if (!_dev) {
        return;
    }
    WITH_SEMAPHORE(_dev->get_semaphore());
    _dev->write_register(PCA9685_RA_MODE1, PCA9685_MODE1_RESTART_BIT | PCA9685_MODE1_AI_BIT);
}

#endif
