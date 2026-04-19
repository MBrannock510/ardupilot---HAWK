#pragma once

#include "AP_HAL_ChibiOS.h"

#if HAL_USE_I2C == TRUE && defined(HAL_PCA9685_RCOUT_ENABLED) && HAL_PCA9685_RCOUT_ENABLED

#include <AP_HAL/I2CDevice.h>

namespace ChibiOS {

class RCOutput_PCA9685 {
public:
    static constexpr uint8_t CHANNEL_COUNT = 16;

    RCOutput_PCA9685() = default;

    void init(uint8_t bus, uint8_t address);
    bool available() const { return bool(_dev); }

    void set_freq(uint16_t freq_hz);
    uint16_t get_freq() const { return _frequency; }

    void write(uint8_t ch, uint16_t period_us);
    void disable_ch(uint8_t ch) { write(ch, 0); }
    void push();

    bool force_safety_on();
    void force_safety_off();

private:
    static constexpr uint8_t PCA9685_RA_MODE1 = 0x00;
    static constexpr uint8_t PCA9685_RA_LED0_ON_L = 0x06;
    static constexpr uint8_t PCA9685_RA_ALL_LED_OFF_H = 0xFD;
    static constexpr uint8_t PCA9685_RA_PRE_SCALE = 0xFE;

    static constexpr uint8_t PCA9685_MODE1_RESTART_BIT = (1U << 7);
    static constexpr uint8_t PCA9685_MODE1_AI_BIT = (1U << 5);
    static constexpr uint8_t PCA9685_MODE1_SLEEP_BIT = (1U << 4);
    static constexpr uint8_t PCA9685_ALL_LED_OFF_H_SHUT = (1U << 4);

    static constexpr float PCA9685_INTERNAL_CLOCK = (1.04f * 25000000.0f);

    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;
    uint16_t _frequency = 50;
    uint16_t _pulse_buffer[CHANNEL_COUNT] {};
    uint16_t _pending_write_mask = 0;
    bool _hw_initialised = false;

    bool ensure_initialised();
    void reset_all_channels();
};

}

#endif
