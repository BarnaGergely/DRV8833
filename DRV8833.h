#ifndef DRV8833_h
#define DRV8833_h

#include <Arduino.h>
#include <SimpleDebugLog.h>

class DRV8833 {
   public:
    DRV8833(int pin1, int pin2, boolean isInverted);
    /// @brief Initialize the motor driver. Must be called before using the motor in the setup() function.
    void begin();
    /// @brief Set the current on a motor channel using PWM and directional logic.
    /// @param pwm PWM duty cycle ranging from -255 full reverse to 255 full forward
    int setMotorPwm(int pwm);

   private:
    const int _pin1;
    const int _pin2;
    const boolean _isInverted;
    boolean _isReady = false;
};

DRV8833::DRV8833(int pin1, int pin2, boolean isInverted) : _pin1(pin1), _pin2(pin2), _isInverted(isInverted) {}

void DRV8833::begin() {
    pinMode(_pin1, OUTPUT);
    pinMode(_pin2, OUTPUT);
    digitalWrite(_pin1, LOW);
    digitalWrite(_pin2, LOW);
    _isReady = true;
    LOG_DEBUG("[DRV8833] Motor driver ready");
}

int DRV8833::setMotorPwm(int pwm) {
    if (pwm < -255 || pwm > 255) {
        LOG_ERROR("[DRV8833] PWM out of range: ", pwm);
        return -1;
    }
    if (!_isReady) {
        LOG_ERROR("[DRV8833] Motor not ready. Please call begin() before using this motor driver.");
        return -1;
    }
    if (_isInverted) { // invert driving direction
        pwm = -pwm;
    }
    if (pwm < 0) {  // reverse speeds
        analogWrite(_pin1, -pwm);
        digitalWrite(_pin2, LOW);

    } else {  // stop or forward
        digitalWrite(_pin1, LOW);
        analogWrite(_pin2, pwm);
    }
    return 0;
}

#endif