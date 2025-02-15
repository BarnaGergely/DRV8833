#ifndef DRV8833_MOTOR_DRIVER_H
#define DRV8833_MOTOR_DRIVER_H

#include <Arduino.h>
#include <DRV8833.h>
#include <SimpleDebugLog.h>

#include "RampFilter.h"

class DRV8833MotorDriver {
   public:
    IFilter& filter;
    unsigned int neutralWidth = 25;
    int maxSpeed = 127;
    int minSpeed = -127;

    DRV8833MotorDriver(DRV8833& motor);
    void begin();
    void run();
    int setSpeed(int speed);
    int stop();

   private:
    DRV8833& _motor;
    boolean _isReady = false;
    int _currentSpeed = 0;
    int _targetSpeed = 0;
    unsigned int _neutralPwmWidth = map(neutralWidth, 0, maxSpeed, 0, 255);

    int setSpeedUnsafe(int speed);
};

DRV8833MotorDriver::DRV8833MotorDriver(DRV8833& motor) : _motor(motor), filter(*new RampFilter()) {}

void DRV8833MotorDriver::begin() {
    _motor.begin();
    LOG_DEBUG("[DRV8833MotorDriver] Motor driver ready");
    _isReady = true;
}

void DRV8833MotorDriver::run() {
    // TODO: 0 is not working, Fix it!
    // if the current speed of the motor is not the target speed
    if (_currentSpeed != _targetSpeed) {
        LOG_DEBUG("[DRV8833MotorDriver] Motor speed change detected.");
        LOG_DEBUG("    [DRV8833MotorDriver] Current speed: ", _currentSpeed);
        LOG_DEBUG("    [DRV8833MotorDriver] Target speed: ", _targetSpeed);

        // calculate new speed
        int filteredSpeed = filter.apply(_currentSpeed);

        LOG_DEBUG("    [DRV8833MotorDriver] Filtered speed: ", filteredSpeed);

        // apply neutral width filtering
        int neutralFilteredSpeed;
        if (filteredSpeed > 0) {
            neutralFilteredSpeed = map(filteredSpeed, 0, maxSpeed, _neutralPwmWidth, 255);
        } else if (filteredSpeed < 0) {
            neutralFilteredSpeed = map(filteredSpeed, minSpeed, 0, -255, -_neutralPwmWidth);
        } else {
            neutralFilteredSpeed = 0;
        }

        // apply new speed
        if (!_motor.setMotorPwm(neutralFilteredSpeed)) {
            LOG_DEBUG("    [DRV8833MotorDriver] Motor PWM set to: ", neutralFilteredSpeed);
            _currentSpeed = filteredSpeed;
        } else {
            LOG_ERROR("    [DRV8833MotorDriver] Failed to set motor PWM: ", neutralFilteredSpeed);
        }
    }
}

int DRV8833MotorDriver::setSpeed(int speed) {
    if (!_isReady) {
        LOG_ERROR("[DRV8833MotorDriver] Not ready. Please call begin() in the setup() function before using the motor driver.");
        return -1;
    }
    if (speed < minSpeed || speed > maxSpeed) {
        LOG_ERROR("[DRV8833MotorDriver] Speed out of range: ", speed);
        return -1;
    }
    return setSpeedUnsafe(speed);
}

int DRV8833MotorDriver::stop() {
    LOG_DEBUG("[DRV8833MotorDriver] Stopping motor");
    return setSpeedUnsafe(0);
}

int DRV8833MotorDriver::setSpeedUnsafe(int speed) {
    LOG_DEBUG("[DRV8833MotorDriver] Setting speed to: ", speed);
    _targetSpeed = speed;
    filter.setTargetSpeed(_targetSpeed);
    run();  // call run() to apply change in case the loop() is blocked
    return 0;
}

#endif