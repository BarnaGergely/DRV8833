#ifndef DRV8833_MOTOR_DRIVER_H
#define DRV8833_MOTOR_DRIVER_H

#include <Arduino.h>
#include <DRV8833.h>
#include <SimpleDebugLog.h>

#include "RampFilter.h"

/**
 * @brief Provides bounded, ramped speed control for a DRV8833 motor.
 */
class MotorController {
   public:
    /** The ramp filter used to smooth speed changes. */
    RampFilter filter;
    /** Width of the neutral compensation range in speed units. */
    unsigned int neutralWidth = 25;
    /** Maximum accepted forward speed. */
    int maxSpeed = 127;
    /** Minimum accepted reverse speed. */
    int minSpeed = -127;

    /**
     * @brief Constructs a motor driver around an initialized motor channel.
     * @param motor The low-level DRV8833 motor to control.
     */
    MotorController(DRV8833& motor);

    /** @brief Initializes the motor channel and ramp filter. */
    void begin();

    /** @brief Advances the motor toward its target speed. */
    void run();

    /**
     * @brief Sets the target speed.
     * @param speed Speed between minSpeed and maxSpeed.
     * @return 0 on success, or -1 if the driver is not ready or the speed is out of range.
     */
    int setSpeed(int speed);

    /**
     * @brief Requests a ramped stop.
     * @return 0 on success, or -1 if the driver is not ready.
     */
    int stop();

   private:
    DRV8833& _motor;
    boolean _isReady = false;
    int _currentSpeed = 0;
    int _targetSpeed = 0;
    unsigned int _neutralPwmWidth = map(neutralWidth, 0, maxSpeed, 0, 255);

    int setSpeedUnsafe(int speed);
};

inline MotorController::MotorController(DRV8833& motor) : _motor(motor) {}

inline void MotorController::begin() {
    _motor.begin();
    filter.begin();
    _neutralPwmWidth = map(neutralWidth, 0, maxSpeed, 0, 255);
    LOG_DEBUG("[MotorController] Motor driver ready");
    _isReady = true;
}

inline void MotorController::run() {
    // if the current speed of the motor is not the target speed
    if (_currentSpeed != _targetSpeed) {
        LOG_DEBUG("[MotorController] Motor speed change detected.");
        LOG_DEBUG("    [MotorController] Current speed: ", _currentSpeed);
        LOG_DEBUG("    [MotorController] Target speed: ", _targetSpeed);

        // calculate new speed
        int filteredSpeed = filter.apply(_currentSpeed);

        LOG_DEBUG("    [MotorController] Filtered speed: ", filteredSpeed);

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
            LOG_DEBUG("    [MotorController] Motor PWM set to: ", neutralFilteredSpeed);
            _currentSpeed = filteredSpeed;
        } else {
            LOG_ERROR("    [MotorController] Failed to set motor PWM: ", neutralFilteredSpeed);
        }
    }
}

inline int MotorController::setSpeed(int speed) {
    if (!_isReady) {
        LOG_ERROR("[MotorController] Not ready. Please call begin() in the setup() function before using the motor driver.");
        return -1;
    }
    if (speed < minSpeed || speed > maxSpeed) {
        LOG_ERROR("[MotorController] Speed out of range: ", speed);
        return -1;
    }
    return setSpeedUnsafe(speed);
}

inline int MotorController::stop() {
    if (!_isReady) {
        LOG_ERROR("[MotorController] Not ready. Please call begin() in the setup() function before stopping the motor.");
        return -1;
    }
    LOG_DEBUG("[MotorController] Stopping motor");
    return setSpeedUnsafe(0);
}

inline int MotorController::setSpeedUnsafe(int speed) {
    LOG_DEBUG("[MotorController] Setting speed to: ", speed);
    _targetSpeed = speed;
    filter.setTargetSpeed(_targetSpeed);
    run();  // call run() to apply change in case the loop() is blocked
    return 0;
}

#endif