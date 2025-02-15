#ifndef RAMPFILTER_H
#define RAMPFILTER_H

#include <Arduino.h>
#include <SimpleDebugLog.h>

/**
 * @brief Interface for filters that can be applied to motor speed.
 *
 */
class IFilter {
   public:
    /**
     * @brief Initialize the filter. Please call this method from void setup() before using the filter.
     *
     */
    virtual void begin() = 0;

    /**
     * @brief Set the target speed for the filter. The filter will start approaching the target speed with apply() method.
     *
     * @param targetSpeed The target speed to reach.
     * @return int 0 if the target speed is set successfully, -1 if not (the filter is not enabled or ready).
     *
     */
    virtual int setTargetSpeed(int targetSpeed) = 0;

    /**
     * @brief Set the "strength" of the filter. The higher the value, the slower the motor speed will change.
     *
     * @param filterFactor Filter factor to set
     *
     */
    virtual void setFilterFactor(int filterFactor) = 0;

    /**
     * @brief Get the "strength" of the filter.
     *
     * @return int Filter factor
     *
     */
    virtual int getFilterFactor() = 0;

    /**
     * @brief Disable the filter.
     *
     */
    virtual void disable() = 0;

    /**
     * @brief Apply the filter to the current speed. Call this method in a void loop() to get frequently updated speed after setting the target speed.
     *
     * @param currentSpeed The current speed of the motor.
     * @return int The new speed after applying the filter.
     *
     */
    virtual int apply(int currentSpeed) = 0;

    /**
     * @brief Destroy the IFilter object
     *
     */
    virtual ~IFilter() = default;
};

/**
 *  @brief Acceleration Fader Filter: allows to ramp the motor speed slowly up and down to prevent damage and make the movement smoother, more realistic.
 *
 */
class RampFilter : public IFilter {
   public:
    /**
     * @brief Acceleration Fader Filter: allows to ramp the motor speed slowly up and down to prevent damage and make the movement smoother, more realistic
     *
     */
    explicit RampFilter();

    /**
     * @brief Initialize the Ramp filter. Please call this method from void setup() before using the filter.
     *
     */
    void begin() override;

    /**
     * @brief Set the target speed for the ramp filter. The filter will start approaching the target speed with apply() method.
     *
     * @param targetSpeed The target speed to reach.
     * @return int 0 if the target speed is set successfully, -1 if not or the ramp filter is not enabled.
     */
    int setTargetSpeed(int targetSpeed) override;

    /**
     * @brief Apply the filter to the current speed. Call this method in a void loop() to get frequently updated speed after setting the target speed.
     *
     * @param currentSpeed The current speed of the motor.
     * @return int The new speed after applying the filter.
     *
     */
    int apply(int currentSpeed) override;

    /**
     * @brief motor acceleration/descelleration in 1 step/100 milliseconds. To disable the filter, set it to 0.
     *
     */
    unsigned int acceleration = 10;

    /**
     * @brief Set the filter factor (in this case the speed)
     *
     * @param filterFactor Acceleration to set
     *
     */
    void setFilterFactor(int filterFactor) override { acceleration = filterFactor; }

    /**
     * @brief Get the Filter Factor number
     *
     * @return int acceleration
     */
    int getFilterFactor() override { return acceleration; }

    /**
     * @brief Disable the filter
     *
     */
    void disable() override { acceleration = 0; }

   private:
    boolean isFilterEnabled() { return acceleration > 0; }
    boolean isAccelerating(int currentSpeed) { return _targetSpeed > currentSpeed; }
    int _targetSpeed = 0;
    unsigned long int _lastRampTime = 0;
};

RampFilter::RampFilter() {}

void RampFilter::begin() {
    LOG_DEBUG("RampFilter initialized");
    LOG_TRACE("  Acceleration: ", acceleration);
    LOG_TRACE("  Is ramp enabled: ", isFilterEnabled());
}

int RampFilter::setTargetSpeed(int targetSpeed) {
    if (!isFilterEnabled()) {
        return -1;
    }

    LOG_TRACE("Starting ramp to speed: ", targetSpeed);

    _targetSpeed = targetSpeed;
    _lastRampTime = millis();
    return 0;
}

int RampFilter::apply(int currentSpeed) {
    if (!isFilterEnabled()) {
        return _targetSpeed;
    }

    if (_targetSpeed == currentSpeed) {
        LOG_TRACE("  Target speed reached.");
        return currentSpeed;
    }

    unsigned long int currentTime = millis();

    int elapsedTime = currentTime - _lastRampTime;
    LOG_TRACE("  Elapsed time: ", elapsedTime);

    int currentAcceleration;
    if (isAccelerating(currentSpeed)) {
        currentAcceleration = acceleration;
    } else {
        currentAcceleration = -acceleration;
    }

    LOG_TRACE("  Calculateing new filtered speed.");

    // apply acceleration/deceleration in steps/100 milliseconds
    int acceleratedSpeed = currentSpeed + (elapsedTime * currentAcceleration / 100);
    _lastRampTime = currentTime;

    // prevent overshooting
    if (isAccelerating(currentSpeed)) {
        if (acceleratedSpeed > _targetSpeed) {
            acceleratedSpeed = _targetSpeed;
        }
    } else {
        if (acceleratedSpeed < _targetSpeed) {
            acceleratedSpeed = _targetSpeed;
        }
    }

    LOG_TRACE("  Filtered speed: ", acceleratedSpeed);
    return acceleratedSpeed;
}

#endif