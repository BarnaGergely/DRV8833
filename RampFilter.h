#ifndef RAMPFILTER_H
#define RAMPFILTER_H

#include <Arduino.h>
#include <SimpleDebugLog.h>

class IFilter {
   public:
    virtual ~IFilter() = default;
    virtual void begin(int targetValue, int time) = 0;
    virtual int apply(int controlValue) = 0;
};

// Fader (allows to ramp the motor speed slowly up & down)
class RampFilter : public IFilter {
   public:
    explicit RampFilter();
    void begin(int targetValue, int time) override;
    int apply(int controlValue) override;
    unsigned int rampTime = 500;
    int maxSpeed = 127;
    int minSpeed = -127;

   private:
    boolean isRampEnabled() { return rampTime >= 1; }
    unsigned long int _rampStartTime = 0;
};

RampFilter::RampFilter() {}

void RampFilter::begin(int targetValue, int time) {
    if (!isRampEnabled()) {
        return;
    }

    _rampStartTime = millis();
}

int RampFilter::apply(int controlValue) {
    static int _speedRamp = 0;
    static unsigned long int _previousMillis = 0;
    if (isRampEnabled()) {
        unsigned long int currentMillis = millis();
        if (currentMillis - _previousMillis >= rampTime) {
            // Increase
            if (controlValue > _speedRamp && _speedRamp < maxSpeed) {
                _speedRamp++;
            }
            // Decrease
            if (controlValue < _speedRamp && _speedRamp > minSpeed) {
                _speedRamp--;
            }
            _previousMillis = currentMillis;
        }
    }
    return _speedRamp;
}

#endif