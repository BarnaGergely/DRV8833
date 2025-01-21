/*
  DRV8833.h - Library for the Texas Instruments DRV8833 motor driver.
  Created by TheDIYGuy999 June 2016
  Released into the public domain.
*/

#ifndef DRV8833_h
#define DRV8833_h

#include "Arduino.h"

/// @brief Texas Instruments DRV8833 advanced motor driver
class DRV8833 {
  public:
    /// @brief DRV8833 motor driver constructor
    /// @param pin1 The first pin connected to the motor driver. Must always be PWM capable.
    /// @param pin2 The second pin connected to the motor driver. If the doublePWM is set to true, must also be PWM capable
    /// @param minInput Lower limit of the controlValue (minimum of controlValue)
    /// @param maxInput Upper limit of the controlValue (maximum of controlValue)
    /// @param neutralWidth The width of the neutral zone where the motor does not move
    /// @param invert Invert rotation direction
    /// @param doublePWM Use pin2 as PWM capable
    DRV8833(int pin1, int pin2, int minInput, int maxInput, int neutralWidth, boolean invert, boolean doublePWM);
    
    /// @brief Drive the motor with specified speed and settings
    /// @param controlValue Motor speed and direction
    /// @param maxPWM Maximum PWM value limit
    /// @param rampTime Ramp the motor speed slowly up & down to protect the gearbox. In ms per 1 PWM increment.
    /// @param brake Activate break
    /// @param neutralBrake Activate brake in neutral position, meaning stop the motor when the controlValue is within the neutralWidth.
    void drive(int controlValue, int maxPWM, int rampTime, boolean brake, boolean neutralBrake);

  private:
    int _pin1;
    int _pin2;
    int _minInput;
    int _maxInput;
    int _minNeutral;
    int _maxNeutral;
    int _controlValue;
    int _controlValueRamp;
    int _maxPWM;
    int _rampTime;
    boolean _brake;
    boolean _neutralBrake;
    boolean _invert;
    boolean _doublePWM;
    unsigned long _previousMillis = 0;
    byte _state = 0;
};

#endif
