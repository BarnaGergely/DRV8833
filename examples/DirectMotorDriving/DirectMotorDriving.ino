#include <DRV8833.h>

// Example code for using the DRV8833 motor driver with two motors.
// This code demonstrates how to control the speed and direction of two motors using PWM signals.
// Only for advanced users, because this code does not use any safety features like speed ramping or speed limiting. Use at your own risk!

// Pins should support PWM for variable speed in both directions.
const int motor1In1 = 5;
const int motor1In2 = 6;
const int motor2In1 = 9;
const int motor2In2 = 10;

DRV8833 motor1(motor1In1, motor1In2);
DRV8833 motor2(motor2In1, motor2In2);

void setup() {
    motor1.begin();
    motor2.begin();
}

void loop() {
    motor1.setMotorPwm(512);   // Set motor speed to 512 (forward)
    delay(2000);               // Run for 2 seconds
    motor1.setMotorPwm(0);     // Stop the motor
    delay(2000);               // Wait for 2 seconds
    motor1.setMotorPwm(-512);  // Set motor speed to -512 (reverse)
    delay(2000);               // Run for 2 seconds

    motor2.setMotorPwm(512);   // Set motor speed to 512 (forward)
    delay(2000);               // Run for 2 seconds
    motor2.setMotorPwm(0);     // Stop the motor
    delay(2000);               // Wait for 2 seconds
    motor2.setMotorPwm(-512);  // Set motor speed to -512 (reverse)
    delay(2000);               // Run for 2 seconds
}
