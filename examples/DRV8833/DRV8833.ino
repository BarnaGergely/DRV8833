#include <MotorController.h>

// Example usage of the MotorController class with a DRV8833 motor driver.
// Recommended for beginners, as it provides a simple interface for controlling 
// and safety features like speed ramping and speed limiting.

// Both pins should support PWM for variable speed in both directions.
const int motorIn1 = 5;
const int motorIn2 = 6;

DRV8833 motor(motorIn1, motorIn2);
MotorController driver(motor);

void setup() {
  Serial.begin(115200);
  driver.begin();
}

void loop() {
  const int input = analogRead(A0);
  const int speed = map(input, 0, 1023, driver.minSpeed, driver.maxSpeed);

  driver.setSpeed(speed);
  driver.run();
}
