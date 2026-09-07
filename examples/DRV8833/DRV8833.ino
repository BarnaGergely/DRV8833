#include <DRV8833MotorDriver.h>

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
