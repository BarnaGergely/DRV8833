# DRV8833 Arduino library

A non-blocking Arduino library compatible with ESP32 and ESP8266 for controlling DC motors with the Texas Instruments DRV8833 dual H-bridge motor driver.

## Features

- Signed speed control with configurable minimum and maximum speed values.
- Software direction inversion.
- Optional ramp filtering for smoother acceleration and deceleration.
- Neutral-width compensation for motors that need a higher duty cycle to start moving.
- Header-only non-blocking implementation with no dynamic allocation.

## Installation

In the Arduino IDE, use **Sketch > Include Library > Add .ZIP Library...** and select a ZIP of this repository. The library can also be installed through the Arduino Library Manager after publication.

## Wiring

Connect one motor to `AOUT1` and `AOUT2` on the DRV8833 breakout board. Connect the corresponding logic inputs to two Arduino pins:

| DRV8833 | Arduino |
| --- | --- |
| AIN1 | `motorIn1` |
| AIN2 | `motorIn2` |
| GND | GND |
| VM | Motor power supply |

The motor supply, logic supply, and current limit must follow the requirements of the specific DRV8833 breakout board. Do not power the motor from an Arduino I/O pin.

For variable speed in both directions, use PWM-capable pins for both inputs. The library drives one input with PWM and holds the other low for each direction.

## Basic usage

Include `DRV8833MotorDriver.h`, construct a low-level motor and a motor driver, then call `begin()` from `setup()`:

```cpp
#include <DRV8833MotorDriver.h>

DRV8833 motor(5, 6);
DRV8833MotorDriver driver(motor);

void setup() {
  driver.begin();
}

void loop() {
  driver.setSpeed(80); // Forward, from minSpeed to maxSpeed.
  driver.run();
}
```

`setSpeed()` accepts values from `minSpeed` through `maxSpeed` (both default to `-127` and `127`). Call `run()` regularly to advance the ramp. `stop()` sets the target speed to zero. Call `begin()` before `setSpeed()`, `run()`, or `stop()`.

## Ramp filter

The `RampFilter` is available as `driver.filter`:

```cpp
driver.filter.setFilterFactor(20); // Speed units per 100 ms.
driver.setSpeed(driver.maxSpeed);
```

Set the factor to `0` to disable ramping. A disabled filter applies the target speed on the next `run()` call.

## Direct PWM control

For applications that do not need speed limits or ramping, use `DRV8833` directly. `setMotorPwm()` accepts `-255` to `255`, where negative values reverse the motor:

```cpp
DRV8833 motor(5, 6, true); // Optional direction inversion.

void setup() {
  motor.begin();
}

void loop() {
  motor.setMotorPwm(150);
}
```

## Example

Open `examples/DRV8833/DRV8833.ino` in the Arduino IDE. It reads a potentiometer on `A0` and maps it to the configured signed speed range.

## Tasks

- [ ] Use better logging framework
- [ ] Add schematics
- [ ] Refactor code to be compatible with all the popular DC motor drivers

## License

MIT. See [LICENSE.txt](LICENSE.txt).
