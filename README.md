# Advanced Arduino H-Bridge control library

## Tasks

- Add deadzone (or something like that)
- add speed limit
- Create examples
- Create docs
- Create arduino library documents
- Publish on Github, platform IO, Arduino
- Create unniversal adapter architecture: Gabor sad that it is a bad idea, because I need to use a lot of defines and it will mess up the code.
- [x] Add Fader Filter

This is an Arduino library for the Texas Instruments DRV8833 DC motor driver

## Features:
- Adjustable fader for smooth speed changes
- 3 brake modes: no brake (floating motor), brake active, if PWM > 0, brake always active
- "Light" operation mode, if only one PWM capable pin is available. The brake is then always active in one direction and inactive in the other
- selectable input signal range (e.g. 0 - 100, 0 - 1023, -255 - 255 etc.)
- selectable neutral position width. This allows you to optimize it for your joystick
- the motor rotation direction is reversible in software, so you don't have to switch your motor wires, if the direction is reversed
- The end-speed is adjustable during runtime. This allows you to simulate different gear ratios

## Usage

See [example](https://github.com/TheDIYGuy999/DRV8833/blob/master/examples/DRV8833/DRV8833.ino).


(c) 2016 TheDIYGuy999