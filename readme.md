# MotorDriveUnit

High-level Arduino library for controlling single and dual DC motors using PWM or digital pins, compatible with Arduino-compatible boards (ESP32, Arduino Uno, Mega, etc.). Designed for robotics projects. Supports differential and tank drive control, with optional direct input sources.

## Features

* High-level control of individual DC motors (`Motor` class).
* Dual-motor control with differential or tank drive (`MotorDriveUnit` class).
* Deadzone filtering to prevent unintended small motor movements.
* Temporary and persistent tank drive modes.
* Manual motor control for direct power and direction assignment.
* Exhibition mode (persisted motor power while a button is released).
* Utility functions for clamping and PWM mapping (`Utils` namespace).
* Works with any Arduino-compatible board (ESP32, Arduino Uno, Mega, etc.).
* Input convention: values < 0 = reverse/left, values > 0 = forward/right.

## Installation

1. Download the library source files and place them in your Arduino `libraries` folder:

```
ESP32MotorControl/
├─ src/
│  ├─ Motor.h
│  ├─ Motor.cpp
│  ├─ MotorDriveUnit.h
│  ├─ MotorDriveUnit.cpp
│  └─ Utils.h
└─ examples/
   ├─ ps2x_four_pwm/ps2x_four_pwm.ino
   └─ bluepad32_four_pwm/bluepad32_four_pwm.ino
```

2. Include the library in your sketch:

```cpp
#include <MotorDriveUnit.h>
```

## MotorDriveUnit Configurations

The `MotorDriveUnit` constructor supports multiple configurations to suit different H-bridge or motor driver setups:

1. **Four PWM inputs (e.g., L298N)**

```cpp
MotorDriveUnit motor_driver(false, false, forward_left_pin, backward_left_pin, forward_right_pin, backward_right_pin, Motor::PIN_UNUSED, Motor::PIN_UNUSED);
```

All four inputs are PWM, ideal for standard H-bridge boards.

2. **Digital inputs + PWM enable**

```cpp
MotorDriveUnit motor_driver(true, false, forward_left_pin, backward_left_pin, forward_right_pin, backward_right_pin, left_enable_pin, right_enable_pin);
```

All four direction pins are digital, while the enable pins use PWM. This allows controlling speed with only two PWM pins instead of four.

3. **PWM inputs + digital enable**

```cpp
MotorDriveUnit motor_driver(false, true, forward_left_pin, backward_left_pin, forward_right_pin, backward_right_pin, left_enable_pin, right_enable_pin);
```

Same as the first configuration, but allows for digital enable, possibly to manually control the activation of bridges h instead of always being active. This can be useful in special cases.

## Classes and Functions

### `Motor`

High-level controller for a single DC motor.

**Constructor:**

```cpp
Motor(bool digital_direction, bool digital_enable,
      uint8_t forward_pin, uint8_t backward_pin, uint8_t enable_pin = Motor::PIN_UNUSED);
```

**Methods:**

* `begin()` – Initialize pins and motor.
* `setPower(int16_t power)` – Set power (-255 to +255). Positive = forward, negative = backward.
* `forward(uint8_t power)` – Move forward at specified power.
* `backward(uint8_t power)` – Move backward at specified power.
* `stop()` – Stop the motor immediately.
* `setDigitalPinDeadZone(uint8_t deadzone)` – Set threshold for digital output pins.

### `MotorDriveUnit`

Controller for dual motors with differential or tank drive modes.

**Constructors:**

```cpp
MotorDriveUnit(uint8_t forward_left, uint8_t backward_left,
               uint8_t forward_right, uint8_t backward_right);

MotorDriveUnit(uint8_t forward_left, uint8_t backward_left,
               uint8_t forward_right, uint8_t backward_right,
               uint8_t enable_left, uint8_t enable_right);

MotorDriveUnit(bool digital_direction, bool digital_enable,
               uint8_t forward_left, uint8_t backward_left,
               uint8_t forward_right, uint8_t backward_right,
               uint8_t enable_left, uint8_t enable_right);
```

**Methods:**

* `begin()` – Initialize both motors.
* `stop()` – Stop both motors immediately.
* `update()` – Update motor outputs based on current power and direction sources.
* `setDeadzone(uint8_t deadzone)` – Minimum effective input magnitude.
* `setPowerSource(int16_t (*)())` – Function providing global power (-255 … 255).
* `setDirectionSource(int16_t (*)())` – Function providing direction offset (-255 … 255).
* `setExpositionActive(bool state)` – Activate exhibition mode (press/release behavior).
* `toggleTankDriveMode()` – Switch between differential and tank drive behavior.
* `useTankDrive()` – Temporarily enable tank drive for a single update cycle.
* `useManualDrive(uint8_t left_power, bool left_forward, uint8_t right_power, bool right_forward)` – Directly control motor powers and directions.
* `getLeftMotor()` / `getRightMotor()` – Access underlying `Motor` instances.
* `getPowerSourceFunction()` / `getDirectionSourceFunction()` – Access the current input source callbacks.

### `Utils` Namespace

Helper functions for motor input handling.

```cpp
int16_t utils::clamp(int16_t value, int16_t min_value, int16_t max_value);
int16_t utils::map_for_pwm(int16_t value, int16_t in_min, int16_t in_max);
```

## Example Usage

The library includes ready-to-use examples demonstrating different gamepad integrations:

1. **PS2X controller with four PWM motor inputs:** `examples/ps2x_four_pwm/ps2x_four_pwm.ino`
   Demonstrates controlling motors using the classic PS2X controller and full PWM pins.

2. **Bluepad32 controller with four PWM motor inputs:** `examples/bluepad32_four_pwm/bluepad32_four_pwm.ino`
   Shows integration with Bluepad32-supported gamepads for motor control.

## Notes

* Always call `begin()` before controlling motors.
* Adjust the deadzone for smoother control and to avoid motor jitter.
* Use `setExpositionActive()`, `toggleTankDriveMode()`, and `useTankDrive()` to enable advanced behaviors.
* `useTankDrive()` only lasts for a single `update()` cycle.
* Input convention: negative = left/reverse, positive = right/forward.
* Compatible with ESP32, Arduino Uno, Mega, and any board supporting PWM/digital pins.

## License

MIT License – free for personal and commercial use.
