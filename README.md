# MotorDriveUnit

High-level Arduino library for controlling single and dual DC motors using PWM or digital pins, compatible with Arduino-compatible boards (ESP32, Arduino Uno, Mega, etc.). Designed for robotics projects such as Robo-Futbol. Supports differential and tank drive control, with optional input sources like joysticks or gamepads.

## Features

* High-level control of individual DC motors (`Motor` class).
* Dual-motor control with differential or tank drive (`MotorDriveUnit` class).
* Deadzone filtering to prevent unintended small motor movements.
* Exhibition mode (persisted motor power while a button is released).
* Utility functions for clamping and PWM mapping (`Utils` namespace).
* Compatible with any gamepad library, as long as input values are mapped to -255 to 255.
* Works with any Arduino-compatible board (ESP32, Arduino Uno, Mega, etc.).
* Input convention: values < 0 are interpreted as -x (left) and -y (reverse); values > 0 are +x (right) and +y (forward).

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
   └─ RoboFutbol.ino
```

2. Include the library in your sketch:

```cpp
#include <Motor.h>
#include <MotorDriveUnit.h>
#include <Utils.h>
```

3. Install any gamepad library you wish to use and map the input signals to a range of -255 to 255.

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
* `getLeftMotor()` / `getRightMotor()` – Access underlying `Motor` instances.

### `Utils` Namespace

Helper functions for motor input handling.

```cpp
int16_t utils::clamp(int16_t value, int16_t min_value, int16_t max_value);
int16_t utils::map_for_pwm(int16_t value, int16_t in_min, int16_t in_max);
```

## Example Usage

```cpp
#include <MotorDriveUnit.h>
#include <Bluepad32.h>

MotorDriveUnit motor_driver(27, 14, 17, 16);

ControllerPtr current_controller = nullptr;

int16_t power_source() {
    uint8_t LT = current_controller->l1() ? 255 : current_controller->brake() / 4;
    uint8_t RT = current_controller->r1() ? 255 : current_controller->throttle() / 4;
    return RT - LT;
}

int16_t direction_source() {
    return current_controller->axisX() / 2;
}

void setup() {
    Serial.begin(115200);
    BP32.setup(&on_connect, &on_disconnect);
    motor_driver.begin();
    motor_driver.setDeadzone(75);
    motor_driver.setPowerSource(power_source);
    motor_driver.setDirectionSource(direction_source);
}

void loop() {
    BP32.update();

    if (current_controller && current_controller->isConnected() && current_controller->isGamepad()) {
        motor_driver.setExpositionActive(current_controller->y());

        if (current_controller->x())
            motor_driver.toggleTankDriveMode();

        motor_driver.update();
    } else {
        motor_driver.stop();
    }
}
```

## Notes

* Always call `begin()` before controlling motors.
* Adjust the deadzone for smoother control and to avoid motor jitter.
* Use `setExpositionActive()` and `toggleTankDriveMode()` to enable advanced behaviors.
* Input convention: negative = left/reverse, positive = right/forward.
* Compatible with ESP32, Arduino Uno, Mega, and any board supporting PWM/digital pins.

## License

MIT License – free for personal and commercial use.
