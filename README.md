# MotorDriveUnit

A high-level Arduino library for controlling single and dual DC motors using PWM or digital pins, designed for Arduino-compatible boards (ESP32, Arduino Uno, Mega, etc.) for robotics projects such as Robo-Futbol. Includes support for differential and tank drive control, with optional input sources like joysticks or gamepads.

## Features

* High-level control of individual DC motors (`Motor` class).
* Dual-motor control with differential or tank drive (`MotorDriveUnit` class).
* Deadzone filtering to prevent small unintended motor movements.
* Utility functions for clamping and PWM mapping (`Utils` namespace).
* Compatible with any gamepad library, as long as input values are mapped to a range of -255 to 255.
* Works with any Arduino-compatible board (ESP32, Arduino Uno, Mega, etc.).
* Input convention: values < 0 are interpreted as -x (left) and -y (reverse); values > 0 are interpreted as +x (right) and +y (forward).

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
   └─ Example.ino
```

2. Include the library in your sketch:

```cpp
#include <Motor.h>
#include <MotorDriveUnit.h>
#include <Utils.h>
```

3. Install any gamepad library you wish to use and map the input signals to the range -255 to 255 for compatibility.

## Classes and Functions

### `Motor`

High-level controller for a single DC motor.

**Constructor:**

```cpp
Motor(bool digital_direction, bool digital_enable,
      uint8_t forward_pin, uint8_t backward_pin, uint8_t enable_pin = 255);
```

**Methods:**

* `begin()` – Initialize pins and motor.
* `setPower(int16_t power)` – Set power (-255 to +255).
* `forward(uint8_t power)` – Move forward at specified power.
* `backward(uint8_t power)` – Move backward at specified power.
* `stop()` – Stop the motor.
* `setDigitalPinDeadZone(uint8_t deadzone)` – Set threshold for digital outputs.

### `MotorDriveUnit`

Controller for dual motors with optional differential/tank drive.

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
* `applyDifferentialDrive()` – Apply differential steering using power/direction sources.
* `applyTankDrive()` – Apply tank drive steering using power/direction sources.
* `stop()` – Stop both motors.
* `setDeadzone(uint8_t deadzone)` – Minimum effective input magnitude.
* `setPowerSource(int16_t (*)())` – Function providing global power.
* `setDirectionSource(int16_t (*)())` – Function providing direction offset.

### `Utils` Namespace

Utility functions for motor input handling.

```cpp
int16_t utils::clamp(int16_t value, int16_t min_value, int16_t max_value);
int16_t utils::map_for_pwm(int16_t value, int16_t in_min, int16_t in_max);
```

## Example Usage

```cpp
#include <Motor.h>
#include <MotorDriveUnit.h>
#include <Utils.h>
#include <Bluepad32.h>

MotorDriveUnit motor_driver(27, 14, 17, 16);

int16_t power_source() {
    return current_controller->r1() ? 255 : current_controller->l1() ? -255 : 0;
}

int16_t direction_source() {
    return current_controller->axisX() / 2;
}

void setup() {
    Serial.begin(115200);
    motor_driver.begin();
    motor_driver.setDeadzone(75);
    motor_driver.setPowerSource(power_source);
    motor_driver.setDirectionSource(direction_source);
}

void loop() {
    BP32.update();

    if (current_controller && current_controller->isConnected()) {
        motor_driver.applyDifferentialDrive();
    } else {
        motor_driver.stop();
    }
}
```

## Notes

* Ensure `begin()` is called before controlling motors.
* Adjust the deadzone for smoother control and to avoid motor jitter.
* Use either `applyDifferentialDrive()` or `applyTankDrive()` depending on your robot steering style.
* Compatible with any Arduino board and any gamepad library, as long as the input signals are mapped to -255 ... 255.
* Convention for inputs: negative values indicate left (-x) or reverse (-y), positive values indicate right (+x) or forward (+y).

## License

MIT License – free for personal and commercial use.
