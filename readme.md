# MotorDriveUnit

High-level Arduino library for controlling single and dual DC motors using digital or PWM pins. Designed for robotics projects on ESP32, Arduino Uno, Mega, and similar boards. Version **2.0.0** introduces a modern, flexible, header-only architecture.

---

# 🚀 What's New in V2.0.0

The library has been **fully redesigned** for clarity, flexibility, and safer runtime control.

### ✔ Header-only architecture

* `Motor.cpp` and `MotorDriveUnit.cpp` were removed.
* All logic now resides in `.h` files.

### ✔ No more constructors

Pin configuration is now done **at runtime** using:

* `Motor::setDirectionPins(...)`
* `Motor::setEnablePin(...)`
* `MotorDriveUnit::setDriverEnablePin(...)`

### ✔ Safe, consistent motor behavior

* Direction reversal now stops the motor momentarily.
* Digital dead-zone supported.
* Unified filtering and clamping.

### ✔ Clear initialization flow

1. Retrieve motors
2. Assign direction + enable pins
3. (Optional) Assign global driver enable pin
4. Call `begin()`
5. Apply control modes

---

# 📦 Installation

Place the library in your Arduino `libraries` folder with this structure:

```
MotorDriveUnit
/
└─ src/
   ├─ Motor.h
   ├─ MotorDriveUnit.h
   └─ Utils.h
```

Include it in your sketch:

```cpp
#include <MotorDriveUnit.h>
```

---

# 🧩 Key Components

## `Motor`

Controls a **single DC motor** using either digital or PWM outputs.

### ✔ Configuration

```cpp
motor.setDirectionPins(forward_pin, backward_pin, /*digital?*/ true_or_false);
motor.setEnablePin(enable_pin, /*digital?*/ true_or_false);
```

### ✔ Methods

* `begin()` — initializes assigned pins.
* `setPower(int16_t power)` — range: `-255` to `+255`.
* `forward(uint8_t power)`
* `backward(uint8_t power)`
* `stop()`
* `setDigitalPinDeadZone(uint8_t value)`

### Behavior Notes

* If direction changes from forward → backward, the motor **stops briefly** for safety.
* In PWM mode, effective power is the **max** of forward/backward values.

---

## `MotorDriveUnit`

Controls **two motors together**, providing high-level drive logic.

### ✔ Configuration Flow

```cpp
MotorDriveUnit mdu;

auto& left  = mdu.getLeftMotor();
auto& right = mdu.getRightMotor();

left.setDirectionPins(...);
left.setEnablePin(...);

right.setDirectionPins(...);
right.setEnablePin(...);

mdu.setDriverEnablePin(pin, /*digital?*/ true_or_false);

mdu.begin();
```

### ✔ Features

* Differential drive (global power + direction offset)
* Tank drive mode (left/right independent inputs)
* Manual mode for direct control
* Exhibition Mode
* Unified dead-zone filtering
* Global driver enable control

### ✔ Methods

* `begin()`
* `stop()`
* `update()`
* `setDeadzone(uint8_t)`
* `setPowerSource(int16_t (*)())`
* `setDirectionSource(int16_t (*)())`
* `setExpositionActive(bool)`
* `toggleTankDriveMode()`
* `useTankDrive()` — one-cycle only
* `useManualDrive(left_power, left_forward, right_power, right_forward)`

---

# ⚙ Drive Modes

## **1. Differential Drive (default)**

Uses two inputs:

* **Power** (forward/reverse)
* **Direction** (right/left offset)

MDU computes the resulting left/right motor outputs.

## **2. Tank Drive**

Left and right inputs are independent.

* Activate permanently → `toggleTankDriveMode()`
* Activate for one update → `useTankDrive()`

## **3. Exhibition Mode**

Allows power to persist while a button is released.
Activated with:

```cpp
mdu.setExpositionActive(true);
```

---

# 🔌 Global Driver Enable Pin

Used to activate/deactivate **both motors at once**.

```cpp
mdu.setDriverEnablePin(pin, /*digital?*/ true_or_false);
```

If digital: HIGH = ON, LOW = OFF.
If PWM: value is scaled dynamically.

MDU ensures this pin is enabled **before applying power**.

---

# 📚 Example (Basic Differential Drive)

```cpp
#include <MotorDriveUnit.h>

MotorDriveUnit mdu;

int16_t readPower() { return 150; }
int16_t readDir()   { return -50; }

void setup() {
  auto& L = mdu.getLeftMotor();
  auto& R = mdu.getRightMotor();

  L.setDirectionPins(18, 19, true);
  L.setEnablePin(23, false);

  R.setDirectionPins(25, 26, true);
  R.setEnablePin(27, false);

  mdu.setDriverEnablePin(5, true);

  mdu.setPowerSource(readPower);
  mdu.setDirectionSource(readDir);

  mdu.begin();
}

void loop() {
  mdu.update();
}
```

---

# ⚠ Breaking Changes from Previous Versions

* **All constructors removed.**
* No more digital/PWM flags inside constructors.
* Pin assignments must now be done through setter methods.
* `.cpp` files removed.
* Old initialization examples are incompatible.

To migrate:

1. Instantiate `MotorDriveUnit` with no arguments.
2. Retrieve motors.
3. Assign direction and enable pins.
4. Call `begin()`.

---

# 📝 Utils

Helper functions:

```cpp
int16_t utils::clamp(int16_t v, int16_t mn, int16_t mx);
int16_t utils::map_for_pwm(int16_t value, int16_t in_min, int16_t in_max);
```

---

# 🧷 Notes

* Always call `begin()` before use.
* Digital dead-zone is essential when using digital direction pins.
* Tank Drive temporary mode applies only to the next `update()` call.
* Negative values → reverse/left; positive → forward/right.

---

# 📄 License

MIT License — free for personal and commercial use.
