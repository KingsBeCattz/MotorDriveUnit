# MotorDriveUnit

High-level Arduino library for controlling single and dual DC motors using digital or PWM pins. Designed for robotics projects on ESP32, Arduino Uno, Mega, and similar boards. Version **2.2.2** fixes direction flag logic and method delegation in `Motor`.

---

# 🚀 What's New in V2.2.2

### ✔ `Motor` — Bug Fixes

- `_forward` direction flag: `(forward_power >= backward_power)` incorrectly evaluated to `true` when both powers were equal (motor stopped). Fixed to `(forward_power - backward_power >= 0)`.
- `setPower` now delegates to `forward()` and `backward()` instead of calling `_apply_power()` directly.
- `forward()` and `backward()` no longer apply redundant clamping — `setPower` already delivers a clamped value before delegating.

---

# 🚀 What's New in V2.2.1

### ✔ `MotorDriveUnit` — Const Overloads

- `getLeftMotor() const` — returns a `const Motor &` to the left motor.
- `getRightMotor() const` — returns a `const Motor &` to the right motor.

---

# 🚀 What's New in V2.2.0

### ✔ `Motor` — New Getters

- `getCurrentPower() const` — returns the last applied PWM value as `UnsignedPWM`.
- `isForward() const` — returns `true` if the motor is currently spinning forward.
- `isDirectionDigital() const` — returns whether direction pins are configured as digital.
- `getDirectionPins() const` — returns a `std::pair<Pin, Pin>` of `(forward_pin, backward_pin)`.
- `getEnablePin() const` — returns a `std::pair<Pin, bool>` of `(enable_pin, digital_mode)`.

### ✔ `MotorDriveUnit` — New Getters

- `getPowerSourceFunction() const` — returns the currently assigned power source function, or `nullptr` if none.
- `getDirectionSourceFunction() const` — returns the currently assigned direction source function, or `nullptr` if none.
- `getDriverEnablePin() const` — returns a `std::pair<Pin, bool>` of `(driver_enable_pin, digital_mode)`.

---

# 🚀 What's New in V2.1.0

### ✔ Typed PWM system

A new `types.h` header introduces semantic type aliases:

* `SignedPWM` — signed value in `[-255, 255]`, encodes both magnitude and direction.
* `UnsignedPWM` — unsigned value in `[0, 255]`, encodes magnitude only.
* `Pin` — pin number alias, replaces raw `uint8_t` in all pin parameters.
* `SourceFn` — function pointer type `SignedPWM (*)()`, used for injectable input sources.

### ✔ Updated method signatures

All public methods now use the typed aliases instead of raw primitives:

* `setPowerSource(SourceFn)` replaces `setPowerSource(int16_t (*)())`
* `setDirectionSource(SourceFn)` replaces `setDirectionSource(int16_t (*)())`
* `setDeadzone(UnsignedPWM)` replaces `setDeadzone(uint8_t)`
* `useManualDrive(UnsignedPWM, bool, UnsignedPWM, bool)` replaces `uint8_t` parameters
* `setPower(SignedPWM)`, `forward(UnsignedPWM)`, `backward(UnsignedPWM)` in `Motor`

### ✔ Updated utils

```cpp
utils::clamp(int16_t, int16_t, int16_t);
utils::signed_pwm_clamp(int16_t);   // clamps to [-255, 255] → SignedPWM
utils::unsigned_pwm_clamp(int16_t); // clamps to [0, 255]   → UnsignedPWM
```

---

# 📦 Installation

Place the library in your Arduino `libraries` folder with this structure:

```
MotorDriveUnit/
└─ src/
   ├─ types.h
   ├─ Utils.h
   ├─ Motor.h
   └─ MotorDriveUnit.h
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

> **Note:** Pin modes are configured during `begin()`. Call setters before `begin()`.

### ✔ Methods

* `begin()` — initializes assigned pins.
* `setPower(SignedPWM power)` — range: `-255` to `+255`.
* `forward(UnsignedPWM power)`
* `backward(UnsignedPWM power)`
* `stop()`
* `setDigitalPinDeadZone(UnsignedPWM value)`
* `getCurrentPower() const` — last applied PWM value.
* `isForward() const` — current rotation direction.
* `isDirectionDigital() const` — direction pin mode.
* `getDirectionPins() const` — returns `std::pair<Pin, Pin>` of `(forward_pin, backward_pin)`.
* `getEnablePin() const` — returns `std::pair<Pin, bool>` of `(enable_pin, digital_mode)`.

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
* Exhibition mode
* Unified dead-zone filtering
* Global driver enable control

### ✔ Methods

* `begin()`
* `stop()`
* `update()`
* `setDeadzone(UnsignedPWM)`
* `setPowerSource(SourceFn)`
* `setDirectionSource(SourceFn)`
* `setExpositionActive(bool)`
* `toggleTankDriveMode()`
* `useTankDrive()` — one-cycle only
* `useManualDrive(UnsignedPWM, bool, UnsignedPWM, bool)`
* `getDeadzone() const` — current deadzone threshold.
* `getPowerSourceFunction() const` — assigned power source, or `nullptr`.
* `getDirectionSourceFunction() const` — assigned direction source, or `nullptr`.
* `getDriverEnablePin() const` — returns `std::pair<Pin, bool>` of `(driver_enable_pin, digital_mode)`.
* `getLeftMotor() const` — returns a `const Motor &` to the left motor.
* `getRightMotor() const` — returns a `const Motor &` to the right motor.

---

# ⚙ Drive Modes

## **1. Differential Drive (default)**

Uses two inputs:

* **Power** — forward/reverse magnitude and direction (`SignedPWM`)
* **Direction** — left/right offset (`SignedPWM`)

MDU computes the resulting left/right motor outputs each `update()`.

## **2. Tank Drive**

Left and right inputs are independent.

* Activate permanently → `toggleTankDriveMode()`
* Activate for one update → `useTankDrive()`

## **3. Exhibition Mode**

Spins the robot in place. When deactivated, the last non-zero power is held indefinitely until it returns to zero.

```cpp
mdu.setExpositionActive(true);  // start
mdu.setExpositionActive(false); // release — sustained spin if power was non-zero
```

---

# 🔌 Global Driver Enable Pin

Used to activate/deactivate **both motors at once** (e.g. TB6612FNG STBY pin).

```cpp
mdu.setDriverEnablePin(pin, /*digital?*/ true_or_false);
```

* Digital: `HIGH` = enabled, `LOW` = standby.
* PWM: `255` = enabled, `0` = standby.

MDU ensures this pin is asserted **before** sending any motor commands.

---

# 📚 Example (Basic Differential Drive)

```cpp
#include <MotorDriveUnit.h>

MotorDriveUnit mdu;

SignedPWM readPower() { return 150; }
SignedPWM readDir()   { return -50; }

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

# ⚠ Breaking Changes

## From 2.2.1 → 2.2.2

No breaking changes. Bug fixes only.

## From 2.2.0 → 2.2.1

No breaking changes. Adds const overloads for `getLeftMotor()` and `getRightMotor()`.

## From 2.1.x → 2.2.0

No breaking changes. All additions are backwards-compatible getters.

## From 2.0.0 → 2.1.0

* `setPowerSource` and `setDirectionSource` now require a `SourceFn` (`SignedPWM (*)()`) instead of `int16_t (*)()`.
* `setDeadzone`, `setDigitalPinDeadZone` now take `UnsignedPWM` instead of `uint8_t`.
* `useManualDrive` now takes `UnsignedPWM` instead of `uint8_t` for power parameters.
* `setPower` now takes `SignedPWM` instead of `int16_t`.
* `forward` and `backward` now take `UnsignedPWM` instead of `uint8_t`.
* Pin parameters across all methods now use `Pin` instead of `uint8_t`.
* `utils::map_for_pwm` removed. Use `utils::signed_pwm_clamp` or `utils::unsigned_pwm_clamp`.

## From pre-2.0.0

* All constructors removed.
* `.cpp` files removed — library is now header-only.
* Pin assignments must be done through setter methods before `begin()`.

---

# 🧷 Notes

* Always call `begin()` after configuring pins and before `update()`.
* Digital dead-zone is essential when using digital direction pins.
* Tank Drive temporary mode applies only to the next `update()` call.
* Negative `SignedPWM` values → reverse/left; positive → forward/right.

---

# 📄 License

MIT License — free for personal and commercial use.