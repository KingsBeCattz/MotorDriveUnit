# CHANGELOG

## V2.2.0

### `Motor` — New Getters

- `getCurrentPower() const` — returns the last applied PWM value as `UnsignedPWM`.
- `isForward() const` — returns `true` if the motor is currently spinning forward.
- `isDirectionDigital() const` — returns whether direction pins are configured as digital.
- `getDirectionPins() const` — returns a `std::pair<Pin, Pin>` of `(forward_pin, backward_pin)`.
- `getEnablePin() const` — returns a `std::pair<Pin, bool>` of `(enable_pin, digital_mode)`.

### `MotorDriveUnit` — New Getters

- `getPowerSourceFunction() const` — returns the currently assigned power source function, or `nullptr` if none.
- `getDirectionSourceFunction() const` — returns the currently assigned direction source function, or `nullptr` if none.
- `getDriverEnablePin() const` — returns a `std::pair<Pin, bool>` of `(driver_enable_pin, digital_mode)`.

---

## V2.1.1

### `MotorDriveUnit` — Internal Changes

- `SourceFn` typedef moved from `types.h` into `MotorDriveUnit` class scope (public section).

### `Motor` — Internal Changes

- `PIN_UNUSED` changed from `static const Pin` to `static constexpr Pin`.

---

## V2.1.0

### New: Type System (`types.h`)

A dedicated `types.h` header was introduced, centralizing all primitive type aliases used across the library. This eliminates raw `uint8_t` and `int16_t` from public APIs and replaces them with semantically meaningful types.

#### New types:

- `Pin` (`unsigned char`) — represents a hardware pin number. Replaces all `uint8_t` pin parameters across `Motor` and `MotorDriveUnit`.
- `SignedPWM` (`short`) — signed PWM value in `[-255, 255]`. Encodes both magnitude and direction. Negative = reverse/left, positive = forward/right.
- `UnsignedPWM` (`unsigned char`) — unsigned PWM value in `[0, 255]`. Encodes magnitude only, used where direction is not relevant.
- `SourceFn` (`SignedPWM (*)()`) — function pointer type for injectable input sources. Replaces raw `int16_t (*)()` in `setPowerSource` and `setDirectionSource`.

---

### `Motor` — Breaking API Changes

All method signatures updated to use typed aliases:

- `setPower(int16_t)` → `setPower(SignedPWM)`
- `forward(uint8_t)` → `forward(UnsignedPWM)`
- `backward(uint8_t)` → `backward(UnsignedPWM)`
- `setDigitalPinDeadZone(uint8_t)` → `setDigitalPinDeadZone(UnsignedPWM)`
- `getDigitalPinDeadZone()` → returns `UnsignedPWM` instead of `uint8_t`
- `setDirectionPins(uint8_t, uint8_t, bool)` → `setDirectionPins(Pin, Pin, bool)`
- `setEnablePin(uint8_t, bool)` → `setEnablePin(Pin, bool)`
- `PIN_UNUSED` → type changed from `uint8_t` to `Pin`

#### Internal changes:

- `_forward_pin`, `_backward_pin`, `_enable_pin` → type changed to `Pin`
- `_digital_pin_dead_zone`, `_pwm_value` → type changed to `UnsignedPWM`
- `_write_pin(uint8_t, uint8_t, bool, bool)` → `_write_pin(Pin, UnsignedPWM, bool, bool)`
- `_set_enable(uint8_t)` → `_set_enable(UnsignedPWM)`
- `_apply_power(uint8_t, uint8_t)` → `_apply_power(UnsignedPWM, UnsignedPWM)`
- In `setPower`: `clamped_power` type changed from `int16_t` to `UnsignedPWM`
- Redundant `static_cast` removed from `forward()` and `backward()` — `unsigned_pwm_clamp` already returns `UnsignedPWM`

#### Behavior changes:

- `setDirectionPins()` and `setEnablePin()` no longer call `pinMode()` internally when invoked after `begin()`. Pin mode setup is now exclusively handled by `begin()`. Setters must be called before `begin()`.

#### Guard change:

- `#ifndef Motor_h` / `#define Motor_h` / `#endif` replaced with `#pragma once`

#### Includes:

- `#include "types.h"` added

---

### `MotorDriveUnit` — Breaking API Changes

All method signatures updated to use typed aliases:

- `setPowerSource(int16_t (*)())` → `setPowerSource(SourceFn)`
- `setDirectionSource(int16_t (*)())` → `setDirectionSource(SourceFn)`
- `setDeadzone(uint8_t)` → `setDeadzone(UnsignedPWM)`
- `getDeadzone()` → returns `UnsignedPWM` instead of `uint8_t`
- `setDriverEnablePin(uint8_t, bool)` → `setDriverEnablePin(Pin, bool)`
- `useManualDrive(uint8_t, bool, uint8_t, bool)` → `useManualDrive(UnsignedPWM, bool, UnsignedPWM, bool)`
- `getPowerSourceFunction()` → return type changed from `int16_t (*)()` to `SourceFn`
- `getDirectionSourceFunction()` → return type changed from `int16_t (*)()` to `SourceFn`

#### Internal changes:

- `_driver_enable_pin` → type changed from `uint8_t` to `Pin`
- `_deadzone` → type changed from `uint8_t` to `UnsignedPWM`
- `_exposition_hold_power` → type changed from `int16_t` to `SignedPWM`
- `_get_power_source` → type changed from `int16_t (*)()` to `SourceFn`
- `_get_direction_source` → type changed from `int16_t (*)()` to `SourceFn`
- `_apply_deadzone(int16_t, uint8_t)` → second parameter changed to `UnsignedPWM`
- `_apply_deadzone_to_source` → return type and parameter changed to `SignedPWM`
- `power` variable in `update()` → type changed from `int16_t` to `SignedPWM`
- `powers[2]` array in `update()` → element type changed from `int16_t` to `SignedPWM`

#### Behavior changes:

- `useManualDrive()` now manages the driver enable / STBY pin consistently with `update()`:
  - If both powers are zero, motors stop and driver enters standby.
  - Otherwise, driver is woken before motor commands are issued.
  - Previously, `useManualDrive()` sent motor commands without managing the STBY pin at all.
- `setExpositionActive()` now pre-asserts driver enable before sustained exposition activates, eliminating a one-frame window where motor commands could be issued while STBY was still LOW.
- In `update()`, exhibition mode (immediate) now wakes the driver **before** sending motor commands. Previously the order was reversed.
- `_write_driver_enable_pin()` private helper introduced, centralizing all writes to the driver enable pin and respecting `_digital_driver_enable`. Previously `toggleDriverEnabled()`, `setDriverEnabled()`, and `setDriverEnablePin()` each performed their own `digitalWrite()` directly, and none respected the analog write path.
- `setDriverEnablePin()` now uses `_write_driver_enable_pin()` to reflect initial state, correctly respecting `digital_enable`. Previously it always called `digitalWrite()` regardless of the `digital_enable` flag.

#### Documentation changes:

- Class-level doc updated: `setPowerSource(int16_t (*)())` and `setDirectionSource(int16_t (*)())` in the Usage Lifecycle section replaced with `setPowerSource(SourceFn)` and `setDirectionSource(SourceFn)`.
- All `// FIX:` inline comments removed from implementation.

#### Guard change:

- `#ifndef MotorDriveUnit_h` / `#define MotorDriveUnit_h` / `#endif` replaced with `#pragma once`

#### Includes:

- `#include "types.h"` added

---

### `Utils` — API Changes

- `utils::clamp(int16_t, int16_t, int16_t)` — retained, now serves as the base primitive for the two typed wrappers below.
- `utils::signed_pwm_clamp(int16_t)` — new. Clamps to `[-255, 255]` and returns `SignedPWM`. Replaces direct calls to `utils::clamp(value, -255, 255)` throughout the codebase.
- `utils::unsigned_pwm_clamp(int16_t)` — new. Clamps to `[0, 255]` and returns `UnsignedPWM`.
- `utils::map_for_pwm()` — removed.

---

## V2.0.0

### Motor Class Updates
- Reworked the Motor class into a **header-only implementation** (removed `Motor.cpp`).
- Removed constructor-based configuration; the class now uses **runtime pin assignment** via:
  - `setDirectionPins()`
  - `setEnablePin()`
- Improved flexibility: direction pins and enable pin now support **independent digital/PWM modes**.
- Added **safe direction switching**: `setPower()` now stops the motor and waits briefly before reversing.
- `_apply_power()` updated to use the **max** of forward/backward PWM as the effective enable value.
- Added digital-mode **dead-zone support** (`setDigitalPinDeadZone()`).
- `begin()` now initializes only assigned pins and always starts the motor in a safe stopped state.
- Internal logic consolidated; simplified state transitions and removed redundant operations.

---

### MotorDriveUnit Class Updates

#### New Features
- Added support for a **global driver enable pin** via `setDriverEnablePin()`, allowing complete activation/deactivation of both motors through a single digital or PWM signal.
- New configuration architecture based on **external motor setup**:
  - Motors now receive pin configuration through:
    - `Motor::setDirectionPins(...)`
    - `Motor::setEnablePin(...)`
  - Old constructor-based pin assignment removed.
- Extensive documentation added, covering initialization flow, operating modes, dead-zone behavior, Exposition Mode, Tank Drive behavior, and input requirements.

#### Behavior Changes
- Unified clamping and dead-zone filtering for all drive inputs.
- Exposition Mode and Tank Drive modes now follow clearly defined, predictable behavior.
- Driver enable pin is now explicitly controlled before applying any motor power.

#### Refactoring and Internal Improvements
- Private internals reorganized into clean categories (config, modes, input sources, global driver state).
- Naming improved and all fields documented with professional detail.
- Removed multiple legacy constructors; replaced by flexible setup through getters and external configuration.
- Clear isolation between:
  - `setPower()` logic
  - Filtering
  - Special drive modes
  - Global enable/disable controller

#### Breaking Changes
- **All previous constructors removed**:
  - Direction-only constructor
  - Direction + enable constructor
  - Configurable digital/PWM constructor
- Users must now:
  1. Retrieve motors via `getLeftMotor()` and `getRightMotor()`.
  2. Assign pins manually.
  3. Call `begin()` before usage.

This change breaks compatibility with any older code that relied on the previous constructor system.

#### Documentation Updates
- Complete rewrite of class documentation.
- Clearly defined initialization workflow.
- Detailed explanation of Exposition Mode, Tank Drive, and state-based behaviors.
- Updated explanation of input source behavior and return requirements.