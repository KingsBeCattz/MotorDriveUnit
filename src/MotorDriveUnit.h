#pragma once

#include <Arduino.h>
#include "Motor.h"
#include "types.h"

/**
 * @class MotorDriveUnit
 * @brief High-level dual-motor controller designed for skid-steer or differential
 *        drive robots. Provides configurable deadzone handling, dynamic drive modes,
 *        exposition (showcase) motion logic, and pluggable input source functions.
 *
 * @details
 * This class manages two independent Motor instances (left and right), applies
 * deadzone filtering, merges power and direction sources, and computes the final
 * wheel outputs each frame via update().
 *
 * ## TB6612FNG Standby (STBY) Pin Wiring
 * Connect the TB6612FNG STBY pin to a digital output pin and pass it to
 * `setDriverEnablePin(pin)`. The driver wakes (STBY = HIGH) when motors are
 * in use and enters low-power standby (STBY = LOW) when stopped.
 *
 * ## Usage Lifecycle
 * **Setup phase:**
 *   1. Instantiate MotorDriveUnit.
 *   2. Retrieve both Motor instances through getLeftMotor() / getRightMotor().
 *   3. Configure their pins using:
 *        - `Motor::setDirectionPins(forward_pin, backward_pin, digital_direction)`
 *        - `Motor::setEnablePin(enable_pin, digital_enable)`
 *   4. Optionally set a global driver-enable pin via:
 *        - `setDriverEnablePin(pin, digital_enable)`
 *   5. Call `begin()` to initialize both motors.
 *   6. Configure filtering and input sources:
 *        - `setDeadzone(deadzone_value)`
 *        - `setPowerSource(SourceFn)`
 *        - `setDirectionSource(SourceFn)`
 *
 * **Loop phase:**
 *   - Call `update()` to automatically compute outputs based on the source functions.
 *   - Unless special modes are enabled (tank drive / exposition),
 *     the motors will follow the power and direction functions.
 *
 * ## Input Ranges
 * The input source functions must return values within [-255, 255].
 * Out-of-range values are clamped internally.
 *
 * ## Exposition Mode Summary
 * Exposition features allow the robot to spin in place indefinitely for demos.
 *
 * - `_exposition_active_now == true`
 *      → Immediate mirror-drive motion:
 *        left = power, right = -power
 *      → If power == 0, motors stop.
 *      → The absolute value of power determines speed.
 *      → The sign determines direction:
 *          * positive → left forward, right backward
 *          * negative → left backward, right forward
 *      → `_exposition_hold_power` stores the last power.
 *
 * - When exposition_active_now transitions from true → false:
 *      * If `_exposition_hold_power != 0`, sustained exposition mode activates.
 *
 * - `_exposition_mode == true` (sustained):
 *      → Uses `_exposition_hold_power` indefinitely.
 *      → If `_exposition_hold_power == 0`, mode ends and motors stop.
 *
 * ## Tank Drive Summary
 * - Permanent tank mode toggled by `toggleTankDriveMode()`.
 * - One-shot tank mode triggered by `useTankDrive()` and consumed in the next update().
 * - In tank drive, direction input causes one wheel to reverse instead of scaling speeds.
 *
 */
class MotorDriveUnit
{
public:
  /**
   * @brief Pointer to a function that provides a SignedPWM value.
   *
   * Represents a dynamically injectable power or direction source for
   * the motor driver. The pointed-to function takes no parameters and
   * returns a SignedPWM value in [-255, 255].
   *
   * This indirection allows the motor driver's input behavior to be
   * swapped at runtime without modifying its internal logic.
   *
   * @return SignedPWM — a value in [-255, 255]
   */
  typedef SignedPWM (*SourceFn)();

private:
  // ────────────────────────────────────────────────
  // Motor Configuration
  // ────────────────────────────────────────────────

  /** @brief Left motor instance controlled by this unit. */
  Motor _left_motor;

  /** @brief Right motor instance controlled by this unit. */
  Motor _right_motor;

  /** @brief Global driver-enable state. HIGH means enabled if digital. */
  bool _driver_enabled = true;

  /** @brief Output pin used to enable/disable the entire motor driver (e.g. TB6612FNG STBY). */
  Pin _driver_enable_pin = Motor::PIN_UNUSED;

  /**
   * @brief Whether driver enable pin should be controlled digitally (digitalWrite).
   * @details
   * When true, the pin is driven HIGH (enabled) or LOW (standby) via digitalWrite().
   * When false, analogWrite() is used: 255 = enabled, 0 = standby.
   */
  bool _digital_driver_enable = true;

  // ────────────────────────────────────────────────
  // Internal State Variables
  // ────────────────────────────────────────────────

  /** @brief Marks whether the system was initialized via begin(). */
  bool _initialized = false;

  /** @brief Deadzone threshold applied to both power and direction inputs. */
  UnsignedPWM _deadzone = 0;

  /** @brief Permanent exposition sustain mode flag. */
  bool _exposition_mode = false;

  /** @brief Immediate exposition mode, active only when explicitly enabled. */
  bool _exposition_active_now = false;

  /** @brief Permanent tank-drive mode toggle. */
  bool _tank_drive_mode = false;

  /**
   * @brief One-shot tank drive flag. Consumed on the next update().
   * @details
   * When true, tank drive mode is applied only for the next control cycle,
   * after which it resets to false automatically.
   */
  bool _use_tank_drive = false;

  /**
   * @brief Last power value used during immediate exposition mode.
   * @details
   * If this is non-zero when exposition_active_now is deactivated,
   * exposition_mode becomes permanently engaged until this value returns to zero.
   */
  SignedPWM _exposition_hold_power = 0;

  /**
   * @brief Function pointer providing current power input as a SignedPWM [-255..255].
   */
  SourceFn _get_power_source = nullptr;

  /**
   * @brief Function pointer providing current direction input as a SignedPWM [-255..255].
   */
  SourceFn _get_direction_source = nullptr;

  // ────────────────────────────────────────────────
  // Private Helpers
  // ────────────────────────────────────────────────

  /**
   * @brief Applies clamping and deadzone filtering to a value.
   *
   * @param value Raw input value in range [-255..255].
   * @param deadzone Threshold under which the value becomes zero.
   * @return Filtered and remapped output in range [-255..255].
   */
  int16_t _apply_deadzone(int16_t value, UnsignedPWM deadzone)
  {
    value = utils::signed_pwm_clamp(value);
    if (abs(value) < deadzone)
      return 0;
    bool positive = (value >= 0);
    int16_t result = map(abs(value) - deadzone, 0, 255 - deadzone, 0, 255);
    result = utils::signed_pwm_clamp(result);
    return positive ? result : -result;
  }

  /**
   * @brief Convenience wrapper applying deadzone to a source value.
   */
  SignedPWM _apply_deadzone_to_source(SignedPWM source_result)
  {
    return _apply_deadzone(utils::signed_pwm_clamp(source_result), _deadzone);
  }

  /**
   * @brief Low-level write to the driver-enable / STBY pin.
   *
   * @details
   * Respects `_digital_driver_enable`:
   *   - true  → digitalWrite(HIGH / LOW)
   *   - false → analogWrite(255 / 0)
   *
   * No-ops when `_driver_enable_pin == Motor::PIN_UNUSED`.
   *
   * @param state true = enable driver (STBY HIGH), false = standby (STBY LOW).
   */
  void _write_driver_enable_pin(bool state)
  {
    if (_driver_enable_pin == Motor::PIN_UNUSED)
      return;

    if (_digital_driver_enable)
      digitalWrite(_driver_enable_pin, state ? HIGH : LOW);
    else
      analogWrite(_driver_enable_pin, state ? 255 : 0);
  }

public:
  // ────────────────────────────────────────────────
  // Initialization and Configuration
  // ────────────────────────────────────────────────

  /**
   * @brief Initializes both internal Motor instances.
   *
   * @note Must be called during setup(), after motor pins have been configured.
   */
  void begin()
  {
    _left_motor.begin();
    _right_motor.begin();
    _initialized = true;
  }

  /**
   * @brief Configures the global driver-enable pin (e.g. TB6612FNG STBY).
   *
   * @details
   * The pin is immediately driven to reflect the current `_driver_enabled` state.
   * Use `digital_enable = false` for PWM-controlled enable pins; in that case
   * analogWrite() is used instead of digitalWrite().
   *
   * @param enable_pin  Arduino pin number connected to the driver enable/STBY input.
   * @param digital_enable  true  → manage with digitalWrite() (default, TB6612FNG STBY).
   *                        false → manage with analogWrite().
   */
  void setDriverEnablePin(Pin enable_pin, bool digital_enable = true)
  {
    _driver_enable_pin = enable_pin;
    _digital_driver_enable = digital_enable;

    pinMode(_driver_enable_pin, OUTPUT);
    _write_driver_enable_pin(_driver_enabled);
  }

  // ────────────────────────────────────────────────
  // Accessors
  // ────────────────────────────────────────────────

  /**
   * @brief Returns a non-const reference to the left motor.
   *
   * The reference is only valid while the owning MotorDriveUnit remains alive —
   * do not store it beyond the lifetime of this object.
   *
   * @return Motor& Reference to the left motor.
   */
  inline Motor &getLeftMotor() { return _left_motor; }

  /**
   * @brief Returns a non-const reference to the right motor.
   *
   * The reference is only valid while the owning MotorDriveUnit remains alive —
   * do not store it beyond the lifetime of this object.
   *
   * @return Motor& Reference to the right motor.
   */
  inline Motor &getRightMotor() { return _right_motor; }

  /** @return Current deadzone threshold. */
  inline UnsignedPWM getDeadzone() const
  {
    return _deadzone;
  }

  /** @return The currently assigned power source function, or nullptr if none. */
  SourceFn getPowerSourceFunction() { return _get_power_source; }

  /** @return The currently assigned direction source function, or nullptr if none. */
  SourceFn getDirectionSourceFunction() { return _get_direction_source; }

  // ────────────────────────────────────────────────
  // Mutators
  // ────────────────────────────────────────────────

  /**
   * @brief Sets the deadzone value used for input filtering.
   *
   * @param deadzone Value in range [0..255].
   */
  inline void setDeadzone(UnsignedPWM deadzone)
  {
    _deadzone = deadzone;
  }

  /**
   * @brief Assigns the power source callback.
   *
   * @param source A SourceFn to be called each cycle to obtain the current power value.
   *               Pass nullptr to disable power input.
   */
  inline void setPowerSource(SourceFn source) { _get_power_source = source; }

  /**
   * @brief Assigns the direction source callback.
   *
   * @param source A SourceFn to be called each cycle to obtain the current direction value.
   *               Pass nullptr to disable direction input.
   */
  inline void setDirectionSource(SourceFn source) { _get_direction_source = source; }

  // ────────────────────────────────────────────────
  // Dynamic Control Methods
  // ────────────────────────────────────────────────

  /**
   * @brief Toggles the global driver-enable / STBY state.
   *
   * @details
   * If a driver-enable pin is configured, it will be updated accordingly
   * using the correct write mode (digital or analog).
   */
  void toggleDriverEnabled()
  {
    _driver_enabled = !_driver_enabled;
    _write_driver_enable_pin(_driver_enabled);
  }

  /**
   * @brief Sets the global driver-enable / STBY state.
   *
   * @details
   * If a driver-enable pin is configured, it will be updated accordingly
   * using the correct write mode (digital or analog).
   *
   * @param state true = driver active (STBY HIGH), false = standby (STBY LOW).
   */
  void setDriverEnabled(bool state)
  {
    _driver_enabled = state;
    _write_driver_enable_pin(state);
  }

  /**
   * @brief Immediately stops both motors.
   */
  void stop()
  {
    _left_motor.stop();
    _right_motor.stop();
  }

  /**
   * @brief Enables or disables immediate exposition mode.
   *
   * @details
   * When exposition transitions from active→inactive, and the last hold power
   * was non-zero, exposition sustain mode activates automatically.
   * The driver-enable pin is asserted before the transition so there is no
   * frame in which commands are sent while STBY is still LOW.
   */
  void setExpositionActive(bool state)
  {
    if (_exposition_active_now && !state)
    {
      _exposition_mode = (_exposition_hold_power != 0);

      if (_exposition_mode)
        setDriverEnabled(true);
    }

    _exposition_active_now = state;
  }

  /**
   * @brief Toggles permanent tank drive mode.
   */
  void toggleTankDriveMode()
  {
    _tank_drive_mode = !_tank_drive_mode;
  }

  /**
   * @brief Activates tank drive for the next update() call only.
   */
  void useTankDrive()
  {
    _use_tank_drive = true;
  }

  /**
   * @brief Directly sets power and direction for each motor, bypassing sources.
   *
   * @details
   * The driver-enable / STBY pin is managed automatically:
   *   - If both powers are zero, the driver enters standby.
   *   - Otherwise the driver is woken before the motor commands are issued.
   *
   * @param left_power   0–255 magnitude for the left motor.
   * @param left_forward true = forward, false = backward for the left motor.
   * @param right_power  0–255 magnitude for the right motor.
   * @param right_forward true = forward, false = backward for the right motor.
   */
  void useManualDrive(UnsignedPWM left_power, bool left_forward,
                      UnsignedPWM right_power, bool right_forward)
  {
    bool any_power = (left_power > 0 || right_power > 0);
    if (!any_power)
    {
      stop();
      setDriverEnabled(false);
      return;
    }

    if (!_driver_enabled)
      setDriverEnabled(true);

    _left_motor.setPower(static_cast<int16_t>(left_power) * (left_forward ? 1 : -1));
    _right_motor.setPower(static_cast<int16_t>(right_power) * (right_forward ? 1 : -1));
  }

  // ────────────────────────────────────────────────
  // Main Update Loop
  // ────────────────────────────────────────────────

  /**
   * @brief Computes the final motor outputs for this frame.
   *
   * @details
   * This method:
   *   - Reads and filters power/direction inputs
   *   - Applies exposition mode logic
   *   - Applies tank or normal steering logic
   *   - Sends final output to both motors
   *
   * Must be called every loop() cycle to maintain real-time responsiveness.
   */
  void update()
  {
    if (!_initialized || _get_power_source == nullptr || _get_direction_source == nullptr)
      return;

    SignedPWM power = _apply_deadzone_to_source(_get_power_source());

    // ──────────────── EXHIBITION MODE (IMMEDIATE) ────────────────
    if (_exposition_active_now)
    {
      _exposition_hold_power = power;
      _exposition_mode = false;

      if (power == 0)
      {
        stop();
        setDriverEnabled(false);
      }
      else
      {
        if (!_driver_enabled)
          setDriverEnabled(true);

        _left_motor.setPower(power);
        _right_motor.setPower(-power);
      }
      return;
    }

    // ──────────────── EXHIBITION MODE (SUSTAINED) ────────────────
    if (_exposition_mode)
    {
      if (_exposition_hold_power == 0)
      {
        _exposition_mode = false;
        stop();
        setDriverEnabled(false);
      }
      else
      {
        if (!_driver_enabled)
          setDriverEnabled(true);

        _left_motor.setPower(_exposition_hold_power);
        _right_motor.setPower(-_exposition_hold_power);
      }
      return;
    }

    // ───────────── NORMAL / TANK DRIVE ─────────────
    if (power == 0)
    {
      stop();
      setDriverEnabled(false);
      return;
    }

    if (!_driver_enabled)
      setDriverEnabled(true);

    float direction = (float)(_apply_deadzone_to_source(_get_direction_source())) / 255.0f;
    SignedPWM powers[2] = {power, power};

    bool tank_mode = _use_tank_drive || _tank_drive_mode;

    if (direction > 0.0f)
    {
      if (tank_mode)
        powers[1] = -(SignedPWM)(powers[1] * direction);
      else
        powers[1] = (SignedPWM)(powers[1] * (1.0f - direction));
    }
    else if (direction < 0.0f)
    {
      if (tank_mode)
        powers[0] = -(SignedPWM)(powers[0] * -direction);
      else
        powers[0] = (SignedPWM)(powers[0] * (1.0f + direction));
    }

    _left_motor.setPower(powers[0]);
    _right_motor.setPower(powers[1]);

    if (_use_tank_drive)
      _use_tank_drive = false;
  }
};