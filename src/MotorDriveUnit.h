#ifndef MotorDriveUnit_h
#define MotorDriveUnit_h

#include <Arduino.h>
#include "Motor.h"

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
 *        - `setPowerSource(int16_t (*)())`
 *        - `setDirectionSource(int16_t (*)())`
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

  /** @brief Output pin used to enable/disable the entire motor driver. */
  uint8_t _driver_enable_pin = Motor::PIN_UNUSED;

  /** @brief Whether driver enable pin should be controlled digitally (digitalWrite). */
  bool _digital_driver_enable = true;

  // ────────────────────────────────────────────────
  // Internal State Variables
  // ────────────────────────────────────────────────

  /** @brief Marks whether the system was initialized via begin(). */
  bool _initialized = false;

  /** @brief Deadzone threshold applied to both power and direction inputs. */
  uint8_t _deadzone = 0;

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
  int16_t _exposition_hold_power = 0;

  /**
   * @brief Function pointer providing current power input [-255..255].
   */
  int16_t (*_get_power_source)() = nullptr;

  /**
   * @brief Function pointer providing current direction input [-255..255].
   */
  int16_t (*_get_direction_source)() = nullptr;

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
  int16_t _apply_deadzone(int16_t value, uint8_t deadzone)
  {
    value = utils::clamp(value, -255, 255);
    if (abs(value) < deadzone)
      return 0;

    bool positive = (value >= 0);
    int16_t result = map(abs(value) - deadzone, 0, 255 - deadzone, 0, 255);
    return positive ? result : -result;
  }

  /**
   * @brief Convenience wrapper applying deadzone to a source value.
   */
  int16_t _apply_deadzone_to_source(int16_t source_result)
  {
    return _apply_deadzone(utils::clamp(source_result, -255, 255), _deadzone);
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
   * @brief Configures a global driver-enable pin controlling the motor driver hardware.
   *
   * @param enable_pin Digital or analog-controlled output pin.
   * @param digital_enable Whether this pin is managed using digitalWrite().
   */
  void setDriverEnablePin(uint8_t enable_pin, bool digital_enable = true)
  {
    _driver_enable_pin = enable_pin;
    _digital_driver_enable = digital_enable;

    pinMode(_driver_enable_pin, OUTPUT);
    digitalWrite(_driver_enable_pin, _driver_enabled ? HIGH : LOW);
  }

  // ────────────────────────────────────────────────
  // Accessors
  // ────────────────────────────────────────────────

  /**
   * @brief Get a reference to the left motor.
   *
   * Returns a non-const reference to the Motor instance representing the left motor
   * controlled by this MotorDriveUnit. The caller may use the returned reference to
   * query state or invoke control methods that modify the motor.
   *
   * Note: The reference is only valid while the owning MotorDriveUnit object remains
   * alive — do not store this reference beyond the lifetime of the MotorDriveUnit.
   *
   * @return Motor& Reference to the left motor.
   */
  inline Motor &getLeftMotor()
  {
    return _left_motor;
  }

  /**
   * @brief Get a reference to the right motor.
   *
   * Returns a non-const reference to the Motor instance representing the right motor
   * controlled by this MotorDriveUnit. The caller may use the returned reference to
   * query state or invoke control methods that modify the motor.
   *
   * Note: The reference is only valid while the owning MotorDriveUnit object remains
   * alive — do not store this reference beyond the lifetime of the MotorDriveUnit.
   *
   * @return Motor& Reference to the right motor.
   */
  inline Motor &getRightMotor()
  {
    return _right_motor;
  }

  /** @return Current deadzone threshold. */
  inline uint8_t getDeadzone() const
  {
    return _deadzone;
  }

  /** @return Pointer to the currently assigned power source function. */
  int16_t (*getPowerSourceFunction())()
  {
    return _get_power_source;
  }

  /** @return Pointer to the currently assigned direction source function. */
  int16_t (*getDirectionSourceFunction())()
  {
    return _get_direction_source;
  }

  // ────────────────────────────────────────────────
  // Mutators
  // ────────────────────────────────────────────────

  /**
   * @brief Sets the deadzone value used for input filtering.
   *
   * @param deadzone Value in range [0..255].
   */
  inline void setDeadzone(uint8_t deadzone)
  {
    _deadzone = deadzone;
  }

  /**
   * @brief Assigns the power source callback.
   */
  inline void setPowerSource(int16_t (*source)())
  {
    _get_power_source = source;
  }

  /**
   * @brief Assigns the direction source callback.
   */
  inline void setDirectionSource(int16_t (*source)())
  {
    _get_direction_source = source;
  }

  // ────────────────────────────────────────────────
  // Dynamic Control Methods
  // ────────────────────────────────────────────────

  /**
   * @brief Toggles the global driver-enable state.
   *
   * @details
   * If a driver-enable pin is configured, it will be updated accordingly.
   */
  void toggleDriverEnabled()
  {
    _driver_enabled = !_driver_enabled;
    if (_driver_enable_pin != Motor::PIN_UNUSED)
    {
      digitalWrite(_driver_enable_pin, _driver_enabled ? HIGH : LOW);
    }
  }

  /**
   * @brief Sets the global driver-enable state.
   *
   * @details
   * If a driver-enable pin is configured, it will be updated accordingly.
   */
  void setDriverEnabled(bool state)
  {
    _driver_enabled = state;
    if (_driver_enable_pin != Motor::PIN_UNUSED)
    {
      digitalWrite(_driver_enable_pin, _driver_enabled ? HIGH : LOW);
    }
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
   */
  void setExpositionActive(bool state)
  {
    if (_exposition_active_now && !state)
    {
      _exposition_mode = (_exposition_hold_power != 0);
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
   * @param left_power  0–255 magnitude, direction controlled by left_forward.
   * @param left_forward Motor direction.
   * @param right_power 0–255 magnitude, direction controlled by right_forward.
   * @param right_forward Motor direction.
   */
  void useManualDrive(uint8_t left_power, bool left_forward,
                      uint8_t right_power, bool right_forward)
  {
    _left_motor.setPower(left_power * (left_forward ? 1 : -1));
    _right_motor.setPower(right_power * (right_forward ? 1 : -1));
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

    int16_t power = _apply_deadzone_to_source(_get_power_source());

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
        _left_motor.setPower(power);
        _right_motor.setPower(-power);
        if (!_driver_enabled)
        {
          setDriverEnabled(true);
        }
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
        _left_motor.setPower(_exposition_hold_power);
        _right_motor.setPower(-_exposition_hold_power);
        if (!_driver_enabled)
        {
          setDriverEnabled(true);
        }
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
    else if (!_driver_enabled)
    {
      setDriverEnabled(true);
    }

    float direction = (float)(_apply_deadzone_to_source(_get_direction_source())) / 255.0f;
    int16_t powers[2] = {power, power};

    bool tank_mode = _use_tank_drive || _tank_drive_mode;

    if (direction > 0.0f)
    {
      if (tank_mode)
        powers[1] = -(int16_t)(powers[1] * direction);
      else
        powers[1] = (int16_t)(powers[1] * (1.0f - direction));
    }
    else if (direction < 0.0f)
    {
      if (tank_mode)
        powers[0] = -(int16_t)(powers[0] * -direction);
      else
        powers[0] = (int16_t)(powers[0] * (1.0f + direction));
    }

    _left_motor.setPower(powers[0]);
    _right_motor.setPower(powers[1]);

    if (_use_tank_drive)
      _use_tank_drive = false;
  }
};

#endif
