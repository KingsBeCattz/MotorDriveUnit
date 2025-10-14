#ifndef MotorDriveUnit_h
#define MotorDriveUnit_h

#include <Arduino.h>
#include "Motor.h"

/**
 * @class MotorDriveUnit
 * @brief High-level dual-motor controller supporting multiple drive modes and dynamic input sources.
 *
 * The `MotorDriveUnit` class orchestrates two `Motor` instances (left and right)
 * to implement flexible control patterns such as **Differential Drive**, **Tank Drive**,
 * and an optional **Exposition Mode** for demonstration purposes.
 *
 * It allows runtime assignment of function pointers as dynamic sources for
 * power and direction input, typically used to interface with gamepads, joysticks,
 * or sensors. The `update()` method should be called repeatedly inside the main loop
 * to continuously process inputs and update motor states accordingly.
 *
 * ### Drive Modes:
 * - **Differential Drive:** turns are achieved by proportionally reducing power on one motor.
 * - **Tank Drive:** turns are achieved by inverting the direction of one motor.
 * - **Exposition Mode:** allows continuous spinning for demonstrations when a button is pressed or toggled.
 *
 * @note Call `begin()` once before any motion commands to properly initialize both motors.
 */
class MotorDriveUnit
{
private:
  Motor _left_motor;  ///< Internal motor controlling the left side.
  Motor _right_motor; ///< Internal motor controlling the right side.

  bool _initialized = false; ///< True once both motors have been initialized via `begin()`.
  uint8_t _deadzone = 0;     ///< Deadzone threshold applied to all analog input sources.

  bool _exposition_mode = false;       ///< Indicates whether exposition mode is persistently active.
  bool _exposition_active_now = false; ///< True while the exposition button is currently held.
  bool _tank_drive_mode = false;       ///< When true, Tank Drive mode replaces Differential Drive logic.
  int16_t _exposition_hold_power = 0;  ///< Stores the last valid power value during exposition press.

  /**
   * @brief Callback returning the current power input (-255 to +255).
   * Used by `update()` to define the base motor power level.
   */
  int16_t (*_get_power_source)() = nullptr;

  /**
   * @brief Callback returning the current directional input (-255 to +255).
   * Used by `update()` to determine turning bias between the two motors.
   */
  int16_t (*_get_direction_source)() = nullptr;

  /**
   * @brief Applies a deadzone filter to the given value.
   *
   * Values whose absolute magnitude fall within the deadzone range
   * are clamped to zero. Remaining values are remapped linearly
   * to retain full output resolution above the threshold.
   *
   * @param value Raw input value to process.
   * @param deadzone Deadzone threshold (0–255).
   * @return Processed value within [-255, 255].
   */
  int16_t _apply_deadzone(int16_t value, uint8_t deadzone);

  /**
   * @brief Applies the current instance deadzone configuration to a given source value.
   * @param source_result The raw result from the assigned power or direction source.
   * @return Deadzone-filtered value.
   */
  int16_t _apply_deadzone_to_source(int16_t source_result);

public:
  /**
   * @brief Constructs a MotorDriveUnit using only directional pins (no enable pins).
   *
   * Each motor operates in digital direction mode with no enable control.
   *
   * @param forward_left_pin Forward pin for the left motor.
   * @param backward_left_pin Backward pin for the left motor.
   * @param forward_right_pin Forward pin for the right motor.
   * @param backward_right_pin Backward pin for the right motor.
   */
  MotorDriveUnit(uint8_t forward_left_pin, uint8_t backward_left_pin,
                 uint8_t forward_right_pin, uint8_t backward_right_pin);

  /**
   * @brief Constructs a MotorDriveUnit using directional and enable pins.
   *
   * This variant supports enable control for both motors and assumes
   * digital operation for both direction and enable signals.
   *
   * @param forward_left_pin Forward pin for the left motor.
   * @param backward_left_pin Backward pin for the left motor.
   * @param forward_right_pin Forward pin for the right motor.
   * @param backward_right_pin Backward pin for the right motor.
   * @param enable_left_pin Enable pin for the left motor.
   * @param enable_right_pin Enable pin for the right motor.
   */
  MotorDriveUnit(uint8_t forward_left_pin, uint8_t backward_left_pin,
                 uint8_t forward_right_pin, uint8_t backward_right_pin,
                 uint8_t enable_left_pin, uint8_t enable_right_pin);

  /**
   * @brief Constructs a MotorDriveUnit with full control over direction and enable signal modes.
   *
   * Allows selecting whether direction and enable pins act as digital or PWM outputs.
   *
   * @param digital_direction If true, direction pins behave digitally; if false, they use PWM.
   * @param digital_enable If true, enable pins behave digitally; if false, they use PWM.
   * @param forward_left_pin Forward pin for the left motor.
   * @param backward_left_pin Backward pin for the left motor.
   * @param forward_right_pin Forward pin for the right motor.
   * @param backward_right_pin Backward pin for the right motor.
   * @param enable_left_pin Enable pin for the left motor.
   * @param enable_right_pin Enable pin for the right motor.
   */
  MotorDriveUnit(bool digital_direction, bool digital_enable,
                 uint8_t forward_left_pin, uint8_t backward_left_pin,
                 uint8_t forward_right_pin, uint8_t backward_right_pin,
                 uint8_t enable_left_pin, uint8_t enable_right_pin);

  /**
   * @brief Initializes both motor instances.
   *
   * This method must be called once before any drive operations.
   * It internally calls `begin()` on both underlying `Motor` objects.
   * Subsequent calls are safely ignored if initialization was already done.
   */
  void begin();

  /**
   * @brief Retrieves the left motor instance.
   * @return A copy of the internal left `Motor` object.
   */
  Motor getLeftMotor()
  {
    return _left_motor;
  }

  /**
   * @brief Retrieves the right motor instance.
   * @return A copy of the internal right `Motor` object.
   */
  Motor getRightMotor()
  {
    return _right_motor;
  }

  /**
   * @brief Immediately stops both motors by setting output power to zero.
   */
  void stop()
  {
    _left_motor.stop();
    _right_motor.stop();
  }

  /**
   * @brief Activates or releases exposition mode based on button state.
   *
   * When active (button pressed), the motors continuously spin opposite directions
   * to perform an in-place rotation. When released, if the last held power was nonzero,
   * the unit remains spinning persistently until stopped.
   *
   * @param state True while the exposition button is pressed; false when released.
   */
  void setExpositionActive(bool state);

  /**
   * @brief Toggles between Differential Drive and Tank Drive behavior.
   *
   * - In **Differential Drive**, one motor’s speed is reduced proportionally for turns.
   * - In **Tank Drive**, one motor reverses direction for sharper rotation.
   */
  void toggleTankDriveMode();

  /**
   * @brief Main update routine controlling the entire drive behavior.
   *
   * This method reads input values from the assigned power and direction sources,
   * applies deadzone correction, and drives both motors according to the current mode:
   * - Normal (Differential) Drive
   * - Tank Drive
   * - Exposition Mode (live or persistent)
   *
   * Should be invoked regularly inside the main control loop.
   */
  void update();

  /**
   * @brief Sets the global deadzone threshold.
   *
   * Any absolute input value below this threshold is treated as zero to eliminate
   * input noise or joystick drift.
   *
   * @param deadzone Deadzone magnitude (0–255).
   */
  inline void setDeadzone(uint8_t deadzone)
  {
    _deadzone = deadzone;
  }

  /**
   * @brief Returns the current deadzone threshold.
   * @return Current deadzone magnitude.
   */
  inline uint8_t getDeadzone() const
  {
    return _deadzone;
  }

  /**
   * @brief Assigns a dynamic function source for motor power input.
   *
   * The provided function must return an `int16_t` in the range [-255, 255].
   * Passing `nullptr` disables the power source.
   *
   * @param source Function pointer providing the current power level.
   */
  inline void setPowerSource(int16_t (*source)())
  {
    _get_power_source = source;
  }

  /**
   * @brief Assigns a dynamic function source for directional control input.
   *
   * The provided function must return an `int16_t` in the range [-255, 255].
   * Passing `nullptr` disables the direction source.
   *
   * @param source Function pointer providing the current direction offset.
   */
  inline void setDirectionSource(int16_t (*source)())
  {
    _get_direction_source = source;
  }
};

#endif
