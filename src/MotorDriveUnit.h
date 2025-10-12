#ifndef MotorDriveUnit_h
#define MotorDriveUnit_h

#include <Arduino.h>
#include "Motor.h"

/**
 * @class MotorDriveUnit
 * @brief High-level controller for a dual-motor system using two Motor instances.
 *
 * This class manages two DC motors (left and right) and allows synchronized control
 * using abstracted power and direction values. It builds on top of the `Motor` class,
 * which handles low-level pin operations, PWM, and enable logic.
 *
 * Optionally, the user can attach external input functions (via function pointers)
 * to dynamically provide power and direction values (e.g., from a joystick or sensor).
 *
 * @note You must call `begin()` once before using the motors.
 */
class MotorDriveUnit
{
private:
  Motor _left_motor;  ///< Internal motor instance controlling the left side.
  Motor _right_motor; ///< Internal motor instance controlling the right side.

  bool _initialized = false; ///< True once both motors are initialized.
  uint8_t _deadzone = 0;     ///< Dead zone threshold applied to all power and direction inputs.

  /**
   * @brief Function pointer returning current global power (-255 to +255).
   * Used by `update()` to control both motors simultaneously.
   */
  int16_t (*_get_power_source)() = nullptr;

  /**
   * @brief Function pointer returning current direction offset (-255 to +255).
   * Used by `update()` to apply differential steering between left and right motors.
   */
  int16_t (*_get_direction_source)() = nullptr;

  /**
   * @brief Applies deadzone filtering to an input value.
   *
   * Any absolute value below `_deadzone` is clamped to zero.
   *
   * @param value Input value to process.
   * @param deadzone Deadzone range threshold.
   * @return Filtered value, or zero if within the deadzone.
   */
  int16_t _apply_deadzone(int16_t value, uint8_t deadzone);

  void _process_power_and_direction(int16_t (&powers)[2]);

public:
  /**
   * @brief Constructs a MotorDriveUnit using only direction pins.
   *
   * Both motors are initialized in digital direction mode (no enable pin used).
   *
   * @param forward_left_pin Forward pin for the left motor.
   * @param backward_left_pin Backward pin for the left motor.
   * @param forward_right_pin Forward pin for the right motor.
   * @param backward_right_pin Backward pin for the right motor.
   */
  MotorDriveUnit(uint8_t forward_left_pin, uint8_t backward_left_pin,
                 uint8_t forward_right_pin, uint8_t backward_right_pin);

  /**
   * @brief Constructs a MotorDriveUnit using direction and enable pins.
   *
   * Both motors are initialized in digital mode for direction and enable control.
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
   * @brief Constructs a MotorDriveUnit using direction, enable, and PWM mode.
   *
   * Both motors are initialized for full PWM-based speed and enable control.
   *
   * @param forward_left_pin Forward pin for the left motor.
   * @param backward_left_pin Backward pin for the left motor.
   * @param forward_right_pin Forward pin for the right motor.
   * @param backward_right_pin Backward pin for the right motor.
   * @param enable_left_pin Enable pin for the left motor.
   * @param enable_right_pin Enable pin for the right motor.
   * @param digital_direction If true, direction pins act as digital; if false, PWM.
   * @param digital_enable If true, enable pins act as digital; if false, PWM.
   */
  MotorDriveUnit(bool digital_direction, bool digital_enable,
                 uint8_t forward_left_pin, uint8_t backward_left_pin,
                 uint8_t forward_right_pin, uint8_t backward_right_pin,
                 uint8_t enable_left_pin, uint8_t enable_right_pin);

  /**
   * @brief Initializes both motor instances.
   *
   * This function calls `begin()` on both motors. If already initialized,
   * subsequent calls are ignored.
   */
  void begin();

  Motor getLeftMotor()
  {
    return _left_motor;
  }

  Motor getRightMotor()
  {
    return _right_motor;
  }

  void applyDifferentialDrive();

  void applyTankDrive();

  void stop()
  {
    _left_motor.stop();
    _right_motor.stop();
  }

  /**
   * @brief Sets the deadzone threshold for all motor input processing.
   *
   * Any input value with an absolute magnitude below this threshold
   * will be treated as zero during `update()` or manual control.
   *
   * @param deadzone Minimum effective input magnitude (0–255).
   */
  inline void MotorDriveUnit::setDeadzone(uint8_t deadzone)
  {
    _deadzone = deadzone;
  }

  /**
   * @brief Gets the deadzone threshold for all motor input processing.
   */
  inline uint8_t getDeadzone(void)
  {
    return _deadzone;
  }

  /**
   * @brief Assigns a function source for power input.
   *
   * The function must return an `int16_t` value between -255 and +255.
   *
   * @param source Function pointer providing current power.
   */
  void setPowerSource(int16_t (*source)())
  {
    if (source == nullptr)
    {
      // Disable power source if null is passed
      _get_power_source = nullptr;
      return;
    }

    _get_power_source = source;
  }

  /**
   * @brief Assigns a function source for direction input.
   *
   * The function must return an `int16_t` value between -255 and +255.
   *
   * @param source Function pointer providing current direction.
   */
  void setDirectionSource(int16_t (*source)())
  {
    if (source == nullptr)
    {
      // Disable power source if null is passed
      _get_direction_source = nullptr;
      return;
    }

    _get_direction_source = source;
  }
};

#endif
