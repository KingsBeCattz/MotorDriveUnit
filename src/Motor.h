#ifndef Motor_h
#define Motor_h

#include <Arduino.h>

/**
 * @class Motor
 * @brief Provides high-level control for a single DC motor using configurable
 *        forward, backward, and optional enable pins.
 *
 * This class supports both digital and PWM-based control modes, allowing use
 * with H-bridges, motor drivers, or direct GPIO control. The user can specify
 * whether each control line (direction and enable) is digital or PWM-capable.
 *
 * Typical usage involves calling `begin()` once, then using `setPower()`,
 * `forward()`, `backward()`, or `stop()` to control motor behavior.
 */
class Motor
{
private:
  bool _digital_direction; ///< Defines if the direction pins are digital (true) or PWM (false).
  bool _digital_enable;    ///< Defines if the enable pin is digital (true) or PWM (false).
  uint8_t _forward_pin;    ///< Pin controlling the forward direction signal.
  uint8_t _backward_pin;   ///< Pin controlling the backward direction signal.
  uint8_t _enable_pin;     ///< Optional enable pin (255 if unused). Used to control motor activation or speed.

  uint8_t _digital_pin_dead_zone = 0; ///< Minimum threshold (0–255) to activate output when using digital mode.

  uint8_t _pwm_value = 0;    ///< Currently applied power value (0–255).
  bool _forward = true;      ///< Indicates last direction (true = forward, false = backward).
  bool _initialized = false; ///< True once `begin()` has been called successfully.

  /**
   * @brief Writes a value to a pin depending on its control mode.
   *
   * If the pin is configured as digital, the value is interpreted as a boolean
   * state (HIGH or LOW). When PWM mode is active, the full analog range (0–255)
   * is written using `analogWrite()`. If `apply_deadzone` is true, a minimum
   * threshold (`_digital_pin_dead_zone`) is used to prevent low activation noise.
   *
   * @param pin Target pin to write.
   * @param value Output value (0–255).
   * @param digital True if the pin is digital, false if it is PWM.
   * @param apply_deadzone True to apply dead zone filtering when using digital output.
   */
  void _write_pin(uint8_t pin, uint8_t value, bool digital, bool apply_deadzone = false);

  /**
   * @brief Updates the enable pin according to the requested power level.
   *
   * This method is automatically invoked by `_apply_power()` or by any function
   * that modifies motor output. It is only executed if the enable pin is defined
   * (not equal to `PIN_UNUSED`).
   *
   * @param power Power value (0–255) to send to the enable pin.
   */
  inline void _set_enable(uint8_t power)
  {
    _write_pin(_enable_pin, power, _digital_enable, false);
  }

  /**
   * @brief Applies the desired power and direction levels to the motor.
   *
   * This is the core internal function responsible for actually updating the
   * hardware pins. It sets the forward and backward outputs according to their
   * power values and updates the enable pin if present. It also stores the
   * current `_pwm_value` and `_forward` state internally.
   *
   * - If `forward_power > backward_power`, the motor spins forward.
   * - If `backward_power > forward_power`, the motor spins backward.
   * - If both are zero, the motor is stopped.
   *
   * @param forward_power PWM/digital value for the forward direction (0–255).
   * @param backward_power PWM/digital value for the backward direction (0–255).
   */
  void _apply_power(uint8_t forward_power, uint8_t backward_power);

public:
  /**
   * @brief Constant representing an unused pin.
   *
   * If a pin parameter is set to `PIN_UNUSED`, it is ignored by the motor logic.
   */
  static const uint8_t PIN_UNUSED = 255;

  /**
   * @brief Constructs a new Motor instance.
   *
   * @param digital_direction If true, both direction pins are digital (HIGH/LOW).
   *                          If false, PWM output (0–255) is used for both.
   * @param digital_enable If true, the enable pin is digital; if false, it is PWM.
   * @param forward_pin Pin used to control forward movement.
   * @param backward_pin Pin used to control backward movement.
   * @param enable_pin Optional enable pin (255 if unused). Used in bridges that require
   *                   a separate activation or power modulation line.
   */
  Motor(bool digital_direction, bool digital_enable, uint8_t forward_pin, uint8_t backward_pin, uint8_t enable_pin = PIN_UNUSED);

  /**
   * @brief Initializes the motor's pins and sets them to safe default states.
   *
   * This function must be called once before operating the motor.
   * It sets all control pins as outputs, initializes the enable pin if present,
   * and ensures the motor starts in a stopped state.
   *
   * If `begin()` is called more than once, subsequent calls are ignored to
   * prevent redundant pin reconfiguration.
   */
  void begin();

  /**
   * @brief Immediately stops the motor.
   *
   * All control pins are set to zero, including the enable pin if defined.
   * The internal state (`_pwm_value` and `_forward`) is updated accordingly.
   */
  inline void stop()
  {
    _apply_power(0, 0);
  }

  /**
   * @brief Sets the motor’s power level and direction.
   *
   * - Positive values move the motor forward.
   * - Negative values move it backward.
   * - Zero stops the motor.
   *
   * This method automatically determines direction and applies the appropriate
   * pin outputs through `_apply_power()`. No redundant operations occur if the
   * power or direction hasn’t changed since the last call.
   *
   * @param power Motor power level: -255 (full backward) to +255 (full forward).
   */
  void setPower(int16_t power);

  /**
   * @brief Moves the motor forward at a specified power.
   *
   * This is a direct call to `_apply_power()` with the backward channel set to zero.
   * It also updates internal direction and power tracking.
   *
   * @param power Power level (0–255).
   */
  void forward(uint8_t power);

  /**
   * @brief Moves the motor backward at a specified power.
   *
   * This is a direct call to `_apply_power()` with the forward channel set to zero.
   * It also updates internal direction and power tracking.
   *
   * @param power Power level (0–255).
   */
  void backward(uint8_t power);

  void setDigitalPinDeadZone(uint8_t deadzone)
  {
    _digital_pin_dead_zone = deadzone;
  }

  uint8_t getDigitalPinDeadZone(void)
  {
    return _digital_pin_dead_zone;
  }
};

#endif
