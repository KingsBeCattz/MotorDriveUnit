#ifndef Motor_h
#define Motor_h

#include <Arduino.h>
#include "Utils.h"

/**
 * @class Motor
 * @brief Provides a flexible motor control interface supporting 2-pin and 3-pin H-bridge configurations.
 *
 * This class supports both PWM-based and digital-direction motor control. It allows:
 * - Two-pin mode: (direction/PWM) OR (direction-forward + direction-backward)
 * - Three-pin mode: dedicated enable pin (digital or PWM), plus forward/backward pins
 *
 * It fully supports situations where PWM resources are limited by allowing:
 * - PWM on direction pins and digital enable
 * - Digital direction pins and PWM on enable
 *
 * A dead zone can be configured for digital-mode pins to avoid false HIGH states
 * when small PWM values are applied.
 */
class Motor
{
private:
  /** Indicates whether forward/backward pins operate in digital mode. */
  bool _digital_direction = false;

  /** Indicates whether the enable pin (if present) operates in digital mode. */
  bool _digital_enable = false;

  /** Pin used to drive forward rotation direction or PWM. */
  uint8_t _forward_pin = PIN_UNUSED;

  /** Pin used to drive backward rotation direction or PWM. */
  uint8_t _backward_pin = PIN_UNUSED;

  /** Optional enable pin (digital or PWM depending on configuration). */
  uint8_t _enable_pin = PIN_UNUSED;

  /** Minimum PWM value required to treat a digital pin as HIGH. */
  uint8_t _digital_pin_dead_zone = 0;

  /** Last PWM value applied to the motor (0–255). */
  uint8_t _pwm_value = 0;

  /** Indicates current rotation direction (true = forward, false = backward). */
  bool _forward = true;

  /** Tracks whether the motor has already been initialized via begin(). */
  bool _initialized = false;

  /**
   * @brief Writes a value to a pin handling PWM or digital logic.
   *
   * @param pin Pin number. If PIN_UNUSED, no action is taken.
   * @param value PWM value (0–255).
   * @param digital Whether the pin behaves as a digital pin.
   * @param apply_deadzone Whether to apply the dead zone check.
   */
  void _write_pin(uint8_t pin, uint8_t value, bool digital, bool apply_deadzone = false)
  {
    if (pin == PIN_UNUSED)
      return;

    if (digital)
    {
      bool state = (value > 0);
      if (apply_deadzone)
        state = (value >= _digital_pin_dead_zone);

      digitalWrite(pin, state);
    }
    else
    {
      analogWrite(pin, value);
    }
  }

  /**
   * @brief Writes to the enable pin, respecting digital/PWM mode.
   * @param power PWM/digital value.
   */
  inline void _set_enable(uint8_t power)
  {
    _write_pin(_enable_pin, power, _digital_enable, false);
  }

  /**
   * @brief Internal method to apply power distribution to forward/backward pins.
   *
   * Selects the highest of forward/backward values as the effective power for
   * the enable pin (if used). This supports H-bridges where enable controls
   * overall magnitude and direction pins handle sign.
   *
   * @param forward_power PWM value for forward pin.
   * @param backward_power PWM value for backward pin.
   */
  void _apply_power(uint8_t forward_power, uint8_t backward_power)
  {
    _write_pin(_forward_pin, forward_power, _digital_direction);
    _write_pin(_backward_pin, backward_power, _digital_direction);

    uint8_t enable_power = (forward_power > backward_power) ? forward_power : backward_power;

    if (_enable_pin != PIN_UNUSED)
      _set_enable(enable_power);

    _pwm_value = enable_power;
    _forward = (forward_power >= backward_power);
  }

public:
  /** Special marker indicating that a pin is not assigned. */
  static const uint8_t PIN_UNUSED = 255;

  /**
   * @brief Initializes the pins and internal state. Must be called before use.
   */
  void begin()
  {
    if (_initialized)
      return;

    _initialized = true;

    if (_forward_pin != PIN_UNUSED)
      pinMode(_forward_pin, OUTPUT);
    if (_backward_pin != PIN_UNUSED)
      pinMode(_backward_pin, OUTPUT);

    if (_enable_pin != PIN_UNUSED)
    {
      pinMode(_enable_pin, OUTPUT);
      _set_enable(0);
    }

    stop();
  }

  // -------------------------------
  // Motor configuration methods
  // -------------------------------

  /**
   * @brief Assigns forward/backward pins and configures their mode.
   * @param forward_pin Pin controlling forward motion.
   * @param backward_pin Pin controlling backward motion.
   * @param digital_direction Whether these pins operate as digital rather than PWM.
   */
  void setDirectionPins(uint8_t forward_pin, uint8_t backward_pin, bool digital_direction = false)
  {
    _forward_pin = forward_pin;
    _backward_pin = backward_pin;
    _digital_direction = digital_direction;

    if (_initialized)
    {
      if (_forward_pin != PIN_UNUSED)
        pinMode(_forward_pin, OUTPUT);
      if (_backward_pin != PIN_UNUSED)
        pinMode(_backward_pin, OUTPUT);

      stop();
    }
  }

  /**
   * @brief Sets the enable pin and whether it operates in digital mode.
   * @param enable_pin Pin controlling overall power.
   * @param digital_enable Whether the enable pin is digital.
   */
  void setEnablePin(uint8_t enable_pin, bool digital_enable = false)
  {
    _enable_pin = enable_pin;
    _digital_enable = digital_enable;

    if (_initialized && _enable_pin != PIN_UNUSED)
    {
      pinMode(_enable_pin, OUTPUT);
      _set_enable(0);
    }
  }

  // -------------------------------
  // Motor control methods
  // -------------------------------

  /**
   * @brief Stops the motor immediately.
   */
  inline void stop()
  {
    _apply_power(0, 0);
  }

  /**
   * @brief Sets power and direction using a signed value.
   *
   * @param power Range -255 (full backward) to +255 (full forward).
   */
  void setPower(int16_t power)
  {
    if (power == 0)
    {
      stop();
      return;
    }

    power = utils::clamp(power, -255, 255);

    bool is_forward = (power >= 0);
    uint8_t clamped_power = static_cast<uint8_t>(abs(power));

    if (is_forward == _forward && clamped_power == _pwm_value)
      return;

    if (is_forward != _forward)
    {
      stop();
      delayMicroseconds(300);
    }

    if (is_forward)
      _apply_power(clamped_power, 0);
    else
      _apply_power(0, clamped_power);
  }

  /**
   * @brief Forces forward rotation at a given PWM value.
   * @param power 0–255
   */
  void forward(uint8_t power)
  {
    power = static_cast<uint8_t>(utils::clamp(power, 0, 255));
    _apply_power(power, 0);
  }

  /**
   * @brief Forces backward rotation at a given PWM value.
   * @param power 0–255
   */
  void backward(uint8_t power)
  {
    power = static_cast<uint8_t>(utils::clamp(power, 0, 255));
    _apply_power(0, power);
  }

  /**
   * @brief Sets the dead zone threshold for pins operating in digital mode.
   * @param deadzone Minimum PWM value considered HIGH.
   */
  void setDigitalPinDeadZone(uint8_t deadzone)
  {
    _digital_pin_dead_zone = deadzone;
  }

  /**
   * @brief Retrieves current dead zone threshold.
   */
  uint8_t getDigitalPinDeadZone(void)
  {
    return _digital_pin_dead_zone;
  }
};

#endif
