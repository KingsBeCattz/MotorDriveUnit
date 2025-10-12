#ifndef Utils_h
#define Utils_h

#include <Arduino.h>

/**
 * @namespace utils
 * @brief Provides numerical helper functions for motor control and input normalization.
 *
 * This namespace includes mathematical tools for constraining and scaling values
 * into ranges suitable for PWM motor control. It is primarily intended to
 * standardize power mapping and prevent invalid or unstable motor inputs.
 */
namespace utils
{
  /**
   * @brief Constrains a value within a specified range.
   *
   * Ensures that the given value does not exceed defined lower or upper limits.
   * Commonly used to keep motor power or direction values within valid boundaries.
   *
   * @param value The input value to constrain.
   * @param min_value The minimum allowable limit.
   * @param max_value The maximum allowable limit.
   * @return The constrained value within `[min_value, max_value]`.
   *
   * @example
   * int16_t safeValue = utils::clamp(300, -255, 255); // returns 255
   */
  inline int16_t clamp(int16_t value, int16_t min_value, int16_t max_value)
  {
    if (value < min_value)
      return min_value;
    if (value > max_value)
      return max_value;
    return value;
  }

  /**
   * @brief Maps an input value to a signed PWM-compatible range (-255 to 255).
   *
   * This function converts an arbitrary input range into a range compatible with PWM-based
   * motor control. The mapping keeps the sign of the input (positive for forward,
   * negative for backward) and includes a dead zone where small inputs are ignored.
   *
   * @param value The input value to be mapped, typically from a joystick or analog input.
   * @param in_min The minimum input value.
   * @param in_max The maximum input value.
   * @return A signed integer between -255 and 255, representing motor power.
   *
   * @note If the absolute value of `value` is smaller than `in_min`, the result will be `0`.
   * This effectively introduces a "dead zone" around the center position to prevent
   * unintended small movements due to noise or minor input variations.
   *
   * @warning The returned range is symmetric: -255 represents full backward,
   * +255 represents full forward, and 0 represents stop.
   *
   * @example
   * int16_t pwm = utils::map_for_pwm(512, -1023, 1023); // ≈ 128
   * int16_t reverse = utils::map_for_pwm(-800, -1023, 1023); // ≈ -200
   */
  inline int16_t map_for_pwm(int16_t value, int16_t in_min, int16_t in_max)
  {
    if (value > -in_min && value < in_min)
      return 0;

    return (value - in_min) * (255 - (-255)) / (in_max - in_min) + (-255);
  }
}

#endif
