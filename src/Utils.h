#pragma once

#include <Arduino.h>
#include "types.h"

namespace utils
{
  /**
   * @brief Constrains an int16_t value within a specified range.
   *
   * @param value The input value to constrain.
   * @param min_value The lower bound that `value` is allowed to represent.
   * @param max_value The upper bound that `value` is allowed to represent.
   * @return The constrained value within `[min_value, max_value]`.
   *
   * @example
   * int16_t safe = utils::clamp(300, -255, 255); // returns 255
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
   * @brief Constrains an int16_t value to the valid SignedPWM range [-255, 255].
   *
   * Convenience wrapper around `clamp` that enforces the SignedPWM domain,
   * casting the result to SignedPWM.
   *
   * @param value The raw int16_t value to constrain.
   * @return The value clamped to [-255, 255] as a SignedPWM.
   *
   * @example
   * SignedPWM pwm = utils::signed_pwm_clamp(300); // returns 255
   */
  inline SignedPWM signed_pwm_clamp(int16_t value)
  {
    return static_cast<SignedPWM>(clamp(value, -255, 255));
  }

  /**
   * @brief Constrains an int16_t value to the valid UnsignedPWM range [0, 255].
   *
   * Convenience wrapper around `clamp` that enforces the UnsignedPWM domain,
   * casting the result to UnsignedPWM.
   *
   * @param value The raw int16_t value to constrain.
   * @return The value clamped to [0, 255] as an UnsignedPWM.
   *
   * @example
   * UnsignedPWM pwm = utils::unsigned_pwm_clamp(300); // returns 255
   */
  inline UnsignedPWM unsigned_pwm_clamp(int16_t value)
  {
    return static_cast<UnsignedPWM>(clamp(value, 0, 255));
  }
}