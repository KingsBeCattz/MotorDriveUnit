#pragma once

/**
 * @brief Type for representing a pin number.
 */
typedef unsigned char Pin;

/**
 * @brief Signed PWM value in the range [-255, 255].
 *
 * Represents a motor power level where the sign encodes direction
 * (negative = reverse, positive = forward) and the magnitude encodes
 * the duty cycle intensity.
 */
typedef short SignedPWM;

/**
 * @brief Unsigned PWM value in the range [0, 255].
 *
 * Represents a non-directional power level, typically used for
 * enable pins or digital logic where only magnitude matters.
 */
typedef unsigned char UnsignedPWM;