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