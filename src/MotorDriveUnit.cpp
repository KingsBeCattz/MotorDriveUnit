#include "MotorDriveUnit.h"
#include "Utils.h"

MotorDriveUnit::MotorDriveUnit(uint8_t forward_left_pin, uint8_t backward_left_pin,
                               uint8_t forward_right_pin, uint8_t backward_right_pin)
    : _left_motor(false, false, forward_left_pin, backward_left_pin, Motor::PIN_UNUSED),
      _right_motor(false, false, forward_right_pin, backward_right_pin, Motor::PIN_UNUSED)
{
}

MotorDriveUnit::MotorDriveUnit(uint8_t forward_left_pin, uint8_t backward_left_pin,
                               uint8_t forward_right_pin, uint8_t backward_right_pin,
                               uint8_t enable_left_pin, uint8_t enable_right_pin)
    : _left_motor(false, true, forward_left_pin, backward_left_pin, enable_left_pin),
      _right_motor(false, true, forward_right_pin, backward_right_pin, enable_right_pin)
{
}

MotorDriveUnit::MotorDriveUnit(bool digital_direction, bool digital_enable,
                               uint8_t forward_left_pin, uint8_t backward_left_pin,
                               uint8_t forward_right_pin, uint8_t backward_right_pin,
                               uint8_t enable_left_pin, uint8_t enable_right_pin)
    : _left_motor(digital_direction, digital_enable, forward_left_pin, backward_left_pin, enable_left_pin),
      _right_motor(digital_direction, digital_enable, forward_right_pin, backward_right_pin, enable_right_pin)
{
}

int16_t MotorDriveUnit::_apply_deadzone(int16_t value, uint8_t deadzone)
{
  value = utils::clamp(value, -255, 255);
  if (abs(value) < deadzone)
    return 0;

  bool positive = (value >= 0);
  int16_t result = map(abs(value) - deadzone, 0, 255 - deadzone, 0, 255);
  return positive ? result : -result;
}

void MotorDriveUnit::begin()
{
  if (_initialized)
    return;

  _initialized = true;

  _left_motor.begin();
  _right_motor.begin();
}

int16_t MotorDriveUnit::_apply_deadzone_to_source(int16_t source_result)
{
  return _apply_deadzone(utils::clamp(source_result, -255, 255), _deadzone);
}

void MotorDriveUnit::applyDifferentialDrive() // FIX THISSSSSSS
{
  int16_t power = _apply_deadzone_to_source(_get_power_source());
  float direction = (float)(_apply_deadzone_to_source(_get_direction_source())) / 255.0f;
  int16_t powers[2] = {0, 0};

  if (power == 0 || direction == 0.0f)
  {
    powers[0] = powers[1] = power;
    return;
  }

  if (direction > 0.0f)
  {
    // Turning right
    powers[0] = power;
    powers[1] = (int16_t)(power * (1.0f - direction));
  }
  else
  {
    // Turning left
    powers[0] = (int16_t)(power * (1.0f + direction));
    powers[1] = power;
  }

  _left_motor.setPower(powers[0]);
  _right_motor.setPower(powers[1]);
}

int16_t invert_power(int16_t power)
{
  bool positive = power >= 0;
  int16_t inverted = 255 - abs(power);
  return inverted * (!positive ? 1 : -1);
}

void MotorDriveUnit::applyTankDrive() // FIX THISSSSSSS
{
  int16_t power = _apply_deadzone_to_source(_get_power_source());
  float direction = (float)(_apply_deadzone_to_source(_get_direction_source())) / 255.0f;
  int16_t powers[2] = {0, 0};

  if (power == 0 || direction == 0.0f)
  {
    powers[0] = powers[1] = power;
    return;
  }

  if (direction > 0.0f)
  {
    // Turning right
    powers[0] = power;
    powers[1] = -((int16_t)(power * direction));
  }
  else
  {
    // Turning left
    powers[0] = -((int16_t)(power * direction));
    powers[1] = power;
  }

  _left_motor.setPower(powers[0]);
  _right_motor.setPower(powers[1]);
}