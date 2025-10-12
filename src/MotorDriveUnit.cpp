#include "MotorDriveUnit.h"
#include "Utils.h"

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

void MotorDriveUnit::_process_power_and_direction(int16_t (&powers)[2])
{
  int16_t power = _apply_deadzone(utils::clamp(_get_power_source(), -255, 255), _deadzone);
  int16_t direction = _apply_deadzone(utils::clamp(_get_direction_source(), -255, 255), _deadzone);

  powers[0] = powers[1] = abs(power);
  bool forward = (power >= 0);
  uint8_t abs_direction = abs(direction);
  bool turn_right = (direction >= 0);

  if (turn_right)
  {
    powers[1] -= abs_direction;
  }
  else
  {
    powers[0] -= abs_direction;
  }

  if (!forward)
  {
    powers[0] = -powers[0];
    powers[1] = -powers[1];
  }
  return;
}

void MotorDriveUnit::applyDifferentialDrive()
{
  int16_t powers[2] = {0, 0};
  _process_power_and_direction(powers);

  _left_motor.setPower(powers[0]);
  _right_motor.setPower(powers[1]);
}

void MotorDriveUnit::applyTankDrive()
{
  int16_t direction = utils::clamp(_get_direction_source(), -255, 255);
  int16_t powers[2] = {0, 0};
  _process_power_and_direction(powers);

  if (direction > 0)
  {
    bool right_is_forward = (powers[1] >= 0);
    powers[1] = 255 - abs(powers[1]);
    if (!right_is_forward)
      powers[1] = -powers[1];

    _left_motor.setPower(powers[0]);
  }
  else if (direction < 0)
  {
    bool left_is_forward = (powers[0] >= 0);
    powers[0] = 255 - abs(powers[0]);
    if (!left_is_forward)
      powers[0] = -powers[0];

    _right_motor.setPower(powers[1]);
  }
  else
  {
    _left_motor.setPower(powers[0]);
    _right_motor.setPower(powers[1]);
  }
}