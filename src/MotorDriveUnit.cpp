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

void MotorDriveUnit::setExpositionActive(bool state)
{
  if (_exposition_active_now && !state)
  {
    _exposition_mode = (_exposition_hold_power != 0);
  }

  _exposition_active_now = state;
}

void MotorDriveUnit::toggleTankDriveMode()
{
  _tank_drive_mode = !_tank_drive_mode;
}

void MotorDriveUnit::update()
{
  if (!_initialized || _get_power_source == nullptr || _get_direction_source == nullptr)
    return;

  int16_t power = _apply_deadzone_to_source(_get_power_source());

  // ──────────────── EXHIBITION MODE ────────────────
  if (_exposition_active_now)
  {
    _exposition_hold_power = power;
    _exposition_mode = false;

    if (power == 0)
      stop();
    else
    {
      _left_motor.setPower(power);
      _right_motor.setPower(-power);
    }
    return;
  }

  if (_exposition_mode)
  {
    if (_exposition_hold_power == 0)
    {
      _exposition_mode = false;
      stop();
    }
    else
    {
      _left_motor.setPower(_exposition_hold_power);
      _right_motor.setPower(-_exposition_hold_power);
    }
    return;
  }

  // ──────────────── NORMAL / TANK DRIVE ────────────────
  float direction = (float)(_apply_deadzone_to_source(_get_direction_source())) / 255.0f;
  int16_t powers[2] = {power, power};

  if (direction > 0.0f)
  {
    if (_tank_drive_mode)
      powers[1] = -(int16_t)(powers[1] * direction);
    else
      powers[1] = (int16_t)(powers[1] * (1.0f - direction));
  }
  else if (direction < 0.0f)
  {
    if (_tank_drive_mode)
      powers[0] = -(int16_t)(powers[0] * -direction);
    else
      powers[0] = (int16_t)(powers[0] * (1.0f + direction));
  }

  _left_motor.setPower(powers[0]);
  _right_motor.setPower(powers[1]);
}
