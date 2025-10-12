#include "Motor.h"
#include "Utils.h"

Motor::Motor(bool digital_direction, bool digital_enable, uint8_t forward_pin, uint8_t backward_pin, uint8_t enable_pin)
    : _digital_direction(digital_direction),
      _digital_enable(digital_enable),
      _forward_pin(forward_pin),
      _backward_pin(backward_pin),
      _enable_pin(enable_pin),
      _pwm_value(0),
      _forward(true)
{
}

void Motor::_apply_power(uint8_t forward_power, uint8_t backward_power)
{
  _write_pin(_forward_pin, forward_power, _digital_direction);
  _write_pin(_backward_pin, backward_power, _digital_direction);

  uint8_t enable_power = (forward_power > backward_power) ? forward_power : backward_power;

  if (_enable_pin != PIN_UNUSED)
    _set_enable(enable_power);

  _pwm_value = enable_power;
  _forward = (forward_power >= backward_power);
}

void Motor::_write_pin(uint8_t pin, uint8_t value, bool digital, bool apply_deadzone)
{
  if (pin == PIN_UNUSED)
    return;

  if (digital)
  {
    bool state = (value > 0);
    if (apply_deadzone)
      state = (value > _digital_pin_dead_zone);

    digitalWrite(pin, state);
  }
  else
  {
    analogWrite(pin, value);
  }
}

void Motor::begin()
{
  if (_initialized)
    return;

  _initialized = true;
  pinMode(_forward_pin, OUTPUT);
  pinMode(_backward_pin, OUTPUT);

  if (_enable_pin != PIN_UNUSED)
  {
    pinMode(_enable_pin, OUTPUT);
    _set_enable(0);
  }

  stop();
}

void Motor::setPower(int16_t power)
{
  if (power == 0)
  {
    stop();
    return;
  }

  // Clamp the power within safe range
  power = utils::clamp(power, -255, 255);

  bool is_forward = (power > 0);
  uint8_t clamped_power = static_cast<uint8_t>(abs(power));

  // Skip redundant updates
  if (is_forward == _forward && clamped_power == _pwm_value)
    return;

  if (is_forward)
    _apply_power(clamped_power, 0);
  else
    _apply_power(0, clamped_power);
}

void Motor::forward(uint8_t power)
{
  power = static_cast<uint8_t>(utils::clamp(power, 0, 255));
  _apply_power(power, 0);
}

void Motor::backward(uint8_t power)
{
  power = static_cast<uint8_t>(utils::clamp(power, 0, 255));
  _apply_power(0, power);
}
