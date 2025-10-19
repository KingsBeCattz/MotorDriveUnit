// This code has not been tested at the time of publication. Please report any problems.

#include <PS2X_lib.h>
#include <MotorDriveUnit.h>

#define IN1_PWM 9
#define IN2_PWM 6
#define IN3_PWM 5
#define IN4_PWM 3

PS2X ps2x;
MotorDriveUnit motor_driver(false, false, IN1_PWM, IN2_PWM, IN3_PWM, IN4_PWM, Motor::PIN_UNUSED, Motor::PIN_UNUSED);

uint8_t left_stick[2] = {128, 128};
uint8_t right_stick[2] = {128, 128};

uint8_t *x_stick = right_stick;
uint8_t *y_stick = left_stick;

// --- Stick helpers ---
inline int16_t map_stick(uint8_t value)
{
  int16_t delta = (int16_t)value - 128;
  if (delta > 0)
    return delta * 255 / 127;
  if (delta < 0)
    return delta * 255 / 128;
  return 0;
}

inline int16_t use_left_y_axis()
{
  return map_stick(y_stick[1]);
}

inline int16_t use_left_x_axis()
{
  return map_stick(x_stick[0]);
}

void set_stick_position(uint8_t (&stick)[2], uint8_t x, uint8_t y)
{
  stick[0] = x;
  stick[1] = 255 - y;
}

// --- Stick swapping ---
void switch_power_source()
{
  int16_t (*current_source)() = motor_driver.getPowerSourceFunction();
  if (current_source == use_left_y_axis)
    motor_driver.setPowerSource(use_left_x_axis);
  else
    motor_driver.setPowerSource(use_left_y_axis);
}

void switch_direction_source()
{
  int16_t (*current_source)() = motor_driver.getDirectionSourceFunction();
  if (current_source == use_left_x_axis)
    motor_driver.setDirectionSource(use_left_y_axis);
  else
    motor_driver.setDirectionSource(use_left_x_axis);
}

void setup()
{
  Serial.begin(115200);
  pinMode(2, OUTPUT);

  motor_driver.begin();
  motor_driver.setDeadzone(70);
  motor_driver.setPowerSource(use_left_y_axis);
  motor_driver.setDirectionSource(use_left_x_axis);

  switch (ps2x.config_gamepad(10, 12, 11, 13, false, false))
  {
  case 0:
    Serial.println("Controller found.");
    break;
  case 1:
    Serial.println("No controller found.");
    break;
  case 2:
    Serial.println("Controller found but not accepting commands.");
    break;
  case 3:
    Serial.println("Controller refusing to enter Pressures mode.");
    break;
  }
}

void loop()
{
  ps2x.read_gamepad();
  set_stick_position(left_stick, ps2x.Analog(PSS_LX), ps2x.Analog(PSS_LY));
  set_stick_position(right_stick, ps2x.Analog(PSS_RX), ps2x.Analog(PSS_RY));

  if (ps2x.ButtonPressed(PSB_SELECT))
    switch_power_source();
  if (ps2x.ButtonPressed(PSB_START))
    switch_direction_source();

  if (ps2x.Button(PSB_BLUE))
  {
    int16_t left_power = map_stick(y_stick[0]);
    int16_t right_power = map_stick(y_stick[0]);
    motor_driver.useManualDrive(static_cast<uint8_t>(abs(left_power)), left_power >= 0, static_cast<uint8_t>(abs(right_power)), right_power >= 0);
  }

  else if (ps2x.Button(PSB_GREEN))
  {
    motor_driver.setExpositionActive(map_stick(y_stick[1]));
  }
  else if (ps2x.Button(PSB_PINK))
  {
    motor_driver.useTankDrive();
  }
  else
  {
    motor_driver.update();
  }

  delay(10);
}
