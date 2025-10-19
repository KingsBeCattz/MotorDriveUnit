#include <Arduino.h>
#include <HardwareSerial.h>
#include <MotorDriveUnit.h>
#include <Motor.h>
#include <Bluepad32.h>

#define forward_left_pin 27
#define backward_left_pin 14
#define forward_right_pin 17
#define backward_right_pin 16

unsigned int unsigned_ceil(float x)
{
  unsigned int i = (unsigned int)x;
  return (x > i) ? i + 1 : i;
}

bool SELECT_PRESSED = false;
bool START_PRESSED = false;

uint16_t disconnect_controller_time = 0;

void listenButtonState(bool buttonState, bool &previousState, void (*onPressed)(), void (*onReleased)())
{
  // Detect button press
  if (!previousState && buttonState)
  {
    previousState = true;
    if (onPressed)
      onPressed();
  }

  // Detect button release
  else if (previousState && !buttonState)
  {
    previousState = false;
    if (onReleased)
      onReleased();
  }
}

ControllerPtr current_controller = nullptr; // This is the current control

MotorDriveUnit motor_driver(false, false, forward_left_pin, backward_left_pin, forward_right_pin, backward_right_pin, Motor::PIN_UNUSED, Motor::PIN_UNUSED);

void on_connect(ControllerPtr new_gamepad)
{
  if (current_controller == nullptr)
  {
    Serial.print("--- Controller of ");
    Serial.print(new_gamepad->getModelName());
    Serial.println(" is connected ---");
    current_controller = new_gamepad;
    delay(5);
    digitalWrite(2, HIGH);
    BP32.enableNewBluetoothConnections(false);
  }
  else
  {
    new_gamepad->disconnect();
  }
}

void on_disconnect(ControllerPtr gamepad)
{
  if (current_controller == gamepad)
  {
    Serial.println("--- Controller is disconnected ---");
    current_controller = nullptr;

    motor_driver.stop();
    BP32.enableNewBluetoothConnections(true);
  }
}

int16_t use_triggers()
{
  uint8_t LT = current_controller->l1() ? 255 : unsigned_ceil(current_controller->brake() / 4);    // Backward
  uint8_t RT = current_controller->r1() ? 255 : unsigned_ceil(current_controller->throttle() / 4); // Forward

  // Divide by 4, maps 1023 to ~255

  return RT - LT;
}

int16_t use_left_y_axis()
{
  if (current_controller->r1() || current_controller->l1())
  {
    uint8_t forward = (current_controller->r1() & 1) ? 255 : 0;
    uint8_t backward = (current_controller->l1() & 2) ? 255 : 0;

    return forward - backward;
  }

  int16_t axis = current_controller->axisY();
  bool positive = axis <= 0;
  axis = abs(axis);
  axis = unsigned_ceil(abs(axis) / 2);

  if (!positive)
    axis = -axis;

  return axis;
}

int16_t use_left_x_axis()
{
  if (current_controller->dpad())
  {
    uint8_t left = (current_controller->dpad() & 8) ? 255 : 0;
    uint8_t right = (current_controller->dpad() & 4) ? 255 : 0;

    return right - left;
  }

  int16_t axis = current_controller->axisX() - 2;
  bool positive = axis >= 0;
  axis = abs(axis);
  axis = unsigned_ceil(abs(axis) / 2);

  if (!positive)
    axis = -axis;

  return axis;
}

int16_t use_right_x_axis()
{
  if (current_controller->dpad())
  {
    uint8_t left = (current_controller->dpad() & 8) ? 255 : 0;
    uint8_t right = (current_controller->dpad() & 4) ? 255 : 0;

    return right - left;
  }

  int16_t axis = current_controller->axisRX() - 2;
  bool positive = axis >= 0;
  axis = abs(axis);
  axis = unsigned_ceil(abs(axis) / 2);

  if (!positive)
    axis = -axis;

  return axis;
}

void switch_power_source()
{
  int16_t (*current_source)() = motor_driver.getPowerSourceFunction();

  if (current_source == use_triggers)
  {
    motor_driver.setPowerSource(use_left_y_axis);
  }
  else
  {
    motor_driver.setPowerSource(use_triggers);
  }
}

void switch_direction_source()
{
  int16_t (*current_source)() = motor_driver.getDirectionSourceFunction();

  if (current_source == use_left_x_axis)
  {
    motor_driver.setDirectionSource(use_right_x_axis);
  }
  else
  {
    motor_driver.setDirectionSource(use_left_x_axis);
  }
}

void setup()
{
  Serial.begin(115200);
  pinMode(2, OUTPUT);

  BP32.setup(&on_connect, &on_disconnect);
  BP32.forgetBluetoothKeys();

  motor_driver.begin();
  motor_driver.setDeadzone(70);

  motor_driver.setPowerSource(use_triggers);
  motor_driver.setDirectionSource(use_left_x_axis);
}

void loop()
{
  BP32.update();

  if (!current_controller)
  {
    if (disconnect_controller_time != 0)
    {
      disconnect_controller_time = 0;
    }

    motor_driver.stop();
    digitalWrite(2, HIGH);
    delay(100);
    digitalWrite(2, LOW);
    delay(100);
    return;
  }
  else
  {
    digitalWrite(2, HIGH);
  }

  if (current_controller->isConnected() && current_controller->isGamepad())
  {
    if (current_controller->miscSystem())
    {
      disconnect_controller_time += 10;
    }
    else if (disconnect_controller_time != 0)
    {
      disconnect_controller_time = 0;
    }

    if (disconnect_controller_time >= 2000)
    {
      current_controller->disconnect();

      return;
    }

    listenButtonState(current_controller->miscBack(), SELECT_PRESSED, switch_power_source, nullptr);    // miscBack is synonymous with miscSelect
    listenButtonState(current_controller->miscHome(), START_PRESSED, switch_direction_source, nullptr); // miscHome is synonymous with miscStart

    motor_driver.setExpositionActive(current_controller->y());

    if (current_controller->a())
      motor_driver.useManualDrive(unsigned_ceil(current_controller->brake() / 4), !current_controller->l1(), unsigned_ceil(current_controller->throttle() / 4), !current_controller->r1());
    else
    {
      if (current_controller->x())
        motor_driver.useTankDrive();

      motor_driver.update();
    }
  }

  delay(10);
}