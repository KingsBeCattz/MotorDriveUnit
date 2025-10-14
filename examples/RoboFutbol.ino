/**
 * ROBO-FUTBOL XOCHIMILCO 2025
 * ESP32-WROOM-32-N4
 * SUPPORTED GAMEPADS: https://bluepad32.readthedocs.io/en/latest/supported_gamepads/
 * Written by: Johan 5CMT
 */

#include <MotorDriveUnit.h>
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

ControllerPtr current_controller = nullptr; // This is the current control

MotorDriveUnit motor_driver(forward_left_pin, backward_left_pin, forward_right_pin, backward_right_pin);

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

int16_t power_source()
{
  uint8_t LT = current_controller->l1() ? 255 : unsigned_ceil(current_controller->brake() / 4);    // Backward
  uint8_t RT = current_controller->r1() ? 255 : unsigned_ceil(current_controller->throttle() / 4); // Forward

  // Divide by 4, maps 1023 to ~255

  return RT - LT;
}

int16_t direction_source()
{
  return current_controller->axisX() / 2;
}

void setup()
{
  Serial.begin(115200);
  pinMode(2, OUTPUT);

  BP32.setup(&on_connect, &on_disconnect);
  BP32.forgetBluetoothKeys();

  motor_driver.begin();
  motor_driver.setDeadzone(75);

  motor_driver.setPowerSource(power_source);
  motor_driver.setDirectionSource(direction_source);
}

void loop()
{
  BP32.update();

  if (current_controller && current_controller->isConnected() && current_controller->isGamepad())
  {

    motor_driver.setExpositionActive(current_controller->y());

    if (current_controller->x())
      motor_driver.toggleTankDriveMode();

    motor_driver.update();
    digitalWrite(2, HIGH);
    delay(10);
  }
  else
  {
    motor_driver.stop();
    digitalWrite(2, HIGH);
    delay(100);
    digitalWrite(2, LOW);
    delay(100);
  }
}