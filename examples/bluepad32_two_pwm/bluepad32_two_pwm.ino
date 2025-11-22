#include <Arduino.h>
#include <HardwareSerial.h>
#include <MotorDriveUnit.h>
#include <Motor.h>
#include <Bluepad32.h>

// -----------------------------------------------------
// Motor pin definitions
// -----------------------------------------------------
#define LEFT_FORWARD_PIN 27
#define LEFT_BACKWARD_PIN 14
#define LEFT_ENABLE 32
#define RIGHT_FORWARD_PIN 17
#define RIGHT_BACKWARD_PIN 16
#define RIGHT_ENABLE 23

// -----------------------------------------------------
// Utility Functions
// -----------------------------------------------------
unsigned int unsignedCeil(float x)
{
  unsigned int i = (unsigned int)x;
  return (x > i) ? i + 1 : i;
}

// -----------------------------------------------------
// Controller State
// -----------------------------------------------------
bool SELECT_PRESSED = false;
bool START_PRESSED = false;

uint16_t controllerDisconnectTimer = 0;

ControllerPtr currentController = nullptr;

// -----------------------------------------------------
// Motor Driver
// -----------------------------------------------------
MotorDriveUnit motorDriver;

// -----------------------------------------------------
// Button Event Listener
// -----------------------------------------------------
void listenButtonState(bool buttonState, bool &previousState,
                       void (*onPressed)(), void (*onReleased)())
{

  if (!previousState && buttonState)
  {
    previousState = true;
    if (onPressed)
      onPressed();
  }
  else if (previousState && !buttonState)
  {
    previousState = false;
    if (onReleased)
      onReleased();
  }
}

// -----------------------------------------------------
// Controller Connection Callbacks
// -----------------------------------------------------
void onConnect(ControllerPtr pad)
{
  if (!currentController)
  {
    Serial.printf("--- Controller %s connected ---\n", pad->getModelName());
    currentController = pad;

    digitalWrite(2, HIGH);
    delay(5);

    BP32.enableNewBluetoothConnections(false);
  }
  else
  {
    pad->disconnect();
  }
}

void onDisconnect(ControllerPtr pad)
{
  if (currentController == pad)
  {
    Serial.println("--- Controller disconnected ---");
    currentController = nullptr;

    motorDriver.stop();
    BP32.enableNewBluetoothConnections(true);
  }
}

// -----------------------------------------------------
// Input Mapping Functions
// -----------------------------------------------------
int16_t useTriggers()
{
  // Map triggers to ±255
  uint8_t LT = currentController->l1() ? 255 : unsignedCeil(currentController->brake() / 4);
  uint8_t RT = currentController->r1() ? 255 : unsignedCeil(currentController->throttle() / 4);

  return RT - LT;
}

int16_t useLeftYAxis()
{
  if (currentController->l1() || currentController->r1())
  {
    uint8_t forward = currentController->r1() ? 255 : 0;
    uint8_t backward = currentController->l1() ? 255 : 0;
    return forward - backward;
  }

  int16_t axis = currentController->axisY();
  bool positive = axis <= 0;

  axis = abs(axis);
  axis = unsignedCeil(axis / 2);

  return positive ? axis : -axis;
}

int16_t useLeftXAxis()
{
  if (currentController->dpad())
  {
    uint8_t left = (currentController->dpad() & 8) ? 255 : 0;
    uint8_t right = (currentController->dpad() & 4) ? 255 : 0;
    return right - left;
  }

  int16_t axis = currentController->axisX();
  axis -= 2;

  bool positive = axis >= 0;
  axis = unsignedCeil(abs(axis) / 2);

  return positive ? axis : -axis;
}

int16_t useRightXAxis()
{
  if (currentController->dpad())
  {
    uint8_t left = (currentController->dpad() & 8) ? 255 : 0;
    uint8_t right = (currentController->dpad() & 4) ? 255 : 0;
    return right - left;
  }

  int16_t axis = currentController->axisRX();
  axis -= 2;

  bool positive = axis >= 0;
  axis = unsignedCeil(abs(axis) / 2);

  return positive ? axis : -axis;
}

// -----------------------------------------------------
// Power / Direction Input Switching
// -----------------------------------------------------
void switchPowerSource()
{
  auto current = motorDriver.getPowerSourceFunction();
  motorDriver.setPowerSource(
      current == useTriggers ? useLeftYAxis : useTriggers);
}

void switchDirectionSource()
{
  auto current = motorDriver.getDirectionSourceFunction();
  motorDriver.setDirectionSource(
      current == useLeftXAxis ? useRightXAxis : useLeftXAxis);
}

// -----------------------------------------------------
// Setup & Loop
// -----------------------------------------------------
void setup()
{
  Serial.begin(115200);
  pinMode(2, OUTPUT);

  BP32.setup(&onConnect, &onDisconnect);
  BP32.forgetBluetoothKeys();

  motorDriver.begin();
  motorDriver.setDeadzone(70);

  Motor leftMotor = motorDriver.getLeftMotor();
  leftMotor.setDirectionPins(LEFT_FORWARD_PIN, LEFT_BACKWARD_PIN, true);
  leftMotor.setEnablePin(LEFT_ENABLE, false);

  Motor rightMotor = motorDriver.getRightMotor();
  rightMotor.setDirectionPins(RIGHT_FORWARD_PIN, RIGHT_BACKWARD_PIN, true);
  rightMotor.setEnablePin(RIGHT_ENABLE, false);

  motorDriver.setPowerSource(useTriggers);
  motorDriver.setDirectionSource(useLeftXAxis);
}

void loop()
{
  BP32.update();

  if (!currentController)
  {
    controllerDisconnectTimer = 0;

    motorDriver.stop();
    digitalWrite(2, HIGH);
    delay(100);
    digitalWrite(2, LOW);
    delay(100);
    return;
  }

  digitalWrite(2, HIGH);

  if (currentController->isConnected() && currentController->isGamepad())
  {

    if (currentController->miscSystem())
    {
      controllerDisconnectTimer += 10;
    }
    else
    {
      controllerDisconnectTimer = 0;
    }

    if (controllerDisconnectTimer >= 2000)
    {
      currentController->disconnect();
      return;
    }

    // Handle mode switches
    listenButtonState(currentController->miscBack(), SELECT_PRESSED, switchPowerSource, nullptr);
    listenButtonState(currentController->miscHome(), START_PRESSED, switchDirectionSource, nullptr);

    // Exposition mode
    motorDriver.setExpositionActive(currentController->y());

    // Manual drive (A button)
    if (currentController->a())
    {
      motorDriver.useManualDrive(
          unsignedCeil(currentController->brake() / 4), !currentController->l1(),
          unsignedCeil(currentController->throttle() / 4), !currentController->r1());
    }
    else
    {
      if (currentController->x())
        motorDriver.useTankDrive();

      motorDriver.update();
    }
  }

  delay(10);
}
