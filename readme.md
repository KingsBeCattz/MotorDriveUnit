# 🚀 MotorDriveUnit
High-level, hardware-agnostic Arduino library for controlling single and dual DC motors.  
Supports PWM or digital H-bridge control, tank drive, differential drive, exposition mode, deadzones, and fully pluggable input sources.

Designed primarily for mechatronics and robotics projects using boards like ESP32, Arduino Uno, and Mega.

---

# ✨ Features

### 🎮 Input & Control
- Fully pluggable **power source** and **direction source** callbacks  
- Compatible with gamepads, joysticks, sensors, or custom logic  
- Built-in examples for Bluepad32

### 🔧 Motor Control
- Robust `Motor` class for single-motor abstraction
- Support for **2-pin** and **3-pin** H-bridge configurations
- Direction pins can be PWM or digital
- Enable pin can be PWM or digital
- Automatic safe-reverse braking (short idle period)

### 🚗 Drive Modes
- Differential steering (default)
- Tank drive (permanent or one-shot)
- Manual motor override mode

### 🌀 Exposition Mode
- Immediate spinning behavior (left forward, right backward)
- Sustained demo mode using last non-zero power

### 🛡 Filtering & Safety
- Deadzone filtering for jitter-free control
- Global driver enable pin support
- Internal clamping and safe power transitions

---

# 📦 Installation

Place the library folder in your Arduino `libraries/` directory:

```
MotorDriveUnit/
├─ src/
│  ├─ Motor.h
│  ├─ Motor.cpp
│  ├─ MotorDriveUnit.h
│  ├─ MotorDriveUnit.cpp
│  └─ Utils.h
└─ examples/
   └─ bluepad32_four_pwm/
```

Include it in your sketch:

```cpp
#include <MotorDriveUnit.h>
```

---

# 🧩 Library Architecture

```
 +------------------------+
 |    MotorDriveUnit      |
 |------------------------|
 | - Input sources        |
 | - Deadzone filtering   |
 | - Tank drive logic     |
 | - Exposition logic     |
 | - Output mixing        |
 +-----------+------------+
             |
   +---------+---------+
   |                   |
+-------+         +---------+
| Motor |         |  Motor  |
+-------+         +---------+
 (Left)             (Right)
```

---

# 🛠 Usage Lifecycle

## 1. Create the controller
```cpp
MotorDriveUnit drive;
```

## 2. Configure motor pins
```cpp
drive.getLeftMotor().setDirectionPins(LF_PIN, LB_PIN, false);
drive.getLeftMotor().setEnablePin(LEFT_EN_PIN, false);

drive.getRightMotor().setDirectionPins(RF_PIN, RB_PIN, false);
drive.getRightMotor().setEnablePin(RIGHT_EN_PIN, false);
```

## 3. Optional: set global driver enable pin
```cpp
drive.setDriverEnablePin(DRIVER_EN_PIN, true);
```

## 4. Initialize motors
```cpp
drive.begin();
```

## 5. Configure behavior
```cpp
drive.setDeadzone(60);
drive.setPowerSource(myPowerFunction);
drive.setDirectionSource(myDirectionFunction);
```

## 6. In loop()
```cpp
drive.update();
```

---

# 🔷 Motor Class

```cpp
void begin();
void stop();
void setPower(int16_t power);
void forward(uint8_t power);
void backward(uint8_t power);
void setDirectionPins(uint8_t forward, uint8_t backward, bool digital);
void setEnablePin(uint8_t enablePin, bool digital);
void setDigitalPinDeadZone(uint8_t deadzone);
```

---

# 🔷 MotorDriveUnit Class

```cpp
void begin();
void update();
void stop();
void setDriverEnablePin(uint8_t pin, bool digitalEnable);
void toggleDriverEnabled();
void setDeadzone(uint8_t deadzone);
void setPowerSource(int16_t (*func)());
void setDirectionSource(int16_t (*func)());
void toggleTankDriveMode();
void useTankDrive();
void setExpositionActive(bool state);
void useManualDrive(uint8_t lPower, bool lForward,
                    uint8_t rPower, bool rForward);

const Motor& getLeftMotor() const;
const Motor& getRightMotor() const;

int16_t (*getPowerSourceFunction())();
int16_t (*getDirectionSourceFunction())();
```

---

# 🎮 Input Sources

Example:

```cpp
int16_t readJoystickY() {
    int16_t raw = analogRead(A0) - 512;
    return raw / 2;
}
```

Assign:

```cpp
drive.setPowerSource(readJoystickY);
drive.setDirectionSource(readJoystickX);
```

---

# 🌀 Exposition Mode

| State | Behavior |
|-------|----------|
| Active now | Spins immediately (L=power, R=−power) |
| Disabled with saved power | Enters sustained spinning |
| Sustained power = 0 | Stops motors and exits |

---

# 🚗 Tank Drive

### Permanent:
```cpp
drive.toggleTankDriveMode();
```

### One-shot:
```cpp
drive.useTankDrive();
```

---

# 📘 Example

### Bluepad32 Controller  
```
examples/bluepad32_four_pwm/
```

---

# 📄 License

MIT License  
