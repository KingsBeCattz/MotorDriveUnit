# CHANGELOG V2.0.0

## Motor Class Updates

- Reworked the Motor class into a **header-only implementation** (removed `Motor.cpp`).
- Removed constructor-based configuration; the class now uses **runtime pin assignment** via:
  - `setDirectionPins()`
  - `setEnablePin()`
- Improved flexibility: direction pins and enable pin now support **independent digital/PWM modes**.
- Added **safe direction switching**: `setPower()` now stops the motor and waits briefly before reversing.
- `_apply_power()` updated to use the **max** of forward/backward PWM as the effective enable value.
- Added digital-mode **dead-zone support** (`setDigitalPinDeadZone()`).
- `begin()` now initializes only assigned pins and always starts the motor in a safe stopped state.
- Internal logic consolidated; simplified state transitions and removed redundant operations.

---

## MotorDriveUnit Class Updates

### New Features  
- Added support for a **global driver enable pin** via `setDriverEnablePin()`, allowing complete activation/deactivation of both motors through a single digital or PWM signal.  
- New configuration architecture based on **external motor setup**:
  - Motors now receive pin configuration through:
    - `Motor::setDirectionPins(...)`
    - `Motor::setEnablePin(...)`
  - Old constructor-based pin assignment removed.  
- Extensive documentation added, covering initialization flow, operating modes, dead-zone behavior, Exposition Mode, Tank Drive behavior, and input requirements.

### Behavior Changes  
- Unified clamping and dead-zone filtering for all drive inputs.  
- Exposition Mode and Tank Drive modes now follow clearly defined, predictable behavior.  
- Driver enable pin is now explicitly controlled before applying any motor power.

### Refactoring and Internal Improvements  
- Private internals reorganized into clean categories (config, modes, input sources, global driver state).  
- Naming improved and all fields documented with professional detail.  
- Removed multiple legacy constructors; replaced by flexible setup through getters and external configuration.  
- Clear isolation between:
  - `setPower()` logic  
  - Filtering  
  - Special drive modes  
  - Global enable/disable controller  

### Breaking Changes  
- **All previous constructors removed**:
  - Direction-only constructor  
  - Direction + enable constructor  
  - Configurable digital/PWM constructor  
- Users must now:
  1. Retrieve motors via `getLeftMotor()` and `getRightMotor()`.  
  2. Assign pins manually.  
  3. Call `begin()` before usage.

This change breaks compatibility with any older code that relied on the previous constructor system.

### Documentation Updates  
- Complete rewrite of class documentation.  
- Clearly defined initialization workflow.  
- Detailed explanation of Exposition Mode, Tank Drive, and state-based behaviors.  
- Updated explanation of input source behavior and return requirements.
