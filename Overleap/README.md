# Overleap Arduino Library

This library is for controlling Overleap with Arduino. The full project can be found at the [Overleap repository](https://github.com/aarondls/overleap).

## Usage

### Structures

There are two structs defined: `MotorTorquePair` and `ForceProfile`.

```c++
struct MotorTorquePair {
  float motor_0;
  float motor_1;
};

struct ForceProfile {
  unsigned long duration; // duration of force in ms
  float a; // amplitude
  float b; // period factor, which is 1000/force duration in ms
  float max_fx_force; // maximum fx force to be applied in N
  float fx_proportion; // proportion of fy force that is applied as fx
};
```

`MotorTorquePair` is used to group together the torque values for both axis, while `ForceProfile` is used to track relevant parameters of the applied force profile.

### Enums

One enum is defined: `Phase`.

```c++
enum Phase {AERIAL_PHASE = 0, TOUCHDOWN_PHASE};
```

This describes the current phase of the leg. `AERIAL_PHASE` is when the leg is in the air and not touching the ground, while `TOUCHDOWN_PHASE` is when the leg is touching the ground. The phase is determined by the custom foot contact sensor.

### Overleap Class

The Overleap class contains several data members and member functions important for controlling Overleap.

```c++
Overleap(float length_0, float length_1, Stream& odrive_serial, Stream& arduino_serial, ODriveArduino& odrive);
```

* This is a constructor that takes in the length of the upper and lower leg as `length_0` and `length_1`, the ODrive and Arduino serial streams, and the ODrive object.

```c++
void SearchEncoderIndex();
```

* This searches for the encoder index signal on both axis, starting with axis 0. There is an intentional slight delay before searching on axis 1. Before calling this function, both the Arduino and ODrive serial ports must have begun. This function is required to run at every start-up to prepare the encoders.

```c++
MotorTorquePair CalculateTorques(float f_x, float f_y);
```

* This calculates needed torque on motors 0 and 1 to achieve desired forces `f_x` and `f_y` on the foot based on current leg position. The calculation is done using transpose of Jacobian. Sign of force directions are with respect to foot: positive x force is to the right while positive y force is downwards, and positive torque is counterclockwise.

```c++
bool LowBatteryVoltage();
```

* This returns true if battery voltage is below 12V. The current voltage is also printed to the Arduino serial monitor.

```c++
bool WithinSafeAngles();
```

* This returns true if current leg position is within safe angles. Safe angles are defined to be between -10 and 75 degrees for θ<sub>0</sub>, and within 0 and 135 degrees for θ<sub>1</sub>.

```c++
bool SafeTorqueDirection(MotorTorquePair torque_pair);
```

* This returns true if torque is in safe direction, even if the leg is not within safe angles. Safe direction is defined to be a value of torque that will not cause the leg to move away from safe angle. This is useful if leg is not within safe angles, but current torque will bring it towards safe angles.
* The direction of torque follows Overleap's direction of torque for both axis: positive torque goes clockwise for axis 0, while positive torque goes counterclockwise for axis 1.

```c++
bool Touchdown();
```

* This returns true if leg is on the ground as detected by the foot contact sensor.

```c++
void UpdateForceProfile(float force_impulse, float fx_max, float fx_proportion);
```

* This updates the force profile member variable with impulse of `force_impulse` in N ms, maximum fx force of `fx_max`, and proportion of fy to be applied as fx of `fx_proportion`.

```c++
void UpdatePhase();
```

* This updates the current phase of the leg as detected by the foot contact sesor.

```c++
bool RequestControlMode(int axis, int requested_control_mode, int timeout=1);
```

* This requests the given control mode. Timeout is in seconds and is approximate. It returns true if sucessful.

```c++
bool RequestAxisState(int axis, int requested_axis_state, int timeout=1);
```

* This requests the given axis state. Timeout is in seconds and is approximate. It returns true if sucessful.


```c++
bool RequestIdleState();
```

* This puts both axis into idle state. It returns true if successful, false if not.

```c++
bool EnterTorqueControl();
```

* Before calling, angles must already be calibrated. This sets the control mode to torque and enters closed loop control for both axis. The initial torque is set to 0. It returns true if successful, false if not.

```c++
void HoldPosition(float pd_0, float pd_1, bool end_at_touchdown, unsigned long duration=10000);
```

* Before calling, leg must be in torque mode and closed loop control. `pd_0` is desired position in degrees for axis 0, while `pd_1` is desired position in degrees for axis 1. `end_at_touchdown` should be set if leg needs to end position hold at touchdown. `duration` is the length of the position hold in milliseconds.
* If `end_at_touchdown` is set, position hold also ends when leg touches the ground. `end_at_touchdown` is also used to determine how "stiff" the position hold is: if ending at touchdown, the gains are less, if not ending at touchdown, then the gains are higher so the leg can support itself
* It ends by commanding zero torque.

```c++
void ExecuteForceProfile();
```

* Before calling, leg must be in torque control mode and closed loop control. This executes the force profile with the set parameters. Execution of force profile stops if foot is no longer on the ground or if force duration is exceeded. It ends by commanding zero torque.

```c++
bool CalibrateAngles();
```

* This calibrates zero angle position for both angles. Upper leg zero angle is taken with respect from the horizontal, while lower leg zero angle is taken with respect from (extending) the upper leg.

```c++
bool CalibratedAngles();
```

* This returns true if angles have been calibrated. This is done by calling the `CalibrateAngles` function.

```c++
void SetCurrentPhase(Phase current_phase);
```

* This sets the current phase of the leg.

```c++
Phase CurrentPhase();
```

* This returns the current phase of the leg.

```c++
void PrintLegPosVel();
```

* This prints the current position and velocity of both axis, as well as if angles are within safe position, to the Arduino serial monitor in one line.
