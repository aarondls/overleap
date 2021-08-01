#ifndef Overleap_h
#define Overleap_h

// Header for Overleap's usage of the ODrive controller through ASCII protocol

// Needed headers 
#include <HardwareSerial.h>
#include <SoftwareSerial.h>
#include <ODriveArduino.h>
#include <ODriveEnums.h>

// Structs

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

// Enums

enum Phase {AERIAL_PHASE = 0, TOUCHDOWN_PHASE};

// Class

class Overleap {
  public:
    // Constructor that takes in the length of leg 0 and 1 in meters, 
    // the output stream for ODrive,
    // and the output stream for the Arduino serial monitor
    Overleap(float length_0, float length_1, Stream& odrive_serial, Stream& arduino_serial, ODriveArduino& odrive);

    // Required to run at startup
    // Both arduino and odrive serial ports must have begun
    // Searches for the index on both encoders, starting with axis 0
    void SearchEncoderIndex();

    // Calculate needed torque on motors 0 and 1 to achieve desired force
    // based on current leg position using transpose of Jacobian
    // sign of force directions are with respect to foot:
    // positive x force to the right
    // positive y force downwards
    // positive torque counterclockwise
    MotorTorquePair CalculateTorques(float f_x, float f_y);

    // Returns true if battery voltage is below 12V
    // Prints the current voltage to the arduino serial monitor
    bool LowBatteryVoltage();

    // Returns true if current leg position is within safe angles
    // Safe angles are between -10 and 75 degrees for theta0,
    // between 0 and 135 degrees for theta 1
    bool WithinSafeAngles();

    // Returns true if torque is in safe direction even if not in safe angles
    // ie, torque will not cause leg to move away from safe angle
    // Useful if leg is not within safe angle, but current torque will bring it towards safe angle
    // Direction of positive torque is based on overleap: 
    // axis 0 positive torque goes clockwise while axis 1 positive torque goes counterclockwise
    bool SafeTorqueDirection(MotorTorquePair torque_pair);

    // Returns true if leg is on the ground based on the foot contact sensor
    bool Touchdown();

    // Updates force profile with:
    // impulse of force_impulse in N ms
    // maximum fx force of fx_max
    // proportion of fy to be applied as fx of fx_proportion
    void UpdateForceProfile(float force_impulse, float fx_max, float fx_proportion);

    // Updates the current phase of the leg based on the foot contact sensor
    void UpdatePhase();

    // Requests the given control mode
    // Timeout is in seconds and is approximate
    // Returns true if sucessful
    bool RequestControlMode(int axis, int requested_control_mode, int timeout=1);

    // Requests the given axis state
    // Timeout is in seconds and is approximate
    // Returns true if sucessful
    bool RequestAxisState(int axis, int requested_axis_state, int timeout=1);

    // Puts both axis into idle state
    // Returns true if successful, false if not
    bool RequestIdleState();

    // Angles must be calibrated before calling
    // Sets control mode to torque and enters closed loop control for both axis
    // Sets initial torque to 0
    // Returns true if successful, false if not
    bool EnterTorqueControl();

    // Must be in torque mode and closed loop control before calling
    // pd_0 is desired position in degree for axis 0, pd_1 is desired position in degree for axis 1
    // end_at_touchdown should be set if need to end position hold when phase updates to touchdown
    // Duration is length of position hold in ms
    // If end_at_touchdown is set, position hold also ends when leg touches the ground
    // end_at_touchdown is also used to determine how "stiff" the position hold is: if ending at touchdown,
    // the gains are less, if not ending at touchdown, then the gains are higher so the leg can support itself
    // Ends by commanding zero torque
    void HoldPosition(float pd_0, float pd_1, bool end_at_touchdown, unsigned long duration=10000);

    // Must be in torque control mode and closed loop control before calling
    // Executes the force profile with the set parameters
    // Stops if foot is no longer on the ground or if force duration is exceeded
    // Ends by commanding zero torque
    void ExecuteForceProfile();

    // Calibrates zero angle position for both angles
    // Upper leg zero angle with respect from the horizontal,
    // lower leg zero angle with respect from (extending) the upper leg
    void CalibrateAngles();

    // Returns true if angles have been calibrated
    bool CalibratedAngles();

    // Sets current_phase
    void SetCurrentPhase(Phase current_phase);

    // Returns the current phase
    Phase CurrentPhase();

    // Prints the current position and velocity of both axis, as well as if angles are within safe position,
    // to the arduino serial monitor in one line
    void PrintLegPosVel();

  private:
    void PrepareForceProfile();

    Stream& odrive_serial_;
    Stream& arduino_serial_;

    ODriveArduino& odrive_;

    float length_0_;
    float length_1_;

    float theta0_zero_position_;
    float theta1_zero_position_;

    bool calibrated_angles_;

    Phase current_phase_;

    ForceProfile force_profile_;

    unsigned long takeoff_time_;
    unsigned long touchdown_time_;
};

#endif