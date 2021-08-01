#include "Overleap.h"

Overleap::Overleap(float length_0, float length_1, Stream& odrive_serial, Stream& arduino_serial, ODriveArduino& odrive) : odrive_serial_(odrive_serial), arduino_serial_(arduino_serial), odrive_(odrive) {
    length_0_ = length_0;
    length_1_ = length_1;

    // initialize to 0
    theta0_zero_position_ = 0;
    theta1_zero_position_ = 0;

    // initialize to false
    calibrated_angles_ = false;

    // initialize to ground
    current_phase_ = TOUCHDOWN_PHASE;

    // set force duration
    PrepareForceProfile();
}

void Overleap::SearchEncoderIndex() {
    int requested_state = AXIS_STATE_ENCODER_INDEX_SEARCH;
    for (int motor = 0; motor<=1; motor++) {
        arduino_serial_.print("Searching for index on motor ");
        arduino_serial_.println(motor);
        if(!odrive_.run_state(motor, requested_state, true, 5.0f)) {
            arduino_serial_.println("Unsuccessful. Ending.");
            break;
        }
        arduino_serial_.print("Sucessfully found index for motor ");
        arduino_serial_.println(motor);
        arduino_serial_.println(odrive_.readFloat());
        arduino_serial_.println(odrive_.readFloat());
    }
}

MotorTorquePair Overleap::CalculateTorques(float f_x, float f_y) {
    MotorTorquePair torque_pair = {0.0f,0.0f};
    
    // Ensure positive and negative angle direction is correct
    // Gear ratio is 9:1

    // This is in radians
    float axis0_position, axis1_position;

    axis0_position = odrive_.GetPosition(0);
    float theta_0 = ((axis0_position-theta0_zero_position_)/9.0f) * 2.0f * PI;

    axis1_position = odrive_.GetPosition(1);
    float theta_1 = ((theta1_zero_position_-axis1_position)/9.0f) * 2.0f * PI;

    // use Jacobian
    // In this calculation, positive fx is to the right, negative fx is to the left
    // positive fy is downwards, negative fy is upwards
    // May be changed to sinf if desired
    // torque 0
    torque_pair.motor_0 = ( (length_0_ * sin(theta_0) + (length_1_ * sin(theta_0+theta_1))) * f_x) + ( ((-length_0_ * cos(theta_0)) + (-length_1_ * cos(theta_0+theta_1))) * f_y);
    // torque 1
    torque_pair.motor_1 = (length_1_ * sin(theta_0+theta_1) * f_x) + (-length_1_ * cos(theta_0+theta_1) * f_y);

    // consider gear ratio
    torque_pair.motor_0 /= 9.0f;
    torque_pair.motor_1 /= 9.0f;

    return torque_pair;
}

bool Overleap::LowBatteryVoltage() {
  odrive_serial_.print("r vbus_voltage\n");
  float voltage = odrive_.readFloat();
  arduino_serial_.print("Current battery voltage is: ");
  arduino_serial_.println(voltage);
  return (voltage<12); 
}


bool Overleap::WithinSafeAngles() {
  // This is in degrees
  // This depends on motor orientation
  float theta_0 = ((odrive_.GetPosition(0)-theta0_zero_position_)/9.0f) * 360.0f;
  float theta_1 = ((theta1_zero_position_-odrive_.GetPosition(1))/9.0f) * 360.0f;

  if (theta_0 < -10 || theta_0 > 75) return false;
  if (theta_1 < 0 || theta_1 > 135) return false;

  return true;    
}

bool Overleap::SafeTorqueDirection(MotorTorquePair torque_pair) {
  // for axis 0, positive torque and velocity clockwise
  // for axis 1, positive torque and velocity counterclockwise
  float theta_0 = ((odrive_.GetPosition(0)-theta0_zero_position_)/9.0f) * 360.0f;
  float theta_1 = ((theta1_zero_position_-odrive_.GetPosition(1))/9.0f) * 360.0f;
  
  if (theta_0 < -10 && torque_pair.motor_0 < 0) {
    // Upper leg is too far counterclockwise, so torque cannot be counterclockwise
    return false;
  }
  
  if (theta_0 > 75 && torque_pair.motor_0 > 0) {
    // Upper leg is too far clockwise, so torque cannot be clockwise 
    return false;
  }

  if (theta_1 < 0 && torque_pair.motor_1 > 0) {
    // Lower leg is too far counterclockwise, so torque cannot be counterclockwise
    return false;
  }
  
  if (theta_1 > 135 && torque_pair.motor_1 < 0) {
    // Lower leg is too far clockwise, so torque cannot be clockwise 
    return false;
  }

  return true;
}

bool Overleap::Touchdown() {
    int FSR_value = analogRead(0);
    // adjust based on sensor
    return (FSR_value > 1);
}

void Overleap::PrepareForceProfile() {
    // we will define the force duration to be 200ms, so b=1/duration=1/250=0.005
    // note force profile is in ms
    force_profile_.duration = 200.0f;
    force_profile_.b = 0.005f;

    // set all other values to zero to protect against executing random force profile inadvertently
    force_profile_.a = 0;
    force_profile_.fx_proportion = 0;
    force_profile_.max_fx_force = 0;
}

void Overleap::UpdateForceProfile(float force_impulse, float fx_max, float fx_proportion) {
    // Use constant profile by default
    // Another way is to take into consideration takeoff and touchdown times
  
    arduino_serial_.println("Updating force profile.");

    // We will define a constant force impulse in N ms
    // this is special case for 200ms force duration; found through intgerating the force profile for one duration
    force_profile_.a = force_impulse/100.0f;

    force_profile_.max_fx_force = fx_max;
    force_profile_.fx_proportion = fx_proportion;
}

void Overleap::UpdatePhase() {
    // here, "current" phase is the "last" phase, until it is updated
    // require "two" touchdowns to avoid error
    if (Touchdown() && Touchdown()) {
        /*
        if (current_phase == AERIAL_PHASE) { // this is still the last phase
            // last in air
            // for now, our force profile is constant
            // UpdateForceProfile();
        }
        */
        // set phase to touchdown
        current_phase_ = TOUCHDOWN_PHASE;
    } else {
        /*
        if (current_phase == TOUCHDOWN_PHASE) {
            // set takeoff time
            // for now, no need
        }
        */
        current_phase_ = AERIAL_PHASE;
    }
}

bool Overleap::RequestControlMode(int axis, int requested_control_mode, int timeout) {
    int timeout_counter = timeout * 10;

    odrive_serial_.print("w axis");
    odrive_serial_.print(axis);
    odrive_serial_.print(".controller.config.control_mode ");
    odrive_serial_.print(requested_control_mode);
    odrive_serial_.print("\n");

    do {
        delay(100);
        odrive_serial_.print("r axis");
        odrive_serial_.print(axis);
        odrive_serial_.print(".controller.config.control_mode");
        odrive_serial_.print("\n");
        timeout_counter--;
    } while (timeout_counter > 0 && odrive_.readInt() != requested_control_mode);

    return (timeout_counter > 0);
}

bool Overleap::RequestAxisState(int axis, int requested_axis_state, int timeout) {
  int timeout_counter = timeout * 10;
  
  odrive_serial_.print("w axis");
  odrive_serial_.print(axis);
  odrive_serial_.print(".requested_state ");
  odrive_serial_.print(requested_axis_state);
  odrive_serial_.print("\n");

  do {
    delay(100);
    odrive_serial_.print("r axis");
    odrive_serial_.print(axis);
    odrive_serial_.print(".current_state");
    odrive_serial_.print("\n");
    timeout_counter--;
  } while (timeout_counter > 0 && odrive_.readInt() != requested_axis_state);
  
  return (timeout_counter > 0);
}


bool Overleap::RequestIdleState() {
    arduino_serial_.println("Axis0: Requesting idle state.");
    if(!RequestAxisState(0, AXIS_STATE_IDLE, 3)) {
        arduino_serial_.println("Failed to enter idle state for axis 0.");
        return false;
    }

    arduino_serial_.println("Axis1: Requesting idle state.");
    if(!RequestAxisState(1, AXIS_STATE_IDLE, 3)) {
        arduino_serial_.println("Failed to enter idle state for axis 1.");
        return false;
    }
    return true;
}

bool Overleap::EnterTorqueControl() {
    // check if calibrated
    if (!calibrated_angles_) {
        arduino_serial_.println("Calibrate angles first.");
        return false;
    }

    int requested_control_mode = CONTROL_MODE_TORQUE_CONTROL;

    arduino_serial_.println("Axis0: Requesting torque control mode.");
    if (!RequestControlMode(0, requested_control_mode, 3)) {
        arduino_serial_.println("Axis0 unsuccessful in requesting torque control mode.");
        return false;
    }
    delay(5);

    arduino_serial_.println("Axis1: Requesting torque control mode.");
    if (!RequestControlMode(1, requested_control_mode, 3)) {
        arduino_serial_.println("Axis1 unsuccessful in requesting torque control mode.");
        return false;
    }

    arduino_serial_.println("Sucessfully entered torque control mode for both axis.");
    delay(5);

    int requested_axis_state = AXIS_STATE_CLOSED_LOOP_CONTROL;

    arduino_serial_.println("Axis0: Requesting closed loop control.");
    if (!RequestAxisState(0, requested_axis_state, 3)) {
        arduino_serial_.println("Fail to enter closed loop control for axis 0. Reverting to idle state.");
        if(!RequestAxisState(0, AXIS_STATE_IDLE, 3)) {
        arduino_serial_.println("Failed to enter idle state for axis 0.");
        }
        return false;
    }

    arduino_serial_.println("Axis1: Requesting closed loop control.");
    if (!RequestAxisState(1, requested_axis_state, 3)) {
        arduino_serial_.println("Fail to enter closed loop control for axis 1. Reverting to idle state.");
        if(!RequestAxisState(0, AXIS_STATE_IDLE, 3)) {
        arduino_serial_.println("Failed to enter idle state for axis 0.");
        }
        if(!RequestAxisState(1, AXIS_STATE_IDLE, 3)) {
        arduino_serial_.println("Failed to enter idle state for axis 1.");
        }
        return false;
    }

    arduino_serial_.println("Successfully entered closed loop control.");

    // This ensures the torque isnt some random value
    odrive_serial_.print("c 0 0");
    odrive_serial_.print("\n");
    delay(5);
    odrive_serial_.print("c 1 0");
    odrive_serial_.print("\n");
    delay(5);

    return true;
}

void Overleap::HoldPosition(float pd_0, float pd_1, bool end_at_touchdown, unsigned long duration) {
    // torques
    MotorTorquePair torque_pair = {0, 0};

    // current position
    float p_0, p_1;

    // current velocity
    float v_0, v_1;
    
    // gain values
    float kp_0, kd_0;
    float kp_1, kd_1;
    // velocity is so delayed when controlling with uart
    float slow_kp_0 = 0, slow_kp_1 = 0;

    if (end_at_touchdown) {
        kp_0 = 0.005f;
        kd_0 = 0.0001f;
        kp_1 = 0.006f;
        kd_1 = 0.0001f; 
    } else {
        // stiffer
        kp_0 = 0.009f;
        kd_0 = 0.0001f;
        kp_1 = 0.004f;
        kd_1 = 0.0001f;
    }

    unsigned long start = millis();

    while (millis() - start < duration) {
        odrive_serial_.print("f 0\n");
        p_0 = odrive_.readFloat(); // this is in turns of motor
        // simplify p_0 = ((p_0-theta0_zero_position)/9.0f) * 360.0f; 
        p_0 = (p_0-theta0_zero_position_) * 40.0f; // this is in degrees of output
        // simplify v_0 = (odrive_.readFloat()/9.0f) * 360.0f;
        v_0 = odrive_.readFloat() * 40.0f; // in degrees/sec of output

        odrive_serial_.print("f 1\n");
        p_1 = odrive_.readFloat(); 
        p_1 = ((theta1_zero_position_-p_1)) * 40.0f;
        v_1 = odrive_.readFloat() * 40.0f;

        // for axis 0, positive torque and velocity clockwise
        // for axis 1, positive torque and velocity counterclockwise
        
        // slowly increase kp
        if (millis() - start < 400) {
            slow_kp_0 = ((millis() - start)/400.0f)*kp_0;
            slow_kp_1 = ((millis() - start)/400.0f)*kp_1;
        } else {
            slow_kp_0 = kp_0;
            slow_kp_1 = kp_1;
        }
        
        torque_pair.motor_0 = -slow_kp_0*(p_0-pd_0) + kd_0*(-v_0);
        torque_pair.motor_1 = slow_kp_1*(p_1-pd_1) + kd_1*(-v_1);
        
        // Debugging purposes
        // arduino_serial_.print("Pos 0: ");
        // arduino_serial_.print(p_0,5);
        // arduino_serial_.print(" Pos 1: ");
        // arduino_serial_.println(p_1,5);

        // check direction of torque if safe 
        if (SafeTorqueDirection(torque_pair)) {
            odrive_serial_.print("c 0 ");
            odrive_serial_.print(torque_pair.motor_0,5);
            odrive_serial_.print("\n");
            odrive_serial_.print("c 1 ");
            odrive_serial_.print(torque_pair.motor_1,5);
            odrive_serial_.print("\n");
        } else {
            arduino_serial_.println("Angle limits reached.");
            // command zero torque
            odrive_serial_.print("c 0 0");
            odrive_serial_.print("\n");
            odrive_serial_.print("c 1 0");
            odrive_serial_.print("\n");
            break;
        }
        UpdatePhase();
        if (end_at_touchdown && current_phase_ == TOUCHDOWN_PHASE) {
            arduino_serial_.println("Touchdown");
            break;
        }
    }

    arduino_serial_.println("Ending hold position");
    // command zero torque
    odrive_serial_.print("c 0 0");
    odrive_serial_.print("\n");
    odrive_serial_.print("c 1 0");
    odrive_serial_.print("\n");
}


void Overleap::ExecuteForceProfile() {
    arduino_serial_.println("Executing force profile");
    
    // forces
    float f_x, f_y;

    // force profile values
    float a = force_profile_.a;
    float b = force_profile_.b;
    float f_xmax = force_profile_.max_fx_force;
    float f_x_proportion = force_profile_.fx_proportion;

    unsigned long duration = force_profile_.duration;
    
    unsigned long start = millis();
    unsigned long current_duration = 0;
    
    while (current_duration < duration) {
        // execute force profile
        arduino_serial_.print("Cur dur = ");
        arduino_serial_.print(current_duration);
        
        /*
        f_y = b * PI * current_duration;
        arduino_serial_.print(" Fy = ");
        arduino_serial_.print(f_y,5);
        f_y = sin(b * PI * current_duration);
        arduino_serial_.print(" Fy after sin = ");
        arduino_serial_.print(f_y,5);
        f_y *= f_y;
        arduino_serial_.print(" Fy after mult = ");
        arduino_serial_.print(f_y,5);
        f_y *= a;
        arduino_serial_.print(" Fy after a = ");
        arduino_serial_.print(f_y,5);
        */

        f_y = a * (pow(sin(b * PI * current_duration), 2));
        arduino_serial_.print("Fy = ");
        arduino_serial_.print(f_y,5);

        // consider friction by applying a set percent of fy as fx to avoid foot slipping
        f_x = f_x_proportion * f_y;

        if (f_x > f_xmax) {
            f_x = f_xmax;
        }

        arduino_serial_.print(" Fx = ");
        arduino_serial_.print(f_x,5);

        // see if sensor registers touchdown
        arduino_serial_.print(" Touchdown: ");
        arduino_serial_.println(Touchdown());
        

        MotorTorquePair torque = CalculateTorques(f_x, f_y);
        // output here is positive torque is counterclockwise, so let us reverse for axis 0
        torque.motor_0 = -torque.motor_0;

        // check angles 
        if (SafeTorqueDirection(torque)) {
            odrive_serial_.print("c 0 ");
            odrive_serial_.print(torque.motor_0,6);
            odrive_serial_.print("\n");
            delay(5);
            odrive_serial_.print("c 1 ");
            odrive_serial_.print(torque.motor_1,6);
            odrive_serial_.print("\n");
            } else {
            arduino_serial_.println("Angle limits reached.");
            // command zero torque
            odrive_serial_.print("c 0 0");
            odrive_serial_.print("\n");
            odrive_serial_.print("c 1 0");
            odrive_serial_.print("\n");
            break;
        }
        
        UpdatePhase();
        // wait for few ms since touchdown sensor does not trigger at low forces and also so
        // force profile can be applied even when not in touchdown
        // within safe torque direction protects force application when leg goes beyond safe limits
        if (current_phase_ == AERIAL_PHASE && current_duration > 400) {
            arduino_serial_.println("Takeoff");
            break;
        }
        current_duration = millis()-start;
    }

    arduino_serial_.println("Ending force profile execution");
    
    // command zero torque
    odrive_serial_.print("c 0 0");
    odrive_serial_.print("\n");
    odrive_serial_.print("c 1 0");
    odrive_serial_.print("\n"); 
}

void Overleap::CalibrateAngles() {
    arduino_serial_.println("Position the upper leg to 0 degrees with respect to the horizontal.");
    arduino_serial_.println("Type 'c' when positioned: ");
    char read_char = 'a';
    while (read_char != 'c') {
        while (!arduino_serial_.available()) {
            delay(100);
        }
        read_char = arduino_serial_.read();
    }

    arduino_serial_.println(odrive_.GetPosition(0));

    theta0_zero_position_ = odrive_.GetPosition(0);
    arduino_serial_.print("Read position as ");
    arduino_serial_.println(theta0_zero_position_);

    arduino_serial_.print("Using position ");
    arduino_serial_.print(theta0_zero_position_);
    arduino_serial_.println(" as zero position.");

    arduino_serial_.print("Sucessfully calibrated zero degree for upper leg. For reference, current position is: ");
    arduino_serial_.println(odrive_.GetPosition(0));

    arduino_serial_.println("Position the lower leg to 0 degrees with respect to the upper leg.");
    arduino_serial_.println("Type any character when positioned");
    read_char = 'a';
    while (read_char != 'c') {
        while (!arduino_serial_.available()) {
            delay(100);
        }
        read_char = arduino_serial_.read();
    }
    theta1_zero_position_ = odrive_.GetPosition(1);
    arduino_serial_.print("Using position ");
    arduino_serial_.print(theta1_zero_position_);
    arduino_serial_.println(" as zero position.");

    arduino_serial_.print("Sucessfully calibrated zero degree for lower leg. For reference, current position is: ");
    arduino_serial_.println(odrive_.GetPosition(1));

    arduino_serial_.println("Calibration sucessful for both axis.");
    calibrated_angles_ = true;
}

bool Overleap::CalibratedAngles() {
    return calibrated_angles_;
}

void Overleap::SetCurrentPhase(Phase current_phase) {
    current_phase_ = current_phase;
}

Phase Overleap::CurrentPhase() {
    return current_phase_;
}

void Overleap::PrintLegPosVel() {
    // note odrive position is reported in turns
    // p is in degrees, v is in degrees/sec;
    float p_0, v_0, p_1, v_1;

    odrive_serial_.print("f 0\n");
    p_0 = (odrive_.readFloat()-theta0_zero_position_) * 40.0f; // this is in degrees of output
    v_0 = odrive_.readFloat() * 40.0f; // in degrees/sec of output

    odrive_serial_.print("f 1\n");
    p_1 = (theta1_zero_position_-odrive_.readFloat()) * 40.0f; 
    v_1 = odrive_.readFloat() * 40.0f; 

    arduino_serial_.print("Axis 0 1 in deg: ");
    arduino_serial_.print(p_0);
    arduino_serial_.print(" ");
    arduino_serial_.print(p_1);
    arduino_serial_.print(" Vel in deg/s: ");
    arduino_serial_.print(v_0);
    arduino_serial_.print(" ");
    arduino_serial_.print(v_1);
    arduino_serial_.print(" Safe angles: ");
    arduino_serial_.println(WithinSafeAngles());
}