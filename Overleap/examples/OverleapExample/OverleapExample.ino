// Required headers
#include "Overleap.h"
#include <HardwareSerial.h>
#include <SoftwareSerial.h>
#include <ODriveArduino.h>
#include <ODriveEnums.h>

HardwareSerial& odrive_serial = Serial1;
ODriveArduino odrive(odrive_serial);

// Declare overleap leg
Overleap leg(0.19f, 0.22f, odrive_serial, Serial, odrive);

void setup() {
  // 9600 baud rate
  delay(1000);
  Serial.begin(9600);
  while (!Serial); // wait for Arduino Serial Monitor
  
  delay(1000);
  // ODrive uses 115200 baud
  odrive_serial.begin(115200);

  delay(1000);

  // Search for encoder index on both axis
  leg.SearchEncoderIndex();

  Serial.println("OverleapOS loaded.");
  Serial.println("c to calibrate angles");
  Serial.println("t to test torque calculations");
  Serial.println("d to show current position in degrees and current velocity");
  Serial.println("h to hold current position with direct torque control");
  Serial.println("p for hold aerial position using PD loop");
  Serial.println("j to perform a single jump");
  Serial.println("g for go perform multiple jumps in circular direction");
  Serial.println("y to check positive direction of torque");
  Serial.println("i for idle");
  Serial.println("r for clearing input");

  // Check battery voltage
  if (leg.LowBatteryVoltage()) {
    Serial.println("Warning: Low battery voltage");
  }

  // Check if anything wrong; remove later
  Serial.println(odrive.readFloat());
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    Serial.print("Read ");
    Serial.println(c);

    if (c == 'c') {
      leg.CalibrateAngles();
    }

    if (c == 't') {
        if (!leg.CalibratedAngles()) {
          Serial.println("Calibrate first.");
          return;
        }

        Serial.println("Calculating torques at current joint positions.");

        float force_x = 0, force_y = 0;

        const unsigned long duration = 30000;
        unsigned long start = millis();

        char response;
        while (millis() - start < duration) {
            if (Serial.available()) {
                response = Serial.read();
                if (response == 'e') {
                    Serial.println("Ending");
                    break;
                }
                if (response == 'n') {
                    // flush useless information
                    if (Serial.available()) Serial.read();
                    
                    Serial.print("Enter new force x value: ");
                    String new_fx = "";
                    while (!new_fx.length()) {
                        while (!Serial.available()) {
                            delay(100);
                        }
                        new_fx = Serial.readString();
                        Serial.print("Read line as ");
                        Serial.println(new_fx);
                    }
                    force_x = new_fx.toFloat();

                    Serial.print("Enter new force y value: ");

                    String new_fy = "";
                    while (!new_fy.length()) {
                        while (!Serial.available()) {
                            delay(100);
                        }
                        new_fy = Serial.readString();
                        Serial.print("Read line as ");
                        Serial.println(new_fy);
                    }
                    force_y = new_fy.toFloat();

                    Serial.print("Force x: ");
                    Serial.println(force_x);
                    Serial.print("Force y: ");
                    Serial.println(force_y);
                    delay(2000);
                }
            }

            MotorTorquePair torque = leg.CalculateTorques(force_x, force_y);

            Serial.print("Torque 0: ");
            Serial.println(-torque.motor_0);
            Serial.print("Torque 1: ");
            Serial.println(torque.motor_1);

            delay(100);
        }

        Serial.println("Ending torque test");
    }

    if (c == 'd') {
        const unsigned long duration = 2000;
        unsigned long start = millis();

        while (millis() - start < duration) {
            // Print leg position, velocity, and whether position is within safe limits
            leg.PrintLegPosVel();
        }

    }

    if (c == 'h') {
        Serial.println("Attempting to hold position with direct torque control.");
        Serial.println("Note this uses no PD control loop; the torque values are calculated directly to apply the needed vertical force to remain stationary at the current leg position");

        if (!leg.EnterTorqueControl()) {
          Serial.println("Aborting.");
          return;
        }

        // clear and check for possible errors anywhere
        float test = odrive.readFloat();
        Serial.print("Read test value: ");
        Serial.println(test);

        // forces
        float f_x = 0, f_y = 2;

        const unsigned long duration = 3000;
        unsigned long start = millis();

        while (millis() - start < duration) {
            MotorTorquePair torque = leg.CalculateTorques(f_x, f_y);

            // invert axis 0 since CalculateTorques follows positive torque is CCW
            // while axis 0 is positive torque goes CW
            torque.motor_0 = -torque.motor_0;

            Serial.print("Torque 0: ");
            Serial.print(torque.motor_0);
            Serial.print(" Torque 1: ");
            Serial.print(torque.motor_1);
            Serial.print(" Touchdown: ");
            Serial.println(leg.Touchdown());
            
            if (leg.SafeTorqueDirection(torque)) {
              odrive_serial.print("c 0 ");
              odrive_serial.print(torque.motor_0);
              odrive_serial.print("\n");
              delay(5);
              odrive_serial.print("c 1 ");
              odrive_serial.print(torque.motor_1);
              odrive_serial.print("\n");
            } else {
              Serial.println("Angle limits reached and torques will exceed limit.");
              // command zero torque
              odrive_serial.print("c 0 0");
              odrive_serial.print("\n");
              delay(5);
              odrive_serial.print("c 1 0");
              odrive_serial.print("\n");
              break;
            }
        }

        Serial.println("Ending hold position");
        // command zero torque
        odrive_serial.print("c 0 0");
        odrive_serial.print("\n");
        odrive_serial.print("c 1 0");
        odrive_serial.print("\n");

        delay(100);
        
        leg.RequestIdleState();
    }

    if (c == 'p') {
      Serial.println("Attempting to hold aerial position with PD control loop on torque.");

      if (!leg.EnterTorqueControl()) {
        Serial.println("Aborting.");
        return;
      }

      // clear and check for possible errors
      float test = odrive.readFloat();
      Serial.print("Read test value: ");
      Serial.println(test);

      // test aerial position that waits for touchdown
      // set phase to aerial
      leg.SetCurrentPhase(AERIAL_PHASE);
      // leg.HoldPosition(25.0f, 110.0f, true);
      leg.SetCurrentPhase(TOUCHDOWN_PHASE);

      // test aerial position that does not wait for touchdown
      // this should be firm enough to support leg
      // leg.HoldPosition(45.0f, 90.0f, false, 3000); 
      // the test dip down
      // leg.HoldPosition(25.0f, 110.0f, false, 400);
      // go back up
      leg.HoldPosition(45.0f, 90.0f, false, 7000); 

      delay(100);

      leg.RequestIdleState();
    }

    if (c == 'j') {
      Serial.println("Executing a single jump.");

      // set phase to ground
      leg.SetCurrentPhase(TOUCHDOWN_PHASE);

      if (!leg.EnterTorqueControl()) {
        Serial.println("Aborting.");
        return;
      }

      // Set the impulse provided by the force profile, the maximum fx force and proportion of fy applied as fx
      // This is needed to be called before ExecuteForceProfile
      // Since a single jump is desired to go straight up, fx_max is set to 0
      leg.UpdateForceProfile(2500.0f, 0, 0.4f);
      
      // prepare to jump stance
      leg.HoldPosition(25.0f, 110.0f, false, 3000);

      // execute single force profile
      leg.ExecuteForceProfile();
      
      // tuck leg in position to avoid ground and be ready for next jump
      // ends if either time runs out or if touchdown occurs
      // next force profile can execute if expected to touchdown soon, even without currently touching ground
      leg.HoldPosition(25.0f, 110.0f, true, 800);

      // and we go to raised stance with ample time to secure leg before powering down
      leg.HoldPosition(45.0f, 90.0f, false, 3000);

      delay(100);
        
      leg.RequestIdleState();
    }

    if (c == 'g') {
      Serial.println("Go for executing multiple jumps.");

      // Set starting phase to ground
      leg.SetCurrentPhase(TOUCHDOWN_PHASE);

      if (!leg.EnterTorqueControl()) {
        Serial.println("Aborting.");
        return;
      }

      // do the initial jump
      
      // update force profile
      leg.UpdateForceProfile(2200.0f, 10.0f, 0.4f);
      
      // prepare to jump stance
      leg.HoldPosition(25.0f, 110.0f, false, 3000);

      // execute single force profile
      leg.ExecuteForceProfile();

      // tuck leg in position to avoid ground and be ready for next jump
      // ends if either time runs out or if touchdown occurs
      // next force profile can execute if expected to touchdown soon, even without currently touching ground
      leg.HoldPosition(25.0f, 110.0f, true, 800);

      // after the initial jump, we can now do multiple consecutive jumps
      // start by updating the force profile to account for downward momentum
      // this value is empirical, but can be changed to account for takeoff and touchdown times
      leg.UpdateForceProfile(2600.0f, 10.0f, 0.4f);
      
      for (int jump=1; jump<10; jump++) {
        // execute single force profile
        leg.ExecuteForceProfile();
        // ready for landing and end at touchdown or 800ms
        leg.HoldPosition(25.0f, 110.0f, true, 800);
      }
      
      // Put into raised stance to secure leg before powering down
      leg.HoldPosition(35.0f, 100.0f, false, 4000);

      delay(100);
        
      leg.RequestIdleState();
    }

    if (c == 'y') {
      Serial.println("Torque direction check.");

      if (!leg.EnterTorqueControl()) {
        Serial.println("Aborting.");
        return;
      }

      for (int axis=0; axis<=1; axis++) {
        Serial.print("Type 'r' when ready for axis "); Serial.print(axis); Serial.print(": ");
        char read_char = 'a';
        while (read_char != 'r') {
            while (!Serial.available()) {
                delay(100);
            }
            read_char = Serial.read();
        }

        const unsigned long duration = 1000;
        unsigned long start = millis();
        float torque = 0.02;

        // delay by a bit
        delay(200);

        while (millis() - start < duration) {
          odrive_serial.print("c "); 
          odrive_serial.print(axis); 
          odrive_serial.print(" ");
          odrive_serial.print(torque,3); 
          odrive_serial.print("\n");
        }

        // command zero torque
        odrive_serial.print("c "); 
        odrive_serial.print(axis);
        odrive_serial.print(" 0"); 
        odrive_serial.print("\n");
      }

      Serial.println("Ending torque direction check.");

      delay(100);
        
      leg.RequestIdleState();
    }

    if (c == 'i') {
      leg.RequestIdleState();
    }

    if (c == 'r') {
      Serial.println(odrive.readFloat());
    }
    
  }
}