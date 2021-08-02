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
  Serial.println("d to show current position in degrees and current velocity");
  Serial.println("j to perform a single jump");
  Serial.println("g for go perform multiple jumps in circular direction");
  Serial.println("y to check positive direction of torque");
  Serial.println("i for idle");

  // Check battery voltage
  if (leg.LowBatteryVoltage()) {
    Serial.println("Warning: Low battery voltage");
  }
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    Serial.print("Read ");
    Serial.println(c);

    if (c == 'c') {
      leg.CalibrateAngles();
    }

    if (c == 'd') {
        const unsigned long duration = 2000;
        unsigned long start = millis();

        while (millis() - start < duration) {
            // Print leg position, velocity, and whether position is within safe limits
            leg.PrintLegPosVel();
        }

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
  }
}