#include <Arduino.h>
#include <Wire.h>

#include "../Header Files/Config.h"
#include "../Header Files/SBUS.h"
#include "../Header Files/Movement.h"
#include "../Header Files/Inverse Kinematics.h"
#include "../Header Files/Servos.h"


//     ___________________________________________________________________ 
//    |     Z                                                             |
//    |    ^                                                              |
//    |    |    Y                                                         |
//    |    |   /                                                          |
//    |    |  /                                                           |
//    |    | /                            Front                           |
//    |    o --------> X                _________                         |
//    |                       1 /------/   /    /------\ 2                |
//    |                        /      /   /    /        \                 |
//    |             Left    3 /------/  0,0   /------\ 4 \    Right       |
//    |                      /      /        /        \                   |
//    |                   5 /------/________/------\ 6 \                  |
//    |                    /          Rear          \                     |
//    |                   /                          \                    |
//    |___________________________________________________________________|

unsigned long timeStamp;                   // keeps track of the last time the main loop was run
bool receiver_Connected = false;  // if "true", the receiver is connected to the transmitter

int input_channels[SBUS_NumberOfChannels] = {0, 0, 0, 0, 0, 0, 0, 0}; // array of all 8 SBUS Channels

int   all_Arm_Lengths         [ArmCount][LengthsPerArm] = { // 2D array of all hexapod arm lengths (A / B), defined in Config.h
                    { ArmLength_Arm1_A,
                      ArmLength_Arm1_B,
                    },                 
                    { ArmLength_Arm2_A,
                      ArmLength_Arm2_B,
                    },                 
                    { ArmLength_Arm3_A,
                      ArmLength_Arm3_B,
                    },                 
                    { ArmLength_Arm4_A,
                      ArmLength_Arm4_B,
                    },                 
                    { ArmLength_Arm5_A,
                      ArmLength_Arm5_B,
                    },                 
                    { ArmLength_Arm6_A,
                      ArmLength_Arm6_B,
                    }};
float all_Arm_Points_1        [ArmCount][XYZ_Axis]      = { // 2D array of 6 Points (X/Y/Z) that lie at the shoulder joints
                   { initial_Arm1_P1_X,
                     initial_Arm1_P1_Y,
                     initial_Arm1_P1_Z 
                   },                  
                   { initial_Arm2_P1_X,
                     initial_Arm2_P1_Y,
                     initial_Arm2_P1_Z 
                   },                  
                   { initial_Arm3_P1_X,
                     initial_Arm3_P1_Y,
                     initial_Arm3_P1_Z 
                   },                  
                   { initial_Arm4_P1_X,
                     initial_Arm4_P1_Y,
                     initial_Arm4_P1_Z 
                   },                  
                   { initial_Arm5_P1_X,
                     initial_Arm5_P1_Y,
                     initial_Arm5_P1_Z 
                   },                  
                   { initial_Arm6_P1_X,
                     initial_Arm6_P1_Y,
                     initial_Arm6_P1_Z 
                   }};
float all_Arm_Points_2        [ArmCount][XYZ_Axis]      = { // 2D array of 6 Points (X/Y/Z) that lie at the tips of the arms
                   { Arm1_CircleCenter_X,
                     Arm1_CircleCenter_Y,
                     0
                   },                  
                   { Arm2_CircleCenter_X,
                     Arm2_CircleCenter_Y,
                     0
                   },                  
                   { Arm3_CircleCenter_X,
                     Arm3_CircleCenter_Y,
                     0
                   },                  
                   { Arm4_CircleCenter_X,
                     Arm4_CircleCenter_Y,
                     0
                   },                  
                   { Arm5_CircleCenter_X,
                     Arm5_CircleCenter_Y,
                     0
                   },                  
                   { Arm6_CircleCenter_X,
                     Arm6_CircleCenter_Y,
                     0
                   }};
float all_Arm_Points_2_rotated[ArmCount][XYZ_Axis]      = { // 2D array of 6 Points (X/Y/Z) that lie at the tips of the arms, after the main chassis got rotated/translated
                   { Arm1_CircleCenter_X,
                     Arm1_CircleCenter_Y,
                     0
                   },                  
                   { Arm2_CircleCenter_X,
                     Arm2_CircleCenter_Y,
                     0
                   },                  
                   { Arm3_CircleCenter_X,
                     Arm3_CircleCenter_Y,
                     0
                   },                  
                   { Arm4_CircleCenter_X,
                     Arm4_CircleCenter_Y,
                     0
                   },                  
                   { Arm5_CircleCenter_X,
                     Arm5_CircleCenter_Y,
                     0
                   },                  
                   { Arm6_CircleCenter_X,
                     Arm6_CircleCenter_Y,
                     0
                   }};
float all_Arm_Angles          [ArmCount][AnglesPerArm];     // 2D array of all servo angles (alpha/beta/gamma), for all arms


void setup() {
  // put your setup code here, to run once:
  Serial.begin(SBUS_Baud, SERIAL_8E2);    // set the Serial Port to 100'000 BAUD, with 8 data bits, 2 stop bits, and even parity
  TWBR = 24;                              //Two Wire Baud Rate 400kHz I2C clock
  Serial.println("reset");

  while (!receiver_Connected)             // wait until the receiver is connected, by waiting for SBUS packets and checking the return value
  {
    while(Serial.available() < SBUS_PacketSize);  
    receiver_Connected = SBUS_Process_Packet(input_channels);
  }
  
  all_Arms_inverse_Kinematics(*all_Arm_Angles, *all_Arm_Points_1, *all_Arm_Points_2_rotated, *all_Arm_Lengths); // calculate the servo angles for all arms

  init_Servos(*all_Arm_Angles);           // initialize all servos, and then, one after the other, set them to the previously calculated angles
  timeStamp = millis();
}



void loop() {
  // put your main code here, to run repeatedly:
  if (millis() - timeStamp > loopPeriod) digitalWrite(LED_BUILTIN, true );  // checks if the main loop is taking longer than the loopPeriod
  else                                   digitalWrite(LED_BUILTIN, false);  // and if it is longer, it turns on the builtin LED

  while ((millis() - timeStamp) < loopPeriod);  // waits, until the time since the last loop was run is >= than the loopPeriod
  timeStamp += loopPeriod;  //updates the time since the loop was run

  //Receive Transmitter Data
  if (Serial.available() >= SBUS_PacketSize)  // checks if there is a SBUS packet available, if so, it reads that packet
  {
    receiver_Connected = SBUS_Process_Packet(input_channels);
  }


  //calculate Movement
  move_Gate(input_channels, *all_Arm_Points_2, false);
  move_Arms_Towards_Target(*all_Arm_Points_2);   // slowy moves the tips of the arms towards the previously calculated target points
  move_main_Chassis(input_channels, *all_Arm_Points_1, *all_Arm_Points_2, *all_Arm_Points_2_rotated, 0.1);  // rotates / translates the main chassis based on transmitter input


  //Calculate Servo Angles
  all_Arms_inverse_Kinematics(*all_Arm_Angles, *all_Arm_Points_1, *all_Arm_Points_2_rotated, *all_Arm_Lengths); // calculate the servo angles for all arms


  //Update Servos
  Update_Servo_Positions(*all_Arm_Angles);  //sets all of the servos to the previously calculated angles
}