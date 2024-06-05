#include <Servo.h>
#include <Adafruit_PWMServoDriver.h>

#include "../Header Files/Config.h"
#include "../Header Files/Servos.h"

Adafruit_PWMServoDriver pwm_Board = Adafruit_PWMServoDriver();
Servo Servo__18;
Servo Servo__17;

void init_Servos(float* Angle_Array)
{
  pwm_Board.begin();

  pwm_Board.setOscillatorFrequency(27000000);
  pwm_Board.setPWMFreq(servoFrequency);

  Wire.begin();
  Wire.setClock(400000);

  //disable all servos
  pwm_Board.writeMicroseconds( 0,0);
  pwm_Board.writeMicroseconds( 1,0);
  pwm_Board.writeMicroseconds( 2,0);
  pwm_Board.writeMicroseconds( 3,0);
  pwm_Board.writeMicroseconds( 4,0);
  pwm_Board.writeMicroseconds( 5,0);
  pwm_Board.writeMicroseconds( 6,0);
  pwm_Board.writeMicroseconds( 7,0);
  pwm_Board.writeMicroseconds( 8,0);
  pwm_Board.writeMicroseconds( 9,0);
  pwm_Board.writeMicroseconds(10,0);
  pwm_Board.writeMicroseconds(11,0);
  pwm_Board.writeMicroseconds(12,0);
  pwm_Board.writeMicroseconds(13,0);
  pwm_Board.writeMicroseconds(14,0);
  pwm_Board.writeMicroseconds(15,0);

  //set servos to calculated angles in the order of (all betas, all alphas, all gammas)(arms 1 to 6)
  pwm_Board.writeMicroseconds( 2,((Angle_Array[servo__2_ID] + servo__2_Offset) * servo__2_Direction - ServoAngle_Min) * (PPM_Max - PPM_Min) / (ServoAngle_Max - ServoAngle_Min) + PPM_Min);
  delay(delayPerServo);
  pwm_Board.writeMicroseconds( 3,((Angle_Array[servo__3_ID] + servo__3_Offset) * servo__3_Direction - ServoAngle_Min) * (PPM_Max - PPM_Min) / (ServoAngle_Max - ServoAngle_Min) + PPM_Min);
  delay(delayPerServo);
  pwm_Board.writeMicroseconds( 8,((Angle_Array[servo__8_ID] + servo__8_Offset) * servo__8_Direction - ServoAngle_Min) * (PPM_Max - PPM_Min) / (ServoAngle_Max - ServoAngle_Min) + PPM_Min);
  delay(delayPerServo);
  pwm_Board.writeMicroseconds( 9,((Angle_Array[servo__9_ID] + servo__9_Offset) * servo__9_Direction - ServoAngle_Min) * (PPM_Max - PPM_Min) / (ServoAngle_Max - ServoAngle_Min) + PPM_Min);
  delay(delayPerServo);
  pwm_Board.writeMicroseconds(14,((Angle_Array[servo_14_ID] + servo_14_Offset) * servo_14_Direction - ServoAngle_Min) * (PPM_Max - PPM_Min) / (ServoAngle_Max - ServoAngle_Min) + PPM_Min);
  delay(delayPerServo);
  pwm_Board.writeMicroseconds(15,((Angle_Array[servo_15_ID] + servo_15_Offset) * servo_15_Direction - ServoAngle_Min) * (PPM_Max - PPM_Min) / (ServoAngle_Max - ServoAngle_Min) + PPM_Min);
  delay(delayPerServo);
  pwm_Board.writeMicroseconds( 1,((Angle_Array[servo__1_ID] + servo__1_Offset) * servo__1_Direction - ServoAngle_Min) * (PPM_Max - PPM_Min) / (ServoAngle_Max - ServoAngle_Min) + PPM_Min);
  delay(delayPerServo);
  pwm_Board.writeMicroseconds( 0,((Angle_Array[servo__0_ID] + servo__0_Offset) * servo__0_Direction - ServoAngle_Min) * (PPM_Max - PPM_Min) / (ServoAngle_Max - ServoAngle_Min) + PPM_Min);
  delay(delayPerServo);
  pwm_Board.writeMicroseconds( 7,((Angle_Array[servo__7_ID] + servo__7_Offset) * servo__7_Direction - ServoAngle_Min) * (PPM_Max - PPM_Min) / (ServoAngle_Max - ServoAngle_Min) + PPM_Min);
  delay(delayPerServo);
  pwm_Board.writeMicroseconds( 6,((Angle_Array[servo__6_ID] + servo__6_Offset) * servo__6_Direction - ServoAngle_Min) * (PPM_Max - PPM_Min) / (ServoAngle_Max - ServoAngle_Min) + PPM_Min);
  delay(delayPerServo);
  pwm_Board.writeMicroseconds(13,((Angle_Array[servo_13_ID] + servo_13_Offset) * servo_13_Direction - ServoAngle_Min) * (PPM_Max - PPM_Min) / (ServoAngle_Max - ServoAngle_Min) + PPM_Min);
  delay(delayPerServo);
  pwm_Board.writeMicroseconds(12,((Angle_Array[servo_12_ID] + servo_12_Offset) * servo_12_Direction - ServoAngle_Min) * (PPM_Max - PPM_Min) / (ServoAngle_Max - ServoAngle_Min) + PPM_Min);
  delay(delayPerServo);
  Servo__17.attach( 9);
  Servo__17.writeMicroseconds(   ((Angle_Array[servo_17_ID] + servo_17_Offset) * servo_17_Direction - ServoAngle_Min) * (PPM_Max - PPM_Min) / (ServoAngle_Max - ServoAngle_Min) + PPM_Min);
  delay(delayPerServo);
  Servo__18.attach(10);
  Servo__18.writeMicroseconds(   ((Angle_Array[servo_18_ID] + servo_18_Offset) * servo_18_Direction - ServoAngle_Min) * (PPM_Max - PPM_Min) / (ServoAngle_Max - ServoAngle_Min) + PPM_Min);
  delay(delayPerServo);
  pwm_Board.writeMicroseconds( 4,((Angle_Array[servo__4_ID] + servo__4_Offset) * servo__4_Direction - ServoAngle_Min) * (PPM_Max - PPM_Min) / (ServoAngle_Max - ServoAngle_Min) + PPM_Min);
  delay(delayPerServo);
  pwm_Board.writeMicroseconds( 5,((Angle_Array[servo__5_ID] + servo__5_Offset) * servo__5_Direction - ServoAngle_Min) * (PPM_Max - PPM_Min) / (ServoAngle_Max - ServoAngle_Min) + PPM_Min);
  delay(delayPerServo);
  pwm_Board.writeMicroseconds(10,((Angle_Array[servo_10_ID] + servo_10_Offset) * servo_10_Direction - ServoAngle_Min) * (PPM_Max - PPM_Min) / (ServoAngle_Max - ServoAngle_Min) + PPM_Min);
  delay(delayPerServo);
  pwm_Board.writeMicroseconds(11,((Angle_Array[servo_11_ID] + servo_11_Offset) * servo_11_Direction - ServoAngle_Min) * (PPM_Max - PPM_Min) / (ServoAngle_Max - ServoAngle_Min) + PPM_Min);
  delay(delayPerServo);
}

//     ___________________________________________________________________ 
//    |                           Y                                       |
//    |                          /                                        |
//    |                         /       /\                                |
//    |                 Z      /      / γ \                               |
//    |                 |__   /     /_    _\                              |
//    |                 |  '-_  A /   '--'  \                             |
//    |                 | β / ',/ \          \                            |
//    |                 |  /  /    \          \                           |
//    |                 | / /    δ  :          \                          |
//    |                 |//         |           \  B                      |
//    |              P1 O - _-  -  -' -  - .- X- \                        |
//    |                /  α  _/- _ /       |      \                       |
//    |               /__ --'      - _   ε :       \                      |
//    |              /            LPP  - _/         \                     |
//    |             /                      - _       \                    |
//    |            /                           - _    \                   |
//    |           /                                - _ \                  |
//    |                                                -O P2              |
//    |___________________________________________________________________|

void Update_Servo_Positions(float* Angle_Array)
{
  pwm_Board.writeMicroseconds( 0,((Angle_Array[servo__0_ID] + servo__0_Offset) * servo__0_Direction - ServoAngle_Min) * (PPM_Max - PPM_Min) / (ServoAngle_Max - ServoAngle_Min) + PPM_Min);
  pwm_Board.writeMicroseconds( 1,((Angle_Array[servo__1_ID] + servo__1_Offset) * servo__1_Direction - ServoAngle_Min) * (PPM_Max - PPM_Min) / (ServoAngle_Max - ServoAngle_Min) + PPM_Min);
  pwm_Board.writeMicroseconds( 2,((Angle_Array[servo__2_ID] + servo__2_Offset) * servo__2_Direction - ServoAngle_Min) * (PPM_Max - PPM_Min) / (ServoAngle_Max - ServoAngle_Min) + PPM_Min);
  pwm_Board.writeMicroseconds( 3,((Angle_Array[servo__3_ID] + servo__3_Offset) * servo__3_Direction - ServoAngle_Min) * (PPM_Max - PPM_Min) / (ServoAngle_Max - ServoAngle_Min) + PPM_Min);
  pwm_Board.writeMicroseconds( 4,((Angle_Array[servo__4_ID] + servo__4_Offset) * servo__4_Direction - ServoAngle_Min) * (PPM_Max - PPM_Min) / (ServoAngle_Max - ServoAngle_Min) + PPM_Min);
  pwm_Board.writeMicroseconds( 5,((Angle_Array[servo__5_ID] + servo__5_Offset) * servo__5_Direction - ServoAngle_Min) * (PPM_Max - PPM_Min) / (ServoAngle_Max - ServoAngle_Min) + PPM_Min);
  pwm_Board.writeMicroseconds( 6,((Angle_Array[servo__6_ID] + servo__6_Offset) * servo__6_Direction - ServoAngle_Min) * (PPM_Max - PPM_Min) / (ServoAngle_Max - ServoAngle_Min) + PPM_Min);
  pwm_Board.writeMicroseconds( 7,((Angle_Array[servo__7_ID] + servo__7_Offset) * servo__7_Direction - ServoAngle_Min) * (PPM_Max - PPM_Min) / (ServoAngle_Max - ServoAngle_Min) + PPM_Min);
  pwm_Board.writeMicroseconds( 8,((Angle_Array[servo__8_ID] + servo__8_Offset) * servo__8_Direction - ServoAngle_Min) * (PPM_Max - PPM_Min) / (ServoAngle_Max - ServoAngle_Min) + PPM_Min);
  pwm_Board.writeMicroseconds( 9,((Angle_Array[servo__9_ID] + servo__9_Offset) * servo__9_Direction - ServoAngle_Min) * (PPM_Max - PPM_Min) / (ServoAngle_Max - ServoAngle_Min) + PPM_Min);
  pwm_Board.writeMicroseconds(10,((Angle_Array[servo_10_ID] + servo_10_Offset) * servo_10_Direction - ServoAngle_Min) * (PPM_Max - PPM_Min) / (ServoAngle_Max - ServoAngle_Min) + PPM_Min);
  pwm_Board.writeMicroseconds(11,((Angle_Array[servo_11_ID] + servo_11_Offset) * servo_11_Direction - ServoAngle_Min) * (PPM_Max - PPM_Min) / (ServoAngle_Max - ServoAngle_Min) + PPM_Min);
  pwm_Board.writeMicroseconds(12,((Angle_Array[servo_12_ID] + servo_12_Offset) * servo_12_Direction - ServoAngle_Min) * (PPM_Max - PPM_Min) / (ServoAngle_Max - ServoAngle_Min) + PPM_Min);
  pwm_Board.writeMicroseconds(13,((Angle_Array[servo_13_ID] + servo_13_Offset) * servo_13_Direction - ServoAngle_Min) * (PPM_Max - PPM_Min) / (ServoAngle_Max - ServoAngle_Min) + PPM_Min);
  pwm_Board.writeMicroseconds(14,((Angle_Array[servo_14_ID] + servo_14_Offset) * servo_14_Direction - ServoAngle_Min) * (PPM_Max - PPM_Min) / (ServoAngle_Max - ServoAngle_Min) + PPM_Min);
  pwm_Board.writeMicroseconds(15,((Angle_Array[servo_15_ID] + servo_15_Offset) * servo_15_Direction - ServoAngle_Min) * (PPM_Max - PPM_Min) / (ServoAngle_Max - ServoAngle_Min) + PPM_Min);
  Servo__17.writeMicroseconds(   ((Angle_Array[servo_17_ID] + servo_17_Offset) * servo_17_Direction - ServoAngle_Min) * (PPM_Max - PPM_Min) / (ServoAngle_Max - ServoAngle_Min) + PPM_Min);
  Servo__18.writeMicroseconds(   ((Angle_Array[servo_18_ID] + servo_18_Offset) * servo_18_Direction - ServoAngle_Min) * (PPM_Max - PPM_Min) / (ServoAngle_Max - ServoAngle_Min) + PPM_Min);
}