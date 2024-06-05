#pragma once

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


//general

#define loopPeriod 40//40

#define input_Move_X      0
#define input_Move_Y      1
#define input_Rotate      3
#define input_Move_Z      2
#define input_Toggle_TiltMode 4
#define input_Toggle_TranslateMode 5

#define ArmCount      6
#define AnglesPerArm  3
#define LengthsPerArm 2

#define minimumChassisHeight 21.588

#define MaxLiftHeight_Max 40.0  //mm
#define MaxLiftHeight_Min 10.0  //mm
#define MaxLiftHeight_scaler (((MaxLiftHeight_Max - MaxLiftHeight_Min) / 2) / ((SBUS_mappedValueMax - SBUS_mappedValueMin) / 2))
#define MaxHeight_At_DistanceToTarget 20.0  //mm
#define LiftingSpeed  6.0   //mm per frame

#define XY_Axis        2
#define XYZ_Axis       3
#define X              0
#define Y              1
#define Z              2
#define Alpha          0
#define Beta           1
#define Gamma          2

#define ServoAngle_Max  90.0
#define ServoAngle_Min -90.0
#define PPM_Max       2500.0
#define PPM_Min        500.0

#define ArmLength_Arm1_A 50
#define ArmLength_Arm1_B 70

#define ArmLength_Arm2_A 50
#define ArmLength_Arm2_B 70

#define ArmLength_Arm3_A 50
#define ArmLength_Arm3_B 70

#define ArmLength_Arm4_A 50
#define ArmLength_Arm4_B 70

#define ArmLength_Arm5_A 50
#define ArmLength_Arm5_B 70

#define ArmLength_Arm6_A 50
#define ArmLength_Arm6_B 70


// SBUS

#define SBUS_Baud              100000
#define SBUS_PacketSize        25
#define SBUS_NumberOfChannels  8
#define SBUS_mappedValueMax    1000
#define SBUS_mappedValueMin   -1000
#define SBUS_mappedValueCenter 0
#define SBUS_rawValueMax       1814
#define SBUS_rawValueMin       172

#define SBUS_DeadZone_Channel_1 5
#define SBUS_DeadZone_Channel_2 5
#define SBUS_DeadZone_Channel_3 0
#define SBUS_DeadZone_Channel_4 5
#define SBUS_DeadZone_Channel_5 0
#define SBUS_DeadZone_Channel_6 0
#define SBUS_DeadZone_Channel_7 0
#define SBUS_DeadZone_Channel_8 0

#define SBUS_Failsafe_Channel_1     0
#define SBUS_Failsafe_Channel_2     0
#define SBUS_Failsafe_Channel_3  1000
#define SBUS_Failsafe_Channel_4     0
#define SBUS_Failsafe_Channel_5  1000
#define SBUS_Failsafe_Channel_6  1000
#define SBUS_Failsafe_Channel_7     0
#define SBUS_Failsafe_Channel_8     0

#define USE_CHANNEL_1_FAILSAFE
#define USE_CHANNEL_2_FAILSAFE
// #define USE_CHANNEL_3_FAILSAFE
#define USE_CHANNEL_4_FAILSAFE
#define USE_CHANNEL_5_FAILSAFE
#define USE_CHANNEL_6_FAILSAFE
#define USE_CHANNEL_7_FAILSAFE
#define USE_CHANNEL_8_FAILSAFE

//Movement

#define Tilt_input_scaler_X  1 / (2.0 * 3 * 1000)
#define Tilt_input_scaler_Y -1 / (2.0 * 3 * 1000)
#define Tilt_input_scaler_Z -1 / (2.0 * 6 * 1000)

#define Translate_input_scaler_X -1 / 50.0
#define Translate_input_scaler_Y -1 / 50.0

#define ChassisHeight_input_scaler 1 / 30.0

#define Tilt_rotationPoint_Height 30



#define initial_Arm1_P1_X -30
#define initial_Arm1_P1_Y 40
#define initial_Arm1_P1_Z 21.588

#define initial_Arm2_P1_X 30
#define initial_Arm2_P1_Y 40
#define initial_Arm2_P1_Z 21.588

#define initial_Arm3_P1_X -30
#define initial_Arm3_P1_Y 0
#define initial_Arm3_P1_Z 21.588

#define initial_Arm4_P1_X 30
#define initial_Arm4_P1_Y 0
#define initial_Arm4_P1_Z 21.588

#define initial_Arm5_P1_X -30
#define initial_Arm5_P1_Y -40
#define initial_Arm5_P1_Z 21.588

#define initial_Arm6_P1_X 30
#define initial_Arm6_P1_Y -40
#define initial_Arm6_P1_Z 21.588



#define initial_Arm1_P2_X -42.5
#define initial_Arm1_P2_Y 40
#define initial_Arm1_P2_Z 0

#define initial_Arm2_P2_X 42.5
#define initial_Arm2_P2_Y 40
#define initial_Arm2_P2_Z 0

#define initial_Arm3_P2_X -42.5
#define initial_Arm3_P2_Y 0
#define initial_Arm3_P2_Z 0

#define initial_Arm4_P2_X 42.5
#define initial_Arm4_P2_Y 0
#define initial_Arm4_P2_Z 0

#define initial_Arm5_P2_X -42.5
#define initial_Arm5_P2_Y -40
#define initial_Arm5_P2_Z 0

#define initial_Arm6_P2_X 42.5
#define initial_Arm6_P2_Y -40
#define initial_Arm6_P2_Z 0



#define Arm1_CircleCenter_X -83.623
#define Arm1_CircleCenter_Y 84.995

#define Arm2_CircleCenter_X 83.623
#define Arm2_CircleCenter_Y 84.995

#define Arm3_CircleCenter_X -100
#define Arm3_CircleCenter_Y 0

#define Arm4_CircleCenter_X 100
#define Arm4_CircleCenter_Y 0

#define Arm5_CircleCenter_X -83.623
#define Arm5_CircleCenter_Y -84.995

#define Arm6_CircleCenter_X 83.623
#define Arm6_CircleCenter_Y -84.995



#define Arm1_CircleRadius 23.94
#define Arm2_CircleRadius 23.94
#define Arm3_CircleRadius 23.94
#define Arm4_CircleRadius 23.94
#define Arm5_CircleRadius 23.94
#define Arm6_CircleRadius 23.94

// 3-3 Gate

#define stepMargain 10.0
#define LiftArm_targetMargin 5.0

#define Walking_rotation_input_scaler 1.0 * PI / (180 * (float)SBUS_mappedValueMax)  //1 deg as max
#define Walking_X_input_scaler 2.0 / SBUS_mappedValueMax                      //2 mm as max
#define Walking_Y_input_scaler 2.0 / SBUS_mappedValueMax

// Servo

#define servoFrequency 1000.0 / loopPeriod
#define delayPerServo 300
#define Servo_PPM_scaler 


#define servo__0_Offset  5.67
#define servo__1_Offset -6.22
#define servo__2_Offset  4.81
#define servo__3_Offset -3.93
#define servo__4_Offset  3.13
#define servo__5_Offset -7.24
#define servo__6_Offset -7.74
#define servo__7_Offset -5.29
#define servo__8_Offset -1.13
#define servo__9_Offset -1.50
#define servo_10_Offset  4.81
#define servo_11_Offset -5.65
#define servo_12_Offset  8.23
#define servo_13_Offset -1.13
#define servo_14_Offset  0.76
#define servo_15_Offset -1.96
#define servo_17_Offset -8.50
#define servo_18_Offset -3.75


#define servo__0_ID 1 * AnglesPerArm + Alpha
#define servo__1_ID 0 * AnglesPerArm + Alpha
#define servo__2_ID 0 * AnglesPerArm + Beta 
#define servo__3_ID 1 * AnglesPerArm + Beta 
#define servo__4_ID 2 * AnglesPerArm + Gamma
#define servo__5_ID 3 * AnglesPerArm + Gamma
#define servo__6_ID 3 * AnglesPerArm + Alpha
#define servo__7_ID 2 * AnglesPerArm + Alpha
#define servo__8_ID 2 * AnglesPerArm + Beta 
#define servo__9_ID 3 * AnglesPerArm + Beta 
#define servo_10_ID 4 * AnglesPerArm + Gamma
#define servo_11_ID 5 * AnglesPerArm + Gamma
#define servo_12_ID 5 * AnglesPerArm + Alpha
#define servo_13_ID 4 * AnglesPerArm + Alpha
#define servo_14_ID 4 * AnglesPerArm + Beta 
#define servo_15_ID 5 * AnglesPerArm + Beta 
#define servo_17_ID 0 * AnglesPerArm + Gamma
#define servo_18_ID 1 * AnglesPerArm + Gamma


#define servo__0_Direction -1
#define servo__1_Direction -1
#define servo__2_Direction  1
#define servo__3_Direction -1
#define servo__4_Direction  1
#define servo__5_Direction -1
#define servo__6_Direction -1
#define servo__7_Direction -1
#define servo__8_Direction  1
#define servo__9_Direction -1
#define servo_10_Direction  1
#define servo_11_Direction -1
#define servo_12_Direction -1
#define servo_13_Direction -1
#define servo_14_Direction  1
#define servo_15_Direction -1
#define servo_17_Direction  1
#define servo_18_Direction -1