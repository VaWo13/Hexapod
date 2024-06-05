#include <arduino.h>

#include "../Header Files/Config.h"
#include "../Header Files/Movement.h"
#include "../Header Files/3-3 Gate.h"

float all_Arm_Circle_Centers   [ArmCount][XY_Axis] = {
                          { Arm1_CircleCenter_X,
                            Arm1_CircleCenter_Y 
                          },                    
                          { Arm2_CircleCenter_X,
                            Arm2_CircleCenter_Y 
                          },                    
                          { Arm3_CircleCenter_X,
                            Arm3_CircleCenter_Y 
                          },                    
                          { Arm4_CircleCenter_X,
                            Arm4_CircleCenter_Y 
                          },                    
                          { Arm5_CircleCenter_X,
                            Arm5_CircleCenter_Y 
                          },                    
                          { Arm6_CircleCenter_X,
                            Arm6_CircleCenter_Y 
                          }};
float all_Arm_Circle_Radius    [ArmCount] = {
                            Arm1_CircleRadius,
                            Arm2_CircleRadius,
                            Arm3_CircleRadius,
                            Arm4_CircleRadius,
                            Arm5_CircleRadius,
                            Arm6_CircleRadius
                            };
float target_Points            [ArmCount][XY_Axis];
float estimated_Arrival_Time   [ArmCount];                                             //distance traveled / remaining distance
bool  arm_lifting_Mode         [ArmCount] = {true, false, false, true, true, false};
bool  arm_outside_circle       [ArmCount];
bool  arm_liftHeight_increasing[ArmCount];
float arm_liftHeight_max       [ArmCount];
float minimum_estimated_arrival_time;
float maxLiftHeight;

float smoothTilt              [XYZ_Axis] = {0, 0, 0};
float smoothTranslate         [XY_Axis]  = {0, 0};
float smoothHeight                       = 30;

void move_Gate(int* input_Channels, float* all_Arm_Points_2, bool center_arms)
{
  if (input_Channels[6] < 0)  minimum_estimated_arrival_time = _3x3_Movement(*target_Points, estimated_Arrival_Time, arm_lifting_Mode, input_Channels, all_Arm_Points_2, *all_Arm_Circle_Centers, all_Arm_Circle_Radius);
  else                        minimum_estimated_arrival_time = 0;
  // if (center_arms == true)
  // {
    
  // }
  maxLiftHeight = MaxLiftHeight_Min + ((MaxLiftHeight_Max - MaxLiftHeight_Min) / 2) + (input_Channels[7] * MaxLiftHeight_scaler);
}

void move_Arms_Towards_Target(float* all_Arm_Points_2)
{
  if ((minimum_estimated_arrival_time < 0) | (minimum_estimated_arrival_time > 1)) minimum_estimated_arrival_time = 1;
  
  for (size_t i = 0; i < ArmCount; i++)
  {
    if (arm_lifting_Mode[i] == true)
    {
      //move P2 towards target, scaled by estimated arrival time
      *(all_Arm_Points_2 + (i * XYZ_Axis + X)) += ((target_Points[i][X] - all_Arm_Points_2[i * XYZ_Axis + X]) * minimum_estimated_arrival_time);
      *(all_Arm_Points_2 + (i * XYZ_Axis + Y)) += ((target_Points[i][Y] - all_Arm_Points_2[i * XYZ_Axis + Y]) * minimum_estimated_arrival_time);

      //get arm lift height
      float P2_Target_distance = sqrt(((target_Points[i][X] - all_Arm_Points_2[i * XYZ_Axis + X]) * (target_Points[i][X] - all_Arm_Points_2[i * XYZ_Axis + X])) + ((target_Points[i][Y] - all_Arm_Points_2[i * XYZ_Axis + Y]) * (target_Points[i][Y] - all_Arm_Points_2[i * XYZ_Axis + Y])));
      if (P2_Target_distance < MaxHeight_At_DistanceToTarget)
      {
        if (arm_liftHeight_increasing[i] == true) //set max height for descent when first switching from riding, to lowering the arm
        {
          arm_liftHeight_increasing[i] = false;
          arm_liftHeight_max[i] = all_Arm_Points_2[i * XYZ_Axis + Z];
        }
        
        *(all_Arm_Points_2 + (i * XYZ_Axis + Z)) = (-(P2_Target_distance * P2_Target_distance) + (2 * MaxHeight_At_DistanceToTarget * P2_Target_distance)) * (arm_liftHeight_max[i] / (MaxHeight_At_DistanceToTarget * MaxHeight_At_DistanceToTarget));
      }
      else
      {
        *(all_Arm_Points_2 + (i * XYZ_Axis + Z)) += LiftingSpeed;
        if (all_Arm_Points_2[i * XYZ_Axis + Z] > maxLiftHeight) *(all_Arm_Points_2 + (i * XYZ_Axis + Z)) = maxLiftHeight;
        arm_liftHeight_increasing[i] = true;
      }
      
      //if P2 has reached the target, that arm turns lift mode off (all arms in liftmode turn off at the same time, cus they share the estimated arrival time)
      if (minimum_estimated_arrival_time == 1)
      {
        arm_lifting_Mode[i] = false;
        all_Arm_Points_2[i * XYZ_Axis + Z] = 0;
      }
    }
    else
    {
      *(all_Arm_Points_2 + (i * XYZ_Axis + X)) = target_Points[i][X];
      *(all_Arm_Points_2 + (i * XYZ_Axis + Y)) = target_Points[i][Y];
    }
  }



  //check if target points of grounded arms are outside the circle-----------------------------------------------------
  for (size_t i = 0; i < ArmCount; i++)
  {
    if (arm_lifting_Mode[i] == false)
    {
      //distance from TP to CC ( k )
      float delta_TP_CC_X = all_Arm_Circle_Centers[i][X] - target_Points[i][X];
      float delta_TP_CC_Y = all_Arm_Circle_Centers[i][Y] - target_Points[i][Y];
      float distance_TP_CC = sqrt(((delta_TP_CC_X) * (delta_TP_CC_X)) + ((delta_TP_CC_Y) * (delta_TP_CC_Y)));
      if (distance_TP_CC >= all_Arm_Circle_Radius[i])  arm_outside_circle[i] = true;
      else  arm_outside_circle[i] = false;
    }
    else  arm_outside_circle[i] = false;
  }

  //if so, check which group, and then change all arms in that group to lift mode--------------------------------------
  if ((arm_outside_circle[0] == true) | (arm_outside_circle[3] == true) | (arm_outside_circle[4] == true))
  {
    arm_lifting_Mode[0] = true;
    arm_lifting_Mode[3] = true;
    arm_lifting_Mode[4] = true;
    //show a warning, if 2 or more groups are outside at the same time-------------------------------------------------
    if ((arm_outside_circle[1] == true) | (arm_outside_circle[2] == true) | (arm_outside_circle[5] == true)) Serial.println("!WARNING! : Arm Group A and B outside circle (A)");
  }
  if ((arm_outside_circle[1] == true) | (arm_outside_circle[2] == true) | (arm_outside_circle[5] == true))
  {
    arm_lifting_Mode[1] = true;
    arm_lifting_Mode[2] = true;
    arm_lifting_Mode[5] = true;
    //show a warning, if 2 or more groups are outside at the same time-------------------------------------------------
    if ((arm_outside_circle[0] == true) | (arm_outside_circle[3] == true) | (arm_outside_circle[4] == true)) Serial.println("!WARNING! : Arm Group A and B outside circle (B)");
  }
}

void move_main_Chassis(int* input_Channels,float* all_Arm_Points_1, float* all_Arm_Points_2, float* all_Arm_Points_2_rotated, float smoothing)
{ 
  //chassis Tilt
  if (input_Channels[input_Toggle_TiltMode] < 0)
  {
   smoothTilt[X] += ((input_Channels[input_Move_X] * Tilt_input_scaler_X) - smoothTilt[X]) * smoothing;
   smoothTilt[Y] += ((input_Channels[input_Move_Y] * Tilt_input_scaler_Y) - smoothTilt[Y]) * smoothing;
   smoothTilt[Z] += ((input_Channels[input_Rotate] * Tilt_input_scaler_Z) - smoothTilt[Z]) * smoothing;
  }

  //chassis Translate
  if (input_Channels[input_Toggle_TranslateMode] < 0)
  {
    smoothTranslate[X] += ((input_Channels[input_Move_X] * Translate_input_scaler_X) - smoothTranslate[X]) * smoothing;
    smoothTranslate[Y] += ((input_Channels[input_Move_Y] * Translate_input_scaler_Y) - smoothTranslate[Y]) * smoothing;
  }

  //Chassis Height
  smoothHeight += (minimumChassisHeight + ((input_Channels[input_Move_Z] - SBUS_mappedValueMin) * ChassisHeight_input_scaler) - smoothHeight) * smoothing;

  float TiltQuaternion[4] = {
          cos( smoothTilt[Z]),
          sin(-smoothTilt[Y]),
          sin(-smoothTilt[X]),
          sin( smoothTilt[Z])};

  for (size_t i = 0; i < ArmCount; i++)
  {
    //Height
    *(all_Arm_Points_1 + (i * XYZ_Axis + Z)) = smoothHeight;

    //translation
    float Arm_Point_Quaternion[4] = { 0, all_Arm_Points_2[i * XYZ_Axis + X] + smoothTranslate[X], all_Arm_Points_2[i * XYZ_Axis + Y] + smoothTranslate[Y], all_Arm_Points_2[i * XYZ_Axis + Z] - smoothHeight - Tilt_rotationPoint_Height};

    //Tilt
    float *p3 = QuaternionProduct(QuaternionProduct(QuaternionNormalize(TiltQuaternion), Arm_Point_Quaternion), QuaternionInverse(QuaternionNormalize(TiltQuaternion)));
    *(all_Arm_Points_2_rotated + (i * XYZ_Axis + X)) = *(p3 + 1);
    *(all_Arm_Points_2_rotated + (i * XYZ_Axis + Y)) = *(p3 + 2);
    *(all_Arm_Points_2_rotated + (i * XYZ_Axis + Z)) = *(p3 + 3) + smoothHeight + Tilt_rotationPoint_Height;
  }
}

//Quaternion math

float *QuaternionProduct(float *q1, float *q2)
{
  static float q3[4];
  q3[0] = (q1[0] * q2[0]) - (q1[1] * q2[1]) - (q1[2] * q2[2]) - (q1[3] * q2[3]);
  q3[1] = (q1[0] * q2[1]) + (q1[1] * q2[0]) + (q1[2] * q2[3]) - (q1[3] * q2[2]);
  q3[2] = (q1[0] * q2[2]) - (q1[1] * q2[3]) + (q1[2] * q2[0]) + (q1[3] * q2[1]);
  q3[3] = (q1[0] * q2[3]) + (q1[1] * q2[2]) - (q1[2] * q2[1]) + (q1[3] * q2[0]);
  return q3;
}
float *QuaternionInverse(float *q1)
{
  static float qi[4];
  qi[0] = q1[0];
  qi[1] = q1[1] * -1;
  qi[2] = q1[2] * -1;
  qi[3] = q1[3] * -1;
  return qi;
}
float *QuaternionNormalize(float *q1)
{
  float vectorlength = sqrt((q1[0] * q1[0]) + (q1[1] * q1[1]) + (q1[2] * q1[2]) + (q1[3] * q1[3]));
  static float qn[4];
  qn[0] = q1[0] / vectorlength;
  qn[1] = q1[1] / vectorlength;
  qn[2] = q1[2] / vectorlength;
  qn[3] = q1[3] / vectorlength;
  return qn;
}