#include <arduino.h>

#include "../Header Files/Config.h"
#include "../Header Files/3-3 Gate.h"

float _3x3_Movement(float* target_Points,float* estimated_Arrival_Time,bool* arm_lifting_Mode,int* input_Channels,float* all_Arm_Points_2,float* all_Arm_Circle_Centers,float* all_Arm_Circle_Radius)
{
  float minimum_estimated_arrival_time = 0;


  float scaled_input_Vectorlength = sqrt(((input_Channels[input_Move_X] * Walking_X_input_scaler) * (input_Channels[input_Move_X] * Walking_X_input_scaler)) + ((input_Channels[input_Move_Y] * Walking_Y_input_scaler) * (input_Channels[input_Move_Y] * Walking_Y_input_scaler)));  //m

  //find Virtual Rotation point (if translating and rotating at the same time) (for Virtual Rotation point refer to Fusion360 Model)
  float Virtual_Rotation_Point[XY_Axis];
  float input_Vectorlength = sqrt(((float)input_Channels[input_Move_X] * (float)input_Channels[input_Move_X]) + ((float)input_Channels[input_Move_Y] * (float)input_Channels[input_Move_Y]));
  if ((input_Channels[input_Rotate] != 0) & (input_Vectorlength != 0))
  {
    float rotationPointDistance = scaled_input_Vectorlength / (2 * sin((-input_Channels[input_Rotate] * Walking_rotation_input_scaler) / 2));
    Virtual_Rotation_Point[X] = -input_Channels[input_Move_Y] * rotationPointDistance / input_Vectorlength;
    Virtual_Rotation_Point[Y] =  input_Channels[input_Move_X] * rotationPointDistance / input_Vectorlength;
  }
  else
  {
    Virtual_Rotation_Point[X] = 0;
    Virtual_Rotation_Point[Y] = 0;
  }
  

  float sin_half_input_rotate = asin(input_Channels[input_Rotate]* 0.5 * Walking_rotation_input_scaler);  //if input is positive, rotate counterclockwise
  float cos_half_input_rotate = sqrt(1 - (sin_half_input_rotate * sin_half_input_rotate));

  for (size_t i = 0; i < ArmCount; i++)
  {
    //distance from CC(circle Center) to VRP(Virtual Rotation Point) ( f )
    float delta_CC_VRP_X = all_Arm_Circle_Centers[i * XY_Axis + X] - Virtual_Rotation_Point[X];
    float delta_CC_VRP_Y = all_Arm_Circle_Centers[i * XY_Axis + Y] - Virtual_Rotation_Point[Y];
    float distance_CC_VRP = sqrt((delta_CC_VRP_X * delta_CC_VRP_X) + (delta_CC_VRP_Y * delta_CC_VRP_Y));

    if (arm_lifting_Mode[i] == true)
    {
      //arm is in lift mode

      //get target point coincident on arm circle----------------------------------------------------------------------
      if (input_Channels[input_Rotate] != 0)
      {
        if (distance_CC_VRP > ((all_Arm_Circle_Radius[i] - LiftArm_targetMargin) / 2))  //the equation breaks, if the VRP is within half of the circle
        {
          float r_half = (all_Arm_Circle_Radius[i] - LiftArm_targetMargin) / 2;
          float sin_alpha = ((-input_Channels[input_Rotate] > 0) ? r_half : -r_half) / distance_CC_VRP;
          float cos_alpha = sqrt((distance_CC_VRP * distance_CC_VRP) - (r_half * r_half)) / distance_CC_VRP;
          float midPoint[XY_Axis] = {   //create the intermediate point, by translating the VRP to 0,0(and the CC with it), and rotating the CC
            ((all_Arm_Circle_Centers[i * XY_Axis + X] - Virtual_Rotation_Point[X]) * cos_alpha) - ((all_Arm_Circle_Centers[i * XY_Axis + Y] - Virtual_Rotation_Point[Y]) * sin_alpha),
            ((all_Arm_Circle_Centers[i * XY_Axis + Y] - Virtual_Rotation_Point[Y]) * cos_alpha) + ((all_Arm_Circle_Centers[i * XY_Axis + X] - Virtual_Rotation_Point[X]) * sin_alpha)};
          //rotate a second time, and translate back to original position
          *(target_Points + (i * XY_Axis + X)) = (midPoint[X] * cos_alpha) - (midPoint[Y] * sin_alpha) + Virtual_Rotation_Point[X];
          *(target_Points + (i * XY_Axis + Y)) = (midPoint[Y] * cos_alpha) + (midPoint[X] * sin_alpha) + Virtual_Rotation_Point[Y];

          //correct for TP not being coincident with the circle
          //distance from TP to CC ( k )
          float delta_TP_CC_X = all_Arm_Circle_Centers[i * XY_Axis + X] - target_Points[i * XY_Axis + X];
          float delta_TP_CC_Y = all_Arm_Circle_Centers[i * XY_Axis + Y] - target_Points[i * XY_Axis + Y];
          float distance_TP_CC = sqrt(((delta_TP_CC_X) * (delta_TP_CC_X)) + ((delta_TP_CC_Y) * (delta_TP_CC_Y)));
          target_Points[i * XY_Axis + X] += delta_TP_CC_X * (distance_TP_CC - (all_Arm_Circle_Radius[i] - LiftArm_targetMargin)) / distance_TP_CC;
          target_Points[i * XY_Axis + Y] += delta_TP_CC_Y * (distance_TP_CC - (all_Arm_Circle_Radius[i] - LiftArm_targetMargin)) / distance_TP_CC;
        }
        else
        {
          target_Points[i * XY_Axis + X] = all_Arm_Circle_Centers[i * XY_Axis + X];
          target_Points[i * XY_Axis + Y] = all_Arm_Circle_Centers[i * XY_Axis + Y];
        }
      }
      else
      {
        if (input_Vectorlength != 0)  //cant divide by 0
        {
          target_Points[i * XY_Axis + X] = all_Arm_Circle_Centers[i * XY_Axis + X] + ((all_Arm_Circle_Radius[i] - LiftArm_targetMargin) * (float)input_Channels[input_Move_X] / input_Vectorlength);
          target_Points[i * XY_Axis + Y] = all_Arm_Circle_Centers[i * XY_Axis + Y] + ((all_Arm_Circle_Radius[i] - LiftArm_targetMargin) * (float)input_Channels[input_Move_Y] / input_Vectorlength);
        }

        //correct for TP not being coincident with the circle
        //distance from TP to CC ( k )
        float delta_TP_CC_X = all_Arm_Circle_Centers[i * XY_Axis + X] - target_Points[i * XY_Axis + X];
        float delta_TP_CC_Y = all_Arm_Circle_Centers[i * XY_Axis + Y] - target_Points[i * XY_Axis + Y];
        float distance_TP_CC = sqrt(((delta_TP_CC_X) * (delta_TP_CC_X)) + ((delta_TP_CC_Y) * (delta_TP_CC_Y)));
        target_Points[i * XY_Axis + X] += delta_TP_CC_X * (distance_TP_CC - (all_Arm_Circle_Radius[i] - LiftArm_targetMargin)) / distance_TP_CC;
        target_Points[i * XY_Axis + Y] += delta_TP_CC_Y * (distance_TP_CC - (all_Arm_Circle_Radius[i] - LiftArm_targetMargin)) / distance_TP_CC;
      }
    }
    else
    {
      //arm is not in lift mode

      //get target point for next arm movement-------------------------------------------------------------------------

      float midPoint[XY_Axis] = {   //create the intermediate point and rotate the P2 around 0,0
        (all_Arm_Points_2[i * XYZ_Axis + X] * cos_half_input_rotate) - (all_Arm_Points_2[i * XYZ_Axis + Y] * sin_half_input_rotate),
        (all_Arm_Points_2[i * XYZ_Axis + Y] * cos_half_input_rotate) + (all_Arm_Points_2[i * XYZ_Axis + X] * sin_half_input_rotate)};
      //rotate a second time, and subtract(cus opposite direction to input) the half-rotated input vector
      *(target_Points + (i * XY_Axis + X)) = (midPoint[X] * cos_half_input_rotate) - (midPoint[Y] * sin_half_input_rotate) - ((input_Channels[input_Move_X] * Walking_X_input_scaler) * cos_half_input_rotate) - ((input_Channels[input_Move_Y] * Walking_Y_input_scaler) * sin_half_input_rotate);
      *(target_Points + (i * XY_Axis + Y)) = (midPoint[Y] * cos_half_input_rotate) + (midPoint[X] * sin_half_input_rotate) - ((input_Channels[input_Move_Y] * Walking_Y_input_scaler) * cos_half_input_rotate) + ((input_Channels[input_Move_X] * Walking_X_input_scaler) * sin_half_input_rotate);


      //get distance, to be traveled (P2 to target point)--------------------------------------------------------------

      //distance from CC to TP ( m )
      float delta_P2_TP_X = all_Arm_Points_2[i * XYZ_Axis + X] - target_Points[i * XY_Axis + X]; 
      float delta_P2_TP_Y = all_Arm_Points_2[i * XYZ_Axis + Y] - target_Points[i * XY_Axis + Y];
      float distance_P2_TP = sqrt((delta_P2_TP_X * delta_P2_TP_X) + (delta_P2_TP_Y * delta_P2_TP_Y));


      //get remaining distance-----------------------------------------------------------------------------------------

      //distance from TP to CC ( k )
      float delta_TP_CC_X = all_Arm_Circle_Centers[i * XY_Axis + X] - target_Points[i * XY_Axis + X];
      float delta_TP_CC_Y = all_Arm_Circle_Centers[i * XY_Axis + Y] - target_Points[i * XY_Axis + Y];
      float distance_TP_CC = sqrt(((delta_TP_CC_X) * (delta_TP_CC_X)) + ((delta_TP_CC_Y) * (delta_TP_CC_Y)));

      //get estimated arrival time-------------------------------------------------------------------------------------
      if (input_Channels[input_Rotate] != 0)
      {
        //distance from P2 to VRP ( d )
        float delta_P2_VRP_X = all_Arm_Points_2[i * XYZ_Axis + X] - Virtual_Rotation_Point[X];
        float delta_P2_VRP_Y = all_Arm_Points_2[i * XYZ_Axis + Y] - Virtual_Rotation_Point[Y];
        float distance_P2_VRP = sqrt((delta_P2_VRP_X * delta_P2_VRP_X) + (delta_P2_VRP_Y * delta_P2_VRP_Y));

        if ((distance_CC_VRP + distance_P2_VRP) > all_Arm_Circle_Radius[i]) //the equation breaks, if the VRP is too far inside the arm circle
        {
          float h = 2 * heron(distance_P2_VRP, all_Arm_Circle_Radius[i], distance_CC_VRP) / distance_CC_VRP;
          float g = sqrt((all_Arm_Circle_Radius[i] * all_Arm_Circle_Radius[i]) - (h * h));
          float P3_becomes_P4[XY_Axis] = {  //create P3, which later gets turned into P4, by using g as a scaler to move the VRP towards the CC
            ((Virtual_Rotation_Point[X] - all_Arm_Circle_Centers[i * XY_Axis + X]) * g / distance_CC_VRP) + all_Arm_Circle_Centers[i * XY_Axis + X],
            ((Virtual_Rotation_Point[Y] - all_Arm_Circle_Centers[i * XY_Axis + Y]) * g / distance_CC_VRP) + all_Arm_Circle_Centers[i * XY_Axis + Y]};

          P3_becomes_P4[X] += (((input_Channels[input_Rotate] > 0) ? -h :  h) * delta_CC_VRP_Y / distance_CC_VRP);
          P3_becomes_P4[Y] += (((input_Channels[input_Rotate] > 0) ?  h : -h) * delta_CC_VRP_X / distance_CC_VRP);

          float delta_P2_P4_X = all_Arm_Points_2[i * XYZ_Axis + X] - P3_becomes_P4[X];
          float delta_P2_P4_Y = all_Arm_Points_2[i * XYZ_Axis + Y] - P3_becomes_P4[Y];
          float l = 2 * asin((sqrt((delta_P2_P4_X * delta_P2_P4_X) + (delta_P2_P4_Y * delta_P2_P4_Y)) / 2) / distance_P2_VRP) * distance_P2_VRP;
          //get estimated arrival time---------------------------------------------------------------------------------
          estimated_Arrival_Time[i] = (l - stepMargain) <= 0 ? 1 : (distance_P2_TP / (l - stepMargain));
        }
        else  estimated_Arrival_Time[i] = 0;
      }
      else
      {
        if (scaled_input_Vectorlength != 0) //cant divide by 0
        {
          //distance from CC to P2 ( j )
          float delta_P2_CC_X = all_Arm_Points_2[i * XYZ_Axis + X] - all_Arm_Circle_Centers[i * XY_Axis + X];
          float delta_P2_CC_Y = all_Arm_Points_2[i * XYZ_Axis + Y] - all_Arm_Circle_Centers[i * XY_Axis + Y];
          float distance_P2_CC = sqrt(((delta_P2_CC_X) * (delta_P2_CC_X)) + ((delta_P2_CC_Y) * (delta_P2_CC_Y)));

          float h = 2 * heron(distance_P2_CC, distance_TP_CC, scaled_input_Vectorlength) / scaled_input_Vectorlength;
          float n = sqrt((distance_P2_CC * distance_P2_CC) - (h * h));
          float l = sqrt((all_Arm_Circle_Radius[i] * all_Arm_Circle_Radius[i]) - (h * h)) + (((( distance_TP_CC * distance_TP_CC) - (distance_P2_CC * distance_P2_CC) - (scaled_input_Vectorlength * scaled_input_Vectorlength)) < 0) ? n : -n);
          //get estimated arrival time---------------------------------------------------------------------------------
          estimated_Arrival_Time[i] = (l - stepMargain) <= 0 ? 1 : (distance_P2_TP / (l - stepMargain));
        }
        else  estimated_Arrival_Time[i] = 0;
      }
      
      //get minimum estimated arrival time-------------------------------------------------------------------------------
      if (estimated_Arrival_Time[i] > minimum_estimated_arrival_time)   minimum_estimated_arrival_time = estimated_Arrival_Time[i];
    }
  }
  return minimum_estimated_arrival_time;
}

float heron(float a, float b, float c)
{
  //sort, so that a is the biggest number, b is 2nd largest, c is smallest
  if (a < b)
  {
    float t = b;
    b = a;
    a = t;
  }
  if (a < c)
  {
    float t = c;
    c = a;
    a = t;
  }
  if (b < c)
  {
    float t = c;
    c = b;
    b = t;
  }
  float p = (a + (b + c)) * (c - (a - b)) * (c + (a - b)) * (a + (b - c));
  if (p > 0)
  {
    return 0.25 * sqrt(p);
  }
  else
  {
    return 0;
  }
}