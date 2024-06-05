#include <arduino.h>

#include "../Header Files/Config.h"
#include "../Header Files/Inverse Kinematics.h"

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

void Single_Arm_Inverse_Kinematics(float* Array_Pointer, float delta_x_axis, float delta_y_axis, float delta_z_axis, int arm_length_A, int arm_length_B)
{
  //math - - see diagramm above
    float lpp = sqrt(((delta_x_axis * delta_x_axis) + (delta_y_axis * delta_y_axis) + ((delta_z_axis * delta_z_axis))));
    *(Array_Pointer + Alpha) = ((atan((delta_y_axis) / (delta_x_axis)) * 180) / PI);
    float epsilon = (asin((delta_z_axis) / lpp) * 180) / PI;
    float delta = acos(((arm_length_A * arm_length_A) + (lpp * lpp) - (arm_length_B * arm_length_B)) / (2 * arm_length_A * lpp));
    *(Array_Pointer + Gamma) = 90 - (asin(((float)arm_length_A / arm_length_B) * sin(delta)) * 180 / PI) - (delta * 180 / PI);
    *(Array_Pointer + Beta ) = (((delta * 180) / PI) + epsilon);
}

void all_Arms_inverse_Kinematics(float* All_Arm_Angles_Array_Pointer, float* arm_points_1, float* arm_points_2, int* arm_lengths)
{
  for (size_t i = 0; i < ArmCount; i++)   //do for all arms
  {
    Single_Arm_Inverse_Kinematics((float*)(All_Arm_Angles_Array_Pointer + (i * XYZ_Axis)), \
                          arm_points_1[i * XYZ_Axis + X] - arm_points_2[i * XYZ_Axis + X], \
                          arm_points_1[i * XYZ_Axis + Y] - arm_points_2[i * XYZ_Axis + Y], \
                          arm_points_2[i * XYZ_Axis + Z] - arm_points_1[i * XYZ_Axis + Z], \
                          arm_lengths [i * XY_Axis  + X],                                  \
                          arm_lengths [i * XY_Axis  + Y]);
  }
}