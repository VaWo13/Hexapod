#pragma once

void move_Gate(int* input_Channels, float* all_Arm_Points_2, bool center_arms);

void move_Arms_Towards_Target(float* all_Arm_Points_2);

void move_main_Chassis(int* input_Channels,float* all_Arm_Points_1, float* all_Arm_Points_2,float* all_Arm_Points_2_rotated, float smoothing);


float *QuaternionProduct(float *q1, float *q2);
float *QuaternionInverse(float *q1);
float *QuaternionNormalize(float *q1);