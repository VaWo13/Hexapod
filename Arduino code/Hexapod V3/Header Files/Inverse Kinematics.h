#pragma once

/**
 * @brief calcultes the angles for a single arm
 * 
 * @param Array_Pointer array in which to store the calculated angles
 * @param delta_x_axis the distance from point 1 to point 2 on the X axis
 * @param delta_y_axis the distance from point 1 to point 2 on the Y axis
 * @param delta_z_axis the distance from point 1 to point 2 on the Z axis
 * @param arm_length_A then length of the A segment of the arm
 * @param arm_length_B then length of the B segment of the arm
 */
void Single_Arm_Inverse_Kinematics(float* Array_Pofloater, float delta_x_axis, float delta_y_axis, float delta_z_axis, int arm_length_A, int arm_length_B);


/**
 * @brief calculates the angles for all arms
 * 
 * @param All_Arm_Angles_Array_Pointer 2D array in which to store all of the calculated angles
 * @param arm_points_1 // 2D array of Points (X/Y/Z) that lie at the shoulder joints
 * @param arm_points_2 // 2D array of Points (X/Y/Z) that lie at the tips of the arms
 * @param arm_lengths  // 2D array of all hexapod arm lengths (A / B)
 */
void all_Arms_inverse_Kinematics(float* All_Arm_Angles_Array_Pofloater, float* arm_pofloats_1, float* arm_pofloats_2, int* arm_lengths);