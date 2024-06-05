#pragma once

/**
 * @brief initializes all servos and the PWM driver board, and then, one after the other(reduce surge current), sets them to the calculated angel
 * 
 * @param Angle_Array the 2D array of all the calculated servo angles
 */
void init_Servos(float* Angle_Array);

/**
 * @brief sets all the servos to the calulated angles, by converting deg° to microseconds
 * 
 * @param Angle_Array the 2D array of all the calculated servo angles
 */
void Update_Servo_Positions(float* Angle_Array);