#pragma once

#define SBUS_rawValueCenter (SBUS_rawValueMin + ((SBUS_rawValueMax - SBUS_rawValueMin) / 2.0))  //calculates the center of a given range
#define SBUS_ConversionRation ((float)(SBUS_mappedValueMax - SBUS_mappedValueCenter) / (float)(SBUS_rawValueMax - SBUS_rawValueCenter)) //calculates the scaler value, to convert raw to mapped

/**
 * @brief copies the SBUS bytes to an array, and then clears the Rx buffer
 * 
 */
void SBUS_CopyPacket();

/**
 * @brief converts the array of SBUS bytes into 8 channels
 * 
 * @param Input_Channel_Array array into which the channel values get written
 * @return true return true if the receiver is connected
 * -------
 * @return false return false if the receiver is not connected
 */
bool SBUS_Process_Packet(int* Input_Channel_Array);