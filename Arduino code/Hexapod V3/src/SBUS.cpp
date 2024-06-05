#include <Arduino.h>

#include "../Header Files/Config.h"
#include "../Header Files/SBUS.h"

uint8_t SBUS_bytes[SBUS_PacketSize];  // array of bytes, that the SBUS packet is made of

void SBUS_CopyPacket()
{
  Serial.readBytes(SBUS_bytes, SBUS_PacketSize);  //copy SBUS bytes to the SBUS bytes array
  
  while (Serial.available())  Serial.read();  //clear the Rx buffer
}

bool SBUS_Process_Packet(int* Input_Channel_Array)
{
  SBUS_CopyPacket();

  if (SBUS_bytes[0] == 0x0F)  //if the first byte in the packet is 0x0F, the packet is valid, and the receiver is connected
  {
    if ((uint8_t)(SBUS_bytes[23] && 0x00000100) == 0) //if "Frame Lost" isnt active read the SBUS packet, if active, use the Failsafe
    {
      *(Input_Channel_Array + 0) = (((uint16_t)SBUS_bytes[1]       | (uint16_t)SBUS_bytes[2]  << 8) & 0x07FF);
      *(Input_Channel_Array + 1) = (((uint16_t)SBUS_bytes[2]  >> 3 | (uint16_t)SBUS_bytes[3]  << 5) & 0x07FF);
      *(Input_Channel_Array + 2) = (((uint16_t)SBUS_bytes[3]  >> 6 | (uint16_t)SBUS_bytes[4]  << 2  | (uint16_t)SBUS_bytes[5] << 10) & 0x07FF);
      *(Input_Channel_Array + 3) = (((uint16_t)SBUS_bytes[5]  >> 1 | (uint16_t)SBUS_bytes[6]  << 7) & 0x07FF);
      *(Input_Channel_Array + 4) = (((uint16_t)SBUS_bytes[6]  >> 4 | (uint16_t)SBUS_bytes[7]  << 4) & 0x07FF);
      *(Input_Channel_Array + 5) = (((uint16_t)SBUS_bytes[7]  >> 7 | (uint16_t)SBUS_bytes[8]  << 1  | (uint16_t)SBUS_bytes[9] << 9) & 0x07FF);
      *(Input_Channel_Array + 6) = (((uint16_t)SBUS_bytes[9]  >> 2 | (uint16_t)SBUS_bytes[10] << 6) & 0x07FF);
      *(Input_Channel_Array + 7) = (((uint16_t)SBUS_bytes[10] >> 5 | (uint16_t)SBUS_bytes[11] << 3) & 0x07FF);

      for (size_t i = 0; i < SBUS_NumberOfChannels; i++) *(Input_Channel_Array + i) = (((float)Input_Channel_Array[i] - SBUS_rawValueCenter) * (float)SBUS_ConversionRation); //convert raw values to mapped
      
      if ((Input_Channel_Array[0] < SBUS_DeadZone_Channel_1) & (Input_Channel_Array[0] > -SBUS_DeadZone_Channel_1)) *(Input_Channel_Array + 0) = 0;
      if ((Input_Channel_Array[1] < SBUS_DeadZone_Channel_2) & (Input_Channel_Array[1] > -SBUS_DeadZone_Channel_2)) *(Input_Channel_Array + 1) = 0;
      if ((Input_Channel_Array[2] < SBUS_DeadZone_Channel_3) & (Input_Channel_Array[2] > -SBUS_DeadZone_Channel_3)) *(Input_Channel_Array + 2) = 0;
      if ((Input_Channel_Array[3] < SBUS_DeadZone_Channel_4) & (Input_Channel_Array[3] > -SBUS_DeadZone_Channel_4)) *(Input_Channel_Array + 3) = 0;
      if ((Input_Channel_Array[4] < SBUS_DeadZone_Channel_5) & (Input_Channel_Array[4] > -SBUS_DeadZone_Channel_5)) *(Input_Channel_Array + 4) = 0;
      if ((Input_Channel_Array[5] < SBUS_DeadZone_Channel_6) & (Input_Channel_Array[5] > -SBUS_DeadZone_Channel_6)) *(Input_Channel_Array + 5) = 0;
      if ((Input_Channel_Array[6] < SBUS_DeadZone_Channel_7) & (Input_Channel_Array[6] > -SBUS_DeadZone_Channel_7)) *(Input_Channel_Array + 6) = 0;
      if ((Input_Channel_Array[7] < SBUS_DeadZone_Channel_8) & (Input_Channel_Array[7] > -SBUS_DeadZone_Channel_8)) *(Input_Channel_Array + 7) = 0;

      
      return true;
    }
    else
    {
      #ifdef USE_CHANNEL_1_FAILSAFE
      *(Input_Channel_Array + 0) = SBUS_Failsafe_Channel_1;
      #endif
      #ifdef USE_CHANNEL_2_FAILSAFE
      *(Input_Channel_Array + 1) = SBUS_Failsafe_Channel_2;
      #endif
      #ifdef USE_CHANNEL_3_FAILSAFE
      *(Input_Channel_Array + 2) = SBUS_Failsafe_Channel_3;
      #endif
      #ifdef USE_CHANNEL_4_FAILSAFE
      *(Input_Channel_Array + 3) = SBUS_Failsafe_Channel_4;
      #endif
      #ifdef USE_CHANNEL_5_FAILSAFE
      *(Input_Channel_Array + 4) = SBUS_Failsafe_Channel_5;
      #endif
      #ifdef USE_CHANNEL_6_FAILSAFE
      *(Input_Channel_Array + 5) = SBUS_Failsafe_Channel_6;
      #endif
      #ifdef USE_CHANNEL_7_FAILSAFE
      *(Input_Channel_Array + 6) = SBUS_Failsafe_Channel_7;
      #endif
      #ifdef USE_CHANNEL_8_FAILSAFE
      *(Input_Channel_Array + 7) = SBUS_Failsafe_Channel_8;
      #endif
    }
  }
  return false;
}