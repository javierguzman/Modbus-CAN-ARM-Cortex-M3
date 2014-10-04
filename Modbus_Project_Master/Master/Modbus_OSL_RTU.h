// Author: Francisco Javier Guzman Jimenez, <dejavits@gmail.com>
#ifndef CAN_Mode
#ifndef __Modbus_OSL_RTU_H__
#define __Modbus_OSL_RTU_H__

#include "stdint.h"

void Modbus_OSL_RTU_Mount_ADU (unsigned char *mb_pdu,unsigned char Slave,
                               unsigned char L_pdu, unsigned char *mb_adu);
unsigned char Modbus_OSL_RTU_Control_CRC(void);

void Modbus_OSL_RTU_Init (void); 
void Modbus_OSL_RTU_15T (void);
void Modbus_OSL_RTU_35T (void);
void Modbus_OSL_RTU_UART(void);

uint32_t Modbus_OSL_RTU_Get_Timeout_35 (void);
unsigned char Modbus_OSL_RTU_Char_Get(unsigned char i);
unsigned char Modbus_OSL_RTU_L_Msg_Get(void);

#endif // __Modbus_OSL_H__
#endif