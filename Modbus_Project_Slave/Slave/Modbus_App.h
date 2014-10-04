// Author: Francisco Javier Guzman Jimenez, <dejavits@gmail.com>
#ifndef __Modbus_App_H__
#define __Modbus_App_H__

//! \addtogroup App
//! @{

#include "stdint.h"

//! Communication modes of Modbus. OSL and CAN are the only ones implemented for now.
enum Modbus_Comm_Modes 
{
    MODBUS_SERIAL, //!< Serial communication
    MODBUS_CAN_MODE,    //!< CAN communication
    CDEFAULT       //!< Serial communication
};
//! @}

#if OSL_Mode
	#include "Modbus_OSL.h"
	#undef CAN_Mode
		unsigned char Modbus_Slave_Init(uint16_t N_Coils, unsigned char *Coils,
                                uint16_t N_D_Inputs, unsigned char *D_Inputs,
                                uint16_t N_H_Registers, uint16_t *H_Registers,
                                uint16_t N_I_Registers, uint16_t *I_Registers,
                                enum Modbus_Comm_Modes Com_Mode, 
                                unsigned char Slave, enum Baud Baudrate,
                                enum Modbus_OSL_Modes Mode);
#elif CAN_Mode
	#include "Modbus_CAN.h"
	#undef OSL_Mode
                unsigned char Modbus_Slave_Init(uint16_t N_Coils, unsigned char *Coils,
                                uint16_t N_D_Inputs, unsigned char *D_Inputs,
                                uint16_t N_H_Registers, uint16_t *H_Registers,
                                uint16_t N_I_Registers, uint16_t *I_Registers,
                                enum Modbus_CAN_BitRate bit_rate, unsigned char slave);
#endif

void Modbus_Slave_Communication (void);//
void Modbus_App_Manage_Request (void);
void Modbus_App_Receive_Char (unsigned char Msg,unsigned char i);
void Modbus_App_L_Msg_Set(unsigned char Index);
void Modbus_App_Send(void);

#endif // __Modbus_App_H__