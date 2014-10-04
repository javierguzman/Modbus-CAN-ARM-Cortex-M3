// Author: Francisco Javier Guzman Jimenez, <dejavits@gmail.com>
#ifndef __Modbus_App_H__
#define __Modbus_App_H__

//! \addtogroup App
//! @{

#include "stdint.h"
#include "Modbus_FIFO.h"

#if OSL_Mode
	#include "Modbus_OSL.h"        
	#undef CAN_Mode
        void Modbus_Master_Init(enum Modbus_Comm_Modes Com_Mode, enum Baud Baudrate,
                    unsigned char Attempts, enum Modbus_OSL_Modes Mode);
#elif CAN_Mode
	#include "Modbus_CAN.h"       
	#undef OSL_Mode
        unsigned char Modbus_Master_Init(enum Modbus_CAN_BitRate bit_rate, unsigned char attempts);
#endif

//! Modbus implemented communication modes.
enum Modbus_Comm_Modes
{
    MODBUS_SERIAL, //!< Serial communication
    MODBUS_CAN_MODE,    //!< CAN communication
    CDEFAULT       //!< Serial communication
};
//! @}
              
unsigned char Modbus_Master_Communication (void);//inside is different, header the same
void Modbus_App_Manage_CallBack (void);//inside different, same header
unsigned char Modbus_App_Enqueue_Or_Send(void);//inside different, same header
void Modbus_App_Send(void);//inside different, same header
void Modbus_App_Receive_Char (unsigned char Msg,unsigned char i);
void Modbus_App_L_Msg_Set(unsigned char Index);
void Modbus_App_No_Response(void);
unsigned char Modbus_Get_Error (struct Modbus_FIFO_E_Item *Error);
unsigned char Modbus_App_FIFOSend(void);

unsigned char Modbus_Read_Coils (unsigned char Slave, uint16_t Adress, 
                                 uint16_t Coils, unsigned char *Response);
unsigned char Modbus_Read_D_Inputs (unsigned char Slave, uint16_t Adress, 
                                    uint16_t Inputs, unsigned char *Response);
unsigned char Modbus_Read_H_Registers (unsigned char Slave, uint16_t Adress,
                                       uint16_t Registers, uint16_t *Response);
unsigned char Modbus_Read_I_Registers (unsigned char Slave, uint16_t Adress,
                                       uint16_t Registers, uint16_t *Response);
unsigned char Modbus_Write_Coil (unsigned char Slave, uint16_t Adress,unsigned char Coil);
unsigned char Modbus_Write_Register (unsigned char Slave, uint16_t Adress, uint16_t Register);
unsigned char Modbus_Write_M_Coils (unsigned char Slave, uint16_t Adress,
                                    uint16_t Coils, unsigned char *Value);
unsigned char Modbus_Write_M_Registers (unsigned char Slave, uint16_t Adress,
                                        uint16_t Registers, uint16_t *Value);
unsigned char Modbus_Mask_Write_Register (unsigned char Slave, uint16_t Adress,
                                          uint16_t AND_Mask, uint16_t OR_Mask);
unsigned char Modbus_Read_Write_M_Registers (unsigned char Slave, uint16_t R_Adress,
                                             uint16_t R_Registers, uint16_t *Response,
                                             uint16_t W_Adress, uint16_t W_Registers,
                                             uint16_t *Value);
#endif // __Modbus_App_H__