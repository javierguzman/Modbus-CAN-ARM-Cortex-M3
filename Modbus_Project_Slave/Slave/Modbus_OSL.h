// Author: Francisco Javier Guzman Jimenez, <dejavits@gmail.com>
#ifndef CAN_Mode
#ifndef __Modbus_OSL_H__
#define __Modbus_OSL_H__

//! \addtogroup OSL
//! @{

#include "stdint.h"

//!Maximum PDU DATA OSL
#define MAX_PDU 253

//! Baudrates implementados para las comunicaciones.
enum Baud
{
    B1200   = 1200,   //!< 1200 Bps
    B2400   = 2400,   //!< 2400 Bps
    B4800   = 4800,   //!< 4800 Bps
    B9600   = 9600,   //!< 9600 Bps
    B19200  = 19200,  //!< 19200 Bps
    B38400  = 38400,  //!< 38400 Bps
    B57600  = 57600,  //!< 57600 Bps
    B115200 = 115200, //!< 115200 Bps
    BDEFAULT          //!< 19200 Bps
};

//! Modos de Comunicación por el puerto Serie.
enum Modbus_OSL_Modes 
{
    MODBUS_OSL_MODE_RTU,    //!< RTU: Remote Terminal Unit
    MODBUS_OSL_MODE_ASCII,  //!< ASCII (no implementado)
    MDEFAULT                //!< RTU  
};

//! Estados del diagrama de comportamiento del Slave.
enum Modbus_OSL_MainStates
{
    MODBUS_OSL_INITIAL,       //!< Estado Inicial
    MODBUS_OSL_IDLE,          //!< Estado Idle
    MODBUS_OSL_CHECKING,      //!< Estado Checking Request
    MODBUS_OSL_PROCESSING,    //!< Estado Processing required Action 
    MODBUS_OSL_REPLY,         //!< Estado Formatting normal Reply
    MODBUS_OSL_ERROR          //!< Estado Formatting Error Reply
};

//! Estados del diagrama de modo de Transmisión RTU/ASCII.
enum Modbus_OSL_States 
{ 
    // RTU
    MODBUS_OSL_RTU_INITIAL,            //!< __RTU__: Estado Initial State
    MODBUS_OSL_RTU_IDLE,               //!< __RTU__: Estado Idle
    MODBUS_OSL_RTU_RECEPTION,          //!< __RTU__: Estado Reception
    MODBUS_OSL_RTU_CONTROLANDWAITING,  //!< __RTU__: Estado Control and Waiting
    MODBUS_OSL_RTU_EMISSION,           //!< __RTU__: Estado Emission
    
    // ASCII
    MODBUS_OSL_ASCII_IDLE,             //!< __ASCII__: Estado Idle
    MODBUS_OSL_ASCII_RECEPTION,        //!< __ASCII__: Estado Reception
    MODBUS_OSL_ASCII_WAITING_EOF,      //!< __ASCII__: Estado Waiting "End of Frame"
    MODBUS_OSL_ASCII_EMISSION_START,   //!< __ASCII__: Estado Emission Start
    MODBUS_OSL_ASCII_EMISSION,         //!< __ASCII__: Estado Emission
    MODBUS_OSL_ASCII_EMISSION_END      //!< __ASCII__: Estado Emission End
};

//! Estados de la Corrección de una trama de mensaje entrante.
enum Modbus_OSL_Frames 
{
    MODBUS_OSL_Frame_OK,     //!< Trama Correcta. 
    MODBUS_OSL_Frame_NOK     //!< Error en la Trama. Por CRC/LRC, paridad, exceso
                             //!< de caracteres o recepción en Control and Waiting
};
//! @}

uint32_t Modbus_OSL_Get_Baudrate(void);
enum Modbus_OSL_Frames Modbus_OSL_Frame_Get (void);
void Modbus_OSL_Frame_Set (enum Modbus_OSL_Frames Flag);
enum Modbus_OSL_States Modbus_OSL_State_Get (void);
void Modbus_OSL_State_Set (enum Modbus_OSL_States State);
enum Modbus_OSL_MainStates Modbus_OSL_MainState_Get (void);
void Modbus_OSL_MainState_Set (enum Modbus_OSL_MainStates State);
unsigned char Modbus_OSL_BroadCast_Get(void);

unsigned char Modbus_OSL_Init (unsigned char Slave, enum Baud Baudrate,
                              enum Modbus_OSL_Modes Mode);
void Modbus_OSL_Serial_Comm (void);
void Modbus_Fatal_Error(unsigned char Error);

void Modbus_OSL_Reception_Complete (void);

void Modbus_OSL_Output (unsigned char *mb_rsp_pdu, unsigned char L_pdu);


#endif // __Modbus_OSL_H__
#endif