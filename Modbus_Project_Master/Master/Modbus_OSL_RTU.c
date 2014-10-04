// Author: Francisco Javier Guzman Jimenez, <dejavits@gmail.com>
#ifndef CAN_Mode
//******************************************************************************
//! \defgroup RTU Modbus OSL_RTU 
//! \brief Módulo Modbus Over Serial Line - Remote Terminal Unit mode.
//!
//! Este Módulo se encarga de gestionar el modo RTU de las comunicaciones 
//! Serie de Modbus. En este modo cada Byte transmitido contiene 2 
//! caracteres hexadecimales de 4 bits de modo que permite una mayor densidad
//! de información en las comunicaciones. El formato para cada Byte consta de
//! los 11 bits: los 8 de mensaje, uno de inicio, otro de paridad y el de 
//! finalización; la paridad implementada es la par, indicada por defecto en 
//! las especificaciones de Modbus Over Serial Line.  
//******************************************************************************
//! @{

#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/interrupt.h"
#include "driverlib/uart.h"
#include "driverlib/timer.h"
#include "Modbus_OSL.h"                   
#include "Modbus_OSL_RTU.h"

//*****************************************************************************
//
// Variables locales del módulo OSL_RTU usadas en el desarrollo del programa 
// para comprobaciones de comportamiento del sistema y depuración de errores.
//
//*****************************************************************************
//uint16_t Debug_OSL_RTU_Initial=0, Debug_OSL_RTU_Idle=0,Debug_OSL_RTU_Reception=0,
//         Debug_OSL_RTU_CW=0, Debug_OSL_RTU_Emission=0;

//*****************************************************************************
//
// Variables globales del módulo OSL_RTU.
//
//*****************************************************************************
//! \brief Nº de cuentas para establecer un timer que desborde en el tiempo de  
//! transmisión de 1,5 caracteres (1,5T).
static uint32_t Modbus_OSL_RTU_Timeout_15;
//! \brief Nº de cuentas para establecer un timer que desborde en el tiempo de  
//! transmisión de 3,5 caracteres (3,5T).
static uint32_t Modbus_OSL_RTU_Timeout_35;
//! \brief Divisores calculados para establecer 1,5T en función del Baudrate.
//!
//! Se transmiten 11 bits por carácter, así pues 1,5T es el tiempo transmisión
//! de 16,5 bits, luego como el Baudrate son los bits transmitidos en 1 segundo:
//! > ``Divisor = Baudrate/16,5``
static uint16_t Timeout_15[6] = {
72,145,290,581,1163,1333
};
//! \brief Divisores calculados para establecer 3,5T en función del Baudrate.
//!
//! Se transmiten 11 bits por carácter, así pues 3,5T es el tiempo transmisión
//! de 38,5 bits, luego como el Baudrate son los bits transmitidos en 1 segundo:
//! > ``Divisor = Baudrate/38,5``
static uint16_t Timeout_35[6] = {
32,63,125,250,499,572
};
//! Vector nº1 para almacenar los caracteres recibidos en una trama.
static unsigned char Modbus_OSL_RTU_Msg1[256];
//! Vector nº2 para almacenar los caracteres recibidos en una trama.
static unsigned char Modbus_OSL_RTU_Msg2[256];
//! Puntero para intercalar el vector que almacenará los caracteres recibidos.
static volatile unsigned char *Modbus_OSL_RTU_Msg;
//! Puntero hacia el vector que contenga una trama completa.
static volatile unsigned char *Modbus_OSL_RTU_Msg_Complete;
//! Longitud del Mensaje que contiene una trama completa. Máximo 256 caracteres. 
static volatile unsigned char Modbus_OSL_RTU_L_Msg;
//! Indice de Recepción del mensaje entrante.
static volatile uint16_t Modbus_OSL_RTU_Index;
//! @}

//*****************************************************************************
//
// Prototipos de las funciones locales del módulo OSL_RTU.
//
//*****************************************************************************

static void Modbus_OSL_RTU_Mount_CRC (unsigned char *mb_pdu,unsigned char L_pdu);
static void Modbus_OSL_RTU_Check_CRC (volatile unsigned char *mb_pdu,
                                      unsigned char L_pdu);
static void Modbus_OSL_RTU_Set_Timeout_35 (uint32_t Baudrate);
static void Modbus_OSL_RTU_Set_Timeout_15 (uint32_t Baudrate);

//*****************************************************************************
//! \defgroup RTU_CRC Tratamiento del CRC 
//! \ingroup RTU
//! \brief Funciones encargadas de la Comprobación de Redundancia Cíclica. 
//!
//! Las siguientes funciones se encargan de montar el CRC para los mensajes
//! de Salida y comprobarlo para los mensajes de entrada, siguiendo las  
//! especificaciones y ejemplo de montaje del CRC de Modbus Over Serial Line.
//*****************************************************************************
//! @{

//! Tabla con los valores para el MSB del CRC.
static unsigned char auchCRCHi[ ] = {
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81,
0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40
} ;

//! Tabla con los valores para el LSB del CRC.
static char auchCRCLo[ ] = {
0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4,
0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD,
0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7,
0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE,
0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2,
0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB,
0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 0x50, 0x90, 0x91,
0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88,
0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80,
0x40
};

//! \brief Monta el CRC de un Vector.
//!
//! Con la ayuda de las tablas de valores _auchCRCLo[ ]_ y _auchCRCHi[ ]_
//! monta al final de un vector el CRC correspondiente a los caracteres 
//! del mismo. Basada en el ejemplo propuesto por la especificación.
//! \param *mb_pdu  Puntero al inicio del vector con el mensaje
//! \param L_pdu   Longitud del mensaje, sin CRC
//! \sa auchCRCLo, auchCRCHi
static void Modbus_OSL_RTU_Mount_CRC (unsigned char *mb_pdu,unsigned char L_pdu)
{
  unsigned char uchCRCHi=0xFF, uchCRCLo=0xFF;
  unsigned uIndex;
  
  while (L_pdu--) 
  {
    uIndex = uchCRCLo ^ *mb_pdu++ ; 
    uchCRCLo = uchCRCHi ^ auchCRCHi[uIndex] ;
    uchCRCHi = auchCRCLo[uIndex] ;
  }
  
  mb_pdu[0]=uchCRCLo;
  mb_pdu[1]=uchCRCHi;
}

//! \brief Función para que el Módulo OSL monte el mensaje.
//!
//! Añade el numero de Slave al principio del mensaje de salida y añade al
//! final el CRC con una llamada a _Modbus_OSL_RTU_Mount_CRC_.
//! \param *mb_pdu Puntero a la Trama PDU sin Slave ni CRC 
//! \param Slave   Nº Slave 
//! \param L_pdu   Longitud de la trama PDU
//! \param *mb_adu Puntero al Mensaje completo, listo para enviar
//! \sa Modbus_OSL_RTU_Mount_CRC
void Modbus_OSL_RTU_Mount_ADU (unsigned char *mb_pdu,unsigned char Slave,
                               unsigned char L_pdu, unsigned char *mb_adu)
{
  unsigned char i;
  
  mb_adu[0]=Slave;
  
  for (i=0;i<L_pdu;i++)
    mb_adu[i+1]=mb_pdu[i];
  
  Modbus_OSL_RTU_Mount_CRC (mb_adu,L_pdu+1);
}

//! \brief Comprueba la corrección del CRC de un Vector.
//!
//! Con la ayuda de las tablas de valores _auchCRCLo[ ]_ y _auchCRCHi[ ]_ 
//! calcula el CRC correspondiente a los caracteres del vector y comprueba 
//! si se corresponden con el CRC del mensaje entrante completo. En caso 
//! afirmativo Marca la trama como OK, en caso negativo como NOK.
//! Basada en el ejemplo propuesto por la especificación.
//! \param *mb_pdu  Puntero al inicio del vector con el mensaje
//! \param L_pdu   Longitud del mensaje, sin CRC
//! \sa auchCRCLo, auchCRCHi, Modbus_OSL_Frame_Set
static void Modbus_OSL_RTU_Check_CRC (volatile unsigned char *mb_pdu,
                                      unsigned char L_pdu)
{
  unsigned char uchCRCHi=0xFF,uchCRCLo=0xFF; 
  unsigned uIndex;  
  
  while (L_pdu--) 
  {
    uIndex = uchCRCLo ^ *mb_pdu++ ; 
    uchCRCLo = uchCRCHi ^ auchCRCHi[uIndex] ;  
    uchCRCHi = auchCRCLo[uIndex] ;
  }
  
  if(Modbus_OSL_RTU_Msg_Complete[Modbus_OSL_RTU_L_Msg-1]==uchCRCHi &&
     Modbus_OSL_RTU_Msg_Complete[Modbus_OSL_RTU_L_Msg-2]==uchCRCLo)
        Modbus_OSL_Frame_Set(MODBUS_OSL_Frame_OK);
  else
        Modbus_OSL_Frame_Set(MODBUS_OSL_Frame_NOK);
}

//! \brief Función para que el Módulo OSL pueda comprobar el CRC.
//!
//! Con una llamada a _Modbus_OSL_RTU_Check_CRC_ comprueba el CRC de una
//! trama entrante completada.
//! \return __1__   CRC Correcto
//! \return __0__   CRC Incorrecto
//! \sa Modbus_OSL_RTU_Check_CRC, Modbus_OSL_Frame_Get
unsigned char Modbus_OSL_RTU_Control_CRC(void)
{
  // Modbus_OSL_RTU_L_Msg-2 debido a que los 2 últimos char son el propio CRC.
  Modbus_OSL_RTU_Check_CRC(Modbus_OSL_RTU_Msg_Complete,Modbus_OSL_RTU_L_Msg-2);
  
  if(Modbus_OSL_Frame_Get()==MODBUS_OSL_Frame_OK)
      return 1;
  return 0;
}
//! @}

//*****************************************************************************
//! \defgroup RTU_Manage Gestión del modo RTU
//! \ingroup RTU
//! \brief Funciones para la configuración y manejo de las comunicaciones RTU. 
//!
//! Las funciones siguientes se encargan tanto de configurar los timers para
//! las interrupciones de 1,5T y 3,5T (siendo T el tiempo de transmisión de
//! un carácter), como de gestionar dichas interrupciones y la recepción de  
//! caracteres siguiendo el diagrama de estados que aparece en las 
//! especificaciones del modo de transmisión RTU.
//! ![Diagrama de Estados Modbus Serial RTU](../../RTU.png
//! "Diagrama de Estados Modbus Serial RTU")
//*****************************************************************************
//! @{

//! \brief Establece el Nº de cuentas para que un timer desborde en 1,5T.
//!
//! En función del Baudrate de las comunicaciones Serie almacena en la 
//! variable correspondiente con la ayuda de _Timeout_15_ el Nº de cuentas 
//! necesario para establecer el tiempo de desborde de un timer en 1,5T. Tener
//! en cuenta que _SysCtlClockGet_ devuelve el numero de ciclos por segundo,
//! luego es el numero de cuentas para que desborde en 1 segundo.
//! \param Baudrate Baudrate de las comunicaciones Serie
//! \sa Modbus_OSL_RTU_Timeout_15, Timeout_15
void Modbus_OSL_RTU_Set_Timeout_15 (uint32_t Baudrate)
{ 
  switch (Baudrate) 
  {
    case (1200):
      Modbus_OSL_RTU_Timeout_15=SysCtlClockGet()/Timeout_15[0];
      break;
      
    case (2400):
      Modbus_OSL_RTU_Timeout_15=SysCtlClockGet()/Timeout_15[1];
      break;
      
    case (4800):
      Modbus_OSL_RTU_Timeout_15=SysCtlClockGet()/Timeout_15[2];
      break;
      
    case (9600):
      Modbus_OSL_RTU_Timeout_15=SysCtlClockGet()/Timeout_15[3];
      break;
      
    case (19200):
      Modbus_OSL_RTU_Timeout_15=SysCtlClockGet()/Timeout_15[4];
      break;
      
    default:
      Modbus_OSL_RTU_Timeout_15=SysCtlClockGet()/Timeout_15[5];
      break;     
  }
}

//! \brief Establece el Nº de cuentas para que un timer desborde en 3,5T.
//!
//! En función del Baudrate de las comunicaciones Serie almacena en la 
//! variable correspondiente con la ayuda de _Timeout_35_ el Nº de cuentas 
//! necesario para establecer el tiempo de desborde de un timer en 3,5T. Tener
//! en cuenta que _SysCtlClockGet_ devuelve el numero de ciclos por segundo,
//! luego es el numero de cuentas para que desborde en 1 segundo.
//! \param Baudrate Baudrate de las comunicaciones Serie
//! \sa Modbus_OSL_RTU_Timeout_35, Timeout_35
void Modbus_OSL_RTU_Set_Timeout_35 (uint32_t Baudrate)
{ 
  switch (Baudrate) 
  {
    case (1200):
      Modbus_OSL_RTU_Timeout_35=SysCtlClockGet()/Timeout_35[0];
      break;
      
    case (2400):
      Modbus_OSL_RTU_Timeout_35=SysCtlClockGet()/Timeout_35[1];
      break;
      
    case (4800):
      Modbus_OSL_RTU_Timeout_35=SysCtlClockGet()/Timeout_35[2];
      break;
      
    case (9600):
      Modbus_OSL_RTU_Timeout_35=SysCtlClockGet()/Timeout_35[3];
      break;
      
    case (19200):
      Modbus_OSL_RTU_Timeout_35=SysCtlClockGet()/Timeout_35[4];
      break;
      
    default:
      Modbus_OSL_RTU_Timeout_35=SysCtlClockGet()/Timeout_35[5];
      break;     
  }
}

//! \brief Configura y Arranca las comunicaciones RTU.
//!
//! Establece el puntero de mensajes, el estado RTU al estado inicial y el
//! índice y longitud a 0. Configura dos Timers para habilitar interrupciones,
//! el _Timer 0_ para 3,5T y el _Timer 1_ para 1,5T. Finalmente activa el
//! _Timer 0_ para iniciar el diagrama de estados de RTU.
//! \sa Modbus_OSL_RTU_L_Msg, Modbus_OSL_RTU_Index, Modbus_OSL_RTU_Msg
//! \sa Modbus_OSL_RTU_Msg1, Modbus_OSL_State
void Modbus_OSL_RTU_Init (void) 
{ 
  // Valores iniciales de las variables.
  Modbus_OSL_RTU_L_Msg=0;
  Modbus_OSL_RTU_Index=0;
  Modbus_OSL_RTU_Msg=Modbus_OSL_RTU_Msg1;
    
  // Configura el Estado y las Interrupciones de los Timers.
  Modbus_OSL_State_Set(MODBUS_OSL_RTU_INITIAL); 
  Modbus_OSL_RTU_Set_Timeout_15 (Modbus_OSL_Get_Baudrate());
  Modbus_OSL_RTU_Set_Timeout_35 (Modbus_OSL_Get_Baudrate());
    
  // Activa los periféricos correspondientes.
  SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);

  // Configura los dos timers de 32-bits.
  TimerConfigure(TIMER0_BASE, TIMER_CFG_ONE_SHOT);
  TimerConfigure(TIMER1_BASE, TIMER_CFG_ONE_SHOT);
  TimerLoadSet(TIMER0_BASE, TIMER_A, Modbus_OSL_RTU_Timeout_35);      
  TimerLoadSet(TIMER1_BASE, TIMER_A, Modbus_OSL_RTU_Timeout_15);        
    
  // Activa las interrupciones para los Timeouts de los timers.
  IntEnable(INT_TIMER0A);
  IntEnable(INT_TIMER1A);
  TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
  TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
   
  // Activa el timer de 3,5T.
  TimerEnable(TIMER0_BASE, TIMER_A);
}

//! \brief Función para la interrupción de 1,5T.
//!
//! Las interrupciones de 1,5T y 3,5T se utilizan en el diagrama de estados RTU
//! como triggers para el cambio de estado. El programa esta implementado
//! de modo que esta interrupción sólo puede saltar en el estado del diagrama
//! MODBUS_OSL_RTU_RECEPTION. Se  vuelve a cargar el valor de cuentas del timer
//! y se cambia el estado a MODBUS_OSL_RTU_CONTROLANDWAITING.
//! \sa Modbus_OSL_State_Get, Modbus_OSL_RTU_Timeout_15
//! \sa Modbus_OSL_State, Modbus_OSL_State_Set
void Modbus_OSL_RTU_15T (void) 
{
  switch (Modbus_OSL_State_Get())
  {
    case MODBUS_OSL_RTU_RECEPTION:    
      Modbus_OSL_State_Set (MODBUS_OSL_RTU_CONTROLANDWAITING);
      TimerLoadSet(TIMER1_BASE, TIMER_A, Modbus_OSL_RTU_Timeout_15);
      break;
     
    default:
      Modbus_Fatal_Error(200);
      break;
  }
}

//! \brief Función para la interrupción de 3,5T.
//!
//! Las interrupciones de 1,5T y 3,5T se utilizan en el diagrama de estados RTU
//! como triggers para el cambio de estado. Esta interrupción vuelve a cargar
//! el valor de cuentas del timer y realiza las siguientes acciones en función
//! del estado actual:
//! > - __MODBUS_OSL_RTU_INITIAL__: Cambia el estado a MODBUS_OSL_RTU_IDLE y el
//! >     estado principal MODBUS_OSL_IDLE.
//! > - __MODBUS_OSL_RTU_CONTROLANDWAITING__: Si no se han detectado errores de 
//! >     paridad, exceso de caracteres o Timeout de Respuesta (Master), activa
//! >     el flag de Trama completa mediante _Modbus_OSL_Reception_Complete_ y
//! >     apunta _Modbus_OSL_RTU_Msg_Complete_ hacia el mensaje; almacenando la
//! >     longitud en  _Modbus_OSL_RTU_L_Msg_; el puntero _Modbus_OSL_RTU_Msg_
//! >     cambia el vector al que apunta para recibir nuevos mensajes. En caso
//! >     contrario el mensaje se descarta. Se reinician las variables para 
//! >     poder recibir un nuevo mensaje, y se vuelve a MODBUS_OSL_RTU_IDLE.
//! > - __MODBUS_OSL_RTU_EMISSION__: Vuelve a MODBUS_OSL_RTU_IDLE.
//! \sa Modbus_OSL_RTU_Msg, Modbus_OSL_RTU_Msg1, Modbus_OSL_RTU_Msg2
//! \sa Modbus_OSL_RTU_Msg_Complete, Modbus_OSL_RTU_Index, Modbus_OSL_RTU_L_Msg 
//! \sa Modbus_OSL_State, Modbus_OSL_MainState, Modbus_OSL_Reception_Complete
void Modbus_OSL_RTU_35T (void) 
{
  switch (Modbus_OSL_State_Get())
  {
      
    case MODBUS_OSL_RTU_INITIAL:
      // Cambiar a IDLE y recargar Timer 0.
      Modbus_OSL_State_Set (MODBUS_OSL_RTU_IDLE);   
      Modbus_OSL_MainState_Set (MODBUS_OSL_IDLE);
      TimerLoadSet(TIMER0_BASE, TIMER_A, Modbus_OSL_RTU_Timeout_35);
      break;

    case MODBUS_OSL_RTU_CONTROLANDWAITING:
      // Comprobar Trama (paridad, timeout respuesta en master)
      // Configurar/Resetear Variables; Recargar Timer0 y volver a IDLE.
      IntDisable(INT_UART1);
      if(Modbus_OSL_Frame_Get()==MODBUS_OSL_Frame_OK  &&
         Modbus_OSL_MainState_Get()!=MODBUS_OSL_ERROR)
      {
        if(Modbus_OSL_RTU_Msg==Modbus_OSL_RTU_Msg1)
        {
          Modbus_OSL_RTU_Msg=Modbus_OSL_RTU_Msg2;
          Modbus_OSL_RTU_Msg_Complete=Modbus_OSL_RTU_Msg1;     
        }      
        else
        {
          Modbus_OSL_RTU_Msg=Modbus_OSL_RTU_Msg1;        
          Modbus_OSL_RTU_Msg_Complete=Modbus_OSL_RTU_Msg2;
        }
              
        Modbus_OSL_RTU_L_Msg=Modbus_OSL_RTU_Index;
        Modbus_OSL_Reception_Complete();    
      }  
      Modbus_OSL_Frame_Set(MODBUS_OSL_Frame_OK);
      Modbus_OSL_RTU_Index=0;
      Modbus_OSL_State_Set (MODBUS_OSL_RTU_IDLE);
      IntEnable(INT_UART1);
      TimerLoadSet(TIMER0_BASE, TIMER_A, Modbus_OSL_RTU_Timeout_35);
      break;
      
      
    case MODBUS_OSL_RTU_EMISSION:   
      Modbus_OSL_State_Set (MODBUS_OSL_RTU_IDLE);
      TimerLoadSet(TIMER0_BASE, TIMER_A, Modbus_OSL_RTU_Timeout_35);
      break;
      
    default: 
      Modbus_Fatal_Error(210);
      break;
  }
}

//! \brief Función para la interrupción por Recepción en modo RTU.
//!
//! Según el estado en que se encuentre el programa en el momento de recibir
//! un carácter se realizan distintas acciones acordes al diagrama de estados
//! RTU. Las posibilidades son:
//! > - __MODBUS_OSL_RTU_INITIAL__: Se descarta el carácter y se resetea el
//! >     _Timer 0_ en espera que desborde sin recepción de caracteres.
//! > - __MODBUS_OSL_RTU_IDLE__: Almacenar el carácter,aumentar el indice de
//! >     recepción, activar ambos Timers y pasar a _MODBUS_OSL_RTU_RECEPTION_
//! > - __MODBUS_OSL_RTU_RECEPTION__: Almacenar el carácter,aumentar el indice 
//! >     de recepción y recargar la cuenta de ambos Timers que al estar aun 
//! >     activados empezaran la cuenta entera de nuevo. Si se excede el índice
//! >     máximo por trama de 255 (0-255), se marca la trama como NOK.
//! > - __MODBUS_OSL_RTU_CONTROLANDWAITING__: Descartar el carácter y marcar
//! >     la trama como NOK
//! > - __MODBUS_OSL_RTU_EMISSION__: No se debería recibir en este estado; por 
//! >     mera cuestión de robustez en la programación se descarta el carácter.
//! \sa Modbus_OSL_RTU_Msg, Modbus_OSL_RTU_Index, Modbus_OSL_State 
//! \sa Modbus_OSL_Frame_Set, Modbus_OSL_Frame
void Modbus_OSL_RTU_UART(void)
{
  switch (Modbus_OSL_State_Get())
  {         
    case MODBUS_OSL_RTU_INITIAL:    
      //Debug_OSL_RTU_Initial++;
      UARTCharGetNonBlocking(UART1_BASE);
      TimerLoadSet(TIMER0_BASE, TIMER_A, Modbus_OSL_RTU_Timeout_35);
      break;
                
    case MODBUS_OSL_RTU_IDLE:
      //Debug_OSL_RTU_Idle++;
      Modbus_OSL_RTU_Msg[Modbus_OSL_RTU_Index]=UARTCharGetNonBlocking(UART1_BASE);
      IntDisable(INT_TIMER1A);
      IntDisable(INT_TIMER0A);
      TimerEnable(TIMER1_BASE, TIMER_A);
      TimerEnable(TIMER0_BASE, TIMER_A); 
      Modbus_OSL_RTU_Index++;
      Modbus_OSL_State_Set (MODBUS_OSL_RTU_RECEPTION);
      IntEnable(INT_TIMER1A);
      IntEnable(INT_TIMER0A);
      break;
            
    case MODBUS_OSL_RTU_RECEPTION:
      //Debug_OSL_RTU_Reception++;
      
      if(Modbus_OSL_RTU_Index>255)
          Modbus_OSL_Frame_Set(MODBUS_OSL_Frame_NOK);
  
      Modbus_OSL_RTU_Msg[Modbus_OSL_RTU_Index]=UARTCharGetNonBlocking(UART1_BASE);
      TimerLoadSet(TIMER0_BASE, TIMER_A, Modbus_OSL_RTU_Timeout_35);
      TimerLoadSet(TIMER1_BASE, TIMER_A, Modbus_OSL_RTU_Timeout_15);
      Modbus_OSL_RTU_Index++;
      break;
            
    case MODBUS_OSL_RTU_CONTROLANDWAITING:
      //Debug_OSL_RTU_CW++;
      Modbus_OSL_Frame_Set(MODBUS_OSL_Frame_NOK);
      UARTCharGetNonBlocking(UART1_BASE);
      break;
            
    case MODBUS_OSL_RTU_EMISSION:
      //Debug_OSL_RTU_Emission++;
      UARTCharGetNonBlocking(UART1_BASE);
      break;
  }
}
//! @}

//*****************************************************************************
//! \defgroup RTU_Exchange Intercambio con OSL
//! \ingroup RTU
//! \brief Funciones de Intercambio de Datos con el módulo OSL. 
//!
//! Engloba funciones para el intercambio de datos de información entre los
//! módulos OSL y OSL_RTU permitiendo a OSL la lectura de mensajes entrantes 
//! completos y su gestión.
//*****************************************************************************
//! @{

//! \brief Permite al Módulo OSL consultar _Modbus_OSL_RTU_Timeout_35_.
//!
//! Función que permite conocer _Modbus_OSL_RTU_Timeout_35_ 
//! desde módulos distintos a OSL_RTU. 
//! \return Modbus_OSL_RTU_Timeout_35 Nº de cuentas para 3,5T
//! \sa Modbus_OSL_RTU_Timeout_35
uint32_t Modbus_OSL_RTU_Get_Timeout_35 (void)
{  
  return(Modbus_OSL_RTU_Timeout_35);
}

//! \brief Devuelve un carácter del mensaje entrante en RTU.
//!
//! Permite a OSL obtener el carácter de índice `i` en un mensaje  
//! entrante completo en RTU.
//! \param i Indice en la Trama entrante completa del carácter a devolver 
//! \return Modbus_OSL_RTU_Msg_Complete[i] Carácter Nº `i` del Mensaje
//! \sa Modbus_OSL_RTU_Msg_Complete
unsigned char Modbus_OSL_RTU_Char_Get (unsigned char i)
{
  return Modbus_OSL_RTU_Msg_Complete[i];
}

//! \brief Devuelve la longitud de un Mensaje Entrante para OSL.
//!
//! Permite a OSL obtener la longitud de un Mensaje entrante completo sin el  
//! CRC; puesto que una vez comprobado ya no es necesario. 
//! \return Modbus_OSL_RTU_L_Msg-2 Longitud de la Trama sin contar el CRC
//! \sa Modbus_OSL_RTU_L_Msg
unsigned char Modbus_OSL_RTU_L_Msg_Get(void)
{
  return Modbus_OSL_RTU_L_Msg-2;
}
//! @}
#endif