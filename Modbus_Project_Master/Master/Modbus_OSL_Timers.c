// Author: Francisco Javier Guzman Jimenez, <dejavits@gmail.com>
#ifndef CAN_Mode
//*****************************************************************************
//! \defgroup Timers Modbus Timers
//! \brief Módulo de Interrupciones de los Timers.
//!
//! Para implementar el Modo RTU de las comunicaciones Serie se necesitan dos
//! interrupciones; una que se active en 1,5 veces el tiempo que un carácter
//! tarda en transmitirse y otra en 3,5 veces ese tiempo. Para esto se usan
//! las interrupciones por desborde de cuenta de dos Timers. Aunque este módulo
//! es el que contiene las rutinas de dichas interrupciones, las acciones se
//! realizan llamando a funciones del propio Módulo RTU, puesto que forman
//! parte de sus atribuciones; las rutinas de interrupción sólo resetean el
//! Flag y llaman a la función apropiada.
//!
//! Análogamente, para el reenvío de Peticiones que no reciben respuesta se 
//! establece en OSL un Timeout de respuesta, en función del Baudrate, que dé
//! el tiempo suficiente para que el mensaje sea procesado y la respuesta 
//! enviada. Si la petición es de BroadCast el Timeout será para evitar que se
//! envíen otros mensajes sin dar tiempo a procesar la petición. Estos 2 
//! Timeouts se implementan con la interrupción del Timer 2 puesto que sólo 
//! uno puede estar activo cada vez que se envía un mensaje.
//*****************************************************************************
//! @{

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/timer.h"
#include "Modbus_OSL.h"
#include "Modbus_OSL_RTU.h"

//! \brief Interrupción de 3,5T.
//!
//! Interrupción para asegurar un intervalo de silencio de más de 3,5 caracteres
//! entre distintos mensajes recibidos.
//! \sa Modbus_OSL_RTU_35T
void Timer0IntHandler(void)
{
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    Modbus_OSL_RTU_35T();
}

//! \brief Interrupción de 1,5T.
//!
//! Interrupción para asegurar que los caracteres de un mismo mensaje no se 
//! reciben con intervalos de silencio superiores a 1,5 caracteres.
//! \sa Modbus_OSL_RTU_15T
void Timer1IntHandler(void)
{
    TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
    Modbus_OSL_RTU_15T();
}

//! \brief Interrupción de Respuesta/BroadCast de mensajes.
//!
//! Interrupción para asegurar el reenvío de peticiones normales que no reciban 
//! respuestas correctas (o mensajes de excepción) o para asegurar que las 
//! peticiones BroadCast se han procesado antes de enviar un nuevo mensaje.
//! \sa Modbus_OSL_Timeouts
void Timer2IntHandler(void)
{  
  TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
  Modbus_OSL_Timeouts();
}
//! @}
#endif