// Author: Francisco Javier Guzman Jimenez, <dejavits@gmail.com>
#ifndef CAN_Mode
//******************************************************************************
//! \defgroup OSL Modbus OSL
//! \brief Módulo Modbus Over Serial Line.
//!
//! Este Módulo se encarga de implementar las comunicaciones Serie de Modbus, 
//! como un sistema de comunicación entre un Master y uno o varios Slaves.
//! Existen 2 opciones de comunicación; el Master envía a peticiones a un Slave
//! y éste realiza las acciones demandadas y responde o se envía una petición
//! a _todos_ los Slaves (BroadCast) sin que éstos emitan respuesta alguna. En
//! ningún caso se comunican los Slaves entre ellos.
//!
//! Por una parte, éste módulo recoge los mensajes entrantes de los módulos 
//! correspondientes a los modos de comunicación Serie (de momento sólo el RTU 
//! está implementado aunque el programa se encuentra perfectamente adaptado
//! para la adición de un módulo OSL_ASCII); comprueba su corrección y los 
//! envía al nivel de Aplicación Modbus App para el proceso de las acciones
//! correspondientes. 
//!
//! Por otro lado gestiona también la Salida de mensajes del Sistema añadiendo
//! a la trama proveniente de App (PDU) el Numero de Slave y el checksum 
//! (CRC/LRC) y traduciendo el mensaje a ASCII, si ese es el modo activo, para
//! formar la trama de mensaje completa o Application Data Unit (ADU).
//******************************************************************************
//! @{

#include "inc/lm3s8962.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/uart.h"
#include "driverlib/timer.h"
#include "Modbus_App.h"
#include "Modbus_OSL.h"                   
#include "Modbus_OSL_RTU.h"

//*****************************************************************************
//
// Variables locales del módulo OSL usadas en el desarrollo del programa para
// la comprobación del comportamiento del sistema y la depuración de errores.
//
//*****************************************************************************

//uint16_t Debug_OSL_OutChar=0,Debug_OSL_IncChar=0;
//unsigned char Debug_OSL_OutMsg=0, Debug_OSL_IncMsg=0, Debug_OSL_CRC_OK=0;

//*****************************************************************************
//
// Variables globales del módulo OSL.
//
//*****************************************************************************

// Para configurar las comunicaciones.

//! \brief Baudrate de las comunicaciones Serie. Su valor debe corresponder con 
//! uno de los contenidos en _enum_ _Baud_. Por defecto es 19200 bps.
static uint32_t Modbus_OSL_Baudrate;
//! Modo de las comunicaciones Serie, RTU o ASCII. Por defecto RTU. 
static volatile enum Modbus_OSL_Modes Modbus_OSL_Mode;

// Para datos de Mensaje y Flags del Sistema.

//! Marca los mensajes entrantes como MODBUS_OSL_Frame_OK/MODBUS_OSL_Frame_NOK.
static volatile enum Modbus_OSL_Frames Modbus_OSL_Frame;
//! Flag de Mensaje entrante Completo.
static volatile unsigned char Modbus_OSL_Processing_Flag;
//! Variable propia del Slave que contiene su Nº de identificación.
static unsigned char Modbus_OSL_Slave_Adress;
//! Vector para almacenar los mensajes de Salida en el Slave.
static unsigned char Modbus_OSL_Response_ADU[256];
//! Longitud del mensaje de Salida en el Slave.
static unsigned char Modbus_OSL_L_Response_ADU;
//! Flag de Broadcast; se activa para evitar el envío de respuesta en el Slave.
static unsigned char Modbus_OSL_BroadCast;

// Para los distintos estados de los diagramas de Slave y RTU.

//! Estado del Sistema en el diagrama de Slave.
static volatile enum Modbus_OSL_MainStates Modbus_OSL_MainState;
//! Estado del Sistema en el diagrama RTU o ASCII.
static volatile enum Modbus_OSL_States Modbus_OSL_State;
//! @}

//*****************************************************************************
//
// Prototipos de las funciones locales del módulo OSL.
//
//*****************************************************************************

static unsigned char Modbus_OSL_Processing_Msg(void);
static void Modbus_OSL_RTU_to_App (void);
static void Modbus_OSL_Send (unsigned char *mb_rsp_pdu, unsigned char L_pdu);
static unsigned char Modbus_OSL_Receive_Request(void);

//*****************************************************************************
//! \defgroup OSL_Var Gestión de Variables 
//! \ingroup OSL
//! \brief Funciones para consultar/modificar variables desde otros módulos. 
//!
//! Los módulos App y OSL_RTU necesitan en algunos casos conocer o modificar 
//! los valores de ciertas variables del Módulo OSL, como los Estados del 
//! Sistema, el Baudrate, la corrección de la Trama o el flag de Broadcast. Las
//! llamada a las siguientes funciones lo permite sin externalizar dichas
//! variables.
//*****************************************************************************
//! @{

//! \brief Obtiene el Baudrate del Sistema.
//!
//! \return Modbus_OSL_Baudrate Baudrate del Sistema
//! \sa Modbus_OSL_Baudrate, enum Baud
uint32_t Modbus_OSL_Get_Baudrate(void)
{  
  return(Modbus_OSL_Baudrate);
}

//! \brief Obtiene el Estado de corrección de la trama entrante.
//!
//! El mensaje entrante tiene marcado en _Modbus_OSL_Frame_ si la trama 
//! recibida es correcta o bien se ha detectado algún error, bien sea por
//! paridad, exceso de caracteres o error en el CRC; esta función permite
//! conocer dicho estado.
//! \return Modbus_OSL_Frame Puede ser MODBUS_OSL_Frame_OK/MODBUS_OSL_Frame_NOK
//! \sa Modbus_OSL_Frame, Modbus_OSL_Frame_Set, enum Modbus_OSL_Frames
enum Modbus_OSL_Frames Modbus_OSL_Frame_Get(void)
{
  return Modbus_OSL_Frame;
}

//! \brief Fija el Estado de corrección de la trama entrante.
//!
//! El mensaje entrante tiene marcado en _Modbus_OSL_Frame_ si la trama 
//! recibida es correcta o bien se ha detectado algún error, bien sea por
//! paridad, exceso de caracteres o error en el CRC; esta función permite
//! marcar el valor de dicho estado para modificarlo desde otro módulo.
//! \param Flag Puede ser MODBUS_OSL_Frame_OK/MODBUS_OSL_Frame_NOK
//! \sa Modbus_OSL_Frame, Modbus_OSL_Frame_Get, enum Modbus_OSL_Frames
void Modbus_OSL_Frame_Set(enum Modbus_OSL_Frames Flag)
{
  Modbus_OSL_Frame = Flag;
}

//! \brief Obtiene el Estado del diagrama RTU/ASCII.
//!
//! Esta función permite conocer el estado del diagrama de comunicaciones Serie
//! RTU o ASCII, dependiendo de que modo de comunicaciones serie se esté usando.
//! \return Modbus_OSL_State Estado del Diagrama de Estados RTU/ASCII
//! \sa Modbus_OSL_State, Modbus_OSL_State_Set, enum Modbus_OSL_States 
enum Modbus_OSL_States Modbus_OSL_State_Get(void)
{
  return Modbus_OSL_State;
}

//! \brief Cambia el Estado del diagrama RTU/ASCII.
//!
//! Esta función permite fijar o cambiar el estado del diagrama de 
//! comunicaciones Serie RTU o ASCII, dependiendo del modo que se esté usando. 
//! \param State Estado del Diagrama de Estados RTU/ASCII a escribir
//! \sa Modbus_OSL_State, Modbus_OSL_State_Get, enum Modbus_OSL_States 
void Modbus_OSL_State_Set(enum Modbus_OSL_States State)
{
  Modbus_OSL_State = State;
}

//! \brief Obtiene el Estado del diagrama del Slave.
//!
//! \return Modbus_OSL_MainState Estado del Diagrama de Estados del Slave
//! \sa Modbus_OSL_MainState, Modbus_OSL_MainState_Set, enum Modbus_OSL_MainStates 
enum Modbus_OSL_MainStates Modbus_OSL_MainState_Get (void)
{
   return Modbus_OSL_MainState;
}

//! \brief Cambia el Estado del diagrama del Slave.
//!
//! Esta función permite fijar o cambiar el estado del diagrama de 
//! comunicaciones del Slave. 
//! \param State Estado del Diagrama de Estados del Slave a escribir
//! \sa Modbus_OSL_MainState, Modbus_OSL_MainState_Get, enum Modbus_OSL_MainStates 
void Modbus_OSL_MainState_Set (enum Modbus_OSL_MainStates State)
{
  Modbus_OSL_MainState = State;
}

//! \brief Obtiene el Estado del Flag de BroadCast.
//!
//! Esta función permite conocer el estado del Flag de Broadcast para evitar la
//! respuesta de un Slave a una petición BroadCast.
//! \return Modbus_OSL_BroadCast Activado si la peticion recibida es BroadCast 
//! \sa Modbus_OSL_BroadCast, Modbus_OSL_Receive_Request
unsigned char Modbus_OSL_BroadCast_Get(void)
{
  return Modbus_OSL_BroadCast;
}
//! @}

//*****************************************************************************
//! \defgroup OSL_Manage Gestión de comunicaciones Serie
//! \ingroup OSL
//! \brief Funciones para la configuración y manejo de las comunicaciones Serie. 
//!
//! Las funciones siguientes se encargan tanto de configurar el sistema para 
//! comunicaciones por puerto Serie, gestionando la interrupción de la UART1
//! usada en esas comunicaciones, como de Gestionar la Recepción/Envío de los
//! mensajes siguiendo el diagrama de comportamiento del Slave de las 
//! especificaciones del protocolo Modbus sobre puerto Serie.
//! ![Diagrama de Comportamiento del Slave](../../Slave.png "Diagrama de Comportamiento del Slave")
//*****************************************************************************
//! @{

//! \brief Configura las comunicaciones Serie.
//!
//! Establece el Nº de identificación del Slave, el modo de comunicación RTU o
//! ASCII y el Baudrate e inicia el Estado de Comportamiento y los flags de 
//! Mensaje entrante y corrección de la trama a sus valores iniciales.
//! Configura la UART1 según el modo RTU/ASCII para cumplir sus respectivas 
//! especificaciones y configura el LED1 para que se encienda al transmitir y
//! recibir datos. Finalmente llama a la función de configuración e inicio del 
//! modo de comunicación RTU o ASCII.
//! \param Slave Nº de identificación del Slave
//! \param Baudrate Baudrate con que iniciar las comunicaciones.
//! \param Mode Modo de comunicación en Serie, RTU (por defecto) o ASCII
//! \return 1 Nº Slave demasiado alto; no se realiza la configuración
//! \return 0 Todo correcto
//! \sa enum Baud, enum Modbus_OSL_Modes,Modbus_OSL_RTU_Init
unsigned char Modbus_OSL_Init (unsigned char Slave, enum Baud Baudrate, 
                               enum Modbus_OSL_Modes Mode)
{
      if (Slave>247)
        return 1;
      
    // Pone a 0 el flag de inicio de mensaje y empieza la trama como OK.
    Modbus_OSL_Slave_Adress=Slave;
    Modbus_OSL_Processing_Flag=0;
    Modbus_OSL_Frame_Set(MODBUS_OSL_Frame_OK);    
    
    // Establece el Baudrate del sistema, por defecto 19200.
    if (Baudrate == BDEFAULT) 
      Modbus_OSL_Baudrate=B19200;
    else
      Modbus_OSL_Baudrate=Baudrate;
    
    Modbus_OSL_MainState=MODBUS_OSL_INITIAL;
    
    if (Mode == MDEFAULT || Mode == MODBUS_OSL_MODE_RTU)
    {
      Modbus_OSL_Mode=MODBUS_OSL_MODE_RTU;
    }
    else
    {
      Modbus_OSL_Mode=MODBUS_OSL_MODE_ASCII;    
    }
    
    // Habilita los periféricos de la UART y los pins usados para las
    // comunicaciones. Se utiliza la UART1, que requiere pins del puerto GPIOD. 
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    
    // Habilita las interrupciones del sistema.
    IntMasterEnable();
    
    // Fija GPIO D2 y D3 como los pins de la UART1.
    GPIOPinTypeUART(GPIO_PORTD_BASE, GPIO_PIN_2 | GPIO_PIN_3);   
    
    switch(Modbus_OSL_Mode)
    {
      case MODBUS_OSL_MODE_RTU:
        // Configura la UART1 para el Baudrate, 8-Par-1.
        UARTConfigSetExpClk(UART1_BASE, SysCtlClockGet(), Modbus_OSL_Baudrate,
                           (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                            UART_CONFIG_PAR_EVEN));
        break;
      case MODBUS_OSL_MODE_ASCII:
                // Configura la UART1 para el Baudrate, 7-Par-1.
        UARTConfigSetExpClk(UART1_BASE, SysCtlClockGet(), Modbus_OSL_Baudrate,
                           (UART_CONFIG_WLEN_7 | UART_CONFIG_STOP_ONE |
                            UART_CONFIG_PAR_EVEN));
        break;
    }
    
    // Desactiva la cola FIFO de la UART1 para asegurar que las interrupciones 
    // salten por cada carácter recibido.
    UARTFIFODisable(UART1_BASE);
    
    // Habilita el puerto GPIO usado para el LED1.
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOF;
    // Lectura aleatoria para fijar unos pocos ciclos al activar el periférico.
    volatile unsigned long ulLoop = SYSCTL_RCGC2_R;
    // Habilita el pin GPIO para el LED (PF0) y fija la dirección como salida.
    GPIO_PORTF_DIR_R = 0x01;
    GPIO_PORTF_DEN_R = 0x01;
    
    // Habilita la interrupción de la UART, para Recepción y error de paridad.
    UARTIntEnable(UART1_BASE, UART_INT_RX | UART_INT_PE);
    IntEnable(INT_UART1);
    
    switch(Modbus_OSL_Mode)
    {
      case MODBUS_OSL_MODE_RTU:
        Modbus_OSL_RTU_Init();
        break;
      case MODBUS_OSL_MODE_ASCII:
        break;
    }
    return 0;
}

//! \brief Interrupción por Recepción de Carácter.
//! 
//! Esta función se activa con la interrupción de la UART1. Enciende el LED1 
//! para indicar que se esta produciendo la comunicación, limpia el status de
//! la interrupción y comprueba si es de error de paridad para marcar la trama
//! como NOK; en caso contrario llama a la función de interrupción RTU/ASCII  
//! que corresponda según el modo de comunicación Serie.
//! \sa Modbus_OSL_Frame_Set, Modbus_OSL_Mode, Modbus_OSL_RTU_UART
void UART1IntHandler(void)
{
    unsigned long ulStatus;
    
    // Enciende el Led1.
    GPIO_PORTF_DATA_R |= 0x01;  
    
    // Obtiene el estado de la interrupción y lo borra.
    ulStatus = UARTIntStatus(UART1_BASE, true);
    UARTIntClear(UART1_BASE, ulStatus);
    
    // Si el estado de la interrupción es UART_INT_PE (por error de paridad)
    // marca la trama como NOK; Si no, llama a la función correspondiente.
    if ((UART_INT_PE)==ulStatus)
    {
      Modbus_OSL_Frame_Set(MODBUS_OSL_Frame_NOK); 
    }
    else
    { 
      //Debug_OSL_IncChar++;
      switch (Modbus_OSL_Mode)
      {
          case MODBUS_OSL_MODE_RTU:
              Modbus_OSL_RTU_UART();
              break;
                
          case MODBUS_OSL_MODE_ASCII:
              break;
            
          default:
              Modbus_Fatal_Error(100);
      }
    }
    
    // Apaga el LED1.
    GPIO_PORTF_DATA_R &= ~(0x01);
}

//! \brief Implementación práctica del Diagrama de Comportamiento del Slave.
//! 
//! Espera en el estado MODBUS_OSL_IDLE hasta recibir un mensaje, lo comprueba
//! y si es correcto pasa a procesar las acciones demandadas y devolver la 
//! respuesta apropiada si no es una petición BroadCast. En caso de error en
//! algún paso del proceso la respuesta enviada es un aviso del error.
//! \sa Modbus_OSL_Receive_Request, Modbus_App_Manage_Request, Modbus_App_Send
//! \sa Modbus_OSL_BroadCast_Get, enum Modbus_OSL_MainStates
void Modbus_OSL_Serial_Comm (void)
{
  if(Modbus_OSL_MainState_Get()==MODBUS_OSL_IDLE)
  {
    if (Modbus_OSL_Receive_Request())
    {
      Modbus_App_Manage_Request();
    
      if(Modbus_OSL_BroadCast_Get()==0)
      {
        Modbus_OSL_MainState_Set(MODBUS_OSL_REPLY);
        Modbus_App_Send();
      }
      Modbus_OSL_MainState_Set(MODBUS_OSL_IDLE);
    }
  }
}

//! \brief Error Inesperado del Programa.
//!
//! Por Seguridad y robustez de la programación se incluye esta función que
//! detiene el proceso del programa si se da alguna posibilidad en principio
//! excluida en la implementación y considerada imposible. Esto también puede
//! pasar por detenciones e inicios abruptos e inesperados en el proceso o
//! defectos/averías de Hardware. En circunstancias normales es de suponer que
//! nunca se llega a este punto.
//!
//! Esta función está en OSL por estar relacionado éste módulo con OSL y App,
//! evitando tener que duplicar la función para errores en dichos módulos.
//! Además en su llamada incluye un parámetro con distintos valores en función
//! de donde salta el error.
//! \param Error Adopta un valor distinto para localizar la procedencia.
//! > - _Error_ = 10: En _Modbus_App_Manage_Request_ se llega a un tipo de 
//! >    mensaje de Excepción no implementado.
//! > - _Error_ = 20: En _Modbus_App_Process_Action_ una función ya aceptada se  
//! >    determina como desconocida.
//! > - _Error_ = 100: Se llega a la interrupción de la UART sin determinar el 
//! >    modo de la conexión Serie.
//! > - _Error_ = 200: Interrupción 1,5T en un estado donde no deberia poder 
//! >    activarse.
//! > - _Error_ = 210: Interrupción 3,5T en un estado donde no deberia poder
//! >    activarse.
//! \sa Modbus_OSL_Serial_Comm, Modbus_OSL_RTU_15T, Modbus_OSL_RTU_35T
//! \sa Modbus_App_Manage_Request, Modbus_App_Process_Action
void Modbus_Fatal_Error(unsigned char Error)
{  
  while(1)
  {     
    
  }
}
//! @}

//*****************************************************************************
//! \defgroup OSL_Input Entrada de Mensajes
//! \ingroup OSL_Manage
//! \brief Funciones para la gestión de la entrada de mensajes. 
//!
//! Funciones encargadas de detectar y procesar los mensajes entrantes una vez
//! completos para, si son correctos, enviar la información al módulo de 
//! aplicación Modbus App. Cabe destacar que la comprobación del Nº de Slave se
//! realiza en esta parte; de modo que Modbus App sólo debe ocuparse de la
//! información referente a las funciones de Usuario de Modbus.
//*****************************************************************************
//! @{

//! \brief Activa el Flag de Mensaje Completo Recibido.
//! \sa Modbus_OSL_Processing_Flag,Modbus_OSL_RTU_35T,Modbus_OSL_Processing_Msg
void Modbus_OSL_Reception_Complete(void)
{
  Modbus_OSL_Processing_Flag = 1;
}

//! \brief Leer y borrar el Flag de Mensaje Completo Recibido.
//! 
//! Devuelve el valor de _Modbus_OSL_Processing_Flag_ y lo borra para que el 
//! Flag de Mensaje entrante esté activo sólo 1 vez por activación. Deshabilita
//! las interrupciones durante el proceso para evitar una posible activación del 
//! Flag durante el propio proceso, perdiendo un mensaje entrante. 
//! \return  Devuelve 0/1 en función del estado del Flag
//! \sa Modbus_OSL_Processing_Flag, Modbus_OSL_Receive_Request
static unsigned char Modbus_OSL_Processing_Msg(void) 
{
   unsigned char res;
   IntMasterDisable();
   res = Modbus_OSL_Processing_Flag;
   Modbus_OSL_Processing_Flag = 0;
   IntMasterEnable();
   return res;
}

//! \brief Envía un Mensaje entrante Correcto a Modbus App.
//! 
//! Cuando se ha comprobado completamente la corrección de un mensaje entrante
//! se envía al módulo Modbus App para procesar la información contenida. Se
//! utilizan las funciones _Modbus_App_Receive_Char_ y _Modbus_OSL_RTU_Char_Get_ 
//! para evitar la necesidad de guardar una copia del mensaje en el propio 
//! Módulo OSL. Del mismo modo se envía también la longitud del mensaje enviado
//! a App (no la longitud original del mensaje, sin CRC ni Nº de Slave).
//! \sa Modbus_App_Receive_Char, Modbus_OSL_RTU_Char_Get
//! \sa Modbus_App_L_Msg_Set, Modbus_OSL_RTU_L_Msg_Get 
static void Modbus_OSL_RTU_to_App (void)
{
  unsigned char i;
  
  // El primer carácter no se envía por ser el Nº Slave, ademas, por éste motivo
  // se disminuye la longitud del mensaje en 1. El CRC ya ha sido considerado. 
  for(i=1;i<Modbus_OSL_RTU_L_Msg_Get();i++)
      Modbus_App_Receive_Char (Modbus_OSL_RTU_Char_Get(i),i-1);
  Modbus_App_L_Msg_Set(Modbus_OSL_RTU_L_Msg_Get()-1);
}

//! \brief Leer Mensaje Entrante Completo.
//!
//! Si existe un mensaje entrante completo se comprueba el Nº Slave para saber
//! si debe procesarse. De ser así se comprueba el CRC y si es correcto se 
//! envía a App para su procesado mediante _Modbus_OSL_RTU_Control_CRC_ y se
//! vuelve al estado _MODBUS_OSL_IDLE_ para seguir recibiendo mensajes
//! return 1 Un mensaje completo correcto ha sido enviado a App para su Lectura
//! return 0 No hay mensaje o Ignorar mensaje incorrecto.
//! \sa Modbus_OSL_Processing_Msg, Modbus_OSL_RTU_Char_Get
//! \sa Modbus_OSL_RTU_Control_CRC 
static unsigned char Modbus_OSL_Receive_Request(void) 
{
  unsigned char Modbus_OSL_Slave;
  
   // Si hay un mensaje entrante completo.
   if (Modbus_OSL_Processing_Msg()) 
   { 
     // Pasar a estado Checking siguiendo el diagrama.
     Modbus_OSL_MainState_Set(MODBUS_OSL_CHECKING);
     switch (Modbus_OSL_Mode) 
     {
	case MODBUS_OSL_MODE_RTU:
              // Recibir numero de Slave.
              Modbus_OSL_Slave=Modbus_OSL_RTU_Char_Get(0);
              break;

        case MODBUS_OSL_MODE_ASCII:
              // Recibir numero de Slave.
              break;
     }
     
     // Comprobar si el mensaje va dirigido a este Slave.
     if(Modbus_OSL_Slave==Modbus_OSL_Slave_Adress || Modbus_OSL_Slave==0)
     { 
       //Debug_OSL_IncMsg++;
       // Comprobar si el mensaje es BroadCast i activar Flag. 
       if(Modbus_OSL_Slave==0)
         Modbus_OSL_BroadCast=1;
       else
         Modbus_OSL_BroadCast=0;
       
        // Comprobar CRC/LRC y enviar información a App si es correcto.
        switch (Modbus_OSL_Mode) 
        {
            case MODBUS_OSL_MODE_RTU:
                  
                if(Modbus_OSL_RTU_Control_CRC())                
                {
                  //Debug_OSL_CRC_OK++;
                  Modbus_OSL_RTU_to_App();
                  return 1;  
                }
                else
                {
                  
                  /* Si se descarta el mensaje por CRC volver la comprobación de
                  trama a OK para no descartar siguientes mensajes y volver a IDLE. */
                  Modbus_OSL_Frame_Set(MODBUS_OSL_Frame_OK);
                  Modbus_OSL_MainState_Set(MODBUS_OSL_IDLE);
                  return 0;
                }
                break;

            case MODBUS_OSL_MODE_ASCII:
                 /* Comprobar corrección y enviar a App el mensaje ASCII. */	
                break;
        }         
     }
     else
     {
       // Vuelta a IDLE ignorando el mensaje.
       Modbus_OSL_MainState_Set(MODBUS_OSL_IDLE);
     }
   }
   return 0;
}
//! @}

//*****************************************************************************
//! \defgroup OSL_Output Salida de Mensajes
//! \ingroup OSL_Manage
//! \brief Funciones para la gestión de la Salida de mensajes. 
//!
//! Funciones encargadas de la salida de mensajes en las comunicaciones Serie. 
//! Se recoge la trama PDU saliente del Módulo App y se le añade el numero de 
//! Slave al principio y el CRC/LRC al final para formar el ADU que se enviará.
//*****************************************************************************
//! @{

//! \brief Monta y envía el Mensaje.
//!
//! Del módulo App llega la información perteneciente a la función de usuario
//! de Modbus, bien sea de petición o de respuesta. Se le añaden el Nº de Slave
//! y el CRC mediante _Modbus_OSL_RTU_Mount_ADU_ (en caso de Modo ASCII se 
//! deberá implementar la adición del LRC y la traducción del formato) y se 
//! envía el mensaje mediante _Modbus_OSL_Send_.
//! \param *mb_rsp_pdu Puntero al vector con el Mensaje de Salida de App (PDU)
//! \param L_pdu Longitud del Mensaje de Salida de App
//! \sa Modbus_App_Send, Modbus_OSL_RTU_Mount_ADU, Modbus_OSL_L_Response_ADU
//! \sa Modbus_OSL_Send 
void Modbus_OSL_Output (unsigned char *mb_rsp_pdu, unsigned char L_pdu)
{ 
  switch (Modbus_OSL_Mode) 
  {
      case MODBUS_OSL_MODE_RTU:
              // Montar ADU la longitud aumenta en 3 caracteres por el Slave y el CRC.
              // Pasa al estado Emission para cumplir el diagrama de estados de RTU.
              Modbus_OSL_RTU_Mount_ADU (mb_rsp_pdu,Modbus_OSL_Slave_Adress,
                                        L_pdu,Modbus_OSL_Response_ADU);
              Modbus_OSL_L_Response_ADU=L_pdu+3;
              Modbus_OSL_State_Set(MODBUS_OSL_RTU_EMISSION);
          break;

      case MODBUS_OSL_MODE_ASCII:
          // Montar ADU, traducir a ASCII    
          break;
  }    
  Modbus_OSL_Send(Modbus_OSL_Response_ADU, Modbus_OSL_L_Response_ADU);
  
  if (Modbus_OSL_Mode==MODBUS_OSL_MODE_RTU)
  {
    // En RTU se activa el Timer 0 para volver a IDLE cuando desborde.
    TimerLoadSet(TIMER0_BASE, TIMER_A, Modbus_OSL_RTU_Get_Timeout_35());
    TimerEnable(TIMER0_BASE, TIMER_A); 
  }
}

//! \brief Función de Envio de Mensaje.
//!
//! Enciende el LED1 de comunicaciones y envía secuencialmente el numero de 
//! caracteres indicado del vector señalado en los parámetros. Al terminar 
//! apaga el LED de comunicaciones.
//! \param *mb_rsp_adu Puntero al vector con el Mensaje de Salida completo(ADU)
//! \param L_adu Longitud del Mensaje de Salida Completo.
//! \sa Modbus_OSL_Output
static void Modbus_OSL_Send (unsigned char *mb_rsp_adu, unsigned char L_adu)
{
  // Enciende el LED1.
  GPIO_PORTF_DATA_R |= 0x01;        
    
  unsigned char i;
  for (i=0;i<L_adu;i++)
  {
    //Debug_OSL_OutChar++;
    UARTCharPut(UART1_BASE,mb_rsp_adu[i]);
  }
  //Debug_OSL_OutMsg++;
  
  // Apaga el LED1.
  GPIO_PORTF_DATA_R &= ~(0x01);
}
//! @}
#endif