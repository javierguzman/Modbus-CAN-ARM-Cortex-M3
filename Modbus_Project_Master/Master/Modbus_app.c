// Author: Francisco Javier Guzman Jimenez, <dejavits@gmail.com>
/**
*   @mainpage  Master Program Explanation
* 
*   @author Francisco Javier Guzman Jimenez, <dejavits@gmail.com>
*
*   This document shows the application control of the Modbus communication used by the Master device.
*   Such text will be used to understand both the code and the execution. Also, it is made to ease the future scalability,
*   which means adding new features to the application.
*
*   The application is divided into different modules which contain all the necessary function
*   to make a correct communication following the Modbus standard. The App module/layer should be the only one
*   allowed to be accessed by the Master, the rest of them should be like a "hidden implementation".
*
*   FIFO module has the needed structs to store Master's petitions and the communication error messages.
*   OSL module has the communication implementation for the Serial port and OSL RTU the implementation for
*   the communications using RTU. On the other hand, CAN module has the implementation to use CAN as physical layer.
*
*   As previously mentioned, the project is open to new features as the TCP implementation or the Serial communication using
*   ASCII instead of RTU.
*/

/**
*   @defgroup App Modbus App
*   @brief Modbus Application Module
*
*   This module contains the application layer of the Modbus communication following
*   the standard specifics. It is implemented the Modbus user functions referred
*   to both I/O digital and analogic. These functions are used to create bytes chunks called PDU,
*   Protocol Data Unit. The maximum length for these chunks are 253 bytes per message if it is used OSL and
*   128 bytes. On the other hand, it is used big-endian encoding, so the most significant byte is sent first.
*
*   Moreover, this module initialize the configuration of the Master, all FIFO
*   queues and the communication mode with its options. Also, it contains the
*   user queues which are the ones that have to be used by the user to call the
*   I/O read and write operations. So, if it is needed to use Modbus in a program,
*   it should be necessary to include in it the header _Modbus_App.h_
*/
//! @{

#include "Modbus_App.h"

//*****************************************************************************
//
// Variables locales del módulo App usadas en el desarrollo del programa para
// la comprobación del comportamiento del sistema y la depuración de errores.
//
//*****************************************************************************

//unsigned char Debug_App_W_Data=0,Debug_App_Rsp_Resend=0,Debug_App_Msg_Ok=0,
//              Debug_App_Rsp_E1=0,Debug_App_Rsp_E2=0,Debug_App_Rsp_E3=0,
//              Debug_App_W_Function=0;

//*****************************************************************************
//
// App module global variables.
//
//*****************************************************************************

//! FIFO Request. It stores the request which have not been sent yet.
static struct Modbus_FIFO_s Modbus_FIFO_Tx;
//! \brief Error Communication FIFO; It stores the error responses next to the request
//! who provoked it and the messages not replied.
static struct Modbus_FIFO_Errors Modbus_FIFO_Error;
//! It stores temporary a request to add it later into the Request FIFO.
static struct Modbus_FIFO_Item Modbus_App_Request;
//! \brief It stores the actual request, in this way, it is conserved its data while
//! the answer arrives.
static struct Modbus_FIFO_Item Modbus_App_Actual_Req;
//! It stores temporary a request to add it later into the Error FIFO
static struct Modbus_FIFO_E_Item Modbus_App_Error_Msg;
//! Array to store the incoming PDU
static unsigned char Modbus_App_Msg[MAX_PDU];
//! Incoming message length
static unsigned char Modbus_App_L_Msg;
//! Array to store the outcoming PDU
static unsigned char Modbus_App_Req_pdu[MAX_PDU];
//! Outcoming message length
static unsigned char Modbus_App_L_Req_pdu;
//! Modbus communication mode. Only Serial & CAN communication.
enum Modbus_Comm_Modes Modbus_Comm_Mode;// = MODBUS_CANN; //WATCH OUT WITH THISS!!!!!!!!!!!!!!!!!

//! @}

//*****************************************************************************
//
// Module App, local functions prototypes
//
//*****************************************************************************

// Responses

static unsigned char Modbus_App_Read_Single_Bits_CallBack(void);
static unsigned char Modbus_App_Read_Registers_CallBack(void);
static unsigned char Modbus_App_Write_CallBack(void);
static unsigned char Modbus_App_Mask_Write_CallBack(void);
static unsigned char Modbus_App_Read_Write_M_Registers_CallBack(void);

// To tune up output requests

static void Modbus_App_Standard_Request(void);
static void Modbus_App_Write_M_Coils(void);
static void Modbus_App_Write_M_Registers(void);
static void Modbus_App_Mask_Write_Register(void);
static void Modbus_App_Read_Write_M_Registers(void);

/**
*   @defgroup App_Control Application Control for the Communication Mode: OSL/CAN
*   @ingroup App
*   @brief Init and Control Functions
*
*   It determines the Modbus communication type and it is configured both input
*   and output FIFOs and the communication.
*/

#if OSL_Mode
//! \brief Configura las comunicaciones del Master.
//! \ingroup App_Control
//!
//! Llama a las funciones de inicio de las colas FIFO de Errores y de Peticiones
//! y determina el Modo de Comunicaciones de Modbus; llamando a la función de  
//! configuración de las Comunicaciones. 
//! \param Com_Mode Modo de Comunicación de Modbus.
//! \param Baudrate  Baudrate de las comunicaciones
//! \param Attempts  Numero Máximo de Intentos de Envío antes de descartar
//! \param OSL_Mode  Mode RTU/ASCII de la comunicación Serie.
//! \sa Modbus_FIFO_Init, Modbus_FIFO_E_Init, Modbus_OSL_Init, Modbus_CAN_Init
void Modbus_Master_Init(enum Modbus_Comm_Modes Com_Mode, enum Baud Baudrate, 
                        unsigned char Attempts, enum Modbus_OSL_Modes OSL_Mode)
{ 
  Modbus_FIFO_Init(&Modbus_FIFO_Tx);
  Modbus_FIFO_E_Init(&Modbus_FIFO_Error);
  
  if (Com_Mode == CDEFAULT) 
    Modbus_Comm_Mode=MODBUS_SERIAL;
    
  switch(Modbus_Comm_Mode)  
  {
    case (MODBUS_SERIAL):
      Modbus_OSL_Init(Baudrate,OSL_Mode, Attempts);  
      break;
      /* Other communications do not use this Init function*/
    default:  
      break; 
  }
}

//! \brief Función de Usuario Para la Comunicación.
//! \ingroup App_Control
//!
//! Esta es la función que el usuario de esta aplicación debe incorporar a su
//! bucle de proceso para que se realicen las Comunicaciones. Previamente se 
//! debe llamar a Modbus_Master_Init para configurar las comunicaciones. Para  
//! llevar el control de los errores se debe llamar en el bucle principal del
//! programa a Modbus_Get_Error, que lee la cola de Errores y lee el primero que
//! haya encolado.
//!
//! Esta función devuelve 1 mientras queden comunicaciones pendientes para
//! información del usuario y permitir fijar las comunicaciones hasta que se
//! hayan realizado todas si se desea. En caso contrario se puede, simplemente,
//! ignorar esta respuesta.
//! \return 1 Se están procesando comunicaciones
//! \return 0 No queda ninguna comunicación que realizar, no hay peticiones
//! \sa Modbus_OSL_Serial_Comm, Modbus_OSL_Init, Modbus_Master_Init, Modbus_CAN_Init, Modbus_CAN_Controller
unsigned char Modbus_Master_Communication (void)
{
  if(Modbus_OSL_Serial_Comm())
      return 1;
  return 0;
}
//! \brief Gestion de las Respuestas Recibidas.
//! \ingroup App_Control
//!
//! Ante una respuesta aceptada del Slave esperado y en función del Nº de función
//! de la petición se comprueba la corrección de los datos de la respuesta, y si
//! la petición era de lectura se vuelcan los datos en el vector destino. Si se
//! descarta la función por errores en los datos se pasa al estado ERROR para
//! que se active el Flag de Reenvío. Si la respuesta recibida es de excepción
//! se encola la petición y la respuesta en la cola FIFO de Errores y se pasa
//! a IDLE para seguir con las peticiones.
//! \sa Modbus_App_Read_Single_Bits_CallBack, Modbus_App_Read_Registers_CallBack
//! \sa Modbus_App_Write_CallBack, Modbus_App_Mask_Write_CallBack
//! \sa Modbus_OSL_Reset_Attempt, Modbus_FIFO_E_Enqueue, Modbus_CAN_Reset_Attempt
void Modbus_App_Manage_CallBack (void)
{
  // Si la Respuesta es normal y de la función esperada se gestiona.
  if(Modbus_App_Msg[0]==Modbus_App_Actual_Req.Function)
  {
      unsigned char CallBack;

      CallBack=Modbus_App_Actual_Req.Function;

      /* Algunas funciones de Modbus se engloban y se analizan con la misma
      función, la variable auxiliar CallBack las agrupa en un único valor */

      if(Modbus_App_Msg[0]==1 || Modbus_App_Msg[0]==2)
        CallBack=1 ;
      if(Modbus_App_Msg[0]==3 || Modbus_App_Msg[0]==4)
        CallBack=2;
      if(Modbus_App_Msg[0]==5 || Modbus_App_Msg[0]==6 ||
         Modbus_App_Msg[0]==15 || Modbus_App_Msg[0]==16)
        CallBack=3;


     switch (CallBack)
     {
        case 1:
          if(Modbus_App_Read_Single_Bits_CallBack())
            Modbus_OSL_MainState_Set(MODBUS_OSL_ERROR);
          break;
        case 2:
          if(Modbus_App_Read_Registers_CallBack())
            Modbus_OSL_MainState_Set(MODBUS_OSL_ERROR);
          break;
        case 3:
          if(Modbus_App_Write_CallBack())
            Modbus_OSL_MainState_Set(MODBUS_OSL_ERROR);
          break;
        case 22:
          if(Modbus_App_Mask_Write_CallBack())
            Modbus_OSL_MainState_Set(MODBUS_OSL_ERROR);
          break;
        case 23:
          if(Modbus_App_Read_Write_M_Registers_CallBack())
            Modbus_OSL_MainState_Set(MODBUS_OSL_ERROR);
          break;
        default:
          Modbus_Fatal_Error(10);
          break;
     }

     /* Si los datos eran incorrectos el estado será ERROR y se reenviará. Si
     los datos eran correctos se pasa a la siguiente petición. */

     if(Modbus_OSL_MainState_Get()!=MODBUS_OSL_ERROR)
     {
       /* Respuesta Correcta, se pasa a la siguiente petición. */
       Modbus_OSL_Reset_Attempt();
       Modbus_OSL_MainState_Set(MODBUS_OSL_IDLE);
       //Debug_App_Msg_Ok++;
     }
  }
  // Función de excepción o inesperada.
  else
  {
    // Se pone en estado ERROR para gestionar el error. Si no entra en todos los
    // "if" el mensaje es erróneo u el estado seguirá siendo ERROR al salir de
    // la función, de modo que acabará reenviándose si corresponde.
    Modbus_OSL_MainState_Set(MODBUS_OSL_ERROR);
    // Si la respuesta es la de Excepción esperada.
    if(Modbus_App_Msg[0]==Modbus_App_Actual_Req.Function | 128)
    {
      // Y el mensaje de excepción es correcto. Tipo de 1-8, 10 o 11.
      if(Modbus_App_L_Msg==2 &&
        (Modbus_App_Msg[1]<=8 || Modbus_App_Msg[1]==10 || Modbus_App_Msg[1]!=11))
      {
        /* Encolar Petición + Mensaje de Excepción. */
        Modbus_App_Error_Msg.Request=Modbus_App_Actual_Req;
        Modbus_App_Error_Msg.Response[0]=Modbus_App_Msg[0];
        Modbus_App_Error_Msg.Response[1]=Modbus_App_Msg[1];
        Modbus_FIFO_E_Enqueue(&Modbus_FIFO_Error,&Modbus_App_Error_Msg);

        /* Resetear Nº Envíos; se pasa a la siguiente petición. */
        Modbus_OSL_Reset_Attempt();
        Modbus_OSL_MainState_Set(MODBUS_OSL_IDLE);
      }
    }
  }
}
//! \brief Encola o Envía una petición.
//! \ingroup App_Exchange
//!
//! Llamada por las funciones de Modbus de usuario, esta función envía una
//! petición directamente si la cola de Peticiones está vacía y las comunicaciones
//! libres y si no la encola para su posterior envío.
//! return 1 La cola está llena y no se puede encolar
//! return 0 Todo correcto
//! \sa Modbus_FIFO_Enqueue, Modbus_App_Send
unsigned char Modbus_App_Enqueue_Or_Send(void)
{
  if(Modbus_OSL_MainState_Get()==MODBUS_OSL_IDLE && Modbus_FIFO_Empty(&Modbus_FIFO_Tx))
  {
    Modbus_App_Actual_Req=Modbus_App_Request;
    Modbus_App_Send();
  }
  else
  {
    if(Modbus_FIFO_Enqueue(&Modbus_FIFO_Tx,&Modbus_App_Request))
      return 1;
  }
  return 0;
}
//! \brief Envía una petición.
//! \ingroup App_Exchange
//!
//! Envía la petición almacenada en _Modbus_App_Actual_Req_; utiliza para dar
//! formato al mensaje una función que depende del tipo de petición y para
//! enviarla llama a _Modbus_OSL_Output_.
//! \sa struct Modbus_FIFO_Item, Modbus_OSL_Output, Modbus_CAN_Fit_Output, Modbus_App_Standard_Request
//! \sa Modbus_App_Write_M_Coils, Modbus_App_Write_M_Registers
//! \sa Modbus_App_Mask_Write_Register, Modbus_App_Read_Write_M_Registers
void Modbus_App_Send(void)
{
  unsigned char Request;

  if(Modbus_App_Actual_Req.Function==1 || Modbus_App_Actual_Req.Function==2 ||
     Modbus_App_Actual_Req.Function==3 || Modbus_App_Actual_Req.Function==4 ||
     Modbus_App_Actual_Req.Function==5 || Modbus_App_Actual_Req.Function==6)
    Request=1 ;
  else
    Request=Modbus_App_Actual_Req.Function;

  switch(Request)
  {
      case 1:
        Modbus_App_Standard_Request();
        break;
      case 15:
        Modbus_App_Write_M_Coils();
        break;
      case 16:
        Modbus_App_Write_M_Registers();
        break;
      case 22:
        Modbus_App_Mask_Write_Register();
        break;
      case 23:
        Modbus_App_Read_Write_M_Registers();
        break;
      default:
        Modbus_Fatal_Error(20);
        break;
  }
  Modbus_OSL_Output (Modbus_App_Req_pdu,Modbus_App_Actual_Req.Slave,Modbus_App_L_Req_pdu);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////CCCCCCCCCCCCAAAAAAAAAAAAANNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNN////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#elif CAN_Mode
/**
*   @brief Tune up the Master communications.
*   @ingroup App_Control
*
*   Initialise both Error and Request FIFOs. In addition, it is set up the CAN module.
*   @param bit_rate Bit rate chosen
*   @param attempts  Maximum number of attempts of sending before discarding the message
*   @sa Modbus_FIFO_Init, Modbus_FIFO_E_Init, Modbus_CAN_Init, Modbus_OSL_Init
*/
unsigned char Modbus_Master_Init(enum Modbus_CAN_BitRate bit_rate, unsigned char attempts)///
{
          if(attempts >= 1)
          {            
              Modbus_Comm_Mode = MODBUS_CAN_MODE;  
              Modbus_FIFO_Init(&Modbus_FIFO_Tx);
              Modbus_FIFO_E_Init(&Modbus_FIFO_Error);
              Modbus_CAN_Init(bit_rate, attempts);  
              return 1;
          }
          else
          {
              return 0;
          }
}

/**
*   @brief User function for the communication.
*   @ingroup App_Control
*
*   This function has to be used by the user in the process loop of his application
*   to make run the communication. Previously, it has to be called _Modbus_Master_Init_ to configure the
*   proper communication module. To supervise the errors, it has to be called _Modbus_Get_Error_ in
*   the program main loop. Such a function reads the first element from the Error FIFO.
*
*   This function return 1 if there are pending communications and these ones can be fixed until all of
*   them are done if it is wished. In any case, this answer can be ignored.
*   @return 1 There are communications running.
*   @return 0 No requests. So there are not communications.
*   @sa Modbus_CAN_Controller, Modbus_CAN_Init, Modbus_Master_Init, Modbus_OSL_Init, Modbus_OSL_Serial_Comm
*/
unsigned char Modbus_Master_Communication (void)///
{
  if( Modbus_CAN_Controller() )
      return 1;
  return 0;
}

/**
*   @brief Received responses management.
*   @ingroup App_Control
* 
*   It is checked the data correctness depending on the function number of a response
*   from the expected Slave. If the request was a read, the data is stored in the destination vector.
*   If the function is discarded because of a data error, the status is changed to ERROR to activate the
*   forward flag. If the answer is an exception, the request and its answer are enqueued in the Error FIFO, after that,
*   the status is switched to IDLE to continue with the rest of petitions.
*   @sa Modbus_App_Read_Single_Bits_CallBack, Modbus_App_Read_Registers_CallBack
*   @sa Modbus_App_Write_CallBack, Modbus_App_Mask_Write_CallBack
*   @sa Modbus_CAN_Reset_Attempt, Modbus_FIFO_E_Enqueue, Modbus_OSL_Reset_Attempt
*/
void Modbus_App_Manage_CallBack (void)///
{
  //If the response is normal and the function is the waited one, then it is managed.
  if( Modbus_App_Msg[0] == Modbus_App_Actual_Req.Function)
  {
      unsigned char CallBack;

      CallBack = Modbus_App_Actual_Req.Function;

      /* Some Modbus functions have the same structure, CallBack join the similar ones*/

      if(Modbus_App_Msg[0]==1 || Modbus_App_Msg[0]==2)
        CallBack=1 ;
      if(Modbus_App_Msg[0]==3 || Modbus_App_Msg[0]==4)
        CallBack=2;
      if(Modbus_App_Msg[0]==5 || Modbus_App_Msg[0]==6 ||
         Modbus_App_Msg[0]==15 || Modbus_App_Msg[0]==16)
        CallBack=3;


     switch (CallBack)
     {
        case 1:
          if(Modbus_App_Read_Single_Bits_CallBack())
        	  Modbus_SetMainState(MODBUS_ERROR);
          break;
        case 2:
          if(Modbus_App_Read_Registers_CallBack())
        	  Modbus_SetMainState(MODBUS_ERROR);
          break;
        case 3:
          if(Modbus_App_Write_CallBack())
        	  Modbus_SetMainState(MODBUS_ERROR);
          break;
        case 22:
          if(Modbus_App_Mask_Write_CallBack())
        	  Modbus_SetMainState(MODBUS_ERROR);
          break;
        case 23:
          if(Modbus_App_Read_Write_M_Registers_CallBack())
        	  Modbus_SetMainState(MODBUS_ERROR);
          break;
        default:
          Modbus_CAN_Error_Management(10);
          break;
     }

     /* If the data was wrong, the status is ERROR, then a resend must be done.
      * If the data was correct the next request is handle
      */

     if(Modbus_GetMainState() != MODBUS_ERROR)
     {
       /* Correct answer, next request */
       Modbus_CAN_Reset_Attempt();
       Modbus_SetMainState(MODBUS_IDLE);       
     }
  }
  // Exception or unexpected function
  else
  {
    //Status is changed to ERROR to manage the error.
	//If at the end of the next statements the error continues being ERROR a resend will be done
	  Modbus_SetMainState(MODBUS_ERROR);
    //If the answer is the expected exception
    if(Modbus_App_Msg[0] == Modbus_App_Actual_Req.Function | 128)
    {
      // The exception message is correct. Type 1-8, 10 or 11
      if(Modbus_App_L_Msg==2 &&
        (Modbus_App_Msg[1]<=8 || Modbus_App_Msg[1]==10 || Modbus_App_Msg[1]!=11))
      {
    	  /*Request and exception message are added to the ERROR queue*/
        Modbus_App_Error_Msg.Request=Modbus_App_Actual_Req;
        Modbus_App_Error_Msg.Response[0]=Modbus_App_Msg[0];
        Modbus_App_Error_Msg.Response[1]=Modbus_App_Msg[1];
        Modbus_FIFO_E_Enqueue(&Modbus_FIFO_Error,&Modbus_App_Error_Msg);

        /*Number of deliveries reseted; next request can be handle*/
        Modbus_CAN_Reset_Attempt();
        Modbus_SetMainState(MODBUS_IDLE);
      }
    }
  }
}

/**
*   @brief Enqueue or Send a request.
*   @ingroup App_Exchange
*
*   This function is called from user Modbus functions. This one sends a request directly if the
*   Request FIFO is empty and the communications are not occupied, otherwise, the petition is enqueued to send it later.
*   return 1 The Request FIFO is full and the petition cannot be enqueued.
*   return 0 Everything ok
*   @sa Modbus_FIFO_Enqueue, Modbus_App_Send
*/
unsigned char Modbus_App_Enqueue_Or_Send(void)///
{
  if(Modbus_GetMainState() == MODBUS_IDLE && Modbus_FIFO_Empty(&Modbus_FIFO_Tx))
  {
    Modbus_App_Actual_Req = Modbus_App_Request;
    Modbus_App_Send();
  }
  else
  {
    if(Modbus_FIFO_Enqueue(&Modbus_FIFO_Tx,&Modbus_App_Request))
      return 1;
  }
  return 0;
}

/**
*   @brief Send a request.
*   @ingroup App_Exchange
*
*   The request stored in _Modbus_App_Actual_Req_ is sent; This function builds
*   the message depending on the type of Modbus function. To send it is used
*   _Modbus_CAN_Fix_Output_.
*   @sa struct Modbus_FIFO_Item, Modbus_CAN_Fix_Output, Modbus_OSL_Output, Modbus_App_Standard_Request
*   @sa Modbus_App_Write_M_Coils, Modbus_App_Write_M_Registers
*   @sa Modbus_App_Mask_Write_Register, Modbus_App_Read_Write_M_Registers
*/
void Modbus_App_Send(void)///
{
  unsigned char Request;
  //a guess of the number of bytes that will be receive as answer, just for the CAN timeout
  uint16_t data_amount_to_wait;
  
  if(Modbus_App_Actual_Req.Function==1 || Modbus_App_Actual_Req.Function==2 ||
     Modbus_App_Actual_Req.Function==3 || Modbus_App_Actual_Req.Function==4 ||
     Modbus_App_Actual_Req.Function==5 || Modbus_App_Actual_Req.Function==6)
    Request=1 ;
  else
    Request=Modbus_App_Actual_Req.Function;

  switch(Request)
  {
      case 1:
        Modbus_App_Standard_Request();
        //If I ask for 112 coils, I will receive 14 "extra" bytes
        data_amount_to_wait = (Modbus_App_Req_pdu[4] | Modbus_App_Req_pdu[3]);
        if(Modbus_App_Actual_Req.Function==3 || Modbus_App_Actual_Req.Function==4)
            data_amount_to_wait = (data_amount_to_wait * 2) + 1 + 5 + 2;
        else
            data_amount_to_wait = (data_amount_to_wait) + 1 + 5 + 2;        
        break;
      case 15:
        Modbus_App_Write_M_Coils();
        data_amount_to_wait = Modbus_App_Req_pdu[5] + 1 + 5 + 6;        
        break;
      case 16:
        Modbus_App_Write_M_Registers();
        data_amount_to_wait = Modbus_App_Req_pdu[5] * 2;
        data_amount_to_wait += 1 + 5 + 6;
        break;
      case 22:
        Modbus_App_Mask_Write_Register();
        data_amount_to_wait = 14 + 1;
        break;
      case 23:
        Modbus_App_Read_Write_M_Registers();
        data_amount_to_wait = (Modbus_App_Req_pdu[4] | Modbus_App_Req_pdu[3]) * 2;
        data_amount_to_wait += (Modbus_App_Req_pdu[10] * 2) + 1 + 2 +10;        
        break;
      default:
        Modbus_CAN_Error_Management(20);
        break;
  }
  Modbus_CAN_FixOutput(Modbus_App_Req_pdu,Modbus_App_Actual_Req.Slave,Modbus_App_L_Req_pdu, data_amount_to_wait);
}
#endif
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////SEPARATION ENDING////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
*   @brief Get error messages
*   @ingroup App_Control
*
*   Function to let the user know when there are errors in the execution of the requests.
*   This can be noticed when it arrives an exception response or there is no answer (exceeding the total number of sending attempts).
*   When that happens, a message is stored in the error FIFO. This function removes an item of such a queue (if there is some);
*   in that way, user is able to know and act accordingly. If the message is due to an exception response, then this one is made of the
*   request who triggered the exception and an exception code in the "answer" field. If the message is due to the missing responses, 
*   the "answer" field will be empty. 
*
*   @warning THe user has to create the structs _Modbus_FIFO_E_Item_ and _Modbus_FIFO_Item in his program.
*   @param *Error Structure where it is stored the error reading
*   @return 1 One item was removed from the queue
*   @return 0 Error queue is empty
*   @sa struct Modbus_FIFO_E_Item, struct Modbus_FIFO_Item, Modbus_FIFO_Error
*   @sa Modbus_FIFO_E_Dequeue
*/
unsigned char Modbus_Get_Error (struct Modbus_FIFO_E_Item *Error)
{
  return Modbus_FIFO_E_Dequeue (&Modbus_FIFO_Error, Error);
}

/**
*   @brief No answer; It enqueues the request in the Error FIFO.
*   @ingroup App_Control 
*
*   If this function is activated means that the maximum number of sendings of one function was exceeded without achieving any answer.
*   Therefore, the proper request is enqueued as an exception message, the difference is that in the "answer" field of the message is
*   stored [0,0].
*   @sa Modbus_FIFO_E_Enqueue, Modbus_OSL_Repeat_Request, Modbus_CAN_Repeat_Request
*/
void Modbus_App_No_Response(void)
{
  Modbus_App_Error_Msg.Request=Modbus_App_Actual_Req;  
  Modbus_App_Error_Msg.Response[0]=0;
  Modbus_App_Error_Msg.Response[1]=0;
  Modbus_FIFO_E_Enqueue(&Modbus_FIFO_Error,&Modbus_App_Error_Msg);
}

/**
*   @defgroup App_Exchange Interconnection between OSL/CAN Layer and App Layer
*   @ingroup App
*   @brief Functions to exchange data between App and OSL/CAN module.
*
*   It includes functions to exchange information data between the CAN/OSL and App module,
*   allowing APP to send/receive messages through CAN/OSL.
*/

/**
*   @brief It gets and sends a petition from the request FIFO if it is not empty.
*   @ingroup App_Exchange
*
*   @return 0 It has sent a request from the queue
*   @return 1 Empty queue, there is no requests to be sent
*   @sa Modbus_FIFO_Dequeue, Modbus_App_Send
*/
unsigned char Modbus_App_FIFOSend(void)
{
  // La función devuelve 1 si ha desencolado y entra en el "if"
  if (Modbus_FIFO_Dequeue(&Modbus_FIFO_Tx,&Modbus_App_Actual_Req))
  {
    Modbus_App_Send();  
    return 0;
  }
  return 1;
}

/**   
*   @brief It receives a char from another module.
*   @ingroup App_Exchange
*
*   It allows to store chars in some position of the vector _Modbus_App_Msg_; It is used in _Modbus_OSL_RTU_to_App(which also uses
*   _Modbus_OSL_RTU_Char_Get_) and in _Modbus_CAN_to_App to transfer messages from RTU to App through OSL layer without connecting them;
*   Also to transfer from CAN to App.
*   @param Msg Char value to be stored in _Modbus_App_Msg_
*   @param i Vector index where to store the char
*   @sa Modbus_App_Msg, Modbus_OSL_RTU_to_App, Modbus_OSL_RTU_Char_Get
*/
void Modbus_App_Receive_Char (unsigned char Msg, unsigned char i)
{
    Modbus_App_Msg[i]=Msg;
  
}

/** 
*   @brief It receives the message length.
*   @ingroup App_Exchange
*
*   It is used in _Modbus_OSL_RTU_to_App_ and _Modbus_CAN_to_App_ to send from OSL/CAN the length 
*   of the correct incoming message to App.
*   @param Index Length message value
*   @sa Modbus_App_Msg, Modbus_App_L_Msg, Modbus_OSL_RTU_to_App
*/
void Modbus_App_L_Msg_Set(unsigned char Index)
{
  Modbus_App_L_Msg=Index;
}

/**
*   @defgroup App_Modbus Modbus Functions
*   @ingroup App
*   @brief Modbus user functions to send requests in addition with internal functions from App Module to give the proper format to the requests.
*   Also, it manages the responses.
*   
*   For each public Modbus function, it exists an user one to make the request; Also, other functions to give format to the request 
*   before the sending are grouped here. Moreover, it is implemented the proper functions to understand the received responses 
*   from the Slaves and to store the received values of the Read requests.
*/

/**
*   @defgroup App_User User Functions
*   @ingroup App_Modbus
*   @brief User functions to make the requests.
*
*   Public Modbus functions to Read/Write the I/O.
*   It checks the data correctness of the request and it is enqueued or sent in 
*   _Modbus_App_Enqueue_Or_Send_. It returns 0 if all is correct, or 1 if either there was a wrong parameter or 
*   the request FIFO was filled. It has to be taken into account that the Read/Write multiple I/O functions have a limit number of I/O
*   each one. Furthermore, the addresses are from 0 to 65.535; then if the address is near this number it can exceed the maximum and
*   the request would not be done.
*   >_Example_: For the address 65.000, it can not be done a Read of 600 Coils.
*   Additionally, it has to be heeded that the maximum number of slaves is 247 in OSL/CAN.
*
*   @warning If the response of one of these functions is 1, the request was not done.
*/
//! @{

/**
*   @brief Read multiple Coils.
*
*   It reads from 1 to 2000 continuous Coils from one Slave. The read is stored in the vector pointed by *Response.
*   @param Slave Slave number which it is requested the data.
*   @param Adress Initial address of the read
*   @param Coils Coils amount to be read
*   @param *Response Pointer to where the read will be stored
*   @return 0 Correct request 
*   @return 1 It cannot be enqueued or wrong parameters
*   @sa Modbus_App_Enqueue_Or_Send, Modbus_App_Request
*/
unsigned char Modbus_Read_Coils (unsigned char Slave, uint16_t Adress, 
                                 uint16_t Coils, unsigned char *Response)
{ 
  if(Slave>247 || Slave==0 || Coils>2000  || Coils==0 || ((long)Adress+(long)Coils)>65535)
      return 1;
  else
  { 
    Modbus_App_Request.Slave=Slave;
    Modbus_App_Request.Function=1;
    Modbus_App_Request.Data[0].UI2=Adress;
    Modbus_App_Request.Data[1].UI2=Coils;
    Modbus_App_Request.Data[2].PC=Response;
      
    if(Modbus_App_Enqueue_Or_Send())
      return 1;
    
    return 0;
  } 
}

/**
*   @brief Read multiple Discrete Inputs.
*   
*   It reads from 1 to 2000 continuous Discrete Inputs from one Slave. The read is stored in the vector pointed by *Response.
*   @param Slave Slave number which it is requested the data.
*   @param Adress Initial address of the read
*   @param Inputs Amount of Discrete Inputs to be read
*   @param *Response Pointer to where the read will be stored
*   @return 0 Correct request
*   @return 1 It cannot be enqueued or wrong parameters
*   @sa Modbus_App_Enqueue_Or_Send, Modbus_App_Request
*/
unsigned char Modbus_Read_D_Inputs (unsigned char Slave, uint16_t Adress, 
                                    uint16_t Inputs, unsigned char *Response)
{ 
  if(Slave>247 || Slave==0 || Inputs>2000 || Inputs==0 || ((long)Adress+(long)Inputs)>65535)
      return 1;
  else
  { 
    Modbus_App_Request.Slave=Slave;
    Modbus_App_Request.Function=2;
    Modbus_App_Request.Data[0].UI2=Adress;
    Modbus_App_Request.Data[1].UI2=Inputs;
    Modbus_App_Request.Data[2].PC=Response;
  
    if(Modbus_App_Enqueue_Or_Send())
      return 1;
    
    return 0;
  } 
}

/**
*   @brief Read multiple Holding Registers.
*
*   It reads from 1 to 125 continuous registers from one slave. The read is stored in the vector pointed by *Response.
*   @param Slave Slave number which it is requested the data.
*   @param Adress Initial address of the read
*   @param Registers Amount of Holding Registers to be read
*   @param *Response Pointer to where the read will be stored
*   @return 0 Correct request
*   @return 1 It cannot be enqueued or wrong parameters
*   @sa Modbus_App_Enqueue_Or_Send, Modbus_App_Request
*/
unsigned char Modbus_Read_H_Registers (unsigned char Slave, uint16_t Adress,
                                       uint16_t Registers, uint16_t *Response)
{ 
  if(Slave>247 || Slave==0 || Registers>125 || Registers==0 || ((long)Adress+(long)Registers)>65535)
      return 1;
  else
  { 
    Modbus_App_Request.Slave=Slave;
    Modbus_App_Request.Function=3;
    Modbus_App_Request.Data[0].UI2=Adress;
    Modbus_App_Request.Data[1].UI2=Registers;
    Modbus_App_Request.Data[2].PUI2=Response;
      
    if(Modbus_App_Enqueue_Or_Send())
      return 1;
    
    return 0;
  } 
}
/**
*   @brief Read multiple Input Registers.
*
*   It reads from 1 to 125 continuous registers from one slave. The read is stored in the vector pointed by *Response.
*   @param Slave Slave number which it is requested the data.
*   @param Adress Initial address of the read
*   @param Registers Amount of Input Registers to be read
*   @param *Response Pointer to where the read will be stored
*   @return 0 Correct request
*   @return 1 It cannot be enqueued or wrong parameters
*   @sa Modbus_App_Enqueue_Or_Send, Modbus_App_Request
*/
unsigned char Modbus_Read_I_Registers (unsigned char Slave, uint16_t Adress,
                                       uint16_t Registers, uint16_t *Response)
{ 
  if(Slave>247 || Slave==0 || Registers>125 || Registers==0 || ((long)Adress+(long)Registers)>65535)
      return 1;
  else
  { 
    Modbus_App_Request.Slave=Slave;
    Modbus_App_Request.Function=4;
    Modbus_App_Request.Data[0].UI2=Adress;
    Modbus_App_Request.Data[1].UI2=Registers;
    Modbus_App_Request.Data[2].PUI2=Response;
      
    if(Modbus_App_Enqueue_Or_Send())
      return 1;
    
    return 0;
  } 
}

/**
*   @brief Write one Coil.
*
*   It set to 0 or 1 one coil in the concrete address.
*   @param Slave Slave number which it is requested the data.
*   @param Adress Address to write
*   @param Coil Coil value (If it is not 0, it will be set to 1)
*   @return 0 Correct request
*   @return 1 It cannot be enqueued or wrong parameters
*   @sa Modbus_App_Enqueue_Or_Send, Modbus_App_Request
*/
unsigned char Modbus_Write_Coil (unsigned char Slave, uint16_t Adress,unsigned char Coil)
{ 
  if(Slave>247)   
    return 1;
  else      
  {
    Modbus_App_Request.Slave=Slave;
    Modbus_App_Request.Function=5;
    Modbus_App_Request.Data[0].UI2=Adress;
    
    // Se envia 0 o 0xFF00
    if(Coil==0)
      Modbus_App_Request.Data[1].UI2=0;
    else
      Modbus_App_Request.Data[1].UI2=65280; 
    
    if(Modbus_App_Enqueue_Or_Send())
      return 1;
    
    return 0;
  }
}

/*
*   @brief Write one I/O Register.
*
*   The value is written in the register of the indicated address.
*   @param Slave Slave number which it is requested the data.
*   @param Adress Address to write
*   @param Register Value to be written in the Register
*   @return 0 Correct request
*   @return 1 It cannot be enqueued or wrong parameters
*   @sa Modbus_App_Enqueue_Or_Send, Modbus_App_Request
*/
unsigned char Modbus_Write_Register (unsigned char Slave, uint16_t Adress, uint16_t Register)
{ 
  if(Slave>247)   
    return 1;
  else      
  {
    Modbus_App_Request.Slave=Slave;
    Modbus_App_Request.Function=6;
    Modbus_App_Request.Data[0].UI2=Adress;
    Modbus_App_Request.Data[1].UI2=Register;
      
    if(Modbus_App_Enqueue_Or_Send())
      return 1;
    
    return 0;
  }
}

/**
*   @brief Write multiple Coils.
*
*   It writes from 1 to 1968 continuous Coils from one slave from the sent address with the indicated values.
*   @param Slave Slave number which it is requested the data.
*   @param Adress Initial address to write
*   @param Coils Number of Coils to write
*   @param *Value Pointer to where the values to write are stored
*   @return 0 Correct Request 
*   @return 1 It cannot be enqueued or wrong parameters
*   @sa Modbus_App_Enqueue_Or_Send, Modbus_App_Request
*/
unsigned char Modbus_Write_M_Coils (unsigned char Slave, uint16_t Adress,
                                    uint16_t Coils, unsigned char *Value)
{ 
  if(Slave>247 || Coils>1968 || Coils==0 || ((long)Adress+(long)Coils)>65535)   
    return 1;
  else      
  {
    Modbus_App_Request.Slave=Slave;
    Modbus_App_Request.Function=15;
    Modbus_App_Request.Data[0].UI2=Adress;
    Modbus_App_Request.Data[1].UI2=Coils;
    Modbus_App_Request.Data[2].PC=Value;
      
    if(Modbus_App_Enqueue_Or_Send())
      return 1;
    
    return 0;
  }
}

/**
*   @brief Write multiple I/O Registers.
*
*   It writes from 1 to 123 continuous Registers from one slave from the sent address with the indicated values.
*   @param Slave Slave number which it is requested the data.
*   @param Adress Initial address to write
*   @param Registers Number of Registers to write
*   @param *Value Pointer to where the values to write are stored
*   @return 0 Correct request
*   @return 1 It cannot be enqueued or wrong parameters
*   @sa Modbus_App_Enqueue_Or_Send, Modbus_App_Request
*/
unsigned char Modbus_Write_M_Registers (unsigned char Slave, uint16_t Adress,
                                        uint16_t Registers, uint16_t *Value)
{ 
  if(Slave>247 || Registers>123 || Registers==0 || ((long)Adress+(long)Registers)>65535)   
    return 1;
  else      
  {
    Modbus_App_Request.Slave=Slave;
    Modbus_App_Request.Function=16;
    Modbus_App_Request.Data[0].UI2=Adress;
    Modbus_App_Request.Data[1].UI2=Registers;
    Modbus_App_Request.Data[2].PUI2=Value;
      
    if(Modbus_App_Enqueue_Or_Send())
      return 1;
    
    return 0;
  }
}

/**
*   @brief Write one I/O Register using masks.
*
*   It modifies the I/O Register value using one AND mask and one OR mask. The operation is the following:
*>  Value = (Register value AND AND_Mask) OR (OR_Mask AND (NOT AND_Mask))
*   That means, in one side is kept the initial values of the register bits which are set to 1 in the AND mask and are removed the ones
*   that are set to 0. On the other hand, the removed ones are set to 1 if they are set to 1 in the OR Mask. (A 1 in the OR Mask in the
*   same position where in the AND there is a 1 also, it is not considered)
*   @param Slave Slave number which it is requested the data.
*   @param Adress Initial address to write
*   @param AND_Mask AND Mask
*   @param OR_Mask OR mask
*   @return 0 Correct request
*   @return 1 It cannot be enqueued or wrong parameters
*   @sa Modbus_App_Enqueue_Or_Send, Modbus_App_Request
*/
unsigned char Modbus_Mask_Write_Register (unsigned char Slave, uint16_t Adress,
                                          uint16_t AND_Mask, uint16_t OR_Mask)
{ 
  if(Slave>247)   
    return 1;
  else      
  {
    Modbus_App_Request.Slave=Slave;
    Modbus_App_Request.Function=22;
    Modbus_App_Request.Data[0].UI2=Adress;
    Modbus_App_Request.Data[1].UI2=AND_Mask;
    Modbus_App_Request.Data[2].UI2=OR_Mask;
      
    if(Modbus_App_Enqueue_Or_Send())
      return 1;
    
    return 0;
  }
}

/**
*   @brief Read/Write multiple I/O Registers.
*
*   It writes some continuous Registers, after that a read is done and stored in the pointer *Response.
*   @param Slave Slave number which it is requested the data.
*   @param R_Adress Initial address of the read
*   @param R_Registers Number of Registers to read
*   @param *Response Pointer to where the read will be stored
*   @param W_Adress Initial address to write
*   @param W_Registers Number of Registers to write
*   @param *Value Pointer to where the values to write are stored
*   @return 0 Correct request
*   @return 1 It cannot be enqueued or wrong parameters
*   @sa Modbus_App_Enqueue_Or_Send, Modbus_App_Request
*/
unsigned char Modbus_Read_Write_M_Registers (unsigned char Slave, uint16_t R_Adress,
                                             uint16_t R_Registers, uint16_t *Response,
                                             uint16_t W_Adress, uint16_t W_Registers,
                                             uint16_t *Value)
{ 
  if(Slave>247 || Slave==0 || R_Registers>125 || R_Registers==0 || ((long)R_Adress+(long)R_Registers)>65535
     || W_Registers>121 || W_Registers==0 || ((long)W_Adress+(long)W_Registers)>65535)   
    return 1;
  else      
  {
    Modbus_App_Request.Slave=Slave;
    Modbus_App_Request.Function=23;
    Modbus_App_Request.Data[0].UI2=R_Adress;
    Modbus_App_Request.Data[1].UI2=R_Registers;
    Modbus_App_Request.Data[2].UI2=W_Adress;
    Modbus_App_Request.Data[3].UI2=W_Registers;
    Modbus_App_Request.Data[4].PUI2=Value;
    Modbus_App_Request.Data[5].PUI2=Response;
    
    if(Modbus_App_Enqueue_Or_Send())
      return 1;
    
    return 0;
  }
}
//! @}

/**
*   @defgroup App_Out Functions to give format to the Output
*   @ingroup App_Modbus
*   @brief It gives format to the Output message.
*
*   It gives the proper format to the output requests of the public Modbus functions implemented for both I/O reading and writing.
*   There are similarities between Modbus functions, so it is used the same function to give format to the message.
*
*   The request is stored in _Modbus_FIFO_Item_; There are stored the message chars of the Modbus PDU and the length of the message in
*   _Modbus_App_L_Req_pdu_.
*/
//! @{


/**
*   @brief Standard request format.
*
*   Some Modbus functions have an output format of five chars, so these ones are formatted in the same way with this function.
*   Although the meaning of the struct variables of the request is different, it is simply created a sequence of five bytes with the
*   number of the function firstly and the two first data splitted in two continuous bytes each one.
*   @sa Modbus_App_Req_pdu, Modbus_App_L_Req_pdu, struct Modbus_FIFO_Item
*   @sa Modbus_Read_Coils, Modbus_Read_D_Inputs, Modbus_Read_H_Registers
*   @sa Modbus_Read_I_Registers, Modbus_Write_Coil, Modbus_Write_Register
*/
void Modbus_App_Standard_Request(void)
{
  Modbus_App_Req_pdu[0]=Modbus_App_Actual_Req.Function;
  Modbus_App_Req_pdu[1]=Modbus_App_Actual_Req.Data[0].UI2>>8;
  Modbus_App_Req_pdu[2]=Modbus_App_Actual_Req.Data[0].UI2;
  Modbus_App_Req_pdu[3]=Modbus_App_Actual_Req.Data[1].UI2>>8; 
  Modbus_App_Req_pdu[4]=Modbus_App_Actual_Req.Data[1].UI2;
  Modbus_App_L_Req_pdu=5;
}

/**
*   @brief Format of the function Write Multiple Coils
*
*   The parameters are set in the first 6 bytes of the request (0-5) with the last one containing the number of the total bytes 
*   for the write. After that, Coils are wrapped, 8 per byte as one Coil is just one bit.
*   @sa Modbus_App_Req_pdu, Modbus_App_L_Req_pdu, struct Modbus_FIFO_Item
*   @sa Modbus_Write_M_Coils
*/
void Modbus_App_Write_M_Coils(void)
{
  unsigned char i, k;//j=0,k;
  uint16_t j=0;
  
  Modbus_App_Req_pdu[0]=Modbus_App_Actual_Req.Function;  
  Modbus_App_Req_pdu[1]=Modbus_App_Actual_Req.Data[0].UI2>>8;
  Modbus_App_Req_pdu[2]=Modbus_App_Actual_Req.Data[0].UI2;
  Modbus_App_Req_pdu[3]=Modbus_App_Actual_Req.Data[1].UI2>>8; 
  Modbus_App_Req_pdu[4]=Modbus_App_Actual_Req.Data[1].UI2;
      
  // Si el numero de Coils no es divisible por 8 el Nº de Bytes es superior
  // porque hay otro Byte con los bits restantes.
  if(Modbus_App_Actual_Req.Data[1].UI2%8==0)
    Modbus_App_Req_pdu[5]=Modbus_App_Actual_Req.Data[1].UI2/8;
  else
    Modbus_App_Req_pdu[5]=(Modbus_App_Actual_Req.Data[1].UI2/8)+1;
      
  // Empaquetado de los bits; "6+k" marca la posición en el vector, "j" el índice
  // en el origen de datos además de limitar el total de Coils a empaquetar,
  // "i" desplaza el bit a la posición dentro del Byte a enviar.
  for(k=0;j<Modbus_App_Actual_Req.Data[1].UI2;k++)
  {
    Modbus_App_Req_pdu[6+k]=0;
    for(i=0;i<8 && j<Modbus_App_Actual_Req.Data[1].UI2;i++)
      Modbus_App_Req_pdu[6+k]=Modbus_App_Req_pdu[6+k] | Modbus_App_Actual_Req.Data[2].PC[j++]<<i;
  } 
  
  Modbus_App_L_Req_pdu=6+Modbus_App_Req_pdu[5];          
}

/**
*   @brief Format of the function Write Multiple Registers.
*
*   The parameters are set in the first 6 bytes of the request (0-5) with the last one containing the number of the total bytes 
*   for the write. After that, Register values are wrapped, two bytes each one.
*   @sa Modbus_App_Req_pdu, Modbus_App_L_Req_pdu, struct Modbus_FIFO_Item
*   @sa Modbus_Write_M_Registers
*/
void Modbus_App_Write_M_Registers(void)
{
  unsigned char i;
  
  Modbus_App_Req_pdu[0]=Modbus_App_Actual_Req.Function;
  Modbus_App_Req_pdu[1]=Modbus_App_Actual_Req.Data[0].UI2>>8;
  Modbus_App_Req_pdu[2]=Modbus_App_Actual_Req.Data[0].UI2;
  Modbus_App_Req_pdu[3]=Modbus_App_Actual_Req.Data[1].UI2>>8; 
  Modbus_App_Req_pdu[4]=Modbus_App_Actual_Req.Data[1].UI2; 
  Modbus_App_Req_pdu[5]=Modbus_App_Actual_Req.Data[1].UI2*2;
  
  for(i=0;i<Modbus_App_Actual_Req.Data[1].UI2;i++)
  {
    Modbus_App_Req_pdu[6+2*i]=Modbus_App_Actual_Req.Data[2].PUI2[i]>>8;
    Modbus_App_Req_pdu[7+2*i]=Modbus_App_Actual_Req.Data[2].PUI2[i];
  }
  
  Modbus_App_L_Req_pdu=6+Modbus_App_Req_pdu[5];
}

/**
*   @brief Format of the function to Write a Register using Masks.
*
*   It is format a message of seven bytes(0-6) with the function in the first one, two bytes for the addres, two for the AND mask and
*   two for the OR mask.
*   @sa Modbus_App_Req_pdu, Modbus_App_L_Req_pdu, struct Modbus_FIFO_Item
*   @sa Modbus_Mask_Write_Register
*/
void Modbus_App_Mask_Write_Register(void)
{
  Modbus_App_Req_pdu[0]=Modbus_App_Actual_Req.Function;
  Modbus_App_Req_pdu[1]=Modbus_App_Actual_Req.Data[0].UI2>>8;
  Modbus_App_Req_pdu[2]=Modbus_App_Actual_Req.Data[0].UI2;
  Modbus_App_Req_pdu[3]=Modbus_App_Actual_Req.Data[1].UI2>>8; 
  Modbus_App_Req_pdu[4]=Modbus_App_Actual_Req.Data[1].UI2;
  Modbus_App_Req_pdu[5]=Modbus_App_Actual_Req.Data[2].UI2>>8;
  Modbus_App_Req_pdu[6]=Modbus_App_Actual_Req.Data[2].UI2;
  Modbus_App_L_Req_pdu=7;
}

/**
*   @brief Format the function Read/Write Multiple Registers.
*
*   In the first 5 bytes is set the data of the read request, as in the standard request; after that, it is set the bytes of the
*   write function similarly to its request function, but 5 positions behind.
*   @sa Modbus_App_Req_pdu, Modbus_App_L_Req_pdu, struct Modbus_FIFO_Item
*   @sa Modbus_Read_Write_M_Registers, Modbus_App_Standard_Request
*   @sa Modbus_App_Write_M_Registers
*/
void Modbus_App_Read_Write_M_Registers(void)
{
  unsigned char i;
  
  Modbus_App_Req_pdu[0]=Modbus_App_Actual_Req.Function;
  Modbus_App_Req_pdu[1]=Modbus_App_Actual_Req.Data[0].UI2>>8;
  Modbus_App_Req_pdu[2]=Modbus_App_Actual_Req.Data[0].UI2;
  Modbus_App_Req_pdu[3]=Modbus_App_Actual_Req.Data[1].UI2>>8; 
  Modbus_App_Req_pdu[4]=Modbus_App_Actual_Req.Data[1].UI2;
  Modbus_App_Req_pdu[5]=Modbus_App_Actual_Req.Data[2].UI2>>8;
  Modbus_App_Req_pdu[6]=Modbus_App_Actual_Req.Data[2].UI2;
  Modbus_App_Req_pdu[7]=Modbus_App_Actual_Req.Data[3].UI2>>8;
  Modbus_App_Req_pdu[8]=Modbus_App_Actual_Req.Data[3].UI2;
  Modbus_App_Req_pdu[9]=Modbus_App_Actual_Req.Data[3].UI2*2;
  
  for(i=0;i<Modbus_App_Actual_Req.Data[3].UI2;i++)
  {
    Modbus_App_Req_pdu[10+2*i]=Modbus_App_Actual_Req.Data[4].PUI2[i]>>8;
    Modbus_App_Req_pdu[11+2*i]=Modbus_App_Actual_Req.Data[4].PUI2[i];
  }
  
  Modbus_App_L_Req_pdu=10+Modbus_App_Req_pdu[9];
}
//! @}

/**
*   @defgroup App_CallBack Functions to manage the responses.
*   @ingroup App_Modbus
*   @brief It receives the response of the request.
*
*   It checks the received answers of the public Modbus functions requests to Read and Write I/O.
*   Also in this case, some Modbus functions have responses very similar, so it is used one common function for them.
*   If the received data is wrong, it is returned 1 to resend the request.
*
*   If it was a write request, before next request it is simply checked if it is the expected answer. If it was a read request,
*   it is checked if the answer is the expected one and also the read is stored in the specific pointer.
*/
//! @{

/**
*   @brief Read Bits.
*
*   It is used to check the operations to read Bits; If the Bytes counter (second char of the message) is equal to the expected one and
*   the message length is proper, the Bits are unwrapped and stored where the request pointer pointed.
*   @return 0 All correct
*   @return 1 Data error
*   @sa Modbus_App_Msg, Modbus_App_L_Msg, struct Modbus_FIFO_Item
*   @sa Modbus_App_Actual_Req, Modbus_Read_Coils, Modbus_Read_D_Inputs
*/
unsigned char Modbus_App_Read_Single_Bits_CallBack(void)
{
  unsigned char i,j;//,k=0;
  uint16_t k=0;
  
  if(Modbus_App_Actual_Req.Data[1].UI2%8==0)
  {
    if(Modbus_App_Msg[1]!=Modbus_App_Actual_Req.Data[1].UI2/8 ||
       Modbus_App_L_Msg!=(Modbus_App_Actual_Req.Data[1].UI2/8)+2)
        return 1;
  }
  else
  {
    if(Modbus_App_Msg[1]!=(Modbus_App_Actual_Req.Data[1].UI2/8)+1 ||
       Modbus_App_L_Msg!=(Modbus_App_Actual_Req.Data[1].UI2/8)+3)
        return 1;
  }
  
  // Desempaquetar los bits; "i+2" marca la posición en la respuesta, "k" el 
  // índice en el vector donde se guardan los bits y limita cuando se llega al 
  // total de bits, "j" desplaza el bit a la primera posición para que "& 1" 
  // elimine los otros y lo deje preparado para guardarlo.
  for(i=0;i<Modbus_App_Msg[1];i++)
    for(j=0;j<8 && k<Modbus_App_Actual_Req.Data[1].UI2;j++)
    {
      Modbus_App_Actual_Req.Data[2].PC[k]=(Modbus_App_Msg[i+2]>>j) & 1;
      k++;
    }  
    return 0;
}

/**
*   @brief Read Registers.
*
*   It is used to check the operations to read Registers; If the Bytes counter (second char of the message) is equal to the 
*   expected one and the message length is proper, the Registers (2 bytes) are unwrapped and stored where the request pointer pointed.   
*   @return 0 All correct
*   @return 1 Data error
*   @sa Modbus_App_Msg, Modbus_App_L_Msg, struct Modbus_FIFO_Item
*   @sa Modbus_Read_H_Registers, Modbus_Read_I_Registers
*/
unsigned char Modbus_App_Read_Registers_CallBack(void)
{
  unsigned char i;
  
  if(Modbus_App_Msg[1]!=Modbus_App_Actual_Req.Data[1].UI2*2 ||
     Modbus_App_L_Msg!=2+Modbus_App_Actual_Req.Data[1].UI2*2)
    return 1;
  
  for(i=0;i<Modbus_App_Msg[1]/2;i++)
    Modbus_App_Actual_Req.Data[2].PUI2[i]=(Modbus_App_Msg[2*i+2]<<8) | Modbus_App_Msg[2*i+3];
  
  return 0;
}

/**
*   @brief It checks Write responses.
*
*   It is used to check the operations to write simple/multiple Coils and Registers; It checks that the answer is an echo of the request.
*   @return 0 All correct
*   @return 1 Data error
*   @sa Modbus_App_Msg, Modbus_App_L_Msg, struct Modbus_FIFO_Item 
*   @sa Modbus_Write_Coil, Modbus_Write_Register 
*   @sa Modbus_Write_M_Coils, Modbus_Write_M_Registers
*/
unsigned char Modbus_App_Write_CallBack(void)
{  
  if((Modbus_App_Msg[1]<<8|Modbus_App_Msg[2])!=Modbus_App_Actual_Req.Data[0].UI2 ||
     (Modbus_App_Msg[3]<<8|Modbus_App_Msg[4])!=Modbus_App_Actual_Req.Data[1].UI2 ||
      Modbus_App_L_Msg!=5)
    return 1;
  
  return 0;
}

/**
*   @brief It checks the Write using maks responses.
*
*   It checks that the answer is an echo of the request, in this case, seven bytes.
*   @return 0 All correct
*   @return 1 Data error
*   @sa Modbus_App_Msg, Modbus_App_L_Msg, struct Modbus_FIFO_Item 
*   @sa Modbus_Mask_Write_Register
*/
unsigned char Modbus_App_Mask_Write_CallBack(void)
{  
  if((Modbus_App_Msg[1]<<8|Modbus_App_Msg[2])!=Modbus_App_Actual_Req.Data[0].UI2 ||
     (Modbus_App_Msg[3]<<8|Modbus_App_Msg[4])!=Modbus_App_Actual_Req.Data[1].UI2 ||
     (Modbus_App_Msg[5]<<8|Modbus_App_Msg[6])!=Modbus_App_Actual_Req.Data[2].UI2 ||
      Modbus_App_L_Msg!=7)
    return 1;
  
  return 0;
}

/**
*   @brief It checks the Read/Write Registers responses.
*
*   The answer of this function is similar to Read Registers function, so the same process is done. It is used a different function
*   because the pointer where it should be stored the registers read is placed in a different data vector index of the request 
*   structure.
*   @return 0 All correct
*   @return 1 Data error
*   @sa Modbus_App_Msg, Modbus_App_L_Msg, struct Modbus_FIFO_Item 
*   @sa Modbus_Read_Write_M_Registers, Modbus_Read_H_Registers
*/
unsigned char Modbus_App_Read_Write_M_Registers_CallBack(void)
{
  unsigned char i;
  
  if(Modbus_App_Msg[1]!=Modbus_App_Actual_Req.Data[1].UI2*2 ||
     Modbus_App_L_Msg!=2+Modbus_App_Actual_Req.Data[1].UI2*2)
    return 1;
  
  for(i=0;i<Modbus_App_Msg[1]/2;i++)
    Modbus_App_Actual_Req.Data[5].PUI2[i]=(Modbus_App_Msg[2*i+2]<<8) | Modbus_App_Msg[2*i+3];
  
  return 0;
}
//! @}