// Author: Francisco Javier Guzman Jimenez, <dejavits@gmail.com>
/**
*   @mainpage  Slave Program Manual
*
*   @author Francisco Javier Guzman Jimenez, <dejavits@gmail.com>
*
*   This document shows in detail the control application for the Modbus communications in the slave devices; Hereby, it is explained
*   the functionality and use of the program to make easier its understanding. In that way, it eases the extension of the program if 
*   it is needed in the future.
*   it eases the program expansion
*
*   The application is made up by two modules; The App module is the main application and the only one allowed to access to the slave
*   implementation; The OSL Module contains elements for the OSl communication through the serial port using RTU as codification.
*   It is left to be done the implementation using OSL with ASCII as codification and the TCP implementation. 
*
*   For the proper slave configuration, in the user application it has to be created some vectors with the I/O data mapped into memory.
*   This application handles all this information using pointers to such vectors. This pointers are fixed at _Modbus_Slave_Init()_. 
*
*
*/

/**
*   @defgroup App Modbus App
*   @brief Modbus Application layer.
*
*   This module includes the application layer of the Modbus protocol following
*   its specifics. It is implemented the user Modbus functions related with both
*   analogic and digital I/O. These functions build/manage chunks of bytes known as
*   Protocol Data Unit (PDU). The maximum length is 253 bytes per message in OSL and 256 bytes in CAN.
*   It is used big-endian encoding.
*
*    Also, this module configures the Slave, it is assigned its slave number and
*    establish the I/O, communication mode and its options. It manages the received request to
*    make the requested actions and return the response following the specifics of Modbus.
*    To utilise the user functions to be able to set communications is needed to include
*    _Modbus_App.h_
*/
//! @{

#include "Modbus_App.h"

//*****************************************************************************
//
// Variables locales del módulo App usadas en el desarrollo del programa para
// la comprobación del comportamiento del sistema y la depuración de errores.
//
//*****************************************************************************

//unsigned char Debug_App_Sent=0, Debug_App_E1=0,Debug_App_E2=0,
//              Debug_App_E3=0;

//*****************************************************************************
//
// Variables globales del módulo App.
//
//*****************************************************************************

//! Vector to store the incoming PDUs.
static unsigned char Modbus_App_Msg[MAX_PDU];

//! Incoming message length.
static unsigned char Modbus_App_L_Msg;

//! Vector to store the outcoming PDUs.
static unsigned char Modbus_App_Response_pdu[MAX_PDU];

//! Outcoming message length.
static unsigned char Modbus_App_L_Response_pdu;

//! Internal variable to store data addresses of incoming messages.
static uint16_t Modbus_App_Adress;

//! Internal variable to store the amount of data to read/write in incoming messages.
static uint16_t Modbus_App_Quantity;

//! Auxiliary internal variable to store different values.
static uint16_t Modbus_App_Value;

//! Number of available coils in the slave.
static uint16_t Modbus_App_N_Coils;

//! Number of available discrete inputs in the slave.
static uint16_t Modbus_App_N_D_Inputs;

//! Number of available input registers in the slave.
static uint16_t Modbus_App_N_I_Registers;

/** @brief Number of available holding registers in the slave. Actually, these registers can be considered as internal memory, 
*   but as read/write operations work with them, they are called I/O as well.
*/
static uint16_t Modbus_App_N_H_Registers;

/** @brief Pointer to the mapped coils. Any vector value different than 1 or 0 will arise wrong reads as it does not have the rest of the
*   seven bits to 0.
*/
static unsigned char *Modbus_App_Coils;

/** @brief Pointer to the mapped discrete inputs. Any vector value different than 1 or 0 will arise wrong reads as it does not have the 
*   rest of the seven bits to 0.
*/
static unsigned char *Modbus_App_D_Inputs;

//! Pointer to the mapped holding registers.
static uint16_t *Modbus_App_H_Registers;

//! Pointer to the mapped input registers.
static uint16_t *Modbus_App_I_Registers;

//! Modbus communication mode; It is only implemented OSL with RTU codification and CAN.
static enum Modbus_Comm_Modes Modbus_Comm_Mode;

//! Bit rate range.
enum Modbus_CAN_BitRate bit_rate_range;
//! @}

//*****************************************************************************
//
// Prototipos de las funciones locales del módulo App.
//
//*****************************************************************************

// De Comprobación de Datos.

static unsigned char Modbus_App_Read_Coils_Check(void);
static unsigned char Modbus_App_Read_D_Inputs_Check(void);
static unsigned char Modbus_App_Read_H_Registers_Check(void);
static unsigned char Modbus_App_Read_I_Registers_Check(void);
static unsigned char Modbus_App_Write_Coil_Check(void);
static unsigned char Modbus_App_Write_Register_Check(void);
static unsigned char Modbus_App_Write_M_Coils_Check(void);
static unsigned char Modbus_App_Write_M_Registers_Check(void);
static unsigned char Modbus_App_Mask_Write_Register_Check(void);
static unsigned char Modbus_App_Read_Write_M_Registers_Check(void);

// De Ejecución de las Acciones demandadas.

static void Modbus_App_Read_Coils(void);
static void Modbus_App_Read_D_Inputs(void);
static void Modbus_App_Read_H_Registers(void);
static void Modbus_App_Read_I_Registers(void);
static void Modbus_App_Write_Coil(void);
static void Modbus_App_Write_Register(void);
static void Modbus_App_Write_M_Coils(void);
static void Modbus_App_Write_M_Registers(void);
static void Modbus_App_Mask_Write_Register(void);
static void Modbus_App_Read_Write_M_Registers(void);
  
// De Control de la Aplicación.

static unsigned char Modbus_App_Check_Request_Data(void);
static void Modbus_App_Process_Action(void);

/**
* @defgroup App_Control Application Control
* @ingroup App
* @brief Functions to initialise and control de application.
*
* The I/O is mapped in data memory, the type of the Modbus communication
* and its configuration is done also. It manages the communications and actions
* to be done with both input and output messages.
*/
////////////////////////////////////////////////////////////////////////////////
/**
*   @defgroup App_Exchange Interconnection between OSL/CAN Layer and App Layer
*   @ingroup App
*   @brief Functions to exchange data between App and OSL/CAN module.
*
*   It includes functions to exchange information data between the CAN/OSL and App module,
*   allowing APP to send/receive messages through CAN/OSL.
*/

/**
*   @defgroup App_Modbus Modbus functions
*   @ingroup App
*   @brief It implements the checking, elaboration and answer of the Modbus public functions.
*
*   For each public user function of Modbus that is received, the slave checks, executes and replies according to the next functions;
*   They do not establish any communication, they only check and process data building up a response PDU if it is needed, following the
*   Modbus specifics.
*/

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////OSL Mode//////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#ifdef OSL_Mode
//! \brief Configura el Slave.
//! \ingroup App_Control
//!
//! Fija el Nº de Identificación del Slave, la cantidad de E/S de cada tipo
//! habilitadas y determina el Modo de Comunicaciones de Modbus. Las E/S de 
//! cada tipo se almacenan en los vectores apuntados por los punteros de los
//! parámetros. La dirección en dicho vector es de 0 al Total de E/S de cada 
//! tipo habilitadas,sin huecos; Es decir que si se habilitan 3 Coils éstas se  
//! corresponderán necesariamente con las posiciones 0-2 del vector. Por tanto,
//! es necesario que el usuario cree en su programa unos vectores donde mapee
//! las E/S da cada tipo y los enlace con los punteros de los parámetros de 
//! esta función. Llama a la función de configuración de las Comunicaciones 
//! \param N_Coils Cantidad de Coils habilitadas
//! \param *Coils Dirección donde apuntará Modbus_App_Coils
//! \param N_D_Inputs Cantidad de Entradas Discretas habilitadas
//! \param *D_Inputs Dirección donde apuntará Modbus_App_D_Inputs
//! \param N_H_Registers Cantidad de Registros de E/S habilitados
//! \param *H_Registers Dirección donde apuntará Modbus_App_H_Registers
//! \param N_I_Registers Cantidad de Registros de Entrada habilitados
//! \param *I_Registers Dirección donde apuntará Modbus_App_I_Registers
//! \param Com_Mode Modo de Comunicación de Modbus.
//! \param Slave  Nº de Identificación del Slave
//! \param Baudrate  Baudrate de las comunicaciones
//! \param OSL_Mode  Mode RTU/ASCII de la comunicación Serie.
//! \return 1 ERROR: Nº Slave incorrecto o opción de comunicación no Existente 
//! \return 0 Todo correcto
//! \sa Modbus_App_N_Coils, Modbus_App_N_D_Inputs, Modbus_App_N_H_Registers
//! \sa Modbus_App_N_I_Registers, Modbus_Comm_Mode, Modbus_OSL_Init
unsigned char Modbus_Slave_Init(uint16_t N_Coils, unsigned char *Coils, 
                                uint16_t N_D_Inputs, unsigned char *D_Inputs,
                                uint16_t N_H_Registers, uint16_t *H_Registers,
                                uint16_t N_I_Registers, uint16_t *I_Registers,
                                enum Modbus_Comm_Modes Com_Mode, 
                                unsigned char Slave, enum Baud Baudrate,
                                enum Modbus_OSL_Modes OSL_Mode)
{
  // Cantidades de E/S habilitadas.
  Modbus_App_N_Coils=N_Coils;
  Modbus_App_N_D_Inputs=N_D_Inputs;
  Modbus_App_N_H_Registers=N_H_Registers;
  Modbus_App_N_I_Registers=N_I_Registers;

  // Apuntar hacia los vectores de E/S que haya definido el usuario.
  Modbus_App_Coils=Coils;
  Modbus_App_D_Inputs=D_Inputs;
  Modbus_App_H_Registers=H_Registers;
  Modbus_App_I_Registers=I_Registers;
  
  // Modo por defecto: Serie.
  if (Com_Mode == CDEFAULT) 
    Modbus_Comm_Mode=MODBUS_SERIAL;
    
  switch(Modbus_Comm_Mode)
    {
      case (MODBUS_SERIAL):
        return Modbus_OSL_Init(Slave,Baudrate,OSL_Mode);
        break;
        
      /* Añadir en caso de Implementar otros modos de Comunicación. */
        
      default:
        return 1;
        break;
    }
}

//! \brief Función de Usuario Para la Comunicación.
//! \ingroup App_Control
//!
//! Esta es la función que el usuario de esta aplicación debe incorporar a su
//! bucle de proceso para que se realicen las Comunicaciones. Previamente se 
//! debe llamar a Modbus_Slave_Init para configurar las comunicaciones. Para la 
//! programación del Slave resulta imprescindible además que el bucle incluya el 
//! Mapeo de E/S a sus Vectores de usuario leyendo las entradas al principio del 
//! bucle y escribiendo las salidas al final.
//!  
//! \sa Modbus_OSL_Serial_Comm, Modbus_OSL_Init, Modbus_Slave_Init, Modbus_CAN_Init, Modbus_CAN_Controller
void Modbus_Slave_Communication (void)
{
  Modbus_OSL_Serial_Comm();
}

//! \brief Gestión de las Peticiones Recibidas.
//! \ingroup App_Control
//!
//! Comprueba la corrección de los datos de la petición y si son correctos
//! realiza las acciones correspondientes a dicha petición y prepara el mensaje
//! de respuesta. En caso contrario prepara la respuesta de Excepción pertinente.
//! Si la petición es de BroadCast no se enviará respuesta alguna. Los Errores
//! que provocan mensajes de respuesta de excepción son:
//! > - __Tipo_1__: Numero de función no implementada o petición de lectura en
//! >     modo BroadCast, es decir sin sentido al no poder enviar la lectura.
//! > - __Tipo_2__: Dirección Inalcanzable. Los datos son acordes a las
//! >     especificaciones pero se llega a una dirección de datos no existente
//! >     en la configuración del dispositivo. Por ejemplo, habilitadas sólo
//! >     5 coils (direcciones 0-4) del slave escribir en la dirección 5 o leer
//! >     4 coils empezando en la dirección 2.
//! > - __Tipo_3__: Valor de algún campo de Datos incorrecto. Algún dato no se
//! >     corresponde con las especificaciones de Modbus Serie, por ejemplo,
//! >     la lectura de más de 2000 coils o 125 Registros o el numero de datos
//! >     no cuadra con el esperado aún superando los otros filtros de error.
//! Los mensajes de excepción se componen de 2 Bytes, el primero con el numero
//! de función de la petición con el primer bit a 1 y el segundo con el numero
//! del tipo del error, de 1 a 3.
//! \sa Modbus_App_Check_Request_Data, Modbus_App_Process_Action
void Modbus_App_Manage_Request (void)
{
  // Analizar la corrección de datos. Devuelve 0 si es correcto o el numero del
  // tipo de error detectado.
  switch(Modbus_App_Check_Request_Data())
  {
    case 0:
      /* Datos Correctos. Procesar acción. */
      Modbus_OSL_MainState_Set(MODBUS_OSL_PROCESSING);
      Modbus_App_Process_Action();
      break;
    case 1:
      //Error tipo 1.
      if(Modbus_OSL_BroadCast_Get()==0)
      {
        Modbus_OSL_MainState_Set(MODBUS_OSL_ERROR);
        Modbus_App_Response_pdu[0]=Modbus_App_Msg[0] | 128;
        Modbus_App_Response_pdu[1]=1;
        Modbus_App_L_Response_pdu=2;
        //Debug_App_E1++;
      }
        break;
    case 2:
      //Error tipo 2.
      if(Modbus_OSL_BroadCast_Get()==0)
      {
        Modbus_OSL_MainState_Set(MODBUS_OSL_ERROR);
        Modbus_App_Response_pdu[0]=Modbus_App_Msg[0] | 128;
        Modbus_App_Response_pdu[1]=2;
        Modbus_App_L_Response_pdu=2;
        //Debug_App_E2++;
      }
      break;
    case 3:
      //Error tipo 3.
      if(Modbus_OSL_BroadCast_Get()==0)
      {
        Modbus_OSL_MainState_Set(MODBUS_OSL_ERROR);
        Modbus_App_Response_pdu[0]=Modbus_App_Msg[0] | 128;
        Modbus_App_Response_pdu[1]=3;
        Modbus_App_L_Response_pdu=2;
        //Debug_App_E3++;
      }
      break;
    default:

      /* Si se implementan otros tipos de error pueden añadirse aquí, de momento
       Nunca debería llegar a esta opción. */

      Modbus_Fatal_Error(10);
      break;
  }
}

//! \brief Función de Comprobación de datos de la petición.
//! \ingroup App_Control
//!
//! Dependiendo del numero de función de la petición realizada se comprueban
//! los datos del mensaje, aprovechando para almacenar los datos analizados en
//! las variables _Modbus_App_Adress_, _Modbus_App_Quantity_ y
//! _Modbus_App_Value_ para mayor agilidad en su uso posterior y se devuelve 0
//! como OK o el numero del Tipo de Error para el mensaje de excepción.
//! \sa Modbus_App_Adress, Modbus_App_Quantity, Modbus_App_Value
//! \sa Modbus_App_Read_Coils_Check, Modbus_App_Read_D_Inputs_Check
//! \sa Modbus_App_Read_H_Registers_Check, Modbus_App_Read_I_Registers_Check
//! \sa Modbus_App_Write_Coil_Check, Modbus_App_Write_Register_Check
//! \sa Modbus_App_Write_M_Coils_Check, Modbus_App_Write_M_Registers_Check
//! \sa Modbus_App_Mask_Write_Register_Check, Modbus_App_Read_Write_M_Registers_Check
static unsigned char Modbus_App_Check_Request_Data()
{
  // Byte con el Nº de Función del Mensaje.
  switch(Modbus_App_Msg[0])
  {
    //Los casos con comprobación del flag de BroadCast son funciones de lectura,
    //no admiten modo BroadCast puesto que carecen de sentido sin devolver la
    //lectura de los valores demandados. Aunque devuelvan Error Tipo 1 no se
    //realizará mensaje de excepción al ser una petición BroadCast.
    case 1:
      if(Modbus_OSL_BroadCast_Get()==0)
        return Modbus_App_Read_Coils_Check();
      else
        return 1;
      break;
    case 2:
      if(Modbus_OSL_BroadCast_Get()==0)
        return Modbus_App_Read_D_Inputs_Check();
      else
        return 1;
      break;
    case 3:
      if(Modbus_OSL_BroadCast_Get()==0)
        return Modbus_App_Read_H_Registers_Check();
      else
        return 1;
      break;
    case 4:
      if(Modbus_OSL_BroadCast_Get()==0)
        return Modbus_App_Read_I_Registers_Check();
      else
        return 1;
      break;
    case 5:
      return Modbus_App_Write_Coil_Check();
      break;
    case 6:
      return Modbus_App_Write_Register_Check();
      break;
    case 15:
      return Modbus_App_Write_M_Coils_Check();
      break;
    case 16:
      return Modbus_App_Write_M_Registers_Check();
      break;
    case 22:
      return Modbus_App_Mask_Write_Register_Check();
      break;
    case 23:
      if(Modbus_OSL_BroadCast_Get()==0)
        return Modbus_App_Read_Write_M_Registers_Check();
      else
        return 1;
      break;
    default:
      return 1;
      break;
  }
}

//! \brief Envía un mensaje de Salida.
//! \ingroup App_Exchange
//!
//! Una vez montado el mensaje de salida se envía por el puerto serie mediante
//! una llamada a _Modbus_OSL_Output_. En caso de implementar otros modos de
//! comunicación esta función debería comprobar el modo y llamar a la función
//! de envío correspondiente al caso.
//! \sa Modbus_OSL_Output, Modbus_App_Response_pdu, Modbus_App_L_Response_pdu
void Modbus_App_Send(void)
{
  Modbus_OSL_Output (Modbus_App_Response_pdu,Modbus_App_L_Response_pdu);
  //Debug_App_Sent++;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////CAN MODE//////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#elif CAN_Mode
/**
*   @brief It configures the slave.
*   @ingroup App_Control
*
*   It is set the slave number and also the amount of I/O of each type. The I/O of each type
*   are stored in the vectors pointed by the pointers of the parameters.
*   The addresses in such vectors goes from 0 to the total I/O amount depending on the type of I/O. For example,
*   if it is enabled 3 Coils, they will be at the positions 0-2 into the vector. Therefore, it is necessary 
*   for the user to make his program using mapped vectors to each type of I/O and the pointers pointing to these ones.
*   After all the mapping, it it inisialised the CAN module.
*   @param N_Coils Amount of Coils
*   @param *Coils Modbus_App_Coils address
*   @param N_D_Inputs Amount of Discrete Inputs
*   @param *D_Inputs Modbus_App_D_Inputs address
*   @param N_H_Registers Amount of Holding Registers
*   @param *H_Registers Modbus_App_H_Registers address
*   @param N_I_Registers Amount of Input Registers
*   @param *I_Registers Modbus_App_I_Registers address
*   @param bit_rate Bit rate in CAN communications
*   @param slave  Slave number
*   @return 1 Slave number incorrect
*   @return 0 All correct
*   @sa Modbus_App_N_Coils, Modbus_App_N_D_Inputs, Modbus_App_N_H_Registers
*   @sa Modbus_App_N_I_Registers, Modbus_Comm_Mode, Modbus_OSL_Init
*/
unsigned char Modbus_Slave_Init(uint16_t N_Coils, unsigned char *Coils,
                                uint16_t N_D_Inputs, unsigned char *D_Inputs,
                                uint16_t N_H_Registers, uint16_t *H_Registers,
                                uint16_t N_I_Registers, uint16_t *I_Registers,
                                enum Modbus_CAN_BitRate bit_rate, unsigned char slave)
{
  // Amount of available I/O
  Modbus_App_N_Coils=N_Coils;
  Modbus_App_N_D_Inputs=N_D_Inputs;
  Modbus_App_N_H_Registers=N_H_Registers;
  Modbus_App_N_I_Registers=N_I_Registers;

  // Vector mapping
  Modbus_App_Coils=Coils;
  Modbus_App_D_Inputs=D_Inputs;
  Modbus_App_H_Registers=H_Registers;
  Modbus_App_I_Registers=I_Registers;
  bit_rate_range = bit_rate;
  if(slave <= 247)
  {
    Modbus_CAN_Init(bit_rate_range, slave);
    return 0;
  }
  else
    return 1;
}

/** 
*    @brief User function for the communication.
*    @ingroup App_Control
*
*    This function has to be included in the process loop of the user program to
*    make possible the communications. Previously, it has to be called _Modbus_Slave_Init()_
*    to configure the communication parameters. The I/O mapping is also done in the former function.
*    @sa Modbus_OSL_Serial_Comm, Modbus_OSL_Init, Modbus_Slave_Init, Modbus_CAN_Controller, Modbus_CAN_Init
*/
void Modbus_Slave_Communication (void)
{
	Modbus_CAN_Controller();
}

/**
*   @brief Received Request Management
*   @ingroup App_Control
*
*   Check if data of the request is correct, if so, it is prepared the response.
*   Otherwise, it is prepared the exception message as answer. If the request is broadcast,
*   it will not be sent any answer. The different errors which can provoke exceptions are:
*   > - __Type_1__: Number of function not implemented or a read request in broadcast, which it does not make sense as it is 
*   not possible to send a response.
*   > - __Type_2__: Unattainable address. The data is correct but the address is not valid in such a slave.
*   > - __Type_3__: Some data is invalid or it does not follow the Modbus specifics.
*   The exception messages are compounded by two bytes. The first one with the function number and the first bit to 1,
*   and the second byte with the error type number, this is from 1 to 3.
*   @sa Modbus_App_Check_Request_Data, Modbus_App_Process_Action
*/
void Modbus_App_Manage_Request (void)
{ 
  // Check the data. Return 0 if there is no error or the number of the error type.
  switch(Modbus_App_Check_Request_Data())
  {
    case 0:
      /* Correct data. Accion processing. */
      Modbus_SetMainState(MODBUS_PROCESSING);
      Modbus_App_Process_Action();
      break;
    case 1:
      //Type 1 error
      if(Modbus_CAN_BroadCast_Get()==0)
      { 
        Modbus_SetMainState(MODBUS_ERROR);
        Modbus_App_Response_pdu[0]=Modbus_App_Msg[0] | 128;
        Modbus_App_Response_pdu[1]=1;
        Modbus_App_L_Response_pdu=2;        
      }
        break;
    case 2:
      //Type 2 error
      if(Modbus_CAN_BroadCast_Get()==0)
      {
        Modbus_SetMainState(MODBUS_ERROR);
        Modbus_App_Response_pdu[0]=Modbus_App_Msg[0] | 128;
        Modbus_App_Response_pdu[1]=2;
        Modbus_App_L_Response_pdu=2;        
      }
      break;
    case 3:
      //Type 3 error
      if(Modbus_CAN_BroadCast_Get()==0)
      {
        Modbus_SetMainState(MODBUS_ERROR);
        Modbus_App_Response_pdu[0]=Modbus_App_Msg[0] | 128;
        Modbus_App_Response_pdu[1]=3;
        Modbus_App_L_Response_pdu=2;        
      }
      break;
    default: /*weird case*/
            Modbus_CAN_Error_Management(10);
            break;
  }
}

/**
*   @brief Function to check the request data.
*   @ingroup App_Control
*
*   Depending on the function number of the request, it is checked the data of the message.
*   In addition, the information about the data requested is stored temporarily in the variables _Modbus_App_Adress_, 
*   _Modbus_App_Quantity_ and _Modbus_App_Value for its later use. It is returned 0 is no errors occurred, or the number of error 
*   for the exception message.
*   @sa Modbus_App_Adress, Modbus_App_Quantity, Modbus_App_Value 
*   @sa Modbus_App_Read_Coils_Check, Modbus_App_Read_D_Inputs_Check
*   @sa Modbus_App_Read_H_Registers_Check, Modbus_App_Read_I_Registers_Check
*   @sa Modbus_App_Write_Coil_Check, Modbus_App_Write_Register_Check
*   @sa Modbus_App_Write_M_Coils_Check, Modbus_App_Write_M_Registers_Check
*   @sa Modbus_App_Mask_Write_Register_Check, Modbus_App_Read_Write_M_Registers_Check
*/
static unsigned char Modbus_App_Check_Request_Data()
{
  // Byte[0] = Function number
  switch(Modbus_App_Msg[0])
  {
    /*
        If the request was a broadcast one, and the function is a read, then 
        it is marked as error type 1 but it will not generate any exception message
        as is because of the broadcasd, and the master will not expect any answer of any kind.
    */    
    case 1:
      if(Modbus_CAN_BroadCast_Get()==0)
        return Modbus_App_Read_Coils_Check();
      else
        return 1;
      break;
    case 2:
      if(Modbus_CAN_BroadCast_Get()==0)
        return Modbus_App_Read_D_Inputs_Check();
      else
        return 1;
      break;
    case 3:
      if(Modbus_CAN_BroadCast_Get()==0)
        return Modbus_App_Read_H_Registers_Check();
      else
        return 1;
      break;
    case 4:
      if(Modbus_CAN_BroadCast_Get()==0)
        return Modbus_App_Read_I_Registers_Check();
      else
        return 1;
      break;
    case 5:
      return Modbus_App_Write_Coil_Check();
      break;
    case 6:
      return Modbus_App_Write_Register_Check();
      break;
    case 15:
      return Modbus_App_Write_M_Coils_Check();
      break;
    case 16:
      return Modbus_App_Write_M_Registers_Check();
      break;
    case 22:
      return Modbus_App_Mask_Write_Register_Check();
      break;
    case 23:
      if(Modbus_CAN_BroadCast_Get()==0)
        return Modbus_App_Read_Write_M_Registers_Check();
      else
        return 1;
      break;
    default:
      return 1;
      break;  
  }
}

/**
*   @brief Send an output message.
*   @ingroup App_Exchange
*
*   Once the output message is built, it is sent by CAN calling to
*   _Modbus_CAN_FixOutput()_. 
*   @sa Modbus_OSL_Output, Modbus_App_Response_pdu, Modbus_App_L_Response_pdu, Modbus_CAN_FixOutput
*/
void Modbus_App_Send(void)
{
  Modbus_CAN_FixOutput (Modbus_App_Response_pdu, Modbus_App_L_Response_pdu);  
}
#endif
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////SEPARATION ENDING////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
*   @brief Process the required action.
*   @ingroup App
*
*   Depending on the function number of the request a precise function is called.
*   Such functions prepare also the normal response of the petition, storing it at _Modbus_App_Response_pdu_.
*   @sa Modbus_App_Response_pdu, Modbus_App_Read_Coils, Modbus_App_Read_D_Inputs  
*   @sa Modbus_App_Read_H_Registers, Modbus_App_Read_I_Registers
*   @sa Modbus_App_Write_Coil, Modbus_App_Write_Register
*   @sa Modbus_App_Write_M_Coils, Modbus_App_Write_M_Registers
*   @sa Modbus_App_Mask_Write_Register, Modbus_App_Read_Write_M_Registers
*/
static void Modbus_App_Process_Action(void)
{
  // Byte in which the function number is stored.
  switch(Modbus_App_Msg[0])
  {

    case 1:
      Modbus_App_Read_Coils();
      break;
    case 2:
      Modbus_App_Read_D_Inputs();
      break;
    case 3:
      Modbus_App_Read_H_Registers();
      break;
    case 4:
      Modbus_App_Read_I_Registers();
      break;
    case 5:
      Modbus_App_Write_Coil();
      break;
    case 6:
      Modbus_App_Write_Register();
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
      Modbus_CAN_Error_Management(20);
      break;  
  }
}

/**
*   @brief It receives a char from another module.
*   @ingroup App_Exchange
*
*   It stores chars en some position of the vector _Modbus_App_Msg_; It is used in _Modbus_OSL_RTU_to_App() (which it uses also 
*   _Modbus_OSL_RTU_Char_Get()_) or in _Modbus_CAN_to_App()_ to transfer the messages from these modules to App without linking both modules
*   neither making a copy of the CAN/OSL message.
*   @param Msg Char value to store in _Modbus_App_Msg_
*   @param i Vector index where the char will be placed
*   @sa Modbus_App_Msg, Modbus_OSL_RTU_to_App, Modbus_OSL_RTU_Char_Get
*/
void Modbus_App_Receive_Char (unsigned char Msg, unsigned char i)
{
    Modbus_App_Msg[i]=Msg;
  
}

/**
*   @brief Recibe la Longitud del Mensaje.
*   @ingroup App_Exchange
*
*   It is used in _Modbus_OSL_RTU_to_App()_ or _Modbus_CAN_to_App()_ to send from CAN/OSL layer to App layer the incoming message length.
*   @param Index Length message value
*   @sa Modbus_App_Msg, Modbus_App_L_Msg, Modbus_OSL_RTU_to_App
*/
void Modbus_App_L_Msg_Set(unsigned char Index)
{
  Modbus_App_L_Msg=Index;
}

////////////////////////////////////////////////////////////////////////////////////////
/**
*   @defgroup App_Check Checking functions
*   @ingroup App_Modbus
*   @brief It checks the data correctness of the incoming PDU requests.
*
*   It returns 0 if all is correct or the number of error to generate later the exception response.
*   Also, it is stored the needed value in the auxiliary variables _Modbus_App_Adress_, _Modbus_App_Quantity_ and Modbus_App_Value_
*   to make use of them later.
*   @{ 
*/
//////////////////////////////////
/*   
*   @brief Data check of Read Coils request.
*
*   It stores in _Modbus_App_Adress_ the initial address and in _Modbus_App_Quantity_ the amount of coils to be read;
*   After that, it checks if data meets the specifics; if so, 0 is returned, if not, the error type is returned.
*   @return 0 Correct Data
*   @return 2 I/O requested not available
*   @return 3 Function data error
*   @sa Modbus_App_Msg, Modbus_App_L_Msg, Modbus_App_N_Coils 
*   @sa Modbus_App_Adress, Modbus_App_Quantity, Modbus_App_Read_Coils
*/
static unsigned char Modbus_App_Read_Coils_Check(void)
{
  Modbus_App_Adress=Modbus_App_Msg[1]<<8|Modbus_App_Msg[2];
  Modbus_App_Quantity=Modbus_App_Msg[3]<<8|Modbus_App_Msg[4];
  
  if(Modbus_App_Quantity>2000 || Modbus_App_Quantity==0 || Modbus_App_L_Msg!=5)
    return 3;
  if( ((long)Modbus_App_Adress+(long)Modbus_App_Quantity)>Modbus_App_N_Coils)
    return 2;
    
  return 0; 
}

/**
*   @brief Data check of Read Discrete Inputs request.
*
*   It stores in _Modbus_App_Adress_ the initial address and in _Modbus_App_Quantity_ the amount of coils to be read;
*   After that, it checks if data meets the specifics; if so, 0 is returned, if not, the error type is returned.
*   @return 0 Correct Data
*   @return 2 I/O requested not available
*   @return 3 Function data error
*   @sa Modbus_App_Msg, Modbus_App_L_Msg, Modbus_App_N_D_Inputs 
*   @sa Modbus_App_Adress, Modbus_App_Quantity, Modbus_App_Read_D_Inputs
*/
static unsigned char Modbus_App_Read_D_Inputs_Check (void)
{
  Modbus_App_Adress=Modbus_App_Msg[1]<<8|Modbus_App_Msg[2];
  Modbus_App_Quantity=Modbus_App_Msg[3]<<8|Modbus_App_Msg[4];
  
  if(Modbus_App_Quantity>2000  || Modbus_App_Quantity==0 || Modbus_App_L_Msg!=5)
    return 3;
  if( ((long)Modbus_App_Adress+(long)Modbus_App_Quantity)>Modbus_App_N_D_Inputs)
    return 2;
    
  return 0; 
}

/**
*   @brief Data check of Read Holding Registers request.
*
*   It stores in _Modbus_App_Adress_ the initial address and in _Modbus_App_Quantity_ the amount of coils to be read;
*   After that, it checks if data meets the specifics; if so, 0 is returned, if not, the error type is returned.
*   @return 0 Correct Data
*   @return 2 I/O requested not available
*   @return 3 Function data error
*   @sa Modbus_App_Msg, Modbus_App_L_Msg, Modbus_App_N_H_Registers 
*   @sa Modbus_App_Adress, Modbus_App_Quantity, Modbus_App_Read_H_Registers
*/
static unsigned char Modbus_App_Read_H_Registers_Check (void)
{
  Modbus_App_Adress=Modbus_App_Msg[1]<<8|Modbus_App_Msg[2];
  Modbus_App_Quantity=Modbus_App_Msg[3]<<8|Modbus_App_Msg[4];
  
  if(Modbus_App_Quantity>125  || Modbus_App_Quantity==0 || Modbus_App_L_Msg!=5)
    return 3;
  if( ((long)Modbus_App_Adress+(long)Modbus_App_Quantity)>Modbus_App_N_H_Registers)
    return 2;
    
  return 0; 
}

/**   
*   @brief Data check of Read Input Registers request.
*
*   It stores in _Modbus_App_Adress_ the initial address and in _Modbus_App_Quantity_ the amount of coils to be read;
*   After that, it checks if data meets the specifics; if so, 0 is returned, if not, the error type is returned.
*   @return 0 Correct Data
*   @return 2 I/O requested not available
*   @return 3 Function data error
*   @sa Modbus_App_Msg, Modbus_App_L_Msg, Modbus_App_N_I_Registers 
*   @sa Modbus_App_Adress, Modbus_App_Quantity, Modbus_App_Read_I_Registers
*/
static unsigned char Modbus_App_Read_I_Registers_Check (void)
{
  Modbus_App_Adress=Modbus_App_Msg[1]<<8|Modbus_App_Msg[2];
  Modbus_App_Quantity=Modbus_App_Msg[3]<<8|Modbus_App_Msg[4];
  
  if(Modbus_App_Quantity>125  || Modbus_App_Quantity==0 || Modbus_App_L_Msg!=5)
    return 3;
  if( ((long)Modbus_App_Adress+(long)Modbus_App_Quantity)>Modbus_App_N_I_Registers)
    return 2;
    
  return 0; 
}

/** 
*   @brief Data check of Write Single Coil request.
*
*   It is stored in _Modbus_App_Adress_ the address; in _Modbus_App_Value_ the value to be written(0xFF = 1; 0x00 = 0); after that,
*   it checks if data meets the specifics; if so, 0 is returned, if not, the error type is returned.
*   @return 0 Correct Data
*   @return 2 I/O requested not available
*   @return 3 Function data error
*   @sa Modbus_App_Msg, Modbus_App_L_Msg, Modbus_App_N_Coils 
*   @sa Modbus_App_Adress, Modbus_App_Value, Modbus_App_Write_Coil
*/
static unsigned char Modbus_App_Write_Coil_Check (void)
{
  Modbus_App_Adress=Modbus_App_Msg[1]<<8|Modbus_App_Msg[2];
  Modbus_App_Value=Modbus_App_Msg[3]<<8|Modbus_App_Msg[4];
  
  if(Modbus_App_Value!=65280 && Modbus_App_Value!=0 || Modbus_App_L_Msg!=5)
    return 3;
  if(Modbus_App_Adress >= Modbus_App_N_Coils)
    return 2;
    
  return 0; 
}

/**
*   @brief Data check of Write Single Register request.
*
*   It is stored in _Modbus_App_Adress_ the address; in _Modbus_App_Value_ the value to be written; after that,
*   it checks if data meets the specifics; if so, 0 is returned, if not, the error type is returned.
*   @return 0 Correct Data
*   @return 2 I/O requested not available
*   @return 3 Function data error
*   @sa Modbus_App_Msg, Modbus_App_L_Msg, Modbus_App_N_H_Registers 
*   @sa Modbus_App_Adress, Modbus_App_Value, Modbus_App_Write_Register
*/
static unsigned char Modbus_App_Write_Register_Check (void)
{
  Modbus_App_Adress=Modbus_App_Msg[1]<<8|Modbus_App_Msg[2];
  Modbus_App_Value=Modbus_App_Msg[3]<<8|Modbus_App_Msg[4];
  
  if(Modbus_App_L_Msg!=5)
    return 3;
  if(Modbus_App_Adress >= Modbus_App_N_H_Registers)
    return 2;
    
  return 0; 
}

/**   
*   @brief Data check of Write Multiple Coils request.
*
*   It stores in _Modbus_App_Adress_ the initial address, in _Modbus_App_Quantity_ the amount of Coils to be written and 
*   the number of bits in _Modbus_App_Value_; after that, it checks if data meets the specifics; if so, 0 is returned, if not, 
*   the error type is returned.
*   @return 0 Correct Data
*   @return 2 I/O requested not available
*   @return 3 Function data error
*   @sa Modbus_App_Msg, Modbus_App_L_Msg, Modbus_App_N_Coils, Modbus_App_Value
*   @sa Modbus_App_Adress, Modbus_App_Quantity, Modbus_App_Write_M_Coils
*/
static unsigned char Modbus_App_Write_M_Coils_Check (void)
{ 
  unsigned char N_Bytes;
  
  Modbus_App_Adress=Modbus_App_Msg[1]<<8|Modbus_App_Msg[2];
  Modbus_App_Quantity=Modbus_App_Msg[3]<<8|Modbus_App_Msg[4];
  Modbus_App_Value=Modbus_App_Msg[5];
  
  // Si el módulo no es 0 significa que ésa cantidad menor que 8 de bits irá en
  // el Byte siguiente, por eso el "+1"
  if(Modbus_App_Quantity%8==0)
    N_Bytes=Modbus_App_Quantity/8;
  else
    N_Bytes=(Modbus_App_Quantity/8)+1;
  
  if(Modbus_App_Quantity>1968  || Modbus_App_Quantity==0 
     || N_Bytes!=Modbus_App_Value || Modbus_App_L_Msg!=(6+N_Bytes))
    return 3;
  if( ((long)Modbus_App_Adress+(long)Modbus_App_Quantity)>Modbus_App_N_Coils)
    return 2;
  
  return 0; 
}

/**
*   @brief Data check of Write Multiple Registers request.
*
*   It stores in _Modbus_App_Adress_ the initial address, in _Modbus_App_Quantity_ the amount of Registers to be written and 
*   the number of bits in _Modbus_App_Value_; after that, it checks if data meets the specifics; if so, 0 is returned, if not, 
*   the error type is returned.
*   @return 0 Correct Data
*   @return 2 I/O requested not available
*   @return 3 Function data error
*   @sa Modbus_App_Msg, Modbus_App_L_Msg, Modbus_App_N_H_Registers, Modbus_App_Value
*   @sa Modbus_App_Adress, Modbus_App_Quantity, Modbus_App_Write_M_Coils
*/
static unsigned char Modbus_App_Write_M_Registers_Check (void)
{ 
  Modbus_App_Adress=Modbus_App_Msg[1]<<8|Modbus_App_Msg[2];
  Modbus_App_Quantity=Modbus_App_Msg[3]<<8|Modbus_App_Msg[4];
  Modbus_App_Value=Modbus_App_Msg[5];
  
  // La cantidad se multiplica por 2 puesto que los registros son de 2 Bytes.
  if(Modbus_App_Quantity>123  || Modbus_App_Quantity==0 || 
     Modbus_App_Quantity*2!=Modbus_App_Value || Modbus_App_L_Msg!=(6+Modbus_App_Value))
    return 3;
  if( ((long)Modbus_App_Adress+(long)Modbus_App_Quantity)>Modbus_App_N_H_Registers)
    return 2;
  
  return 0;
}

/**
*   @brief Data check of Mask Write Register request.
*
*   It stores in _Modbus_App_Adress_ the concrete address; after that, it checks if data meets the specifics; if so, 0 is returned, 
*   if not, the error type is returned. Masks do not create any kind of error, so they are not checked.
*   @return 0 Correct Data
*   @return 2 I/O requested not available
*   @return 3 Function data error
*   @sa Modbus_App_Msg, Modbus_App_L_Msg, Modbus_App_N_H_Registers,
*   @sa Modbus_App_Adress, Modbus_App_Mask_Write_Register
*/
static unsigned char Modbus_App_Mask_Write_Register_Check (void)
{ 
  Modbus_App_Adress=Modbus_App_Msg[1]<<8|Modbus_App_Msg[2];
  
  if(Modbus_App_L_Msg!=7)
    return 3;
  if(Modbus_App_Adress>=Modbus_App_N_H_Registers)
    return 2;
  
  return 0;
}

/*
*   @brief Check data of Read/Write Multiple Registers request.
*
*   It checks Read data in similar way than _Modbus_App_Read_H_Registers()_ and Write data in similar way than
*   _Modbus_App_Write_M_Registers()_; _Modbus_App_Adress_, _Modbus_App_Quantity_ and _Modbus_App_Value_ will have
*   data regarding the write request as it will be done firstly.
*   @return 0 Correct Data
*   @return 2 I/O requested not available
*   @return 3 Function data error
*   @sa Modbus_App_Msg, Modbus_App_L_Msg, Modbus_App_N_H_Registers, Modbus_App_Quantity
*   @sa Modbus_App_Adress, Modbus_App_Value, Modbus_App_Read_Write_M_Registers
*/
static unsigned char Modbus_App_Read_Write_M_Registers_Check (void)
{ 
  Modbus_App_Adress=Modbus_App_Msg[1]<<8|Modbus_App_Msg[2];
  Modbus_App_Quantity=Modbus_App_Msg[3]<<8|Modbus_App_Msg[4];
  
  if(Modbus_App_Quantity>125  || Modbus_App_Quantity==0 || 
     Modbus_App_L_Msg!=10+Modbus_App_Msg[9])
    return 3;
  if( ((long)Modbus_App_Adress+(long)Modbus_App_Quantity)>Modbus_App_N_H_Registers)
    return 2;
  
  Modbus_App_Adress=Modbus_App_Msg[5]<<8|Modbus_App_Msg[6];
  Modbus_App_Quantity=Modbus_App_Msg[7]<<8|Modbus_App_Msg[8];
  Modbus_App_Value=Modbus_App_Msg[9];
  
  if(Modbus_App_Quantity>123  || Modbus_App_Quantity==0 || 
     Modbus_App_Quantity*2!=Modbus_App_Value)
    return 3;
  if( ((long)Modbus_App_Adress+(long)Modbus_App_Quantity)>Modbus_App_N_H_Registers)
    return 2;
  
  return 0;
}
/** @} */

/**
*   @defgroup App_Process Process Functions
*   @ingroup App_Modbus
*   @brief Process the requested action and the response is built.
*   @{ 
*/

/** 
*   @brief Read the Coils and the values are wrapped in the response.
*
*   In _Modbus_App_Response_pdu_ is built the response PDU with the data of the auxiliary variables and the coils values. 
*   Coils are wrapped in chunks of 8 coils/byte following the Modbus specifics.
*   @sa Modbus_App_Response_pdu, Modbus_App_L_Response_pdu, Modbus_App_Coils 
*   @sa Modbus_App_Adress, Modbus_App_Quantity, Modbus_App_Read_Coils_Check
*/
static void Modbus_App_Read_Coils(void)
{
  unsigned char i,k;
  uint16_t j=0; /*EL CHICO PUSO UNSIGNED CHAR y ESTÁ MAL*/
  Modbus_App_Response_pdu[0]=1;
  
  if(Modbus_App_Quantity%8==0)
    Modbus_App_Response_pdu[1]=Modbus_App_Quantity/8;
  else
    Modbus_App_Response_pdu[1]=(Modbus_App_Quantity/8)+1;
  
  // "k+2" marca la posición en el vector, "j" el índice en el vector de lectura y
  // limita cuando se llega a total de Coils a leer. "i" desplaza el bit a la
  // posición dentro del Byte de respuesta.
  for(k=0;j<Modbus_App_Quantity;k++)
  {
    Modbus_App_Response_pdu[2+k]=0;
    for(i=0;i<8 && j<Modbus_App_Quantity;i++)
      Modbus_App_Response_pdu[2+k]=Modbus_App_Response_pdu[2+k] | 
      Modbus_App_Coils[Modbus_App_Adress + j++]<<i;
  }
  
  Modbus_App_L_Response_pdu=2+Modbus_App_Response_pdu[1];
}

/**
*   @brief Read the Discrete Inputs and the values are wrapped in the response.
*
*   In _Modbus_App_Response_pdu_ is built the response PDU with the data of the auxiliary variables and the Discrete Inputs values. 
*   Values are wrapped in chunks of 8 values/byte following the Modbus specifics.
*   @sa Modbus_App_Response_pdu, Modbus_App_L_Response_pdu, Modbus_App_D_Inputs
*   @sa Modbus_App_Adress, Modbus_App_Quantity, Modbus_App_Read_D_Inputs_Check
*/
static void Modbus_App_Read_D_Inputs (void)
{
  unsigned char i, k;
  uint16_t j=0; /*EL CHICO PUSO UNSIGNED CHAR y ESTÁ MAL*/
  Modbus_App_Response_pdu[0]=2;
  
  if(Modbus_App_Quantity%8==0)
    Modbus_App_Response_pdu[1]=Modbus_App_Quantity/8;
  else
    Modbus_App_Response_pdu[1]=(Modbus_App_Quantity/8)+1;

  // "k+2" marca la posición en el vector, "j" el índice en el vector de lectura y
  // limita cuando se llega a total de Entradas a leer. "i" desplaza el bit a la
  // posición dentro del Byte de respuesta.  
  for(k=0;j<Modbus_App_Quantity;k++)
  {
    Modbus_App_Response_pdu[2+k]=0;
    for(i=0;i<8 && j<Modbus_App_Quantity;i++)
      Modbus_App_Response_pdu[2+k]=Modbus_App_Response_pdu[2+k] | 
      Modbus_App_D_Inputs[Modbus_App_Adress + j++]<<i;
  }
  
  Modbus_App_L_Response_pdu=2+Modbus_App_Response_pdu[1];
}

/** 
*   @brief Read the Holding Registers and the values are wrapped in the response.
*
*   In _Modbus_App_Response_pdu_ is built the response PDU with the data of the auxiliary variables and the Holding Registers values.
*   @sa Modbus_App_Response_pdu, Modbus_App_L_Response_pdu, Modbus_App_H_Registers
*   @sa Modbus_App_Adress, Modbus_App_Quantity, Modbus_App_Read_H_Registers_Check
*/
static void Modbus_App_Read_H_Registers (void)
{
  unsigned char i;
  
  Modbus_App_Response_pdu[0]=3;  
  Modbus_App_Response_pdu[1]=Modbus_App_Quantity*2;
  
  for(i=0;i<Modbus_App_Quantity;i++)
  {
    Modbus_App_Response_pdu[2+2*i]=Modbus_App_H_Registers[Modbus_App_Adress+i]>>8;
    Modbus_App_Response_pdu[3+2*i]=Modbus_App_H_Registers[Modbus_App_Adress+i];
  }
  
  Modbus_App_L_Response_pdu=2+Modbus_App_Response_pdu[1];
}

/*   @brief Read the Input Registers and the values are wrapped in the response.
*   
*   In _Modbus_App_Response_pdu_ is built the response PDU with the data of the auxiliary variables and the Input Registers values.
*   @sa Modbus_App_Response_pdu, Modbus_App_L_Response_pdu, Modbus_App_I_Registers
*   @sa Modbus_App_Adress, Modbus_App_Quantity, Modbus_App_Read_I_Registers_Check
*/
static void Modbus_App_Read_I_Registers (void)
{
  unsigned char i;
  
  Modbus_App_Response_pdu[0]=4;  
  Modbus_App_Response_pdu[1]=Modbus_App_Quantity*2;
  
  for(i=0;i<Modbus_App_Quantity;i++)
  {
    Modbus_App_Response_pdu[2+2*i]=Modbus_App_I_Registers[Modbus_App_Adress+i]>>8;
    Modbus_App_Response_pdu[3+2*i]=Modbus_App_I_Registers[Modbus_App_Adress+i];
  }
  
  Modbus_App_L_Response_pdu=2+Modbus_App_Response_pdu[1];
}

/**
*   @brief The proper value is written and it answers with an request echo.
*
*   @sa Modbus_App_Response_pdu, Modbus_App_L_Response_pdu, Modbus_App_Coils
*   @sa Modbus_App_Adress, Modbus_App_Value, Modbus_App_Write_Coil_Check
*/
static void Modbus_App_Write_Coil (void)
{
  Modbus_App_Response_pdu[0]=5;  
  Modbus_App_Response_pdu[1]=Modbus_App_Adress>>8;
  Modbus_App_Response_pdu[2]=Modbus_App_Adress;
  
  if(Modbus_App_Value==65280)
  {
    Modbus_App_Coils[Modbus_App_Adress]=1;
    Modbus_App_Response_pdu[3]=255;
    Modbus_App_Response_pdu[4]=0;
  }
  if(Modbus_App_Value==0)
  {
    Modbus_App_Coils[Modbus_App_Adress]=0;
    Modbus_App_Response_pdu[3]=0;
    Modbus_App_Response_pdu[4]=0;
  } 
  
  Modbus_App_L_Response_pdu=5;
}

/**
*   @brief The proper value is written and it answers with an echo of the request.
*
*   @sa Modbus_App_Response_pdu, Modbus_App_L_Response_pdu, Modbus_App_H_Registers
*   @sa Modbus_App_Adress, Modbus_App_Value, Modbus_App_Write_Register_Check
*/
static void Modbus_App_Write_Register (void)
{
  Modbus_App_Response_pdu[0]=6;  
  Modbus_App_Response_pdu[1]=Modbus_App_Adress>>8;
  Modbus_App_Response_pdu[2]=Modbus_App_Adress;
  Modbus_App_Response_pdu[3]=Modbus_App_Value>>8;
  Modbus_App_Response_pdu[4]=Modbus_App_Value; 
  
  Modbus_App_H_Registers[Modbus_App_Adress]=Modbus_App_Value;
  Modbus_App_L_Response_pdu=5;
}

/**
*   @brief Coils are unwrapped and written; after that it answers.
*
*   Bytes with the bits of each Coil are wrapped and written in the proper address, after that it answers with the first five bytes 
*   of the request.
*   @sa Modbus_App_Response_pdu, Modbus_App_L_Response_pdu, Modbus_App_Coils
*   @sa Modbus_App_Adress, Modbus_App_Quantity, Modbus_App_Value,
*   @sa Modbus_App_Write_M_Coils_Check
*/
static void Modbus_App_Write_M_Coils (void)
{
  unsigned char i,j;
  uint16_t k=0;  /*EL CHICO PUSO UNSIGNED CHAR y ESTÁ MAL*/
  
  Modbus_App_Response_pdu[0]=15;  
  Modbus_App_Response_pdu[1]=Modbus_App_Adress>>8;
  Modbus_App_Response_pdu[2]=Modbus_App_Adress;
  Modbus_App_Response_pdu[3]=Modbus_App_Quantity>>8;
  Modbus_App_Response_pdu[4]=Modbus_App_Quantity; 

  // "6+i" marca la posición en la petición, "k+Adress" el índice donde escribir 
  //  "k" limita cuando se llega a total de Coils a escribir. 
  // "j" desplaza el bit a la primera posición para que "& 1" elimine los otros y
  // lo deje preparado para su escritura.
  for(i=0;i<Modbus_App_Value;i++)
    for(j=0;j<8 && k<Modbus_App_Quantity;j++)
    {
      Modbus_App_Coils[k+Modbus_App_Adress]=(Modbus_App_Msg[6+i]>>j) & 1;
      k++;
    }
  
  Modbus_App_L_Response_pdu=5;
}

/**
*   @brief Registers are written and it answers with the first five bytes of the request.
*
*   @sa Modbus_App_Response_pdu, Modbus_App_L_Response_pdu, Modbus_App_H_Registers
*   @sa Modbus_App_Adress, Modbus_App_Quantity, Modbus_App_Write_M_Registers_Check
*/
static void Modbus_App_Write_M_Registers (void)
{ 
  unsigned char i;
  
  Modbus_App_Response_pdu[0]=16;  
  Modbus_App_Response_pdu[1]=Modbus_App_Adress>>8;
  Modbus_App_Response_pdu[2]=Modbus_App_Adress;
  Modbus_App_Response_pdu[3]=Modbus_App_Quantity>>8;
  Modbus_App_Response_pdu[4]=Modbus_App_Quantity; 
  
  for(i=0;i<Modbus_App_Quantity;i++)
    Modbus_App_H_Registers[Modbus_App_Adress+i]=Modbus_App_Msg[6+2*i] |
    Modbus_App_Msg[7+2*i];
  
  Modbus_App_L_Response_pdu=5;
}

/**
*   @brief It modifies the Register using masks and it answers with an echo of the request.
*
*   It reads the masks of the request and it applies them in the proper Register following the method:
*   Value = (Register Value AND AND_Mask) OR (OR_Mask AND (NOT AND_Mask))
*   In this way the mask OR only can affect to the values that AND Mask set to 0.
*   @sa Modbus_App_Response_pdu, Modbus_App_L_Response_pdu, Modbus_App_H_Registers
*   @sa Modbus_App_Adress, Modbus_App_Mask_Write_Register_Check
*/
static void Modbus_App_Mask_Write_Register (void)
{ 
  uint16_t AND_Mask,OR_Mask;
  
  Modbus_App_Response_pdu[0]=22;  
  Modbus_App_Response_pdu[1]=Modbus_App_Adress>>8;
  Modbus_App_Response_pdu[2]=Modbus_App_Adress;
  Modbus_App_Response_pdu[3]=Modbus_App_Msg[3];
  Modbus_App_Response_pdu[4]=Modbus_App_Msg[4];
  Modbus_App_Response_pdu[5]=Modbus_App_Msg[5];
  Modbus_App_Response_pdu[6]=Modbus_App_Msg[6];
  
  AND_Mask=Modbus_App_Msg[3]<<8|Modbus_App_Msg[4];
  OR_Mask= Modbus_App_Msg[5]<<8|Modbus_App_Msg[6];    
  Modbus_App_H_Registers[Modbus_App_Adress]=(Modbus_App_H_Registers[Modbus_App_Adress] & AND_Mask)
    | (OR_Mask & ~AND_Mask);
  
  Modbus_App_L_Response_pdu=7;
}

/**
*   @brief It Reads/Writes some Registers and it answers.
*
*   Firstly, it does the Writes in the same way than its similar function; after that, it reads the registers in the same way than its
*   similar function (these registers can be different to the Writes ones). It answers as the Read function but with a different function
*   number.
*   @sa Modbus_App_Response_pdu, Modbus_App_L_Response_pdu, Modbus_App_H_Registers
*   @sa Modbus_App_Adress, Modbus_App_Quantity, Modbus_App_Value,
*   @sa Modbus_App_Write_M_Coils, Modbus_App_Read_H_Registers
*   @sa Modbus_App_Read_Write_M_Registers_Check
*/
static void Modbus_App_Read_Write_M_Registers (void)
{
  unsigned char i;
  
  // Escribir valores
  for(i=0;i<Modbus_App_Quantity;i++)
  Modbus_App_H_Registers[Modbus_App_Adress+i]=Modbus_App_Msg[10+2*i] |
  Modbus_App_Msg[11+2*i];
  
  // Leer valores 
  Modbus_App_Adress=Modbus_App_Msg[1]<<8|Modbus_App_Msg[2];
  Modbus_App_Quantity=Modbus_App_Msg[3]<<8|Modbus_App_Msg[4];
  
  Modbus_App_Response_pdu[0]=23;  
  Modbus_App_Response_pdu[1]=Modbus_App_Quantity*2;
  
  for(i=0;i<Modbus_App_Quantity;i++)
  {
    Modbus_App_Response_pdu[2+2*i]=Modbus_App_H_Registers[Modbus_App_Adress+i]>>8;
    Modbus_App_Response_pdu[3+2*i]=Modbus_App_H_Registers[Modbus_App_Adress+i];
  }
  
  Modbus_App_L_Response_pdu=2+Modbus_App_Response_pdu[1];
}
/** @} */