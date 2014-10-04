// Author: Francisco Javier Guzman Jimenez, <dejavits@gmail.com>
#ifdef CAN_Mode
#ifndef MODBUS_CAN_H_
#define MODBUS_CAN_H_

/**
*   @defgroup CAN Modbus CAN
*   @brief CAN Module for Modbus.
*
*   @author Francisco Javier Guzman Jimenez, <dejavits@gmail.com>
*
*   This module is used to make a proper Modbus communication using a <b> control area network</b>(CAN), as physical layer. For such purpose,
*   it is implemented the elements and functions to establish a communication from master to one or more slaves, and vice versa.
*
*   To achieve this goal, it was used the libraries implemented by Texas Instrument, which the most important in this case is the CAN library.
*   In the proper library is implemented the basic functions to make CAN work in a "lower level", as initialisation of CAN, message objects 
*   setting up, data transfer and so on.
*
*   To understand the code is important to first understand CAN. In this case, the CAN module is compounded by 32 message object, which 
*   their function is to send or receive data according to some parameters, like some kind of mailbox. Moreover, each message sent has 
*   an ID which will be taken into account to accept the receive message in the reception, that is done by using a filter mask. So, when
*   a message is needed to be put in a message object(a message to send or a message to receive), it will be only placed into the message
*   object only if it passes the filter. In C, that function would be represented approximately by the following sentence:
*   
*   if (((OutMessage/InMessage) & MessageObjectMask) == MessageObjectID){ Message accepted and placed into the message object}
*   
*   In addition, CAN is able to use messageIDs of 11 or 29 bits. As Modbus is made to only address 247 slaves, it is only necessary 8 bits 
*   for the ID. Furthermore, to make a proper filtering it is needed to make a difference between request messages and responses messages, 
*   for that, it is used 1 bit, which will be called <b>request/answer bit</b>. Last but not least, CAN sends message of maximum 64 bits, 
*   then, if it is neeeded to send more data, it has to be splitted in chunks. Therefore, we have the next four types of messages (they 
*   will be explained later more deeply in the appropiate functions):
*
*       -00: Individual frame (data to be sent less than 64 bits)
*       -01: Beginning long frame (first chunk of data more than 64 bits)
*       -10: Continuation long frame
*       -11: End long frame (last chunk of data)
*  
*   As conclusion, 11-bits message IDs fix perfectly with our purpose and is enough. In any case, CAN is able to send other types of
*   messages as remote frames, so, in the future, if it is needed to make more differences, it can be used the 29-bits message IDs.
*   
*   The elements and functions that are explained in this module, instead of the module CAN Master or CAN Slave, are common between
*   the master and slaves, therefore, it is not needed to make a distinction and are used in the same way in both parts.
*   The functions in this module are the common ones in the master and in the slave.
*/
/** @{ */
#include "stdint.h"

//! As a board can have different CAN modules, we specify which one we want.
#define MODBUS_CAN CAN0_BASE
//! Following the Modbus specifications the maximum of PDU should be 256.
#define MAX_PDU 256
//! CAN only can send chunks of 8 bytes.
#define MAX_FRAME 8

//!Possible bit rate ranges implemented
enum Modbus_CAN_BitRate
{
      MODBUS_100KBPS,   //! Bit rate of 100 Kbps
      MODBUS_1MBPS      //! Bit rate of 1 Mbps
};
/** @} */
//////////////////////////////////////////////////MASTER/////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////MASTER/////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////MASTER/////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////MASTER/////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////MASTER/////////////////////////////////////////////////////////////////
//*****************************************************************************
#if MODBUS_MASTER
#undef MODBUS_SLAVE
/** 
*   @defgroup Master_CAN CAN Master elements.
*   @ingroup CAN
*   @brief Master elements and functions.
*   @author Francisco Javier Guzman Jimenez, <dejavits@gmail.com>
*   
*   These are the elements which compound the master node and are different in the slave nodes. The first difference is that messages sent 
*   by the master will have the <b>request/answer bit set to 1</b>, for indicating a "request" and the message received will be set to 0 
*   indicating a response. That is because of the Modbus spefication.
*
*   The second important difference is that master uses timeouts. In broadcast requests, it is used to stall until is assumed that all slaves
*   received and processed the request. In unicast requests, it is used to wait for an answer, if that does not arrive, it is assumed that 
*   something was wrong and master will try to send again the request. The values of the timeouts will be explained deeply in the proper 
*   functions.
* 
*   Additionaly, to understand the code, it has to be kept in my mind that master works following the next behaviour according to the Modbus 
*   specifications:
*
*   @image html images/masterstates.jpg
*   @image latex images/masterstates.eps "Master Diagram" width=10cm
*/
//*****************************************************************************
/** @{ */

//! This are the possible master states according to the Modbus specifications.
enum Modbus_MainState
{
    MODBUS_INITIAL,        //!< Init state
    MODBUS_IDLE,           //!< Idle state
    MODBUS_WAITREPLY,      //!< Waiting for an unicast answer
    MODBUS_TURNAROUND,     //!< Waiting for broascast answers
    MODBUS_PROCESSING,     //!< Reply process state
    MODBUS_ERROR           //!< Error process state
};

/////////////////////////////////////////////MASTER PROTOTYPES//////////////////////////////////////////////
/**
*    @brief CAN Initialisation function.
*
*    This is the function to initialise the CAN module. The system and some variables used for Modbus are also initialised.
*    The message object 1 would be set up for transfers in the function made to send data.
*    The message object 17 will be put as receive message object for unicast requests.
*    CAN message's IDs(11-bit) in Modbus will be compound by a header(3 bits) and a slave number (8 bits)
*    Modbus header frames in CAN as was previously mentioned are as follows:
*
*               -00: Individual Frame
*               -01: Beginning Long Frame
*               -10: Continuation Long Frame
*               -11: End Long Frame
*
*    Last bit of the header is a request(1)/ answer(0) bit;
*    @param bit_rate Indicate the bit rate to be used in the communications.
*    @param attempts Maximum number of sending attempts.
*    @note It is assumed that number of attempts is at least 1.
*    @sa SysCtlPeripheralEnable, IntMasterEnable, TimerConfigure,IntEnable,TimerIntEnable, GPIOPinTypeGPIOOutput 
*    @sa GPIOPinTypeCAN, CANInit, CANSetBitTiming, CANEnable, CANIntEnable, Modbus_SetMainState, ledOff, ledOn
*/
void Modbus_CAN_Init(enum Modbus_CAN_BitRate bit_rate, unsigned char attempts);

/**
*     @brief Function to send information.
*
*     This function is called when the master wants to send data. The messages in CAN are built up as 8 bytes as maximum, so, this function
*     will process all data sending chunks of 8 bytes. That makes to have some kind of control to know which chunk is expected, therefore,
*     there are 4 types of messages with the following different headers in the message ID, in this way it can be known when the big messages 
*     start and when they finish:
*
*               -001: Individual Frame + request bit
*               -011: Beginning Long Frame + request bit
*               -101: Continuation Long Frame + request bit
*               -111: End Long Frame + request bit
*
*     The rest of the message ID will be the slave number. 
*     When the function splits up all the data and sends the last chunk, the timer is set up to be triggered if a complete reception does 
*     not arrive.
*     @param mb_req_pdu The information to be sent.
*     @param slave The number of the slave who will receive the data.
*     @param pdu_length The amount of data to be sent.
*     @param amount_guess A guess of the amount of data (in bytes) that will pass through the bus.
*     @note Slave number is supposed to be right.
*     @sa CANMessageSet, Modbus_CAN_ReceptionConfiguration, Modbus_CAN_Delay, Modbus_SetMainState, Modbus_CAN_UnicastTimeout, Modbus_CAN_BroadcastTimeout
*/
void Modbus_CAN_FixOutput(unsigned char *mb_req_pdu, unsigned char slave, unsigned char pdu_length, uint16_t amount_guess);

/**
*     @brief Function to configure the message object to receive data.
* 
*     This function is called when the master wants to send data to a particular slave and receive an answer. In such way, it is configured 
*     the receive message object (num. 17). It is set up with the ID equal to the number of the slave, because it is expected the message 
*     from the slave we send data. The mask is put to receive all messages from that slave which are responses (request bit=0), independently
*     of the other two bits of the header, which indicates the type of frame as was explained previously.
*     @param slave The number of the slave from whom it will be received the data.
*     @note Slave number is supposed to be right.
*     @sa CANMessageSet, Modbus_CAN_FixOutput
*/
void Modbus_CAN_ReceptionConfiguration(unsigned char slave);

/** 
*    @brief Function to acknowledge if a forward is needed.
*
*    This function is called to know if a resend should be done. After get the proper value, the forward flag is cleared.
*    A resend has to be done if an unicast timeout was triggered with a reception not completed.
*    @note A resend is always possible as much as the maximum number of forward attempts is not achieved.
*    @return <b>0</b> If it is not needed a forward, or <b>1</b> if it is needed.
*/
unsigned char Modbus_CAN_GetForwardFlag(void);

/**
*   @brief Function to reset the number of sending attempts.
*
*   This function is called when there is a completely new transmission. So, the actual number of sending attempts has to be
*   reset.
*/
void Modbus_CAN_Reset_Attempt(void);

/**
*   @brief Function to repeat a request.
* 
*   This function checks if it is possible to send a request again, probably because an error.
*   If it is possible, _modbus_forward_flag_ is marked, if it is not possible because
*   it was already achieved the maximum number of attempts, then it is notified to APP layer to discard this request.
*   @sa Modbus_App_No_Response
*/
void Modbus_CAN_Repeat_Request(void);

/**
*   @brief This function is used to change the master state in case a timeout shows up.
* 
*   This function is called when one timeout is trigered. If the timeout was the unicast timeout,
*   then, it is marked as _MODBUS_ERROR_, to lately, try to resend the request. If it was the broadcast timeout, it is assumed all slaves
*   received the broadcast message, then is passed to _MODBUS_IDLE_ to deal with the next request.
*   @sa Modbus_GetMainState, Modbus_SetMainState, Modbus_CAN_Error_Management
*/
void Modbus_CAN_Timeouts(void);

/**
*    @brief Function to handle the unicast timeout interruption.
* 
*    This function is triggered to handle the timeout interruption when the timer
*    was configured with the unicast time value. If the completely reception was not processed, then
*    it is called to _Modbus_CAN_Timeouts_. If the reception was processed nothing happens.
*    @sa TimerIntClear, Modbus_CAN_Timeouts
*/
void Modbus_CAN_UnicastTimeoutHandler(void);

/**
*    @brief Function to configure the unicast timeout value.
*
*    This function is called when an unicast request has to be made; Timer is set up with the value indicated in _modbus_unicast_timeout_, 
*    which will depend on the CAN bit rate range chosen and a guess of the amount data which will pass through the bus in both the request
*    as in the answer.
*    The unicast timeout value is compounded by:
*
*              -(amount_guess * 933333) = Transfer time; It represents how much time is needed to transfer _amount_guess_ bytes through a bus working at
*                      1Mbps. In addition, if it is working at 1 Kbps, for example, transfer time will be 10 times higher.
*              -(900000 * amount_guess * 4) = Process time; It represents how much time is necessary to process _amount_guess_ bytes.
*              -((modbus_attempts-1) * 8000) = Congestion avoidance; It simulates a congestion avoidance regulator, as more attempts are done, 
*                      more the master will wait.
*
*    @param amount_guess A guess of the amount data that will pass through the bus in this transfer.
*    @warning Timeout value is not needed to be as high as it is right now, but as it is used serial port and a terminal for debugging,
*    then, value has to be that high. It is more than known that showing stuff on screen is slower than CPU.
*    @sa TimerLoadSet, TimerEnable, Modbus_SetBitRate
*/
void Modbus_CAN_UnicastTimeout(uint16_t amount_guess);

/**
*       @brief Function to handle the broadcast timeout interruption.
*
*       This function is triggered to handle the timeout interruption when the timer
*       was configured with the broadcast time value. As it is not expected any answer when is used a broadcast,
*       then it is called directly to Modbus_CAN_Timeouts().
*       @sa TimerIntClear, Modbus_CAN_Timeouts
*/
void Modbus_CAN_BroadcastTimeoutHandler(void);

/** 
*       @brief Function to configure the broadcast timeout value.
*
*       This function is called when a broadcast sending has to be made; Then the timer is set up with the value indicated in
*       _modbus_broadcast_timeout_, which will depend on the CAN bit rate range chosen and a guess of the amount data which will 
*       pass through the bus in the request.
*       The broadcast timeout is compounded by:
*
*               -(amount_guess * 933333) = Transfer time; It represents how much time is needed to transfer _amount_guess_ bytes through a 
*                       bus working at 1Mbps. In addition, if it is working at 1 Kbps, for example, transfer time will be 10 times higher.
*               -(900000 * amount_guess * 4) = Process time; It represents how much time is necessary to process _amount_guess_ bytes.
*
*       @note The broadcast timeout is multiplied by two to be sure that data is able to stay in the bus enough time to be listened by
*       all slaves, and also, to wait slaves to process the request.
*       @param amount_guess A guess of the amount data that will pass through the bus in this transfer.
*       @sa TimerLoadSet, TimerEnable, Modbus_SetBitRate
*/
void Modbus_CAN_BroadcastTimeout(uint16_t amount_guess);

/**
*       @brief Function to disable the unicast timeout.
*
*       This function is called to disable the unicast timeout when a complete answer was received.
*       @sa TimerDisable
*/
void Modbus_CAN_RemoveTimeout(void);

//DEPRECATED:
unsigned char Debug_Timeout(void);
unsigned char getIndex(void);
unsigned char getbu(void);
uint16_t getboo(void);
unsigned char getAttempts(void);
void setbu(void);
/** @} */
#elif MODBUS_SLAVE
#undef MODBUS_MASTER
//////////////////////////////////////////////////SLAVE/////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////SLAVE/////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////SLAVE/////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////SLAVE/////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////SLAVE/////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////SLAVE/////////////////////////////////////////////////////////////////
//*****************************************************************************
/**
*       @defgroup Slave_CAN CAN Slave elements.
*       @ingroup CAN
*       @brief Slave elements and functions.
*       @author Francisco Javier Guzman Jimenez, <dejavits@gmail.com>
*
*       These are the elements which compound the slave node and are different in the master node. First difference is that 
*       request/answer bit is set to 0, representing an answer. Second difference is that there is not timeouts, as the slaves
*       only wait for requests, process them and if it is needed, they send a response. If such a response does not arrive, the master
*       will send a request.
*       
*      The slaves behave following the next diagram obtained from the Modbus specification:
*      @image html images/slavestates.JPG
*      @image latex images/slavestates.eps "Slave Diagram" width=10cm
*/
//*****************************************************************************
/** @{ */


//! Possible Slave states
enum Modbus_MainState
{
    MODBUS_INITIAL,        //!< Init state
    MODBUS_IDLE,           //!< Idle state
    MODBUS_CHECKING,       //!< Checking request state       
    MODBUS_PROCESSING,     //!< Process action requested
    MODBUS_REPLY,          //!< Process normal reply
    MODBUS_ERROR           //!< Process error reply
};

////////////////////////////////////////////////////////SLAVE PROTOTYPES//////////////////////////////////////////////////

/**
*       @brief CAN Initialisation.
*
*       This is the function to initialise the CAN module. The system and some CAN variables are also initialised.
*       The message object 1 would be set up in the function to send data.
*       The message objects 17 and 18 will be configured as receive message object for unicast and broadcast requests respectively.
*       CAN message IDs(11-bits) in Modbus will be compounded by a header(3 bits) and a slave number (8 bits).
*       Modbus header frames in CAN will be built up by three bits, as was previously mentioned:
*
*               -00: Individual Frame
*               -01: Beginning Long Frame
*               -10: Continuation Long Frame
*               -11: End Long Frame
*
*       Last bit of the header is the request(1)/ answer(0) bit; In this case, a 0.
*       @param bit_rate Indicate the bit rate to be used in the communications.
*       @param slave Slave number.
*       @sa SysCtlPeripheralEnable, GPIOPinTypeCAN, CANInit, CANSetBitTiming, CANIntEnable, Modbus_SetMainState, Modbus_CAN_ReceptionConfiguration
*/
void Modbus_CAN_Init(enum Modbus_CAN_BitRate bit_rate, unsigned char slave);

/**
*       @brief Function to send information.
*
*       This function is called when the slave wants to send data. The messages in CAN are built up as 8 bytes as maximum, so, this function
*       will process all data sending chunks of 8 bytes. That makes to have some kind of control to know which chunk is expected, therefore,
*       there are 4 types of messages with the following different headers in the message ID, in this way it can be known when the big messages
*       start and when they finish.
*
*               -000: Individual Frame + request bit
*               -010: Beginning Long Frame + request bit
*               -100: Continuation Long Frame + request bit
*               -110: End Long Frame + request bit
*
*       The rest of the message's ID will be the slave number itself as the master is waiting frames from this slave.
*       There is no timeout in the slave, if the data does not arrive, the master will send the request again.
*       @param mb_req_pdu The information to be sent.
*       @param pdu_length The amount of data to be sent.
*       @sa CANMessageSet, Modbus_CAN_Delay, Modbus_SetMainState
*/
void Modbus_CAN_FixOutput(unsigned char *mb_req_pdu, unsigned char pdu_length);

/**
*       @brief Function to configure receive message objects.
*
*       This function is called to configure the receive message objects. One is set up as unicast receive message object
*       with the ID equal to the number of the slave (message object 17). The other one is set up as a broadcast receive message object 
*       with the ID equal to 0, which represent a broadcast request (message object 18) according to the Modbus specification.
*       The mask is set to accept all matched messages taking into account the request/answer bit and the 8-bits that represent the slave
*       number.
*       @sa CANMessageSet, Modbus_CAN_ReceptionConfiguration, Modbus_CAN_Delay, Modbus_SetMainState
*/
void Modbus_CAN_ReceptionConfiguration();

/**
*       @brief Function to get the broadcast flag.
*
*       This function return the value of the broadcast flag. Such a flag is activated when a broadcast arrived, it is useful to know it
*       to prepare a reply or not.
*       @return <b>1</b> if the reception is a broadcast request or <b>0</b> if the reception is an unicast request.
*/
unsigned char Modbus_CAN_BroadCast_Get(void);

/** @} */
#endif

//////////////////////////////////////////////////COMMON/////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////COMMON/////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////COMMON/////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////COMMON/////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////COMMON/////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////COMMON/////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////COMMON PROTOTYPES////////////////////////////////////

/**
*       @brief CAN Interrupt Handler.
*       @ingroup CAN
*
*       This is the CAN interrupt handler. First, it checks the CAN status, if something went wrong in the communication as acks, etc.
*       the timeout and resending method will fix that, so they are not taken into account. If the proper CAN node enters into a Bus Off 
*       state, error passive level or warning level, then, it is stopped for security, although in a Bus Off state CAN is disabled automatically.
*       Secondly, it checks if there was a complete transmission. In such a case, it is notified activating the proper flag. It should be 
*       only one notification regarding the transmission. Such an interruption should be configured when the last output chunk is sent.
*       Last thing to check is the reception data, if there was an unicast reception then the incoming data is placed by message object 17. 
*       In the broadcast case, data will be handled by message object 18 in the slave and it will not be handled by the master as this one 
*       does not receive broadcast messages. The incoming data is processed in Modbus_CAN_CallBack().
*       @sa CANIntStatus, CANStatusGet, CANIntClear, Modbus_CAN_CallBack
*/
void Modbus_CAN_IntHandler(void);

/**
*       @brief Function to set up the bit rate and time, and delays.
*       @ingroup CAN
*
*       This function initialise the bit timing parameters depending on the bit time range chosen. 
*       Also, depending on the former, it is configure the delay used between transmission. The value of that delay was chosen in the same
*       way than the timeouts.
*       
*       The bit rate of the bus depends on the values stored in the struct _tCANBitClkParms_, the maximum in CAN is 1MBPS. It is only 
*       implemented 1 KBPS and 1 MBPS. If it is desired others ranges, the proper values from the former struct has to be calculated.
*       @note If the range selected is not implemented, it stalls as it is impossible to establish communication without specifying a range.
*       @sa Modbus_CAN_Error_Management
*/
void Modbus_CAN_SetBitRate(enum Modbus_CAN_BitRate bit_rate);

/**
*       @brief Function to obtain the actual state of the master/slave.
*       @ingroup CAN
*
*       This function is used to get known the status of the master/slave according with
*       the diagrams of the Modbus specification.
*       @return Modbus_MainState The status of the master.
*       @sa Modbus_MainState, Modbus_SetMainState
*/
enum Modbus_MainState Modbus_GetMainState(void);

/**
*       @brief Function to set the state of the master/slave.
*       @ingroup CAN
*
*       This function is used to set the status of the master/slave according with
*       the diagrams of the Modbus specification.
*       @param state The new status of the master/slave.
*       @sa Modbus_MainState, Modbus_GetMainState
*/
void Modbus_SetMainState(enum Modbus_MainState state);

/**
*       @brief Function to process the received information.
*       @ingroup CAN
*
*       This function is called when the master/slave receives data. If there is new data in the specific message object(it should be 
*       if we are in this function), it is checked if data is according to one of the possible header frames.
*       Depending on the received header, it is processed in one way or other. The destinations are always the expected ones
*       because the receive message objects were configured either to receive a concrete slave when the function Modbus_CAN_ReceptionConfiguration()
*       was called in the master, or to receive from the master always in the case of the slave.
*       In the slave, if the reception is a broadcast request it is looked in the message object num.18 instead of the num.17 and it is 
*       raised a flag to notify such a request.
*       @result <b>1</b> If there was a successful complete reception, or <b>0</b> if there was some error.
*       @sa CANMessageGet, CANStatusGet, Modbus_CAN_ReceptionConfiguration, Modbus_SetMainState
*/
void Modbus_CAN_CallBack();

/**
*       @brief Function to manage the master/slave behaviour.
*       @ingroup CAN
*
*       This function is used to handle the behaviour of the master/slave following the diagrams of the Modbus
*       specification. Depending on the status of the master/slave, an action or other will be taken.
*
*       @return <b>Master</b>: <b>0</b> if there are no more communications, or <b>1</b> if there are still pending communications.
*       @return <b>Slave</b>: <b>1</b> if a message was sent to APP layer, or <b>0</b> if not.
*       
*       @sa Modbus_GetMainState, Modbus_CAN_GetForwardFlag, Modbus_App_Send, Modbus_App_FIFOSend
*       @sa Modbus_App_Manage_CallBack, Modbus_CAN_Repeat_Request, Modbus_SetMainState, Modbus_CAN_to_App
*/
unsigned char Modbus_CAN_Controller(void);

/**
*       @brief Function to make a delay.
*       @ingroup CAN
*
*       This function is called between transfers to make a delay between them. In this way,
*       the data will arrive at the destination more slowly and not all at once, avoiding lost data.
*       @sa SysCtlDelay
*/
void Modbus_CAN_Delay(void);

/**
*       @brief Function to transfer receive data from CAN Layer to APP Layer
*       @ingroup CAN
*
*       This function is called when there was a complete reception and it is desired to transfer the data
*       from the CAN layer to the APP layer to be processed.
*       @sa Modbus_App_L_Msg_Set, Modbus_App_Receive_Char
*/
void Modbus_CAN_to_App(void);

/**
*       @brief Function to manage errors.
*       @ingroup CAN
*
*       This function manage the errors depending on the type of each one, for now there is only one error handled:
*
*       - <b>110</b>: Important error, the application becomes blocked for security.
*
*       @param error Number of the error.
*/
void Modbus_CAN_Error_Management(unsigned char error);

/**     @brief Function to turn on the led.
*       @ingroup CAN
*
*       Function to turn on the led.
*/
static inline void ledOn(void);

/**     @brief Function to turn off the led.
*       @ingroup CAN
*
*       Function to turn off the led.
*/
static inline void ledOff(void);

//DEBUG
//unsigned char * getOutput();
unsigned char * getInput();
unsigned char Debug_Transmission(void);
unsigned char Debug_Reception(void);
#endif
#endif