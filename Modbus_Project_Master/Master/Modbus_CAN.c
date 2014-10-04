// Author: Francisco Javier Guzman Jimenez, <dejavits@gmail.com>
#ifdef CAN_Mode
//! \\addtogroup Master_CAN
//! @{
//includes
#include "inc/hw_ints.h"
#include "inc/hw_can.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/lm3s8962.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/can.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"
#include "Modbus_App.h"
#include "Modbus_CAN.h"

//GLOBAL VARIABLES:
//-SYSTEM
//! Variable used to represent the status of the Master
static  enum Modbus_MainState modbus_master_state;
//! Variable used to store the maximum attempts to send data
static  unsigned char modbus_max_attempts;
//! Variable used to store how many attempts are already done
static  unsigned char modbus_attempts;
//! Variable used to store if data needs to be resent
static  unsigned char modbus_forward_flag;
//! Variable used to store if a complete transmission was done
static  unsigned char modbus_complete_transmission;
//! Variable used to store if a complete reception was done
static  unsigned char modbus_complete_reception;
//!Variable used to store the timeout for broadcast
static  unsigned long modbus_broadcast_timeout;
//!Variable used to store the timeout for unicast requests
static  unsigned long modbus_unicast_timeout;
//!Variable to index the incoming data
static unsigned char modbus_index;
//! Input data
static unsigned char input_pdu[MAX_PDU];
//! Input data buffer
static  unsigned char input_pdu_buffer[MAX_FRAME];
//! Input data length
static unsigned char input_length;
//! Waiting time in cycles*3 between sendings
static unsigned long modbus_delay;

//-CAN
//!Variable used to store the bit rate range of the communications
static enum  Modbus_CAN_BitRate modbus_bit_rate;
//! Variable used to store both bit time and rate information
static tCANBitClkParms modbus_canbit;
//! Receive Message Object
static  tCANMsgObject RxObject;
//! Transmit Message Object
static  tCANMsgObject TxObject;
//! @}

//FOR DEBUGGING:
// Output data length; NOT NEEDED
//static  unsigned char output_length;
// Output data; NOT NEEDED
//static  unsigned char output_pdu[MAX_PDU];
static unsigned char modbus_timeout;

static unsigned char buu = 0;
static uint16_t boo = 0;

void Modbus_CAN_IntHandler(void)
{
    unsigned long can_status, can_sts_status;   
    //I search the cause of the interruption
    can_status = CANIntStatus(MODBUS_CAN, CAN_INT_STS_CAUSE);
    if(can_status == CAN_INT_INTID_STATUS)
    {             
       //some "errors" occurred
        can_sts_status = CANStatusGet(MODBUS_CAN, CAN_STS_CONTROL);  
        switch(can_sts_status)
        {
                //error processing, bus-off state, etc.
                case CAN_STATUS_BUS_OFF:
                case CAN_STATUS_EWARN:
                case CAN_STATUS_EPASS: 
                                      //RESET SHOULD BE DONE
                                      Modbus_CAN_Error_Management(110);
                                      break;
                case CAN_STATUS_RXOK:                                       
                case CAN_STATUS_TXOK: /*ALL OK*/
                                      break;
                case CAN_STATUS_LEC_MSK:                  
                case CAN_STATUS_LEC_STUFF:
                case CAN_STATUS_LEC_FORM:
                case CAN_STATUS_LEC_BIT1:
                case CAN_STATUS_LEC_BIT0:                                      
                case CAN_STATUS_LEC_ACK:
                case CAN_STATUS_LEC_CRC: /*CAN ERROR FRAME, NO PROBLEM, TIMEOUT WILL BE TRIGGERED*/
                                        break;
                default:             
                                        break;
        }            
        CANIntClear(MODBUS_CAN, can_status);
    }
    else if(can_status >= 1 && can_status <= 16)
    {
        //LAST SENDING message object should have the interruption pending, so last data was sent             
        CANIntClear(MODBUS_CAN, can_status);//clear interruption        
        // I should notify in some way that I sent the data correctly       
        modbus_complete_transmission = 1;                
    }
    else if(can_status == 17) // Message 17 should be the first one to have an interruption
    {        
        //I process the received data:                                      
         buu = 1;     //DEBUGGGGGGGGGGGGGGGGG
         if(!modbus_complete_reception)
         {     
             ledOn();
             Modbus_CAN_CallBack();                                                    
             ledOff();             
         }                         
    }
    else if(can_status == 18)
    {
        //NO BROADCAST RESPONSE SHOULD BE RECEIVED in the master!        
        //Modbus_CAN_CallBack();
    }
    else
    {
          //spurious cause          
    }
}


void Modbus_CAN_Init(enum Modbus_CAN_BitRate bit_rate, unsigned char attempts)
{                
        Modbus_SetMainState(MODBUS_INITIAL);
        /////////////////Variables///////////
	modbus_max_attempts = attempts;
        modbus_bit_rate = bit_rate;
        Modbus_CAN_SetBitRate(bit_rate);         
        modbus_forward_flag = 0;                
        modbus_attempts = 1;  
        modbus_index = 0;
        modbus_complete_transmission = 0;
        modbus_complete_reception = 0;
        modbus_timeout = 0; // DEBUGGGGGGGGGGGGGGGGGGGGG
	//CAN ENABLING	
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);      
        //I enable the pins to be used as CAN pins
        GPIOPinTypeCAN(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_1);  
        SysCtlPeripheralEnable(SYSCTL_PERIPH_CAN0);       
        //Timers Initialisation
        SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
        SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
        IntMasterEnable();
        TimerConfigure(TIMER1_BASE, TIMER_CFG_ONE_SHOT);                
        TimerConfigure(TIMER2_BASE, TIMER_CFG_ONE_SHOT);                        
        IntEnable(INT_TIMER1A);
        IntEnable(INT_TIMER2A);
        TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
        TimerIntEnable(TIMER2_BASE, TIMER_TIMA_TIMEOUT);        
        //Init CAN Module
        CANInit(MODBUS_CAN);
        //Set bit timing
        CANSetBitTiming(MODBUS_CAN, &modbus_canbit);               
        //ENABLING CAN INTERRUPTIONS        
        CANIntEnable(MODBUS_CAN, CAN_INT_ERROR |CAN_INT_STATUS | CAN_INT_MASTER);       
        IntEnable(INT_CAN0);
        //LED STARTING
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
        GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0);
        //TURN OFF LED
        ledOff();
        //Enable CAN Module
        CANEnable(MODBUS_CAN);        
	Modbus_SetMainState(MODBUS_IDLE);
}

enum Modbus_MainState Modbus_GetMainState(void)
{
	return modbus_master_state;
}

void Modbus_SetMainState(enum Modbus_MainState state)
{
	modbus_master_state = state;
}

void Modbus_CAN_SetBitRate(enum Modbus_CAN_BitRate bit_rate)
{
      switch(bit_rate)
      {
            case MODBUS_100KBPS:
                          modbus_canbit.ulSyncPropPhase1Seg = 5;      //3 *tq; following the example from the microcontroller's pdf
                          modbus_canbit.ulPhase2Seg = 4;
                          modbus_canbit.ulSJW = 4;
                          modbus_canbit.ulQuantumPrescaler = 8;
                          /*(4000000/3) * 0.07 *10 = 9333333 ("TRANSFER TIME"); 
                             900000 * 4 = process time
                          */
                          modbus_delay = 9333333 + 900000*4; // 10 times slower THAN 1MBPS
                          break;
            case MODBUS_1MBPS:
                          modbus_canbit.ulSyncPropPhase1Seg = 5;      //3 *tq; following the example from the microcontroller's pdf
                          modbus_canbit.ulPhase2Seg = 2;
                          modbus_canbit.ulSJW = 2;
                          modbus_canbit.ulQuantumPrescaler = 1;
                          /*(4000000/3) * 0.07 = 933333 ("TRANSFER TIME"); 
                            900000*4 = process time
                          */
                          modbus_delay = 933333 + 900000*4;
                          break;
            default : Modbus_CAN_Error_Management(110); break;
      }
}

void Modbus_CAN_FixOutput(unsigned char *mb_req_pdu, unsigned char slave, unsigned char pdu_length, uint16_t amount_guess)
{
        unsigned char aux_length;
        unsigned char local_output[8];        
        int i, iterations, objNumber;// index;// OUTPUT_PDU IS NOT NEEDED, ONLY IN DEBUG
        uint16_t registerr;               
            //init variables            
            objNumber = 1;    
            iterations = 0;  
            aux_length = pdu_length;
            registerr = 0x000;
            //index = 0;            
            //storing in global variables
            //output_length = pdu_length;
            modbus_complete_transmission = 0;                 
            //TURN ON LED
            ledOn();
            if(slave)
                Modbus_CAN_ReceptionConfiguration(slave);
            else
                modbus_complete_reception = 0;            
            //body:
            // 001 + 00000000(slave)= Individual Frame (1)
            // 011 + slave = Beginning Long Frame (3)
            // 101 + slave = Continuation Long Frame (5)
            // 111 + slave = End Long Frame (7)
            TxObject.ulMsgIDMask = 0x000;//It's not used mask, I send all messages without filtering                                                             
            while(aux_length) // != 0 true
            {                
                if(aux_length <= MAX_FRAME) //ONE FRAME OR THE LAST LONG ONE
                {                    
                    registerr=0x7; // if I do not enter in the below statement, I mark the frame as End Long Frame                    
                    if(pdu_length <= MAX_FRAME) //I send an Individual Frame, so I will leave soon
                    {              
                          registerr = 0x1;                               
                    }                    
                    TxObject.ulFlags = MSG_OBJ_TX_INT_ENABLE;
                    TxObject.ulMsgID = ((registerr << 8) | slave);                    
                    TxObject.ulMsgLen = aux_length;      
                    /* It's the last frame so:
                       Maximum size to send - remainder + i
                    */                      
                    for(i=0; i < aux_length; i++)
                    {
                        local_output[i] = mb_req_pdu[i + (iterations * MAX_FRAME)]; 
                        //output_pdu[index++] = local_output[i];                        
                    }                                              
                    aux_length -= aux_length;// it should be 0 now                        
                    //It is checked if is an unicast or a broadcast, and it is put the timers
                    if(slave) //unicast
                    {
                          Modbus_SetMainState(MODBUS_WAITREPLY);
                          Modbus_CAN_UnicastTimeout(amount_guess);
                    }
                    else//slave == 0
                    {
                          Modbus_SetMainState(MODBUS_TURNAROUND);
                          Modbus_CAN_BroadcastTimeout(amount_guess);
                    }
                    //TURN OFF LED
                    ledOff();
                }
                else //division needed because it's > 8
                {
                    //if not the first Long Frame, it's a continuation
                    registerr = 0x5;
                    // first Long Frame
                    if(aux_length == pdu_length)
                    {
                        registerr = 0x3;
                    }
                    TxObject.ulMsgID = ((registerr << 8) | slave);
                    // not the last frame, so I do NOT use INTERRUPTIONS
                    TxObject.ulFlags = MSG_OBJ_NO_FLAGS;
                    TxObject.ulMsgLen = MAX_FRAME;
                    for(i=0; i < MAX_FRAME; i++)
                    {
                        local_output[i] = mb_req_pdu[i + (iterations * MAX_FRAME)];
                        //output_pdu[index++] = local_output[i];                        
                    }
                    aux_length -= MAX_FRAME;
                    iterations++;
                }
                TxObject.pucMsgData = local_output;
                CANMessageSet(MODBUS_CAN, objNumber, &TxObject, MSG_OBJ_TYPE_TX);                    
                // If is not the last frame
                if(aux_length)               
                    Modbus_CAN_Delay();                
            }                       
}

void Modbus_CAN_ReceptionConfiguration(unsigned char slave)
{
        int objNumber = 17;        
        modbus_complete_reception = 0;
       //RECEPTION MESSAGE OBJECT num.17          
        //I will receive all types of answer from the concrete slave
        RxObject.ulMsgID = slave; //xx0+ 0000+ 0000
        RxObject.ulMsgIDMask = 0x1FF;        
        RxObject.ulFlags = MSG_OBJ_USE_ID_FILTER | MSG_OBJ_RX_INT_ENABLE;
        RxObject.pucMsgData = &input_pdu_buffer[0];
        CANMessageSet(MODBUS_CAN, objNumber, &RxObject, MSG_OBJ_TYPE_RX);    
        //No broadcast receive message object is needed
}

void Modbus_CAN_CallBack(void)
{
    // I am expecting for xx0 | slave because the mask of the message object was 1FF;    
    int i, numObj;
    uint32_t new_data, mask;
    numObj = 17;
    mask = 0x10000;
    new_data = CANStatusGet(MODBUS_CAN, CAN_STS_NEWDAT);      
    if(( (new_data & mask) >> (numObj-1) ) == 1)
    {                                             
        CANMessageGet(MODBUS_CAN, numObj, &RxObject, true); // I DO CLEAN THE INTERRUPTION                
        //header should be 000
        if( (RxObject.ulMsgID & 0x700) == 0x000) //Individual Frame
        {
              modbus_complete_reception = 1;
              Modbus_CAN_RemoveTimeout();
              input_length = RxObject.ulMsgLen;
              modbus_index = input_length;                          
              for(i=0; i < RxObject.ulMsgLen; i++)
              {
                    input_pdu[i] = RxObject.pucMsgData[i];
              }                              
              boo = 1;
        }
        //header should be 010
        else if( (RxObject.ulMsgID & 0x700) == 0x200) // Beginning Long Frame
        {              
              for(i=0; i < RxObject.ulMsgLen; i++)
              {
                    input_pdu[i] = RxObject.pucMsgData[i];
              }              
              modbus_index = RxObject.ulMsgLen;                 
              boo = 1;
        }        
        // I CATCH OUT THE CONTINUATION LONG FRAMES and the END ONES                
        else if( ( (RxObject.ulMsgID & 0x700) == 0x400) || ( (RxObject.ulMsgID & 0x700) == 0x600) )
        {
            for(i=0; i < RxObject.ulMsgLen; i++)
            {
                input_pdu[modbus_index + i] = RxObject.pucMsgData[i];
            }            
            modbus_index += RxObject.ulMsgLen;                    
            //END LONG FRAME
            if( (RxObject.ulMsgID & 0x700) == 0x600)
            {                  
                  modbus_complete_reception = 1;                     
                  Modbus_CAN_RemoveTimeout();                  
                  input_length = modbus_index;                        
            }
            boo = 0;
         }
         else
         {
             // IT WAS EXPECTED A CONTINUATION OR AN END;IT SHOULD NOT ENTER HERE
             Modbus_SetMainState(MODBUS_ERROR);
         }                          
      }         
      else
      {     // IT WAS EXPECTED NEW DATA; IT SHOULD NOT ENTER HERE
            Modbus_SetMainState(MODBUS_ERROR);
      }                
}

unsigned char Modbus_CAN_Controller(void)
{
	 switch(Modbus_GetMainState())
	 {
	 	 case MODBUS_IDLE:
                                 // Is it needed to resend?
                                 if(Modbus_CAN_GetForwardFlag())
                                 {
                                        Modbus_App_Send(); //From APP layer I send again the Output data
                                 }
                                 else
                                 {
                                   //If forward is not needed and there are messages left in the FIFO, the next one is sent.
                                   // If there are no messages left in the FIFO, it's returned 0.
                                   if(Modbus_App_FIFOSend())
                                     return 0;
                                 }
                                 break;
	 	 case MODBUS_WAITREPLY:
                                   //I am waiting an answer, if there is already one, it's processed
                                   if(modbus_complete_reception)
                                   {
                                          modbus_complete_reception = 0;
                                          Modbus_SetMainState(MODBUS_PROCESSING);
                                          Modbus_CAN_to_App();
                                          Modbus_App_Manage_CallBack();                                         
                                   }                                                                   
                                    break;
                 case MODBUS_TURNAROUND: /* NOTHING, I JUST WAIT FOR BROADCAST TIMEOUT*/
                                    break;
	 	 case MODBUS_ERROR:	 		 	
	 		          // Wrong answer, forward flag activated
	 		          // If max. attempts is achieved, we forget & pass to IDLE
                                    Modbus_CAN_Repeat_Request();                                    
                                    Modbus_SetMainState(MODBUS_IDLE);	 		        
                                    break;
	 }
         return 1;
}

void Modbus_CAN_Delay(void)
{
    SysCtlDelay(modbus_delay);
}

unsigned char Modbus_CAN_GetForwardFlag(void)
{
	unsigned char result;
	result = modbus_forward_flag;
	modbus_forward_flag = 0;
	return result;
}

void Modbus_CAN_Reset_Attempt(void)
{
	modbus_attempts = 1;
}


void Modbus_CAN_Repeat_Request(void)
{
  if(modbus_attempts < modbus_max_attempts)
  {
      //I resend, so I activate the flag
      modbus_attempts++;
      modbus_forward_flag = 1;
  }
  else
  {     
      // I cannot resend, so I forget
      Modbus_App_No_Response();
      modbus_attempts = 1;
  }
}

void Modbus_CAN_Timeouts(void)
{
  switch(Modbus_GetMainState())
  {
    case MODBUS_WAITREPLY:          
          Modbus_SetMainState(MODBUS_ERROR);
          break;
    case MODBUS_TURNAROUND:
          Modbus_SetMainState(MODBUS_IDLE);
          break;
    default:
          Modbus_CAN_Error_Management(110);
  }
}

void Modbus_CAN_UnicastTimeoutHandler(void)
{            
    if(!modbus_complete_reception)
    {        
        Modbus_CAN_Timeouts();
        modbus_timeout = 1; //DEBUGGGGGGGGGGGGGGGGGGGGGGGGG
    }    
    TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
}

void Modbus_CAN_UnicastTimeout(uint16_t amount_guess)
{
   switch(modbus_bit_rate)
   {
         case MODBUS_100KBPS:    
                          /*
                             (amount_guess * 9333333) = TRANSFER TIME
                             (900000 * amount_guess * 4) = PROCESS TIME
                             ((modbus_attempts-1) * 8000) = CONGESTION AVOIDANCE
                          */  
                          modbus_unicast_timeout = (amount_guess * 9333333) + (900000 * amount_guess * 4) + ((modbus_attempts-1) * 8000);                                                    
                          break;
                          
         case MODBUS_1MBPS: /*
                             (amount_guess * 933333) = TRANSFER TIME
                             (900000 * amount_guess * 4) = PROCESS TIME
                             ((modbus_attempts-1) * 8000) = CONGESTION AVOIDANCE
                            */                         
                          modbus_unicast_timeout = (amount_guess * 933333) + (900000 * amount_guess * 4) + ((modbus_attempts-1) * 8000);
                          break;     
   }
   TimerLoadSet(TIMER1_BASE, TIMER_A, modbus_unicast_timeout);      
   TimerEnable(TIMER1_BASE, TIMER_A);
}

void Modbus_CAN_BroadcastTimeoutHandler(void)
{    
        Modbus_CAN_Timeouts();
        TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
}

void Modbus_CAN_BroadcastTimeout(uint16_t amount_guess)
{
   switch(modbus_bit_rate)
   {
         case MODBUS_100KBPS:                          
                          modbus_broadcast_timeout = ((amount_guess * 9333333) + (900000 * amount_guess * 4)) * 2;
                          break;
         case MODBUS_1MBPS:                          
                          modbus_broadcast_timeout = ((amount_guess * 933333) + (900000 * amount_guess * 4)) * 2;
                          break;    
   }
   TimerLoadSet(TIMER2_BASE, TIMER_A, modbus_broadcast_timeout);      
   TimerEnable(TIMER2_BASE, TIMER_A);      
}

void Modbus_CAN_RemoveTimeout(void)
{
   //Disable Unicast Timer
   modbus_timeout = 0;//DEBUGGGGGGGGGGGGGGGGGGGGGGGGGGGGG
   TimerDisable(TIMER1_BASE, TIMER_A); 
}

void Modbus_CAN_to_App(void)
{        
	//This is used to store the message length and the data
	int index;                
        Modbus_App_L_Msg_Set(input_length);        
        for(index = 0; index < input_length; index++)
        {
                Modbus_App_Receive_Char(input_pdu[index], index);
        }         
}

void Modbus_CAN_Error_Management(unsigned char error)
{
        switch(error)
        {             
              case 110: while(1){} 
                        break;
              default: while(1){} 
                     break;
        }
}

static inline void ledOn(void)
{
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0 , GPIO_PIN_0);
}

static inline void ledOff(void)
{
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0 , ~GPIO_PIN_0);
}

//////DEBUGGG
unsigned char Debug_Transmission(void)
{
  return modbus_complete_transmission;
}

unsigned char Debug_Reception(void)
{
  return modbus_complete_reception;
}

unsigned char * getInput()
{
    return input_pdu;
}

/*unsigned char * getOutput()
{
    return output_pdu;
}*/

unsigned char Debug_Timeout(void)
{
    return modbus_timeout;
}

unsigned char getbu(void)
{
    return buu;
}

void setbu(void)
{
      buu = 0;
}

unsigned char getIndex(void)
{
    return modbus_index;
}
unsigned char getAttempts(void)
{
    return modbus_attempts;
}
uint16_t getboo(void)
{
    return boo;
}
#endif