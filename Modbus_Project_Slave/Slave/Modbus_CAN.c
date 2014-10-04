// Author: Francisco Javier Guzman Jimenez, <dejavits@gmail.com>
#ifdef CAN_Mode
//! \\addtogroup Slave_CAN
//! @{
//includes
#include "inc/hw_ints.h"
#include "inc/hw_can.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/lm3s2110.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/can.h"
#include "driverlib/interrupt.h"
#include "Modbus_App.h"
#include "Modbus_CAN.h"

//GLOBAL VARIABLES:
//-SYSTEM
//! Variable used to represent the status of the slave.
static  enum Modbus_MainState modbus_slave_state;
//! Variable used to store which slave is this one itself.
static  unsigned char slave;
//! Variable used to store if a reception was completed.
static unsigned char modbus_complete_reception;
// Variable used to store if a transmission was completed
//static unsigned char modbus_complete_transmission;
//! Variable to activate when a Broadcast was received; In this way the response is not built.
static unsigned char modbus_broadcast;
//! Variable to save the input length.
static  unsigned char input_length;
//!Variable to store the input data.
static  unsigned char input_pdu[MAX_PDU];
//! Variable to index the input_pdu.
static unsigned char modbus_index;
//!Variable to store the buffer input data.
static unsigned char buffer_input_pdu[MAX_FRAME];

//-CAN
//!Variable used to store the bit rate of the communications.
static enum  Modbus_CAN_BitRate modbus_bit_rate;
//! Variable used to store both bit time and rate information.
static tCANBitClkParms modbus_canbit;
//! Waiting time in cycles*3 between sendings.
static unsigned long modbus_delay;
//! Receive Message Object.
static  tCANMsgObject RxObject;
//! Transmit Message Object.
static  tCANMsgObject TxObject;
//! @}

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
                //error processing, resend, etc.
                case CAN_STATUS_BUS_OFF:
                case CAN_STATUS_EWARN:
                case CAN_STATUS_EPASS:
                                      //RESET SHOULD BE DONE
                                      Modbus_CAN_Error_Management(110);                                      
                                      break;
                case CAN_STATUS_RXOK:  
                case CAN_STATUS_TXOK:
                                      //ALL OK
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
        // I  notify that I sent the data correctly
        //modbus_complete_transmission = 1;
    }
    else if(can_status == 17) // Message 17 gets the unicasts
    {                    
        if(!modbus_complete_reception)
        {
            ledOn();
            modbus_broadcast = 0;
            Modbus_CAN_CallBack();                     
            ledOff();
        }
      
    }
    else if(can_status == 18) //Message 18 gets the broadcasts
    {      
      if(!modbus_complete_reception)
      {  
          ledOn();
          modbus_broadcast = 1;
          Modbus_CAN_CallBack();               
          ledOff();
      }
    }
    else
    {
          //spurious cause
    }
}

void Modbus_CAN_Init(enum Modbus_CAN_BitRate bit_rate, unsigned char slave_number)
{
                Modbus_SetMainState(MODBUS_INITIAL);     
                //LED CONFIGURATION
                SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
                GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);   
                ledOn();
                //////////Variables//////////
		slave = slave_number;      
                //modbus_complete_transmission = 0;               
                modbus_bit_rate = bit_rate;
                //set bit timing, bit rate and delay
                Modbus_CAN_SetBitRate(modbus_bit_rate);                                
                //CAN ENABLING	
                SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);      
                //I enable the pins to be used as CAN pins
                GPIOPinTypeCAN(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_1);  
                SysCtlPeripheralEnable(SYSCTL_PERIPH_CAN0);    
                //Init CAN Module
                CANInit(MODBUS_CAN);
                //Set bit timing
                CANSetBitTiming(MODBUS_CAN, &modbus_canbit);                                       
                //ENABLING CAN INTERRUPTIONS                
                CANIntEnable(MODBUS_CAN, CAN_INT_ERROR |CAN_INT_STATUS | CAN_INT_MASTER);        
                IntEnable(INT_CAN0);
                //Enable CAN Module
                CANEnable(MODBUS_CAN);
                //Receive message object configuration
                Modbus_CAN_ReceptionConfiguration();
                /////////////////LED off///////////////////////
                ledOff();
                /////////////Status/////
                Modbus_SetMainState(MODBUS_IDLE);     
}

enum Modbus_MainState Modbus_GetMainState(void)
{
	return modbus_slave_state;
}


void Modbus_SetMainState(enum Modbus_MainState state)
{
	modbus_slave_state = state;
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

void Modbus_CAN_FixOutput(unsigned char *mb_req_pdu, unsigned char pdu_length)
{
	unsigned char aux_length;
        unsigned char local_output[MAX_FRAME];
        int i, iterations, objNumber;
        uint16_t registerr;                  
            //init variables
            objNumber = 1;                     
            iterations = 0;  
            aux_length = pdu_length;        
            //storing in global variables                   
            //modbus_complete_transmission = 0;                        
            //body:
            ledOn();
            // 000 + 00000000(slave)= Individual Frame (0)
            // 010 + slave = Beginning Long Frame (1)
            // 100 + slave = Continuation Long Frame (4)
            // 110 + slave = End Long Frame (6)
            TxObject.ulMsgIDMask = 0x000;//It's not used mask, I send all messages without filtering                                       
            while(aux_length) // != 0 true
            {                
                registerr = 0x000;
                if(aux_length <= MAX_FRAME)//ONE FRAME OR THE LAST LONG ONE
                {                             
                    registerr=0x6; // if I do not enter in the below statement, I mark the frame as End Long Frame
                    if(pdu_length <= MAX_FRAME) //I send an Individual Frame
                    {              
                          registerr = 0x0;                               
                    }                    
                    TxObject.ulMsgID = ((registerr << 8) | slave);
                    TxObject.ulFlags = MSG_OBJ_TX_INT_ENABLE;
                    TxObject.ulMsgLen = aux_length;      
                    
                    for(i=0; i < aux_length; i++)
                    {
                        local_output[i] = mb_req_pdu[i + (iterations * MAX_FRAME)];                             
                    }                    
                    aux_length -= aux_length;// it should be 0 now                      
                    ledOff();                    
                }
                else //division needed because it's > 8 bits
                {
                    //in case is not the first Long Frame, then it's a continuation
                    registerr = 0x4;
                    // first Long Frame
                    if(aux_length == pdu_length)
                    {
                        registerr = 0x2;
                    }
                    TxObject.ulMsgID = ((registerr << 8) | slave);                    
                    TxObject.ulFlags = MSG_OBJ_NO_FLAGS;
                    TxObject.ulMsgLen = MAX_FRAME;
                    for(i=0; i < MAX_FRAME; i++)
                    {
                        local_output[i] = mb_req_pdu[i + (iterations * MAX_FRAME)];                        
                    }
                    aux_length -= MAX_FRAME;                    
                    iterations++;                    
                }                             
                TxObject.pucMsgData = local_output;
                CANMessageSet(MODBUS_CAN, objNumber, &TxObject, MSG_OBJ_TYPE_TX);
                // If it is not the last frame
                if(aux_length)
                    Modbus_CAN_Delay();
            }        
}

void Modbus_CAN_ReceptionConfiguration(void)
{
        // It is required to receive unicast frames from Master (P/R = 1)+slave
        // and broadcast frames from Master (P/R = 1) + 0
        int objNumber = 17;        
        modbus_complete_reception = 0;
       //RECEPTION MESSAGE OBJECT num.17 UNICAST num.18 BROADCAST              
        RxObject.ulMsgID = (0x1 << 8) | slave; //xx1+ slave
        RxObject.ulMsgIDMask = 0x1FF;
        RxObject.ulFlags = MSG_OBJ_USE_ID_FILTER | MSG_OBJ_RX_INT_ENABLE;
        RxObject.pucMsgData = &buffer_input_pdu[0];
        CANMessageSet(MODBUS_CAN, objNumber, &RxObject, MSG_OBJ_TYPE_RX);
        //MESSAGE OBJECT 18
        objNumber = 18;
        RxObject.ulMsgID = (0x1 << 8) | 0; //xx1+ slave=0 
        CANMessageSet(MODBUS_CAN, objNumber, &RxObject, MSG_OBJ_TYPE_RX);
}

void Modbus_CAN_CallBack(void)
{
// I wait for xx1 | slave because the mask of the message object was 1FF;    
    int i, numObj;
    uint32_t new_data, mask;
    numObj = 17;   
    mask = 0x10000;
    new_data = CANStatusGet(MODBUS_CAN, CAN_STS_NEWDAT);
    if(modbus_broadcast)    
    {
        numObj = 18;    
        mask = 0x20000;
    }
    if(( (new_data & mask) >> (numObj-1) ) == 1)//is there new data?
    {              
        CANMessageGet(MODBUS_CAN, numObj, &RxObject, true);       
        //header should be 001:
        if( (RxObject.ulMsgID & 0x700) == 0x100) //Individual Frame
        {
              modbus_complete_reception = 1; 
              input_length = RxObject.ulMsgLen;
              modbus_index = input_length;//not needed
              for(i=0; i < RxObject.ulMsgLen; i++)
              {
                    input_pdu[i] = RxObject.pucMsgData[i];
              }                                                                                          
        }
        //header should be 011:
        else if( (RxObject.ulMsgID & 0x700) == 0x300) // Beginning Long Frame
        {              
              for(i=0; i < RxObject.ulMsgLen; i++)
              {
                    input_pdu[i] = RxObject.pucMsgData[i];
              }              
              modbus_index = RxObject.ulMsgLen;           
        }
        else if( ( (RxObject.ulMsgID & 0x700) == 0x500) || ( (RxObject.ulMsgID & 0x700) == 0x700) )//CONTINUATION OR END OF LONG FRAME
        {
              for(i=0; i < RxObject.ulMsgLen; i++)
              {
                    input_pdu[modbus_index + i] = RxObject.pucMsgData[i];
              }              
              modbus_index += RxObject.ulMsgLen;              
              if( (RxObject.ulMsgID & 0x700) == 0x700) // END LONG FRAME
              {                  
                  modbus_complete_reception = 1;  
                  input_length = modbus_index;        
              }
        }                                  
        else
        {     // IT WAS EXPECTED A CONTINUATION OR AN END; IT SHOULD NOT ENTER HERE
              Modbus_SetMainState(MODBUS_ERROR);
        }        
    }
    else
    {   // IT WAS EXPECTED NEW DATA; IT SHOULD NOT ENTER HERE
        Modbus_SetMainState(MODBUS_ERROR);
    }    
}

unsigned char Modbus_CAN_Controller(void)
{
  if(Modbus_GetMainState() == MODBUS_IDLE)
  {
    if(modbus_complete_reception)
    {            
      Modbus_CAN_to_App();    
      Modbus_SetMainState(MODBUS_CHECKING);
      Modbus_App_Manage_Request();    
      modbus_complete_reception = 0;
      if(!modbus_broadcast) //it's not a broadcast request
      {
        Modbus_SetMainState(MODBUS_REPLY);
        Modbus_App_Send();
      }                 
      Modbus_SetMainState(MODBUS_IDLE);                        
      return 1;
    }
  }
  else
  {
      Modbus_CAN_Error_Management(110); //weird case, we go to security loop
  }
  return 0;
}

void Modbus_CAN_Delay(void)
{
    SysCtlDelay(modbus_delay);
}

unsigned char Modbus_CAN_BroadCast_Get(void)
{
    return modbus_broadcast;
}

void Modbus_CAN_to_App(void)
{	
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
     GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2 , GPIO_PIN_2);
}

static inline void ledOff(void)
{
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2 , ~GPIO_PIN_2);
}


/*unsigned char Debug_Reception(void){
        return modbus_complete_reception;
}

void reset_Reception(void){        
        modbus_complete_reception = 0;        
}

unsigned char Debug_Transmission(void){
        return modbus_complete_transmission;
}

unsigned char * getOutput()
{
    return output_pdu;
}

unsigned char * getInput()
{
    return input_pdu;
}

unsigned char getbu(void)
{
    return buu;
}
*/
#endif