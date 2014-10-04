#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_can.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/interrupt.h"
#include "driverlib/uart.h"
#include "inc/lm3s8962.h"
#include "Master/Modbus_App.h"

static unsigned long g_ulDebounceCounter;
static volatile unsigned char g_ucButtonStatus;
static volatile int exitt = 0;

void init(void);
void printString(char * string);
void printSimpleData(unsigned char car);
void printStringSinCarro(char * string);
void printSimpleDataSinCarro(unsigned char car);
void printInt(unsigned char string);
void print(unsigned char string);

void
SysTickIntHandler(void)
{
    unsigned long ulStatus;
    unsigned char ucTemp;
    static unsigned long ulLastStatus = 0;

    //
    // Read the current value of the button pins.
    //
    ulStatus = GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_1);
    if(ulStatus != ulLastStatus)
    {
        //
        // Something changed reset the compare and reset the debounce counter.
        //
        ulLastStatus = ulStatus;
        g_ulDebounceCounter = 0;
    }
    else
    {
        //
        // Increment the number of counts with the push button in a different
        // state.
        //
        g_ulDebounceCounter++;

        //
        // If four consecutive counts have had the push button in a different
        // state then the state has changed.
        //
        if(g_ulDebounceCounter == 4)
        {
            //
            // XOR to see what has changed.
            //
            ucTemp = g_ucButtonStatus ^ ulStatus;
     
            if(ucTemp & GPIO_PIN_1)
            {
                //
                // The button status has changed to pressed.
                //
                if((GPIO_PIN_1 & ulStatus) == 0)
                {
                      exitt = 1;
                }                
            }            
            g_ucButtonStatus = (unsigned char)ulStatus;
        }
    }
}

void main(void)
{
        static unsigned char * resIn1;
        //static unsigned char * resOut1;
        static unsigned char data[2000];                          
        static uint16_t data16[125];
        static unsigned char coils_change[2000];   
        static uint16_t registers_change[125];
        static uint16_t registers_change2[125];
        static int i;//DEBUG
        static unsigned char ant, now, intentos;      
        exitt = 0;        
        //SYSTEM CLOCK PLL-> 400Mhz/2 = 200Mhz / DIV_5= 40 Mhz
        //CAN CLOCK works at 8Mhz always
	SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_8MHZ |  SYSCTL_OSC_MAIN);
        //SysCtlClockSet(SYSCTL_USE_PLL | SYSCTL_XTAL_8MHZ |  SYSCTL_OSC_MAIN);
        //Enabling pull-ups
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
        GPIOPadConfigSet(GPIO_PORTC_BASE,
                     GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3,
                     GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
        //Enabling of two buttons      
         SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
         
         GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_1);
         
          GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_1,
                     GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);      
          
         IntMasterEnable();
         g_ucButtonStatus = GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_1);
         SysTickPeriodSet(SysCtlClockGet() / 100);
         SysTickEnable();
         SysTickIntEnable();	
         //UART STARTING        
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
        SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);        
        GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_1);
        UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 9600,
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                         UART_CONFIG_PAR_NONE));      
        //MODBUS INIT
        for(i=0; i < 2000; i++)
            coils_change[i]= 0;
        for(i=0; i < 125; i++)
           registers_change[i]= 124-i;
        for(i=0; i < 125; i++)
           registers_change2[i]= i;
        init();            
        printString("M: Master configured");
        //I wait to push a button        
        while(!exitt)
        {         
        }                
	//I send the data:                
        Modbus_Read_D_Inputs (1, 0, 2000, data);        
        printString("M: Function 1 sent.");   
        Modbus_Read_Coils( 1, 0, 2000, data);         
        printString("M: Function 2 sent.");
        Modbus_Read_H_Registers (1, 0, 125, data16);         
        printString("M: Function 3 sent.");
        Modbus_Write_M_Coils (1, 0, 1968, coils_change);
        printString("M: Function 4 sent.");       
        Modbus_Read_Coils( 1, 0, 2000, data);         
        printString("M: Function 5 sent.");
        Modbus_Write_M_Registers (1, 0, 123, registers_change);
        printString("M: Function 6 sent.");
        Modbus_Read_H_Registers (1, 0, 125, data16);
        printString("M: Function 7 sent.");
        Modbus_Read_Write_M_Registers (1, 0, 125, data16, 0, 121, registers_change2);
        printString("M: Function 8 sent.");  
        Modbus_Mask_Write_Register (1, 73, 0x0000, 0xFFFF);
        printString("M: Function 9 sent.");  
        Modbus_Read_H_Registers (1, 0, 74, data16);
        printString("M: Function 10 sent.");
        //wait answer
        ant = 0;
        printString("///////////////////");   
        while(Modbus_Master_Communication())
        {       
            now = getbu();
          //  if(ant != now && now != 0)
            if(now == 1)
            {
              if(getboo() == 1){
                    ant = 0;                    
              }
                  ant++;                   
                  printSimpleData(ant);                     
                  resIn1 = getInput();              
                  for(i =0; i < getIndex();i++)
                      printSimpleDataSinCarro(resIn1[i]); 
                  printString(" ");       
                  setbu();
                  intentos = getAttempts();
                  if(intentos > 1)
                      printString("Temporizador ha saltado");
            }
        }                
        printString("///////////////////");   
        printSimpleData(resIn1[1]);
        printSimpleData(getbu());        
        if(Debug_Timeout())
        {
            printString("M: Timeout activado");   
        }
        else
        {            
            printString("M: Data received."); 
            printString("M: Timeout no activado");                      
        }        
        //resOut1 = getOutput();                       
        while(1){}
}

void init(void)
{
  unsigned char attempts = 3;	
  enum Modbus_CAN_BitRate bit_rate = MODBUS_1MBPS;
  Modbus_Master_Init(bit_rate, attempts);  
}

void printString(char * string)
{                
    while(*string != '\0')
    {        
        UARTCharPut(UART0_BASE, *string);
        string++;
    }
    UARTCharPut(UART0_BASE, '\n');
}

void printStringSinCarro(char * string)
{                
    while(*string != '\0')
    {        
        UARTCharPut(UART0_BASE, *string);
        string++;
    }
    //UARTCharPut(UART0_BASE, '\n');
}

void printSimpleDataSinCarro(unsigned char car)
{
    static char str[17];
    if (car < 16) {
      sprintf(str, "0%X", car);        
    } else {
      sprintf(str, "%X", car);        
    }
    printStringSinCarro(str);
}

void printSimpleData(unsigned char car)
{
    static char str[17];
    if (car < 16) {
      sprintf(str, "0%X", car);        
    } else {
      sprintf(str, "%X", car);        
    }
    printString(str);
}

void printInt(unsigned char string)
{                
        UARTCharPut(UART0_BASE, (string +'0') );    
        UARTCharPut(UART0_BASE, '\n');
}

void print(unsigned char string)
{                
        UARTCharPut(UART0_BASE, string );    
        UARTCharPut(UART0_BASE, '\n');
}