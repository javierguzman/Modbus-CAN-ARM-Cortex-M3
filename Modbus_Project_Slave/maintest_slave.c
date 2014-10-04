// Author: Francisco Javier Guzman Jimenez, <dejavits@gmail.com>
#include "stdint.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib - Slave/gpio.h"
#include "driverlib - Slave/sysctl.h"
#include "driverlib - Slave/systick.h"
#include "driverlib - Slave/interrupt.h"
#include "driverlib - Slave/uart.h"
#include "inc/lm3s2110.h"
#include "Slave/Modbus_App.h"

void init(void);

 static uint16_t coils_amount = 2000, discrete_inputs_amount = 2000, holding_registers_amount = 125, input_registers_amount = 125;
       static unsigned char *coils, *discrete_inputs;
       static uint16_t *holding_registers, *input_registers;      
       static unsigned char coils_data[2000], discrete_inputs_data[2000];
       static uint16_t holding_registers_data[125], input_registers_data[125];
      
void main(void)
{        
        //int i=0;
	SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_8MHZ |  SYSCTL_OSC_MAIN);         
        //BODY           
        init();                    
        while(1)
        {       
                  Modbus_Slave_Communication();
        }
}

//APP INIT
void init(void)
{      
      unsigned char slave = 1, num;
      uint16_t i;
      enum Modbus_CAN_BitRate bit_rate;      
      bit_rate = MODBUS_1MBPS;
      num = 1;
      for(i=0; i < coils_amount; i++)
      {                  
          coils_data[i] = num;
          
      }
      num = 0;
      for(i=0; i < discrete_inputs_amount; i++)
      {
          discrete_inputs_data[i] = num;
          num += 1;
          num= num % 2;
          if(i == 999)
              num = 1;
      }      
      for(i=0; i < holding_registers_amount; i++)
      {          
            holding_registers_data[i] = i;        
      }    
      for(i=0; i < input_registers_amount; i++)
      {          
            input_registers_data[i] = i; 
      }
      coils = coils_data;
      discrete_inputs = discrete_inputs_data;
      holding_registers = holding_registers_data;
      input_registers = input_registers_data;
      Modbus_Slave_Init(coils_amount, coils,
                                discrete_inputs_amount, discrete_inputs,
                                holding_registers_amount, holding_registers,
                                input_registers_amount, input_registers,
                                bit_rate, slave);
}