// Author: Francisco Javier Guzman Jimenez, <dejavits@gmail.com>
#ifndef __Modbus_FIFO_h
#define __Modbus_FIFO_h

//! \addtogroup FIFO
//! @{

#include "stdint.h"

//! Maximum number of items at the Request FIFO
#define MAX_ITEMS       255
//! Maximum number of items at the Error FIFO
#define MAX_E_ITEMS     25
//! A request can be the next different types
union Modbus_FIFO_Par
{
  unsigned char UC;    //!< 1 unsigned byte
  uint16_t UI2;        //!< 2 unsigned bytes
  unsigned char *PC;   //!< Pointer to link 1 unsigned byte elements
  uint16_t *PUI2;      //!< Pointer to link 2 unsigned bytes elements
   
};

//! Request FIFO item struct
struct Modbus_FIFO_Item
{
  unsigned char Slave;              //!< The slave which will receive the request
  unsigned char Function;           //!< Modbus public function code
  union Modbus_FIFO_Par Data[6];    //!< Request data
};

//! Communication Error FIFO item struct
struct Modbus_FIFO_E_Item
{
  struct Modbus_FIFO_Item Request; //!< Request which provoked the error
  unsigned char Response[2];       //!< Exception message (0 means no answer)
};

//! Request FIFO
struct Modbus_FIFO_s
{
  unsigned char Items;                        //!< Number of items in the FIFO
  unsigned char Head;                         //!< Head index
  unsigned char  Tail;                        //!< Tail index
  struct Modbus_FIFO_Item Buffer[MAX_ITEMS];  //!< Request petitions list
};

//! Error FIFO
struct Modbus_FIFO_Errors
{
  unsigned char Items;                           //!< Number of items in the FIFO
  unsigned char Head;                            //!< Head index
  unsigned char  Tail;                           //!< Tail index
  struct Modbus_FIFO_E_Item Buffer[MAX_E_ITEMS]; //!< Error messages list
};
//! @}

void Modbus_FIFO_Init (struct Modbus_FIFO_s *Modbus_FIFO_Ptr);
unsigned char Modbus_FIFO_Empty (struct Modbus_FIFO_s *Modbus_FIFO_Ptr);

unsigned char Modbus_FIFO_Enqueue (struct Modbus_FIFO_s *Modbus_FIFO_Ptr, 
                                   struct Modbus_FIFO_Item *Item);
unsigned char Modbus_FIFO_Dequeue (struct Modbus_FIFO_s *Modbus_FIFO_Ptr, 
                                   struct Modbus_FIFO_Item *Item);

void Modbus_FIFO_E_Init (struct Modbus_FIFO_Errors *Modbus_FIFO_Ptr);
unsigned char Modbus_FIFO_E_Enqueue (struct Modbus_FIFO_Errors *Modbus_FIFO_Ptr, 
                                     struct Modbus_FIFO_E_Item *Error);
unsigned char Modbus_FIFO_E_Dequeue (struct Modbus_FIFO_Errors *Modbus_FIFO_Ptr, 
                                     struct Modbus_FIFO_E_Item *Error);

#endif // __Modbus_FIFO_h