// Author: Francisco Javier Guzman Jimenez, <dejavits@gmail.com>
//******************************************************************************
//! \defgroup FIFO Modbus FIFO
//! \brief Modbus FIFO Module
//!
//! In this Module is defined the structs used to store the information needed
//! in the requests in the "Request FIFO" and the structs for the "Error FIFO", the latter
//! is compound by a request identification and an exception message of two bytes from Modbus.
//!
//! Furthermore, it is included the functions to manipulate such FIFOs: initialization, add item,
//! remove item, empty/full checking functions and so on.
//******************************************************************************
//! @{

#include "Modbus_FIFO.h"

//*****************************************************************************
//
// FIFO Module functions
//
//*****************************************************************************

//! \brief Request FIFO Setup
//!
//! Number of Items is set to 0 and the head and tail are set at the beginning
//!because there are no petitions.
//!
//! \param *Modbus_FIFO_Ptr Request FIFO pointer
//! \sa struct Modbus_FIFO_s
void Modbus_FIFO_Init (struct Modbus_FIFO_s *Modbus_FIFO_Ptr)
{
  Modbus_FIFO_Ptr->Items = Modbus_FIFO_Ptr->Head = Modbus_FIFO_Ptr->Tail = 0;
}

//! \brief Check whether the Request FIFO is empty or not
//!
//! \param *Modbus_FIFO_Ptr Request FIFO pointer
//! \return 0 The FIFO is not empty
//! \return 1 The FIFO is empty
//! \sa struct Modbus_FIFO_s
unsigned char Modbus_FIFO_Empty (struct Modbus_FIFO_s *Modbus_FIFO_Ptr)
{
  unsigned char Res;

  Res = (Modbus_FIFO_Ptr->Items == 0);
  return Res;
}

//! \brief Check whether the Request FIFO is full or not
//!
//! \param *Modbus_FIFO_Ptr Request FIFO pointer
//! \return 0 The FIFO is not full
//! \return 1 The FIFO is full
//! \sa struct Modbus_FIFO_s
static unsigned char Modbus_FIFO_Full (struct Modbus_FIFO_s *Modbus_FIFO_Ptr)
{
  unsigned char Res;

  Res = (Modbus_FIFO_Ptr->Items >= MAX_ITEMS);
  return Res;
}

//! \brief Add one item/request to the Request FIFO
//!
//! A request is added to the Request FIFO. If the FIFO is full, such an action
//! is not done and it is returned the value 1.
//! \param *Modbus_FIFO_Ptr Request FIFO pointer
//! \param *Item Request FIFO item pointer
//! \return 0 No errors
//! \return 1 FIFO is full, item is not added
//! \sa struct Modbus_FIFO_s, struct Modbus_FIFO_Item, Modbus_FIFO_Full
unsigned char Modbus_FIFO_Enqueue (struct Modbus_FIFO_s *Modbus_FIFO_Ptr, struct Modbus_FIFO_Item *Item)
{	
  if (Modbus_FIFO_Full(Modbus_FIFO_Ptr))
    return 1;

  Modbus_FIFO_Ptr->Items++;
  Modbus_FIFO_Ptr->Buffer[Modbus_FIFO_Ptr->Head] = *Item;
  Modbus_FIFO_Ptr->Head = (Modbus_FIFO_Ptr->Head + 1) % MAX_ITEMS;
  return 0;
}

//! \brief Remove an item from the Request FIFO
//!
//! Remove an item/request from the Request FIFO and the item's information is set into an item struct.
//! If the FIFO is empty such an action is not done and it is returned 0.
//! \param *Modbus_FIFO_Ptr Request FIFO pointer
//! \param *Item Request FIFO item pointer
//! \return 0 FIFO was empty, item was not removed
//! \return 1 No errors
//! \sa struct Modbus_FIFO_s, struct Modbus_FIFO_Item, Modbus_FIFO_Empty
unsigned char Modbus_FIFO_Dequeue (struct Modbus_FIFO_s *Modbus_FIFO_Ptr, struct Modbus_FIFO_Item *Item)
{
  if (Modbus_FIFO_Empty(Modbus_FIFO_Ptr))
    return 0;  
	
  Modbus_FIFO_Ptr->Items--;

  *Item = Modbus_FIFO_Ptr->Buffer[Modbus_FIFO_Ptr->Tail];
  Modbus_FIFO_Ptr->Tail = (Modbus_FIFO_Ptr->Tail + 1) % MAX_ITEMS;
  return 1;
}

//! \brief Error FIFO Setup
//!
//! Number of items is set to 0 and the head and tail are set at the beginning
//! because there are not errors.
//! \param *Modbus_FIFO_Ptr Error FIFO pointer
//! \sa struct Modbus_FIFO_Errors
void Modbus_FIFO_E_Init (struct Modbus_FIFO_Errors *Modbus_FIFO_Ptr)
{
  Modbus_FIFO_Ptr->Items = Modbus_FIFO_Ptr->Head = Modbus_FIFO_Ptr->Tail = 0;
}

//! \brief Check whether the Error FIFO is empty or not.
//!
//! \param *Modbus_FIFO_Ptr Error FIFO pointer
//! \return 0 The FIFO is not emptyLa Cola no está vacía
//! \return 1 The FIFO is empty
//! \sa struct Modbus_FIFO_Errors
static unsigned char Modbus_FIFO_E_Empty (struct Modbus_FIFO_Errors *Modbus_FIFO_Ptr)
{
  unsigned char Res;
  
  Res = (Modbus_FIFO_Ptr->Items == 0);
  return Res;
}

//! \brief Check whether the Error FIFO is full or not.
//!
//! \param *Modbus_FIFO_Ptr Error FIFO pointer
//! \return 0 The FIFO is not full
//! \return 1 The FIFO is full
//! \sa struct Modbus_FIFO_Errors
static unsigned char Modbus_FIFO_E_Full (struct Modbus_FIFO_Errors *Modbus_FIFO_Ptr)
{
  unsigned char Res;

  Res = (Modbus_FIFO_Ptr->Items >= MAX_ITEMS);
  return Res;
}

//! \brief Add one item/error to the ERROR FIFO
//!
//! An error is added to the Error FIFO.If the FIFO is full, such an action is not done
//! and it is returned the value 1.
//! \param *Modbus_FIFO_Ptr Error FIFO pointer
//! \param *Error Error FIFO item pointer
//! \return 0 No errors
//! \return 1 The FIFO was full, item was not added
//! \sa struct Modbus_FIFO_Errors, struct Modbus_FIFO_E_Item, Modbus_FIFO_E_Full
unsigned char Modbus_FIFO_E_Enqueue (struct Modbus_FIFO_Errors *Modbus_FIFO_Ptr, 
                                     struct Modbus_FIFO_E_Item *Error)
{	
  if (Modbus_FIFO_E_Full(Modbus_FIFO_Ptr))
    return 1;

  Modbus_FIFO_Ptr->Items++;
  Modbus_FIFO_Ptr->Buffer[Modbus_FIFO_Ptr->Head] = *Error;
  Modbus_FIFO_Ptr->Head = (Modbus_FIFO_Ptr->Head + 1) % MAX_ITEMS;
  return 0;
}

//! \brief Remove an item/error from the Error FIFO
//!
//! Remove an item/error from the Error FIFO and the item's information is set into
//! an item struct. If the FIFO is empty such an action is not done and it is returned
//! the value 0.
//! \param *Modbus_FIFO_Ptr Error FIFO pointer
//! \param *Error Error FIFO item pointer
//! \return 0 FIFO was empty, item was not removed
//! \return 1 No Errors
//! \sa struct Modbus_FIFO_Errors, struct Modbus_FIFO_E_Item, Modbus_FIFO_E_Empty
unsigned char Modbus_FIFO_E_Dequeue (struct Modbus_FIFO_Errors *Modbus_FIFO_Ptr, 
                                     struct Modbus_FIFO_E_Item *Error)
{
  if (Modbus_FIFO_E_Empty(Modbus_FIFO_Ptr))
    return 0;  
	
  Modbus_FIFO_Ptr->Items--;

  *Error = Modbus_FIFO_Ptr->Buffer[Modbus_FIFO_Ptr->Tail];
  Modbus_FIFO_Ptr->Tail = (Modbus_FIFO_Ptr->Tail + 1) % MAX_ITEMS;
  return 1;
}
//! @}