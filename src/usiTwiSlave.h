/********************************************************************************

Header file for the USI TWI Slave driver.

Created by Donald R. Blake
donblake at worldnet.att.net

---------------------------------------------------------------------------------

Created from Atmel source files for Application Note AVR312: Using the USI Module
as an I2C slave.

This program is free software; you can redistribute it and/or modify it under the
terms of the GNU General Public License as published by the Free Software
Foundation; either version 2 of the License, or (at your option) any later
version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE.  See the GNU General Public License for more details.

---------------------------------------------------------------------------------

Change Activity:

    Date       Description
   ------      -------------
  15 Mar 2007  Created.

********************************************************************************/

#ifndef _USI_TWI_SLAVE_H_
#define _USI_TWI_SLAVE_H_

/********************************************************************************

                                    includes

********************************************************************************/

#include <stdbool.h>

/********************************************************************************

                                   prototypes

********************************************************************************/

void    usiTwiSlaveInit();
void    usiTwiSlaveInit(uint8_t);
void    usiTwiTransmitByte(uint8_t);
uint8_t usiTwiReceiveByte(void);
// bool    usiTwiDataInReceiveBuffer(void); // Deleted
//void    (*_onTwiDataRequest)(void);
bool    usiTwiDataInTransmitBuffer(void);
uint8_t usiTwiAmountDataInReceiveBuffer(void);
// on_XXX handler pointers
//void    (*usi_onRequestPtr)(void);
//void    (*usi_onReceiverPtr)(uint8_t);

void usi_twi_set_state_idle();

/********************************************************************************

                           driver buffer definitions

********************************************************************************/

// permitted RX buffer sizes: 1, 2, 4, 8, 16, 32, 64, 128 or 256

#ifndef TWI_RX_BUFFER_SIZE
#define TWI_RX_BUFFER_SIZE  ( 16 )
#endif
#define TWI_RX_BUFFER_MASK  ( TWI_RX_BUFFER_SIZE - 1 )

#if ( TWI_RX_BUFFER_SIZE & TWI_RX_BUFFER_MASK )
#  error TWI RX buffer size is not a power of 2
#endif

// permitted TX buffer sizes: 1, 2, 4, 8, 16, 32, 64, 128 or 256

#ifndef TWI_TX_BUFFER_SIZE
#define TWI_TX_BUFFER_SIZE ( 16 )
#endif
#define TWI_TX_BUFFER_MASK ( TWI_TX_BUFFER_SIZE - 1 )

#if ( TWI_TX_BUFFER_SIZE & TWI_TX_BUFFER_MASK )
#  error TWI TX buffer size is not a power of 2
#endif

// I2C states used within driver and main program

typedef enum {
  USI_SLAVE_CHECK_ADDRESS                = 0x00,
  USI_SLAVE_SEND_DATA                    = 0x01,
  USI_SLAVE_REQUEST_REPLY_FROM_SEND_DATA = 0x02,
  USI_SLAVE_CHECK_REPLY_FROM_SEND_DATA   = 0x03,
  USI_SLAVE_REQUEST_DATA                 = 0x04,
  USI_SLAVE_GET_DATA_AND_SEND_ACK        = 0x05,
  USI_SLAVE_ERROR                        = 0x80,
  USI_SLAVE_IDLE                         = 0xff
} overflowState_t;

#endif  // ifndef _USI_TWI_SLAVE_H_
