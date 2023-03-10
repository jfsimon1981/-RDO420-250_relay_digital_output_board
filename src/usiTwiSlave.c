/********************************************************************************

USI TWI Slave driver.

Created by Donald R. Blake. donblake at worldnet.att.net
Adapted by Jochen Toppe, jochen.toppe at jtoee.com

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
  16 Mar 2007  Created.
  27 Mar 2007  Added support for ATtiny261, 461 and 861.
  26 Apr 2007  Fixed ACK of slave address on a read.
  04 Jul 2007  Fixed USISIF in ATtiny45 def
  12 Dev 2009  Added callback functions for data requests
  06 Feb 2015  Minor change to allow mutli-byte requestFrom() from master.
  10 Feb 2015  Simplied RX/TX buffer code and allowed use of full buffer.
  12 Dec 2016  Added support for ATtiny167

********************************************************************************/


/********************************************************************************
                                    includes
********************************************************************************/

#include <avr/io.h>
#include <avr/interrupt.h>
#include "environment.h"

#include "usiTwiSlave.h"
//#include "../common/util.h"

#ifdef DEBUG
#include <util/delay.h>
#endif

#include "program_util.h"

void    (*_onTwiDataRequest)(void);
void    (*usi_onRequestPtr)(void);
void    (*usi_onReceiverPtr)(uint8_t);

/********************************************************************************
                            device dependent defines
********************************************************************************/

#if defined( __AVR_ATtiny167__ )
#  define DDR_USI             DDRB
#  define PORT_USI            PORTB
#  define PIN_USI             PINB
#  define PORT_USI_SDA        PB0
#  define PORT_USI_SCL        PB2
#  define PIN_USI_SDA         PINB0
#  define PIN_USI_SCL         PINB2
#  define USI_START_COND_INT  USISIF
#  define USI_START_VECTOR    USI_START_vect
#  define USI_OVERFLOW_VECTOR USI_OVERFLOW_vect
#endif

#if defined( __AVR_ATtiny2313__ )
#  define DDR_USI             DDRB
#  define PORT_USI            PORTB
#  define PIN_USI             PINB
#  define PORT_USI_SDA        PB5
#  define PORT_USI_SCL        PB7
#  define PIN_USI_SDA         PINB5
#  define PIN_USI_SCL         PINB7
#  define USI_START_COND_INT  USISIF
#  define USI_START_VECTOR    USI_START_vect
#  define USI_OVERFLOW_VECTOR USI_OVERFLOW_vect
#endif

#if defined(__AVR_ATtiny84__) | \
     defined(__AVR_ATtiny44__)
#  define DDR_USI             DDRA
#  define PORT_USI            PORTA
#  define PIN_USI             PINA
#  define PORT_USI_SDA        PORTA6
#  define PORT_USI_SCL        PORTA4
#  define PIN_USI_SDA         PINA6
#  define PIN_USI_SCL         PINA4
#  define USI_START_COND_INT  USISIF
#  define USI_START_VECTOR    USI_START_vect
#  define USI_OVERFLOW_VECTOR USI_OVF_vect
#endif

#if defined( __AVR_ATtiny25__ ) | \
     defined( __AVR_ATtiny45__ ) | \
     defined( __AVR_ATtiny85__ )
#  define DDR_USI             DDRB
#  define PORT_USI            PORTB
#  define PIN_USI             PINB
#  define PORT_USI_SDA        PB0
#  define PORT_USI_SCL        PB2
#  define PIN_USI_SDA         PINB0
#  define PIN_USI_SCL         PINB2
#  define USI_START_COND_INT  USISIF
#  define USI_START_VECTOR    USI_START_vect
#  define USI_OVERFLOW_VECTOR USI_OVF_vect
#endif

#if defined( __AVR_ATtiny26__ )
#  define DDR_USI             DDRB
#  define PORT_USI            PORTB
#  define PIN_USI             PINB
#  define PORT_USI_SDA        PB0
#  define PORT_USI_SCL        PB2
#  define PIN_USI_SDA         PINB0
#  define PIN_USI_SCL         PINB2
#  define USI_START_COND_INT  USISIF
#  define USI_START_VECTOR    USI_STRT_vect
#  define USI_OVERFLOW_VECTOR USI_OVF_vect
#endif

#if defined( __AVR_ATtiny261__ ) | \
      defined( __AVR_ATtiny461__ ) | \
      defined( __AVR_ATtiny861__ )
#  define DDR_USI             DDRB
#  define PORT_USI            PORTB
#  define PIN_USI             PINB
#  define PORT_USI_SDA        PB0
#  define PORT_USI_SCL        PB2
#  define PIN_USI_SDA         PINB0
#  define PIN_USI_SCL         PINB2
#  define USI_START_COND_INT  USISIF
#  define USI_START_VECTOR    USI_START_vect
#  define USI_OVERFLOW_VECTOR USI_OVF_vect
#endif

#if defined( __AVR_ATmega165__ ) | \
     defined( __AVR_ATmega325__ ) | \
     defined( __AVR_ATmega3250__ ) | \
     defined( __AVR_ATmega645__ ) | \
     defined( __AVR_ATmega6450__ ) | \
     defined( __AVR_ATmega329__ ) | \
     defined( __AVR_ATmega3290__ )
#  define DDR_USI             DDRE
#  define PORT_USI            PORTE
#  define PIN_USI             PINE
#  define PORT_USI_SDA        PE5
#  define PORT_USI_SCL        PE4
#  define PIN_USI_SDA         PINE5
#  define PIN_USI_SCL         PINE4
#  define USI_START_COND_INT  USISIF
#  define USI_START_VECTOR    USI_START_vect
#  define USI_OVERFLOW_VECTOR USI_OVERFLOW_vect
#endif

#if defined( __AVR_ATmega169__ )
#  define DDR_USI             DDRE
#  define PORT_USI            PORTE
#  define PIN_USI             PINE
#  define PORT_USI_SDA        PE5
#  define PORT_USI_SCL        PE4
#  define PIN_USI_SDA         PINE5
#  define PIN_USI_SCL         PINE4
#  define USI_START_COND_INT  USISIF
#  define USI_START_VECTOR    USI_START_vect
#  define USI_OVERFLOW_VECTOR USI_OVERFLOW_vect
#endif

/********************************************************************************

                        functions implemented as macros

********************************************************************************/

#define SET_USI_TO_SEND_ACK() { \
  /* prepare ACK */ \
  USIDR = 0; \
  /* set SDA as output */ \
  DDR_USI |= (1 << PORT_USI_SDA); \
  /* clear all interrupt flags, except Start Cond */ \
  USISR = \
       (0 << USI_START_COND_INT) | \
       (1 << USIOIF) | (1 << USIPF) | \
       (1 << USIDC)| \
       /* set USI counter to shift 1 bit */ \
       (0x0E << USICNT0); \
}

#define SET_USI_TO_READ_ACK() { \
  /* set SDA as input */ \
  DDR_USI &= ~(1 << PORT_USI_SDA); \
  /* prepare ACK */ \
  USIDR = 0; \
  /* clear all interrupt flags, except Start Cond */ \
  USISR = \
       (0 << USI_START_COND_INT) | \
       (1 << USIOIF) | \
       (1 << USIPF) | \
       (1 << USIDC) | \
       /* set USI counter to shift 1 bit */ \
       (0x0E << USICNT0); \
}

#define SET_USI_TO_TWI_START_CONDITION_MODE() { \
  USICR = \
       /* enable Start Condition Interrupt, disable Overflow Interrupt */ \
       (1 << USISIE) | (0 << USIOIE) | \
       /* set USI in Two-wire mode, no USI Counter overflow hold */ \
       (1 << USIWM1) | (0 << USIWM0) | \
       /* Shift Register Clock Source = External, positive edge */ \
       /* 4-Bit Counter Source = external, both edges */ \
       (1 << USICS1) | (0 << USICS0) | (0 << USICLK) | \
       /* no toggle clock-port pin */ \
       (0 << USITC); \
  USISR = \
        /* clear all interrupt flags, except Start Cond */ \
        (0 << USI_START_COND_INT) | (1 << USIOIF) | (1 << USIPF) | \
        (1 << USIDC) | (0x0 << USICNT0 ); \
}

#define SET_USI_TO_SEND_DATA() { \
  /* set SDA as output */ \
  DDR_USI |=  (1 << PORT_USI_SDA); \
  /* clear all interrupt flags, except Start Cond */ \
  USISR    =  \
       (0 << USI_START_COND_INT) | (1 << USIOIF) | (1 << USIPF) | \
       (1 << USIDC) | \
       /* set USI to shift out 8 bits */ \
       (0x0 << USICNT0); \
}

#define SET_USI_TO_READ_DATA() { \
  /* set SDA as input */ \
  DDR_USI &= ~(1 << PORT_USI_SDA); \
  /* clear all interrupt flags, except Start Cond */ \
  USISR    = \
       (0 << USI_START_COND_INT) | (1 << USIOIF) | \
       (1 << USIPF) | (1 << USIDC) | \
       /* set USI to shift out 8 bits */ \
       (0x0 << USICNT0); \
}

#define USI_RECEIVE_CALLBACK() { \
    if (usi_onReceiverPtr) { \
        if (usiTwiAmountDataInReceiveBuffer()) { \
            usi_onReceiverPtr(usiTwiAmountDataInReceiveBuffer()); \
        } \
    } \
}

#define ONSTOP_USI_RECEIVE_CALLBACK() { \
    if (USISR & (1 << USIPF)) { \
        USI_RECEIVE_CALLBACK(); \
    } \
}


#define USI_REQUEST_CALLBACK() { \
    USI_RECEIVE_CALLBACK(); \
    if(usi_onRequestPtr) usi_onRequestPtr(); \
}


/********************************************************************************

                                local variables

********************************************************************************/

static uint8_t                  slaveAddress;
/*static*/ volatile overflowState_t overflowState; // JFS temp to read from program.cpp


static uint8_t          rxBuf[ TWI_RX_BUFFER_SIZE ];
static volatile uint8_t rxHead;
static volatile uint8_t rxTail;
static volatile uint8_t rxCount;

static uint8_t          txBuf[ TWI_TX_BUFFER_SIZE ];
static volatile uint8_t txHead;
static volatile uint8_t txTail;
static volatile uint8_t txCount;

/********************************************************************************

                                local functions

********************************************************************************/

// flushes the TWI buffers

static void flushTwiBuffers (void) {
  rxTail =  0;
  rxHead =  0;
  rxCount = 0;
  txTail =  0;
  txHead =  0;
  txCount = 0;
} // end flushTwiBuffers


/********************************************************************************

                                public functions

********************************************************************************/

// initialise USI for TWI slave mode

void usiTwiSlaveInit() {usiTwiSlaveInit(slaveAddress);}

void usiTwiSlaveInit (uint8_t ownAddress) {

  flushTwiBuffers();
  slaveAddress = ownAddress;
  overflowState = USI_SLAVE_IDLE;

  // In Two Wire mode (USIWM1, USIWM0 = 1X), the slave USI will pull SCL
  // low when a start condition is detected or a counter overflow (only
  // for USIWM1, USIWM0 = 11).  This inserts a wait state.  SCL is released
  // by the ISRs (USI_START_vect and USI_OVERFLOW_vect).

  DDR_USI  |= (1 << PORT_USI_SCL) | (1 << PORT_USI_SDA);  // Set SCL and SDA as output
  PORT_USI |= (1 << PORT_USI_SCL);                        // set SCL high
  PORT_USI |= (1 << PORT_USI_SDA);                        // set SDA high
  DDR_USI &= ~(1 << PORT_USI_SDA);                        // Set SDA as input

  // enable Start Condition Interrupt
  // disable Overflow Interrupt
  // set USI in Two-wire mode, no USI Counter overflow hold
  // Shift Register Clock Source = external, positive edge
  // 4-Bit Counter Source = external, both edges
  // no toggle clock-port pin

  USICR = (1 << USISIE) | (0 << USIOIE) | (1 << USIWM1) | (0 << USIWM0) \
        | (1 << USICS1) | (0 << USICS0) | (0 << USICLK) | (0 << USITC);

  // clear all interrupt flags and reset overflow counter
  USISR = (1 << USI_START_COND_INT) | (1 << USIOIF) | (1 << USIPF) | (1 << USIDC);
}


bool usiTwiDataInTransmitBuffer(void) {
  // return 0 (false) if the receive buffer is empty
  return txCount;
}

// put data in the transmission buffer, wait if buffer is full
void usiTwiTransmitByte (uint8_t data) {
//uint8_t tmphead;
  // wait for free space in buffer
  while (txCount == TWI_TX_BUFFER_SIZE) ;
  // store data in buffer
  txBuf[txHead] = data;
  txHead = (txHead + 1) & TWI_TX_BUFFER_MASK;
  txCount++;
}

// return a byte from the receive buffer, wait if buffer is empty
uint8_t usiTwiReceiveByte(void) {
  uint8_t rv;
  // wait for Rx data
  while (!rxCount);

  // TODO Implement timeout

  rv = rxBuf[rxTail];
  rxBuf[rxTail] = 0;
  // calculate buffer index
  rxTail = (rxTail + 1) & TWI_RX_BUFFER_MASK;
  rxCount--;
  return rv;
}

uint8_t usiTwiAmountDataInReceiveBuffer(void) {return rxCount;}

  /*
   * I2C transitions
   * 
   * State sda/scl
   * Idle     11(i)
   * Start    11(i) -> 01(s) -> ... (d) or (r)
   * Stop     01(d) -> 11(i) -> ... (s)
   * 
   * Legend
   * i Idle     Pull-up resistors
   * s Start    Falling edge on SDA with SCL high
   * d Data     Sampled on SCL rising edge
   * t Stop     Rising edge on SDA with SCL high
   * r Restart  
   * a Ack      Active low,  receiver keeps SDA at 0
   * n Nack     Active high, receiver keeps SDA at 1
   */

/********************************************************************************

                            USI Start Condition ISR

********************************************************************************/

ISR (USI_START_VECTOR) {
  /*
  // This triggers on second write, but claims to the callback there is only *one* byte in buffer
  ONSTOP_USI_RECEIVE_CALLBACK();
  */
  /*
  // This triggers on second write, but claims to the callback there is only *one* byte in buffer
  USI_RECEIVE_CALLBACK();
  */

  overflowState = USI_SLAVE_CHECK_ADDRESS; // set default starting conditions for new TWI package
  DDR_USI &= ~(1 << PORT_USI_SDA);         // set SDA as input

  // wait for SCL to go low to ensure the Start Condition has completed (the
  // start detector will hold SCL low ) - if a Stop Condition arises then leave
  // the interrupt to prevent waiting forever - don't use USISR to test for Stop
  // Condition as in Application Note AVR312 because the Stop Condition Flag is
  // going to be set from the last TWI sequence

  // Wait until start sequence finished (SDA low and SCL high)
  while (!((PIN_USI & (1 << PIN_USI_SDA))) && (PIN_USI & (1 << PIN_USI_SCL)));

  if (!(PIN_USI & (1 << PIN_USI_SDA))) {
    // Start condition detected -> enable USI Data (ovfl) INTR
    // keep Start Condition Interrupt enabled to detect RESTART
    // enable Overflow Interrupt
    // set USI in Two-wire mode, hold SCL low on USI Counter overflow (ACK)
    // Shift Register Clock Source = External, positive edge
    // 4-Bit Counter Source = external, both edges, do not toggle clock-port pin

  USICR = (1 << USISIE) | (1 << USIOIE) | (1 << USIWM1) | (1 << USIWM0) \
          | (1 << USICS1) | (0 << USICS0) | (0 << USICLK) | (0 << USITC);
  } else {
    overflowState = USI_SLAVE_IDLE;
    // Stop condition detected -> disable USI Data (ovfl) INTR
    // enable Start Condition Interrupt
    // disable Overflow Interrupt
    // set USI in Two-wire mode, no USI Counter overflow hold
    // Shift Register Clock Source = external, positive edge
    // 4-Bit Counter Source = external, both edges, do not toggle clock-port pin

  USICR = (1 << USISIE) | (0 << USIOIE) | (1 << USIWM1) | (0 << USIWM0) \
          | (1 << USICS1) | (0 << USICS0) | (0 << USICLK) | (0 << USITC);
  }

  // clear interrupt flags - resetting the Start Condition Flag will
  // release SCL
  // set USI to sample 8 bits (count 16 external SCL pin toggles)

  USISR = (1 << USI_START_COND_INT) | (1 << USIOIF) | (1 << USIPF) \
        | (1 << USIDC) | (0x0 << USICNT0);
}

/********************************************************************************

                                USI Overflow ISR

Handles all the communication.

Only disabled when waiting for a new Start Condition.

********************************************************************************/

ISR (USI_OVERFLOW_VECTOR) {
  switch (overflowState) {
    // Address mode: check address and send ACK (and next USI_SLAVE_SEND_DATA) if OK,
    // else reset USI
    case USI_SLAVE_CHECK_ADDRESS:

      if ((USIDR == 0) || ((USIDR >> 1) == slaveAddress)) {
        if (USIDR & 0x01) {
          USI_REQUEST_CALLBACK();
          overflowState = USI_SLAVE_SEND_DATA;
        } else {
          overflowState = USI_SLAVE_REQUEST_DATA;
        }
        SET_USI_TO_SEND_ACK();
      } else {
        SET_USI_TO_TWI_START_CONDITION_MODE();
        overflowState = USI_SLAVE_IDLE;
      }
      break;

    // Master write data mode: check reply and goto USI_SLAVE_SEND_DATA if OK,
    // else reset USI
    case USI_SLAVE_CHECK_REPLY_FROM_SEND_DATA:
      if (USIDR) {
        // if NACK, the master does not want more data
        SET_USI_TO_TWI_START_CONDITION_MODE();
overflowState = USI_SLAVE_IDLE;
        return;
      }
      // from here we just drop straight into USI_SLAVE_SEND_DATA if the
      // master sent an ACK

    // copy data from buffer to USIDR and set USI to shift byte
    // next USI_SLAVE_REQUEST_REPLY_FROM_SEND_DATA
    case USI_SLAVE_SEND_DATA:
      // Get data from Buffer
      if (txCount) {
        USIDR = txBuf[txTail];
        txTail = (txTail + 1) & TWI_TX_BUFFER_MASK;
        txCount--;
      } else {
        // the buffer is empty
        SET_USI_TO_READ_ACK(); // This might be necessary sometimes Ref http://www.avrfreaks.net/index.php?name=PNphpBB2&file=viewtopic&p=805227#805227
        SET_USI_TO_TWI_START_CONDITION_MODE();
        return;
      }
      overflowState = USI_SLAVE_REQUEST_REPLY_FROM_SEND_DATA;
      SET_USI_TO_SEND_DATA();
      break;

    // set USI to sample reply from master
    // next USI_SLAVE_CHECK_REPLY_FROM_SEND_DATA
    case USI_SLAVE_REQUEST_REPLY_FROM_SEND_DATA:
      overflowState = USI_SLAVE_CHECK_REPLY_FROM_SEND_DATA;
      SET_USI_TO_READ_ACK();
      break;

    // Master read data mode: set USI to sample data from master, next
    // USI_SLAVE_GET_DATA_AND_SEND_ACK
    case USI_SLAVE_REQUEST_DATA:
      overflowState = USI_SLAVE_GET_DATA_AND_SEND_ACK;
      SET_USI_TO_READ_DATA();
      break;

    // copy data from USIDR and send ACK
    // next USI_SLAVE_REQUEST_DATA
    case USI_SLAVE_GET_DATA_AND_SEND_ACK:
      // put data into buffer
      // check buffer size
      if (rxCount < TWI_RX_BUFFER_SIZE) {
        rxBuf[rxHead] = USIDR;
        rxHead = (rxHead + 1) & TWI_RX_BUFFER_MASK;
        rxCount++;
      } else {
        // overrun
        // drop data
      }
      // next USI_SLAVE_REQUEST_DATA
      overflowState = USI_SLAVE_REQUEST_DATA;
      SET_USI_TO_SEND_ACK();
      break;
    case USI_SLAVE_IDLE:
      // This is an error, an overflow should occur after a start condition
      // Reinit
      usiTwiSlaveInit(slaveAddress);
      overflowState = USI_SLAVE_ERROR;
      break;
    case USI_SLAVE_ERROR:
      break;
    default:
      overflowState = USI_SLAVE_ERROR;
  }
}


void usi_twi_set_state_idle() {
  overflowState = USI_SLAVE_IDLE;
}

uint8_t usi_twi_buffer_data(void) {
  return rxBuf[rxTail];
}