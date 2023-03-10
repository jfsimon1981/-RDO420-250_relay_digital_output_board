 /* BSD 3-Clause License
  * 
  * Copyright (c) 2023, Jean-François Simon
  * 
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted provided that the following conditions are met:
  * 
  * 1. Redistributions of source code must retain the above copyright notice, this
  *    list of conditions and the following disclaimer.
  * 
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 
  * 3. Neither the name of the copyright holder nor the names of its
  *    contributors may be used to endorse or promote products derived from
  *    this software without specific prior written permission.
  * 
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  */

#include <avr/io.h>
#include "program_io.h"
#include "program_time.h"
#include "program_util.h"
#include "definitions_relays.h"

// Includes for I2C
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include "usiTwiSlave.h"

// Define DEBUG and other flags as necessary
#ifdef DEBUG                    // Define in make -DDEBUG
#define TIMER0_PULSE 1          // Place LED on PA3 (SW1 on port)
#define CHECK_XTAL_FREQUENCY 0  // Runs infinite loop, pulse 1s
#define SHOW_LEDS_ON_STARTUP 0  // Briefly shows-up LEDs on startup (muxed with relays)
#endif

#define UNUSED(expr) do {(void)(expr);} while (0)

extern overflowState_t overflowState;

/*
 * Jumpers 1-3 define I2C address: 0x41/43/45/47
 * I2C exhange specs
 * Write relay number: 0x01 ... 0x04 to send a command to relay followed by
 * 0x00 for off or 0x01 for on. Terminate (0xff).
 */

class States {
  int mode = 1;
  public:
  int read_mode() {return mode;}
  int set_mode(int m_in) {
    mode = m_in;
    return 0;
  }
};

void init_timer0() {
  TCCR0A = (1 << CTC0); // 8 bits width, CTC mode
  TCCR0B = (1 << CS02) | (0 << CS01) | (1 << CS00); // Prescaler 101 = 1:1024
  OCR0A = 0x80;
  TIMSK = (1 << OCIE0A); // CompA Interrupt Enable
}

ISR (BADISR_vect) {
  while (1) {
    SET_LED();
    _delay_ms(310);
    CLR_LED();
    _delay_ms(310);
  }
}

#define I2C_TIMEOUT 3 // Unit is approx 1/100s

ISR (TIMER0_COMPA_vect) {
  static int i2c_timeout = 0;
  if (overflowState != USI_SLAVE_IDLE) {
    i2c_timeout++;
  } else {
    i2c_timeout = 0;
  }

  if (i2c_timeout > I2C_TIMEOUT) {
    usiTwiSlaveInit();
  }

#ifdef DEBUG
#if TIMER0_PULSE == 1
  static unsigned int led = 0;
  static unsigned int i = 0;
  if (!led) {
    if (i == 80) {
      i = 0;
      SET_LED();
      led = 1;
    }
  } else {
    if (i == 4) {
      i = 0;
      CLR_LED();
      led = 0;
    }
  }
  i++;
#endif
#endif
}

void program_loop() {
  States states;
#ifdef DEBUG
  INIT_LED();
#if SHOW_LEDS_ON_STARTUP == 1
  SET_LED();
  display_4bits(0xff);
  _delay_ms(503);
  CLR_LED();
  display_4bits(0x00);
#endif
#endif

#ifdef DEBUG
#if CHECK_XTAL_FREQUENCY == 1
  // Test time accuracy
  while (1) {
    PORTA ^= (1 << PA7);
    _delay_ms(500);
  }
#endif
#endif

  const uint8_t slave_address = 0x41; // I2C address

  usiTwiSlaveInit(slave_address);
  init_timer0();
  sei(); // interrupt enable

  // ************* Main program loop *************

  while (1) {

  #ifdef DEBUG

  // delme
  // if (overflowState == 0xff) SET_LED()
  display_4bits(overflowState);
  //

  // Shows up some data on LEDs
  if (0) {
    #define RS (USIDR >> 1) // Clock hold 1-0
    static uint16_t i = 0;
    if (i++ == 5000) {
    i = 0;
        char c = (RS & 0x0f);
        static char d = 0;
        if (c!=d)
          display_4bits(c);
        d = c;
    }
  }
  #endif

    if (usiTwiAmountDataInReceiveBuffer() >= 2) {
      char cmd   = 0; // Command
      char relay = 0; // Relay number
      char opt   = 0; // Optional
      UNUSED(opt);
      char rcv[3];
      for (int i = 0; i < 3; i++)
        rcv[i] = 0;
      int index = 0;
      // Receive
      while (usiTwiAmountDataInReceiveBuffer() > 0) {
        rcv[index++] = usiTwiReceiveByte();
        if (index == 3) {
          // set_error(TOO_MANY_DATA);
          break;
        }
      }
      // Decode
      cmd = rcv[0];
      relay = rcv[1];
      opt = rcv[2];

      if ((cmd == CMD_OPEN) && (relay == RELAY_K1)) {set_k1(0);}
      else if ((cmd == CMD_OPEN) && (relay == RELAY_K2)) {set_k2(0);}
      else if ((cmd == CMD_OPEN) && (relay == RELAY_K3)) {set_k3(0);}
      else if ((cmd == CMD_OPEN) && (relay == RELAY_K4)) {set_k4(0);}
      else if ((cmd == CMD_CLOSE) && (relay == RELAY_K1)) {set_k1(1);}
      else if ((cmd == CMD_CLOSE) && (relay == RELAY_K2)) {set_k2(1);}
      else if ((cmd == CMD_CLOSE) && (relay == RELAY_K3)) {set_k3(1);}
      else if ((cmd == CMD_CLOSE) && (relay == RELAY_K4)) {set_k4(1);}
      else if ((cmd == CMD_TOGGLE) && (relay == RELAY_K1)) {toggle_k1();}
      else if ((cmd == CMD_TOGGLE) && (relay == RELAY_K2)) {toggle_k2();}
      else if ((cmd == CMD_TOGGLE) && (relay == RELAY_K3)) {toggle_k3();}
      else if ((cmd == CMD_TOGGLE) && (relay == RELAY_K4)) {toggle_k4();}
      else {
        // set_error(UNKNOWN_REQUEST);
        usiTwiSlaveInit();
      }
      // usiTwiTransmitByte((uint8_t)v;

      // Flush I2C line
      while (usiTwiAmountDataInReceiveBuffer())
        usiTwiReceiveByte();

      // Return line to idle state
      overflowState = USI_SLAVE_IDLE;
    }

    #ifndef DEBUG
    /*
      // Mode 1 Direct
      set_k1(!read_sw1());
      set_k2(!read_sw2());
      set_k3(!read_sw3());
      set_k4(!read_sw4());
    */
    // Mode 2 Toggle

    if (states.read_mode() == 1) {
      bool sw1 = read_sw1();
      bool sw2 = read_sw2();
      bool sw3 = read_sw3();
      bool sw4 = read_sw4();

      if (!(sw1 & sw2 & sw3 & sw4)) {
        if (!sw1) {toggle_k1();}
        else if (!sw2) {toggle_k2();}
        else if (!sw3) {toggle_k3();}
        else if (!sw4) {toggle_k4();}
        delay_ms(50);
        while (!(sw1 & sw2 & sw3 & sw4)) {
          sw1 = read_sw1();
          sw2 = read_sw2();
          sw3 = read_sw3();
          sw4 = read_sw4();
        }
        delay_ms(150);
      }
    }
    #endif
  }
}



