 /*
  * BSD 3-Clause License
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

  /*
   * This Arduino template works includes the Olimex boards specific 
   * pin configuration. You may disable LCD_ENABLE or update to suit your needs.
   */

// Configuration

#define LCD_ENABLE 1

// Pins for Arduino board

#define UEXT_PWR_E      8 // External power to UEXT
#define USER_LED_GREEN  7 // User led: green
#define USER_LED_YELLOW 9 // User led: yellow

#include <SoftwareSerial.h>
#include <LCD1x9.h>
#include <Wire.h>
#include "definitions_relays.h"

uint8_t crc4(const uint8_t data, int len); // Make 2 CRC4 quadruplets from uint8_t[]
uint8_t crc4_from_frame(uint8_t command, uint8_t relay, uint8_t optional = 0);

#if LCD_ENABLE == 1
LCD1x9 lcd;
#endif
// SoftwareSerial serial (0, 1); // RX, TX

void interrupt_setup() {
  cli(); // Stop interrupts
  // Set timer1 interrupt at 1Hz
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  // OCR1A = 15624;// = (16*10^6) / (1*1024) - 1 (must be <65536)
  OCR1A = 624; // 25 Hz
  TCCR1B |= (1 << WGM12); // turn on CTC mode
  TCCR1B |= (1 << CS12) | (1 << CS10); // Set CS10 and CS12 bits for 1024 prescaler
  TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt
  sei(); // Allow interrupts
}

long unsigned int t1 {0};

ISR(TIMER1_COMPA_vect) {
  t1++;
  // Serial.print(".");
}

// the setup function runs once when you press reset or power the board
void setup() {
  // Digital
  pinMode(UEXT_PWR_E, OUTPUT);
  digitalWrite(UEXT_PWR_E, LOW); // Active low
  pinMode(USER_LED_GREEN, OUTPUT);
  pinMode(USER_LED_YELLOW, OUTPUT);

  // Demo boards addressed configuration
  // We close D0 and/or D1 to ground to send I2C open/close commands
  // to more than 1 board.

  pinMode(0, INPUT_PULLUP);
  pinMode(1, INPUT_PULLUP);

  // I2C, SDA,SCL
  Wire.begin(1);
  delay(3);
#if LCD_ENABLE == 1
  lcd.initialize();
#endif

  // Serial
  Serial.begin(115200);
//Serial1.begin(115200);
  Serial.setTimeout(100); // set new value to 100 milliseconds
  interrupt_setup();
}

/*
 * Set Relay address by DIP switch 1-2-3:
 * I2C addresses (8) 0x41/43/45/47/49/4b/4d/4f
 * Control via local push-buttons or I2C commands
 */

// Function to send an I2C command to a relay. Provide board address, command,
// relay number, leave optional to 0 if unused
int relay_command(uint8_t board_address, uint8_t command, uint8_t relay_num, uint8_t optional = 0) {
  uint8_t crc = crc4_from_frame(command, relay_num, optional);
  Wire.beginTransmission(board_address); // 0100 xxx1
  Wire.write(byte(command));   // Command (open, close, toggle, ...)
  Wire.write(byte(relay_num)); // Relays Kn
  Wire.write(byte(optional));  // Some commands have an optional data
  Wire.write(byte(crc));       // CRC
  Wire.endTransmission();
  return 0;
}

// Send a command to relay. Option data (set_pulse_duration)
void i2c_test_relay(uint8_t board_address, uint8_t relay_num);
// Demo programs
void demo_slow_motion_single_relay(uint8_t boards_addr_tested);
void demo_fast_trains(uint8_t boards_addr_tested);

  /* 
   * This demo program sends requests to 1, 2, 4 or 8 boards.
   * Configuration with pints D0-D1
   * To send open/close commands to only 1 board, leave D0-D1 open
   * D0-D1 Number of boards requests
   * 0  0  1 (0x41)
   * 0  1  2 (0x41 and 0x43)
   * 1  0  4 (0x41 to  0x47)
   * 1  1  8 (0x41 to  0x4F)
   */

void loop() {
  static uint8_t boards_addr_tested = 1;

  // On every loop we check the strap configuration, how many boards to send
  // I2C test packets to: 1, 2, 4 or 8 (close D0-D1 to GND)
  {
    int d0 = digitalRead(0);
    int d1 = digitalRead(1);

    if (d1 && d0) boards_addr_tested = 1;
    else if (d1 && !d0) boards_addr_tested = 2;
    else if (!d1 && d0) boards_addr_tested = 4;
    else if (!d1 && !d0) boards_addr_tested = 8;
  }

  // Send Demo program coil requests to 1 or more boards

  // demo_slow_motion_single_relay(boards_addr_tested);
  // demo_fast_trains(boards_addr_tested);

  // Test every coil for 1, 2, 4 or 8 relay boards 

  // Usage examples:
  // relay_command(RELAY_BOARD_1_ADDRESS, CMD_CLOSE, RELAY_K1);
  // relay_command(RELAY_BOARD_1_ADDRESS, CMD_OPEN, RELAY_K1);

  for (uint8_t boards_addr = 0; boards_addr < boards_addr_tested; boards_addr++) {
    uint8_t boards_i2c_addr = 0;
    if (boards_addr == 0) boards_i2c_addr = RELAY_BOARD_1_ADDRESS;
    else if (boards_addr == 1) boards_i2c_addr = RELAY_BOARD_2_ADDRESS;
    else if (boards_addr == 2) boards_i2c_addr = RELAY_BOARD_3_ADDRESS;
    else if (boards_addr == 3) boards_i2c_addr = RELAY_BOARD_4_ADDRESS;
    else if (boards_addr == 4) boards_i2c_addr = RELAY_BOARD_5_ADDRESS;
    else if (boards_addr == 5) boards_i2c_addr = RELAY_BOARD_6_ADDRESS;
    else if (boards_addr == 6) boards_i2c_addr = RELAY_BOARD_7_ADDRESS;
    else if (boards_addr == 7) boards_i2c_addr = RELAY_BOARD_8_ADDRESS;

    i2c_test_relay(boards_i2c_addr, RELAY_K1); // Toggle K1
    i2c_test_relay(boards_i2c_addr, RELAY_K2); // Toggle K2
    i2c_test_relay(boards_i2c_addr, RELAY_K3); // Toggle K3
    i2c_test_relay(boards_i2c_addr, RELAY_K4); // Toggle K4
  }

  // Green
  digitalWrite(USER_LED_GREEN, HIGH);
  digitalWrite(USER_LED_YELLOW, LOW);
#if LCD_ENABLE == 1
  lcd.write((char*)("Demo"));
#endif
  Serial.println("Demo");
//Serial1.println("Green");
  delay(1319);
  // Yellow
  digitalWrite(USER_LED_GREEN, LOW);
  digitalWrite(USER_LED_YELLOW, HIGH);
#if LCD_ENABLE == 1
  lcd.write((char*)("Program"));
#endif
  Serial.print("Program (timer ");
//Serial1.println("Yellow");
  Serial.print(t1, DEC);
  Serial.println(")");

  delay(809);
}

// ************* Relay toggle on/off test *************

// Utility to send close then open commands to toggle a relay
// Parameters are board address and relay number
//  RELAY_BOARD_1_ADDRESS ... RELAY_BOARD_8_ADDRESS
//  RELAY_K1 ... RELAY_K4

void i2c_test_relay(uint8_t board_address, uint8_t relay_num) {
  Serial.print(relay_num, HEX);
  // Close relay
  relay_command(board_address, CMD_CLOSE, relay_num);
  delay(370);
  // Open relay
  relay_command(board_address, CMD_OPEN, relay_num);
  delay(370);
}

// ************* CRC4 util *************

static const uint8_t crc4_lookup[16] = {
		0x00, 0x03, 0x06, 0x05, 0x0c, 0x0f, 0x0a, 0x09,
		0x08, 0x0b, 0x0e, 0x0d, 0x04, 0x07, 0x02, 0x01
};

uint8_t crc4(const uint8_t *d, int l) {
	uint8_t crc_h = 0, crc_l = 0;
	for (int i = 0; i < l; i++) {
		crc_l = crc4_lookup[(crc_l ^ d[i]) & 0x0f] ^ (crc_l >> 4);
		crc_h = crc4_lookup[(crc_h ^ (d[i] >> 4)) & 0x0f] ^ (crc_h >> 4);
	}
	return ((crc_h << 4) | crc_l);
}

uint8_t crc4_from_frame(uint8_t command, uint8_t relay, uint8_t optional) {
  uint8_t crc4_d[3]; // Every frame to the board is 3 data packets length + crc
  crc4_d[0] = command;
  crc4_d[1] = relay;
  crc4_d[2] = optional;
  uint8_t rv = crc4(crc4_d, 3);
  return rv;
}