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

// Configuration

#define LCD_ENABLE 1

// Pins for Arduino board

#define UEXT_PWR_E      8 // External power to UEXT
#define USER_LED_GREEN  7 // User led: green
#define USER_LED_YELLOW 9 // User led: yellow

#include <SoftwareSerial.h>
#include <LCD1x9.h>
#include <Wire.h>

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
  Serial.print(".");
}

// the setup function runs once when you press reset or power the board
void setup() {
  // Digital
  pinMode(UEXT_PWR_E, OUTPUT);
  digitalWrite(UEXT_PWR_E, LOW); // Active low
  pinMode(USER_LED_GREEN, OUTPUT);
  pinMode(USER_LED_YELLOW, OUTPUT);

  // I2C, SDA,SCL
  Wire.begin(1);
  delay(3);
#if LCD_ENABLE == 1
  lcd.initialize();
#endif

  // Serial
  Serial.begin(115200);
//Serial1.begin(115200);
  while (!Serial);
//while (!Serial1);

  interrupt_setup();
}

/*
 * Set Relay I2C address by DIP switch 1-2-3 to for 0x41/43/45/47
 * Control via local push-buttons or I2C commands
 */

// Relay address

#define RELAY_BOARD_1_ADDRESS 0x41

// Relays number

#define RELAY_K1                1   // Designator for relay K1
#define RELAY_K2                2   // Designator for relay K2
#define RELAY_K3                3   // Designator for relay K3
#define RELAY_K4                4   // Designator for relay K4 

// Main control commands

#define CMD_OPEN                1   // Open a relay. pass relay number in param. No option.
#define CMD_CLOSE               2   // Close a relay. pass relay number in param. No option.
#define CMD_TOGGLE              3   // Toggle a relay. pass relay number in param. No option.
#define CMD_CLOSE_PULSE         4   // Close a relay. pass relay number in param. Pulse duration in opt or leave 0 for default.
#define CMD_OPEN_PULSE          5   // Open a relay. pass relay number in param. Pulse duration in opt or leave 0 for default.
#define CMD_CLOSE_ALL_RELAYS    6   // Close all relays. No option.
#define CMD_OPEN_ALL_RELAYS     7   // Open all relays. No option.

// Configuration and emergency off

#define SET_PULSE_DURATION      8   // Set pulse duration in seconds (1 ... 4294967296)
#define SET_ENABLE_LOCAL_CTRL   9   // Enable coil open/close from board. No option.
#define SET_DISABLE_LOCAL_CTRL  10  // Disable coil open/close from board. No option.
#define SET_ENABLE_REMOTE_CTRL  11  // Enable coil open/close from remote. No option.
#define SET_DISABLE_REMOTE_CTRL 12  // Disable coil open/close from remote. No option.
#define SET_EMERGENCY_OFF       13  // Open all coils and locks board. Requires local reset to restart operation.

// Read-back

#define READ_RELAY_POSITION     23  // Reads relay position (1 for open, 2 for close)
#define READ_STATUS             24  // Reads board status
#define READ_PORT               25  // Reads board port (PA7-0 k1 k2 k3 k4 s1 s2 s3 s4)

#define RELAY_IS_OPEN           1   // Returned value if coil is open.
#define RELAY_IS_CLOSED         2   // Returned value if coil is closed.

// Status

#define STATUS_BITS_AVAILABLE   0   // Board in available
#define STATUS_BITS_COM_ERROR   1   // A serial communication error occured
#define STATUS_BITS_LOC_CTRL_EN 2   // Local control enabled
#define STATUS_BITS_REM_CTRL_EN 3   // Remote control enabled
#define STATUS_BITS_IS_EOFF     4   // Is in Emergency Off state (local reset required)

// Send a command to relay. Option data (set_pulse_duration)
void i2c_test_relay(uint8_t relay_num) {
  Serial.print(relay_num, HEX);

  // Close relay
  Wire.beginTransmission(RELAY_BOARD_1_ADDRESS); // 0100 xxx1
  Wire.write(byte(CMD_CLOSE));  // Close
  Wire.write(byte(relay_num));  // Relays Kn
  Wire.endTransmission();
  delay(370);

  // Open relay
  Wire.beginTransmission(RELAY_BOARD_1_ADDRESS); // 0100 xxx1
  Wire.write(byte(CMD_OPEN));   // Open
  Wire.write(byte(relay_num));  // Relays Kn
  Wire.endTransmission();
  delay(370);
}

void loop() {
  i2c_test_relay(RELAY_K1); // Toggle K1
  i2c_test_relay(RELAY_K2); // Toggle K2
  i2c_test_relay(RELAY_K3); // Toggle K3
  i2c_test_relay(RELAY_K4); // Toggle K4
  // Green
  digitalWrite(USER_LED_GREEN, HIGH);
  digitalWrite(USER_LED_YELLOW, LOW);
#if LCD_ENABLE == 1
  lcd.write((char*)("Green"));
#endif
  Serial.println("Green");
//Serial1.println("Green");
  delay(1319);
  // Yellow
  digitalWrite(USER_LED_GREEN, LOW);
  digitalWrite(USER_LED_YELLOW, HIGH);
#if LCD_ENABLE == 1
  lcd.write((char*)("Yellow"));
#endif
  Serial.print("Yellow (timer ");
//Serial1.println("Yellow");
  Serial.print(t1, DEC);
  Serial.println(")");

  delay(809);
  // LCD
}

