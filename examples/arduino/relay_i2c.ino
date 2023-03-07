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

#define UEXT_PWR_E      8 // External power to UEXT
#define USER_LED_GREEN  7 // User led: green
#define USER_LED_YELLOW 9 // User led: yellow

#include <SoftwareSerial.h>
#include <LCD1x9.h>
#include <Wire.h>

class chrys {
  public:
    // Accessors for peripheral relay at port.num
    void relay_close(int port, int num); // Close coil loop
    void relay_open(int port, int num);  // Open coil loop
    int relay_state(int port, int num);  // Returns state
};

#ifdef LCD_ENABLE
LCD1x9 lcd;
#endif
// SoftwareSerial serial (0, 1); // RX, TX

void int_setup(){

cli();//stop interrupts
//set timer1 interrupt at 1Hz
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
//OCR1A = 15624;// = (16*10^6) / (1*1024) - 1 (must be <65536)
  OCR1A = 624; // 25 Hz
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS10 and CS12 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);

sei();//allow interrupts
}

long unsigned int t1 {0};
//timer1 interrupt 1Hz toggles pin 13 (LED)
//generates pulse wave of frequency 1Hz/2 = 0.5kHz
// (takes two cycles for full wave- toggle high then toggle low)
ISR(TIMER1_COMPA_vect) {
  t1++;
  Serial.print(".");
}

/*
  // Delme
              Pins Attiny461
    Bleu    a 2
    Blanc   b 3
    Jaune   c 10
    Vert    d 1............
*/

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
#ifdef LCD_ENABLE
  lcd.initialize();
#endif

  // Serial
  Serial.begin(115200);
//Serial1.begin(115200);
  while (!Serial);
//while (!Serial1);

  int_setup();
}

void i2c_send(uint8_t relay) {
  Serial.print(relay, HEX);
  // Close relay
  Wire.beginTransmission(0x41); // 0100 xxx1
  Wire.write(byte(relay)); // Relays K1
  Wire.write(byte(0x01)); // Close
  Wire.write(byte(0xff)); // Terminate
  Wire.endTransmission();
  delay(200);

  // Open relay
  Wire.beginTransmission(0x41); // 0100 xxx1
  Wire.write(byte(relay)); // Relays K1
  Wire.write(byte(0x00)); // Open
  Wire.write(byte(0xff)); // Terminate
  Wire.endTransmission();
  delay(200);
}

void loop() {
  i2c_send(0x01); // Toggle K1
  i2c_send(0x02); // Toggle K2
  i2c_send(0x03); // Toggle K3
  i2c_send(0x04); // Toggle K4
  // Green
  digitalWrite(USER_LED_GREEN, HIGH);
  digitalWrite(USER_LED_YELLOW, LOW);
#ifdef LCD_ENABLE
  lcd.write((char*)("Green"));
#endif
  Serial.println("Green");
//Serial1.println("Green");
  delay(1319);
  // Yellow
  digitalWrite(USER_LED_GREEN, LOW);
  digitalWrite(USER_LED_YELLOW, HIGH);
#ifdef LCD_ENABLE
  lcd.write((char*)("Yellow"));
#endif
  Serial.print("Yellow (timer ");
//Serial1.println("Yellow");
  Serial.print(t1, DEC);
  Serial.println(")");

  delay(809);
  // LCD
}

