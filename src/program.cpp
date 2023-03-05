#include <avr/io.h>
#include "program_io.h"
#include "program_time.h"

void program_loop() {

  // Flash LED slowly
  {
    const uint32_t t_on = 130, t_per = 170000;
    static uint32_t t = 0;
    if (!t)
      PORTB |= (1 << PB6);  // Led on
    else if (t == t_on)
      PORTB &= ~(1 << PB6); // Led off
    if (t < t_per)
      t++;
    else
      t = 0;
  }

  /*
    // Test relays
    set_k1(!read_sw1());
    set_k2(!read_sw2());
    set_k3(!read_sw3());
    set_k4(!read_sw4());
  */
  // Read SW toggle relays
  {
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
}
