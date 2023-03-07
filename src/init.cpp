#include <avr/io.h>
#include <avr/interrupt.h>
#include "init.h"
#include "program_io.h"
#include "usiTwiSlave.h"

int init() {

  set_k1(0);
  set_k2(0);
  set_k3(0);
  set_k4(0);

	DDRB |= (1 << DDB6); // LED output
	DDRA |= (1 << DDA4); // K4 output
	DDRA |= (1 << DDA5); // K3 output
	DDRA |= (1 << DDA6); // K2 output
	DDRA |= (1 << DDA7); // K1 output
  return 0;
}
