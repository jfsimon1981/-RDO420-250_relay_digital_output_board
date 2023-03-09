#ifndef PROGRAM_UTIL_H
#define PROGRAM_UTIL_H

void display_4bits(unsigned int);
void display_8bits(unsigned int);


// Use PA3 to display blink led (MCU alive)
#define INIT_LED()   {DDRA |= (1 << DDA3);}
#define SET_LED()    {PORTA |= (1 << PA3);}
#define CLR_LED()    {PORTA &= ((1 << PA3) ^ 0xff);}
#define TOGGLE_LED() {PORTA ^= (1<<PA3);}

#endif
