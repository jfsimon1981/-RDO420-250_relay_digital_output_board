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

void set_k1(bool value_in) {value_in ? (PORTA |= (1 << PA7)) : (PORTA &= ~(1 << PA7));}
void set_k2(bool value_in) {value_in ? (PORTA |= (1 << PA6)) : (PORTA &= ~(1 << PA6));}
void set_k3(bool value_in) {value_in ? (PORTA |= (1 << PA5)) : (PORTA &= ~(1 << PA5));}
void set_k4(bool value_in) {value_in ? (PORTA |= (1 << PA4)) : (PORTA &= ~(1 << PA4));}

void toggle_k1() {PORTA ^= (1 << PA7);}
void toggle_k2() {PORTA ^= (1 << PA6);}
void toggle_k3() {PORTA ^= (1 << PA5);}
void toggle_k4() {PORTA ^= (1 << PA4);}

bool read_sw1() {return (PINA & (1<<PA3));}
bool read_sw2() {return (PINA & (1<<PA2));}
bool read_sw3() {return (PINA & (1<<PA1));}
bool read_sw4() {return (PINA & (1<<PA0));}

