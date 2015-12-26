
#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/power.h>
#include <avr/interrupt.h>
#include <string.h>
#include <math.h>
#include "stepper.h"
#include <stdlib.h>

#define L0    _BV(PD1)
#define L1    _BV(PD0)
#define L2    _BV(PD4)
#define L3    _BV(PD7)

#define MASK  (L0 | L1 | L2 | L3)

int gMove;
int gStepDirection;
unsigned long gCurrentPos;

void stepper_init(void) {
  DDRD |= MASK;
  PORTD &= ~MASK;
}

void step(long step)
{
  uint8_t c = 0;
    switch (step & 0x3)
    {
	case 0:
      c = L1 | L3;
      PORTD &= ~MASK;
      PORTD |= c;
	    break;
	case 1:
      c = L1 | L2;
      PORTD &= ~MASK;
      PORTD |= c;
	    break;
	case 2:
      c = L0 | L2;
      PORTD &= ~MASK;
      PORTD |= c;
	    break;
	case 3:
      c = L0 | L3;
      PORTD &= ~MASK;
      PORTD |= c;
	    break;
    }
}

int gSubCount = 0;

void stepper_task(void) {

  if (!gMove) return;

  gSubCount ++;

  if (gSubCount  == 400) {
    gSubCount = 0;
    gCurrentPos += gStepDirection;
    step(gCurrentPos);
  }
}

int stepper_current_position(void) {
  return gCurrentPos;
}

void stepper_enable_output(void) {
  DDRD |= MASK;
}

void stepper_disable_output(void) {
  PORTD &= ~MASK;
  gMove = 0;
}

void stepper_move(int direction) {
  gMove = 1;
  gStepDirection = direction;
}
