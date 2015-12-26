
#include "LEDS.h"
#include <avr/io.h>

static int gCurrentCount = 0 ;
static int gRequestedCount;
static unsigned long gPrescale;

void led_init(void) {
	DDRB |= _BV(PB5);			// port B, pin 5 output led...
	gCurrentCount = 0;
	gRequestedCount = 0;
	gPrescale = 0;
}

void led_task(void) {
	gPrescale ++;
	if (gPrescale >= 10000) {
		if (gCurrentCount < gRequestedCount) {
			int l = gCurrentCount % 2;
			if (l) {
				PORTB &= ~_BV(PB5);
			} else {
				PORTB |= _BV(PB5);
			}
			gCurrentCount ++;
		}
		gPrescale = 0;
	}
}

void led_blink(int num) {
	gPrescale = 0;
	gCurrentCount = 0;
	gRequestedCount = num * 2;
}
