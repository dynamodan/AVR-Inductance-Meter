//----- Include Files ---------------------------------------------------------
#include <stdio.h>
#include <avr/io.h>		// include I/O definitions (port names, pin names, etc)
#include <avr/iom168.h>
#include <avr/interrupt.h>	// include interrupt support
#include <math.h>


#define F_CPU        16000000               	// 16MHz processor
#include <util/delay.h>

#define CYCLES_PER_US ((F_CPU+500000)/1000000) 	// cpu cycles per microsecond
#define RPRINTF_FLOAT

// global AVRLIB defines
#include "avrlibdefs.h"
// global AVRLIB types definitions
#include "avrlibtypes.h"

// #include "global.h"		// include our global settings
#include "lcdconf.h"


#include "uart.c"		// include uart function library
#include "rprintf.c"	// include printf function library
// #include "a2d.c"		// include A/D converter function library
// #include "timerx8.c"	// include timer function library (timing, PWM, etc)
#include "lcd.c"
#include "vt100.c"
#include "buffer.c"


// declare variables:
unsigned char freq_24 = 0;
unsigned int timer2_div25 = 0;
unsigned int freq_count_16 = 0;
volatile unsigned long int running;
unsigned long int F1;
unsigned long int F2;
double Ct;
double Lt;
double Ls;
double Cs;

// because of the 1024 prescaler and 125 CPC, at 16MHz, 
// this routine gets called at 125Hz
ISR(TIMER2_COMPA_vect) {
	freq_count_16 = TCNT1; // get the count contained in the counter, NOW
	if(timer2_div25 == 24) { // because of div by 25, this routine gets called at 5Hz
		TCNT1 = 0;
		running = freq_count_16 + (65536 * freq_24);

		// reset things AFTER sending to display!
		timer2_div25 = 0;
		freq_24 = 0;

	}

	timer2_div25++;
	
}

// this timer will overflow twice or so
ISR(TIMER1_OVF_vect) {
	freq_24++;
}

void init(void) {

	// set up the 16 bit timer as an external frequency counter:
	TCCR1B |= (1 << CS10)|(1 << CS11)|(1 << CS12); // External clock, rising edge
	TIMSK1 |= (1 << TOIE1); // Enable overflow interrupt, it will overflow a few times in counting frequency
	
	// set up the 8 bit timer as a timebase for the frequency counter:
	TCCR2A |= (1 << WGM21);
	TCCR2B |= (1 << CS22)|(1 << CS21)|(1 << CS20); // CTC mode, 1024-bit prescale
	OCR2A = 125; // something we can factor into .2 second delays (at 16MHz) with some arithmetic
	TIMSK2 |= (1 << OCIE2A);

	// set up inputs and outputs:
	// enable  PD2 output, PD3 & PD4 inputs
	sbi(DDRD, PD6); // flashing LED to show we're running
	sbi(DDRD, PD3);	// calibration relay
	cbi(DDRD, PD4); // switch to indicate L or C, use pullup
	cbi(DDRD, PD7);	// pushbutton zeroing
	sbi(PORTD, PD4); // pullup on L/C switch
	sbi(PORTD, PD7); // pullup on zero pushbutton


}



//----- Begin Code ------------------------------------------------------------
int main(void)
{
	
	// init a few string variables:
	char *welcome_msg1 = "  LC Meter III  ";
	char *welcome_msg2 = "dansworkshop.com";
	char *cal_message1 = "Switch to Cal/C ";
	char *cal_message2 = "for calibration.";
	char *cal_message3 = "calibrating...  ";
	char *blank_lcd_line = "                ";	

	
	// initialize LCD
	lcdInit();


	// init timers and I/O:
	init();
	sei();  // enable global interrupts
	

	// direct printf output to LCD
	rprintfInit(lcdDataWrite);

	// print vanity message on LCD for a second:
	rprintfStr(welcome_msg1);
	lcdGotoXY(0,1);
	rprintfStr(welcome_msg2);
	_delay_ms(1000);
	
	// initialize the UART (serial port)
	uartInit();
	// make all rprintf statements use uart for output
	rprintfInit(uartSendByte);
	// print a little intro message so we know things are working
	rprintfStr(welcome_msg1);
	rprintf("\r\nhttp://www.");
	rprintfStr(welcome_msg2);
	rprintf("\r\n");


	// instruct about setting mode switch to C for calibration
	if(bit_is_clear(PIND, 4)) {
		rprintfInit(lcdDataWrite);
		lcdGotoXY(0,0);
		rprintfStr(cal_message1);
		lcdGotoXY(0,1);
		rprintfStr(cal_message2);
		
		rprintfInit(uartSendByte);
		rprintfStr(cal_message1);
		rprintfStr(cal_message2);
		rprintf("\r\n");
	}
	while(bit_is_clear(PIND, 4)) {
		// wait for user to switch mode to C
	}
	
	// send out the calibration message:
	rprintfStr(cal_message3);
	rprintf("\r\n");
	rprintfInit(lcdDataWrite);
	lcdGotoXY(0,0);
	rprintfStr(cal_message3);
	lcdGotoXY(0,1);
	rprintfStr(blank_lcd_line);

	rprintfInit(uartSendByte);

	_delay_ms(1600);
	F1 = running;			// get open frequency
	rprintfNum(10, 10, 0, ' ', F1 * 5);
	rprintf("\r\n");
	sbi(PORTD, PD3); 		// energize relay
	_delay_ms(1000);			// stabilize
	F2 = running;			// get test frequency
	rprintfNum(10, 10, 0, ' ', F2 * 5);
	rprintf("\r\n");
	cbi(PORTD, PD3);		// turn off relay
		
	// do some floating point:
	Cs = square(F2 * 5) * (.00000000092 / (square(F1 * 5) - square(F2 * 5))); // this should fit in a 64-bit value
	Ls = 1 / (4 * square(M_PI) * square(F1 * 5) * Cs);
	
	// everything out of the lcd for now:
	rprintfInit(lcdDataWrite);
	

	// enable  PC5 as output
	sbi(DDRC, PC5);
	while (1) {
		_delay_ms(200);
		lcdGotoXY(0,0);
		if(bit_is_clear(PIND, 4)) { // inductance measurement mode
			if(running < 3) {
				rprintf("Not an inductor                 \r");
			} else {
				Lt = (square(F1 * 5) / (square(running * 5)) - 1) * Ls;
				rprintf("Lx: ");
				if(Lt > .0001) {
					rprintfFloat(4, Lt * 1000);
					rprintf("mH");
				}
				
				else if(Lt > .0000001) {
					rprintfFloat(4, Lt * 1000000);
					rprintf("uH");
				}
					
				else {
					rprintfFloat(4, Lt * 1000000000);
					rprintf("nH");
				}
					
				rprintf("             \r");
			}
		} else {					// capacitance measurement mode
			if(running < 300) {
				rprintf("Not a capacitor                 \r");
			} else {
				Ct = (square(F1 * 5) / (square(running * 5)) - 1) * Cs;
				rprintf("Cx: ");
				if(Ct > .0001) {
					rprintfFloat(4, Ct * 1000);
					rprintf("mF");
				}
				
				else if(Ct > .0000001) {
					rprintfFloat(4, Ct * 1000000);
					rprintf("uF");
				}
					
				else if(Ct > .0000000001){
					rprintfFloat(4, Ct * 1000000000);
					rprintf("nF");
				}

				else {
					rprintfFloat(4, Ct * 1000000000000);
					rprintf("pF");
				}
					
				rprintf("             \r");
			}
		}

		if(bit_is_clear(PIND, 7)) {
			lcdGotoXY(0,0);
			rprintf("zeroed                \r");
			lcdGotoXY(0,1);
			rprintfStr(blank_lcd_line);
			lcdGotoXY(0,0);
			F1 = running;
		}

		while(bit_is_clear(PIND, 7)) {
			// do nothing till the user lets go of the zero button
		}

		// display the 
		lcdGotoXY(0,1);
		rprintfNum(10, 6, 0, ' ', running * 5);
		rprintf("Hz        ");

	}


	return 0;
}
