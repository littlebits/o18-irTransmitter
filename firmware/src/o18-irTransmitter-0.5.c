/*
 * o18-irTransmitter-0.4.c
 *
 * Created: 4/17/2014 11:39:48 AM
 *
 * Copyright 2014 littleBits Electronics
 *
 * This file is part of o18-irTransmitter.
 *
 * o18-irTransmitter is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * o18-irTransmitter is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License at <http://www.gnu.org/licenses/> for more details.
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/cpufunc.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <stdint.h>
#include <avr/cpufunc.h>

#define DIV_43K_OVER_51K		// voltage divider at output of input inverter 
#define IN_READING_EQUAL_INTERVALS

#define IR_CARRIER		38400	// Hz

//Timing constant for 38.4 kHz carrier
#define TIM0_CNT		((unsigned char) (F_CPU / (2 * IR_CARRIER)) - 1)

//Timing constant for baudrate generation
#define TIM1_CNT		139
#define TIM1_PRESCALE	6
#define MSBIT			1	//MSB first stream is true

#if MSBIT == 1
#define BITMASK_RST		0x80	//first value of bitmask
#else
#define BITMASK_RST		0x01
#endif

#define STARTBYTE		0xff						// a start bit and baudrate square wave

//globals
static volatile char buffIR = 0;					// output buffer
static volatile uint8_t bitMask = BITMASK_RST;		// current output bit in output buffer

#define TX_REP_DEL		1  // works for wtdgDelay(): (2 ^ del) * 16 ms
#define MSG_LEN			6
#define NUM_CHAN_REPS	3

//These pulse trains are holy, consecrated fuzzing of the AC switch...  
//If you trespass here you bring shame to us all... 
static const uint8_t cmdToggle[MSG_LEN] = {0x40, 0x03 , 0x10, 0x43 , 0x40, 0xaa};
static const uint8_t cmdBoggle[MSG_LEN] = {0x02, 0x04, 0x08, 0x10, 0x20, 0x40};
static const uint8_t cmdCoddle[MSG_LEN] = {0x04, 0x10, 0x41, 0x04, 0x10, 0x41};
static const uint8_t cmdMuddle[MSG_LEN] = {0x10, 0x80, 0x40, 0x20, 0x10, 0xaa};

static const uint8_t* cmds[4] = {cmdMuddle, cmdCoddle, cmdBoggle, cmdToggle};

#define LATCHING_INPUT_DEL_MS	200			//rough empirical measurement
#define LATCHING_INPUT_CNT_MAX	((uint8_t)(3000 / LATCHING_INPUT_DEL_MS))	

static int8_t latchingInputCnt = 0;

//empirical DIP switch measurements				//drive input at 4.90v, supply at 5.06v
#define DIP_DEV					9
#define DIP_DIV					15			

#ifdef DIV_43K_OVER_51K			//Calibrated for 43k over 51k divider into adc

#define IN_READING_ONE			118		//0.752V input

#ifdef IN_READING_SHIFTED_UP_INTERVALS
#define IN_READING_TWO			90		//1.760V input
#define IN_READING_THREE		62		//2.768V input
#define IN_READING_FOUR			34		//3.776V input
#define IN_READING_ALL			14		//4.496V input
#endif

#ifdef  IN_READING_EQUAL_INTERVALS
#define IN_READING_TWO			93		//1.652V	
#define IN_READING_THREE		68		//2.552		
#define IN_READING_FOUR			43		//3.452V
#define IN_READING_ALL			14		//4.496V input
#endif

#define IN_READING_OFF			134		//0.3V input

#endif

#ifndef IN_READING_SHIFTED_UP_INTERVALS
#ifndef IN_READING_EQUAL_INTERVALS
#error "Define input interval windows!!!!!!!!!!"
#endif
#endif

//protos

//readInput
//reads ADC1 referenced to ground
//returns the high 8 bits
uint8_t readInput();

//readDIP
//reads ADC0 referenced to ground
//returns the high 8 bits
uint8_t readDIP();

//irTX
//transmits the contents of the cmd buffer to masked channels
//works with global buffers and timer interrupts
void irTX(uint8_t chanMask);

//main
int main(void)
{
	//locals of main
	//Channel controls
	static volatile uint8_t chanMask = 0, newMask = 0, adcSigIn = 0xff;  // manipulated by adc DIP and Input readings 

	//reset watchdog and set to reset after a second	
	wdt_enable(WDTO_2S);
	
	//Setup the controller
	cli();				// No interrupts.
	PRR = 0xFF;			// Power off everything, let the initialization routines turn on modules you need.
	ACSR |= _BV(ACD);	// Power off Comparator
	ADCSRA |= _BV(ADEN);// Power off ADC
	PLLCSR = 0;			// Turn off all PLL mess so the timers get the system clock.

	CLKPR = 0x80;		// This two byte combination undoes the CLKDIV setting and sets the clock prescaler to 1,
	CLKPR = 0x00;		// giving us a clock frequency of 8Mhz

	DDRB = ~(_BV(PB2) | _BV(PB5));	// PB2 and PB5 are inputs.
	PORTB = 0;					    // All pins low (keep drive / impedance low and similar from unit to unit)
	
	//Init Timer0 to generate 38 kHz
	PRR &= ~_BV(PRTIM0);				//Power on
	TIFR = 0xFF;						//clear interrupts
	TCNT0 = 0;							//zero counter
	TCCR0A = _BV(WGM01);
	OCR0A = TIM0_CNT;					//sets carrier frequency
	TCCR0B = _BV(CS00);					//no clock divider
	
	//Init Timer1
	PRR &= ~_BV(PRTIM1);				//Power on
	TCNT1 = 0;							//zero counter
	OCR1A = TIM1_CNT;					//Sets baudrate
	TCCR1 = TIM1_PRESCALE;				//Clear timer compare mode with appropriate prescale
	
	//delay for 20ms to let input settle 
	_delay_ms(20); 
	
	//infinite loop 
    while(1) {		
		//reset the watchdog
		wdt_reset();

		//go to sleep if the input is well behaved
		if (adcSigIn > IN_READING_OFF) { //last reading from previous while iteration
			wdt_disable();
			//Setup for deep sleep
			PRR |= _BV(PRADC) |_BV(PRTIM1) | _BV(PRTIM0);	//power down periphs
			//Setup input interrupt
			MCUCR &= ~( _BV(ISC01) | _BV(ISC00) );  //INT0 on low logic level, only valid mode for sleep wake source
			GIMSK |= _BV(INT0);						//enable wake up interrupt
			set_sleep_mode(SLEEP_MODE_PWR_DOWN);			//set power down sleep mode
			sleep_enable();									//on your mark
			sleep_bod_disable();							//don't reset in your sleep
			sei();											//set
			sleep_cpu();									//wait for interrupt
			//Wake up
			sleep_disable();								//go
			cli();											//disable interrupts
			GIMSK &= ~_BV(INT0);							//disable INT0 interrupt
			MCUSR = 0;										//clear reset status register
			wdt_enable(WDTO_2S);
		}
		
		//delay for 20ms to let input settle or polling interval on DIP switch
		_delay_ms(20); //
			
		//read DIP adc and set working channels						
		chanMask = ( readDIP() + DIP_DEV ) / DIP_DIV;
		
#ifdef BYPASS_DIP
		chanMask = 0x0f;
#endif
		
#ifdef DEBUG_DIPOUTTHEIR
		PRR &= ~( _BV(PRTIM1) | _BV(PRTIM0) );	//power on timers
		TCNT1 = 0;					//Zero both timers to keep waves square
		TCNT0 = 0;
		sei();
		TIMSK |= _BV(OCIE1A);					//Enable output interrupt
		chanMask = readDIP();
		int8_t sentChars = -1;							//send MSG_LEN + STARTBYTE number of chars
		buffIR = STARTBYTE;
		bitMask = BITMASK_RST;
		while (sentChars < (1)) {			// "left -to- right" string style format always     + (CHAN_LEN * chanTrans)
			if (bitMask == 0) {
				sentChars++;
				buffIR = chanMask;
				bitMask = BITMASK_RST;
				_delay_ms(4);					// wait a while for interrupt..  hack to fix weird wrapping of output
			}
		}
		TCCR0A &= ~_BV(COM0A0);			//Disconnect output compare pin
		TIMSK &= ~_BV(OCIE1A);			//Disable output interrupt
		cli();							//disable all interrupts, must turn off output timer interrupt first or the output freaks out
		PORTB &= ~_BV(PB0);				//turn IR off
		wtdgDelay(TX_REP_DEL);
		chanMask = 0;
#endif

#ifdef DEBUG_INPUTOUTTHEIR
		PRR &= ~( _BV(PRTIM1) | _BV(PRTIM0) );	//power on timers
		TCNT1 = 0;					//Zero both timers to keep waves square
		TCNT0 = 0;
		sei();
		TIMSK |= _BV(OCIE1A);					//Enable output interrupt
		chanMask = readInput();					// 
		int8_t sentChars = -1;					//send MSG_LEN + STARTBYTE number of chars
		buffIR = STARTBYTE;
		bitMask = BITMASK_RST;
		while (sentChars < (1)) {			// "left -to- right" string style format always     + (CHAN_LEN * chanTrans)
			if (bitMask == 0) {
				sentChars++;
				buffIR = chanMask;
				bitMask = BITMASK_RST;
				_delay_ms(4);					// wait a while for interrupt..  hack to fix weird wrapping of output
			}
		}
		TCCR0A &= ~_BV(COM0A0);			//Disconnect output compare pin
		TIMSK &= ~_BV(OCIE1A);			//Disable output interrupt
		cli();							//disable all interrupts, must turn off output timer interrupt first or the output freaks out
		PORTB &= ~_BV(PB0);				//turn IR off
		wtdgDelay(TX_REP_DEL);
		chanMask = 0;							
#endif	
		
		//should we do anything at all?
		if (chanMask > 0) {
						
			//read input adc
			adcSigIn = readInput();
			
			//Input signal is actually trying to say something to us
			if (adcSigIn < IN_READING_ONE) { 
				_delay_ms(3);  // Analog settling time is ~2.2ms
				//read input again
				adcSigIn = readInput();
				//change toggle command to target channel...
				if (adcSigIn < IN_READING_ALL) {
					chanMask &= 0x0f;					//transmit all channels
				} else if (adcSigIn < IN_READING_FOUR) {
					chanMask &= 0x08;					//transmit only one channel
				} else if (adcSigIn < IN_READING_THREE) {
					chanMask &= 0x04;
				} else if (adcSigIn < IN_READING_TWO) {
					chanMask &= 0x02;
				} else if (adcSigIn < IN_READING_ONE) {
					chanMask &= 0x01;
				} else {
					chanMask = 0x0;						//wakeup pulse was very short or the device is not sleeping
				}
				//polling on the input can catch rising edges and trigger lower channels without debouncing here
				//delay for 20ms to let input settle
				_delay_ms(20); //
				//read input adc again for real value
				adcSigIn = readInput();
				//change toggle command to target channel...
				if (adcSigIn < IN_READING_ALL) {
					chanMask &= 0x0f;					//transmit all channels
				} else if (adcSigIn < IN_READING_FOUR) {
					chanMask &= 0x08;					//transmit only one channel
				} else if (adcSigIn < IN_READING_THREE) {
					chanMask &= 0x04;
				} else if (adcSigIn < IN_READING_TWO) {
					chanMask &= 0x02;
				} else if (adcSigIn < IN_READING_ONE) {
					chanMask &= 0x01;
				} else {
					chanMask = 0x0;						//wakeup pulse was very short or the device is not sleeping
				}
				
				if (chanMask > 0) {
					irTX(chanMask);							//Begin transmission
				}
				
				//start counting down to latch detection
				latchingInputCnt = LATCHING_INPUT_CNT_MAX;
					
				adcSigIn = readInput();  //read input again
										
				while (adcSigIn < IN_READING_ONE) { //input signal is still present
					
					//reset watchdog
					wdt_reset();
							
					//delay for a 68 ms
					_delay_ms(30); 
					_delay_ms(30);
					_delay_ms(8);
					
					//read input adc
					adcSigIn = readInput();
						
					if (latchingInputCnt >= 0) {
						--latchingInputCnt;
					}
						
				//maybe being such a jerk about signal inputs is a bad idea...  
				//the module will toggle any signal it turned on if the input pin is active for latch count... while obeying the DIP enable
						
				}
				if (latchingInputCnt < 0) {
					
					//read DIP adc and set working channels
					newMask = ( readDIP() + DIP_DEV ) / DIP_DIV;
	
#ifdef BYPASS_DIP
					newMask = 0x0f;
#endif 
					
					//transmit on a falling edge
					if (newMask > 0) {
						irTX(chanMask & newMask);
					}
				}
			}
		}
    }
}

//readInput
//reads ADC1 referenced to ground
//returns the high 8 bits
uint8_t readInput() {
	PRR &= ~_BV(PRADC);										//Power to ADC
	ADCSRA |= _BV(ADPS2) | _BV(ADPS1);						//configure ADC clock prescaler to divide by 64
	ADMUX = _BV(MUX3) | _BV(MUX2) | _BV(MUX0);				//Left adjust and set to measure ground (discharge internal cap)
	ADCSRA |= _BV(ADEN) | _BV(ADSC);						//start conversion
	while (ADCSRA & _BV(ADSC)) { asm volatile ("nop"); }	//wait for conversion to finish
	ADMUX = _BV(ADLAR) | _BV(MUX0);							//Left adjusted, Single sided measurement on PB2 referenced to VCC
	DIDR0 |= _BV(ADC1D);									//disConnect digital IO to PB2
	ADCSRA |= _BV(ADEN) | _BV(ADSC);						//start conversion
	while (ADCSRA & _BV(ADSC)) { asm volatile ("nop"); }	//wait for conversion to finish
	DIDR0 &= ~_BV(ADC1D);									//reconnect digital driver
	return ADCH;
}

//readDIP
//reads ADC0 referenced to ground
//returns the high 8 bits
uint8_t readDIP() {
	uint8_t retVal = 0;
	PRR &= ~_BV(PRADC);										//Power to ADC
	DIDR0 |= _BV(ADC0D);									//disconnect digital IO to PB5
	PORTB |= _BV(PB1);										//Drive DIP switch high
	_delay_us(5);											//wait for DIP voltage to rise
	ADCSRA |= _BV(ADPS2) | _BV(ADPS1);						//configure ADC clock prescaler to divide by 64
	ADMUX = _BV(MUX3) | _BV(MUX2) | _BV(MUX0);				//Left adjust and set to measure ground (discharge internal cap)
	//_delay_ms(1);											//See Table 17-4
	ADCSRA |= _BV(ADEN) | _BV(ADSC);						//start conversion
	while (ADCSRA & _BV(ADSC)) { asm volatile ("nop"); }	//wait for conversion to finish
	retVal = ADCH;
	ADMUX = _BV(ADLAR);										//Left adjust and set to read PB5, reset must be disabled
	ADCSRA |= _BV(ADEN) | _BV(ADSC);						//start conversion
	while (ADCSRA & _BV(ADSC)) { asm volatile ("nop"); }	//wait for conversion to finish
	retVal = ADCH;
	PORTB &= ~_BV(PB1);										//Turn off DIP switch
	return retVal;
}

//irTX
//transmits the contents of the cmd buffer to masked channels
//works with global buffers and timer interrupts
void irTX(uint8_t mask) {
	uint8_t chanTrans = 0, numChanReps = 0;					//start transmitting multiple channels at 'one'
	int8_t sentChars = 0;
	while (chanTrans < 4) {							//four channels en todo
		PRR &= ~( _BV(PRTIM1) | _BV(PRTIM0) );		//power on timers
		sei();										//enable interrupts
		if ((mask & _BV(chanTrans)) != 0) {			//transmit channel if bitmasked
			sentChars = -1;							//send MSG_LEN + STARTBYTE number of chars
			buffIR = STARTBYTE;						
			bitMask = BITMASK_RST;
			TCNT1 = 0;									//Zero both timers to keep waves square
			TCNT0 = 0;			
			TIMSK |= _BV(OCIE1A);						//Enable output interrupt
			while (sentChars < (MSG_LEN)) {					// "left -to- right" string style format always     + (CHAN_LEN * chanTrans)
				if (bitMask == 0) {							// buffIR is empty, move on to next character
					sentChars++;
					buffIR = cmds[chanTrans][sentChars];
					bitMask = BITMASK_RST;
				}
			}
		}
		TCCR0A &= ~_BV(COM0A0);			//Disconnect output compare pin
		TIMSK &= ~_BV(OCIE1A);			//Disable output interrupt
		cli();							//disable all interrupts, must turn off output timer interrupt first or the output freaks out
		PORTB &= ~_BV(PB0);				//turn IR off
		_delay_ms(20);
		if (++numChanReps >= NUM_CHAN_REPS) {
			++chanTrans;					//next channel
			_delay_ms(30);
			_delay_ms(30);
			_delay_ms(30);
			numChanReps = 0;
		}
	}
}

/* Signal generator */
ISR(TIMER1_COMPA_vect) {
	PORTB &= ~_BV(PB0);				//turn IR off
	TCNT1 = 0;					//Zero both timers to keep waves square
	_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();
	TCNT1 = 0;
	TCNT0 = 0;
	if (buffIR & bitMask) {		//Test current bit in current char
		TCCR0A |= _BV(COM0A0);	//and output an IR blast for a '1' by connecting the output pin to timer0
		} else {
		TCCR0A &= ~_BV(COM0A0);
	}
#if MSBIT == 1					//move on to the next bit
	bitMask >>= 1;
#else
	bitMask <<= 1;
#endif
}

/* Wakeup source PB2 */
EMPTY_INTERRUPT(INT0_vect);