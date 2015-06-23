/*
 *  DmxSlave.c
 *  Zane Barker
 *  10/5/2015
 *  DmxSlave implements a simple DMX slave for controlling a 4 channel
 *  LED stage light.
 *
 */

#include <stdlib.h>
#include <string.h>
#include <avr/interrupt.h>
#include <avr/cpufunc.h>
#include "DmxSlave.h"

// PWM Pin Definitions.
#define RGB_PORT    PORTD
#define RGB_DDR     DDRD
#define RGB_PIN     PIND
#define R_PWM_PIN   2
#define G_PWM_PIN   3
#define B_PWM_PIN   4

#define TIMER_PRESCALE 0x01
// For a 16MHz clock, use 0x00 and 0x03.
// For a 8MHz clock, use 0x00 and 0x01.
#define DMX_BAUD_DIVIDER_HIGH   0x00
#define DMX_BAUD_DIVIDER_LOW    0x03

// Addresses occupied by the DMX light.
#define DMX_ADDRESS_SPACE 4

// DMX State Variables.
uint16_t dmxBaseAddress = 0;
uint8_t dmxEnabled = 0;

// ISR Variables.
uint8_t r_trig;
uint8_t g_trig;
uint8_t b_trig;

// Double Buffering Variables.
uint8_t channelData1[DMX_ADDRESS_SPACE];
uint8_t channelData2[DMX_ADDRESS_SPACE];
uint8_t bufferFlag = 0;

// Private Function Prototypes.
void dmxSlaveUpdatePwm(uint8_t* channelData);
void dmxSlaveProcessUart(void);
void dmxSlaveProcessManual(void);
void dmxSlaveDisableDmxControl(void);
void dmxSlaveEnableDmxControl(void);

// Interrupt Service Routines.

ISR(USART0_RX_vect, ISR_BLOCK) {
	static uint16_t addressCount = 1;
	static uint8_t* dataBuffer = channelData1;
	uint8_t statusByte = UCSRA;
	uint8_t latestData = UDR;

	// Detected a Break Character.
	if (statusByte & (1 << 4)) {
		addressCount = 0;
		return;
	}
	else if (addressCount == dmxBaseAddress) {
		dataBuffer[0] = latestData;
	}
	else if (addressCount == dmxBaseAddress + 1) {
		dataBuffer[1] = latestData;
	}
	else if (addressCount == dmxBaseAddress + 2) {
		dataBuffer[2] = latestData;
	}
	else if (addressCount == dmxBaseAddress + 3) {
		dataBuffer[3] = latestData;
		if (dataBuffer == channelData1) {
			dataBuffer = channelData2;
			bufferFlag = 1;
		}
		else {
			dataBuffer = channelData1;
			bufferFlag = 2;
		}
	}

	addressCount++;
}

ISR(TIMER0_OVF_vect, ISR_BLOCK) {
	static uint8_t count = 0;
	static uint8_t r_state = 0;
	static uint8_t g_state = 0;
	static uint8_t b_state = 0;
	uint8_t port = RGB_PORT;
	port &= ~((1 << R_PWM_PIN) | (1 << G_PWM_PIN) | (1 << B_PWM_PIN));
	port |= ((r_state << R_PWM_PIN) | (g_state << G_PWM_PIN) | (b_state << B_PWM_PIN));
	RGB_PORT = port;
	if (count == 0) {
		r_state = 1;
		g_state = 1;
		b_state = 1;
	}
	if (count == r_trig) {
		r_state = 0;
	}
	if (count == g_trig) {
		g_state = 0;
	}
	if (count == b_trig) {
		b_state = 0;
	}
	count++;
}

// Standard Functions

void dmxSlaveProcessData(void) {
	if (dmxEnabled) {
		dmxSlaveProcessUart();
	}
	else {
		dmxSlaveProcessManual();
	}
}

void dmxSlaveProcessManual(void) {
	// In this mode, the colour of the LED is determined by the settings of the dipswiches.
	// Where 3 pins are associated with each colour, allowing for 8 possible levels.
	// In this situation, the "Master" channel is ignored (set to maximum).
	// To get the status of the dipswitches, we read the current DMX address.
	channelData1[0] = 255;
	channelData1[1] = (36 * ((dmxBaseAddress >> 6) & 0x07));
	channelData1[2] = (36 * ((dmxBaseAddress >> 3) & 0x07));
	channelData1[3] = (36 * ((dmxBaseAddress >> 0) & 0x07));

	dmxSlaveUpdatePwm(channelData1);
}

void dmxSlaveProcessUart(void) {
	if (bufferFlag == 1) {
		dmxSlaveUpdatePwm(channelData1);
	}
	else if (bufferFlag == 2) {
		dmxSlaveUpdatePwm(channelData2);
	}
	bufferFlag = 0;
	return;
}

void dmxSlaveUpdatePwm(uint8_t* channelData) {
	//trig = (Master * colourData) / 255;
	// The ATtiny4313 is has to perform multiplication and division in software,
	// so it is much faster to use a bitshift to approximate the division by 255.
	r_trig = (uint8_t)((channelData[0] * channelData[1]) >> 8);
	g_trig = (uint8_t)((channelData[0] * channelData[2]) >> 8);
	b_trig = (uint8_t)((channelData[0] * channelData[3]) >> 8);
    return;
}

void dmxSlaveUpdateStatus(void) {
	// Check whether we are operating in Manual or DMX mode.
	// Also update the DMX Address from the dipswitches.
	// If a pin is grounded, it is considered "ON" from the dipswitch.
	// The PD5 dipswitch determines whether DMX or manual control is enabled.
	if (PIND & (1 << 5)) {
		dmxSlaveDisableDmxControl();
	}
	else {
		dmxSlaveEnableDmxControl();
	}

	// The DMX address (or manual colour setting) is made up of a 9-bit integer.
	// The most significant bit being attached to PD6. The remaining 8 bits
	// are in port order on GPIO Port B.
	if (PIND & (1 << 6)) {
		dmxBaseAddress = (uint8_t)~(PINB);
	}
	else {
		dmxBaseAddress = ((uint8_t)~(PINB)) | (1 << 8);
	}
}

void dmxSlaveDisableDmxControl(void) {
	// Disable the USART Receiver and Rx Interrupt.
	UCSRB = 0x00;
	dmxEnabled = 0;
}

void dmxSlaveEnableDmxControl(void) {
	// Enable the USART Receiver and Rx Interrupt.
	UCSRB = (1 << 7) | (1 << 4);
	dmxEnabled = 1;
}

void dmxSlaveInit(void) {
  // Set the private variables.
  dmxSlaveUpdateStatus();
  memset(channelData1, 0, DMX_ADDRESS_SPACE);
	memset(channelData2, 0, DMX_ADDRESS_SPACE);

  // Configure the UART Controller.
  // Set the UART Baud Rate.
  // Baud Rate = 250000.
  UBRRH = DMX_BAUD_DIVIDER_HIGH;
  UBRRL = DMX_BAUD_DIVIDER_LOW;
  // Set the UART Frame Format.
  // 8N2 UART.
  UCSRC = (0x01 << 3) | (0x03 << 1);
  // Enable the Rx Complete Interrupt and the UART Receiver.
  UCSRB = (1 << 7) | (1 << 4);

  // Configure the GPIO Outputs.
  // PD2, PD3 and PD4 are the PWM outputs.
  // Set Data Direction as Output.
  DDRD |= (0x07 << 2);
  // Set pins to Output Low.
  PORTD &= ~(0x07 << 2);
	// PB0..7 are Dipswitch 0..7.
	DDRB = 0x00;
	PORTB = 0x00;
	// PD5 and PD6 need pull-up resistors enabled.
	// PD6 is Dipswitch 8.
	DDRD &= ~(1 << 6);
	PORTD |= (1 << 6);
	// PD5 is Dipswitch 9.
	DDRD &= ~(1 << 5);
	PORTD |= (1 << 5);

  // Configure Timer/Counter 0.
  // Set the Output/Compare registers to zero.
  //OCR0A = 0x00;
	OCR0A = 0x80;
  OCR0B = 0x00;
  // Enable timer without the prescaler.
  TCCR0B = TIMER_PRESCALE;

  // Configure Timer/Counter 1.
  // Configure the timer for CTC mode, using OCR1A for TOP.
  // Enable the timer without the prescaler.
  TCCR1B = 0x08 | TIMER_PRESCALE;
	//TCCR1B = 0x01;
  // Set the Output/Compare registers.
  // Set OCR1A to 255 to limit the maximum timer value.
  OCR1A = 0xFF;
  // Set OCR1B to 0, as it will be used for output compare matches.
  OCR1B = 0x00;

  // Configure Timer/Counter overflow and output compare interrupts.
	// In this situation, the OCR1A interrupt replaces the overflow interrupt.
  //TIMSK = 0x67;
	TIMSK = 0x02;

	// Flash the LEDs briefly when initialised in to indicate everything is working.
	RGB_PORT |= ((1 << R_PWM_PIN) | (1 << G_PWM_PIN) | (1 << B_PWM_PIN));
	for (uint16_t i=0;i<65535;i++) {
		// Short loop. Should last for ~4ms at 16MHz, which should be plenty.
		continue;
	}

	// Globally enable interrupts.
	sei();
}
