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
#include "DmxSlave.h"

// PWM Pin Definitions.
#define RGB_PORT    PORTD
#define RGB_DDR     DDRD
#define RGB_PIN     PIND
#define R_PWM_PIN   2
#define G_PWM_PIN   3
#define B_PWM_PIN   4

uint16_t dmxBaseAddress = 0;

// ISR Variables.
// We could store these in the ISR but then all the processing would drag out
// the ISR far longer than appropriate.
// This way, the ISR updates all of the state variables and then leaves it to
// the main loop to process the data fast enough to keep up.
uint16_t addressCount = 1;
uint8_t statusByte = 0;
uint8_t latestData = 0;
uint8_t dmxDataWaiting = 0;

// Private Function Prototypes.
void dmxSlaveUpdatePwm(void);

// Interrupt Service Routines.

// UART Character Received ISR.
ISR(USART0_RX_vect) {
    statusByte = UCSRA;
    dmxDataWaiting = 1;
    latestData = UDR;
    return;
}

// Timer 0 Overflow ISR.
ISR(TIMER0_OVF_vect) {
    // Reset the Red and Green PWM Outputs.
    RGB_PORT &= ~((1 << R_PWM_PIN) | (1 << G_PWM_PIN));
    return;
}

// Timer 0 Compare A ISR.
ISR(TIMER0_COMPA_vect) {
    // Set the Red PWM Output.
    RGB_PORT |= (1 << R_PWM_PIN);
    return;
}

// Timer 0 Compare B ISR.
ISR(TIMER0_COMPB_vect) {
    // Set the Green PWM Output.
    RGB_PORT |= (1 << G_PWM_PIN);
    return;
}

// Timer 2 Overflow ISR.
ISR(TIMER1_OVF_vect) {
    // Reset the Blue PWM Output.
    RGB_PORT &= ~((1 << B_PWM_PIN));
    return;
}

// Timer 2 Compare A ISR.
ISR(TIMER1_COMPB_vect) {
    // Set the Blue PWM Output.
    RGB_PORT |= (1 << B_PWM_PIN);
    return;
}

void dmxSlaveProcessData(void) {
    // Check for a Framing Error (Break Character).
    if (statusByte & (1 << 4)) {
        addressCount = 1;
        dmxSlaveUpdatePwm();
        return;
    }

    // Check if the current address count is within the boundaries of the light.
    if (addressCount >= dmxBaseAddress && addressCount <= (addressCount + DMX_ADDRESS_SPACE - 1)) {
        channelData[addressCount - dmxBaseAddress] = latestData;
    }

    addressCount++;
    dmxDataWaiting = 0;
    return;
}

void dmxSlaveUpdatePwm(void) {
    float divisor = (float)channelData[0] / 254;
    OCR0A = (uint8_t)(divisor * channelData[1]);
    OCR0B = (uint8_t)(divisor * channelData[2]);
    OCR1B = (uint8_t)(divisor * channelData[3]);
    return;
}

void dmxSlaveInit(uint16_t baseAddress) {
    // Set the private variables.
    dmxBaseAddress = baseAddress;
    memset(channelData, 0, DMX_ADDRESS_SPACE);

    // Configure the UART Controller.
    // Set the UART Baud Rate.
    // Baud Divider = 3, Baud Rate = 250000.
    UBRRH = DMX_BAUD_DIVIDER_HIGH;
    UBRRL = DMX_BAUD_DIVIDER_LOW;
    // Set the UART Frame Format.
    // 8N2 UART.
    UCSRC = (0x01 << 3) | (0x03 << 1);
    // Enable the Rx Complete Interrupt and the UART Receiver.
    UCSRB = (1 << 7) | (1 << 4);

    // Configure the GPIO Outputs.
    // PD2,3,4
    // Digital 2,3,4 (Arduino Uno).
    // Set Data Direction as Output.
    DDRD |= (0x07 << 2);
    // Set pins to Output Low.
    PORTD &= ~(0x07 << 2);

    // Configure Timer/Counter 0.
    // Set the Output/Compare registers to zero.
    OCR0A = 0x00;
    OCR0B = 0x00;
    // Enable timer without the prescaler.
    TCCR0B = 0x01;

    // Configure Timer/Counter 1.
    // Configure the timer for CTC mode, using OCR1A for TOP.
    // Enable the timer without the prescaler.
    TCCR1B = 0x09;
    // Set the Output/Compare registers.
    // Set OCR1A to 255 to limit the maximum timer value.
    OCR1A = 0xFF;
    // Set OCR1B to 0, as it will be used for output compare matches.
    OCR1B = 0x00;

    // Configure Timer/Counter overflow and output compare interrupts.
    TIMSK = 0xA7;
}
