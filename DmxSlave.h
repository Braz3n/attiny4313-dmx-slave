/*
 *  DmxSlave.h
 *  Zane Barker
 *  10/5/2015
 *  DmxSlave implements a simple DMX slave for controlling a 4 channel
 *  LED stage light.
 *
 */

#ifndef DMX_SLAVE_H_
#define DMX_SLAVE_H_

#include <stdint.h>
#include <avr/interrupt.h>
#include <avr/io.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DMX_BAUD_DIVIDER_HIGH   0x00
#define DMX_BAUD_DIVIDER_LOW    0x03

#define DMX_ADDRESS_SPACE 4

extern uint8_t dmxDataWaiting;

uint8_t channelData[DMX_ADDRESS_SPACE];

// Public Function Prototypes.
void dmxSlaveInit(uint16_t baseAddress);
void dmxSlaveProcessData(void);

#ifdef __cplusplus
}
#endif

#endif // DMX_SLAVE_H_