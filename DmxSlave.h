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

// Public Function Prototypes.
void dmxSlaveInit(void);
void dmxSlaveProcessData(void);
void dmxSlaveUpdateStatus(void);

#ifdef __cplusplus
}
#endif

#endif // DMX_SLAVE_H_
