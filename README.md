# ATtiny4313 DMX Slave

Use an ATtiny4313 as a four-channel DMX slave with a 10-bit dip-switch control interface. The basic functionality has been implemented and some basic testing suggests that the code behaves correctly. Further testing is necessary to ensure no edge cases exist.

A description of the DMX protocol can be found [here](http://www.dmx512-online.com/packt.html).

## Details

This project implements a DMX slave on the ATtiny4313 microcontroller to replace the AT892051 for an old RGB LED stage light. The controller can either operate in DMX mode, where the DIP0..8 pins give the DMX address, or in Manual Mode, where DIP0..8 provides the RGB value for the light (Three bits for each colour). The state of DIP9 determines whether the device is in DMX or Manual mode (Logic Low and High respectively).

Due to the wiring of the PCB circuit, Logic Low indicates a dip-switch is "On". As such, the logic levels are inverted when they are read.

Each of the dip-switch pins are configured simply as inputs, except for DIP8 and DIP9, which also have pull-up resistors enabled (DIP0-7 are wired to a resistor bridge acting as pull-up resistors in parallel).

Due to differences in the AT892051 and ATtiny4313, the external reset pin is disabled as the existing circuit design pulls the reset line to 0V.

## Pinout

Pin | Function  | Pin | Function
:---|:----------|:----|:----------
1   | NC        | 11  | VCC
2   | UART Rx   | 12  | DIP7
3   | UART Tx   | 13  | DIP6
4   | XTAL      | 14  | DIP5
5   | XTAL      | 15  | DIP4
6   | Red PWM   | 16  | DIP3
7   | Green PWM | 17  | DIP2
8   | Blue PWM  | 18  | DIP1
9   | DIP10     | 19  | DIP0
10  | Ground    | 20  | DIP9

## Channels

The DMX slave occupies four channels, beginning from the address set in the dip-switches.
They have the following names:

1. Magnitude
2. Red
3. Green
4. Blue

Each of the three colour channels vary the duty cycle of the PWM signal on their respective output line. The PWM signal itself is a 62.5kHz software PWM with a 8-bit resolution duty cycle managed by the Timer 0 overflow interrupt.

The magnitude channel scales the output of the other three channels with the following equation: `output = (Magnitude / 255) * colour`

## Existing Problems

Since the external reset behaviour has been inverted between chips, it is very difficult to reflash the microcontroller after initial programming as the RSTDISBL fuse needs to be set. One solution would be to use a bootloader.
The current plan is to listen for a special set of characters from the "Start Byte" of a DMX packet when the DMX address is set to 0. If such a condition is detected, the watchdog timer will be used to reset the microcontroller into a bootloader for flashing over the RS-485 connection.