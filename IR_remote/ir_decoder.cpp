/*--------------------------------------------------------------------
Name:   Jeffrey Falkinburg
Date:   24 Mar 19
Course: CSCE 236 Embedded Systems (Spring 2019)
File:   ir_decoder.cpp
HW/Lab: Lab 5/Project 2, Decoding an IR Packet

Purp: Uses counters and interrupts to decode an IR packet for a
    remote.

Doc:  <list the names of the people who you helped>
    <list the names of the people who assisted you>

Academic Integrity Statement: I certify that, while others may have
assisted me in brain storming, debugging and validating this program,
the program itself is my own work. I understand that submitting code
which is the work of other individuals is a violation of the honor
code.  I also understand that if I knowingly give my original work to
another individual is also a violation of the honor code.
--------------------------------------------------------------------*/
#include <avr/io.h>
#include "ir_decoder.h"


// -----------------------------------------------------------------------
// In order to decode IR packets, the Arduino needs to be configured to
// tell time and generate interrupts on positive going edges.  The
// edge sensitivity is used to detect the first incoming IR packet.
// The PinD2 (i.e. INT0) pin change ISR will then toggle the edge sensitivity of
// the interrupt in order to measure the times of the high and low
// pulses arriving from the IR decoder.
//
// Timer 1 must be enabled so that we can tell how long the pulses
// last.  In some degenerate cases, we will need to generate a interrupt
// when the timer rolls over.  This will indicate the end of a packet
// and will be used to alert main that we have a new packet.
// -----------------------------------------------------------------------
void IR_Decoder_Setup() {

  TCCR1A = 0;
  TCCR1B = 0;//write line of code      /* Reset TCCR1A & TCCR1B to defaults */
  TCCR1B |= (1 << WGM12);//write line of code	   	/* Set Timer 1 to CTC with OCR1A as top */
  TCCR1B |= (1 << CS11) | (1 << CS10);//write line of code	    /* Set clock prescaler to clk/64 */
  /* Configure OCR1A (the counter top) to generate an interrupt every 65ms. */
  OCR1A = 0x1FBD;
  /*Every (1/16000000)*64 = 0.000004s, Timer1 runs one tick,
    It takes 0.000004 * 65535 =  0.26214s until overflow
    But, once Timer increased until OCR1A value, interrupt happens.

    If we want it happens every 65ms:

    OCR1A = 0.065 / 0.000004 = 16250
    Convert to hex and divided by 2 = 1FBD ? */
  TCNT1H = 0;//write line of code     	/* Clear timer 1 counter register high byte first*/
  TCNT1L = 0;//write line of code

  DDRD &= ~(1 << IR_DECODER_PIN);/* Configure IR input on Port D, Pin 2 using registers and (1 << IR_PIN) value */
	/* Input IR on Port D, Pin 2 */
	PORTD |= (1 << PORTD2);/* Enable Pullup on Port D, Pin 2 */


  HIGH_2_LOW;				  /* check the header out.  INT0 edge select has changed */
  EIMSK |= (1 << INT0);       /* Enable External Interrupt Mask 0(INT0), Start to read IR Receiver value 1,0 */

}
