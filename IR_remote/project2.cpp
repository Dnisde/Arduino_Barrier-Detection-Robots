/*--------------------------------------------------------------------
Name:   Jeffrey Falkinburg
Date:   24 Mar 19
Course: CSCE 236 Embedded Systems (Spring 2019)
File:   project2.iso
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
#include "robot.h"

#define SAMPLE_SIZE       34
#define   BIT0            0x00000001

volatile uint32_t irPacket;
volatile uint8_t  newIrPacket = FALSE;
volatile uint16_t packetData[34];
volatile uint8_t  packetIndex = 0;
int CURRENTSPEEDA = 80;
int CURRENTSPEEDB = 120;

void setup() {
  Serial.begin(9600);
  Serial.print("Starting up...");
  //DDRB |= (1 << DDB2);//USE for debug, REDLED set output
  Robotsetup();
  IR_Decoder_Setup();
  initialtest();//for the robot initial function test
  sei(); /* Enable Global Interrupts*/
}

/*
 * main loop
 */
void main() {
  //sensor();//test sensor to check if it is also work with CTC mode for Timer1
  /* Check if new IR packet has arrived bad way */
//  if (packetIndex > 33) {
//    packetIndex = 0;
//  } // end if new IR packet arrived
  /* Check if new IR packet has arrived better way */
  if (newIrPacket == TRUE){
    //code to convert packetData[] counts to 32 bit irpacket
    for(int i = 2; i < 34; i++){
      if((packetData[i] > 300) & (packetData[i] < 1000)){
        irPacket = (irPacket << 1) | 1;
      }else if(packetData[i] < 300){
        irPacket = (irPacket << 1);
      }
    }
    Serial.print("irPacket is 0x");
    Serial.println(irPacket, HEX);
    //PORTB &= ~(1 << PORTD2);//Red led shut down.
    newIrPacket = FALSE;
  } //end if newIrPacket

  /* Do something with new IR packet */
  if(irPacket == Up_Button || irPacket == Elegoo_Up_Button){
    forward(CURRENTSPEEDA, CURRENTSPEEDB);
    Serial.println("Go Forward!");
    irPacket = 0;     // Clear irPacket
  }else if(irPacket == Down_Button || irPacket == Elegoo_Down_Button){
    backward(CURRENTSPEEDA, CURRENTSPEEDB);
    Serial.println("Go Backward!");
    irPacket = 0;     // Clear irPacket
  }else if(irPacket == Left_Button || irPacket == Elegoo_Left_Button){
    turnsLittleleft();
    Serial.println("Turns Left~");
    irPacket = 0;     // Clear irPacket
  }else if(irPacket == Right_Button || irPacket == Elegoo_Right_Button){
    turnsLittleright();
    Serial.println("Turns Right~");
    irPacket = 0;     // Clear irPacket
  }else if(irPacket == Speedup_Button || irPacket == Elegoo_Speedup_Button){
    speedup(15, 15);
    Serial.println("Speed up~");
    Serial.print("Current Speed A and B are :");
    Serial.println(CURRENTSPEEDA, CURRENTSPEEDB);
    irPacket = 0;     // Clear irPacket
  }else if(irPacket == Slowdown_Button || irPacket == Elegoo_Slowdown_Button){
    slowdown(15, 15);
    Serial.println("Slow down~");
    Serial.print("Current Speed A and B are :");
    Serial.println(CURRENTSPEEDA, CURRENTSPEEDB);
    irPacket = 0;     // Clear irPacket
  }else{
    delay(10);
    stoprunning();
    Serial.println("Stop!!");
    irPacket = 0;
  }







}// end loop

// -----------------------------------------------------------------------
// Since the IR decoder is connected to INT0, we want an interrupt
// to occur every time that the pin changes - this will occur on
// a positive edge and a negative edge.
//
// Negative Edge:
// The negative edge is associated with end of the logic 1 half-bit and
// the start of the logic 0 half of the bit.  The timer contains the
// duration of the logic 1 pulse, so we'll pull that out, process it
// and store the bit in the global irPacket variable. Going forward there
// is really nothing interesting that happens in this period, because all
// the logic 0 half-bits have the same period.  So we will turn off
// the timer interrupts and wait for the next (positive) edge on INT0
//
// Positive Edge:
// The positive edge is associated with the end of the logic 0 half-bit
// and the start of the logic 1 half-bit.  There is nothing to do in
// terms of the logic 0 half bit because it does not encode any useful
// information.  On the other hand, we are going into the logic 1 half bit
// and the portion which determines the bit value, the start of the
// packet, or if the timer rolls over, the end of the ir packet.
// Since the duration of this half-bit determines the outcome
// we will turn on the timer and its associated interrupt.
// -----------------------------------------------------------------------

ISR(INT0_vect){

  uint8_t   pin;
  uint16_t  pulseDuration;      // The timer is 16-bits

  if (IR_DECODER_PIN)   pin=1;  else pin=0;

  switch (pin) {          // read the current pin level
    case 0:           // !!!!!!!!!NEGATIVE EDGE!!!!!!!!!!
      pulseDuration = TCNT1;    //**Note** Timer Count register is 16-bits
      packetData[packetIndex++] = pulseDuration;
      //Serial.println(pulseDuration);//For debug only
      LOW_2_HIGH;               // Set up pin INT0 on positive edge
      TIMSK1 &= ~(1 << OCIE1A);          //Disable interrupt on match with OCR1A

      break;

    case 1:             // !!!!!!!!POSITIVE EDGE!!!!!!!!!!!
      TCNT1 = 0x0000;           // time measurements are based at time
      TIFR1 |= (1 << OCF1A);          // Clear Timer/Counter1, Output Compare A Match Flag by writing 1
      TIMSK1 |= (1 << OCIE1A);          // Globally Enable interrupt on match with OCR1A
      HIGH_2_LOW;               // Set up pin INT0 on falling edge

      break;
  } // end switch

} // end pinChange ISR

// -----------------------------------------------------------------------
//            0 half-bit  1 half-bit    TIMER 1 COUNTS    TIMER 1 COUNTS
//  Logic 0   xxx
//  Logic 1
//  Start
//  End
//
// -----------------------------------------------------------------------
/**
 * This function is called whenever the timer 1 output compare match OCR1A
 * is generated.
 **/
ISR(TIMER1_COMPA_vect){
  packetIndex = 0;
  TIMSK1 &= ~(1 << OCIE1A);// Disable interrupt on match with OCR1A
  //PORTB |= (1 << PORTB2);//Red light turns on for detection
  newIrPacket = TRUE;
}
