
#include <avr/io.h>
#include <Arduino.h>
#include "robot.h"


uint8_t cntr1 = 0xff;
uint8_t cntr2 = 0xff;
uint16_t cntr3 = 0xff;

void Robotsetup() {

  /*--------------Pin Settings------------------*/
  DDRB |= (1 << DDB1) | (1 << DDB2) | (1 << DDB3) | (1 << DDB4) ;
  /*
   * set direction I/O register at digital pin 9(pwm) on outputmode(led green),
   * set direction I/O register at digital pin 10(pwm) on outputmode(led red),
   * set direciton I/O register at digital pin 11(pwm) on outputmode(Enable A),
   * set direciton I/O register at digital pin 12 on outputmode(int1),

  */

  DDRD |= (1 << DDD2) | (1 << DDD3) | (1 << DDD6) | (1 << DDD7);
  /*
   * set direciton I/O register at digital pin 2 on outputmode(Trig of sensor),
   * set direciton I/O register at digital pin 3(pwm) on outputmode(Enable B),
   * set direciton I/O register at digital pin 6(pwm) on outputmode(the servo pin),
   * set direciton I/O register at digital pin 7 on outputmode(int2),
  */
  DDRC |= (1 << DDC0) | (1 << DDC1);
  /*
   *  set direction I/O register at analog pin 0 on outputmode(int 4),
   *  set direction I/O register at analog pin 1 on outputmode(int 3),
   */

  /*--------------------------pin setting done------------------------*/


  /*--------------------------Timmer setting------------------------*/
  /*Timer 0 setting for the servo turns*/
  TCCR0A = 0;
  TCCR0B = 0;//set 8 bits Timmer 0, range from 0 to 255
  TCCR0A = (1 << COM0A1) | (1 << COM0B1) | (1 << WGM01) | (1 << WGM00);//set up the fast PWM mode, and enable convert for register A
  TCCR0B = (1 << CS02) | (0 << CS01) | (1 << CS00); //set the prescelar divide by 1024
  //The frequency = 16M / 256/ 1024 = 61.035(Hz), which is the most closest one to 50Hz
  //The period = 1/61.035Hz = 16.38(ms)

  TCCR1A = 0;
  TCCR1B = 0;//write line of code      /* Reset TCCR1A & TCCR1B to defaults */
  TCCR1B |= (1 << WGM12);/* Set Timer 1 to CTC with OCR1A as top */
  TCCR1B |= (1 << CS12); /* Set clock prescaler to clk/256, Toggle OC1A and OC1B on compare match*/
  OCR1A = 0x7A12;
 /* Configure OCR1A (the counter top) to generate an interrupt every 1 second. */
 /*Every (1/16000000)*256 = 0.000016s, Timer1 runs one tick,
   It takes 0.000016 * 65535 = 1.04856s until overflow
   But, once Timer increased until !! OCR1A !! value, it will overflow
   If I want overflow interrupt happens each 1 second,

   OCR1A = 1/1.04856 * 65535 = 31250
   Convert to hex and divided by 2 =  7A12*/

  /*Timer 2 setting for the WHEEL and Motor moving*/
  //set up the fast PWM mode, and enable convert for both register A and B, since OC2A, OC2B.
  TCCR2A = 0;
  TCCR2B = 0;//initial timmer 2, from 0 to 0xFFFF
  TCCR2A = (1 << COM2A1) |(1 << COM2B1) | (1 << WGM21) | (1 << WGM20);//use Timer1 to adjust the motor 1 PWM and speed of the motor
  TCCR2B = (1 << CS22) | (1 << CS21) | (0 << CS20);//set the prescelar divide by 256
  /*  Every (1/16000000)*256 = 0.000016s, Timer2 runs one tick,
   *  It takes 0.000016 * 65535 = 1.04856s until overflow
   *  once Timer increased until 0XFFFF, it will overflow
   *  Overflow time: 0.000016 * 65535 = 1.04856 seconds,
   */

  /*--------------------------Timmer setting------------------------*/

  //Set counter to zero, high byte first
  TCNT1H = 0;
  TCNT1L = 0;

  Serial.println("Initialize finish");
}

void delayfor(int dd){
  for(cntr1 = 0;cntr1 < 255;cntr1++){
    for(cntr2 = 0;cntr2 < 255;cntr2++){
      for(cntr3 = 0;cntr3 < dd;cntr3++){
        asm("nop");//does nothing but only wasting time
      }
    }
  }
}

void delaySensor(int dd){
  for(cntr1 = 0;cntr1 < 255;cntr1++){
    for(cntr2 = 0;cntr2 < dd;cntr2++){
        asm("nop");//does nothing but only wasting time
    }
  }
}

void initialtest(){//test all parts of Robots works normally
  servoTurns();
  PORTB |= (1 << PORTB4);//test running, motor 1
  PORTD &= ~(1 << PORTD7);
  PORTC |= (1 << PORTC1);//test running, motor 2
  PORTC &= ~(1 << PORTC0);

  OCR2B = 100;//set rate of work for motor2 out of possbiel range 0~255
  OCR2A = 100;//set rate of work for motor2 out of possbiel range 0~255
  delayfor(50);
  //shut down all of them
  PORTD &= ~(1<<PORTD7);
  PORTB &= ~(1<<PORTB4);
  PORTC &= ~(1<<PORTC1);
  PORTC &= ~(1<<PORTC0);

  PORTB |= (1 << PORTB1);//TURN ON LED BOTH
  PORTB |= (1 << PORTB2);
  delayfor(20);
  PORTB &= ~(1 << PORTB1);//TURN OFF LED BOTH
  PORTB &= ~(1 << PORTB2);

}

void servoTurns(){

    OCR0A = 22;//16.38 * (22/256) = 1.5ms pulse, rectify the servo to initial point
    delayfor(10);//0.1s = 60, 0.3s = 180
    OCR0A = 7;//16.31 * (7/255) = 1ms pulse, turns all the way to the right
    delayfor(10);
    OCR0A = 37;//16.31 * (37/255) = 2ms pulse, turns all the way to the left
    delayfor(10);
    OCR0A = 22;//16.38 * (22/256) = 1.5ms pulse, rectify the servo to initial point

}

void sensor(){
  Serial.begin(9600);
  PORTD |= (1 << PORTD2); //set HIGH at pin1, "start sending wave"
  //digitalWrite(trig, HIGH);//launch the pulse sent to echo
  TCNT1 = 0;//start counting, initialize
  ICR1 = 0;
  int TTime = 0;
  delaySensor(4);
  PORTD &= ~(1 << PORTD2); //set LOW at pin1, "close the trig"

  while(ICR1 == 0){//once the input capture unit state change
    //wait until the ICR1 changed, that represent echo received the wave, break the loop
  }
  TTime = TCNT1;//the time duration
  long int Time = TTime * ((1000000 * 256) / 16000000);
  /*calculate the Time by sending the until receive,
   * the formula is: one tick running time = 1/1600,0000 * 64 = 0.000004(s)
   * Transfer to the Unit - us is: 0.000004 * 1000000 = 4(us)
   * Thus, the time equals to: Time = TCNT1 * 4us = how much us cost from echo send wave until trigger receive the wave
   */
//  Serial.print("Time is: ");
//  Serial.print(Time);
//  Serial.println(" US");
  distance = (Time * 0.00034 * 100)/ 2 - 10;//The distance in centimeters
  Serial.print(distance);
  Serial.println(" cm");

}

void forward(int i, int j){
  // this function will run the motors in both same directions at a fixed speed
  //MOVE FORWARD
  OCR2A = i;//set rate of work for left motor out of possbiel range 0~255, activate the Enable A pin
  PORTB |= (1 << PORTB4);//set up HIGH OUTPUT mode at PORTB on the pin 12, int1
  PORTD &= ~(1 << PORTD7); //set up LOW OUTPUT mode at PORTD on the pin7, int2

  OCR2B = j;//set rate of work for right motor out of possbiel range 0~255, activate the Enable B pin
  PORTC |= (1 << PORTC1); //set up HIGH OUTPUT mode at PORTD on the analogpin1 , int 3
  PORTC &= ~(1 << PORTC0); //set up LOW OUTPUT mode at PORTD on the analogpin0, int 4

}

void backward(int i, int j){
  // this function will run the motors in both same directions at a fixed speed
  //MOVE BACKWARD
  OCR2A = i;//set rate of work for left motor out of possbiel range 0~255, activate the Enable A pin
  PORTD |= (1 << PORTD7); //set up HIGH OUTPUT mode at PORTD on the pin13, int2
  PORTB &= ~(1 << PORTB4); //set up LOW OUTPUT mode at PORTB on the pin12, int1

  OCR2B = j;//set rate of work for right motor out of possbiel range 0~255, activate the Enable B pin
  PORTC |= (1 << PORTC0); //set up HIGH OUTPUT mode at analogpin0, int 4
  PORTC &= ~(1 << PORTC1); //set up LOW OUTPUT mode at analogpin1 , int 3
}

void stoprunning(){
  PORTD &= ~(1<<PORTD7);//shut them down
  PORTB &= ~(1<<PORTB4);
  PORTC &= ~(1<<PORTC1);
  PORTC &= ~(1<<PORTC0);
}

void turnsBigleft(){
  //turns little left
  //left wheel go backward

  OCR2A = 145;//set rate of work for left motor out of possbiel range 0~255, activate the Enable A pin
  PORTD |= (1 << PORTD7); //set up HIGH OUTPUT mode at PORTD on the pin13, int2
  PORTB &= ~(1 << PORTB4); //set up LOW OUTPUT mode at PORTB on the pin12, int1

  //right wheel go forward

  OCR2B = 145;//set rate of work for right motor out of possbiel range 0~255, activate the Enable B pin
  PORTC |= (1 << PORTC1); //set up HIGH OUTPUT mode at analogpin1, int 3
  PORTC &= ~(1 << PORTC0); //set up LOW OUTPUT mode at analogpin0, int 4
}

void turnsLittleleft(){
  //turns little left
  //left wheel go backward

  OCR2A = 110;//set rate of work for left motor out of possbiel range 0~255, activate the Enable A pin
  PORTD |= (1 << PORTD7); //set up HIGH OUTPUT mode at PORTD on the pin13, int2
  PORTB &= ~(1 << PORTB4); //set up LOW OUTPUT mode at PORTB on the pin12, int1

  //right wheel go forward

  OCR2B = 100;//set rate of work for right motor out of possbiel range 0~255, activate the Enable B pin
  PORTC |= (1 << PORTC1); //set up HIGH OUTPUT mode at analogpin1, int 3
  PORTC &= ~(1 << PORTC0); //set up LOW OUTPUT mode at analogpin0, int 4

}

void turnsBigright(){
  //turns big right
  //Left wheel go forward

  OCR2A = 145;//set rate of work for motor1 out of possbiel range 0~255, activate the Enable A pin
  PORTB |= (1 << PORTB4);//set up LOW OUTPUT mode at PORTB on the pin12, int1
  PORTD &= ~(1 << PORTD7); //set up HIGH OUTPUT mode at PORTD on the pin7, int2

  //right wheel go backward

  OCR2B = 145;//set rate of work for right motor out of possbiel range 0~255, activate the Enable B pin
  PORTC |= (1 << PORTC0); //set up HIGH OUTPUT mode at analogpin0, int 4
  PORTC &= ~(1 << PORTC1); //set up LOW OUTPUT mode at analogpin1, int 3
}

void turnsLittleright(){
  //turns little right
  //Left wheel go forward

  OCR2A = 110;//set rate of work for motor1 out of possbiel range 0~255, activate the Enable A pin
  PORTB |= (1 << PORTB4);//set up LOW OUTPUT mode at PORTB on the pin12, int1
  PORTD &= ~(1 << PORTD7); //set up HIGH OUTPUT mode at PORTD on the pin7, int2

  //right wheel go backward

  OCR2B = 90;//set rate of work for right motor out of possbiel range 0~255, activate the Enable B pin
  PORTC |= (1 << PORTC0); //set up HIGH OUTPUT mode at analogpin0, int 4
  PORTC &= ~(1 << PORTC1); //set up LOW OUTPUT mode at analogpin1, int 3

}

void speedup(int i, int j){
  OCR2A = OCR2A + i;
  CURRENTSPEEDA = OCR2A;
  OCR2B = OCR2B + i;
  CURRENTSPEEDB = OCR2B;
}

void slowdown(int i, int j){
  OCR2A = OCR2A - i;
  CURRENTSPEEDA = OCR2A;
  OCR2B = OCR2B - i;
  CURRENTSPEEDB = OCR2B;
}
