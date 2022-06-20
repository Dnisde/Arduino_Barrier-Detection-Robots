
#include <avr/io.h>
#include <Arduino.h>
#include "robot.h"

int distance = 0;
int i = 0;

void Robotsetup() {
  Serial.begin(9600); //open the serial port

  /*--------------Pin Settings------------------*/
  DDRB |= (1 << DDB1) | (1 << DDB2) | (1 << DDB3) | (1 << DDB4) | (1 << DDB5);
  /*
   * set direction I/O register at digital pin 9(pwm) on outputmode(led green),
   * set direction I/O register at digital pin 10(pwm) on outputmode(led red),
   * set direciton I/O register at digital pin 11(pwm) on outputmode(Enable A),
   * set direciton I/O register at digital pin 12 on outputmode(int2),
   * set direciton I/O register at digital pin 13 on outputmode(Trig of sensor),
  */

  DDRD |= (1 << DDD3) | (1 << DDD6) | (1 << DDD7);
  /*
   * set direciton I/O register at digital pin 6(pwm) on outputmode(the servo pin),
   * set direciton I/O register at digital pin 3(pwm) on outputmode(Enable B),
   * set direciton I/O register at digital pin 7 on outputmode(int2),
  */
  DDRC |= (1 << DDC0) | (1 << DDC1);
  /*
   *  set direction I/O register at analog pin 0 on outputmode(int 4),
   *  set direction I/O register at analog pin 1 on outputmode(int 3),
   */

  DDRD &= ~(1 << DDD2);
  PORTD |= (1 << PORTD2);
  /*set direction I/O register at digital pin 2 on Input
   * And enable pullup resistor
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

//  TCCR1A = 0;
//  TCCR1B = 0;//write line of code      /* Reset TCCR1A & TCCR1B to defaults */
//  TCCR1B |= (1 << WGM12);//write line of code      /* Set Timer 1 to CTC with OCR1A as top */
//  TCCR1B |= (1 << CS11) | (1 << CS10);//write line of code      /* Set clock prescaler to clk/64 */
//  /* Configure OCR1A (the counter top) to generate an interrupt every 65ms. */
//  OCR1A = 0x1FBD;
//  /*Every (1/16000000)*64 = 0.000004s, Timer1 runs one tick,
//    It takes 0.000004 * 65535 =  0.26214s until overflow
//    But, once Timer increased until OCR1A value, interrupt happens.
//
//    If we want it happens every 65ms:
//
//    OCR1A = 0.065 / 0.000004 = 16250
//    Convert to hex and divided by 2 = 1FBD */

  /*Timer 2 setting for the WHEEL and Motor moving*/
  //set up the fast PWM mode, and enable convert for both register A and B, since OC2A, OC2B.
  TCCR2A = 0;
  TCCR2B = 0;//initial timmer 2, from 0 to 0xFFFF
  TCCR2A = (1 << COM2A1) |(1 << COM2B1) | (1 << WGM21) | (1 << WGM20);//use Timer1 to adjust the motor 1 PWM and speed of the motor
  TCCR2B = (1 << CS22) | (1 << CS21) | (0 << CS20);//set the prescelar divide by 256
  //The frequency = 16M / 256 = 62500(Hz),
  //The period = 1/62500 = 0.016(ms), one click for counter
  //Overflow time each cycle = 0.016 * 256 = 4.096(ms)

  /*--------------------------Timmer setting------------------------*/

  //Set counter to zero, high byte first
  TCNT1H = 0;
  TCNT1L = 0;

  Serial.println("Initialize finish");
}

//void loop() {
//  sensor();
//  forward();
//  RunSidefwallRight();
//
//}

void initialtest(){//test all parts of Robots works normally
  servoTurns();
  PORTB |= (1 << PORTB4);//test running, motor 1
  PORTD &= ~(1 << PORTD7);
  PORTC |= (1 << PORTC1);//test running, motor 2
  PORTC &= ~(1 << PORTC0);

  OCR2B = 200;//set rate of work for motor2 out of possbiel range 0~255
  OCR2A = 200;//set rate of work for motor2 out of possbiel range 0~255
  delay(100);
  //shut down all of them
  PORTD &= ~(1<<PORTD7);
  PORTB &= ~(1<<PORTB4);
  PORTC &= ~(1<<PORTC1);
  PORTC &= ~(1<<PORTC0);

  PORTB |= (1 << PORTB1);//TURN ON LED BOTH
  PORTB |= (1 << PORTB2);
  delay(50);
  PORTB &= ~(1 << PORTB1);//TURN OFF LED BOTH
  PORTB &= ~(1 << PORTB2);

//  sensor();
//  delay(30);
//  sensor();
//  delay(30);
//  sensor();
  delay(30);
}

void servoTurns(){

    OCR0A = 22;//16.38 * (22/256) = 1.5ms pulse, rectify the servo to initial point
    delay(30);//0.1s = 60, 0.3s = 180
    OCR0A = 7;//16.31 * (7/255) = 1ms pulse, turns all the way to the right
    delay(30);
    OCR0A = 37;//16.31 * (37/255) = 2ms pulse, turns all the way to the left
    delay(30);
    OCR0A = 22;//16.38 * (22/256) = 1.5ms pulse, rectify the servo to initial point

}

void sensor(){
  Serial.begin(9600);
  PORTB |= (1 << PORTB5); //set HIGH at pin1, "start sending wave"
  //digitalWrite(trig, HIGH);//launch the pulse sent to echo
  TCNT1 = 0;//start counting, initialize
  ICR1 = 0;
  int TTime = 0;
  delayMicroseconds(10);
  PORTB &= ~(1 << PORTB5); //set LOW at pin1, "close the trig"

  while(ICR1 == 0){//once the input capture unit state change
    //wait until the ICR1 changed, that represent echo received the wave, break the loop
  }
  TTime = TCNT1;//the time duration
  long int Time = TTime * ((1000000 * 64) / 16000000);
  /*calculate the Time by sending the until receive,
   * the formula is: one tick running time = 1/1600,0000 * 64 = 0.000004(s)
   * Transfer to the Unit - us is: 0.000004 * 1000000 = 4(us)
   * Thus, the time equals to: Time = TCNT1 * 4us = how much us cost from echo send wave until trigger receive the wave
   */
//  Serial.print("Time is: ");
//  Serial.print(Time);
//  Serial.println(" US");
  distance = (Time * 0.00034 * 100)/ 2;//The distance in centimeters
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

  OCR2A = 130;//set rate of work for left motor out of possbiel range 0~255, activate the Enable A pin
  PORTD |= (1 << PORTD7); //set up HIGH OUTPUT mode at PORTD on the pin13, int2
  PORTB &= ~(1 << PORTB4); //set up LOW OUTPUT mode at PORTB on the pin12, int1

  //right wheel go forward

  OCR2B = 140;//set rate of work for right motor out of possbiel range 0~255, activate the Enable B pin
  PORTC |= (1 << PORTC1); //set up HIGH OUTPUT mode at analogpin1, int 3
  PORTC &= ~(1 << PORTC0); //set up LOW OUTPUT mode at analogpin0, int 4
}

void turnsLittleleft(){
  //turns little left
  //left wheel go backward

  OCR2A = 130;//set rate of work for left motor out of possbiel range 0~255, activate the Enable A pin
  PORTD |= (1 << PORTD7); //set up HIGH OUTPUT mode at PORTD on the pin13, int2
  PORTB &= ~(1 << PORTB4); //set up LOW OUTPUT mode at PORTB on the pin12, int1

  //right wheel go forward

  OCR2B = 110;//set rate of work for right motor out of possbiel range 0~255, activate the Enable B pin
  PORTC |= (1 << PORTC1); //set up HIGH OUTPUT mode at analogpin1, int 3
  PORTC &= ~(1 << PORTC0); //set up LOW OUTPUT mode at analogpin0, int 4
}

void turnsBigright(){
  //turns big right
  //Left wheel go forward

  OCR2A = 150;//set rate of work for motor1 out of possbiel range 0~255, activate the Enable A pin
  PORTB |= (1 << PORTB4);//set up LOW OUTPUT mode at PORTB on the pin12, int1
  PORTD &= ~(1 << PORTD7); //set up HIGH OUTPUT mode at PORTD on the pin7, int2

  //right wheel go backward

  OCR2B = 140;//set rate of work for right motor out of possbiel range 0~255, activate the Enable B pin
  PORTC |= (1 << PORTC0); //set up HIGH OUTPUT mode at analogpin0, int 4
  PORTC &= ~(1 << PORTC1); //set up LOW OUTPUT mode at analogpin1, int 3
}

void turnsLittleright(){
  //turns little right
  //Left wheel go forward

  OCR2A = 130;//set rate of work for motor1 out of possbiel range 0~255, activate the Enable A pin
  PORTB |= (1 << PORTB4);//set up LOW OUTPUT mode at PORTB on the pin12, int1
  PORTD &= ~(1 << PORTD7); //set up HIGH OUTPUT mode at PORTD on the pin7, int2

  //right wheel go backward

  OCR2B = 100;//set rate of work for right motor out of possbiel range 0~255, activate the Enable B pin
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

//void obstacleV1(){
//  OCR0A = 7;//16.31 * (7/255) = 1ms pulse, turns all the way to the right
//  delay(100);//wait it turns totally to right
//
//  //Run forward!
//  forward(CURRENTSPEEDA, CURRENTSPEEDB);
//}

//void RunSidefwallRight(){
//  OCR0A = 7;//16.31 * (7/255) = 1ms pulse, turns all the way to the right
//  delay(100);//wait it turns totally to right
//
//  //Run forward!
//  forward();
//  sensor();
//  delay(5);
//  sensor();
//  while(distance > 40 & distance < 50){//if it is smaller 31cm,or greater than 40cm, break and do the next step, if not, keep tracking
//    sensor();
//  }
//
//  if(distance < 40){//too close to the right wall
//    Serial.print("To close, Warrning! Distance:");
//    Serial.print(distance);
//    Serial.println(" cm");
//    //stop first
//    PORTD &= ~(1<<PORTD7);//shut them down
//    PORTB &= ~(1<<PORTB4);
//    PORTC &= ~(1<<PORTC1);
//    PORTC &= ~(1<<PORTC0);
//    PORTB |= (1 << PORTB2);//turn on RedLed to warning
//    delay(10);
//    PORTB &= ~(1 << PORTB2);
//    //turn little left
//    turnsBigleft();
//    delay(10);
//    forward();
//  }else if(distance > 50 & distance < 100){//too far away from the right wall
//    Serial.print("To far away, Warrning! Distance:");
//    Serial.print(distance);
//    Serial.println(" cm");
//    //stop first
//    PORTD &= ~(1<<PORTD7);//shut them down
//    PORTB &= ~(1<<PORTB4);
//    PORTC &= ~(1<<PORTC1);
//    PORTC &= ~(1<<PORTC0);
//    PORTB |= (1 << PORTB1);//turn on GreenLed to warning
//    delay(10);
//    PORTB &= ~(1 << PORTB1);
//    //turn Big right
//    turnsLittleright();
//    delay(10);
//    forward();
//  }else if(distance > 200){//out of boundary, detect error
//    //stop motors
//    PORTD &= ~(1<<PORTD7);
//    PORTB &= ~(1<<PORTB4);
//    PORTC &= ~(1<<PORTC1);
//    PORTC &= ~(1<<PORTC0);
//
//    PORTB |= (1 << PORTB1);//accident warning
//    PORTB |= (1 << PORTB2);
//    delay(10);
//    PORTB &= ~(1 << PORTB1);
//    PORTB &= ~(1 << PORTB2);
//
//    while(distance > 200){//when not out of boundary, prove it turn backs to normal position,break the loop and go on
//      sensor();
//      if(distance < 100){//back to normal position
//        break;
//      }
//
//      delay(5);
//
//      turnsBigright();
//      sensor();
//      if(distance < 100){//back to normal position
//        break;
//      }
//
//      delay(5);
//
//      turnsBigright();
//      sensor();
//      if(distance < 100){//back to normal position
//        break;
//      }
//
//      delay(5);
//
//      turnsBigleft();
//      sensor();
//      if(distance < 100){//back to normal position
//        break;
//      }
//      delay(5);
////
////      turnsBigleft();
////      sensor();
////      if(distance < 100){//back to normal position
////        break;
////      }
//
//    }
//  }else{
//    OCR2A = 60;//set rate of work for left motor out of possbiel range 0~255, activate the Enable A pin
//    OCR2B = 60;//set rate of work for right motor out of possbiel range 0~255, activate the Enable B pin
//
//  }
//
//}
