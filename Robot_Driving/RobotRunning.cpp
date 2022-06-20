#include <iostream>
#include <stdio.h>
#include <avr/io.h>
#include <Arduino.h>
#include "robot.h"

// int CURRENTSPEEDA = 80;
// int CURRENTSPEEDB = 90;//for hard material ground

int CURRENTSPEEDA = 75;
int CURRENTSPEEDB = 75;//for blanket material ground

int distance = 0;//updating when each timer sensor checking around
int i = 0;

volatile int LeftD = 0;
volatile int RightD = 0;
volatile int FrontD = 0;

void setup(){
//  TIMSK1 |= (1 << TOIE1);          // Globally Enable interrupt on match with OCR1A
//  sei();
//  delayfor(80);//about 2 seconds
//  delayfor(39);//about 1 seconds
//  delayfor(11);//about 0.3 seconds
//  delayfor(5);//about 0.03seconds
//  delayfor(1);//about 0.01seconds !! most shortest delay for function
  Serial.begin(9600); //open the serial port
  Robotsetup(); //set up and initialize
  initialtest();//testing each function
}

void main(){
  /*Running main code
  First Part Running test
  */
  int i = 1
  while(i == 1){
    runfollowingW();//infinity loop
  }
}



void sensorTest(){//Make sure the accurancy
  for(int i=0;i < 2;i++){
    sensor();
    delayfor(1);
  }
}

void runafter(){
  OCR0A = 7;//16.31 * (7/255) = 1ms pulse, turns all the way to the RIGHT
  delayfor(8);
  sensorTest();
  RightD = distance;
  OCR0A = 22;//16.38 * (22/256) = 1.5ms pulse, rectify the servo to initial point: FRONT
  delayfor(8);
  sensorTest();
  FrontD = distance;
  forward(95, 75);
  /* Until the front distance are close enough to the wall,
   * Or Right wall distance not portable not appropirate,
   * Jump out of loop to testing around.
   */
  while((RightD > 25) && (RightD < 40) && (FrontD > 35)){ // Running until it meet obstacle
    OCR0A = 7;//16.31 * (7/255) = 1ms pulse, turns all the way to the RIGHT
    delayfor(8);
    sensorTest();
    RightD = distance;
    OCR0A = 22;//16.38 * (22/256) = 1.5ms pulse, rectify the servo to initial point: FRONT
    delayfor(8);
    sensorTest();
    FrontD = distance;

  }
  stoprunning();
  obstacleV1();
}

void runfollowingW(){
  OCR0A = 7;//16.31 * (7/255) = 1ms pulse, turns all the way to the RIGHT
  delayfor(8);
  sensorTest();
  RightD = distance;
  OCR0A = 22;//16.38 * (22/256) = 1.5ms pulse, rectify the servo to initial point: FRONT
  delayfor(8);
  sensorTest();
  FrontD = distance;
  forward(CURRENTSPEEDA, CURRENTSPEEDB);  //  Go Forward!
  /* Until the front distance are close enough to the wall,
   * Or Right wall distance not portable not appropirate,
   * Jump out of loop to testing around.
   */
  while((RightD > 25) && (RightD < 40)){
    OCR0A = 7;//16.31 * (7/255) = 1ms pulse, turns all the way to the RIGHT
    delayfor(7);
    sensorTest();
    RightD = distance;
    OCR0A = 22;//16.38 * (22/256) = 1.5ms pulse, rectify the servo to initial point: FRONT
    delayfor(7);
    sensorTest();
    FrontD = distance;
    if(FrontD < 20){
      stoprunning();
      obstacleV1();
    }
  }
  stoprunning();
  obstacleV1();

}


void obstacleV1(){
  /* ----------------------------------------------------
   * Testing around situation and check what kinds of problem met
     ----------------------------------------------------
   */
  stoprunning();//  Make sure it is stop
  OCR0A = 10;//16.31 * (7/255) = 1ms pulse, turns all the way to the RIGHT
  delayfor(10);
  sensorTest();
  RightD = distance;
  OCR0A = 37;//16.31 * (37/255) = 2ms pulse, turns all the way to the LEFT
  delayfor(20);
  sensorTest();
  LeftD = distance;
  OCR0A = 22;//16.38 * (22/256) = 1.5ms pulse, rectify the servo to initial point: FRONT
  delayfor(10);
  sensorTest();
  FrontD = distance;
//  Serial.print("Right, Left and Front distance : ");
//  Serial.println(RightD);
//  Serial.println(LeftD);
//  Serial.println(FrontD);

/* ------------------------------------------------
 * Start Analyzing
 * ------------------------------------------------
 */

  if(FrontD > 20){
  /* -------------------------Forward is safety--------------------------*/
    sensorTest();
    FrontD = distance; // double make sure the environment

    if((FrontD > 15) & (FrontD < 60) & (RightD < 200)){
    /*--------------------------------------------------------
     * The third corner, right wall exist, face to the wall
     */
      turnsBigleft();// turn to Left at 90 degree
      delayfor(14);
      stoprunning();
      delayfor(20);
      forward(150, 70);
      delayfor(39);
      stoprunning();
      delayfor(15);
      obstacleV1();
    }
    else if((FrontD > 15) & (FrontD < 60) & (RightD > 200)){
      turnsBigright();// turn to Left at 90 degree
      delayfor(12);
      stoprunning();
      delayfor(20);
      forward(70, 150);
      delayfor(39);
      stoprunning();
      delayfor(15);
      obstacleV1();
    }

   /* if(( (LeftD < 25) || (RightD < 25) ) && (FrontD > 20)) {
     // Too close to the left wall and right wall, too !
     OCR0A = 22;//16.38 * (22/256) = 1.5ms pulse, rectify the servo to initial point: FRONT
     delayfor(15);
     sensorTest();
     if(distance > 25){
       forward(CURRENTSPEEDA, CURRENTSPEEDB);
       delayfor(13);
       stoprunning();
       obstacleV1();
     }else{
       backward(CURRENTSPEEDA, CURRENTSPEEDB);
       delayfor(11);
       stoprunning();
       obstacleV1();
     }
   } 
  */

    if((RightD < 25) && FrontD > 20){
    /*--------------------------------------------------------
     * Too close to the right wall
     */
      turnsLittleleft();
      delayfor(4);
      stoprunning();
      delayfor(15);
      forward(CURRENTSPEEDA, CURRENTSPEEDB);
      delayfor(13);
      stoprunning();
      delayfor(15);
      turnsLittleright();
      delayfor(4);
      stoprunning();
      delayfor(15);
      sensorTest();
      FrontD = distance; //Double check with front to see what's happening
      /*-----Until far away from the right wall enough-----*/
      if(FrontD < 20){
        backward(CURRENTSPEEDA, CURRENTSPEEDB);
        delayfor(11);
        stoprunning();
        turnsLittleleft();
        delayfor(4);
        stoprunning();
        delayfor(15);
        forward(CURRENTSPEEDA, CURRENTSPEEDB);
        delayfor(13);
        stoprunning();
        delayfor(15);
      }
      obstacleV1();

    }

    if((RightD > 50) && (RightD < 400) && (FrontD > 20)){
    /*--------------------------------------------------------
     * too far away from the right wall, but within adjustion range
     */
      turnsLittleright();
      delayfor(4);
      stoprunning();
      delayfor(15);
      forward(CURRENTSPEEDA, CURRENTSPEEDB);
      delayfor(13);
      stoprunning();
      delayfor(15);
      turnsLittleleft();
      delayfor(4);
      stoprunning();
      delayfor(15);
      sensorTest();
      FrontD = distance; //Double check with front to see what's happening
      /*------------- Until close enough to the right wall---------------*/
      if(FrontD > 400){
        turnsLittleright();
        delayfor(4);
        stoprunning();
        delayfor(15);
        forward(CURRENTSPEEDA, CURRENTSPEEDB);
        delayfor(13);
        stoprunning();
        delayfor(15);
      }
      obstacleV1();
    }

    if((LeftD < 25) && (FrontD > 20)){
    /*--------------------------------------------------------
     * too close to the left wall, but right wall have enough space
     */
      turnsLittleright();
      delayfor(4);
      stoprunning();
      delayfor(10);
      forward(CURRENTSPEEDA, CURRENTSPEEDB);
      delayfor(27);
      stoprunning();
      delayfor(15);
      obstacleV1();// back to testing, and check around again
    }

    if((LeftD > 25) && (LeftD < 40) && (FrontD > 20)){
     /*--------------------------------------------------------
     * Available space between the left wall, could going on
     */
      sensorTest();
      forward(CURRENTSPEEDA, CURRENTSPEEDB);
      while(distance > 35){ //  If not too close to the front wall, keep going
        sensorTest(); //  Keep tracking in front
      }
      stoprunning();
      delayfor(15);
      obstacleV1();// Testing around situation again
    }

    if((LeftD < 60) && (RightD < 60) && (FrontD > 20)){
    /*---------------------------------------------------
     *              Between the box
     */
      turnsLittleleft();//turn more left than suitable range
      delayfor(7);
      stoprunning();
      delayfor(15);
      forward(CURRENTSPEEDA, CURRENTSPEEDB);
      delayfor(13);
      stoprunning();
      delayfor(15);
      obstacleV1();
    }


    if((RightD > 400) && (LeftD > 400) && (FrontD > 20)){                          /*!!!!!!!!!!!!!!!!!!!!!!!!!sensor messed up!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/
      sensorTest();
      FrontD = distance;
      if(distance > 35){  // Situation 1: when nothing in front of the robot:
        turnsLittleright();
        delayfor(4);
        stoprunning();
        delayfor(15);
        forward(CURRENTSPEEDA, CURRENTSPEEDB);
        delayfor(13);
        stoprunning();
        delayfor(15);
        OCR0A = 7;//16.31 * (7/255) = 1ms pulse, turns all the way to the RIGHT
        delayfor(10);
        sensorTest();
        RightD = distance;
        OCR0A = 37;//16.31 * (37/255) = 2ms pulse, turns all the way to the LEFT
        delayfor(20);//From right to left, it need more time.
        sensorTest();
        LeftD = distance;   //wait until something appears in front,
        if((RightD > 400) && (LeftD > 400)){
          obstacleV1();
        }
        obstacleV1();
      }
      else{              // Situation 2: when something in front of the robot:
        turnsLittleleft();
        delayfor(4);  // Make Adjustion
        stoprunning();
        delayfor(15);
        forward(CURRENTSPEEDA, CURRENTSPEEDB);
        delayfor(13);
        stoprunning();
        delayfor(15);
        OCR0A = 7;//16.31 * (7/255) = 1ms pulse, turns all the way to the RIGHT
        delayfor(10);
        sensorTest();
        RightD = distance;
        OCR0A = 37;//16.31 * (37/255) = 2ms pulse, turns all the way to the LEFT
        delayfor(20);//From right to left, it need more time.
        sensorTest();
        LeftD = distance;   //wait until something appears in front,
        if((RightD > 400) && (LeftD > 400)){
          obstacleV1();
        }
        obstacleV1();
      }
    }

    else {/*--------------------------------After correcting the distance from the wall, try to run with after wall function*/
      PORTB |= (1 << PORTB1);//TURN ON LED BOTH
      PORTB |= (1 << PORTB2);
      delayfor(20);
      PORTB &= ~(1 << PORTB1);//TURN OFF LED BOTH
      PORTB &= ~(1 << PORTB2);
      turnsLittleleft();
      delayfor(4);  // Make Adjustion
      stoprunning();
      delayfor(15);
      forward(CURRENTSPEEDA, CURRENTSPEEDB);
      delayfor(13);
      stoprunning();
      delayfor(15);
      runafter();
    }
  }

  /* -------------------------Forward is TOO CLOSE!!!!!!!--------------------------*/
  else{
    backward(CURRENTSPEEDA, CURRENTSPEEDB);
    delayfor(12);
    stoprunning();
    delayfor(20);
    sensorTest();

    if((RightD < 70) && (FrontD < 20)){
    /* ----------------------------------------
     * First Corner, the right wall exist and close, have obstacle in front
     */
      turnsBigleft();// turn to Left at 90 degree
      delayfor(13);
      stoprunning();
      delayfor(20);
      forward(160, 65);
      delayfor(39);
      stoprunning();
      delayfor(20);
      OCR0A = 7;//16.31 * (7/255) = 1ms pulse, turns all the way to the RIGHT
      delayfor(10);
      sensorTest();
      RightD = distance;
      delayfor(5);

    }

   /* 
   if((RightD > 20)
   {
   // If wall does not exist
       turnsBigright();// turn to Right at 90 degree
       delayfor(14);
       stoprunning();
       delayfor(20);
       forward(70, 150);
       delayfor(49);
       stoprunning();
    }
    */

    if((RightD > 70) && (RightD < 400) && (LeftD > 100)){
    /*--------------------------------------------------
     * Second corner, the right wall is not close, turn in that way!
     */
      turnsBigright();// turn to Right at 90 degree
      delayfor(12);
      stoprunning();
      delayfor(20);
      forward(70, 150);
      delayfor(39);
      stoprunning();
      delayfor(10);
      obstacleV1();
    }

    if((RightD > 400) && (LeftD < 400)){
    /*--------------------------------------------------
     * Accident, left wall exist, turn around!
     */
      turnsBigleft();// turn around at 180 degree
      delayfor(25);
      stoprunning();
      delayfor(20);
      forward(160, 70);
      delayfor(49);
      stoprunning();
      delayfor(10);
      obstacleV1();

    }

    if((RightD > 400) && (LeftD > 400) && (FrontD < 20)){
      sensorTest();
      if(distance > 35){  // Situation 1: when nothing in front of the robot:
        turnsLittleright();
        delayfor(4);
        stoprunning();
        delayfor(15);
        forward(CURRENTSPEEDA, CURRENTSPEEDB);
        delayfor(13);
        stoprunning();
        delayfor(15);
        OCR0A = 7;//16.31 * (7/255) = 1ms pulse, turns all the way to the RIGHT
        delayfor(7);
        sensorTest();
        RightD = distance;
        OCR0A = 37;//16.31 * (37/255) = 2ms pulse, turns all the way to the LEFT
        delayfor(15);//From right to left, it need more time.
        sensorTest();
        LeftD = distance;   //wait until something appears in front,
        if((RightD > 400) && (LeftD > 400)){
          obstacleV1();
        }
        obstacleV1();
      }
      else{              // Situation 2: when something in front of the robot:
        turnsLittleleft();
        delayfor(4);  // Make Adjustion
        stoprunning();
        delayfor(15);
        forward(CURRENTSPEEDA, CURRENTSPEEDB);
        delayfor(13);
        stoprunning();
        delayfor(15);
        OCR0A = 7;//16.31 * (7/255) = 1ms pulse, turns all the way to the RIGHT
        delayfor(7);
        sensorTest();
        RightD = distance;
        OCR0A = 37;//16.31 * (37/255) = 2ms pulse, turns all the way to the LEFT
        delayfor(15);//From right to left, it need more time.
        sensorTest();
        LeftD = distance;   //wait until something appears in front,
        if((RightD > 400) && (LeftD > 400)){
          obstacleV1();
        }
        obstacleV1();
      }
    }

    else{/*--------------------------------After correcting the distance from the wall, try to run with after wall function*/
      PORTB |= (1 << PORTB1);//TURN ON LED BOTH
      PORTB |= (1 << PORTB2);
      delayfor(20);
      PORTB &= ~(1 << PORTB1);//TURN OFF LED BOTH
      PORTB &= ~(1 << PORTB2);
      turnsLittleleft();
      delayfor(4);  // Make Adjustion
      stoprunning();
      delayfor(15);
      forward(CURRENTSPEEDA, CURRENTSPEEDB);
      delayfor(13);
      stoprunning();
      delayfor(15);
      runafter();
    }

  }
}

/* 
ISR(TIMER1_COMPA_vect) {
 //stoprunning();
 Serial.println("Interrupt happens");
 TIFR1 |= (1 << TOV1);         // Clear Timer/Counter1, Output Compare A Match Flag by writing 1
 //PORTB |= (1 << PORTB2);//Red light turns on for detection
}
*/
