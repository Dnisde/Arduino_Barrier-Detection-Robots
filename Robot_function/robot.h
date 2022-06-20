#ifndef ROBOT_H
#define ROBOT_H

#include <avr/io.h>

extern int CURRENTSPEEDA;
extern int CURRENTSPEEDB;
extern int distance;
extern int i;

void Robotsetup(); // Initialize function, maybe changed by the requirement
void sensor(); // Sensor functional setting-up and initializing
void initialtest();//Initial test robot function
void forward(int i, int j); // Running forward function
void backward(int i, int j); // Running backward function
void stoprunning();//stop Running
void turnsBigleft(); // Running left turn in large degrees
void turnsBigright(); // Running right turn in large degrees
void turnsLittleleft(); // Running left turn in small degrees
void turnsLittleright(); // Running right turn in small degrees
void delayfor(int dd);// delay for sometime
/*Real environment implement function */
void speedup(int i, int j); // speed adjustion ++
void slowdown(int i, int j); // speed adjustion --
void obstacleV1();  // avoid obstacle function
void runfollowingW(); // Running the right side of wall function
void stoprunning(); // Stop the motors for both
//void RunSidefwallRight(); // Wall obstacle test function; Avoid the obstavle wall on the right side
//void obstacleV1(); // Avoid collition function. In debugging...
void servoTurns(); // Servo functional test, 0, 90, 180 degrees turns test


#endif
