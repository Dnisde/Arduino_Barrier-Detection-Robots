# R2 Smart Robot of Car Assembly

---

 - In this project, I basically designed and build a Real-time Robotics embebdded system Robots, which integrating the pulse-width modulation (PWM) and other features of techniques that allowing the car avoid barriers when its running on a specific routes. 
 - It use one of the 8-bit timers to send create two PWM signals to drive the left and right motors on your robot through your L2989N H-Bridge without using the Arduino servo libraries. 
 
---

# Pre-request (Before you start the project)

**Note:** If you are trying follow the basic instructions to re-implementing the project, you will need to manually configure another Timers to generate two output compare PWM signals to control the speed of your motors, and also other two Timers to control the sensor and servo to detect the obstacle for the following steps of your robot. 

1. Check out each part of keys that did included in your Robot keys(R2 Smart Car)
2. Make sure you understand the datasheets with each assembly kernels, such as L298N and sensor, the datasheets has been attached into a seperate folders
3. Install and initialize the environment, ex: Arduino, C programming..
4. Of course, the last and most important, a programmable and basic computer to support your hard works

---

# Installing and Set Environment 

Next, you will need to set-up the environment first

1. Arduino software installed, downloaded from (https://www.arduino.cc/)
2. Robot assembly and wire connection, you could design by your own or search for some good instances to help you figure out the wire connection. Trust me, it is not hard like you thought about it. Here is a good example (https://blog.miguelgrinberg.com/post/building-an-arduino-robot-part-ii-programming-the-arduino)
3. Read the data sheet carefully, the different kernals of the robot, such as the motors and servo, they probably need the different timers to build up and initialize. For here, I used the timer 1 and 0 as prescaler which by using into the PWM control to each of them.


Before you move on to read the guideline, just make sure you have already understand what I described above..

---

# Code Translation and development 

As far as you have understand the works that you should do, I will starting to teach you how to understand my code and concrete it by yourself

1. `void setup()`: Initialize function, maybe changed by the requirement**
2. `void sensor()`: Sensor functional setting-up and initializing
3. `void forward()`: Running forward function, at a constant speed
4. `void backward()`: Running backward function, at a constant speed
5. `void turnsBigleft()`: Running left turn in large degrees, setting the motors in different speed to control it move left
6. `void turnsBigright()`: Running right turn in large degrees, similar as the left one
7. `void turnsLittleleft()`: Running left turn in small degrees, setting the motors in different direction and speed to control it move slightly
8. `void turnsLittleright()`: Running right turn in small degrees, similar as the left one **

9. `void servoTurns()`: testing the servo function, try 0, 90 and 180 degrees for test
10. `void runForward()`: // Make the robot running forward at a constant lower speed
11. `void RunSidefwallRight()`: // Wall obstacle test function; Avoid the obstavle wall on the right side. Between 40cm to 50cm distance from the wall
12. `void obstacleV1()`: // Avoid collition function. In debugging...**

--- 

## About the Timmer and prescaler set-up: 

Timer 0: 
 
`TCCR0A = (1 << COM0A1) | (1 << COM0B1) | (1 << WGM01) | (1 << WGM00);`

`TCCR0B = (1 << CS02) | (0 << CS01) | (1 << CS00);`

**Explain: 
 - Set up the fast PWM mode, and enable convert for register A
 - Set the prescelar divide by 1024.
 - The frequency = 16M / 256/ 1024 = 61.035(Hz), which is the most closest one to 50Hz
 - The period = 1/61.035Hz = 16.38(ms)**

--- 
Timer 1: 

`TCCR1A = 0;`

`TCR1B = (1 << CS12) | (0 << CS11) | (0 << CS10);`

**Explain:**
- Initial timmer 1, from 0 to 0xFFFF
- Initialize the Timer1, range from 0 to 65535

---
Timer 2: 

`TCCR2A = (1 << COM2A1) |(1 << COM2B1) | (1 << WGM21) | (1 << WGM20);`

`TCCR2B = (1 << CS22) | (1 << CS21) | (0 << CS20);`
  
**Explain: **
- Set up the fast PWM mode, and enable convert for both register A and B, since OC2A, OC2B
- Use Timer1 to adjust the motor 1 PWM and speed of the motor
- Set the prescelar divide by 256
- The frequency = 16M / 256 = 62500(Hz),
- The period = 1/62500 = 0.016(ms), one click for counter
- Overflow time each cycle = 0.016 * 256 = 4.096(ms)

Bitbucked hint: 

 - You can [push your change back to Bitbucket with SourceTree](https://confluence.atlassian.com/x/iqyBMg), or you can [add, commit this Readme anytime](https://confluence.atlassian.com/x/8QhODQ) and [push from the command line](https://confluence.atlassian.com/x/NQ0zDQ).
