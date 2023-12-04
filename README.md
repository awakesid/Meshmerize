# Meshmerize
A bot that can solve the maze. 

﻿# Introduction

Meshmerize is a bot that uses sensors to read the maze and find the shortest path. It is the best project to learn electronics and test your coding skills.


# RULE

You can make anything that can solve a maze with a certain size limit and shouldn't be controlled by any wireless controller.**Refer IIT Bombay Meshmerize rule book**

## Our bot



## ELectronics

1. N20 motor 3v, 150 rpm*2
2. Arduino Nano
3. 8-channel analog IR sensor(QTR)
4. Voltage regulator 3,v,5v,12v
5. Voltage booster.
6. Power Supply(lipo 3.7)*2
7. Tb6612fng dual channel motor driver 


## Required libraries
1. Tb6612fng  [sparkfun github](https://github.com/sparkfun/SparkFun_TB6612FNG_Arduino_Library)
2. QTR sensor [qtr sensors](https://www.arduinolibraries.info/libraries/qtr-sensors)



## PID
PID stands for proportional, Integration, and Derivative. It is a closed-loop control system used for our bot to stay bot exactly at the center of the line. It is a mathematical algorithm that has some constant value multiplied with an error generated using sensors to properly function the bot. The constant values are **kp**,**kd**, and **ki**. These values depend upon the building of the bot and the environment.

**PID for our bot**

weight value=3500(the value given by sensor when it is exactly at the middle of the line)

error valve=3500-weight value(may be negative or positive depends upon the position of sensor)

after getting the error value we can calculate the motor speed using the formula(kp*error + kd*change in error + ki*error over time)














