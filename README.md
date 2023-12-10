# Meshmerize
**A bot that can solve the maze.**
Meshmerize is a bot that uses sensors to read the maze and find the shortest path. It is the best project to learn electronics and test your coding skills.


# RULE

You can make anything that can solve a maze with a certain size limit and shouldn't be controlled by any wireless controller.**Refer IIT Bombay Meshmerize rule book**

## Our bot

![img](https://github.com/sito-g/Meshmerize/blob/main/images/DSC00249.JPG)

## Electronics

1. N20 motor 3v, 150 rpm*2
2. Arduino Nano
3. 8-channel analog IR sensor(QTR)
4. Voltage regulator 3v,5v,12v
5. Voltage booster.
6. Power Supply(li-ion 3.7)*2
7. Tb6612fng dual channel motor driver



#### Power Supply:-
The bot is powered by two 3.7V lithium-ion batteries in series and provides sufficient current to power all the components and drive motors smoothly for a sufficiently longer time. 

#### Voltage Regulator:-
When the robot turns, one motor must rotate faster than the other to execute the turn. This creates a differential load on the motors. The motor with the higher load may draw more current, potentially causing a voltage drop in the power supply lines. The voltage drop is proportional to the current flowing through the resistance, according to Ohm's Law (V = I * R). In the case of a turning robot, the increased load on one motor can lead to a higher current draw and, consequently, a voltage drop. Voltage drops can impact the performance of the motors. Motors typically have optimal operating voltage ranges, and if the voltage supplied to them drops below this range, it can result in reduced speed, torque, and overall efficiency. The LM2698 is a voltage booster (step-up converter) that can regulate and increase the voltage supplied to the motors. In the context of a turning robot, the voltage booster can help maintain a stable and sufficient voltage level, even when one motor is experiencing a higher load. The voltage booster monitors the input voltage and adjusts the output voltage as needed. This ensures that both motors receive a consistent and adequate voltage supply, even during turns, helping to prevent performance degradation. Using a voltage booster can mitigate the impact of voltage drops, providing the motors with the necessary voltage for optimal performance. 

For more on Electronics, you can refer [Meshmerize](https://medium.com/@creatorallx/electronics-of-line-maze-solving-bot-meshmerize-14ccb870455a)
  
   


![img](https://github.com/sito-g/Meshmerize/blob/main/images/DSC00250.JPG)

## Required libraries
1. Tb6612fng  [sparkfun github](https://github.com/sparkfun/SparkFun_TB6612FNG_Arduino_Library)
2. QTR sensor [qtr sensors](https://www.arduinolibraries.info/libraries/qtr-sensors)



## PID
PID stands for proportional, Integration, and Derivative. It is a closed-loop control system used for our bot to stay bot exactly at the center of the line. It is a mathematical algorithm that has some constant value multiplied with an error generated using sensors to properly function the bot. The constant values are **kp**,**kd**, and **ki**. These values depend upon the building of the bot and the environment.

**PID for our bot**

weight value=3500(the value given by sensor when it is exactly at the middle of the line)

error valve=3500-weight value(may be negative or positive depends upon the position of sensor)

after getting the error value we can calculate the motor speed using the formula(kp*error + kd*change in error + ki*error over time)














