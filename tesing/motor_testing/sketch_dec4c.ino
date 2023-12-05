

#include <SparkFun_TB6612.h>  


#define AIN1 4   //6
#define BIN1 6   //4
#define AIN2 5   //7
#define BIN2 7   //5
#define PWMA 3   //9
#define PWMB 9   //3
#define STBY 8
const int offsetA = 1;
const int offsetB = 1;

#define sw1 11
#define sw2 12
#define led 2
int s1,s2;


Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);

void setup() {
  pinMode(sw1, INPUT);
  pinMode(sw2,INPUT);
  pinMode(10,OUTPUT);
  pinMode(led, OUTPUT);
  digitalWrite(10,HIGH);

   s1 = digitalRead(sw1);  //s1 for running bot
  while (s1 == HIGH)
  {
    s1 = digitalRead(sw1);    
  }
  digitalWrite(2,HIGH);

}

void loop() {
  // put your main code here, to run repeatedly:
  forward(motor1, motor2, 255);
  delay(400);
  back(motor1, motor2, 255);
  delay(400);
  brake(motor1, motor2);
  delay(50);
  left(motor1, motor2, 200); 
  delay(400);
  right(motor1, motor2, 400);
  delay(400);
}
