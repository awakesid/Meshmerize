
// Download below two library on your arduino IDE you will get this from google
#include <SparkFun_TB6612.h>  // This libaray is for sparkfun motor driver  
#include <QTRSensors.h>       // It is for qtr sensors 

// Initialization of the motors
#define AIN1 6
#define BIN1 4
#define AIN2 7
#define BIN2 5
#define PWMA 9
#define PWMB 3
#define STBY 8

const int offsetA = 1;
const int offsetB = 1;

// Initialization of the controls
#define sw1 11
#define sw2 12
#define led 2
int s1;




// Initialization of sensors
#define NUM_SENSORS 8
unsigned int sensors1[8];
int thr[8];

// Initialization of PID parameter
#define MaxSpeed 250
#define BaseSpeed 250
int lastError = 0;
float kp = 0.1;    // It fully depends on the bot system
float kd = 0.3;    // Please follow the method provided in instructables to get your values
int last_pos = 3500;




// Creating the instance of class for motor and senors
Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);
QTRSensors qtra;

void setup()
{
  qtra.setTypeAnalog();
  qtra.setSensorPins((const uint8_t[]) {   // QTR8 sensor setup
    A0, A1, A2, A3, A4, A5, A6, A7
  }, NUM_SENSORS);

  pinMode(sw1, INPUT);
  pinMode(sw2, INPUT);
  pinMode(led, OUTPUT);     // Control pin setup
  pinMode(13, OUTPUT);
  digitalWrite(10, HIGH);
  digitalWrite(13, LOW);

  Serial.begin(9600);
 

  s1 = digitalRead(sw1);
  while (s1 == HIGH)
  {
    s1 = digitalRead(sw1);      // Calibration phase where the bot get calibrated after pressing s1 switch
  }
  calibration();
  
}
void loop() {
  forward(motor1, motor2, 60);
  delay(40);
  forward(motor1, motor2, 80);
  delay(40);                              // For graduly increasing speed so that bot dose not get direct high speed at the start
  forward(motor1, motor2, 100);
  delay(40);
  s1 = digitalRead(sw1);
  while (s1 == HIGH)
  {
    s1 = digitalRead(sw1);      //s1 to start following line
  }
  follow_segment1();
}
void calibration()
{

  for (int i = 0; i <= 100; i++)
  {

    if (i < 25 || i >= 75)
    {
      left(motor1, motor2, 255); //Left turn

    }
    else
    {

      right(motor1, motor2, 255); //Right turn
    }

    qtra.calibrate();
    delay(10);
  }  // end calibration cycle
  brake(motor1, motor2);
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtra.calibrationOn.minimum[i]);
    Serial.print(' ');
    thr[i] = (qtra.calibrationOn.minimum[i] + qtra.calibrationOn.maximum[i]) / 2;
    // Calculating the threshold value for making the decision above thr black line and below white line
  }

  Serial.println();
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtra.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println(thr[0]);
  Serial.println(thr[7]);

}






void follow_segment1()
{
    int position = qtra.readLineWhite(sensors1);       // FOR BLACK LINE FOLLOWER JUST REPLACE White WITH Black
    int error =  3500 - position;
    int motorSpeed = kp * error + kd * (error - lastError);
    lastError = error;
    int rightMotorSpeed = BaseSpeed - motorSpeed;
    int leftMotorSpeed = BaseSpeed + motorSpeed;
    if (rightMotorSpeed > MaxSpeed ) rightMotorSpeed = MaxSpeed;
    if (leftMotorSpeed > MaxSpeed ) leftMotorSpeed = MaxSpeed;
    if (rightMotorSpeed < 0)rightMotorSpeed = 0;
    if (leftMotorSpeed < 0)leftMotorSpeed = 0;
    motor1.drive(rightMotorSpeed);
    motor2.drive(leftMotorSpeed);
    delay(1);
  }


