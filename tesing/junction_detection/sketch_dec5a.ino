

//glowing led whenever a junction is detected

#include <QTRSensors.h>

//sensors
#define NUM_SENSORS 8
unsigned int sensors1[8];
int thr[8];

QTRSensors qtra;

#define sw1 11
#define sw2 12
#define led 2
int s1,s2;

void setup() {
  Serial.begin(9600);
  qtra.setTypeAnalog();
  qtra.setSensorPins((const uint8_t[]) {   // QTR sensor setup
    A0, A1, A2, A3, A4, A5, A6, A7
  }, NUM_SENSORS);

   pinMode(sw1, INPUT);
  pinMode(sw2,INPUT);
  pinMode(10,OUTPUT);
  pinMode(led, OUTPUT);
  digitalWrite(10,HIGH);

    s1 = digitalRead(sw1);  //s1 for calibration
  while (s1 == HIGH)
  {
    s1 = digitalRead(sw1);    
  }
  digitalWrite(led,HIGH);
  delay(800);
  calibration();
  digitalWrite(led,LOW);


}

void loop() {
  chk_junction();
  digitalWrite(led,HIGH);

}

void calibration()
{
  for (int i = 0; i <= 100; i++)
  {
    qtra.calibrate();
    delay(10);
  }  // end calibration cycle
 
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


void chk_junction(){
digitalWrite(led,LOW);
 while(1){
  int position = qtra.readLineWhite(sensors1);   // Getting the present position of the bot
  if ((sensors1[0] < thr[0]) || (sensors1[7] < thr[7]))
    {
      return ;
      // Found an junction
    }
    else if (sensors1[1] > thr[1] && sensors1[2] > thr[2] && sensors1[3] > thr[3] && sensors1[4] > thr[4] && sensors1[5] > thr[5] && sensors1[6] > thr[6])
    {
      // There is no line visible ahead, and we didn't see any
      // intersection. Must be a dead end.
      return ;
    }

 }

}





