
#define sw1 11
#define sw2 12
#define led 2
int s1,s2;

void setup() {
  pinMode(sw1, INPUT);
  pinMode(sw2,INPUT);
  pinMode(10,OUTPUT);
  pinMode(led, OUTPUT);
  digitalWrite(10,HIGH);

}

void loop() {
  s1 = digitalRead(sw1);  //press s1 to switch on led
  while (s1 == HIGH)
  {
    s1 = digitalRead(sw1);    
  }
  digital(led,HIGH);
  delay(400);


  s2 = digitalRead(sw2);  //press s2 to switch off led
  while (s2 == HIGH)
  {
    s2 = digitalRead(sw2);    
  }
  digital(led,LOW);
}
