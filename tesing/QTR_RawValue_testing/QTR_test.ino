/*
 * QTR-8A Analog Sensor Test using QTRSensors Library
 * For analog reflectance sensors
 */

#include <QTRSensors.h>

QTRSensors qtr;
const uint8_t sensorCount = 8;
uint16_t sensorValues[sensorCount];

void setup() {
  Serial.begin(9600);
  Serial.println("QTR-8A Analog Sensor Test with Library");
  
  analogReference(EXTERNAL);
  
  // Configure for analog sensors
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, sensorCount);
  
  // Optional: Set analog reference (default is 5V)
  // analogReference(DEFAULT); // 5V
  //analogReference(INTERNAL); // 1.1V (for 3.3V systems)
  
  delay(1000);
}

void loop() {
  // Read analog sensors using library
  qtr.read(sensorValues);
  
  // Print all sensor values
  for (uint8_t i = 0; i < sensorCount; i++) {
    Serial.print(sensorValues[i]);
    Serial.print("\t");
  }
  Serial.println();
  
  delay(100);
}
