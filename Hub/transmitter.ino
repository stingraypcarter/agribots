
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(7, 8); // CE, CSN

const byte address[6] = "00001";
const int numReadings = 10;
int sensorPort = A0;
int sensorValue = 0;
float readings[numReadings];
int readIndex = 0;              // the index of the current reading
float total = 0;                  // the running total
float average = 0;                // the average
float averageFloat = 0;
struct MyData {
  byte averageFloat;
};
MyData data;

void setup() {
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = 0;
  }
}

void loop() {
  total = total - readings[readIndex];
  // read from the sensor:
  sensorValue = analogRead(sensorPort);
  readings[readIndex] = sensorValue;
  // add the reading to the total:
  total = total + readings[readIndex];
  // advance to the next position in the array:
  readIndex = readIndex + 1;

    // if we're at the end of the array...
  if (readIndex >= numReadings) {
    // ...wrap around to the beginning:
    readIndex = 0;
  }

  // calculate the average:
  average = total / numReadings;
  data.averageFloat = 100 - ((average/1023) * 100);

//  const char text[] = "1 5 1 5";
//  radio.write(&text, sizeof(text));
  radio.write(&data, sizeof(MyData));
  Serial.print(averageFloat);
//  delay(1);
}
