
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "DHT.h"
#include <Adafruit_BMP280.h>
#include <Adafruit_GFX.h>
#include <nRF24L01.h>
#include <RF24.h>


#define DHTPIN 4
#define DHTTYPE DHT22
#define csPin 10 // chip select for SDCard
#define CE_PIN 7 //
#define CSN_PIN 8 //chip select for rf
#define BMP280_I2C_ADDRESS  0x76


RF24 radio(CE_PIN, CSN_PIN); // rf ce,cs
Adafruit_BMP280 bmp; // I2C
const int capteur_D = 3;       //Rain Sensor
const int capteur_A = A0;       //Rain Sensor
const byte address[6] = "00001";  //Address for receiver
int val_analogique;


DHT dht(DHTPIN, DHTTYPE);
File myFile;
struct MyData{
  byte averageFloat;
};
MyData data;
void setup() {

  Serial.begin(9600);
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
  bmp.begin();        //Begin the sensor
  pinMode(capteur_D, INPUT);
  pinMode(capteur_A, INPUT);
// Open serial communications and wait for port to open:
  dht.begin();

  while (!Serial) {
; // wait for serial port to connect. Needed for native USB port only
}
  Serial.print("Initializing SD card...");
  if (!SD.begin(10)) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");

}

void loop() {
    float sensorValue = analogRead(A2);
    float voltage = (sensorValue / 1023) * 5;
    float wind_speed = mapfloat(voltage, 0.4, 2, 0, 32.4);
    float speed_mph = ((wind_speed *3600)/1609.344);

    delay(1000);

    myFile = SD.open("test10.txt", FILE_WRITE);


    if (myFile) {
      Serial.println("************************************");
      Serial.println("Sample Start");
      Serial.print("Pressure = ");
      Serial.print(bmp.readPressure()/100);
      Serial.println(" hPa");
      myFile.print("Pressure = ");
      myFile.print(bmp.readPressure()/100);   //Save altitude into SD card
      myFile.println(" hPa");
      Serial.print("Temperature = ");
      myFile.print("Temperature = ");
      Serial.print(dht.readTemperature());
      Serial.println(" C");
      myFile.print(dht.readTemperature());    //Save temp into SD card
      myFile.println(" C");
      Serial.print("Humidity = ");
      Serial.print(dht.readHumidity());
      Serial.println("%");
//      myFile.print("Humidity = ");
//      myFile.print(dht.readHumidity());     //Save humidity into SD card
//      myFile.println("%");
      Serial.print("Wind Speed =");
      Serial.print(wind_speed);
      Serial.print("m/s");
      Serial.print(" or ");
      Serial.print(speed_mph);
      Serial.println("mph");
//      myFile.print("Wind Speed =");
//      myFile.print(wind_speed);             //Save wind m/s to SD card
//      myFile.print("m/s");
//      myFile.print(" or ");
//      myFile.print(speed_mph);              //Save wind mph to SD card
//      myFile.println("mph");
     if(digitalRead(capteur_D) == LOW)
        {
          Serial.println("Digital value : wet");
          myFile.println("Digital value : wet");
          delay(10);
        }
      else
        {
          Serial.println("Digital value : dry");
          myFile.println("Digital value : dry");
          delay(10);
        }
        val_analogique=analogRead(capteur_A);
         Serial.print("Analog value : ");
         Serial.println(val_analogique);
//        Serial.print("Writing to test.txt...");
        Serial.println(myFile);
        Serial.println("first if");
      //  myFile.println("first if");
          if (radio.available()) {
            Serial.println("second if");
            myFile.println("second if");
//            char text[32] = "";
//            Serial.print(text);
            radio.read(&data, sizeof(MyData));
            Serial.print("Soil Moisture : ");
            Serial.print(data.averageFloat);
            Serial.println("%");
            myFile.print("Soil Moisture : ");
            myFile.print(data.averageFloat);
            myFile.print("%");
      }
      // close the file:
        myFile.close();
        Serial.println("************************************");
      } else {
        // if the file didn't open, print an error:
        Serial.println(myFile);
        Serial.println("error opening test.txt");
      }
     delay(1000);
}
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
