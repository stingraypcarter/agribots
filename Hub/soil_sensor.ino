#include <SoftwareSerial.h>

const int RO = 2; //RX
const int RE = 8; //Receiver
const int DE = 7; //Driver
const int DI = 3; //TX

const byte ph_inquiry[]= {0x01,0x03,0x00, 0x06, 0x00, 0x01, 0x64, 0x0B};
const byte moisture_inquiry[]= {0x01,0x03,0x00, 0x12, 0x00, 0x01, 0x24, 0x0f};
const byte temp_inquiry[]= {0x01,0x03,0x00, 0x13, 0x00, 0x01, 0x75, 0xcf};
const byte conduct_inquiry[]= {0x01,0x03,0x00, 0x15, 0x00, 0x01, 0x95, 0xce};
const byte npk_inquiry[]= {0x01,0x03,0x00, 0x1e, 0x00, 0x03, 0x65, 0xcd};

const float max_ph = 14.0;
const float min_ph = 0.0;
const float max_temp = 100.0;
const float min_temp = -20.0;
const float max_moisture = 100.0;
const float min_moisture = 0.0;
const float max_conduct = 8000.0;
const float min_conduct = 0.0;
const float max_n = 1000.0;
const float min_n = 0.0;
const float max_p = 1000.0;
const float min_p = 0.0;
const float max_k = 1000.0;
const float min_k = 0.0;

const int error_delay = 50;
const int max_errored_attempts = 5;

float a = -1.0; //pH
float t = -1.0; //˚C
float m = -1.0; //%
float c = -1.0; //US/cm
float n = -1.0; //mg/Kg
float p = -1.0; //mg/Kg
float k = -1.0; //mg/Kg

SoftwareSerial soil_sensor(RO,DI);

void setup() {
  Serial.begin(9600);
  soil_sensor.begin(9600);
  //  soil_sensor.setTimeout(100);
  pinMode(RE, OUTPUT);
  pinMode(DE, OUTPUT); 

}

void Inquiry_Request()
{
  digitalWrite(DE, HIGH);
  digitalWrite(RE, HIGH);
  delay(30);
}

void Read_Request()
{
  digitalWrite(DE,LOW);
  digitalWrite(RE,LOW);
}

bool PH() // pH
{
  Inquiry_Request();
  byte response[] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00};
  if(soil_sensor.write(ph_inquiry,sizeof(ph_inquiry))==8){
    Read_Request();
    for(int i=0;i<7;i++){
      response[i] = soil_sensor.read();
    }
  }

  float return_ph = (((float) response[3])*256 + (float) response[4])/100.0;
  if(return_ph > max_ph || return_ph < min_ph)
  {
    delay(error_delay);
    Serial.print("Incorrect pH Measurement: ");
    Serial.print(return_ph);
    Serial.println("pH");
    return false;
  }
  else
  {
    a = return_ph;
    return true;
  }
}

bool Temp() // ˚C
{
  Inquiry_Request();
  byte response[] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00};
  if(soil_sensor.write(temp_inquiry,sizeof(temp_inquiry))==8){
    Read_Request();
    for(int i=0;i<7;i++){
      response[i] = soil_sensor.read();
    }
  }

  float return_temp = (((float) response[3])*256 + (float) response[4])/10.0;
  if(return_temp > max_temp || return_temp < min_temp)
  {
    delay(error_delay);
    Serial.print("Incorrect Temp Measurement: ");
    Serial.print(return_temp);
    Serial.println("˚C");
    return false;
  }
  else
  {
    t = return_temp;
    return true;
  }
}

bool Moisture()  //% moisture
{
  Inquiry_Request();
  byte response[] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00};
  if(soil_sensor.write(moisture_inquiry,sizeof(moisture_inquiry))==8){
    Read_Request();
    for(int i=0;i<7;i++){
      response[i] = soil_sensor.read();
    }
  }

  float return_moisture = (((float) response[3])*256 + (float) response[4])/10.0;
  if(return_moisture > max_moisture || return_moisture < min_moisture)
  {
    delay(error_delay);
    Serial.print("Incorrect Moisture Measurement: ");
    Serial.print(return_moisture);
    Serial.println("%");
    return false;
  }
  else
  {
    m = return_moisture;
    return true;
  }

}

bool Conduct()  //uS/cm
{
  Inquiry_Request();
  byte response[] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00};
  if(soil_sensor.write(conduct_inquiry,sizeof(conduct_inquiry))==8){
    Read_Request();
    for(int i=0;i<7;i++){
      response[i] = soil_sensor.read();
    }
  }

  float return_conduct = (((float) response[3])*256 + (float) response[4]);
  if(return_conduct > max_conduct || return_conduct < min_conduct)
  {
    delay(error_delay);
    Serial.print("Incorrect Conductivity Measurement: ");
    Serial.print(return_conduct);
    Serial.println("uS/cm");
    return false;
  }
  else
  {
    c = return_conduct;
    return true;
  }

}

bool NPK()   //mg/Kg
{
  Inquiry_Request();
  byte response[] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
  if(soil_sensor.write(npk_inquiry,sizeof(npk_inquiry))==8){
    Read_Request();
    for(int i=0;i<11;i++){
      response[i] = soil_sensor.read();
    }
  }
  float return_n = (((float) response[3])*256 + (float) response[4]);
  float return_p = (((float) response[5])*256 + (float) response[6]);
  float return_k = (((float) response[7])*256 + (float) response[8]);

  if(return_n > max_n || return_p > max_p || return_k > max_k || return_n < min_n || return_p < min_p || return_k > min_k)
  {
    delay(error_delay);
    Serial.println("Incorrect Measurement in one of the three following: ");
    Serial.print("N: ");
    Serial.print(return_n);
    Serial.println("mg/Kg");
    Serial.print("P: ");
    Serial.print(return_p);
    Serial.println("mg/Kg");
    Serial.print("K: ");
    Serial.print(return_k);
    Serial.println("mg/Kg");
    return false;
  }
  else
  {
    n = return_n;
    p = return_p;
    k = return_k;
    return true;
  }
}

float Get_Data(char data_code, int attempt)
{
  if(attempt > max_errored_attempts)
  {
    Serial.println("Max attempts failed, check sensor");
    return -1.0;
  }
  else
  {
    
  }
  switch(data_code)
  {
    case 'A':
      if(PH())
      {
        Serial.print("Acidity: ");
        Serial.print(a);
        Serial.println(" pH");
        return a;
      }
      else
      {
        Get_Data(data_code,attempt+1);
      }
      break;
    case 'T':
      if(PH())
      {
        Serial.print("Temperature: ");
        Serial.print(t);
        Serial.println(" ˚C");
        return t;
      }
      else
      {
        Get_Data(data_code,attempt+1);
      }
      break;
    case 'M':
      if(PH())
      {
        Serial.print("Moisture: ");
        Serial.print(m);
        Serial.println(" %");
        return m;
      }
      else
      {
        Get_Data(data_code,attempt+1);
      }
      break;
    case 'C':
      if(PH())
      {
        Serial.print("Conductivity: ");
        Serial.print(c);
        Serial.println(" uS/cm");
        return c;
      }
      else
      {
        Get_Data(data_code,attempt+1);
      }
      break;
    case 'N':
      if(PH())
      {
        Serial.print("Nitrogen: ");
        Serial.print(n);
        Serial.println(" mg/Kg");
        return n;
      }
      else
      {
        Get_Data(data_code,attempt+1);
      }
      break;
    case 'P':
      if(PH())
      {
        Serial.print("Phosphorus: ");
        Serial.print(p);
        Serial.println(" mg/Kg");
        return p;
      }
      else
      {
        Get_Data(data_code,attempt+1);
      }
      break;
    case 'K':
      if(PH())
      {
        Serial.print("Potassium: ");
        Serial.print(k);
        Serial.println(" mg/Kg");
        return k;
      }
      else
      {
        Get_Data(data_code,attempt+1);
      }
      break;
    default:
      Serial.print("Incorrect input");
      return -1.0;
  }
}

float Sample_Data(char data_code, int times, float interval)
{
  float sum = 0.0;
  for(int i = 0; i < times; i++)
  {
    float data_point = Get_Data(data_code,0);
    if(data_point = -1.0)
    {
      Serial.println("Failed Data Reading");
      return -1.0;
    }
    sum += data_point;
    delay(interval);
  }
  return sum/(float) times;
}


void loop() {
  delay(1000);
  
}
