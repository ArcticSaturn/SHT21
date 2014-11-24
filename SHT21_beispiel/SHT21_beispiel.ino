#include <SoftwareI2C.h>
#include <sht21.h>

SHT21 mySHT21(8,9);

void setup()
{
  Serial.begin(9600);
  pinMode(RED_LED, OUTPUT);
  digitalWrite(RED_LED,HIGH);
  delay(50);  
  digitalWrite(RED_LED,LOW);
  mySHT21.SoftReset();
  delay(50); 
}

void loop()
{
  uint16_t MeasValue;
  float Result;
  uint8_t RegValue;
  
  mySHT21.Measure(TEMPERATURE,T_12BIT, &MeasValue, &Result);
  //Serial.println(MeasValue, HEX);
  Serial.print("Temperature: ");
  Serial.print(Result,2);
  Serial.println("C");
  mySHT21.ReadUserRegister(&RegValue);
  Serial.println(RegValue, HEX);
 
  delay(2000);
  
  mySHT21.Measure(HUMIDITY,RH_12BIT, &MeasValue, &Result);
  Serial.println(MeasValue, HEX);
  Serial.print("Humidity: ");
  Serial.print(Result,1);
  Serial.println("%");
  
  mySHT21.ReadUserRegister(&RegValue);
  Serial.println(RegValue, HEX);
  
  digitalWrite(RED_LED,HIGH);
  delay(50);  
  digitalWrite(RED_LED,LOW);
  delay(2000);  
}


