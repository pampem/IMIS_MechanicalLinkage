//
//    FILE: AS5600_demo.ino
//  AUTHOR: Rob Tillaart
// PURPOSE: demo
//    DATE: 2022-05-28


#include "AS5600.h"
#include "Wire.h"

AS5600 as5600;   //  use default Wire

//Arduino PWM Speed Control：
int E1 = 5;
int M1 = 4;
int E2 = 6;
int M2 = 7;

  double err = 0;
  double pasedErr = 0;
  double integralErr = 0;
  double derivarateErr = 0;
  double tgtAngle = 0;
  double sensorAngle = 0;
  double value = 0;
  unsigned long period = 0; 
  unsigned long t_b = 0;
void setup()
{
  Serial.begin(115200);
  Serial.println(__FILE__);
  Serial.print("AS5600_LIB_VERSION: ");
  Serial.println(AS5600_LIB_VERSION);

  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);

  //  ESP32
  //  as5600.begin(14,15);
  //  AVR
  as5600.begin(4);  //  set direction pin.
  as5600.setDirection(AS5600_CLOCK_WISE);  // default, just be explicit.
  int b = as5600.isConnected();
  Serial.print("Connect: ");
  Serial.println(b);
  sensorAngle = (as5600.rawAngle() * AS5600_RAW_TO_DEGREES);
  if(sensorAngle <= 180){
    err = (tgtAngle - sensorAngle)/360;
  }
  else{
    err = (tgtAngle - (sensorAngle - 360))/360;
  }
  pasedErr = err;
  delay(1000);
}


void loop()
{
  
  if(millis() - period > 5000){
    period = millis();
    tgtAngle = -1*tgtAngle;
  }
  
  Serial.print(millis());
  Serial.print("\t");
  Serial.print(as5600.readAngle());
  Serial.print("\t");
  Serial.println(as5600.rawAngle() * AS5600_RAW_TO_DEGREES);
  sensorAngle = (as5600.rawAngle() * AS5600_RAW_TO_DEGREES);
  
  if(sensorAngle <= 180){
    err = (tgtAngle - sensorAngle);
  }
  else{
    err = (tgtAngle - (sensorAngle - 360));
  }
  Serial.println(err);
  
  
  integralErr += err;
  derivarateErr = (err - pasedErr)/(millis() - t_b);
  value = 255*(0.3*err + 0*integralErr + 0.1*derivarateErr);

  if(value < 0){
    digitalWrite(M1,LOW);
    value = -value;
  }
  else{
    digitalWrite(M1,HIGH);
  }
  t_b = millis();
  analogWrite(E1, value);

}


// -- END OF FILE --