//AS5601と、motorを組み合わせ、
//モーターを動作させながら角度を認識できるかのテスト

#include "AS5600.h"
#include "Wire.h"
#include <MsTimer2.h>

// Setting for MsTimer2
unsigned long prevMilli = 0;
unsigned long count1 = 0;
unsigned long count2 = 0;
unsigned long count3 = 0;
volatile int period;

// Setting for Motor Control
const int E1 = 5;
const int M1 = 4;
const int E2 = 6;
const int M2 = 7;
const float ana2rad = 2*3.141592/360.0 * 330.0 / 1023.0;
volatile float curAngle;
volatile float Kp=1000, Kd=0.3;
volatile float tgtAngle[400];
volatile int tgtLength;  // length of tgtAngle array
volatile int tgtCycle;  // cycle number for target position modification
volatile int repeatTime;  // repeattime for tgtCycle


AS5600 as5600;   //  use default Wire


void periodicFunction() {
  unsigned long curMilli;
  float motorAngle, targetAngle;
  float angleError, angleVelo, oldAngle;
  float torque;
  int outMotor;

  // for checking of cycle period
  curMilli = micros();
  period = curMilli - prevMilli;
  prevMilli = curMilli;

  // Set target angle and read motor angle and velocity
  targetAngle = tgtAngle[count2];
  //motorAngle = ana2rad * analogRead(1);  
  motorAngle = as5600.readAngle()*AS5600_RAW_TO_DADIANS;
  curAngle = motorAngle;
  angleError = targetAngle - motorAngle;
  angleVelo = (motorAngle - oldAngle) * 1000.0;
  oldAngle = motorAngle;

  // Controller calculation
  torque = Kp * angleError - Kd * angleVelo;

  // Output to motor driver
  outMotor = (int)torque;
  if(outMotor > 255) {
    digitalWrite(M1, LOW);
    analogWrite(E1, 255);
  } else if(outMotor > 0) {
    digitalWrite(M1, LOW);
    analogWrite(E1, outMotor);
  } else if(outMotor > -255) {
    digitalWrite(M1, HIGH);
    analogWrite(E1, -outMotor);
  } else {
    digitalWrite(M1, HIGH);
    analogWrite(E1, 255);
  }

  // loop cycle 
  count1++;
  if(count1 == tgtCycle) {
    count2++;
    count1 = 0;
  }
  if(count2 == tgtLength) {
    count3++;
    count2 = 0;
  }
  if(count3 == repeatTime) {
    count2 = 0;
    count3 = 0;
    digitalWrite(M1, HIGH);
    analogWrite(E1, 0);
    MsTimer2::stop();
  }
}

void setup()
{
  Serial.begin(115200);
  Serial.println(__FILE__);
  Serial.print("AS5600_LIB_VERSION: ");
  Serial.println(AS5600_LIB_VERSION);

  //  ESP32
  //  as5600.begin(14,15);
  //  AVR
  as5600.begin(4);  //  set direction pin.
  as5600.setDirection(AS5600_CLOCK_WISE);  // default, just be explicit.
  int b = as5600.isConnected();
  Serial.print("Connect: ");
  Serial.println(b);
  delay(1000);
  
  for(int i=0; i<100; i++) {
    tgtAngle[i] = 3.141592/2 + 3.141592 * (1-cos(3.141592*i/100))/2;
    tgtAngle[199-i] = tgtAngle[i];
  }
  tgtCycle = 5;
  tgtLength = 200;
  repeatTime = 10;
  
  // Motor Controller
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);

  // Set up MsTimer2
  MsTimer2::set(0.5, periodicFunction); //periodicFunctionが呼び出される周期指定?
  MsTimer2::start();
}


void loop()
{
  //  Serial.print(millis());
  //  Serial.print("\t");
  // Serial.print(as5600.readAngle());
  // Serial.print("\t");
  // Serial.println(as5600.rawAngle());
  //  Serial.println(as5600.rawAngle() * AS5600_RAW_TO_DEGREES);

  delay(100);

    // Serial Com.
//  Serial.print(float2String(2,(curAngle*180/3.141592)));
  Serial.print(curAngle*180/3.141592); //
  Serial.print("     ");
  Serial.println(period);
}


// -- END OF FILE --
