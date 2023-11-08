//20231109 ++ Masashi Izumita
//AS5601、マルチプレクサPCA9547Dと、motor２つを組み合わせ、
//モーターの角度値をPID制御するテスト。
//Features: タイマ割り込み使用、目標角度は軌道として与える、PCA9547使用

//PCA9547のChannnel 0,1にAS5600を接続する。
//Channel 0, M1; リンク側モータ
//Channel 1, M2; ベルト側モータ

#include "AS5600.h"
#include "Wire.h"
#include <MsTimer2.h>
#include "PCA9547.h"

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

volatile float Kp=100, Kd=0.03;
volatile float LinktgtAngle[400];
volatile float BelttgtAngle[400];
volatile int tgtLength = 200; //length of tgtAngle array
volatile int tgtCycle = 10; //tgtAngleの更新速度に関わる cycle number for target position modification
volatile int repeatTime = 10; //repeattime for tgtCycle

volatile float LinkMotorAngle;//[rad]
volatile int LinkMotorFirstAngle;//[ASreadAngle(0~4096)]
volatile float BeltMotorAngle;//[rad]
volatile int BeltMotorFirstAngle;//[ASreadAngle(0~4096)]

AS5600 as5600;
PCA9547 i2cSelect;
unsigned int ch = 0;

void setTrajectory(volatile float *tgtAngle){
  //注意点: 範囲を0~とかにすると動作が不安定になる。初期Angleはpi/2にオフセットしてあるので、pi/2~の指定が現状ベスト。
  float pi = 3.14159265;

  // for(int i=0; i<100; i++) { //pi/2 ~ 3/2*pi
  //   tgtAngle[i] = pi/2 + pi * (1-cos(pi*i/100))/2;
  //   tgtAngle[199-i] = tgtAngle[i];
  // }

  for(int i=0; i<100; i++) { //pi/2 ~ 3/4*pi
    tgtAngle[i] = pi/2 + 0.5*pi * (1-cos(pi*i/100))/2;
    tgtAngle[199-i] = tgtAngle[i];
  }
}

void periodicFunction() {
  unsigned long curMilli;
  float LinktargetAngle;
  float LinkangleError, LinkangleVelo, LinkoldAngle;
  float Linktorque;
  float BelttargetAngle;
  float BeltangleError, BeltangleVelo, BeltoldAngle;
  float Belttorque;

  //cycle period
  curMilli = micros();
  period = curMilli - prevMilli;
  prevMilli = curMilli;

  //Set target angle and read motor angle and velocity
  LinktargetAngle = LinktgtAngle[count2];
  LinkangleError = LinktargetAngle - LinkMotorAngle;//+-とりうる
  LinkangleVelo = (LinkMotorAngle - LinkoldAngle) /period*1000;
  LinkoldAngle = LinkMotorAngle;
  BelttargetAngle = BelttgtAngle[count2];
  BeltangleError = BelttargetAngle - BeltMotorAngle;//+-とりうる
  BeltangleVelo = (BeltMotorAngle - BeltoldAngle) /period*1000;
  BeltoldAngle = BeltMotorAngle;

  //Controller calculation　PD
  Linktorque = Kp * LinkangleError - Kd * LinkangleVelo;
  Belttorque = Kp * BeltangleError - Kd * BeltangleVelo;

  if(Linktorque>0){
    Linktorque = normalizeTorque(Linktorque);
    digitalWrite(M1, HIGH);
    analogWrite(E1, Linktorque);
  }else{
    Linktorque = normalizeTorque(-Linktorque);
    digitalWrite(M1, LOW);
    analogWrite(E1, Linktorque);
  }

  if(Belttorque>0){
    Belttorque = normalizeTorque(Belttorque);
    digitalWrite(M2, HIGH);
    analogWrite(E2, Belttorque);
  }else{
    Belttorque = normalizeTorque(-Belttorque);
    digitalWrite(M2, LOW);
    analogWrite(E2, Belttorque);
  }

  // loop cycle軌道を更新する目的と、停止させる目的。
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
    digitalWrite(M2, HIGH);
    analogWrite(E2, 0);
    MsTimer2::stop();
  }
}

void setup()
{
  Serial.begin(115200);

  //PCA9547 related
  Wire.begin();
  i2cSelect.attach(Wire); // default addr : 0x70
  ch = 0;
  i2cSelect.enable(ch);
  Serial.print("AS5600Initialized... AS5600_LIB_VERSION: ");
  Serial.println(AS5600_LIB_VERSION);
  as5600.begin(1);  //  set direction pin.
  as5600.setDirection(AS5600_COUNTERCLOCK_WISE);  // default, just be explicit.
  int b = as5600.isConnected();
  Serial.print("Connect: ");
  Serial.println(b);
  LinkMotorFirstAngle = as5600.readAngle();

  ch = 1;
  i2cSelect.enable(ch);
  Serial.print("AS5600Initialized... AS5600_LIB_VERSION: ");
  Serial.println(AS5600_LIB_VERSION);
  as5600.begin(2);  //  set direction pin.
  as5600.setDirection(AS5600_COUNTERCLOCK_WISE);  // default, just be explicit.
  int c = as5600.isConnected();
  Serial.print("Connect: ");
  Serial.println(b);
  BeltMotorFirstAngle = as5600.readAngle();
    
  delay(2000);

  setTrajectory(LinktgtAngle);
  setTrajectory(BelttgtAngle);//LinkとBeltのtgtAngleにはとりあえず同じTrajectoryを設定。

  // Motor Controller
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);

  // Set up MsTimer2
  MsTimer2::set(1, periodicFunction);
  MsTimer2::start();
}

void loop()
{
  //リング側の角度値を読み取る
  ch = 0;
  i2cSelect.enable(ch);
  Serial.print("LinkMotor's read angle = " + String(as5600.readAngle()));
  //LinkMotorAngle = ((as5600.readAngle() - LinkMotorFirstAngle + 4096) % 4096 )* AS5600_RAW_TO_RADIANS; //初期位置0[rad]version
  LinkMotorAngle = ((as5600.readAngle() - LinkMotorFirstAngle + 4096 + (4096/4)) % 4096) * AS5600_RAW_TO_RADIANS; //初期位置がpi/2[rad]にオフセットされたもの
  Serial.print(", " + String(LinkMotorAngle) + "[rad]");
  Serial.print("\t");

  //ベルト側の角度値を読み取る
  ch = 1;
  i2cSelect.enable(ch);
  Serial.print("BeltMotor's read angle = " + String(as5600.readAngle()));
  //LinkMotorAngle = ((as5600.readAngle() - LinkMotorFirstAngle + 4096) % 4096 )* AS5600_RAW_TO_RADIANS; //初期位置0[rad]version
  BeltMotorAngle = ((as5600.readAngle() - BeltMotorFirstAngle + 4096 + (4096/4)) % 4096) * AS5600_RAW_TO_RADIANS; //初期位置がpi/2[rad]にオフセットされたもの
  Serial.println(", " + String(BeltMotorAngle) + "[rad]");
  }

int normalizeTorque(float torque){
    //Difinition
    float minTorque = 0; // 取り得る最小のトルク
    float maxTorque = 255; // 取り得る最大のトルク
    float maxPower = 255;
    //Clipping
    float clippedTorque = torque;
    clippedTorque = max(clippedTorque, minTorque);
    clippedTorque = min(clippedTorque, maxTorque);
    //Normalize
    float normalizedTor = (clippedTorque - minTorque) / (maxTorque - minTorque) * maxPower;
    //結果のトルクを0から255の範囲に収めていることを保証
    normalizedTor = constrain(normalizedTor, 0, 255);
    return normalizedTor;
}
// -- END OF FILE --