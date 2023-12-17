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
#include "DualMotorPID_Trajectory.h"

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

volatile float KpL=500, KdL=0.06, KiL=3; //i 15
volatile float KpB=300, KdB=0.06, KiB=3;

volatile int tgtLength = sizeof(LinktgtAngle) / sizeof(LinktgtAngle[0]);//LinktgtAngleの配列サイズをリファレンスして設定。

volatile int tgtCycle = 60; //6 Cycle period for tgtAngle modification　[ms] cuz periodicfunctionが1msで読み込まれるので。

volatile int repeatTime = 10; //repeattime for tgtCycle

volatile float LinkMotorAngle;//[rad]
volatile int LinkMotorFirstAngle;//[ASreadAngle(0~4096)]
volatile float BeltMotorAngle;//[rad]
volatile int BeltMotorFirstAngle;//[ASreadAngle(0~4096)]

volatile float BeltAngleRotation = 0.00;
volatile float BeltSensorAngleNow = 0;
volatile float BeltSensorAnglePrev = 0;//At first, the angle is always 0, so.

// 231121 I control add
volatile float LinkintegralError, BeltintegralError;
volatile float LinkOldAngle=0, BeltOldAngle=0;

float pi = 3.14159265;

AS5600 as5600;
PCA9547 i2cSelect;
unsigned int ch = 0;
#define addr 0x70//PCA9547Dのアドレス

// リミットスイッチ用のピン設定
const int limitSwitchPin = 8;


void periodicFunction() {
  unsigned long curMilli;
  float LinktargetAngle;
  //float LinkangleError, LinkangleVelo, LinkoldAngle;
  float LinkangleError, LinkangleVelo;
  float Linktorque;
  float BelttargetAngle;
  //float BeltangleError, BeltangleVelo, BeltoldAngle;
  float BeltangleError, BeltangleVelo;
  float Belttorque;


  //cycle period
  curMilli = micros();
  period = curMilli - prevMilli;
  prevMilli = curMilli;

  if(ch==0){
    //Set target angle and read motor angle and velocity
    LinktargetAngle = LinktgtAngle[count2];
    LinkangleError = LinktargetAngle - LinkMotorAngle;//+-とりうる
    LinkangleVelo = (LinkMotorAngle - LinkOldAngle) /period*1000;
    LinkOldAngle = LinkMotorAngle;
    LinkintegralError += LinkangleError * period / 1000;

    //Controller calculation　PD
    //Linktorque = (KpL * LinkangleError - KdL * LinkangleVelo);

    //PID
    Linktorque = (KpL * LinkangleError - KdL * LinkangleVelo + KiL * LinkintegralError);

    if(Linktorque>0){
      Linktorque = normalizeTorque(Linktorque);
      digitalWrite(M1, LOW);
      analogWrite(E1, Linktorque);
    }else{
      Linktorque = normalizeTorque(-Linktorque);
      digitalWrite(M1, HIGH);
      analogWrite(E1, Linktorque);
    }
  }

  if(ch==1){
    BelttargetAngle = BelttgtAngle[count2];
    BeltangleError = (BelttargetAngle - BeltMotorAngle);//+-とりうる
    BeltangleVelo = (BeltMotorAngle - BeltOldAngle) /period*1000;
    BeltOldAngle = BeltMotorAngle;
    BeltintegralError += BeltangleError * period / 1000;

    //Controller calculation　PD
    //Belttorque = KpB * BeltangleError - KdB * BeltangleVelo;
    //PID
    Belttorque = (KpB * BeltangleError - KdB * BeltangleVelo + KiB * BeltintegralError);

    if(Belttorque>0){
      Belttorque = normalizeTorque(Belttorque);
      digitalWrite(M2, LOW);
      analogWrite(E2, Belttorque);
    }else{
      Belttorque = normalizeTorque(-Belttorque);
      digitalWrite(M2, HIGH);
      analogWrite(E2, Belttorque);
    }
  }

  // loop cycle軌道を更新する目的と、停止させる目的。
  count1++;
  if(count1 == tgtCycle) { // Update the target trajectory
    count2++;
    count1 = 0;
    //LinkintegralError = 0;
    //BeltintegralError = 0;
  }
  if(count2 == tgtLength) { // Repeat Trajectory
    count3++;
    count2 = 0;
  }
  if(count3 == repeatTime) { // Stop motor
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

  ch = 0; //Link
  i2cSelect.enable(ch);
  Serial.print("AS5600Initialized... AS5600_LIB_VERSION: ");
  Serial.println(AS5600_LIB_VERSION);
  //as5600.begin(1);  //  set direction pin.
  as5600.setDirection(AS5600_COUNTERCLOCK_WISE);  // default, just be explicit.
  int b = as5600.isConnected();
  Serial.print("Connect: ");
  Serial.println(b);
  LinkMotorFirstAngle = as5600.readAngle();

  ch = 1; //Belt
  i2cSelect.enable(ch);
  Serial.print("AS5600Initialized... AS5600_LIB_VERSION: ");
  Serial.println(AS5600_LIB_VERSION);
  //as5600.begin(1);  //  set direction pin.
  as5600.setDirection(AS5600_COUNTERCLOCK_WISE);  // default, just be explicit.
  int c = as5600.isConnected();
  Serial.print("Connect: ");
  Serial.println(c);
  delay(1000);
  // CXalibration by Limit Switch
  // リミットスイッチピンを入力として設定し、内部プルアップを有効にする
  pinMode(limitSwitchPin, INPUT_PULLUP);
  // BeltMotorを下げてリミットスイッチを探す
  while(digitalRead(limitSwitchPin) == HIGH) {
    digitalWrite(M2, HIGH);//High ? Low?
    analogWrite(E2, 50);//数値はTorque。現物見て調節して。
  }
  BeltMotorFirstAngle = as5600.readAngle();
  digitalWrite(M2,LOW);
  analogWrite(E2,30);
  delay(2000);
    
  // Motor Controller
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);

  // Set up MsTimer2
  MsTimer2::set(1, periodicFunction);
  MsTimer2::start();
}

void loop()
{
  //リンク側の角度値を読み取る
  ch = 0;
  i2cSelect.enable(ch);
  //Serial.print("LinkMotor's read angle = " + String(as5600.readAngle()));
  float offsetAngle = (as5600.readAngle() - LinkMotorFirstAngle + 4096) % 4096;
  if(offsetAngle <= 2048){
    LinkMotorAngle = offsetAngle * AS5600_RAW_TO_RADIANS;
  }else{
    LinkMotorAngle = (offsetAngle - 4096) * AS5600_RAW_TO_RADIANS;
  }
  //Serial.print(", " + String(LinkMotorAngle) + "[rad]");
  //Serial.print("\t");
  Serial.print(LinkMotorAngle);
  Serial.print(" ");
  delay(10);
 
  //ベルト側の角度値を読み取る
  ch = 1;
  i2cSelect.enable(ch);
  //Serial.print("BeltMotor's read angle = " + String(as5600.readAngle()));
  BeltSensorAngleNow = (as5600.readAngle() - BeltMotorFirstAngle + 4096) % 4096; //0~4096, not rad
  if(BeltSensorAngleNow >= 0 && BeltSensorAngleNow < 200 && BeltSensorAnglePrev > 3800){ //4096から0になったとき
    BeltAngleRotation++;
  }else if(BeltSensorAngleNow > 3800 && BeltSensorAnglePrev < 200 && BeltSensorAnglePrev >= 0){ //0から4096になったとき
    BeltAngleRotation--;
  }
  BeltSensorAnglePrev = BeltSensorAngleNow;
  float offsetBeltAngle = (as5600.readAngle() - BeltMotorFirstAngle + 4096) % 4096;
  float test = ((as5600.readAngle() - BeltMotorFirstAngle + 4096) % 4096 + BeltAngleRotation*4096);
  BeltMotorAngle = ((as5600.readAngle() - BeltMotorFirstAngle + 4096) % 4096 + BeltAngleRotation*4096) * AS5600_RAW_TO_RADIANS;

  //Debug message
  //Serial.println(", " + String(BeltMotorAngle) + "[rad]" + " Rotation "+ String(BeltAngleRotation) + "   " + String(offsetBeltAngle)+ "   " + String(test));
  Serial.print(BeltMotorAngle);
  Serial.print("\n");
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