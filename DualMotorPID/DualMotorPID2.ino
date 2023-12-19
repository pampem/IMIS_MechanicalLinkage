//20231220 ++ Masashi Izumita
//AS5601、マルチプレクサPCA9547Dと、motor２つを組み合わせ、
//MechanicalLinkageをPID制御する。

//PCA9547のChannnel 0,1にAS5600を接続する。
//Channel 0, M1; リンク側モータ
//Channel 1, M2; ベルト側モータ

#include "AS5600.h"
#include "Wire.h"
#include <MsTimer2.h>
#include "PCA9547.h"
sei(); //Set interruput enable

// Setting for MsTimer2
unsigned long prevMilli = 0;
unsigned long pidCount = 0;
unsigned long trjCount = 0;
unsigned long repeatCount = 0;

// Setting for Motor Control
const int E1 = 5;
const int M1 = 4;
const int E2 = 6;
const int M2 = 7;

// // After Modification Fast Circle
// #include "DualMotorPID_Trajectory.h"
// volatile float KpL=400, KdL=0.36, KiL=2;
// volatile float KpB=300, KdB=0.6, KiB=0.1;
// volatile int tgtCycle = 7;
// unsigned long periodicfunctionHz = 3;

// After Modification Fast Line
#include "DualMotorPID_Trajectory_line.h"
volatile float KpL=300, KdL=0.36, KiL=2;
volatile float KpB=200, KdB=0.000, KiB=0.00;
volatile int tgtCycle = 2;
unsigned long periodicfunctionHz = 3;


volatile int tgtLength = sizeof(LinktgtAngle) / sizeof(LinktgtAngle[0]);//LinktgtAngleの配列サイズをリファレンスして設定。

volatile int repeatTime = 5; //何回動作を繰り返すか

// volatile float LinkMotorAngle;//[rad]
volatile float LinkMotorFirstAngle;//[ASreadAngle(0~4096)]

// volatile float BeltMotorAngle;//[rad]
volatile float BeltMotorFirstAngle;//[ASreadAngle(0~4096)]
volatile float BeltAngleRotation = 0;
volatile float BeltSensorAngleNow = 0;
volatile float BeltSensorAnglePrev = 0;
volatile float LinkintegralError, BeltintegralError;
volatile float LinkOldAngle=0, BeltOldAngle=0;

float pi = 3.14159265;

AS5600 as5600;
PCA9547 i2cSelect;
// #define addr 0x70//PCA9547Dのアドレス

// リミットスイッチ用のピン設定
const int limitSwitchPin = 8;


void periodicFunction() {

  unsigned long curMilli;
  float LinktargetAngle;
  float LinkangleError, LinkangleVelo;
  float Linktorque;
  float BelttargetAngle;
  float BeltangleError, BeltangleVelo;
  float Belttorque;

  float LinkMotorAngle, BeltMotorAngle;

  curMilli = micros();
  float step;
  step = (float)(curMilli - prevMilli);
  prevMilli = curMilli;


  i2cSelect.enable(0);
  float offsetAngle = ((float)as5600.readAngle() - LinkMotorFirstAngle + 4096) % 4096;
  if(offsetAngle <= 2048){
    LinkMotorAngle = offsetAngle * AS5600_RAW_TO_RADIANS;
  }else{
    LinkMotorAngle = (offsetAngle - 4096) * AS5600_RAW_TO_RADIANS;
  }  

  //Set target angle and read motor angle and velocity
  LinktargetAngle = LinktgtAngle[trjCount];
  LinkangleError = LinktargetAngle - LinkMotorAngle;//+-とりうる
  LinkangleVelo = (LinkMotorAngle - LinkOldAngle) / step*1000;
  LinkOldAngle = LinkMotorAngle;
  LinkintegralError += LinkangleError * step / 1000;
  //PID
  Linktorque = (KpL * LinkangleError - KdL * LinkangleVelo + KiL * LinkintegralError);
  Serial.print("LinkTor "+String(Linktorque));

  if(Linktorque>0){
    Linktorque = normalizeTorque(Linktorque);
    digitalWrite(M1, LOW);
    analogWrite(E1, Linktorque);
  }else{
    Linktorque = normalizeTorque(-Linktorque);
    digitalWrite(M1, HIGH);
    analogWrite(E1, Linktorque);
  }
  

  //ベルト側の角度値を読み取る
  i2cSelect.enable(1);
  BeltSensorAngleNow = ((float)as5600.readAngle() - BeltMotorFirstAngle + 4096) % 4096; //0~4096, not rad
  if(BeltSensorAngleNow >= 0 && BeltSensorAngleNow < 200 && BeltSensorAnglePrev > 3800){ //4096から0になったとき
    BeltAngleRotation++;
  }else if(BeltSensorAngleNow > 3800 && BeltSensorAnglePrev < 200 && BeltSensorAnglePrev >= 0){ //0から4096になったとき
    BeltAngleRotation--;
  }
  BeltSensorAnglePrev = BeltSensorAngleNow;
  BeltMotorAngle = ((as5600.readAngle() - BeltMotorFirstAngle + 4096) % 4096 + BeltAngleRotation*4096) * AS5600_RAW_TO_RADIANS;

    
  BelttargetAngle = BelttgtAngle[trjCount];
  BeltangleError = (BelttargetAngle - BeltMotorAngle);//+-とりうる
  BeltangleVelo = (BeltMotorAngle - BeltOldAngle) /step*1000;
  BeltOldAngle = BeltMotorAngle;
  BeltintegralError += BeltangleError * step / 1000;
  //PID
  Belttorque = (KpB * BeltangleError - KdB * BeltangleVelo + KiB * BeltintegralError);
  Serial.print(" BeltTor "+String(Belttorque));

  if(Belttorque>0){
    Belttorque = normalizeTorque(Belttorque);
    digitalWrite(M2, LOW);
    analogWrite(E2, Belttorque);
  }else{
    Belttorque = normalizeTorque(-Belttorque);
    digitalWrite(M2, HIGH);
    analogWrite(E2, Belttorque);
  }

  pidCount++;
  if(pidCount == tgtCycle) { // Update the target trajectory
    trjCount++;
    pidCount = 0;
    LinkintegralError = 0;
    BeltintegralError = 0;
  }
  if(trjCount == tgtLength) { // Repeat Trajectory
    repeatCount++;
    trjCount = 0;
  }
  if(repeatCount == repeatTime) { // Stop motor
    digitalWrite(M1, HIGH);
    analogWrite(E1, 0);
    digitalWrite(M2, HIGH);
    analogWrite(E2, 0);
    MsTimer2::stop();
  }
  Serial.print(" "+string(LinkMotorAngle) + " " + String(BeltMotorAngle) + " " + String(CurMilli)+"\n");
}

void setup()
{
  Serial.begin(115200);

  // Motor Controller
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
  delay(100);

  //PCA9547 related
  Wire.begin();
  i2cSelect.attach(Wire); // default addr : 0x70

  //Link
  i2cSelect.enable(0);
  Serial.println("Ch 1 initializing");
  as5600.begin();  //  set direction pin.
  as5600.setDirection(AS5600_COUNTERCLOCK_WISE);  // default, just be explicit.
  LinkMotorFirstAngle = (float)as5600.readAngle();
  Serial.println(b);
  delay(10);

  //Belt
  i2cSelect.enable(1);
  Serial.println("Ch 2 initializing");
  as5600.begin();  //  set direction pin.
  as5600.setDirection(AS5600_COUNTERCLOCK_WISE);  // default, just be explicit.
  delay(1000);

  // Calibration by Limit Switch
  pinMode(limitSwitchPin, INPUT_PULLUP);
  while(digitalRead(limitSwitchPin) == HIGH) {
    digitalWrite(M2, HIGH);
    analogWrite(E2, 100);
  }
  BeltMotorFirstAngle = (float)as5600.readAngle();

  digitalWrite(M2,LOW);
  analogWrite(E2,70);
  delay(1000);

  // Set up MsTimer2
  MsTimer2::set(periodicfunctionHz, periodicFunction);
  MsTimer2::start();
}

void loop()
{
 
}

int normalizeTorque(float torque){
    float clippedTorque = torque;
    clippedTorque = max(clippedTorque, 0);
    clippedTorque = min(clippedTorque, 255);
 
    return ClippedTorque;
}
// -- END OF FILE --