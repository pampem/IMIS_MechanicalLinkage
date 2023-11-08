#include "PCA9547.h"
#include "AS5600.h"
#include "Wire.h"

PCA9547 i2cSelect;

#define addr 0x70//PCA9547Dのアドレス
#define AS0_Channel 0//AS0のチャンネル
#define AS1_Channel 1//AS1のチャンネル



AS5600 as5600;   //  use default Wire

double err = 0;
double tgtAngle = 0;
double sensorAngle = 0;

int result0;
int result1;

unsigned int ch = 0;

void setup()
{
  Serial.begin(115200);
  Serial.println(__FILE__);
  Serial.print("AS5600_LIB_VERSION: ");
  Serial.println(AS5600_LIB_VERSION);

  as5600.begin(4);  //  set direction pin.
  as5600.setDirection(AS5600_CLOCK_WISE);  // default, just be explicit.
  int b = as5600.isConnected();
  Serial.print("Connect: ");
  Serial.println(b);

  Wire.begin();
  i2cSelect.attach(Wire); // default addr : 0x70
  // i2cSelect.attach(Wire, 0x74); // or set your addr

  ChangeChannel(AS0_Channel);//PCA9547DのチャンネルをAS0_Channel(ここでは0)に切り替える


  delay(100);
}

void loop()
{
  i2cSelect.enable(ch % 8); // write to IC
  Serial.print("current ch : ");
  Serial.println(i2cSelect.channel(), HEX); // read from IC
  ++ch;
  
  Serial.print(millis());
  Serial.print("\t");

  ChangeChannel(AS0_Channel);
  result0 = as5600.readAngle();
  ChangeChannel(AS1_Channel);
  result1 = as5600.readAngle();

  Serial.print("\t");
  Serial.print(result0);Serial.print(",");Serial.print(result1);

  delay(1000);
}

void ChangeChannel(char channel) //バスマルチのチャンネル変更
{
  Wire.beginTransmission(addr);
  Wire.write(channel & 0x07 | 0x08);
  Wire.endTransmission();
}