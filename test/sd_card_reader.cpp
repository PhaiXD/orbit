#include <Arduino.h>
#include "LIS3DHTR.h"
#include <Wire.h>
#include "orbit_pin_def.h"

TwoWire i2c1(PIN_SDA, PIN_SCL);
LIS3DHTR<TwoWire> LIS; //IIC
#define WIRE Wire

void setup()
{
  Serial.begin(115200);
  while (!Serial)
  {
  };
  LIS.begin(i2c1, 0x18); //IIC init 0x18
  delay(100);
  LIS.setOutputDataRate(LIS3DHTR_DATARATE_50HZ);

}
void loop()
{
  if (!LIS)
  {
    Serial.println("LIS3DHTR didn't connect.");
    while (1)
      ;
    return;
  }
  //3 axis
   Serial.print("x:"); Serial.print(LIS.getAccelerationX()); Serial.print("  ");
   Serial.print("y:"); Serial.print(LIS.getAccelerationY()); Serial.print("  ");
   Serial.print("z:"); Serial.println(LIS.getAccelerationZ());

}
