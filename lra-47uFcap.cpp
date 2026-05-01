#include <Adafruit_TinyUSB.h>
#include <Wire.h>
#define DRV_ADDR 0x5A

void w(uint8_t r, uint8_t v){
  Wire.beginTransmission(DRV_ADDR);
  Wire.write(r); Wire.write(v);
  Wire.endTransmission();
}
uint8_t rd(uint8_t reg){
  Wire.beginTransmission(DRV_ADDR);
  Wire.write(reg); Wire.endTransmission(false);
  Wire.requestFrom(DRV_ADDR, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0xFF;
}

void setup(){
  Serial.begin(115200);
  while(!Serial && millis()<3000){}
  delay(500);
  Wire.begin();
  Wire.setClock(100000);
  delay(50);

  Wire.beginTransmission(0x5A);
  if (Wire.endTransmission() != 0) {
    Serial.println("0x5A gone, power-cycle");
    return;
  }

  w(0x01, 0x80); delay(50);
  w(0x01, 0x00); delay(10);
  w(0x16, 0x4F);
  w(0x17, 0x80);
  w(0x1A, 0xB6);
  w(0x1B, 0x9A);
  w(0x1C, 0xF5);
  w(0x1D, 0xA9);
  w(0x20, 0x3F);
  w(0x02, 0xA0);

  Serial.println("BRRRT 60/40 — testing 47uF cap headroom.");
}

void loop(){
  w(0x01, 0x05);
  delay(20);
  uint8_t vbat = rd(0x21);
  uint8_t status = rd(0x00);
  w(0x01, 0x40);
  delay(10);

  Serial.print("VBAT=0x"); Serial.print(vbat, HEX);
  Serial.print(" STAT=0x"); Serial.println(status, HEX);

  if (vbat == 0xFF || status == 0xFF) {
    Serial.println("BROWNOUT — halting");
    w(0x01, 0x40);
    while(1) delay(1000);
  }
}
