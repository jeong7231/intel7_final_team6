/*
SDL : A4
SCL : A5

*/

#include <Wire.h>

// 대부분의 TOF10120 계열 I2C 7-bit 주소
const uint8_t TOF_ADDR = 0x52;   // 필요 시 0x57 등으로 바꿔 테스트

// 거리 읽기(mm)
int readDistanceMM() {
  // 데이터 레지스터 포인터(0x00) 지정
  Wire.beginTransmission(TOF_ADDR);
  Wire.write(0x00);
  if (Wire.endTransmission() != 0) return -1;

  delay(1); // 짧은 여유

  // 2바이트 읽기 (MSB, LSB)
  Wire.requestFrom(TOF_ADDR, (uint8_t)2);
  if (Wire.available() < 2) return -1;

  uint16_t msb = Wire.read();
  uint16_t lsb = Wire.read();
  return (int)((msb << 8) | lsb); // mm
}

void setup() {
  Serial.begin(115200);
  Wire.begin(); // UNO: SDA=A4, SCL=A5
  Serial.println("TOF I2C test start");
}

void loop() {
  int d = readDistanceMM();
  if (d >= 0) {
    Serial.print("Distance: ");
    Serial.print(d);
    Serial.println(" mm");
  } else {
    Serial.println("Read fail (check wiring/address)");
  }
  delay(100); // 30ms 이상 권장
}
