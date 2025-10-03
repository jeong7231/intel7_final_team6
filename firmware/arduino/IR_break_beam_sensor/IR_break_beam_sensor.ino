#include <Wire.h>
#include <DFRobot_HX711_I2C.h>

DFRobot_HX711_I2C scale(&Wire, 0x64);

// 부호가 반대면 -1로, 정상이면 +1로
const int SIGN = -1;   // ← 지금은 음수로 증가한다고 했으니 -1

void setup() {
  Serial.begin(115200);
  Wire.begin();

  if (!scale.begin()) {
    Serial.println("Scale not found");
    while (1);
  }

  delay(3000);      // 전원 안정화 후
  scale.peel();     // 또는 scale.tare();
  Serial.println("Ready");
}

void loop() {
  float g = scale.readWeight(10);   // 또는 getWeight(10)
  g *= SIGN;                        // 부호 보정

  Serial.print("Weight: "); Serial.print(g, 1); Serial.println(" g");

  // 근처에서 자동 영점(옵션)
  const float AUTO_ZERO_THRESH = 5.0f;
  const unsigned long AUTO_ZERO_MS = 3000;
  static unsigned long nearZeroStart = 0;
  if (fabs(g) < AUTO_ZERO_THRESH) {
    if (nearZeroStart == 0) nearZeroStart = millis();
    if (millis() - nearZeroStart > AUTO_ZERO_MS) {
      scale.peel();   // 또는 tare()
      nearZeroStart = 0;
      Serial.println("Auto zeroed");
    }
  } else {
    nearZeroStart = 0;
  }

  delay(200);
}
