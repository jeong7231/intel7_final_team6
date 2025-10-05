// L298N (채널 A) + Arduino (옵션 A: ENA 점퍼 꽂음, PWM 미사용)

const int IN1 = 7;   // 방향 입력 1
const int IN2 = 8;   // 방향 입력 2

// 안전한 방향 전환을 위한 짧은 브레이크 시간(필요 시 조정)
const unsigned long DIR_CHANGE_BRAKE_MS = 150;

void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  // 초기: 브레이크 해제 대신 하단 단락 브레이크(안전하게 정지 상태)
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);  // 옵션 A에선 이것도 브레이크(하단 단락)입니다.
}

// 정방향 주행
void runForward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
}

// 역방향 주행
void runReverse() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
}

// 전기적 브레이크(급정지)
// IN1=IN2=HIGH(상단 단락) 또는 IN1=IN2=LOW(하단 단락) 둘 다 브레이크
void brake() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, HIGH);
}

// 안전한 방향 전환: 잠깐 브레이크 후 반대 방향
void changeDirectionSafely(bool toForward) {
  brake();
  delay(DIR_CHANGE_BRAKE_MS);
  if (toForward) runForward();
  else           runReverse();
}

void loop() {
  // 데모: 정방향 2초 → 브레이크 0.5초 → 역방향 2초 → 브레이크 0.5초 반복
  runForward();
  delay(2000);

  brake();
  delay(500);

  changeDirectionSafely(false); // 역방향으로 전환
  delay(2000);

  brake();
  delay(500);
}
