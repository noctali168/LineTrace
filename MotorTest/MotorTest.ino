// モータ制御ピン
const int lCCP_Pin = 5, lSEL1_Pin = 11, lSEL2_Pin = 12;
const int rCCP_Pin = 7, rSEL1_Pin = 8, rSEL2_Pin = 9;

// LEDピン
const int LED1_Pin = 18; // GP18
const int LED2_Pin = 19; // GP19

void setup() {
  // モータ
  pinMode(rCCP_Pin, OUTPUT);
  pinMode(rSEL1_Pin, OUTPUT);
  pinMode(rSEL2_Pin, OUTPUT);

  pinMode(lCCP_Pin, OUTPUT);
  pinMode(lSEL1_Pin, OUTPUT);
  pinMode(lSEL2_Pin, OUTPUT);

  // LED
  pinMode(LED1_Pin, OUTPUT);
  pinMode(LED2_Pin, OUTPUT);

  Serial.begin(9600);
  Serial.println("Motor + LED Test Start");
}

void loop() {
  // 前進
  ledOn();
  setMotor(100, true, 100, true);
  Serial.println("Forward");
  delay(2000);

  stopMotors();
  ledOff();
  delay(1000);

  ledOn();
  setMotor(50, true, 50, true);
  Serial.println("Forward");
  delay(2000);

  stopMotors();
  ledOff();
  delay(1000);

  // 後退
  ledOn();
  setMotor(100, false, 100, false);
  Serial.println("Backward");
  delay(2000);

  stopMotors();
  ledOff();
  delay(1000);

  // 右旋回
  // ledOn();
  // setMotor(100, true, 100, false);
  // Serial.println("Right Turn");
  // delay(2000);

  // stopMotors();
  // ledOff();
  // delay(1000);

  // // 左旋回
  // ledOn();
  // setMotor(100, false, 100, true);
  // Serial.println("Left Turn");
  // delay(2000);

  // stopMotors();
  // ledOff();
  // delay(2000);
}

// モータ動作
void setMotor(int rSpeed, bool rForward, int lSpeed, bool lForward) {
  analogWrite(rCCP_Pin, rSpeed);
  digitalWrite(rSEL1_Pin, rForward);
  digitalWrite(rSEL2_Pin, !rForward);

  analogWrite(lCCP_Pin, lSpeed);
  digitalWrite(lSEL1_Pin, lForward);
  digitalWrite(lSEL2_Pin, !lForward);
}

// 停止
void stopMotors() {
  analogWrite(rCCP_Pin, 0);
  digitalWrite(rSEL1_Pin, LOW);
  digitalWrite(rSEL2_Pin, LOW);
  analogWrite(lCCP_Pin, 0);
  digitalWrite(lSEL1_Pin, LOW);
  digitalWrite(lSEL2_Pin, LOW);
}

// LED制御
void ledOn() {
  digitalWrite(LED1_Pin, HIGH);
  digitalWrite(LED2_Pin, HIGH);
}

void ledOff() {
  digitalWrite(LED1_Pin, LOW);
  digitalWrite(LED2_Pin, LOW);
}
