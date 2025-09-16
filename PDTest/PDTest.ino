// --- センサピン（アナログ入力） ---
const int sensorL = 28;
const int sensorC = 27;
const int sensorR = 26;

// --- モータ制御ピン ---
const int rCCP_Pin = 5, rSEL1_Pin = 11, rSEL2_Pin = 12;
const int lCCP_Pin = 7, lSEL1_Pin = 8, lSEL2_Pin = 9;

// --- 制御パラメータ ---
const int default_speed = 60;      // 基本速度
const float Kp = 10.0;             // 比例ゲイン（直線補正強化）
const float Kd = 20.0;             // 微分ゲイン（振動抑制）
const float max_control = 25.0;    // 最大速度差

// --- センサ値 ---
const int white_val = 1000;        // 白
const int black_val = 3200;        // 黒
const int target_val = (white_val + black_val) / 2;

// --- PD制御用変数 ---
float valL, valC, valR;
float error = 0;
float prev_error = 0;
float control = 0;
float prev_control = 0; // スムージング用

void setup() {
  analogReadResolution(12);
  pinMode(sensorL, INPUT);
  pinMode(sensorC, INPUT);
  pinMode(sensorR, INPUT);

  pinMode(rCCP_Pin, OUTPUT);
  pinMode(rSEL1_Pin, OUTPUT);
  pinMode(rSEL2_Pin, OUTPUT);
  pinMode(lCCP_Pin, OUTPUT);
  pinMode(lSEL1_Pin, OUTPUT);
  pinMode(lSEL2_Pin, OUTPUT);

  Serial.begin(9600);
}

void loop() {
  // センサ読み取り
  readSensors();

  // 誤差計算（中央センサのみ）
  computeError();

  // PD制御（スムージングあり）
  computeControl();

  // モータ制御
  setMotorSpeed();

  // デバッグ出力
  Debug();

  delay(10);
}

// --- センサ読み取り ---
void readSensors() {
  valL = analogRead(sensorL);
  valC = analogRead(sensorC);
  valR = analogRead(sensorR);
}

// --- 誤差計算（中央センサのみ） ---
void computeError() {
  error = (float)valC - target_val;
  error /= 1000;
}

// --- PD制御（スムージングあり） ---
void computeControl() {
  float diff = error - prev_error;
  float raw_control = Kp * error + Kd * diff;

  // スムージングで直線の揺れを抑制
  control = 0.7 * prev_control + 0.3 * raw_control;

  // 最大速度差制限
  control = constrain(control, -max_control, max_control);

  prev_error = error;
  prev_control = control;
}

// --- モータ速度設定 ---
void setMotorSpeed() {
  float r_speed = default_speed + control;
  float l_speed = default_speed - control;

  r_speed = constrain(r_speed, 0, 255);
  l_speed = constrain(l_speed, 0, 255);

  analogWrite(rCCP_Pin, r_speed);
  digitalWrite(rSEL1_Pin, HIGH);
  digitalWrite(rSEL2_Pin, LOW);

  analogWrite(lCCP_Pin, l_speed);
  digitalWrite(lSEL1_Pin, HIGH);
  digitalWrite(lSEL2_Pin, LOW);
}

// --- デバッグ出力 ---
void Debug() {
  Serial.print("L:"); Serial.print(valL);
  Serial.print(" C:"); Serial.print(valC);
  Serial.print(" R:"); Serial.print(valR);
  Serial.print(" | Error:"); Serial.print(error, 3);
  Serial.print(" | Control:"); Serial.println(control, 3);
}
