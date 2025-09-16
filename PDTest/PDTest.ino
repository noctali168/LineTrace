int state = 0;
//0:通常 1:障害物回避中 2:直角

// --- センサピン（アナログ入力） ---
const int sensorR = 28;
const int sensorC = 27;
const int sensorL = 26;

// --- モータ制御ピン ---
const int lCCP_Pin = 5, lSEL1_Pin = 11, lSEL2_Pin = 12;
const int rCCP_Pin = 7, rSEL1_Pin = 8, rSEL2_Pin = 9;

//超音波センサ
const int TRIG_Pin = 14;
const int ECHO_Pin = 15;
const int sound_speed = 340;
float duration = 0;
float distance = 0;

//LED
const int LED1_Pin = 18;
const int LED2_Pin = 19;

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

  pinMode(TRIG_Pin, OUTPUT);
  pinMode(ECHO_Pin, INPUT);

  pinMode(rCCP_Pin, OUTPUT);
  pinMode(rSEL1_Pin, OUTPUT);
  pinMode(rSEL2_Pin, OUTPUT);
  pinMode(lCCP_Pin, OUTPUT);
  pinMode(lSEL1_Pin, OUTPUT);
  pinMode(lSEL2_Pin, OUTPUT);

  pinMode(LED1_Pin, OUTPUT);
  pinMode(LED2_Pin, OUTPUT);

  Serial.begin(9600);
}

void loop() {
  // センサ読み取り
  readSensors();

  if(distance < 10.0 && state == 0){//障害物検知
    Serial.println(distance);
    // AvoidObstacles();
  }

  if(valL > 2500 && valC > 2500){
    RightAngle();
  }

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

  //超音波センサ
  digitalWrite(TRIG_Pin, LOW);
  delayMicroseconds(1);
  digitalWrite(TRIG_Pin, HIGH);
  delayMicroseconds(8);
  digitalWrite(TRIG_Pin, LOW);
  duration = pulseIn(ECHO_Pin,HIGH);
  duration = duration/2;
  distance = duration*100/1000000*sound_speed;
}

// --- 誤差計算（中央センサのみ） ---
void computeError() {
  error = (float)valC - target_val;
  error /= 1000;

  if(state == 1 && error > 0){
    SetState(0);
  }
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
  float r_speed;
  float l_speed;

  switch(state){
    case 0:
      l_speed = default_speed + control;
      r_speed = default_speed - control;
      break;
    case 1:
      l_speed = default_speed * 1.1;
      r_speed = default_speed * 0.7;
      break;
    default:
      l_speed = 0;
      r_speed = 0;
      break;
  }

  r_speed = constrain(r_speed, 0, 255);
  l_speed = constrain(l_speed, 0, 255);

  analogWrite(rCCP_Pin, r_speed);
  digitalWrite(rSEL1_Pin, HIGH);
  digitalWrite(rSEL2_Pin, LOW);

  analogWrite(lCCP_Pin, l_speed);
  digitalWrite(lSEL1_Pin, HIGH);
  digitalWrite(lSEL2_Pin, LOW);
}

void AvoidObstacles(){
  SetState(1);

  analogWrite(rCCP_Pin, default_speed);
  digitalWrite(rSEL1_Pin, HIGH);
  digitalWrite(rSEL2_Pin, LOW);

  analogWrite(lCCP_Pin, 0);
  digitalWrite(lSEL1_Pin, LOW);
  digitalWrite(lSEL2_Pin, LOW);

  delay(1500);
}

void RightAngle(){
  SetState(2);
  Serial.println("RightAngle");

  analogWrite(rCCP_Pin, default_speed);
  digitalWrite(rSEL1_Pin, HIGH);
  digitalWrite(rSEL2_Pin, LOW);

  analogWrite(lCCP_Pin, default_speed / 2);
  digitalWrite(lSEL1_Pin, HIGH);
  digitalWrite(lSEL2_Pin, LOW);

  delay(1500);

  SetState(0);
}

void LEDControll(int num, bool status){
  if(num == 1){
    if(status){
      digitalWrite(LED1_Pin, HIGH);
    }else{
      digitalWrite(LED1_Pin, LOW);
    }
  }else{
    if(status){
      digitalWrite(LED2_Pin, HIGH);
    }else{
      digitalWrite(LED2_Pin, LOW);
    }
  }
}

void SetState(int num){ //状態をセット
  state = num;
  switch(state){
    case 0:
      LEDControll(1, false);
      LEDControll(2, false);
      break;
    case 1:
      LEDControll(1, true);
      LEDControll(2, false);
      break;
    case 2:
      LEDControll(1, false);
      LEDControll(2, true);
      break;
    default:
      LEDControll(1, false);
      LEDControll(2, false);
      break;
  }
}

// --- デバッグ出力 ---
void Debug() {
  Serial.print("L:"); Serial.print(valL);
  Serial.print(" C:"); Serial.print(valC);
  Serial.print(" R:"); Serial.println(valR);
  // Serial.print(" | Error:"); Serial.print(error, 3);
  // Serial.print(" | Control:"); Serial.println(control, 3);
  // Serial.print("distance: "); Serial.println(distance);
  Serial.println(state);
}
