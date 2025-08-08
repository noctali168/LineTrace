// センサピン（アナログ）
const int sensorL = 28;
const int sensorC = 27;
const int sensorR = 26;

// 超音波用
const int TRIG_Pin = 14;
const int ECHO_Pin = 15;
const int SOKUDO = 340;  // 音速
int duration = 0;

// モータ制御ピン
const int rCCP_Pin = 5, rSEL1_Pin = 11, rSEL2_Pin = 12;
const int lCCP_Pin = 7, lSEL1_Pin = 8, lSEL2_Pin = 9;

//制御パラメータ
const int default_speed = 60;
const float Kp = 0.15;
const float Kd = 1.5;
const float control_gain = 100.0;//タイヤ速度感度

// PD制御用変数
float valL, valC, valR;
float error = 0;
float prev_error = 0;
float control = 0;

//回避モード
bool avoiding = false;
//はみ出た時のリカバー
bool recovering = false;

unsigned long avoidStartTime = 0;
const int avoidDuration = 1000; // 回避を継続する時間 [ms]

bool grayMode = false;
bool grayDetected = false;

float dist;//超音波距離

void setup() {
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

  Serial.begin(9600);
}

void loop() {
  dist = Chouonpa(); // 距離取得

  //  障害物回避処理（最優先）
  if (AvoidMode(dist)) return;

  // センサ読み取り
  readSensors();

  // 灰色検知処理
  if (GrayMode()) return;

  // ラインロスト復帰処理
  if (RecoverMode()) return;

  // 通常PD制御
  handlePDControl();
}

//ラインセンサ読み取り
void readSensors() {
  valL = analogRead(sensorL);
  valC = analogRead(sensorC);
  valR = analogRead(sensorR);
}

// 誤差計算
void computeError() {
  // 白（高値）→ 黒（低値）を反転して黒ほど大きくする
  float invL = 4095 - valL;
  float invC =  4095- valC;
  float invR = 4095 - valR;

  float sum = invL + invC + invR;
  if (sum == 0) sum = 1;  // ゼロ除算防止

  error = (-1.0 * invL + 0.0 * invC + 1.0 * invR) / sum;
}


// PD制御値計算
void computeControl() {
  float diff = error - prev_error;
  control = Kp * error + Kd * diff;
  prev_error = error;
}

//  モータ速度設定 
void setMotorSpeed() {
  float r_speed = default_speed - control * control_gain;
  float l_speed = default_speed + control * control_gain;

  r_speed = constrain(r_speed, 0, 255);
  l_speed = constrain(l_speed, 0, 255);

  analogWrite(rCCP_Pin, r_speed);
  digitalWrite(rSEL1_Pin, HIGH);
  digitalWrite(rSEL2_Pin, LOW);

  analogWrite(lCCP_Pin, l_speed);
  digitalWrite(lSEL1_Pin, LOW);
  digitalWrite(lSEL2_Pin, HIGH);
}

// 左に回避旋回（障害物時）
void avoidLeft() {
  analogWrite(rCCP_Pin, 100);  // 右タイヤ速め
  digitalWrite(rSEL1_Pin, HIGH);
  digitalWrite(rSEL2_Pin, LOW);

  analogWrite(lCCP_Pin, 20);   // 左タイヤ遅め
  digitalWrite(lSEL1_Pin, LOW);
  digitalWrite(lSEL2_Pin, HIGH);
}



void recover() {
  analogWrite(rCCP_Pin, 70);
  digitalWrite(rSEL1_Pin, HIGH);
  digitalWrite(rSEL2_Pin, LOW);
  analogWrite(lCCP_Pin, 70);
  digitalWrite(lSEL1_Pin, HIGH);
  digitalWrite(lSEL2_Pin, LOW);  
}

//通常運転モード
void PDControl() {
  computeError();
  computeControl();
  setMotorSpeed();
  Debug();
  delay(10);
}

bool AvoidMode(float dist) {
  static unsigned long avoidStartTime = 0;
  static int phase = 0;  // 0:未開始, 1:左回避, 2:右復帰

  if (dist > 0 && dist < 20.0 && !avoiding) {
    avoiding = true;
    avoidStartTime = millis();
    phase = 1;
  }

  if (avoiding) {
    unsigned long elapsed = millis() - avoidStartTime;

    if (phase == 1) {
      // フェーズ1：左に回避（道を外れる）
      setMotor(100, true, 30, true);
      if (elapsed > 700) {  // 0.7秒後に復帰フェーズへ
        phase = 2;
        avoidStartTime = millis();  // リセット
      }
    }
    else if (phase == 2) {
      // フェーズ2：右旋回してライン復帰を探す
      setMotor(30, true, 100, true);  // 右旋回

      // ライン復帰 or タイムアウト
      if (analogRead(sensorC) < 400 || millis() - avoidStartTime > 2000) {
        avoiding = false;
        phase = 0;
      }
    }

    delay(10);
    return true;
  }

  return false;
}


//ラインロストモード
bool RecoverMode() {
  static unsigned long recoverStartTime = 0;

  if (!recovering && !avoiding && valL > 3500 && valC > 3500 && valR > 3500) {
    recovering = true;
    recoverStartTime = millis();
  }

  if (recovering) {
    // ゆるく右旋回でライン探す
    setMotor(90, true, 30, true);  

    if (analogRead(sensorC) < 3000) { // 黒検出で終了
      recovering = false;
    }
    delay(10);
    return true;
  }

  return false;
}



//グレイレーンチェンジ
bool GrayMode() {
  if (!grayMode && valC > 2500 && valC < 3500) {
    grayMode = true;
    grayDetected = false;

    // 一瞬だけ右斜めにカーブ
    setMotor(80, true, 40, true);  
    delay(300);
  }

  if (grayMode && !grayDetected) {
    // 斜め後は直進
    setMotor(60, true, 60, true);

    if (valC > 2500 && valC < 3500) {
      grayDetected = true;
    }
    delay(10);
    return true;
  }

  if (grayMode && grayDetected) {
    grayMode = false;  // 復帰
  }

  return grayMode;
}y


// 両輪のスピードと方向を指定する基本関数
void setMotor(int rSpeed, bool rForward, int lSpeed, bool lForward) {
  analogWrite(rCCP_Pin, rSpeed);
  digitalWrite(rSEL1_Pin, rForward);
  digitalWrite(rSEL2_Pin, !rForward);

  analogWrite(lCCP_Pin, lSpeed);
  digitalWrite(lSEL1_Pin, lForward);
  digitalWrite(lSEL2_Pin, !lForward);
}

// 一定時間動かす（方向転換用など）
void moveFor(int rSpeed, bool rFwd, int lSpeed, bool lFwd, int durationMs) {
  setMotor(rSpeed, rFwd, lSpeed, lFwd);
  delay(durationMs);
  stopMotors();
}

//デバッグ出力
void Debug() {
  Serial.print(" Dura:"); Serial.print(duration, 3);
  Serial.print("L:"); Serial.print(valL);
  Serial.print(" C:"); Serial.print(valC);
  Serial.print(" R:"); Serial.print(valR);
  Serial.print(" Error:"); Serial.print(error, 3);
  Serial.print(" Control:"); Serial.print(control, 3);
}
