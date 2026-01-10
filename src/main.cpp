#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "ICM_20948.h"
#include <math.h>
#include "par_pid.h"  

Adafruit_PWMServoDriver pwm(PCA_ADDR);  // PCA9685 address
ICM_20948_I2C myICM;  // IMU
HardwareSerial &sbusSerial = Serial1;  // SBUS
uint8_t  sbusBuf[SBUS_FRAME_SIZE];
uint16_t sbusCh[16];
bool     haveFrame = false;
uint32_t lastSbusMs = 0;
//---------------------------參數在main.cpp宣告---------------------------
// 姿態角（估測）
float pitch_deg = 0.0f, roll_deg = 0.0f, yaw_deg = 0.0f;
// 零點 offset（按 CH_CALIB 校正）
float pitch_offset = 0.0f, roll_offset = 0.0f;
bool  calibPrevActive = false;
// Gyro bias
float gx_bias = 0.0f, gy_bias = 0.0f, gz_bias = 0.0f;
// PI 狀態 積分記憶
float Pitch_rateInt = 0.0f, Roll_rateInt  = 0.0f;
// [ADD] 外迴路（角度 loop）I 狀態（Ki=0 時不影響）
float Pitch_angInt = 0.0f, Roll_angInt = 0.0f;   // [ADD]
//內迴路（rate loop）D 狀態（Kd=0 時不影響）
float Pitch_rateErrPrev = 0.0f, Roll_rateErrPrev = 0.0f; // [ADD]
float Pitch_attitude_dterm_filt  = 0.0f, Roll_attitude_dterm_filt  = 0.0f; // [ADD]
float pitch_rate_dterm = 0.0f, roll_rate_dterm = 0.0f; // [ADD]
float Pitch_rate_dterm_filt = 0.0f, Roll_rate_dterm_filt = 0.0f;
// 用於紀錄上一次迴圈之誤差值
float pitch_angle_error_previous = 0.0f, roll_angle_error_previous = 0.0f;
// loop timing
int lastLoopUs = 0;
//---------------------------參數宣告結束---------------------------
  
static inline float clampf(float x, float low, float high) {  //限制大小的function；inline :單行描述函數(直接展開)
  if (x < low) return low;    
  if (x > high) return high;
  return x;
}

void locked(){
  pwm.setPWM(PCA_SERVO_CH1, 0, SERVO_MIN_TICK);
  pwm.setPWM(PCA_SERVO_CH2, 0, SERVO_MIN_TICK);
  pwm.setPWM(PCA_ESC_CH1, 0, 0);
  pwm.setPWM(PCA_ESC_CH2, 0, 0);
}

static inline float wrapAngle180(float degree) {   //將角度限制在+-180度
  while (degree <= -180.0f) degree += 360.0f;
  while (degree >  180.0f)  degree -= 360.0f;
  return degree;
}

static inline float sbus_normalize(int stick_value) {  //搖桿數值轉-1到1(servo用)
  float stick_norm = (stick_value - STICK_MIN) / float(STICK_MAX - STICK_MIN); // 0..1
  stick_norm = clampf(stick_norm, 0.0f, 1.0f);
  return stick_norm * 2.0f - 1.0f; // -1..1
}

static inline float sbusTo01(int stick_value) {   //搖桿數值轉0到1(油門用)
  float stick_norm_2 = (stick_value - STICK_MIN) / float(STICK_MAX - STICK_MIN);
  stick_norm_2 = clampf(stick_norm_2, 0.0f, 1.0f);
  return stick_norm_2;
}

float usToTicks(float us) {  //秒數轉ticks(1 tick = 1/4096 of frame)
  float ticks = us * 4096.0f * SERVO_FREQ / 1000000.0f;
  return ticks;
}

int angleToTick(float ang){
  ang = clampf(ang, SERVO_MIN_ANG, SERVO_MAX_ANG);
  float t = (ang - SERVO_MIN_ANG) / (SERVO_MAX_ANG - SERVO_MIN_ANG); //0..1
  return (int)(SERVO_MIN_TICK + t * (SERVO_MAX_TICK - SERVO_MIN_TICK));  //ticks數值
}

void setServoAngle(uint8_t ch, float ang){
  pwm.setPWM(ch, 0, angleToTick(ang));
}

// float toticks_servo(float servo) {  // Servo設定: 500~2500us(跟油門us範圍不同所以額外寫一個function)
//   servo = constrain(servo, 0.0f, 1.0f);
//   float servo_us = 500.0f + servo * 2000.0f; 
//   float tick = usToTicks(servo_us);
//   return tick;
// }
void setESC01(uint8_t ch, float x01) {  //油門0.3-1.0(1000~2000us)
  x01 = clampf(x01, 0.0f, 1.0f);
  float us = 1000.0f + 1000.0f * x01;
  float esc_ticks = usToTicks(us);
  pwm.setPWM(ch, 0, esc_ticks);
}


// void setServoAngle(uint8_t ch, float servo_angle) {  //servo角度set(70~110度)
//   servo_angle = clampf(servo_angle, (float)SERVO_MIN_ANG, (float)SERVO_MAX_ANG);
//   float servo_norm = (servo_angle - SERVO_MIN_ANG) / float(SERVO_MAX_ANG - SERVO_MIN_ANG); // 0到1
//   int servo_tick = (int)(toticks_servo(servo_norm));
//   servo_tick = map(servo_tick, STICK_MIN_50HZ, STICK_MAX_50HZ, SERVO_MIN_TICK, SERVO_MAX_TICK); //將搖桿最大範圍映射到servo的最大範圍
//   pwm.setPWM(ch, 0, servo_tick);
// }

void decodeSbusFrame(const uint8_t *buf, uint16_t *ch) { //SBUS解碼
  ch[0]  = ((uint16_t)buf[1]  | ((uint16_t)buf[2]  << 8)) & 0x07FF;
  ch[1]  = (((uint16_t)buf[2] >> 3) | ((uint16_t)buf[3]  << 5)) & 0x07FF;
  ch[2]  = (((uint16_t)buf[3] >> 6) | ((uint16_t)buf[4]  << 2) | ((uint16_t)buf[5]  << 10)) & 0x07FF;
  ch[3]  = (((uint16_t)buf[5] >> 1) | ((uint16_t)buf[6]  << 7)) & 0x07FF;
  ch[4]  = (((uint16_t)buf[6] >> 4) | ((uint16_t)buf[7]  << 4)) & 0x07FF;
  ch[5]  = (((uint16_t)buf[7] >> 7) | ((uint16_t)buf[8]  << 1) | ((uint16_t)buf[9]  << 9)) & 0x07FF;
  ch[6]  = (((uint16_t)buf[9] >> 2) | ((uint16_t)buf[10] << 6)) & 0x07FF;
  ch[7]  = (((uint16_t)buf[10]>> 5) | ((uint16_t)buf[11] << 3)) & 0x07FF;
  ch[8]  = ((uint16_t)buf[12] | ((uint16_t)buf[13] << 8)) & 0x07FF;
  ch[9]  = (((uint16_t)buf[13]>> 3) | ((uint16_t)buf[14] << 5)) & 0x07FF;
}

bool tryReadSbusFrame() {  //嘗試讀取SBUS frame，並存取到sbusCh陣列
  static int idx = 0;
  while (sbusSerial.available()) {
    uint8_t data = (uint8_t)sbusSerial.read();
    if (idx == 0) {
      if (data == 0x0F) {
        sbusBuf[0] = data;
        idx = 1;
      }
    } else {
      sbusBuf[idx++] = data;
      if (idx >= SBUS_FRAME_SIZE) {
        idx = 0;
        decodeSbusFrame(sbusBuf, sbusCh);
        lastSbusMs = millis();
        haveFrame = true;
        return true;
      }
    }
  }
  return false;
}

void readIMU(float &axg, float &ayg, float &azg, float &gx_dps, float &gy_dps, float &gz_dps){
    // 讀取 IMU
    myICM.getAGMT();
    // 原始量測
    float ax_mg = myICM.accX();  // mg
    float ay_mg = myICM.accY();
    float az_mg = myICM.accZ();

    gx_dps = myICM.gyrX() - gx_bias;  // deg/s
    gy_dps = myICM.gyrY() - gy_bias;
    gz_dps = myICM.gyrZ() - gz_bias;

    axg = ax_mg / 1000.0f;    // mg → g
    ayg = ay_mg / 1000.0f;
    azg = az_mg / 1000.0f;
}

void updateAttitudeComplementary(float gx_dps, float gy_dps, float gz_dps, float accPitch, float accRoll, float dt, float &pitch_deg, 
  float &roll_deg, float &yaw_deg) {
    // Gyro 積分（短時間內姿態推算）
    float pitch_gyro = pitch_deg + gx_dps * dt;
    float roll_gyro  = roll_deg  + gy_dps * dt;
    float yaw_gyro   = yaw_deg   + gz_dps * dt;

    // 互補濾波（融合加速度計）
    pitch_deg = COMPLEMENTARY_ALPHA * pitch_gyro
              + (1.0f - COMPLEMENTARY_ALPHA) * accPitch;

    roll_deg  = COMPLEMENTARY_ALPHA * roll_gyro
              + (1.0f - COMPLEMENTARY_ALPHA) * accRoll;

    // yaw 沒有加速度參考，只做 gyro 積分
    yaw_deg = wrapAngle180(yaw_gyro);
}


void calibrateGyroBias() { //校正一開始的偏移
  Serial.println("正在校正IMU，請保持靜止...");
  gx_bias = gy_bias = gz_bias = 0;  //重置bias值
  for (uint16_t i = 0; i < gyroCalibSamples; i++) {  //取樣300次做平均(在靜止之下)
    myICM.getAGMT();
    gx_bias += myICM.gyrX();
    gy_bias += myICM.gyrY();
    gz_bias += myICM.gyrZ();
    delay(5);
  }
  gx_bias /= gyroCalibSamples;
  gy_bias /= gyroCalibSamples;
  gz_bias /= gyroCalibSamples;
  Serial.println("IMU校正成功。");
}

float Pitch_ratePID(float pitch_rateErr, float dt) {  //PI控制器(內迴路)
  // I
  Pitch_rateInt += pitch_rateErr * dt; //積分
  Pitch_rateInt = clampf(Pitch_rateInt, -300.0f, 300.0f);  //積分限幅防止wind-up

  // D = d(error)/dt  (加 dt 保護 + 濾波)
  pitch_rate_dterm = 0.0f; //微分
  if (dt > 1e-6f) { // 防止除以零
    float d_raw = (pitch_rateErr - Pitch_rateErrPrev) / dt;

    float tau = 1.0f / (2.0f * PI * DTERM_CUTOFF_HZ);
    float alpha = dt / (dt + tau);
    Pitch_rate_dterm_filt += alpha * (d_raw - Pitch_rate_dterm_filt);

    pitch_rate_dterm = Pitch_rate_dterm_filt;
  }

  Pitch_rateErrPrev = pitch_rateErr; //更新前次誤差

  return Kp_pitch_rate * pitch_rateErr + Ki_pitch_rate * Pitch_rateInt + Kd_pitch_rate * pitch_rate_dterm;  //回傳控制量
}


float Roll_ratePID(float rateErr, float dt) {
  Roll_rateInt += rateErr * dt;
  Roll_rateInt = clampf(Roll_rateInt, -300.0f, 300.0f);

  // D = d(error)/dt（加 dt 保護 + 濾波）
  roll_rate_dterm = 0.0f;
  if (dt > 1e-6f) { // 防止除以零
    float d_raw = (rateErr - Roll_rateErrPrev) / dt;


    float tau = 1.0f / (2.0f * PI * DTERM_CUTOFF_HZ);
    float alpha = dt / (dt + tau);
    Roll_rate_dterm_filt += alpha * (d_raw - Roll_rate_dterm_filt);

    roll_rate_dterm = Roll_rate_dterm_filt;

  }
  Roll_rateErrPrev = rateErr;

  float u = Kp_roll_rate * rateErr + Ki_roll_rate * Roll_rateInt + Kd_roll_rate * roll_rate_dterm;  //回傳控制量;
  u = clampf(u, -ROLL_DIFF_MAX, +ROLL_DIFF_MAX); // 限制差動推力幅度
  return u;   // 回傳「左右油門差動量」
}

void resetPI() {
  // inner-loop I
  Pitch_rateInt = 0.0f;
  Roll_rateInt  = 0.0f;

  // outer-loop I
  Pitch_angInt = 0.0f;
  Roll_angInt  = 0.0f;

  // inner-loop D states (if you already added D in rate loop)
  Pitch_rateErrPrev = 0.0f;
  Roll_rateErrPrev  = 0.0f;
  Pitch_attitude_dterm_filt  = 0.0f;
  Roll_attitude_dterm_filt   = 0.0f;
  Pitch_rate_dterm_filt = 0.0f;
  Roll_rate_dterm_filt  = 0.0f;
  pitch_rate_dterm = 0.0f;
  roll_rate_dterm  = 0.0f;
}


void ZeroCalibration(bool valid, int calib_raw, float pitch_deg, float roll_deg) {
  if (!valid) return;
  if (calib_raw < CALIB_LOW_TH && !calibPrevActive) {
    pitch_offset = pitch_deg;
    roll_offset  = roll_deg;
    calibPrevActive = true;
    resetPI();
    Serial.println("IMU 零點校正完成");
  }

  if (calib_raw > CALIB_HIGH_TH) {
    calibPrevActive = false;
  }
}

//----------------------------------姿態(角度)控制器-------------------------------------- 
float attitude_Pitch_PID(float angle_cmd_deg, float angle_meas_deg,
                         float dt, float Kp, float Ki, float Kd,
                         float rate_limit_dps) {
  float pitch_angle_error = (angle_cmd_deg - angle_meas_deg) * SIGN_PITCH;

  // I (angle error integral)
  Pitch_angInt += pitch_angle_error * dt;
  Pitch_angInt = clampf(Pitch_angInt, -ANG_INT_LIM_PITCH, +ANG_INT_LIM_PITCH);

  float pitch_attitude_d_raw = 0.0f;
  if (dt > 1e-6f) {  // 防止除以零
  pitch_attitude_d_raw = (pitch_angle_error - pitch_angle_error_previous) / dt;
  float tau = 1.0f / (2.0f * PI * DTERM_CUTOFF_HZ);
  float alpha = dt / (dt + tau);
  Pitch_attitude_dterm_filt += alpha * (pitch_attitude_d_raw - Pitch_attitude_dterm_filt);
  }

  float rate_cmd = Kp * pitch_angle_error + Ki * Pitch_angInt + Kd * Pitch_attitude_dterm_filt;
  rate_cmd = clampf(rate_cmd, -rate_limit_dps, +rate_limit_dps);
  pitch_angle_error_previous = pitch_angle_error;  // 更新上一次的角度誤差
  return rate_cmd;
}

float attitude_Roll_PID(float angle_cmd_deg, float angle_meas_deg,
                        float dt, float Kp, float Ki, float Kd,
                        float rate_limit_dps) {

  float roll_angle_error = (angle_cmd_deg - angle_meas_deg) * SIGN_ROLL;

  // I (angle error integral)
  Roll_angInt += roll_angle_error * dt;
  Roll_angInt = clampf(Roll_angInt, -ANG_INT_LIM_ROLL, +ANG_INT_LIM_ROLL);

  // D (error derivative)

  float roll_attitude_d_raw = 0.0f;
  if (dt > 1e-6f) {  // 防止除以零
    roll_attitude_d_raw = (roll_angle_error - roll_angle_error_previous) / dt;
    float tau = 1.0f / (2.0f * PI * DTERM_CUTOFF_HZ);
    float alpha = dt / (dt + tau);
    Roll_attitude_dterm_filt += alpha * (roll_attitude_d_raw - Roll_attitude_dterm_filt);  //filt是上一筆的值
  }

  float rate_cmd = Kp * roll_angle_error + Ki * Roll_angInt + Kd * Roll_attitude_dterm_filt;
  rate_cmd = clampf(rate_cmd, -rate_limit_dps, +rate_limit_dps);

  roll_angle_error_previous = roll_angle_error;  // 更新上一次的角度誤差
  return rate_cmd;
}
//---------------------------------------------------------------------------------------


//----------------------------------角速度控制器--------------------------------------
float Pitch_angle_rate_controller(float gx_dps, float pitch_rate_cmd, float dt){
  float pitch_rate_measure = gx_dps; //實際角速度(deg/s)
  float pitch_rateErr = pitch_rate_cmd - pitch_rate_measure; // 角速度誤差
  float pitch_command = Pitch_ratePID(pitch_rateErr, dt);   //用PI控制器算出來的pitch指令(deg)
  pitch_command = clampf(pitch_command, -(float)SERVO_PITCH_RANGE, +(float)SERVO_PITCH_RANGE);
  return pitch_command;
}

float Roll_angle_rate_controller(float gy_dps, float roll_rate_cmd, float dt){
  float roll_rate_measure = gy_dps; //實際角速度(deg/s)
  float roll_rateErr = roll_rate_cmd - roll_rate_measure; // 角速度誤差
  float roll_command = Roll_ratePID(roll_rateErr, dt);   //用PI控制器算出來的roll指令(deg)
  roll_command = clampf(roll_command, -(float)ROLL_DIFF_MAX, +(float)ROLL_DIFF_MAX); // 限幅 
  return roll_command;
}
//---------------------------------------------------------------------------------------

void mixer_pitch_servo(float pitch_cmd, float *servo1_deg, float *servo2_deg){
  pitch_cmd = clampf(pitch_cmd, -30.0f, +30.0f);

  float s1 = SERVO_CENTER + pitch_cmd;   // 正向
  float s2 = SERVO_CENTER - pitch_cmd;   // 反向

  s1 = clampf(s1, (float)SERVO_MIN_ANG, (float)SERVO_MAX_ANG);
  s2 = clampf(s2, (float)SERVO_MIN_ANG, (float)SERVO_MAX_ANG);

  *servo1_deg = s1;
  *servo2_deg = s2;
}

void mixer_roll_esc(float thr01, float roll_u, float *esc1, float *esc2) {
  // roll_u 是差動量（±ROLL_DIFF_MAX），由 Roll_ratePID 輸出
  float e1 = thr01 + roll_u;   // 左
  float e2 = thr01 - roll_u;   // 右

  *esc1 = clampf(e1, 0.0f, 1.0f);
  *esc2 = clampf(e2, 0.0f, 1.0f);
}

void outputESCServo(bool is_unlocked, bool valid, float esc1_speed, float esc2_speed, float servo1_deg, float servo2_deg){
  if (!is_unlocked || !valid) {
    // PWM 輸出
    pwm.setPWM(PCA_ESC_CH1, 0, 0);
    pwm.setPWM(PCA_ESC_CH2, 0, 0);

    // 伺服回中間
    setServoAngle(PCA_SERVO_CH1, SERVO_CENTER);
    setServoAngle(PCA_SERVO_CH2, SERVO_CENTER);

    resetPI();
    return;
  }

  setESC01(PCA_ESC_CH1, esc1_speed);
  setESC01(PCA_ESC_CH2, esc2_speed);
  setServoAngle(PCA_SERVO_CH1, servo1_deg);
  setServoAngle(PCA_SERVO_CH2, servo2_deg);
}

void setup() {
  Serial.begin(115200);
  delay(100);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);  

  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);  // 50Hz

  sbusSerial.begin(SBUS_BAUD, SERIAL_8E2, SBUS_RX_PIN, -1);  
  sbusSerial.setRxInvert(true);  //反向接收
  
  if (myICM.begin(Wire, ICM_ADDRESS) != ICM_20948_Stat_Ok) {
    Serial.println("ICM20948 not detected!");
    while (1) delay(100);
  }
  calibrateGyroBias();  //校正gyro bias

  myICM.getAGMT();  //取得初始加速度計數值以計算初始角度
  float ax = myICM.accX() / 1000.0f;
  float ay = myICM.accY() / 1000.0f;
  float az = myICM.accZ() / 1000.0f;

  pitch_deg = atan2f(ay, sqrtf(ax*ax + az*az)) * 180.0f / PI;   //初始俯仰角pitch
  roll_deg  = atan2f(-ax, az) * 180.0f / PI;  //初始滾轉角 roll
  yaw_deg   = 0.0f;  //初始偏航角 yaw

  pitch_offset = pitch_deg; //設定初始校正值
  roll_offset = roll_deg;

  // init ESC
  setESC01(PCA_ESC_CH1, 0.0f);
  setESC01(PCA_ESC_CH2, 0.0f);
  // init Servo
  setServoAngle(PCA_SERVO_CH1, SERVO_CENTER);
  setServoAngle(PCA_SERVO_CH2, SERVO_CENTER);

  lastLoopUs = micros();

  Serial.println("SBUS + ICM20948 + Cascaded PI (Pitch+Roll) 控制器啟動完成。");
}

void loop() {
  tryReadSbusFrame();

  //loop限制頻率
  uint32_t nowUs = micros();
  if ((uint32_t)(nowUs - lastLoopUs) < LOOP_DT_US) return; //不夠時間就跳出
  float dt = (nowUs - lastLoopUs) / 1e6f;  //經過的秒數(給積分器用)
  lastLoopUs += LOOP_DT_US;
  if (dt <= 0 || dt > 0.05f) dt = 1.0f / LOOP_HZ;  //防止異常dt值

  //是否為正常解碼
  bool valid = haveFrame && ((millis() - lastSbusMs) < FAILSAFE_TIMEOUT_MS);

  // arm switch:小於1000為解鎖, 大於1000為鎖定
  bool is_unlocked = false;
  if (valid) {
    is_unlocked = (sbusCh[CH_ARM] < 1000);
  }

  // ---------------- IMU更新 -----------------
  float axg, ayg, azg;
  float gx_dps, gy_dps, gz_dps;
  readIMU(axg, ayg, azg, gx_dps, gy_dps, gz_dps);
  // ------------------------------------------

  // 計算角度
  float accPitch = atan2f(ayg, sqrtf(axg*axg + azg*azg)) * 180.0f / PI; 
  float accRoll  = atan2f(-axg, azg) * 180.0f / PI;

  updateAttitudeComplementary(gx_dps, gy_dps, gz_dps, accPitch, accRoll, dt, pitch_deg, roll_deg, yaw_deg);  //更新姿態角(加上互補濾波)

  // ---------------- CH8零點校正----------------
  int calib_raw = sbusCh[CH_CALIB];
  ZeroCalibration(valid, calib_raw, pitch_deg, roll_deg);

  // ----------------指令----------------
  float thr01 = 0.0f;             //油門
  float pitch_cmd_deg = 0.0f;     // deg  
  float roll_cmd_deg = 0.0f;     // deg

  if (valid) {
    //油門0.3~1
    thr01 = sbusTo01((int)sbusCh[CH_THROTTLE]);
    const float THR_MIN = 0.30f;
    thr01 = THR_MIN + thr01 * (1.0f - THR_MIN);
    thr01 = clampf(thr01, THR_MIN, 1.0f);

    // pitch指令-1到1
    float pitchStick = sbus_normalize((int)sbusCh[CH_PITCH]);
    pitch_cmd_deg = pitchStick * PITCH_CMD_MAX_DEG;

    float rollStick = sbus_normalize((int)sbusCh[CH_ROLL]);
    roll_cmd_deg = rollStick * ROLL_CMD_MAX_DEG;
  }

  // ---------------- Cascaded Control (Pitch) ----------------

  float pitch_corr = pitch_deg - pitch_offset; // 先做零點校正
  float roll_corr = roll_deg - roll_offset; // 先做零點校正

  //-------------------Outer: 角度誤差 -> 目標角速度(使用P控制器) -----內迴路應該達到的角速度目標----------------------

  float pitch_rate_cmd = attitude_Pitch_PID(pitch_cmd_deg, pitch_corr, dt, Kp_pitch_ang, Ki_pitch_ang, Kd_pitch_ang, MAX_PITCH_RATE_DPS); //角速度pitch
  float roll_rate_cmd = attitude_Roll_PID(roll_cmd_deg, roll_corr, dt, Kp_roll_ang, Ki_roll_ang, Kd_roll_ang, MAX_ROLL_RATE_DPS);  //角速度roll

  //------------------------------------Outer End---------------------------------------------------------


  // ----------------Inner: 角速度誤差 -> pitch的輸出指令(使用PI控制器)  -------------------------

  float pitch_output_servo = Pitch_angle_rate_controller(gx_dps, pitch_rate_cmd, dt); // 輸出最終pitch角度控制指令
  float roll_output_throttle = Roll_angle_rate_controller(gy_dps, roll_rate_cmd, dt); // 輸出最終roll角度控制指令
  
  //------------------------------------Inner End---------------------------------------------------------

  //------------------ Mixer ------------------

  float servo1_deg, servo2_deg; //兩個伺服角度值
  float esc1_speed, esc2_speed;   //兩個ESC速度值

  mixer_pitch_servo(pitch_output_servo, &servo1_deg, &servo2_deg); //計算兩個伺服角度值
  mixer_roll_esc(thr01, roll_output_throttle, &esc1_speed, &esc2_speed); //計算兩個ESC調整之速度值

  //------------------ Mixer End ------------------


  // ---------------- ESC & Servo 輸出----------------
  outputESCServo(is_unlocked, valid, esc1_speed, esc2_speed, servo1_deg, servo2_deg);
  // -------------------------------------------------

  // ----------------Print----------------
  static uint32_t lastPrintMs = 0;
  if (millis() - lastPrintMs > 400) {
    lastPrintMs = millis();
    Serial.print(is_unlocked ? "解鎖" : "LOCK");
    Serial.print("油門: "); Serial.print(thr01, 3);
    Serial.print(" | pitch實際角: "); Serial.print(pitch_corr, 2);
    Serial.print(" | p目標角: "); Serial.print(pitch_cmd_deg, 2);
    Serial.print(" | pitch實際輸出(deg): ");    Serial.print(pitch_output_servo, 2);

    Serial.print(" | roll實際角: "); Serial.print(roll_corr, 2);
    Serial.print(" | r目標角: "); Serial.print(roll_cmd_deg, 2);

    Serial.print(" | gx(角速度,deg/s): "); Serial.print(gx_dps, 2);
    Serial.print(" | gy(角速度,deg/s): "); Serial.print(gy_dps, 2);
    Serial.print(" | rate_cmd(目標角速度,deg/s): "); Serial.print(pitch_rate_cmd, 2);

    Serial.print(" | 左伺服角: "); Serial.print(servo1_deg, 1);
    Serial.print(" | 右伺服角: "); Serial.print(servo2_deg, 1);
    Serial.print(" | 左ESC速度: "); Serial.print(esc1_speed, 2);
    Serial.print(" | 右ESC速度: "); Serial.print(esc2_speed, 2);
    Serial.println();
  }
}
