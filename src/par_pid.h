#ifndef PAR_PID_H
#define PAR_PID_H

#include <Adafruit_PWMServoDriver.h>
#include "ICM_20948.h"
#include <MadgwickAHRS.h>
#include <Adafruit_NeoPixel.h>


// =======================================================
// 腳位設定
// =======================================================
#define SDA_PIN 21              // I2C SDA 腳位
#define SCL_PIN 47              // I2C SCL 腳位
#define DBG_PIN 4              // DEBUG 腳位
#define START_PIN 48  //RGB

// =======================================================
//  PCA9685設定
// =======================================================
#define PCA_ADDR 0x40           //I2C
#define SERVO_FREQ 50        //PWM頻率

//PCA9685通道
#define PCA_ESC_CH1    1       // ESC 1左馬達
#define PCA_ESC_CH2     14        // ESC 2 右馬達
#define PCA_SERVO_CH1  0      // Servo 1 左伺服
#define PCA_SERVO_CH2  15    // Servo 2 右伺服

// us範圍
#define SERVO_MIN_US 1000.0f
#define SERVO_MAX_US 2000.0f

// =======================================================
// servo角度限制
// =======================================================
#define SERVO_MIN_ANG 50        //伺服最小角度
#define SERVO_MAX_ANG 130      //伺服最大角度
#define SERVO_CENTER  90        //伺服中立角度
#define SERVO_PITCH_RANGE 40    //伺服最大輸出範圍

// =======================================================
// IMU設定
// =======================================================
#define ICM_ADDRESS 0x69        // ICM20948位址（依 AD0 可能是 0x68 或 0x69）

// =======================================================
// 控制迴圈頻率
// =======================================================
static const float LOOP_HZ = 400.0f;    //控制迴圈頻率（Hz）
static const int   LOOP_DT_US = (int)(1000000.0f / LOOP_HZ);  //loop的週期（微秒）

// =======================================================
// SBUS接收設定
// =======================================================
#define SBUS_FRAME_SIZE 25       // SBUS frame 25 bytes
static const int SBUS_RX_PIN = 10;     // SBUS RX 腳位
static const int SBUS_BAUD   = 100000; // SBUS baudrate
static const int FAILSAFE_TIMEOUT_MS = 150; // 失聯秒數

//SBUS 原始數值範圍
static const int STICK_MIN = 206;      // SBUS 最小值
static const int STICK_MAX = 1800;     // SBUS 最大值

//SBUS通道
static const int CH_ROLL     = 0;      // Roll滾轉
static const int CH_PITCH    = 1;      // Pitch俯仰
static const int CH_YAW      = 3;      // Yaw側滾
static const int CH_THROTTLE = 2;      // Throttle油門
static const int CH_CALIB    = 8;      // IMU零點校正
static const int CH_ARM      = 9;      // 解鎖開關

// ===== UART 設定 =====
#define MAV_SERIAL   Serial2
#define MAV_TX_PIN   37
#define MAV_RX_PIN   36
#define MAV_BAUD     57600

// CH8零點校正觸發門檻
static const int CALIB_LOW_TH  = 1000; // 小於此值觸發一次校正
static const int CALIB_HIGH_TH = 1500; // 大於此值允許下一次再觸發

// =======================================================
// 遙控指令限制（Stick → 角度/角速度）
// =======================================================
static const float PITCH_CMD_MAX_DEG     = 40.0f;  // 搖桿最大pitch角度指令（度）
static const float ROLL_CMD_MAX_DEG      = 40.0f;  // 搖桿最大roll角度指令（度）
static const float YAW_CMD_MAX_DPS       = 50.0f; // 搖桿最大yaw角速度指令（度/秒）

static const float MAX_PITCH_RATE_DPS    = 200.0f; // pitch目標角速度上限（deg/s）
static const float MAX_ROLL_RATE_DPS     = 200.0f; // roll目標角速度上限（deg/s）

// 方向符號（方向相反就改 -1）  //這邊正負號可以改變搖桿的方向
static const float SIGN_PITCH = -1.0f;  //Pitch方向
static const float SIGN_ROLL  = -1.0f;  //Roll方向
static const float SIGN_YAW  = -1.0f;


// =======================================================
// ------------------控制器參數------------------------
// =======================================================

// ------- Pitch 外迴路（角度誤差 → 角速度命令）-------
static const float Kp_pitch_ang = 3.0f;    // pitch角度P增益
static const float Ki_pitch_ang = 0.0f;  // pitch角度I增益
static const float Kd_pitch_ang = 0.008f;  // pitch角度D增益
// 外迴路積分限幅：防止 wind-up
static const float ANG_INT_LIM_PITCH = 50.0f; 
//---------------------------------------------------

// --- Pitch 內迴路（角速度誤差 → 伺服角度輸出）---
static const float Kp_pitch_rate = 0.25f;  // pitch rate P
static const float Ki_pitch_rate = 0.0f;  // pitch rate I
static const float Kd_pitch_rate = 0.000125f;   // pitch rate D
static const float DTERM_CUTOFF_HZ = 10.0f; //濾波截止頻率（Hz）
extern float gain;

// --- Roll 外迴路（角度誤差 → 角速度命令）---
static const float Kp_roll_ang = 3.0f;     //roll角度P
static const float Ki_roll_ang = 0.005f;     //roll外迴路I
static const float Kd_roll_ang = 0.002f;     //roll外迴路D
//外迴路積分限幅：防止 wind-up
static const float ANG_INT_LIM_ROLL = 50.0f;

// ---Roll內迴路（角速度誤差 → 左右油門差動量）---
static const float Kp_roll_rate = 0.0010f;       //roll rate P（輸出油門差動較小）
static const float Ki_roll_rate = 0.0f;       //roll rate I
static const float Kd_roll_rate = 0.0f;          //roll rate D
static const float ROLL_DIFF_MAX = 0.15f;        //最大差動油門幅度

extern float pitch_angle_error_previous; //用於紀錄上一次迴圈之誤差值
extern float roll_angle_error_previous; //用於紀錄上一次迴圈之誤差值

// ---Yaw內迴路（角速度誤差 → 左右servo差動量）---
static const float Kp_yaw_rate = 1.0f;        //yaw rate P
static const float Ki_yaw_rate = 0.001f;        //yaw rate I
static const float Kd_yaw_rate = 0.0004f;           //yaw rate D
static const float YAW_DIFF_MAX = 25.0f;         //最大差動servo幅度

// =======================================================

// 姿態估測相關參數
extern float pitch_deg, roll_deg, yaw_deg;
extern float pitch_offset, roll_offset, yaw_offset;
extern bool  calibPrevActive;
extern float gx_bias, gy_bias, gz_bias;

//內迴路積分狀態紀錄
extern float Pitch_rateInt;
extern float Roll_rateInt;
extern float Yaw_rateInt;

//外迴路積分狀態紀錄
extern float Pitch_angInt;
extern float Roll_angInt;

// 內迴路D狀態
extern float Pitch_rateErrPrev; 
extern float Roll_rateErrPrev;
extern float Yaw_rateErrPrev;
extern float Pitch_rate_dterm_filt; 
extern float Roll_rate_dterm_filt;
extern float Yaw_rate_dterm_filt;
//外迴路D狀態
extern float Pitch_attitude_dterm_filt; 
extern float Roll_attitude_dterm_filt;

// =======================================================
// 陀螺儀校正相關參數
// =======================================================
static const uint16_t gyroCalibSamples = 300; //陀螺儀bias校正取樣次數（靜止平均）

// =======================================================
// 全域變數宣告
// SBUS
extern HardwareSerial &sbusSerial;  // SBUS
extern uint8_t  sbusBuf[SBUS_FRAME_SIZE];
extern uint16_t sbusCh[16];
extern bool     haveFrame;
extern uint32_t lastSbusMs;

extern Madgwick filter;
extern Madgwick filterYaw;
extern Adafruit_PWMServoDriver pwm;
extern ICM_20948_I2C myICM;
extern Adafruit_NeoPixel pixel;
extern HardwareSerial &sbusSerial;

// 姿態角（估測）
extern float pitch_deg, roll_deg, yaw_deg;

// 零點 offset（按 CH_CALIB 校正）
extern float pitch_offset, roll_offset, yaw_offset;
extern bool  calibPrevActive;

// 陀螺儀 bias
extern float gx_bias, gy_bias, gz_bias;

// 上一次迴圈時間紀錄
extern int lastLoopUs;

//搖桿值
extern float pitchStick;
extern float rollStick;
extern float yawStick;

// ==================死區設定====================
static const float STICK_CENTER_DB = 0.05f;   // 搖桿死區範圍
static const float RATE_CMD_DB_DPS = 5.0f;    // 角速度指令死區範圍（deg/s）
static const float GYRO_DB_DPS     = 5.0f;     // 陀螺儀死區範圍（deg/s）

static const float gx_cut_off = 15.0f;
static const float gy_cut_off = 15.0f;
static const float gz_cut_off = 15.0f;


//400Hz衰減係數
static const float I_DECAY= 0.98f;    //衰減參數


//---------------------磁力校正參數-----------------------
struct MagCalibration {
    float bias[3];      // 硬鐵干擾項
    float scale[3];     // 軟鐵干擾項
};

extern MagCalibration globalMagCalib;

// 函式宣告
void runMagCalibration(ICM_20948_I2C &myICM);
void applyMagCalibration(float rawX, float rawY, float rawZ, float &calX, float &calY, float &calZ);

//mavlink
void mavlink_init();

void mavlink_send_heartbeat();

// 傳送姿態（單位：rad）
void mavlink_send_attitude(float roll_rad, float pitch_rad, float yaw_rad);

// 傳送伺服角度deg
void mavlink_send_servo_angles(float servo1_deg, float servo2_deg);

// 需要時可呼叫（未來用）
void mavlink_poll_rx();

#endif