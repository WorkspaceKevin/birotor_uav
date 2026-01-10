// =======================================================
// 1) 腳位 / 匯流排設定
// =======================================================
#define SDA_PIN 21              // I2C SDA 腳位
#define SCL_PIN 47              // I2C SCL 腳位

// =======================================================
// 2) PCA9685（PWM 擴展板）設定
// =======================================================
#define PCA_ADDR 0x40           // PCA9685 I2C 位址
#define SERVO_FREQ 50           // PCA9685 PWM 頻率（伺服/ESC 常用 50Hz）

// PCA9685 通道分配
#define PCA_ESC_CH1    0        // ESC 1（左馬達）
#define PCA_ESC_CH2    1        // ESC 2（右馬達）
#define PCA_SERVO_CH1  2        // Servo 1（左伺服）
#define PCA_SERVO_CH2  3        // Servo 2（右伺服）

// PCA tick 範圍（你目前用來 map 的參數）
#define SERVO_MIN_TICK 250     // 伺服輸出最小 tick（對應最小角）
#define SERVO_MAX_TICK 370     // 伺服輸出最大 tick（對應最大角）
#define STICK_MIN_50HZ 102      // 你定義的 50Hz 下搖桿最小 tick
#define STICK_MAX_50HZ 512      // 你定義的 50Hz 下搖桿最大 tick

// =======================================================
// 3) 伺服機械角度限制（物理結構安全範圍）
// =======================================================
#define SERVO_MIN_ANG 60        // 伺服最小角度
#define SERVO_MAX_ANG 120      // 伺服最大角度
#define SERVO_CENTER  90        // 伺服中立角度
#define SERVO_PITCH_RANGE 30    // 伺服最大輸出範圍（±30 度）

// =======================================================
// 4) IMU（ICM-20948）設定
// =======================================================
#define ICM_ADDRESS 0x69        // ICM20948 位址（依 AD0 可能是 0x68 或 0x69）
static const float COMPLEMENTARY_ALPHA = 0.98f;   // 互補濾波：陀螺權重（越大越信任 gyro）

// =======================================================
// 5) 控制迴圈頻率
// =======================================================
static const float LOOP_HZ = 3000.0f;              // 控制迴圈頻率（Hz）
static const int   LOOP_DT_US = (int)(1000000.0f / LOOP_HZ);  // 每次 loop 的週期（微秒）

// =======================================================
// 6) SBUS 接收設定
// =======================================================
#define SBUS_FRAME_SIZE 25       // SBUS frame 長度固定 25 bytes
static const int SBUS_RX_PIN = 10;     // SBUS RX 腳位
static const int SBUS_BAUD   = 100000; // SBUS baudrate
static const int FAILSAFE_TIMEOUT_MS = 150; // 多久沒收到 frame 就視為失聯（ms）

// SBUS 原始數值範圍（用來 normalize）
static const int STICK_MIN = 206;      // SBUS 最小值
static const int STICK_MAX = 1800;     // SBUS 最大值

// SBUS 通道
static const int CH_ROLL     = 0;      // Roll（滾轉，左右傾）
static const int CH_PITCH    = 1;      // Pitch（俯仰，前後傾）
static const int CH_THROTTLE = 2;      // Throttle（油門，上下升降）
static const int CH_CALIB    = 8;      // 校正開關（IMU 零點校正）
static const int CH_ARM      = 9;      // 解鎖/上鎖開關

// CH8 零點校正觸發門檻（避免一直重複觸發）
static const int CALIB_LOW_TH  = 1000; // 小於此值：觸發一次校正
static const int CALIB_HIGH_TH = 1500; // 大於此值：允許下一次再觸發

// =======================================================
// 7) 遙控指令限制（Stick → 角度/角速度）
// =======================================================
static const float PITCH_CMD_MAX_DEG     = 20.0f;  // 搖桿最大 pitch 角度指令（度）
static const float ROLL_CMD_MAX_DEG      = 20.0f;  // 搖桿最大 roll 角度指令（度）
static const float MAX_PITCH_RATE_DPS    = 200.0f; // pitch 目標角速度上限（deg/s）
static const float MAX_ROLL_RATE_DPS     = 200.0f; // roll 目標角速度上限（deg/s）

// 方向符號（如果發現方向相反就改 -1）
static const float SIGN_PITCH = +1.0f;  // Pitch 方向（正負）
static const float SIGN_ROLL  = +1.0f;  // Roll 方向（正負）

// =======================================================
// 8) 控制器參數（外迴路：角度→角速度；內迴路：角速度→致動輸出）
// =======================================================

// --- Pitch 外迴路（角度誤差 → 角速度命令）---
static const float Kp_pitch_ang = 5.0f;    // pitch 角度 P 增益（deg/s per deg）
static const float Ki_pitch_ang = 0.0f;  // 先設0
static const float Kd_pitch_ang = 0.0f;  // 先設0

// [ADD] 外迴路（角度 loop）積分限幅：防止 wind-up（單位約 deg*s）
static const float ANG_INT_LIM_PITCH = 30.0f; // [ADD] pitch 外迴路 I 最大累積

// --- Pitch 內迴路（角速度誤差 → 伺服角度輸出）---
static const float Kp_pitch_rate = 0.12f;  // pitch rate P
static const float Ki_pitch_rate = 0.04f;  // pitch rate I

// [ADD] 內迴路（rate loop）D term：先設 0 => 行為仍是 PI
static const float Kd_pitch_rate = 0.0f;   // [ADD] pitch rate D（先設0）
static const float DTERM_CUTOFF_HZ = 30.0f; // [ADD] D 濾波截止頻率（Hz）建議 20~50

// --- Roll 外迴路（角度誤差 → 角速度命令）---
static const float Kp_roll_ang = 5.0f;     // roll 角度 P 增益

// [ADD] Roll 外迴路 PID（先設0 => 仍是 P）
static const float Ki_roll_ang = 0.0f;     // [ADD] roll 外迴路 I（先設0）
static const float Kd_roll_ang = 0.0f;     // [ADD] roll 外迴路 D（先設0）

// [ADD] 外迴路（角度 loop）積分限幅：防止 wind-up
static const float ANG_INT_LIM_ROLL = 30.0f; // [ADD] roll 外迴路 I 最大累積

// --- Roll 內迴路（角速度誤差 → 左右油門差動量）---
static const float Kp_roll_rate = 0.0020f; // roll rate P（輸出是油門差動，所以要小）
static const float Ki_roll_rate = 0.0008f; // roll rate I
static const float ROLL_DIFF_MAX = 0.20f;  // 最大差動油門幅度（±0.2）

// [ADD] 內迴路（rate loop）D term：先設 0 => 行為仍是 PI
static const float Kd_roll_rate = 0.0f;    // [ADD] roll rate D（先設0）

extern float pitch_angle_error_previous; //用於紀錄上一次迴圈之誤差值
extern float roll_angle_error_previous; //用於紀錄上一次迴圈之誤差值


// =======================================================
// 9) 校正相關參數
// =======================================================
static const uint16_t gyroCalibSamples = 300; // 陀螺儀 bias 校正取樣次數（靜止平均）

// =======================================================
// 10) 全域狀態（如果你要留在 .ino 也可以）
// =======================================================

// SBUS
extern HardwareSerial &sbusSerial;  // SBUS
extern uint8_t  sbusBuf[SBUS_FRAME_SIZE];
extern uint16_t sbusCh[16];
extern bool     haveFrame;
extern uint32_t lastSbusMs;

// 姿態角（估測）
extern float pitch_deg, roll_deg, yaw_deg;

// 零點 offset（按 CH_CALIB 校正）
extern float pitch_offset, roll_offset;
extern bool  calibPrevActive;

// Gyro bias
extern float gx_bias, gy_bias, gz_bias;

// PI 狀態 積分記憶
extern float Pitch_rateInt;
extern float Roll_rateInt;

// [ADD] 外迴路（角度 loop）I 狀態（Ki=0 時不影響）
extern float Pitch_angInt;   // [ADD]
extern float Roll_angInt;    // [ADD]

// [ADD] 內迴路（rate loop）D 狀態（Kd=0 時不影響）
extern float Pitch_rateErrPrev; // [ADD]
extern float Roll_rateErrPrev;  // [ADD]
extern float Pitch_dterm_filt;  // [ADD]
extern float Roll_dterm_filt;   // [ADD]

// loop timing
extern int lastLoopUs;
