#include "par_pid.h"
#include <mavlink.h>

// MAVLink identity
static const uint8_t SYSID  = 1;  //通常都是 1
static const uint8_t COMPID = MAV_COMP_ID_AUTOPILOT1;  //主控板
//----------------------------------------
static void send_msg(const mavlink_message_t &msg){  //發送 MAVLink message 到 UART
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];  //放「序列化後的 MAVLink 封包 bytes」
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);  //序列化 MAVLink message 到 buffer
  MAV_SERIAL.write(buf, len);  //發送 bytes 到 UART
}

//----------------------------------------
void mavlink_init(){
  MAV_SERIAL.begin(MAV_BAUD, SERIAL_8N1, MAV_RX_PIN, MAV_TX_PIN);  //8 位資料、無同位元、1 stop bit
}

//----------------------------------------
void mavlink_send_heartbeat(){  //傳送心跳封包
  mavlink_message_t msg;
  mavlink_msg_heartbeat_pack(
    SYSID, COMPID, &msg,  //ID
    MAV_TYPE_GENERIC,  //無人機類型：generic
    MAV_AUTOPILOT_GENERIC,  //generic autopilot
    0, 0,  // armed, 模式flag(都不宣告)
    MAV_STATE_ACTIVE  //狀態：active
  );
  send_msg(msg);
}

//----------------------------------------
void mavlink_send_attitude(float roll_rad, float pitch_rad, float yaw_rad){  //傳送姿態（單位：rad）
  mavlink_message_t msg; //mavlink容器

  mavlink_msg_attitude_pack(  //將數據打包
    SYSID, COMPID, &msg,
    millis(),
    roll_rad,
    pitch_rad,
    yaw_rad,
    0.0f, 0.0f, 0.0f
  );
  send_msg(msg);
}

static void mavlink_send_named_float(const char* name, float value) {
  mavlink_message_t msg;

  char n[10] = {0};                 // MAVLink定義name[10](最多10byte)
  strncpy(n, name, sizeof(n) - 1);  // 最多使用9個字元，保留結尾，用strncpy複製name到n陣列

  mavlink_msg_named_value_float_pack(
    SYSID, COMPID, &msg,
    millis(),   // time_boot_ms
    n,          // name[10]
    value       // value
  );

  send_msg(msg);
}

// 送伺服角度deg
void mavlink_send_servo_angles(float servo1_deg, float servo2_deg) {
  mavlink_send_named_float("servo1_deg", servo1_deg);
  mavlink_send_named_float("servo2_deg", servo2_deg);
}

