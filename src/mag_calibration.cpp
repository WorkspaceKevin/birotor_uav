#include "par_pid.h"

MagCalibration globalMagCalib = {{0.0f, 0.0f, 0.0f}, {1.0f, 1.0f, 1.0f}};
Adafruit_NeoPixel pixel(1, START_PIN, NEO_GRB + NEO_KHZ800);

void runMagCalibration(ICM_20948_I2C &sensor) {
    // --- 初始化 ---
    float magMin[3] = {32767, 32767, 32767};  //先設定一極大值，之後迭代更新
    float magMax[3] = {-32767, -32767, -32767};
    
    Serial.println("請開始旋轉機身進行 8 字校正...");

    uint32_t startTime = millis();
    uint32_t lastPrint = 0;
    uint32_t lastFlash = 0;
    bool ledStatus = false;

    // 進行20秒採樣
    while (millis() - startTime < 20000) {
        // --- RGB 閃爍邏輯 (紅色) ---
        if (millis() - lastFlash > 200) {
            ledStatus = !ledStatus;
            if (ledStatus) {
                pixel.setPixelColor(0, pixel.Color(255, 0, 0)); // 紅色亮
            } else {
                pixel.setPixelColor(0, pixel.Color(0, 0, 0));   // 熄滅
            }
            pixel.show();
            lastFlash = millis();
        }

        // --- 讀取數據 ---
        if (sensor.dataReady()) { 
            sensor.getAGMT();   // 取得最新數據
            float m[3] = {sensor.magX(), sensor.magY(), sensor.magZ()};
            
            for (int i = 0; i < 3; i++) {
                if (m[i] < magMin[i]) magMin[i] = m[i]; // 更新極值(若有新極值)
                if (m[i] > magMax[i]) magMax[i] = m[i]; // 更新極值(若有新極值)
            }
        }

        if (millis() - lastPrint > 1000) {
            Serial.printf("剩餘秒數: %lu\n", (20000 - (millis() - startTime)) / 1000);
            lastPrint = millis();
        }
    }

    // --- 計算校正參數 ---
    // 1. Hard-iron (偏移)
    for (int i = 0; i < 3; i++) {  //算出被干擾的中心點，之後每次計算時都要減去
        globalMagCalib.bias[i] = (magMax[i] + magMin[i]) / 2.0f;
    }

    // 2. Soft-iron (比例)
    float avgDelta[3];
    for (int i = 0; i < 3; i++) {
        avgDelta[i] = (magMax[i] - magMin[i]) / 2.0f;  // 算出各軸的半徑
    }
    float avgRadius = (avgDelta[0] + avgDelta[1] + avgDelta[2]) / 3.0f;   // 三軸半徑平均值 作為「標準圓球」的理想半徑
    
    for (int i = 0; i < 3; i++) {
        if (avgDelta[i] != 0) {
            globalMagCalib.scale[i] = avgRadius / avgDelta[i];  // 計算比例因子 放大或縮小到平均半徑
        } else {
            globalMagCalib.scale[i] = 1.0f;
        }
    }

    // --- 校正結束燈號 (綠色持續 2 秒) ---
    pixel.setPixelColor(0, pixel.Color(0, 255, 0)); 
    pixel.show();
    
    Serial.println("校正結果已更新。");
    Serial.printf("Bias: [%.2f, %.2f, %.2f]\n", globalMagCalib.bias[0], globalMagCalib.bias[1], globalMagCalib.bias[2]);
    Serial.printf("Scale: [%.2f, %.2f, %.2f]\n", globalMagCalib.scale[0], globalMagCalib.scale[1], globalMagCalib.scale[2]);
    
    delay(2000);
    pixel.setPixelColor(0, pixel.Color(0, 0, 0)); // 關燈
    pixel.show();
}

void applyMagCalibration(float rawX, float rawY, float rawZ, float &calX, float &calY, float &calZ) {
    calX = (rawX - globalMagCalib.bias[0]) * globalMagCalib.scale[0];
    calY = (rawY - globalMagCalib.bias[1]) * globalMagCalib.scale[1];
    calZ = (rawZ - globalMagCalib.bias[2]) * globalMagCalib.scale[2];
}