#include <Wire.h>
#include <Arduino.h>
#include "OV7670_Arduino.h"

// 定義圖像參數 - 降低分辨率以節省內存
#define FRAME_WIDTH     160  // 降低為原來的一半
#define FRAME_HEIGHT    120  // 降低為原來的一半
#define BYTES_PER_PIXEL 2    // RGB565 格式，每像素2字節
#define BUFFER_SIZE     (FRAME_WIDTH * FRAME_HEIGHT * BYTES_PER_PIXEL)

// 基本引腳定義
#define SCCB_SDA_PIN    21    // SIOD
#define SCCB_SCL_PIN    22    // SIOC
#define OV7670_RESET    12    // RESET
#define OV7670_PWDN     13    // PWDN
#define OV7670_XCLK     15    // XCLK (MCLK輸入)
#define OV7670_VSYNC    27    // VSYNC 垂直同步
#define OV7670_HREF     26    // HREF 水平參考
#define OV7670_PCLK     25    // PCLK 像素時鐘
#define OV7670_D0       32    // 數據位 0
#define OV7670_D1       33    // 數據位 1
#define OV7670_D2       34    // 數據位 2
#define OV7670_D3       35    // 數據位 3
#define OV7670_D4       36    // 數據位 4
#define OV7670_D5       39    // 數據位 5
#define OV7670_D6       18    // 數據位 6
#define OV7670_D7       19    // 數據位 7

// 創建OV7670對象
OV7670_Arduino camera;

// 使用PSRAM（如果可用）或靜態分配緩衝區
#ifdef BOARD_HAS_PSRAM
  uint8_t* frameBuffer = NULL;
  bool psramAvailable = true;
#else
  // 使用靜態分配以避免在堆棧上分配大塊內存
  static uint8_t frameBuffer[BUFFER_SIZE];
  bool psramAvailable = false;
#endif

frame_buffer_t fb = {
    .buffer = NULL,  // 將在setup中初始化
    .length = BUFFER_SIZE,
    .is_ready = false
};

void setup() {
    Serial.begin(115200);
    while(!Serial) delay(10);
    
    Serial.println("\nOV7670 圖像捕獲測試開始...");
    
    // 初始化緩衝區
    #ifdef BOARD_HAS_PSRAM
      if (psramInit()) {
        frameBuffer = (uint8_t*)ps_malloc(BUFFER_SIZE);
        if (frameBuffer == NULL) {
          Serial.println("PSRAM分配失敗，使用靜態緩衝區");
          psramAvailable = false;
        } else {
          Serial.println("使用PSRAM分配緩衝區");
        }
      } else {
        Serial.println("PSRAM不可用，使用靜態緩衝區");
        psramAvailable = false;
      }
    #else
      Serial.println("使用靜態緩衝區");
    #endif
    
    fb.buffer = frameBuffer;
    
    // 設置PWDN引腳
    pinMode(OV7670_PWDN, OUTPUT);
    digitalWrite(OV7670_PWDN, LOW);  // 啟動相機
    
    // 配置相機
    if (!setupCamera()) {
        Serial.println("相機初始化失敗！");
        while(1) delay(1000);  // 停止執行
    }
    
    Serial.println("初始化完成，準備捕獲圖像");
}

void loop() {
    // 每5秒嘗試捕獲一幀圖像
    captureAndDisplayFrame();
    delay(5000);
}

bool setupCamera() {
    // 配置相機引腳
    camera_config_t config;
    config.pin_reset = (gpio_num_t)OV7670_RESET;
    config.pin_xclk = (gpio_num_t)OV7670_XCLK;
    config.pin_sscb_sda = (gpio_num_t)SCCB_SDA_PIN;
    config.pin_sscb_scl = (gpio_num_t)SCCB_SCL_PIN;
    config.pin_d0 = (gpio_num_t)OV7670_D0;
    config.pin_d1 = (gpio_num_t)OV7670_D1;
    config.pin_d2 = (gpio_num_t)OV7670_D2;
    config.pin_d3 = (gpio_num_t)OV7670_D3;
    config.pin_d4 = (gpio_num_t)OV7670_D4;
    config.pin_d5 = (gpio_num_t)OV7670_D5;
    config.pin_d6 = (gpio_num_t)OV7670_D6;
    config.pin_d7 = (gpio_num_t)OV7670_D7;
    config.pin_vsync = (gpio_num_t)OV7670_VSYNC;
    config.pin_href = (gpio_num_t)OV7670_HREF;
    config.pin_pclk = (gpio_num_t)OV7670_PCLK;
    
    // 配置時鐘
    config.xclk_freq_hz = 20000000;  // 20MHz
    config.ledc_timer = LEDC_TIMER_0;
    config.ledc_channel = LEDC_CHANNEL_0;
    
    // 初始化相機
    if (!camera.init(config)) {
        return false;
    }
    
    // 設置測試模式以便於測試
    camera.setTestPattern(OV7670_TESTPATTERN_BAR_8);
    
    // 輸出相機配置信息
    camera.dump();
    
    return true;
}

void captureAndDisplayFrame() {
    Serial.println("開始捕獲一幀圖像...");
    
    // 重置緩衝區狀態
    fb.is_ready = false;
    
    // 捕獲一幀圖像
    if (!camera.captureFrame(&fb)) {
        Serial.println("圖像捕獲失敗");
        return;
    }
    
    if (fb.is_ready) {
        Serial.println("圖像捕獲成功");
        
        // 顯示前幾個像素的值（用於調試）
        Serial.println("前10個像素值：");
        for (int i = 0; i < 20 && i < BUFFER_SIZE; i += 2) {
            uint16_t pixelValue = (fb.buffer[i] << 8) | fb.buffer[i + 1];
            Serial.printf("像素 %d: 0x%04X (R:%d, G:%d, B:%d)\n", 
                i/2, 
                pixelValue,
                (pixelValue >> 11) & 0x1F,        // 5位紅色
                (pixelValue >> 5) & 0x3F,         // 6位綠色
                pixelValue & 0x1F                 // 5位藍色
            );
        }
    } else {
        Serial.println("圖像緩衝區未就緒");
    }
}

// 將RGB565像素轉換為灰度值（用於後續處理）
uint8_t rgb565ToGray(uint16_t pixel) {
    uint8_t r = (pixel >> 11) & 0x1F;
    uint8_t g = (pixel >> 5) & 0x3F;
    uint8_t b = pixel & 0x1F;
    
    // 將5位和6位值擴展到8位
    r = (r << 3) | (r >> 2);
    g = (g << 2) | (g >> 4);
    b = (b << 3) | (b >> 2);
    
    // 計算灰度值 (0.3R + 0.59G + 0.11B)
    return (uint8_t)(0.3 * r + 0.59 * g + 0.11 * b);
}