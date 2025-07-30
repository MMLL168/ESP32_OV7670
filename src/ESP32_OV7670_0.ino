#include <Wire.h>
#include <Arduino.h>
#include "driver/ledc.h"  // 添加這行來直接使用ESP32的LEDC驅動

// 添加在程序開頭的定義部分
#define OV7670_REG_GAIN    (0x00)
#define OV7670_REG_BLUE    (0x01)
#define OV7670_REG_RED     (0x02)
#define OV7670_REG_COM1    (0x04)
#define OV7670_REG_COM2    (0x09)
#define OV7670_REG_PID     (0x0A)
#define OV7670_REG_VER     (0x0B)
#define OV7670_REG_COM3    (0x0C)
#define OV7670_REG_COM4    (0x0D)
#define OV7670_REG_COM7    (0x12)
#define OV7670_REG_COM8    (0x13)
#define OV7670_REG_COM15   (0x40)
#define OV7670_REG_MIDH    (0x1C)
#define OV7670_REG_MIDL    (0x1D)


// 基本引腳定義
#define SCCB_SDA_PIN    21    // SIOD
#define SCCB_SCL_PIN    22    // SIOC
#define OV7670_RESET    12    // RESET
#define OV7670_PWDN     13    // PWDN
#define OV7670_XCLK     15    // XCLK (MCLK輸入)
#define OV7670_ADDR     0x21  // OV7670 SCCB地址

// LEDC配置
#define LEDC_TIMER          LEDC_TIMER_0
#define LEDC_MODE           LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO      OV7670_XCLK    // XCLK輸出引腳
#define LEDC_CHANNEL        LEDC_CHANNEL_0
#define LEDC_DUTY_RES      LEDC_TIMER_1_BIT    // 1位分辨率
#define LEDC_FREQUENCY      (12000000)          // 24MHz
#define LEDC_DUTY          (1)                  // 50% 佔空比

void setupXCLK() {
    Serial.println("設置XCLK (24MHz)...");

    // 準備LEDC定時器配置
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .duty_resolution  = LEDC_DUTY_RES,
        .timer_num       = LEDC_TIMER,
        .freq_hz         = LEDC_FREQUENCY,
        .clk_cfg         = LEDC_AUTO_CLK
    };
    
    // 設置LEDC定時器
    esp_err_t ret = ledc_timer_config(&ledc_timer);
    if (ret != ESP_OK) {
        Serial.println("LEDC定時器配置失敗");
        return;
    }

    // 準備LEDC通道配置
    ledc_channel_config_t ledc_channel = {
        .gpio_num       = LEDC_OUTPUT_IO,
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = LEDC_TIMER,
        .duty           = LEDC_DUTY,
        .hpoint        = 0
    };
    
    // 設置LEDC通道
    ret = ledc_channel_config(&ledc_channel);
    if (ret != ESP_OK) {
        Serial.println("LEDC通道配置失敗");
        return;
    }

    Serial.println("XCLK設置完成");
}

void setup() {
    Serial.begin(115200);
    while(!Serial) delay(10);
    
    Serial.println("\nOV7670 測試程序開始...");
    
    // 設置控制引腳
    pinMode(OV7670_RESET, OUTPUT);
    pinMode(OV7670_PWDN, OUTPUT);
    
    // 硬件重置
    hardReset();
    
    // 初始化I2C
    Wire.begin(SCCB_SDA_PIN, SCCB_SCL_PIN);
    Wire.setClock(100000);  // 100kHz
    
    delay(100);
    
    // 測試通訊
    testCommunication();
    
    // 如果通訊成功，進行寄存器配置
    initCameraRegisters();
    
    // 設置RGB格式
    setRGBFormat();
    
    Serial.println("初始化完成");
}


void loop() {
  delay(5000);
  testCommunication();
}

void hardReset() {
    Serial.println("執行硬體重置序列...");
    
    // 步驟1: 確保PWDN為高（斷電狀態）
    digitalWrite(OV7670_PWDN, HIGH);
    digitalWrite(OV7670_RESET, LOW);
    delay(100);
    
    // 步驟2: 啟動XCLK
    setupXCLK();
    delay(100);
    
    // 步驟3: 釋放PWDN
    digitalWrite(OV7670_PWDN, LOW);
    delay(100);
    
    // 步驟4: 釋放RESET
    digitalWrite(OV7670_RESET, HIGH);
    delay(100);
    
    Serial.println("硬體重置完成");
}

void testCommunication() {
    Serial.println("\n開始通訊測試...");
    
    // 讀取製造商ID
    uint8_t midh = SCCB_Read(OV7670_REG_MIDH);
    uint8_t midl = SCCB_Read(OV7670_REG_MIDL);
    
    // 讀取產品ID和版本
    uint8_t pid = SCCB_Read(OV7670_REG_PID);
    uint8_t ver = SCCB_Read(OV7670_REG_VER);
    
    Serial.printf("製造商 ID: 0x%02X%02X\n", midh, midl);
    Serial.printf("產品 ID: 0x%02X\n", pid);
    Serial.printf("版本號: 0x%02X\n", ver);
    
    if (midh == 0x7F && midl == 0xA2 && pid == 0x76) {
        Serial.println("成功檢測到OV7670！");
    } else {
        Serial.println("檢測失敗");
        Serial.println("預期值: MIDH=0x7F, MIDL=0xA2, PID=0x76");
    }
}

void initCameraRegisters() {
    Serial.println("配置相機寄存器...");
    
    // 重置所有寄存器
    SCCB_Write(OV7670_REG_COM7, 0x80);
    delay(100);
    
    // 基本設置
    SCCB_Write(OV7670_REG_COM7, 0x14);  // Output format - QVGA RGB
    SCCB_Write(OV7670_REG_COM3, 0x04);  // Enable scaling
    SCCB_Write(OV7670_REG_COM4, 0x40);  // Standard RGB output
    SCCB_Write(OV7670_REG_COM15, 0xD0); // RGB 565 output
    
    // Clock設置
    SCCB_Write(OV7670_REG_COM8, 0x8F);  // Enable fast AGC/AEC
    SCCB_Write(OV7670_REG_COM2, 0x03);  // Drive capability - 3x
    
    delay(100);
    
    Serial.println("寄存器配置完成");
}

// 位操作輔助函數
static bool getBit(uint8_t value, uint8_t bitNum) {
    return (value & (1 << bitNum)) != 0;
}

static uint8_t setBit(uint8_t value, uint8_t bitNum, bool bitValue) {
    value = value & (~(1 << bitNum));
    value = value | (bitValue << bitNum);
    return value;
}

// 設置RGB格式
void setRGBFormat() {
    uint8_t com7 = SCCB_Read(OV7670_REG_COM7);
    com7 = setBit(com7, 2, true);  // Enable RGB
    SCCB_Write(OV7670_REG_COM7, com7);
    
    uint8_t com15 = SCCB_Read(OV7670_REG_COM15);
    com15 = setBit(com15, 5, true);   // RGB565
    com15 = setBit(com15, 4, true);   // RGB565
    SCCB_Write(OV7670_REG_COM15, com15);
}


void softReset() {
    Serial.println("執行軟件重置...");
    SCCB_Write(0x12, 0x80);  // COM7 寄存器，寫入0x80執行軟件重置
    delay(100);
    Serial.println("軟件重置完成");
}

bool SCCB_Write(uint8_t reg, uint8_t data) {
    Wire.beginTransmission(OV7670_ADDR);
    Wire.write(reg);
    Wire.write(data);
    if (Wire.endTransmission() != 0) {
        Serial.printf("寫入寄存器 0x%02X 失敗\n", reg);
        return false;
    }
    delay(10);  // 增加延遲
    return true;
}

uint8_t SCCB_Read(uint8_t reg) {
    uint8_t data = 0;
    
    // 第一階段：寫入要讀取的寄存器地址
    Wire.beginTransmission(OV7670_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission() != 0) {
        Serial.printf("設置讀取寄存器 0x%02X 失敗\n", reg);
        return 0;
    }
    delay(10);  // 增加延遲
    
    // 第二階段：讀取數據
    if (Wire.requestFrom(OV7670_ADDR, 1) != 1) {
        Serial.printf("讀取寄存器 0x%02X 失敗\n", reg);
        return 0;
    }
    
    if (Wire.available()) {
        data = Wire.read();
    }
    delay(10);  // 增加延遲
    return data;
}

