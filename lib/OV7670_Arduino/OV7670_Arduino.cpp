/*
 * OV7670_Arduino.cpp - 簡化版的OV7670庫，適用於Arduino環境
 * 基於原始OV7670庫修改
 */

#include "OV7670_Arduino.h"

static const char* TAG = "OV7670_Arduino";

// 構造函數
OV7670_Arduino::OV7670_Arduino() {
    m_initialized = false;
}

// 析構函數
OV7670_Arduino::~OV7670_Arduino() {
    // 釋放資源
}

// 位操作：獲取位值
bool OV7670_Arduino::getBit(uint8_t value, uint8_t bitNum) {
    return (value & (1 << bitNum)) != 0;
}

// 位操作：設置位值
uint8_t OV7670_Arduino::setBit(uint8_t value, uint8_t bitNum, bool bitValue) {
    value = value & (~(1 << bitNum));
    value = value | (bitValue << bitNum);
    return value;
}

// 初始化相機
bool OV7670_Arduino::init(camera_config_t config) {
    Serial.println("初始化OV7670相機...");
    
    m_config = config;
    
    // 設置引腳模式
    pinMode(m_config.pin_reset, OUTPUT);
    
    // 硬件重置相機
    reset();
    
    // 設置時鐘
    esp_err_t ret = setupClock();
    if (ret != ESP_OK) {
        Serial.println("設置時鐘失敗");
        return false;
    }
    
    // 初始化I2C
    Wire.begin(m_config.pin_sscb_sda, m_config.pin_sscb_scl);
    Wire.setClock(100000); // 100kHz
    
    delay(100);
    
    // 檢查相機ID
    uint8_t pid = readRegister(OV7670_REG_PID);
    uint8_t ver = readRegister(OV7670_REG_VER);
    
    if (pid != 0x76 || ver != 0x73) {
        Serial.printf("相機ID不匹配: PID=0x%02X, VER=0x%02X\n", pid, ver);
        Serial.println("預期值: PID=0x76, VER=0x73");
        return false;
    }
    
    Serial.println("相機ID驗證成功");
    
    // 軟件重置相機
    resetCamera();
    delay(100);
    
    // 設置默認配置
    setFormat(OV7670_FORMAT_RGB);
    setRGBFormat(OV7670_FORMAT_RGB_RGB_565);
    
    // 設置I2S接口
    ret = setupI2S();
    if (ret != ESP_OK) {
        Serial.println("設置I2S接口失敗");
        return false;
    }
    
    m_initialized = true;
    Serial.println("OV7670相機初始化完成");
    return true;
}

// 設置時鐘
esp_err_t OV7670_Arduino::setupClock() {
    Serial.printf("設置XCLK: %dHz\n", m_config.xclk_freq_hz);
    
    periph_module_enable(PERIPH_LEDC_MODULE);
    
    ledc_timer_config_t timer_conf;
    timer_conf.speed_mode = LEDC_HIGH_SPEED_MODE;
    timer_conf.duty_resolution = LEDC_TIMER_1_BIT; // 1位分辨率
    timer_conf.freq_hz = m_config.xclk_freq_hz;
    timer_conf.timer_num = m_config.ledc_timer;
    timer_conf.clk_cfg = LEDC_AUTO_CLK;
    
    esp_err_t ret = ledc_timer_config(&timer_conf);
    if (ret != ESP_OK) {
        Serial.printf("ledc_timer_config失敗: %d\n", ret);
        return ret;
    }
    
    ledc_channel_config_t ch_conf;
    ch_conf.gpio_num = m_config.pin_xclk;
    ch_conf.speed_mode = LEDC_HIGH_SPEED_MODE;
    ch_conf.channel = m_config.ledc_channel;
    ch_conf.intr_type = LEDC_INTR_DISABLE;
    ch_conf.timer_sel = m_config.ledc_timer;
    ch_conf.duty = 1; // 50%佔空比
    ch_conf.hpoint = 0;
    
    ret = ledc_channel_config(&ch_conf);
    if (ret != ESP_OK) {
        Serial.printf("ledc_channel_config失敗: %d\n", ret);
        return ret;
    }
    
    Serial.println("XCLK設置完成");
    return ESP_OK;
}

// 設置I2S接口
esp_err_t OV7670_Arduino::setupI2S() {
    // 這裡需要根據ESP32的I2S相機模式進行配置
    // 由於這部分較複雜，我們先簡化實現
    Serial.println("I2S接口設置 (簡化版)");
    return ESP_OK;
}

// 重置相機
void OV7670_Arduino::reset() {
    Serial.println("重置相機...");
    digitalWrite(m_config.pin_reset, LOW);
    delay(10);
    digitalWrite(m_config.pin_reset, HIGH);
    delay(10);
    Serial.println("相機重置完成");
}

// 軟件重置相機
void OV7670_Arduino::resetCamera() {
    Serial.println("執行軟件重置...");
    writeRegister(OV7670_REG_COM7, 0x80); // 設置COM7寄存器的最高位以重置所有寄存器
    delay(100);
    Serial.println("軟件重置完成");
}

// 設置圖像格式
void OV7670_Arduino::setFormat(uint8_t value) {
    uint8_t com7 = readRegister(OV7670_REG_COM7);
    com7 = setBit(com7, 2, getBit(value, 1));
    com7 = setBit(com7, 0, getBit(value, 0));
    writeRegister(OV7670_REG_COM7, com7);
    Serial.printf("設置圖像格式: 0x%02X\n", value);
}

// 設置RGB格式
void OV7670_Arduino::setRGBFormat(uint8_t value) {
    uint8_t com15 = readRegister(OV7670_REG_COM15);
    com15 = setBit(com15, 5, getBit(value, 1));
    com15 = setBit(com15, 4, getBit(value, 0));
    writeRegister(OV7670_REG_COM15, com15);
    Serial.printf("設置RGB格式: 0x%02X\n", value);
}

// 設置測試模式
void OV7670_Arduino::setTestPattern(uint8_t value) {
    uint8_t com7 = readRegister(OV7670_REG_COM7);
    if (value == OV7670_TESTPATTERN_NONE) {
        com7 = setBit(com7, 1, false);
    } else {
        com7 = setBit(com7, 1, true);
    }
    writeRegister(OV7670_REG_COM7, com7);
    
    uint8_t scaling_xsc = readRegister(OV7670_REG_SCALING_XSC);
    scaling_xsc = setBit(scaling_xsc, 7, getBit(value, 1));
    writeRegister(OV7670_REG_SCALING_XSC, scaling_xsc);
    
    uint8_t scaling_ysc = readRegister(OV7670_REG_SCALING_YSC);
    scaling_ysc = setBit(scaling_ysc, 7, getBit(value, 0));
    writeRegister(OV7670_REG_SCALING_YSC, scaling_ysc);
    
    Serial.printf("設置測試模式: 0x%02X\n", value);
}

// 捕獲一幀圖像
bool OV7670_Arduino::captureFrame(frame_buffer_t* fb) {
    if (!m_initialized || fb == NULL || fb->buffer == NULL) {
        Serial.println("相機未初始化或緩衝區無效");
        return false;
    }
    
    // 這裡需要實現圖像捕獲邏輯
    // 由於這部分較複雜，我們先簡化實現
    
    Serial.println("模擬捕獲一幀圖像...");
    
    // 填充測試數據
    for (size_t i = 0; i < fb->length; i++) {
        fb->buffer[i] = i % 256;
    }
    
    fb->is_ready = true;
    return true;
}

// 讀取寄存器
uint8_t OV7670_Arduino::readRegister(uint8_t reg) {
    Wire.beginTransmission(OV7670_SCCB_ADDR);
    Wire.write(reg);
    Wire.endTransmission();
    
    Wire.requestFrom(OV7670_SCCB_ADDR, 1);
    if (Wire.available()) {
        return Wire.read();
    }
    return 0;
}

// 寫入寄存器
void OV7670_Arduino::writeRegister(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(OV7670_SCCB_ADDR);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
    delay(10); // 給相機一些時間處理
}

// 輸出相機配置信息
void OV7670_Arduino::dump() {
    Serial.println("\n相機配置信息:");
    
    uint8_t pid = readRegister(OV7670_REG_PID);
    uint8_t ver = readRegister(OV7670_REG_VER);
    uint8_t midh = readRegister(OV7670_REG_MIDH);
    uint8_t midl = readRegister(OV7670_REG_MIDL);
    
    Serial.printf("PID: 0x%02X, VER: 0x%02X, MID: 0x%02X%02X\n", pid, ver, midh, midl);
    
    uint8_t com7 = readRegister(OV7670_REG_COM7);
    uint32_t outputFormat = getBit(com7, 2) << 1 | getBit(com7, 0);
    
    Serial.print("輸出格式: ");
    switch (outputFormat) {
        case 0b00:
            Serial.println("YUV");
            break;
        case 0b10:
            Serial.println("RGB");
            break;
        case 0b01:
            Serial.println("Raw Bayer RGB");
            break;
        case 0b11:
            Serial.println("Process Bayer RGB");
            break;
        default:
            Serial.println("未知");
            break;
    }
    
    if (outputFormat == 0b10) {
        uint8_t com15 = readRegister(OV7670_REG_COM15);
        uint8_t rgbType = getBit(com15, 5) << 1 | getBit(com15, 4);
        
        Serial.print("RGB類型: ");
        switch (rgbType) {
            case 0b00:
            case 0b10:
                Serial.println("Normal RGB Output");
                break;
            case 0b01:
                Serial.println("RGB 565");
                break;
            case 0b11:
                Serial.println("RGB 555");
                break;
            default:
                Serial.println("未知");
                break;
        }
    }
    
    Serial.print("顏色條: ");
    Serial.println(getBit(com7, 1) ? "啟用" : "禁用");
    
    uint8_t scaling_xsc = readRegister(OV7670_REG_SCALING_XSC);
    uint8_t scaling_ysc = readRegister(OV7670_REG_SCALING_YSC);
    uint32_t testPattern = getBit(scaling_xsc, 7) << 1 | getBit(scaling_ysc, 7);
    
    Serial.print("測試模式: ");
    switch (testPattern) {
        case 0b00:
            Serial.println("無測試輸出");
            break;
        case 0b01:
            Serial.println("Shifting 1");
            break;
        case 0b10:
            Serial.println("8-bar color bar");
            break;
        case 0b11:
            Serial.println("Fade to gray color bar");
            break;
        default:
            Serial.println("未知");
            break;
    }
    
    Serial.printf("水平縮放因子: %d\n", scaling_xsc & 0x3f);
    Serial.printf("垂直縮放因子: %d\n", scaling_ysc & 0x3f);
    
    uint8_t com15 = readRegister(OV7670_REG_COM15);
    Serial.print("輸出範圍: ");
    switch ((com15 & 0b11000000) >> 6) {
        case 0b00:
        case 0b01:
            Serial.println("0x10 to 0xf0");
            break;
        case 0b10:
            Serial.println("0x01 to 0xfe");
            break;
        case 0b11:
            Serial.println("0x00 to 0xff");
            break;
        default:
            Serial.println("未知");
            break;
    }
    
    Serial.println();
}