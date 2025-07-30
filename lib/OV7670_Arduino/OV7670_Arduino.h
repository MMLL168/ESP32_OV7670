/*
 * OV7670_Arduino.h - 簡化版的OV7670庫，適用於Arduino環境
 * 基於原始OV7670庫修改
 */

#ifndef OV7670_ARDUINO_H_
#define OV7670_ARDUINO_H_

#include <Arduino.h>
#include <Wire.h>
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/i2s.h"

// OV7670 SCCB 地址
#define OV7670_SCCB_ADDR 0x21

// OV7670 寄存器定義
#define OV7670_REG_GAIN    (0x00)
#define OV7670_REG_BLUE    (0x01)
#define OV7670_REG_RED     (0x02)
#define OV7670_REG_VREF    (0x03)
#define OV7670_REG_COM1    (0x04)
#define OV7670_REG_BAVE    (0x05)
#define OV7670_REG_GbAVE   (0x06)
#define OV7670_REG_AECHH   (0x07)
#define OV7670_REG_RAVE    (0x08)
#define OV7670_REG_COM2    (0x09)
#define OV7670_REG_PID     (0x0A)
#define OV7670_REG_VER     (0x0B)
#define OV7670_REG_COM3    (0x0C)
#define OV7670_REG_COM4    (0x0D)
#define OV7670_REG_COM5    (0x0E)
#define OV7670_REG_COM6    (0x0F)
#define OV7670_REG_AECH    (0x10)
#define OV7670_REG_CLKRC   (0x11)
#define OV7670_REG_COM7    (0x12)
#define OV7670_REG_COM8    (0x13)
#define OV7670_REG_COM9    (0x14)
#define OV7670_REG_COM10   (0x15)
#define OV7670_REG_HSTART  (0x17)
#define OV7670_REG_HSTOP   (0x18)
#define OV7670_REG_VSTRT   (0x19)
#define OV7670_REG_VSTOP   (0x1a)
#define OV7670_REG_PSHFT   (0x1b)
#define OV7670_REG_MIDH    (0x1c)
#define OV7670_REG_MIDL    (0x1d)
#define OV7670_REG_COM15   (0x40)
#define OV7670_REG_COM16   (0x41)
#define OV7670_REG_COM17   (0x42)
#define OV7670_REG_SCALING_XSC  (0x70)
#define OV7670_REG_SCALING_YSC  (0x71)
#define OV7670_REG_SCALING_DCWCTR  (0x72)
#define OV7670_REG_SCALING_PCLK_DIV (0x73)

// 圖像格式定義
#define OV7670_FORMAT_YUV (0b00)
#define OV7670_FORMAT_RGB (0b10)

#define OV7670_FORMAT_RGB_RGB_NORMAL (0b00)
#define OV7670_FORMAT_RGB_RGB_565    (0b01)
#define OV7670_FORMAT_RGB_RGB_555    (0b11)

// 測試模式定義
#define OV7670_TESTPATTERN_NONE      (0b00)
#define OV7670_TESTPATTERN_SHIFT_1   (0b01)
#define OV7670_TESTPATTERN_BAR_8     (0b10)
#define OV7670_TESTPATTERN_GRAY_FADE (0b11)

// 相機配置結構
typedef struct {
    gpio_num_t pin_reset;          // GPIO pin for camera reset line
    gpio_num_t pin_xclk;           // GPIO pin for camera XCLK line
    gpio_num_t pin_sscb_sda;       // GPIO pin for camera SDA line
    gpio_num_t pin_sscb_scl;       // GPIO pin for camera SCL line
    gpio_num_t pin_d7;             // GPIO pin for camera D7 line
    gpio_num_t pin_d6;             // GPIO pin for camera D6 line
    gpio_num_t pin_d5;             // GPIO pin for camera D5 line
    gpio_num_t pin_d4;             // GPIO pin for camera D4 line
    gpio_num_t pin_d3;             // GPIO pin for camera D3 line
    gpio_num_t pin_d2;             // GPIO pin for camera D2 line
    gpio_num_t pin_d1;             // GPIO pin for camera D1 line
    gpio_num_t pin_d0;             // GPIO pin for camera D0 line
    gpio_num_t pin_vsync;          // GPIO pin for camera VSYNC line
    gpio_num_t pin_href;           // GPIO pin for camera HREF line
    gpio_num_t pin_pclk;           // GPIO pin for camera PCLK line

    int xclk_freq_hz;              // Frequency of XCLK signal
    ledc_timer_t ledc_timer;       // LEDC timer to be used for generating XCLK
    ledc_channel_t ledc_channel;   // LEDC channel to be used for generating XCLK
} camera_config_t;

// 圖像緩衝區結構
typedef struct {
    uint8_t* buffer;
    size_t length;
    bool is_ready;
} frame_buffer_t;

class OV7670_Arduino {
public:
    OV7670_Arduino();
    ~OV7670_Arduino();
    
    // 初始化相機
    bool init(camera_config_t config);
    
    // 重置相機
    void reset();
    
    // 設置圖像格式
    void setFormat(uint8_t value);
    
    // 設置RGB格式
    void setRGBFormat(uint8_t value);
    
    // 設置測試模式
    void setTestPattern(uint8_t value);
    
    // 軟件重置相機
    void resetCamera();
    
    // 捕獲一幀圖像
    bool captureFrame(frame_buffer_t* fb);
    
    // 輸出相機配置信息
    void dump();

private:
    // 讀取寄存器
    uint8_t readRegister(uint8_t reg);
    
    // 寫入寄存器
    void writeRegister(uint8_t reg, uint8_t value);
    
    // 設置時鐘
    esp_err_t setupClock();
    
    // 設置I2S接口
    esp_err_t setupI2S();
    
    camera_config_t m_config;
    bool m_initialized;
    
    // 位操作輔助函數
    bool getBit(uint8_t value, uint8_t bitNum);
    uint8_t setBit(uint8_t value, uint8_t bitNum, bool bitValue);
};

#endif /* OV7670_ARDUINO_H_ */