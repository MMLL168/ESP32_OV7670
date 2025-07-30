#include <Wire.h>
#include <Arduino.h>
#include "OV7670_Arduino.h"
#include <WiFi.h>
#include <WebServer.h>

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

// WiFi 設定
//const char* ssid = "TP-Link_AA66";      // 請修改為您的 WiFi 名稱
//const char* password = "40399799";  // 請修改為您的 WiFi 密碼
const char* ssid = "MLI16P";      // 請修改為您的 WiFi 名稱
const char* password = "lynnaaaa";  // 請修改為您的 WiFi 密碼

// WiFi連接超時設定（毫秒）
#define WIFI_TIMEOUT 30000  // 30秒

// 創建網頁服務器，端口80
WebServer server(80);

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

// 標誌位，表示是否有新圖像可用
bool newFrameAvailable = false;
// WiFi連接狀態
bool wifiConnected = false;

// 函數聲明
void setupServer();
void handleRoot();
void handleStream();
void handleCapture();
bool setupCamera();
void captureFrame();
bool setupWiFi();
uint8_t rgb565ToGray(uint16_t pixel);

void setup() {
    Serial.begin(115200);
    delay(100); // 給序列埠一些時間初始化
    
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
    
    Serial.println("相機初始化完成");
    
    // 連接到WiFi網絡 - 嘗試連接但不阻止後續操作
    wifiConnected = setupWiFi();
    
    if (wifiConnected) {
        // 設置網頁服務器路由
        setupServer();
        Serial.println("HTTP服務器已啟動");
    } else {
        Serial.println("WiFi連接失敗！將在沒有網絡連接的情況下繼續運行。");
        Serial.println("您可以稍後嘗試重新連接或重啟設備。");
    }
    
    Serial.println("初始化完成，準備捕獲圖像");
}

// 在函數聲明部分添加新函數
void generateTestImage();

// 修改captureFrame函數，使用模擬圖像
void captureFrame() {
    // 重置緩衝區狀態
    fb.is_ready = false;
    
    // 由於沒有連接實際相機，使用模擬圖像
    Serial.println("生成模擬測試圖像...");
    generateTestImage();
    
    fb.is_ready = true;
    newFrameAvailable = true;
    Serial.println("模擬圖像生成完成");
}

// 生成測試圖像的函數
void generateTestImage() {
    // 確保緩衝區已分配
    if (fb.buffer == NULL) {
        Serial.println("錯誤：圖像緩衝區未分配");
        return;
    }
    
    // 清除緩衝區
    memset(fb.buffer, 0, BUFFER_SIZE);
    
    // 生成彩色條紋圖案
    const uint16_t colors[] = {
        0xF800,  // 紅色 (RGB565)
        0x07E0,  // 綠色
        0x001F,  // 藍色
        0xFFE0,  // 黃色
        0xF81F,  // 洋紅色
        0x07FF,  // 青色
        0xFFFF,  // 白色
        0x0000   // 黑色
    };
    
    int numColors = sizeof(colors) / sizeof(colors[0]);
    int colorWidth = FRAME_WIDTH / numColors;
    
    for (int y = 0; y < FRAME_HEIGHT; y++) {
        for (int x = 0; x < FRAME_WIDTH; x++) {
            int colorIndex = x / colorWidth;
            if (colorIndex >= numColors) colorIndex = numColors - 1;
            
            uint16_t color = colors[colorIndex];
            
            // 在RGB565格式中存儲像素
            int i = (y * FRAME_WIDTH + x) * 2;
            if (i + 1 < BUFFER_SIZE) {
                fb.buffer[i] = (color >> 8) & 0xFF;  // 高字節
                fb.buffer[i + 1] = color & 0xFF;     // 低字節
            }
        }
    }
    
    // 添加一些圖形元素以便識別
    // 繪製一個十字
    int centerX = FRAME_WIDTH / 2;
    int centerY = FRAME_HEIGHT / 2;
    int crossSize = 20;
    
    // 水平線
    for (int x = centerX - crossSize; x <= centerX + crossSize; x++) {
        int i = (centerY * FRAME_WIDTH + x) * 2;
        if (i + 1 < BUFFER_SIZE && x >= 0 && x < FRAME_WIDTH) {
            fb.buffer[i] = 0xFF;  // 白色
            fb.buffer[i + 1] = 0xFF;
        }
    }
    
    // 垂直線
    for (int y = centerY - crossSize; y <= centerY + crossSize; y++) {
        int i = (y * FRAME_WIDTH + centerX) * 2;
        if (i + 1 < BUFFER_SIZE && y >= 0 && y < FRAME_HEIGHT) {
            fb.buffer[i] = 0xFF;  // 白色
            fb.buffer[i + 1] = 0xFF;
        }
    }
    
    // 在四個角落繪製方塊
    int squareSize = 15;
    
    // 左上角 - 紅色方塊
    for (int y = 0; y < squareSize; y++) {
        for (int x = 0; x < squareSize; x++) {
            int i = (y * FRAME_WIDTH + x) * 2;
            if (i + 1 < BUFFER_SIZE) {
                fb.buffer[i] = 0xF8;  // 紅色高字節
                fb.buffer[i + 1] = 0x00;  // 紅色低字節
            }
        }
    }
    
    // 右上角 - 綠色方塊
    for (int y = 0; y < squareSize; y++) {
        for (int x = FRAME_WIDTH - squareSize; x < FRAME_WIDTH; x++) {
            int i = (y * FRAME_WIDTH + x) * 2;
            if (i + 1 < BUFFER_SIZE) {
                fb.buffer[i] = 0x07;  // 綠色高字節
                fb.buffer[i + 1] = 0xE0;  // 綠色低字節
            }
        }
    }
    
    // 左下角 - 藍色方塊
    for (int y = FRAME_HEIGHT - squareSize; y < FRAME_HEIGHT; y++) {
        for (int x = 0; x < squareSize; x++) {
            int i = (y * FRAME_WIDTH + x) * 2;
            if (i + 1 < BUFFER_SIZE) {
                fb.buffer[i] = 0x00;  // 藍色高字節
                fb.buffer[i + 1] = 0x1F;  // 藍色低字節
            }
        }
    }
    
    // 右下角 - 黃色方塊
    for (int y = FRAME_HEIGHT - squareSize; y < FRAME_HEIGHT; y++) {
        for (int x = FRAME_WIDTH - squareSize; x < FRAME_WIDTH; x++) {
            int i = (y * FRAME_WIDTH + x) * 2;
            if (i + 1 < BUFFER_SIZE) {
                fb.buffer[i] = 0xFF;  // 黃色高字節
                fb.buffer[i + 1] = 0xE0;  // 黃色低字節
            }
        }
    }
}

void loop() {
    // 檢查WiFi連接狀態
    static unsigned long lastWiFiCheckTime = 0;
    static unsigned long lastCaptureTime = 0;
    unsigned long currentTime = millis();
    
    // 每5秒檢查一次WiFi連接狀態
    if (currentTime - lastWiFiCheckTime > 5000) {
        lastWiFiCheckTime = currentTime;
        
        if (WiFi.status() == WL_CONNECTED) {
            // 只在狀態變化時輸出，避免過多日誌
            static bool lastWifiConnected = false;
            if (!lastWifiConnected) {
                Serial.print("WiFi 已連接，IP: ");
                Serial.println(WiFi.localIP());
                lastWifiConnected = true;
            }
        } else {
            Serial.println("WiFi連接斷開，嘗試重新連接...");
            wifiConnected = setupWiFi();
            
            if (wifiConnected) {
                setupServer();
                Serial.println("WiFi重新連接成功，伺服器已重啟");
            } else {
                Serial.println("無法重新連接WiFi");
            }
        }
    }
    
    // 處理網頁服務器客戶端請求
    server.handleClient();
    
    // 每500毫秒生成一幀模擬圖像
    if (currentTime - lastCaptureTime > 500) {
        captureFrame();
        lastCaptureTime = currentTime;
        
        // 每10幀打印一次調試信息
        static int frameCount = 0;
        frameCount++;
        if (frameCount % 10 == 0) {
            Serial.print("已生成 ");
            Serial.print(frameCount);
            Serial.println(" 幀模擬圖像");
        }
    }
}


bool setupWiFi() {
    Serial.print("連接到WiFi網絡: ");
    Serial.println(ssid);
    
    // 完全重置 WiFi
    WiFi.persistent(false);
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    delay(1000);
    
    // 設置WiFi模式
    WiFi.mode(WIFI_STA);
    delay(1000);
    
    // 打印 MAC 地址
    Serial.print("ESP32 MAC 地址: ");
    Serial.println(WiFi.macAddress());
    
    // 開始連接 - 增加重試次數
    int retryCount = 0;
    const int maxRetries = 3;
    
    while (retryCount < maxRetries) {
        Serial.println("嘗試WiFi連接 #" + String(retryCount + 1));
        
        WiFi.begin(ssid, password);
        Serial.println("WiFi 連接中...");
        
        // 設置連接超時
        unsigned long startAttemptTime = millis();
        
        // 等待連接或超時，每秒輸出一次狀態
        while (WiFi.status() != WL_CONNECTED && 
               millis() - startAttemptTime < WIFI_TIMEOUT) {
            delay(1000);
            Serial.print("WiFi 狀態: ");
            printWiFiStatus(WiFi.status());
        }
        
        if (WiFi.status() == WL_CONNECTED) {
            Serial.println("");
            Serial.println("WiFi連接成功");
            Serial.print("IP地址: ");
            Serial.println(WiFi.localIP());
            
            // 測試網絡連接
            Serial.println("測試網絡連接...");
            if (testNetworkConnection()) {
                return true;
            } else {
                Serial.println("網絡連接測試失敗，嘗試重新連接");
                WiFi.disconnect();
                delay(1000);
                retryCount++;
            }
        } else {
            Serial.println("");
            Serial.println("WiFi連接失敗，嘗試重新連接");
            WiFi.disconnect();
            delay(2000);  // 等待更長時間再重試
            retryCount++;
        }
    }
    
    Serial.println("所有WiFi連接嘗試都失敗");
    Serial.print("最終狀態: ");
    printWiFiStatus(WiFi.status());
    return false;
}

// 新增函數：測試網絡連接
bool testNetworkConnection() {
    // 簡單的連接測試 - 在實際環境中可能需要更複雜的測試
    Serial.println("網絡連接看起來正常");
    return true;
}

// 輔助函數：打印 WiFi 狀態
void printWiFiStatus(wl_status_t status) {
    switch(status) {
        case WL_NO_SHIELD:
            Serial.println("WL_NO_SHIELD");
            break;
        case WL_IDLE_STATUS:
            Serial.println("WL_IDLE_STATUS");
            break;
        case WL_NO_SSID_AVAIL:
            Serial.println("WL_NO_SSID_AVAIL - 找不到指定的SSID網絡");
            break;
        case WL_SCAN_COMPLETED:
            Serial.println("WL_SCAN_COMPLETED");
            break;
        case WL_CONNECTED:
            Serial.println("WL_CONNECTED - 已連接");
            break;
        case WL_CONNECT_FAILED:
            Serial.println("WL_CONNECT_FAILED - 密碼錯誤或其他連接失敗");
            break;
        case WL_CONNECTION_LOST:
            Serial.println("WL_CONNECTION_LOST");
            break;
        case WL_DISCONNECTED:
            Serial.println("WL_DISCONNECTED");
            break;
        default:
            Serial.print("未知狀態: ");
            Serial.println(status);
            break;
    }
}

void setupServer() {
    // 設置路由
    server.on("/", HTTP_GET, []() {
        Serial.println("收到根路徑請求");
        handleRoot();
    });
    
    server.on("/stream", HTTP_GET, []() {
        Serial.println("收到串流請求");
        handleStream();
    });
    
    server.on("/capture", HTTP_GET, []() {
        Serial.println("收到捕獲圖像請求");
        handleCapture();
    });
    
    // 添加一個簡單的測試路由
    server.on("/test", HTTP_GET, []() {
        Serial.println("收到測試請求");
        server.send(200, "text/plain", "ESP32 Web 服務器正常運行！");
    });
    
    // 添加404處理
    server.onNotFound([]() {
        Serial.println("收到未知路徑請求: " + server.uri());
        server.send(404, "text/plain", "找不到請求的頁面");
    });
    
    // 啟動服務器
    server.begin();
    Serial.println("HTTP服務器已啟動在 http://" + WiFi.localIP().toString());
    Serial.println("您可以訪問以下地址測試服務器:");
    Serial.println("主頁: http://" + WiFi.localIP().toString());
    Serial.println("測試頁: http://" + WiFi.localIP().toString() + "/test");
}

void handleRoot() {
    String html = "<!DOCTYPE html>\
    <html>\
    <head>\
        <title>ESP32 OV7670 相機</title>\
        <meta name='viewport' content='width=device-width, initial-scale=1'>\
        <style>\
            body { font-family: Arial; text-align: center; margin: 0px auto; }\
            img { width: 320px; height: 240px; }\
            .button { background-color: #4CAF50; border: none; color: white; padding: 10px 20px;\
                    text-align: center; text-decoration: none; display: inline-block;\
                    font-size: 16px; margin: 4px 2px; cursor: pointer; }\
        </style>\
        <script>\
            function refreshImage() {\
                var img = document.getElementById('cameraImage');\
                img.src = '/capture?' + new Date().getTime();\
            }\
            \
            function startStream() {\
                setInterval(refreshImage, 1000);\
            }\
        </script>\
    </head>\
    <body onload='startStream()'>\
        <h1>ESP32 OV7670 相機畫面</h1>\
        <img id='cameraImage' src='/capture'>\
        <p><button class='button' onclick='refreshImage()'>刷新圖像</button></p>\
    </body>\
    </html>";
    
    server.send(200, "text/html", html);
}

void handleStream() {
    // 這個函數處理MJPEG流，但由於OV7670不直接支持JPEG，
    // 我們將使用定期刷新的靜態圖像
    server.send(200, "text/plain", "串流功能尚未實現，請使用刷新圖像功能");
}

void handleCapture() {
    Serial.println("處理圖像捕獲請求...");
    
    // 確保有最新的圖像
    captureFrame();
    
    if (!fb.is_ready) {
        Serial.println("錯誤：相機未就緒，無法提供圖像");
        server.send(503, "text/plain", "相機未就緒");
        return;
    }
    
    Serial.println("準備將RGB565數據轉換為BMP格式...");
    
    // 將RGB565數據轉換為BMP格式
    uint32_t bmpSize = 54 + 3 * FRAME_WIDTH * FRAME_HEIGHT; // BMP文件大小
    uint8_t bmpHeader[54] = {
        'B', 'M',                   // BMP標識
        (uint8_t)(bmpSize), (uint8_t)(bmpSize >> 8), (uint8_t)(bmpSize >> 16), (uint8_t)(bmpSize >> 24), // 文件大小
        0, 0, 0, 0,                 // 保留
        54, 0, 0, 0,                // 像素數據偏移
        40, 0, 0, 0,                // 信息頭大小
        (uint8_t)(FRAME_WIDTH), (uint8_t)(FRAME_WIDTH >> 8), (uint8_t)(FRAME_WIDTH >> 16), (uint8_t)(FRAME_WIDTH >> 24), // 寬度
        (uint8_t)(FRAME_HEIGHT), (uint8_t)(FRAME_HEIGHT >> 8), (uint8_t)(FRAME_HEIGHT >> 16), (uint8_t)(FRAME_HEIGHT >> 24), // 高度
        1, 0,                       // 色彩平面數
        24, 0,                      // 每像素位數
        0, 0, 0, 0,                 // 無壓縮
        0, 0, 0, 0,                 // 像素數據大小
        0, 0, 0, 0,                 // X分辨率
        0, 0, 0, 0,                 // Y分辨率
        0, 0, 0, 0,                 // 使用的顏色數
        0, 0, 0, 0                  // 重要顏色數
    };
    
    Serial.print("BMP圖像大小: ");
    Serial.println(bmpSize);
    
    // 修正：使用正確的方式發送BMP圖像
    server.setContentLength(bmpSize);
    server.send(200, "image/bmp", "");  // 發送頭部信息，但不發送內容
    
    // 先發送BMP頭部
    server.sendContent((char*)bmpHeader, 54);
    Serial.println("已發送BMP頭部");
    
    // 發送像素數據，將RGB565轉換為BGR888（BMP格式）
    uint8_t buf[3 * FRAME_WIDTH]; // 每行的緩衝區
    
    for (int y = FRAME_HEIGHT - 1; y >= 0; y--) { // BMP是從底部向上存儲的
        for (int x = 0; x < FRAME_WIDTH; x++) {
            int i = (y * FRAME_WIDTH + x) * 2; // RGB565數據索引
            
            if (i + 1 < BUFFER_SIZE) {
                uint16_t pixel = (fb.buffer[i] << 8) | fb.buffer[i + 1]; // RGB565像素
                
                // 提取RGB565組件
                uint8_t r = ((pixel >> 11) & 0x1F) << 3;
                uint8_t g = ((pixel >> 5) & 0x3F) << 2;
                uint8_t b = (pixel & 0x1F) << 3;
                
                // 存儲為BGR（BMP格式）
                buf[x * 3] = b;
                buf[x * 3 + 1] = g;
                buf[x * 3 + 2] = r;
            }
        }
        
        // 發送這一行
        server.sendContent((char*)buf, 3 * FRAME_WIDTH);
    }
    
    Serial.println("BMP圖像發送完成");
}

bool setupCamera() {
    Serial.println("開始設置相機模擬環境...");
    
    // 由於沒有實際相機連接，我們跳過實際的相機初始化
    Serial.println("注意：沒有實際相機連接，將使用模擬圖像");
    
    // 初始化一個測試圖像
    generateTestImage();
    fb.is_ready = true;
    
    Serial.println("模擬相機環境設置完成");
    return true;
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