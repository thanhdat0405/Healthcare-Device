#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#include <Wire.h>

#include "MAX30105.h"
#include "heartRate.h"
#include "spo2_algorithm.h"

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include <math.h>

// ======================================================
// CONFIG ĐỊNH NGHĨA NÚT BẤM (GPIO)
// ======================================================
#define BTN_K1_SCREEN 3   // Bật/tắt màn hình
#define BTN_K2_MODE   2   // Chuyển đổi màn hình hiển thị
#define BTN_K3_MEASURE 7  // Bật/tắt chế độ đo
#define BTN_K4_RESET   6  // Reset số bước chân

// Trạng thái hệ thống
bool isScreenOn = true;
int displayMode = 0;       // 0: Hiển thị tất cả, 1: Chỉ HR/SpO2, 2: Chỉ Steps
bool isMeasuringOn = true; // Mặc định bật chế độ đo

// Quản lý thời gian tự tắt màn hình (Timeout 30s)
unsigned long lastButtonActivity = 0;
const unsigned long SCREEN_TIMEOUT = 30000; 

// ======================================================
// OLED (SSD1315 tương thích hoàn toàn với SSD1306)
// ======================================================
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ======================================================
// MAX30102
// ======================================================
MAX30105 particleSensor;
#define BUFFER_SIZE 100

uint32_t irBuffer[BUFFER_SIZE];
uint32_t redBuffer[BUFFER_SIZE];
int bufferIndex = 0;

// ===== SpO2 =====
int32_t spo2;
int8_t validSPO2;
int32_t heartRate;
int8_t validHeartRate;

// ===== MAX30102 CONFIG =====
byte ledBrightness = 40;
byte sampleAverage = 4;
byte ledMode = 2;
int sampleRate = 400;
int pulseWidth = 411;
int adcRange = 4096;

// ======================================================
// HEART RATE
// ======================================================
const byte RATE_SIZE = 6;
byte rates[RATE_SIZE];
byte rateSpot = 0;
long lastBeat = 0;
float beatsPerMinute = 0;
float smoothHR = 0;
int beatAvg = 0;

// ======================================================
// MPU6050
// ======================================================
Adafruit_MPU6050 mpu;
int steps = 0;
float lastAccel = 0;
bool stepState = false;

// ======================================================
// BLE
// ======================================================
#define SERVICE_UUID        "85c72eca-2d84-446a-b158-d7af88851e3e"
#define CHARACTERISTIC_UUID "84556562-2fe0-4b64-b350-fefcf02c33ed"

BLECharacteristic *pCharacteristic;
bool deviceConnected = false;
unsigned long lastSend = 0;
const long sendInterval = 5000;

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    Serial.println("Device connected");
  }
  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    Serial.println("Device disconnected");
    BLEDevice::startAdvertising();
  }
};

// ======================================================
// HÀM ĐỌC NÚT BẤM CHỐNG DỘI (DEBOUNCE)
// ======================================================
bool checkButton(int pin) {
  if (digitalRead(pin) == LOW) {
    delay(20); // Chống dội phím ngắn để không ảnh hưởng luồng đo
    if (digitalRead(pin) == LOW) {
      while (digitalRead(pin) == LOW); // Chờ nhả nút
      lastButtonActivity = millis();   // Cập nhật thời gian tương tác cuối
      return true;
    }
  }
  return false;
}

// ======================================================
// SETUP
// ======================================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin(9, 8);
  Serial.println("=== SYSTEM START ===");

  // Cấu hình các chân nút bấm
  pinMode(BTN_K1_SCREEN, INPUT_PULLUP);
  pinMode(BTN_K2_MODE, INPUT_PULLUP);
  pinMode(BTN_K3_MEASURE, INPUT_PULLUP);
  pinMode(BTN_K4_RESET, INPUT_PULLUP);

  // Khởi tạo OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED not found");
    while (1);
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("SYSTEM START");
  display.display();
  Serial.println("OLED OK");

  // Khởi tạo MAX30102
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30102 not found");
    while (1);
  }

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
  particleSensor.setPulseAmplitudeRed(0x3F);
  particleSensor.setPulseAmplitudeIR(0x3F);
  Serial.println("MAX30102 OK");

  // Khởi tạo MPU6050
  if (!mpu.begin()) {
    Serial.println("MPU6050 not found");
    while (1);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println("MPU6050 OK");

  // Khởi tạo BLE
  BLEDevice::init("ESP32C3_Health");
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);

  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
                    );
  pCharacteristic->addDescriptor(new BLE2902());
  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->start();
  Serial.println("BLE STARTED");

  lastButtonActivity = millis(); // Khởi tạo mốc thời gian hoạt động
  delay(1000);
}

// ======================================================
// LOOP
// ======================================================
void loop() {
  
  // ------------------------------------------------------
  // XỬ LÝ NÚT BẤM
  // ------------------------------------------------------
  
  // K1: Bật / Tắt màn hình thủ công
  if (checkButton(BTN_K1_SCREEN)) {
    isScreenOn = !isScreenOn;
    Serial.print("Screen toggled: ");
    Serial.println(isScreenOn ? "ON" : "OFF");
  }

  // K2: Đổi chế độ hiển thị màn hình
  if (checkButton(BTN_K2_MODE)) {
    if (!isScreenOn) isScreenOn = true; // Bật lại màn hình nếu đang tắt
    displayMode = (displayMode + 1) % 3;
    Serial.print("Display mode changed to: ");
    Serial.println(displayMode);
  }

  // K3: Bật / Tắt chế độ đo cảm biến
  if (checkButton(BTN_K3_MEASURE)) {
    isMeasuringOn = !isMeasuringOn;
    Serial.print("Measurement toggled: ");
    Serial.println(isMeasuringOn ? "ON" : "OFF");
    
    if(!isMeasuringOn) {
      smoothHR = 0;
      spo2 = 0;
      validSPO2 = 0;
    }
  }

  // K4: Reset số bước chân về 0
  if (checkButton(BTN_K4_RESET)) {
    steps = 0;
    Serial.println("Steps reset to 0");
  }

  // Tự động tắt màn hình sau 30 giây không hoạt động
  if (isScreenOn && (millis() - lastButtonActivity > SCREEN_TIMEOUT)) {
    isScreenOn = false;
    Serial.println("Screen auto timeout (30s reached).");
  }

  // ------------------------------------------------------
  // ĐỌC VÀ XỬ LÝ SENSOR (Chỉ chạy khi Chế độ đo đang BẬT)
  // ------------------------------------------------------
  long irValue = 0;
  long redValue = 0;

  if (isMeasuringOn) {
    particleSensor.check();
    irValue = particleSensor.getIR();
    redValue = particleSensor.getRed();

    // ===== THUẬT TOÁN HEART RATE =====
    if (irValue > 10000) {
      if (checkForBeat(irValue)) {
        long delta = millis() - lastBeat;
        lastBeat = millis();
        beatsPerMinute = 60.0 / (delta / 1000.0);

        if (beatsPerMinute > 40 && beatsPerMinute < 180) {
          rates[rateSpot++] = (byte)beatsPerMinute;
          rateSpot %= RATE_SIZE;
          beatAvg = 0;
          for (byte x = 0; x < RATE_SIZE; x++) {
            beatAvg += rates[x];
          }
          beatAvg /= RATE_SIZE;

          if (smoothHR == 0) smoothHR = beatAvg;
          smoothHR = 0.85 * smoothHR + 0.15 * beatAvg;
        }
      }
    } else {
      smoothHR = 0;
    }

    // ===== THUẬT TOÁN SpO2 =====
    irBuffer[bufferIndex] = irValue;
    redBuffer[bufferIndex] = redValue;
    bufferIndex++;

    if (bufferIndex >= BUFFER_SIZE) {
      bufferIndex = 0;
      maxim_heart_rate_and_oxygen_saturation(
        irBuffer, BUFFER_SIZE, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate
      );
    }

    // ===== THUẬT TOÁN MPU6050 & STEP COUNTER =====
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    float accel = sqrt(
                    a.acceleration.x * a.acceleration.x +
                    a.acceleration.y * a.acceleration.y +
                    a.acceleration.z * a.acceleration.z
                  );

    if (accel > 11.5 && lastAccel <= 11.5) {
      if (!stepState) {
        steps++;
        stepState = true;
      }
    }
    if (accel < 10) {
      stepState = false;
    }
    lastAccel = accel;
  }

  // ------------------------------------------------------
  // HIỂN THỊ OLED TRÊN MÀN HÌNH (Theo chế độ)
  // ------------------------------------------------------
  display.clearDisplay();

  if (isScreenOn) {
    display.setTextSize(1);
    display.setCursor(0, 0);
    
    if (!isMeasuringOn) {
      display.println("Health Monitor [PAUSED]");
    } else {
      display.println("Health Monitor");
    }

    // Giao diện hiển thị theo Mode (K2)
    if (displayMode == 0 || displayMode == 1) {
      // ===== HIỂN THỊ HR =====
      display.setCursor(0, 18);
      display.print("HR: ");
      if (isMeasuringOn && irValue > 10000)
        display.print((int)smoothHR);
      else
        display.print("--");
      display.println(" bpm");

      // ===== HIỂN THỊ SpO2 =====
      display.setCursor(0, 34);
      display.print("SpO2: ");
      if (isMeasuringOn && validSPO2)
        display.print(spo2);
      else
        display.print("--");
      display.println(" %");
    }

    if (displayMode == 0 || displayMode == 2) {
      // ===== HIỂN THỊ STEPS =====
      int yPos = (displayMode == 2) ? 18 : 50; // Đẩy dòng lên nếu chỉ hiện Step
      display.setCursor(0, yPos);
      display.print("Steps: ");
      display.print(steps);
    }
  } 
  // Nếu isScreenOn == false, display.clearDisplay() sẽ làm màn hình tối đen (Tắt)
  display.display();

  // ------------------------------------------------------
  // SERIAL DEBUG
  // ------------------------------------------------------
  Serial.print("HR: ");
  if (isMeasuringOn && irValue > 10000) Serial.print((int)smoothHR);
  else Serial.print("--");
  Serial.print(" bpm | SpO2: ");
  if (isMeasuringOn && validSPO2) Serial.print(spo2);
  else Serial.print("--");
  Serial.print(" % | Steps: ");
  Serial.print(steps);
  Serial.print(" | Meas: "); Serial.print(isMeasuringOn ? "ON" : "OFF");
  Serial.print(" | Mode: "); Serial.println(displayMode);

  // ------------------------------------------------------
  // BLE SEND (Vẫn giữ nguyên luồng truyền dữ liệu)
  // ------------------------------------------------------
  if (deviceConnected) {
    if (millis() - lastSend > sendInterval) {
      lastSend = millis();

      String data = String((int)smoothHR) + "," + String(spo2) + "," + String(steps);
      pCharacteristic->setValue(data.c_str());
      pCharacteristic->notify();
      Serial.println("Sent BLE: " + data);
    }
  }

  delay(2);
}