/*
  Project: IoT Smart Building Monitoring System
  Board: ESP32
  Description:
  This project monitors building stability using an ESP32 with:
    - VL53L0X distance sensor for displacement/crack detection
    - MPU6050 accelerometer/gyroscope for tilt and vibration
  Features:
    - Real-time monitoring of displacement, tilt, and vibration
    - Three-level alerts (Normal, Warning, Critical)
    - Telegram bot notifications with sensor data and timestamps
    - Buzzer alarms for onsite alerts
  Application:
    - Smart monitoring of buildings, bridges, and infrastructure
    - Early warning system for instability or collapse
*/

#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <MPU6050.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <UniversalTelegramBot.h>

// === Hardware Objects ===
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
MPU6050 mpu;

// === Pins ===
#define BUZZER_PIN 25

// === WiFi & Telegram Config ===
const char* WIFI_SSID = "realme GT NEO 3";
const char* WIFI_PASS = "miskinsangatke";
#define BOT_TOKEN "7709722010:AAE6EH3DVak6my5Az8AX-v1g_iMTekHJkCs"
const char* CHAT_ID = "1034103142";

WiFiClientSecure client;
UniversalTelegramBot bot(BOT_TOKEN, client);

// === Thresholds ===
const float DIST_WARNING_CM = 2.0;
const float DIST_CRITICAL_CM = 5.0;
const float TILT_WARNING_DEG = 5.0;
const float TILT_CRITICAL_DEG = 10.0;

// === States ===
enum AlertLevel { NORMAL, WARNING, CRITICAL };
AlertLevel currentAlert = NORMAL;

// Timer for periodic Telegram
unsigned long lastTelegramTime = 0;
const unsigned long TELEGRAM_INTERVAL = 60000; // 60 sec
const unsigned long ALERT_COOLDOWN = 10000;    // 10 sec min between messages

// === Baseline distance ===
float baseline_cm = 0;

// === Moving Average for Distance ===
#define FILTER_SIZE 5
float distanceBuffer[FILTER_SIZE];
int filterIndex = 0;
bool bufferFilled = false;

// === Buzzer Pattern ===
unsigned long buzzerTimer = 0;
bool buzzerState = false;

// === Helpers ===
void buzzerPattern(AlertLevel level) {
    unsigned long now = millis();
    int interval = 0;

    if (level == WARNING) interval = 400;
    else if (level == CRITICAL) interval = 200;
    else {
        digitalWrite(BUZZER_PIN, LOW);
        buzzerState = false;
        return;
    }

    if (now - buzzerTimer >= interval) {
        buzzerTimer = now;
        buzzerState = !buzzerState;
        digitalWrite(BUZZER_PIN, buzzerState ? HIGH : LOW);
    }
}

void sendTelegram(String msg) {
    Serial.println("📲 Sending Telegram...");
    if (bot.sendMessage(CHAT_ID, msg, "")) Serial.println("✅ Sent");
    else Serial.println("❌ Failed");
}

float getTiltDeg() {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    float axg = ax / 16384.0;
    float ayg = ay / 16384.0;
    float azg = az / 16384.0;
    float roll = atan2(ayg, azg) * 57.2958;
    float pitch = atan2(-axg, sqrt(ayg*ayg + azg*azg)) * 57.2958;
    return max(abs(roll), abs(pitch));
}

float movingAverage(float newValue) {
    distanceBuffer[filterIndex] = newValue;
    filterIndex = (filterIndex + 1) % FILTER_SIZE;
    if (filterIndex == 0) bufferFilled = true;

    int count = bufferFilled ? FILTER_SIZE : filterIndex;
    float sum = 0;
    for (int i = 0; i < count; i++) sum += distanceBuffer[i];
    return sum / count;
}

void ensureWiFi() {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("🔄 Reconnecting WiFi...");
        WiFi.begin(WIFI_SSID, WIFI_PASS);
        unsigned long start = millis();
        while (WiFi.status() != WL_CONNECTED && millis() - start < 10000) {
            delay(500);
            Serial.print(".");
        }
        if (WiFi.status() == WL_CONNECTED) Serial.println("\n✅ WiFi reconnected");
        else Serial.println("\n❌ WiFi failed");
    }
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    Wire.begin(21, 22);

    // VL53L0X
    if (!lox.begin()) {
        Serial.println("❌ VL53L0X not found");
        while (1);
    }
    lox.setMeasurementTimingBudgetMicroSeconds(200000);

    // MPU6050
    mpu.initialize();
    Serial.println(mpu.testConnection() ? "✅ MPU6050 OK" : "❌ MPU6050 not found");

    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);

    // WiFi
    Serial.println("Connecting WiFi...");
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\n✅ WiFi connected");
    client.setInsecure();

    // Baseline (average of 10 readings under 60 cm)
    float sum = 0;
    int valid = 0;
    for (int i = 0; i < 10; i++) {
        VL53L0X_RangingMeasurementData_t measure;
        lox.rangingTest(&measure, false);
        if (measure.RangeStatus != 4) {
            float dist = measure.RangeMilliMeter / 10.0;
            if (dist < 60.0) { // only count < 60 cm
                sum += dist;
                valid++;
            }
        }
        delay(100);
    }
    baseline_cm = (valid > 0) ? sum / valid : 50.0;
    Serial.print("📏 Baseline set to: "); Serial.println(baseline_cm);
    sendTelegram("🏗️ Smart Building Monitor started ✅\n📏 Baseline: " + String(baseline_cm,1) + " cm");
}

void loop() {
    ensureWiFi();

    VL53L0X_RangingMeasurementData_t measure;
    lox.rangingTest(&measure, false);
    float distance_cm = -1;
    float delta = 0;

    if (measure.RangeStatus != 4) {
        distance_cm = movingAverage(measure.RangeMilliMeter / 10.0);
        delta = abs(distance_cm - baseline_cm);
    }

    float tilt = getTiltDeg();

    Serial.print("📏 Distance: ");
    if (distance_cm > 0) Serial.print(distance_cm);
    else Serial.print("Out of range");
    Serial.print(" cm | Δ: "); Serial.print(delta);
    Serial.print(" cm | 📐 Tilt: "); Serial.println(tilt);

    // --- Alert Logic ---
    AlertLevel newAlert = NORMAL;
    if (delta > DIST_CRITICAL_CM || tilt > TILT_CRITICAL_DEG) newAlert = CRITICAL;
    else if (delta > DIST_WARNING_CM || tilt > TILT_WARNING_DEG) newAlert = WARNING;

    unsigned long now = millis();
    static unsigned long lastAlertChange = 0;

    if (newAlert != currentAlert && (now - lastAlertChange > ALERT_COOLDOWN)) {
        currentAlert = newAlert;
        lastAlertChange = now;
        lastTelegramTime = 0; // force immediate send
    }

    // Telegram messages
    if ((currentAlert == WARNING || currentAlert == CRITICAL) && (now - lastTelegramTime > TELEGRAM_INTERVAL)) {
        lastTelegramTime = now;
        String msg = (currentAlert == WARNING) ? "⚠️ WARNING!\n" : "🚨 CRITICAL!\n";
        msg += "📏 Distance: " + String(distance_cm, 2) + " cm\n";
        msg += "Δ from baseline: " + String(delta, 2) + " cm\n";
        msg += "📐 Tilt: " + String(tilt, 2) + "°";
        sendTelegram(msg);
    }

    if (currentAlert == NORMAL) {
        noTone(BUZZER_PIN);
        buzzerState = false;
    }

    buzzerPattern(currentAlert);
    delay(200);
}
