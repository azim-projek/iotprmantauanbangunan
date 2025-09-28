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

// ESP32 + VL53L0X + Adafruit_MPU6050 + Telegram
// Requirements: Adafruit_VL53L0X, Adafruit_MPU6050, UniversalTelegramBot
// Fill WIFI_SSID, WIFI_PASS, BOT_TOKEN and CHAT_ID

#include <Wire.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <UniversalTelegramBot.h>
#include <Arduino.h>

#include <Adafruit_VL53L0X.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#define BUZZER_PIN  25   // choose a free GPIO pin on ESP32
#define BUZZER_CHANNEL 0   // any free ledc channel 0–15


// --- CONFIG (isi sendiri) ---
const char* WIFI_SSID = "iphone";
const char* WIFI_PASS = "12345678";
#define BOT_TOKEN "7709722010:AAE6EH3DVak6my5Az8AX-v1g_iMTekHJkCs"
const char* CHAT_ID = "1034103142"; // string

// Sampling / timing
const unsigned long STATUS_INTERVAL_MS = 60UL * 1000UL; // periodic status message (1 min)
const int BASELINE_SAMPLES = 30; // samples to compute baseline at startup
const int AVG_SAMPLES = 8; // rolling average for distance

// Thresholds (cm / degrees)
const float WARNING_CM = 0.5;      // 5 mm
const float CRITICAL_CM = 1.5;     // 15 mm
const float TILT_DEG_THRESHOLD = 5.0; // degrees (same as before)
const float ACCEL_SPIKE_G = 1.5;   // g (same as before)

// hysteresis: require drop below this to clear alert
const float HYSTERESIS_CM = 0.2;   // 2 mm

// --- Objects ---
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
Adafruit_MPU6050 mpu;
WiFiClientSecure secureClient;
UniversalTelegramBot bot(BOT_TOKEN, secureClient);

// rolling storage
float distBuffer[AVG_SAMPLES];
int distIndex = 0;
bool distBufferFilled = false;

// baseline
float baseline_mm = 0.0;
bool baseline_ready = false;

// state
enum AlertLevel { NORMAL=0, WARNING=1, CRITICAL=2 };
AlertLevel currentAlert = NORMAL;
AlertLevel lastSentLevel = NORMAL;
bool doSend = false;
unsigned long lastStatusMillis = 0;
unsigned long lastAlertTime = 0;
const unsigned long ALERT_COOLDOWN = 30000; // 30 seconds
unsigned long lastBuzzerTime = 0;
int buzzerState = 0;



// helpers
void sendTelegram(const String &msg) {
  Serial.println("Sending Telegram...");
  bool ok = bot.sendMessage(CHAT_ID, msg, "");
  if (ok) Serial.println("Telegram sent.");
  else Serial.println("Telegram failed.");
}

String nowTimestamp() {
  time_t now = time(nullptr);
  struct tm *ti = localtime(&now);
  char buf[32];
  if (ti) strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", ti);
  else strcpy(buf, "no-time");
  return String(buf);
}

float readAverageDistanceMM(int samples=AVG_SAMPLES) {
  double sum = 0;
  for (int i=0;i<samples;i++){
    VL53L0X_RangingMeasurementData_t measure;
    lox.rangingTest(&measure, false);
    float mm = NAN;
    if (measure.RangeStatus == 0) {
      mm = (float)measure.RangeMilliMeter;
    } else {
      // if invalid reading, use previous valid in buffer if exists
      mm = NAN;
    }
    if (isnan(mm)) {
      // try again small delay and fallback to previous buffer value
      delay(20);
      lox.rangingTest(&measure, false);
      if (measure.RangeStatus == 0) mm = (float)measure.RangeMilliMeter;
    }
    if (isnan(mm)) {
      // fallback to last buffer value if available
      int lastIdx = (distIndex - 1 + AVG_SAMPLES) % AVG_SAMPLES;
      if (distBufferFilled || lastIdx != distIndex) mm = distBuffer[lastIdx];
      else mm = 0.0;
    }
    sum += mm;
    delay(10);
  }
  return (float)(sum / samples);
}

float getTiltDegrees() {
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  mpu.getEvent(&accel, &gyro, &temp);
  // compute pitch (rotation around X) and roll (rotation around Y)
  // using accelerometer only (simple)
  float ax = accel.acceleration.x;
  float ay = accel.acceleration.y;
  float az = accel.acceleration.z;
  // prevent div by zero
  float roll = atan2(ay, az) * 57.295779513; // deg
  float pitch = atan2(-ax, sqrt(ay*ay + az*az)) * 57.295779513; // deg
  // return max absolute tilt
  return max(fabs(pitch), fabs(roll));
}

float getAccelGMax() {
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  mpu.getEvent(&accel, &gyro, &temp);
  float ax = accel.acceleration.x;
  float ay = accel.acceleration.y;
  float az = accel.acceleration.z;
  // convert m/s^2 to g (1g ≈ 9.80665 m/s^2)
  float gX = fabs(ax / 9.80665);
  float gY = fabs(ay / 9.80665);
  float gZ = fabs(az / 9.80665);
  return max(gX, max(gY, gZ));
}

void computeBaseline() {
  Serial.println("Computing baseline, keep structure stable...");
  double sum = 0;
  int valid = 0;
  for (int i=0;i<BASELINE_SAMPLES;i++){
    VL53L0X_RangingMeasurementData_t measure;
    lox.rangingTest(&measure, false);
    if (measure.RangeStatus == 0) {
      float mm = (float)measure.RangeMilliMeter;
      sum += mm;
      valid++;
    } else {
      i--; // retry this sample
      delay(50);
      continue;
    }
    delay(100);
  }
  if (valid > 0) {
    baseline_mm = (float)(sum / valid);
    baseline_ready = true;
    Serial.print("Baseline established (mm): ");
    Serial.println(baseline_mm);
  } else {
    Serial.println("Baseline failed to compute.");
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);

  // init time (optional) for timestamp in messages
 configTime(8 * 3600, 0, "pool.ntp.org", "time.google.com");
 // Force timezone to Malaysia
setenv("TZ", "Asia/Kuala_Lumpur", 1);
tzset();

// in setup():
pinMode(BUZZER_PIN, OUTPUT);
noTone(BUZZER_PIN); // start silent

  // Wire and sensors
  Wire.begin();
  Serial.println("Init VL53L0X...");
  if (!lox.begin()) {
    Serial.println("Failed to boot VL53L0X");
    while (1) { delay(10); }
  }
  // increase timing budget for accuracy (default 33ms)
  lox.setMeasurementTimingBudgetMicroSeconds(200000); // 200ms => more stable/accurate

  Serial.println("Init MPU6050...");
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) { delay(10); }
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  delay(100);

  // WiFi
  Serial.print("Connecting to WiFi ");
  Serial.print(WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500); Serial.print(".");
  }
  Serial.println("\nWiFi connected, IP: "); Serial.println(WiFi.localIP());

  // secure client for Telegram
  // simple approach: allow insecure for easier compatibility
  secureClient.setInsecure();

  // compute baseline
  computeBaseline();

  // warm up rolling buffer with baseline
  for (int i=0;i<AVG_SAMPLES;i++) {
    distBuffer[i] = baseline_mm;
  }
  distIndex = 0;
  distBufferFilled = true;

  // initial status message
  String initMsg = "SISTEM MONITOR BANGUNAN AKTIF\nBaseline: " + String(baseline_mm, 2) + " mm\nWaktu: " + nowTimestamp();
  sendTelegram(initMsg);

  lastStatusMillis = millis();
}

void ensureWiFiConnected() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("⚠️ WiFi disconnected! Reconnecting...");
    WiFi.disconnect();
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    unsigned long startAttempt = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < 10000) {
      delay(500);
      Serial.print(".");
    }
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("\n✅ WiFi reconnected");
    } else {
      Serial.println("\n❌ WiFi reconnect failed");
    }
  }
}
// globals:
bool buzzerOn = false;
unsigned long buzzerDuration = 0;

void buzzerWarningLoop() {
  unsigned long now = millis();

  if (!buzzerOn && now - lastBuzzerTime >= 500) {
    // start beep
    lastBuzzerTime = now;
    tone(BUZZER_PIN, 1000); // WARNING
    buzzerOn = true;
    buzzerDuration = 200; // ms
  }

  if (buzzerOn && (now - lastBuzzerTime >= buzzerDuration)) {
      noTone(BUZZER_PIN); 
    buzzerOn = false;
  }
}

void buzzerCriticalLoop() {
  unsigned long now = millis();

  if (!buzzerOn && now - lastBuzzerTime >= 250) {
    lastBuzzerTime = now;
   tone(BUZZER_PIN, 2000); // CRITICAL
    buzzerOn = true;
    buzzerDuration = 150;
  }

  if (buzzerOn && (now - lastBuzzerTime >= buzzerDuration)) {
    noTone(BUZZER_PIN);     // stop
    buzzerOn = false;
  }
}

void loop() {

ensureWiFiConnected();

// Periodic NTP resync every 15 min
static unsigned long lastNtpSync = 0;
if (millis() - lastNtpSync > 900000) {
  configTime(8 * 3600, 0, "pool.ntp.org", "time.google.com");
  setenv("TZ", "Asia/Kuala_Lumpur", 1);
  tzset();
  lastNtpSync = millis();
}
  // read distance average sample
  float mm = readAverageDistanceMM(4); // average 4 quick samples
  // push into circular buffer
  distBuffer[distIndex++] = mm;
  if (distIndex >= AVG_SAMPLES) { distIndex = 0; distBufferFilled = true; }

  // compute rolling avg
  double sum = 0;
  int count = distBufferFilled ? AVG_SAMPLES : distIndex;
  for (int i=0;i<count;i++) sum += distBuffer[i];
  float avg_mm = (float)(sum / max(1, count));

  float delta = baseline_mm - avg_mm; // positive if distance decreased (sensor closer) => structure dropped downwards (depends mounting)
  // NOTE: adjust sign depending how you mount sensor; here assume drop = decrease in measured distance
  float absDelta = fabs(delta);

  // read MPU6050 -> tilt & acceleration
  float tiltDeg = getTiltDegrees();
  float accelG = getAccelGMax();

// Decide alert level with hysteresis
AlertLevel newAlert = currentAlert; // start from previous state

if (currentAlert == NORMAL) {
  // escalate to warning/critical
  if (absDelta >= CRITICAL_CM || tiltDeg >= TILT_DEG_THRESHOLD*2 || accelG >= (ACCEL_SPIKE_G + 0.5)) {
    newAlert = CRITICAL;
  } else if (absDelta >= WARNING_CM || tiltDeg >= TILT_DEG_THRESHOLD || accelG >= ACCEL_SPIKE_G) {
    newAlert = WARNING;
  }

} else if (currentAlert == WARNING) {
  // escalate to critical
  if (absDelta >= CRITICAL_CM || tiltDeg >= TILT_DEG_THRESHOLD*2 || accelG >= (ACCEL_SPIKE_G + 0.5)) {
    newAlert = CRITICAL;
  }
  // de-escalate to normal only if below warning minus hysteresis
  else if (absDelta < (WARNING_CM - HYSTERESIS_CM) &&
           tiltDeg < (TILT_DEG_THRESHOLD - 1.0) &&
           accelG < (ACCEL_SPIKE_G - 0.1)) {
    newAlert = NORMAL;
  }

} else if (currentAlert == CRITICAL) {
  // clear critical only if well below critical minus hysteresis
  if (absDelta < (CRITICAL_CM - HYSTERESIS_CM) &&
      tiltDeg < (TILT_DEG_THRESHOLD*2 - 1.0) &&
      accelG < (ACCEL_SPIKE_G + 0.5 - 0.2)) {
    // maybe drop to WARNING first or straight to NORMAL
    if (absDelta >= WARNING_CM) newAlert = WARNING;
    else newAlert = NORMAL;
  }
}
if (newAlert != currentAlert) {
  currentAlert = newAlert;
  lastSentLevel = currentAlert;
  doSend = true;  // trigger message
} else {
  doSend = false; // no change, don’t resend
}

  // Send status periodically even if normal
  unsigned long now = millis();
  if (now - lastStatusMillis >= STATUS_INTERVAL_MS) {
    lastStatusMillis = now;
    // send periodic status
    String msg = "📊 Building Monitor - STATUS\n📏 Distance: " + String(avg_mm, 2) + " mm\n🔻 Delta: " + String(absDelta, 2) + " mm\n📐 Tilt: " + String(tiltDeg, 2) + "°\n⚙️ Accel(g): " + String(accelG,2) + "\n🕒 " + nowTimestamp();
    sendTelegram(msg);
  }

  // send alert message if state changed/stable
  // send alert message if state changed/stable
if (doSend) {
  unsigned long nowMs = millis();

  // Apply cooldown only for WARNING/CRITICAL
  if (lastSentLevel == NORMAL || (nowMs - lastAlertTime >= ALERT_COOLDOWN)) {
    lastAlertTime = nowMs; // reset cooldown timer

    if (lastSentLevel == NORMAL) {
      String msg = "✅ STATUS NORMAL\n📏 Distance: " + String(avg_mm, 2) +
                   " mm\n🔻 Delta: " + String(absDelta, 2) +
                   " mm\n📐 Tilt: " + String(tiltDeg, 2) +
                   "°\n🕒 " + nowTimestamp();             
      sendTelegram(msg);
    } else if (lastSentLevel == WARNING) {
      String msg = "⚠️ WARNING: Perubahan dikesan\n📏 Distance: " + String(avg_mm, 2) +
                   " mm\n🔻 Delta: " + String(absDelta, 2) +
                   " mm\n📐 Tilt: " + String(tiltDeg, 2) +
                   "°\n🕒 " + nowTimestamp();
      sendTelegram(msg);
    } else if (lastSentLevel == CRITICAL) {
      String msg = "🚨 CRITICAL ALERT! Risiko struktur\n📏 Distance: " + String(avg_mm, 2) +
                   " mm\n🔻 Delta: " + String(absDelta, 2) +
                   " mm\n📐 Tilt: " + String(tiltDeg, 2) +
                   "°\n🕒 " + nowTimestamp();
      sendTelegram(msg);
    } 
  }
}
switch (currentAlert) {
  case NORMAL:
   noTone(BUZZER_PIN);  // stop buzzer
    break;
  case WARNING:
    buzzerWarningLoop();
    break;
  case CRITICAL:
    buzzerCriticalLoop();
    break;
}

  // small delay; actual loop frequency controlled by STATUS_INTERVAL_MS for messages
  delay(300);
}
