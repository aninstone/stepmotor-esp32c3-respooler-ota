// ----------------- LTS RESPOOLER CONTROL BOARD CODE -----------------

#include <TMCStepper.h>
#include <NimBLEDevice.h>
#include <Preferences.h>
#include <math.h>
#include <string>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <Update.h>
#include <ArduinoJson.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <AiEsp32RotaryEncoder.h>
#include <Arduino.h>
#include "esp_bt.h"
#include "esp_coexist.h"
#include "esp_mac.h"
#include <driver/ledc.h>
#include "driver/gpio.h"
#include <freertos/semphr.h>

// ------------------------ Board Info ----------------------------
#define FIRMWARE_VERSION "1.4.0"
#define BOARD_NAME "LTS CB"

// ------------------------ OTA firmware source --------------------
// Default = vendor server. Can be overridden at runtime over BLE with
// {"set":{"OTAURL":"http://192.168.1.10:8000/firmware.bin"}} and it is
// stored in NVS, so you can OTA your own builds without recompiling.
// Both http:// and https:// are supported (https skips cert check).
#define OTA_URL_DEFAULT \
  "https://respooler.lts-design.com/Firmware/ControlBoard_V4_OTA.bin"
String otaUrl = OTA_URL_DEFAULT;

// --------------------- Hardware Pin Defines ---------------------
// NOTE: Original board was ESP32-S3. This project now targets ESP32-C3
// (board = airm2m_core_esp32c3), matching your bajiesp32 setup.
// Remap rules for ESP32-C3:
//   - S3-only GPIO40 / GPIO47 do not exist on C3 -> moved to GPIO4 / GPIO5
//   - C3 GPIO11~17 are wired to the internal SPI flash -> MUST NOT be used
//     (TMC UART 17, FILAMENT 11, FAN 13, SERVO 12 all moved)
// Adjust these to match your actual C3 wiring if it differs.
// ---- LuLei CORE-ESP32-C3 flash pin map (official docs, DIO mode) ----
// GPIO11 = flash VDD | GPIO14=CS / 15=CLK / 16=D0 / 17=D1 = FLASH, FORBIDDEN
// GPIO12/13 = only used in QIO mode; board runs DIO -> free
// GPIO0~10 free (2/8/9 are strapping pins, don't pull low at boot)
// TMC2209 UART: GPIO12(RX)/13(TX) — safe on LuLei C3 (DIO mode, not flash).
// Verified working end-to-end via Web Bluetooth control page.
#define TMC_UART_RX 12
#define TMC_UART_TX 13
#define STEP_PIN 6        // was 14 on the C3 port — 14 is FLASH CS, crashed boot!
// was 21 -> GPIO21 is UART0-RX (CH343 debug), was 16 -> 16 is FLASH D0, crashed!
#define DIR_PIN 7         // free GPIO, exposed as module pin 10
#define EN_PIN 4          // was 40 on S3 (stepper enable, active-LOW) -> C3 GPIO4
#define LED_PIN 9

// ---------------- OLED SSD1306 128x64 I2C ----------------
// SDA=GPIO1, SCL=GPIO3 (free on LuLei C3, GPIO20/21 reserved for debug UART)
// Physical wiring on this board is SDA->GPIO3, SCL->GPIO1 (proven by the
// oled_diag scan: (1,3) finds nothing, (3,1) answers 0x3C). Do NOT "fix"
// these back to 1/3 -- the screen goes dark again.
#define OLED_SDA 3
#define OLED_SCL 1
#define OLED_ADDR 0x3C          // most SSD1306 modules; try 0x3D if blank
#define OLED_W 128
#define OLED_H 64
#define LED_CONN_PIN 5    // was 47 on S3 (conn-status PWM LED)     -> C3 GPIO5
#define FILAMENT_PIN 2    // was 11 (GPIO11 = SPI flash on C3, not usable)
#define BUTTON_PIN 10
#define FAN_PIN 19        // was 13 (GPIO13 = SPI flash on C3, not usable)
#define SERVO_PIN 8       // was 12 on S3; 20 not allowed - GPIO20 is UART0-TX
                          // (CH343 debug log pin), using it hijacks the console

// ---------------- Rotary encoder (speed trim) -------------------
// Speed knob: turn = change SPD%, fast spin = bigger jumps (acceleration).
// The knob wins over BLE: for ENC_BLE_LOCKOUT_MS after a turn, incoming
// "SPD" commands from the phone are ignored (stops the app's slider from
// stomping the value you just dialled in), and every knob change is pushed
// back over BLE so the app follows the hardware.
#define ENC_A_PIN 0       // CLK  (GPIO0 free; 11/14-17 are flash pins!)
#define ENC_B_PIN 18      // DT   (18/19 = native USB D-/D+, unused here:
                          //       the console runs through the CH343 on 20/21)
#define ENC_SW_PIN -1     // knob button: -1 = not wired (button on GPIO10)
#define ENC_STEPS 4       // detents per count -- try 2 if one click adds 2
#define ENC_ACCEL 90      // 0 = off; bigger = stronger "fast spin" boost
#define ENC_MAX_JUMP 10   // cap: at most +/-10 % SPD per encoder event
#define ENC_BLE_LOCKOUT_MS 2000
#define ENC_NVS_SETTLE_MS 800    // write flash only after the knob settles

// ------------------------ Motor Parameters ----------------------
#define MICROSTEPPING 8
#define R_SENSE 0.11f

// -------------------- Timing and Limits --------------------------
#define START_INTERVAL_US 700
#define DEFAULT_INTERVAL_US 116
#define TORQUE_CHECK_INTERVAL_MS 50
#define BUTTON_DEBOUNCE_MS 50
#define ACCEL_UPDATE_INTERVAL 20
#define ACCEL_STEP 5
#define LED_CONN_PULSE_MAX 60
#define STATUS_NOTIFY_INTERVAL 400
#define LED_BLINK_INTERVAL 1000
#define TORQUE_SG_IGNORE 20
#define TORQUE_SG_LIMIT_LOW 0.72
#define TORQUE_SG_LIMIT_MED 0.85
#define TORQUE_SG_LIMIT_HIGH 0.97
#define TORQUE_BELOW_MS 700
#define MIN_RSSI_THRESHOLD -90
#define FILAMENT_LOSS_CONFIRM_MS 100
#define STEP_DUTY_ON 512

#define TARGET_WEIGHT_FACTOR_2 0.59f
#define TARGET_WEIGHT_FACTOR_3 0.34f

#define FAN_CHANNEL LEDC_CHANNEL_1
#define FAN_TIMER LEDC_TIMER_1

#define STEP_LEDC_CHANNEL LEDC_CHANNEL_3
#define STEP_LEDC_TIMER LEDC_TIMER_2

#define SERVO_LEDC_CHANNEL LEDC_CHANNEL_0
#define SERVO_LEDC_TIMER LEDC_TIMER_3

// ------------------- BLE UUIDs -----------------------------------
#define SERVICE_UUID "9E05D06D-68A7-4E1F-A503-AE26713AC101"
#define CHARACTERISTIC_UUID "7CB2F1B4-7E3F-43D2-8C92-DF58C9A7B1A8"

// ------------------- Forward Declarations ------------------------
// These are defined further down the file but called from earlier tasks.
int calculatePWM(int brightnessPercent);
void setConnLed(uint8_t duty);
void setMotorCurrent(int percent);
void handleCommand(const std::string& cmd);
void applySpeedPercent(int pct);   // defined near the rotary-encoder section
bool encoderHasPriority();
void updateOled();

// ------------------- Direct settings variables -------------------
int speedPercent = 85;                        // 50-100
int motorDirection = 0;                       // 0, 1
int ledBrightness = 50;                       // 0-100
bool useFilamentSensor = true;                // true, false
int motorStrength = 100;                      // 80-120
int targetWeight = 0;                         // 0-3
int torqueLimit = 0;                          // 0-3
int jingleStyle = 0;                          // 0-3
unsigned long calibrationAt80Speed = 895000;  // ms

String wifiSSID = "";      // string
String wifiPassword = "";  // string

int fanSpeed = 60;         // 0–100
bool fanAlwaysOn = false;  // true, false
unsigned long fanStopAfter = 0;

int servoAngleR = 5;        // deg
int servoAngleL = 175;      // deg
float servoStepMm = 1.75f;  // mm
bool servoHomeIsR = true;   // true, false

// ----------------- Board variant identifier -----------------
uint8_t boardVariant = 0;

// --------------- High-Speed mode state -----------------
bool highSpeedMode = false;
int savedTorqueLimitHS = 0;
unsigned long baseTargetStepIntervalMicros = DEFAULT_INTERVAL_US;

// -------------- Status variables and objects -----------------
HardwareSerial TMCSerial(1);
TMC2209Stepper driver(&TMCSerial, R_SENSE, 0);
NimBLECharacteristic* pCharacteristic = nullptr;
Preferences prefs;
String serialRxLine;

float guideServoPos = 0.0f;
bool guideServoTowardsL = true;
float motorStepAcc = 0.0f;
unsigned long lastMotorStepAccTime = 0;
bool servoWasRunning = false;
char lastStateForServo = 'I';
bool servoHomingToHome = false;
unsigned long lastServoHomeStepTime = 0;
bool servoHomeAfterDonePending = false;
char lastStateForDone = 'I';
bool servoGotoActive = false;
char servoGotoSide = 0;

StaticJsonDocument<640> lastStatusDoc;

bool deviceConnected = false;
bool isMotorRunning = false;
bool shouldStartMotorNow = false;
bool filamentDetected = false;
bool pendingDirectionChange = false;
bool lastStableButtonState = HIGH;
bool ledState = false;
int newMotorDirection = 0;
int pwmValue = 0;
int triggerJingleNow = 0;
unsigned long delayStartUntil = 0;
unsigned long filamentLostSince = 0;
unsigned long stepIntervalMicros = START_INTERVAL_US;
unsigned long targetStepIntervalMicros = DEFAULT_INTERVAL_US;
unsigned long spoolingStartTime = 0;
unsigned long lastNotify = 0;
unsigned long lastLedToggle = 0;
unsigned long lastAccelUpdate = 0;
unsigned long lastConnLedTime = 0;
unsigned long lastDebounceTime = 0;
unsigned long lastIntervalMicros = 0;
unsigned long totalEstimatedTime = 0;
unsigned long pausedElapsed = 0;
unsigned long buttonPressSince = 0;
bool buttonLongPressHandled = false;
bool buttonAwaitingDecision = false;
bool doneHoldActive = false;
unsigned long doneHoldStart = 0;
bool remHoldActive = false;
unsigned long remHoldExpiry = 0;
bool progHoldActive = false;
unsigned long progHoldExpiry = 0;
unsigned long remStartHoldUntil = 0;
float progress = 0.0;
bool lastIsMotorRunning = false;
bool lastFilamentDetected = false;
float lastProgress = 0.0;
int lastSpeedPercent = 100;
int lastChipTemperature = 0;
int lastRemainingTime = 0;
volatile bool otaInProgress = false;
SemaphoreHandle_t bleNotifyMutex = nullptr;
// --------------------- LED / Indicator Output ----------------------
// Set to 0 to compile out all LED handling (NeoPixel + conn LED).
// Useful when the status LEDs are not wired, or to free flash / GPIO.
#define LED_ENABLE 1

char currentState = 'I';

#if LED_ENABLE
Adafruit_NeoPixel led(1, LED_PIN, NEO_GRB + NEO_KHZ800);

// ---------------- OLED display ----------------
Adafruit_SSD1306 oled(OLED_W, OLED_H, &Wire, -1, 400000);
bool oledOk = false;
uint8_t oledAddr = OLED_ADDR;   // runtime address (resolved by I2C scan)

// ---- rotary encoder object + ISR (knob adjusts speedPercent) ----
AiEsp32RotaryEncoder rotaryEncoder =
    AiEsp32RotaryEncoder(ENC_A_PIN, ENC_B_PIN, ENC_SW_PIN, -1, ENC_STEPS);
void IRAM_ATTR readEncoderISR() { rotaryEncoder.readEncoder_ISR(); }
long lastEncoderValue = 0;
unsigned long lastEncoderMoveMs = 0;      // for NVS settle + BLE lockout
unsigned long lastEncNotifyMs = 0;
bool encSpeedDirty = false;               // NVS write pending
bool encNotifyPending = false;            // BLE push pending
void pollRotaryEncoder();
#else
// No-op stub so the rest of the code compiles unchanged.
struct NoLed {
  uint32_t Color(uint8_t r, uint8_t g, uint8_t b) { return ((uint32_t)r << 16) | ((uint32_t)g << 8) | b; }
  void setPixelColor(int, uint32_t) {}
  void show() {}
  void begin() {}
  void clear() {}
};
NoLed led;
#endif
int ledPulseValue = 0;
int ledPulseDirection = 1;
unsigned long lastLedPulseTime = 0;
uint16_t greenDitherAcc = 0;

// ---------- OTA LED pulse task handle and function ------------
TaskHandle_t otaLedPulseTaskHandle = nullptr;
TaskHandle_t wifiScanTaskHandle = nullptr;
void otaLedPulseTask(void* parameter) {
  pwmValue = calculatePWM(ledBrightness);
  int otaLedPulse = 0;
  int otaLedDir = 1;
  uint16_t blueDitherAcc = 0;

  while (otaInProgress) {
    otaLedPulse += otaLedDir * 1;
    if (otaLedPulse >= LED_CONN_PULSE_MAX) {
      otaLedPulse = LED_CONN_PULSE_MAX;
      otaLedDir = -1;
    } else if (otaLedPulse <= 0) {
      otaLedPulse = 0;
      otaLedDir = 1;
    }

    setConnLed((uint8_t)otaLedPulse);

    uint16_t pulse255 = (uint16_t)((otaLedPulse * 255 + (LED_CONN_PULSE_MAX / 2)) / LED_CONN_PULSE_MAX);
    uint16_t scaled16 = (uint16_t)pulse255 * (uint16_t)pwmValue;
    uint8_t baseVal = scaled16 / 255;
    uint8_t remainder = scaled16 % 255;
    blueDitherAcc = (uint16_t)(blueDitherAcc + remainder);
    if (blueDitherAcc >= 255) {
      baseVal++;
      blueDitherAcc -= 255;
    }

    led.setPixelColor(0, led.Color(0, 0, baseVal));
    led.show();
    vTaskDelay(5 / portTICK_PERIOD_MS);
  }
  setConnLed(LED_CONN_PULSE_MAX);
  led.setPixelColor(0, 0);
  led.show();
  vTaskDelete(NULL);
}

void pollSerialCommands() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r') continue;

    if (c == '\n') {
      serialRxLine.trim();
      if (serialRxLine.length() > 0) {
        if (serialRxLine.length() > 512) {
          serialRxLine = "";
        } else {
          std::string cmd(serialRxLine.c_str());
          handleCommand(cmd);
          serialRxLine = "";
        }
      } else {
        serialRxLine = "";
      }
    } else {
      if (serialRxLine.length() < 512) serialRxLine += c;
    }
  }
}

void wifiScanTask(void* parameter) {
  WiFi.disconnect(true);
  int n = WiFi.scanNetworks();
  StaticJsonDocument<256> ssidDoc;
  JsonArray arr = ssidDoc.createNestedArray("SSID_LIST");
  for (int i = 0; i < n; i++) {
    int rssi = WiFi.RSSI(i);
    if (rssi > MIN_RSSI_THRESHOLD) {
      String ssid = WiFi.SSID(i);
      if (ssid.length() > 0 && ssid.length() < 33) {
        arr.add(ssid);
      }
    }
  }
  if (deviceConnected && pCharacteristic) {
    String jsonOut;
    serializeJson(ssidDoc, jsonOut);
    if (bleNotifyMutex) xSemaphoreTake(bleNotifyMutex, portMAX_DELAY);
    pCharacteristic->setValue(jsonOut.c_str());
    pCharacteristic->notify();
    if (bleNotifyMutex) xSemaphoreGive(bleNotifyMutex);
  }
  WiFi.scanDelete();
  prefs.begin("respooler", true);
  String ssid = prefs.getString("ssid", "");
  String pwd = prefs.getString("pwd", "");
  prefs.end();
  if (ssid.length() > 0 && pwd.length() > 0) {
    WiFi.begin(ssid.c_str(), pwd.c_str());
  }
  wifiScanTaskHandle = nullptr;
  vTaskDelete(NULL);
}

// -------------- Wi-Fi connect async status --------------
bool wifiConnectInProgress = false;
unsigned long wifiConnectStartTime = 0;

// ------------------ Helper and utility functions ---------------
int calculatePWM(int brightnessPercent) {
  float gamma = 1.15f;
  float n = (float)constrain(brightnessPercent, 0, 100) / 100.0f;
  return (int)round(powf(n, gamma) * 255.0f);
}
int fanPWMFromSpeed(int percent) {
  return map(constrain(percent, 0, 100), 0, 100, 0, 255);
}

inline void setServoAngle(int angle) {
  angle = constrain(angle, 0, 180);
  int pulseUs = map(angle, 0, 180, 500, 2500);
  uint32_t duty = (uint32_t)((((uint64_t)pulseUs) * ((1UL << 14) - 1) + 10000UL) / 20000UL);
  ledc_set_duty(LEDC_LOW_SPEED_MODE, SERVO_LEDC_CHANNEL, duty);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, SERVO_LEDC_CHANNEL);
}
inline void setConnLed(uint8_t duty) {
  duty = constrain(duty, 0, 255);
  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, 255 - duty);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
}
inline uint32_t stepFreqFromIntervalUs(unsigned long us) {
  if (us < 1) us = 1;
  return (uint32_t)(1000000UL / (2UL * us));
}
inline void stepperSetFreq(uint32_t freqHz) {
  if (freqHz < 1) freqHz = 1;
  ledc_set_freq(LEDC_LOW_SPEED_MODE, STEP_LEDC_TIMER, freqHz);
}
inline void stepperStart(uint32_t freqHz) {
  stepperSetFreq(freqHz);
  ledc_set_duty(LEDC_LOW_SPEED_MODE, STEP_LEDC_CHANNEL, STEP_DUTY_ON);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, STEP_LEDC_CHANNEL);
}
inline void stepperStop() {
  ledc_set_duty(LEDC_LOW_SPEED_MODE, STEP_LEDC_CHANNEL, 0);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, STEP_LEDC_CHANNEL);
}
int getSpeedPercentFromInterval(unsigned long us) {
  double t = (double)us;
  double p = 50.0 + ((170.0 - t) * 50.0 / (170.0 - 93.0));
  int percent = (int)round(p);
  return constrain(percent, 50, 100);
}

void applySpeedTarget() {
  if (highSpeedMode) {
    unsigned long adjusted = (unsigned long)round((double)baseTargetStepIntervalMicros / 1.4);
    if (adjusted < 50) adjusted = 50;
    targetStepIntervalMicros = adjusted;
  } else {
    targetStepIntervalMicros = baseTargetStepIntervalMicros;
  }
}

void loadSettings() {
  prefs.begin("respooler", true);
  boardVariant = prefs.getUChar("variant", 0);
  baseTargetStepIntervalMicros = prefs.getUInt("speed", DEFAULT_INTERVAL_US);
  speedPercent = getSpeedPercentFromInterval(baseTargetStepIntervalMicros);
  applySpeedTarget();
  motorDirection = prefs.getUInt("dir", 0);
  ledBrightness = prefs.getUInt("led", 50);
  useFilamentSensor = prefs.getBool("filamentSensor", true);
  motorStrength = prefs.getUInt("motorStrength", 100);
  torqueLimit = prefs.getUInt("torqueLimit", 0);
  highSpeedMode = prefs.getBool("hsMode", false);
  if (highSpeedMode) {
    savedTorqueLimitHS = torqueLimit;
    torqueLimit = 0;
  }
  jingleStyle = prefs.getUInt("jingle", 0);
  calibrationAt80Speed = prefs.getULong("cal80_093", 895000);
  fanSpeed = prefs.getUInt("fanSpeed", 60);
  fanAlwaysOn = prefs.getBool("fanAlways", false);
  targetWeight = prefs.getUInt("targetWeight", 0);
  otaUrl = prefs.getString("otaUrl", OTA_URL_DEFAULT);   // custom OTA source

  servoAngleR = (int)prefs.getUInt("servoR", 5);
  servoAngleL = (int)prefs.getUInt("servoL", 175);
  servoStepMm = prefs.getFloat("servoStep", 1.75f);
  servoHomeIsR = prefs.getBool("servoHomeR", true);

  servoAngleR = constrain(servoAngleR, 0, 180);
  servoAngleL = constrain(servoAngleL, 0, 180);
  servoStepMm = constrain(servoStepMm, 0.05f, 20.0f);

  applySpeedTarget();
  prefs.end();
  setMotorCurrent(motorStrength);
  pwmValue = calculatePWM(ledBrightness);
  digitalWrite(DIR_PIN, motorDirection);
}

void setMotorCurrent(int percent) {
  int strengthCurrent = map(percent, 80, 120, 700, 1400);
  driver.rms_current(strengthCurrent);
}

void reSanitizeDriver() {
  driver.GSTAT(0b111);
  driver.en_spreadCycle(false);
  driver.pwm_autoscale(true);
  driver.pwm_autograd(true);
  driver.microsteps(MICROSTEPPING);
  setMotorCurrent(motorStrength);
}

// ------------------- BLE status/error/settings -------------------
void sendStatus(bool forceSend = false) {
  if (!deviceConnected || !pCharacteristic) return;

  float progVal = progress;
  int remVal = 0;
  if (currentState == 'D') {
    progVal = 100.0f;
    remVal = 0;
  } else if (totalEstimatedTime > 0) {
    unsigned long elapsed;
    if (spoolingStartTime > 0) {
      elapsed = millis() - spoolingStartTime;
    } else if (pausedElapsed > 0) {
      elapsed = pausedElapsed;
    } else if (currentState == 'A') {
      elapsed = pausedElapsed;
    } else {
      elapsed = 0;
    }
    unsigned long effectiveTotal = totalEstimatedTime;
    if (targetWeight == 2) effectiveTotal = (unsigned long)(totalEstimatedTime * TARGET_WEIGHT_FACTOR_2);
    else if (targetWeight == 3) effectiveTotal = (unsigned long)(totalEstimatedTime * TARGET_WEIGHT_FACTOR_3);
    if (effectiveTotal == 0) {
      progVal = 0.0f;
      remVal = 0;
    } else if (elapsed >= effectiveTotal) {
      progVal = 100.0f;
      remVal = 0;
    } else {
      progVal = (100.0f * (float)elapsed) / (float)effectiveTotal;
      unsigned long remainingMs = effectiveTotal - elapsed;
      remVal = (int)(remainingMs / 1000UL);
    }
  }
  if (remHoldActive) {
    unsigned long nowMs = millis();
    if (nowMs < remHoldExpiry) {
      remVal = lastRemainingTime;
    } else {
      remHoldActive = false;
    }
  }
  if (progHoldActive) {
    unsigned long nowMs = millis();
    if (nowMs < progHoldExpiry) {
      progVal = lastProgress;
    } else {
      progHoldActive = false;
    }
  }
  if (currentState == 'R') {
    unsigned long nowMs = millis();
    if (nowMs < remStartHoldUntil) {
      remVal = lastRemainingTime;
    }
  }
  int chipTemp = (int)temperatureRead();

  StaticJsonDocument<640> doc;
  doc["STAT"] = String(currentState);
  doc["HAS_FIL"] = filamentDetected;
  doc["USE_FIL"] = useFilamentSensor;
  doc["PROG"] = progVal;
  doc["REM"] = remVal;
  doc["TEMP"] = chipTemp;
  doc["WIFI_SSID"] = wifiSSID.length() > 0 ? wifiSSID.c_str() : nullptr;
  doc["WIFI_OK"] = WiFi.status() == WL_CONNECTED;
  doc["FW"] = FIRMWARE_VERSION;
  doc["OTA_OK"] = nullptr;
  doc["SPD"] = speedPercent;
  doc["JIN"] = jingleStyle;
  doc["LED"] = ledBrightness;
  doc["DIR"] = motorDirection;
  doc["POW"] = motorStrength;
  doc["TRQ"] = highSpeedMode ? 0 : torqueLimit;
  doc["WGT"] = targetWeight;
  doc["DUR"] = calibrationAt80Speed / 1000;
  doc["HS"] = highSpeedMode;
  doc["FAN_SPD"] = fanSpeed;
  doc["FAN_ON"] = (fanAlwaysOn || isMotorRunning || millis() < fanStopAfter);
  doc["FAN_ALW"] = fanAlwaysOn;
  doc["SV_R"] = servoAngleR;
  doc["SV_L"] = servoAngleL;
  doc["SV_STP"] = servoStepMm;
  doc["SV_HOME"] = servoHomeIsR ? "R" : "L";
  doc["VAR"] = (boardVariant == 2) ? "PRO" : (boardVariant == 1) ? "STD"
                                                                 : "UNK";

  lastIsMotorRunning = isMotorRunning;
  lastFilamentDetected = filamentDetected;
  lastProgress = progVal;
  lastChipTemperature = chipTemp;
  lastRemainingTime = remVal;
  lastSpeedPercent = speedPercent;

  String jsonOut;
  serializeJson(doc, jsonOut);
  if (bleNotifyMutex) xSemaphoreTake(bleNotifyMutex, portMAX_DELAY);
  pCharacteristic->setValue(jsonOut.c_str());
  pCharacteristic->notify();
  if (bleNotifyMutex) xSemaphoreGive(bleNotifyMutex);
}

// ------------------- Stepper jingle/tones --------------------------
void playTone(unsigned int freqHz, unsigned long durationMs) {
  uint16_t prevCurrent = driver.rms_current();
  driver.rms_current(1400);
  unsigned long halfPeriod = 1000000UL / (2 * freqHz);
  unsigned long totalCycles = (durationMs * 1000UL) / (2 * halfPeriod);
  bool dirState = false;
  for (unsigned long i = 0; i < totalCycles; i++) {
    dirState = !dirState;
    digitalWrite(DIR_PIN, dirState);
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(40);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(halfPeriod * 2 - 40);
  }
  driver.rms_current(prevCurrent);
}
void playStepperJingle() {
  if (jingleStyle == 0) return;
  delay(500);
  stepperStop();
  ledc_stop(LEDC_LOW_SPEED_MODE, STEP_LEDC_CHANNEL, 0);
  gpio_reset_pin((gpio_num_t)STEP_PIN);
  gpio_set_direction((gpio_num_t)STEP_PIN, GPIO_MODE_OUTPUT);
  digitalWrite(EN_PIN, LOW);
  digitalWrite(STEP_PIN, LOW);
  {
    led.setPixelColor(0, led.Color(0, pwmValue, 0));
    led.show();
  }
  switch (jingleStyle) {
    case 1:  // Simple
      playTone(1000, 140);
      delay(20);
      playTone(800, 140);
      delay(20);
      playTone(1200, 400);
      break;
    case 2:  // Glissando
      playTone(523, 220);
      delay(20);
      playTone(587, 70);
      delay(20);
      playTone(659, 70);
      delay(20);
      playTone(698, 70);
      delay(20);
      playTone(784, 70);
      delay(20);
      playTone(880, 70);
      delay(20);
      playTone(988, 100);
      delay(20);
      playTone(1047, 400);
      break;
    case 3:  // Star Wars
      playTone(440, 400);
      delay(40);
      playTone(440, 400);
      delay(40);
      playTone(440, 400);
      delay(40);
      playTone(349, 250);
      delay(40);
      playTone(523, 150);
      delay(40);
      playTone(440, 400);
      delay(40);
      playTone(349, 250);
      delay(40);
      playTone(523, 150);
      delay(40);
      playTone(440, 500);
      break;
  }
  digitalWrite(EN_PIN, HIGH);
  {
    ledc_channel_config_t step_ledc_chan = {
      .gpio_num = STEP_PIN,
      .speed_mode = LEDC_LOW_SPEED_MODE,
      .channel = STEP_LEDC_CHANNEL,
      .intr_type = LEDC_INTR_DISABLE,
      .timer_sel = STEP_LEDC_TIMER,
      .duty = 0,
      .hpoint = 0
    };
    ledc_channel_config(&step_ledc_chan);
  }
  stepperStop();
}

// --------------------- BLE command handling -------------------------
void sendOTAUpdate() {
  if (otaInProgress) {
    return;
  }
  isMotorRunning = false;
  digitalWrite(STEP_PIN, LOW);
  stepperStop();
  digitalWrite(EN_PIN, HIGH);
  currentState = 'U';
  sendStatus(true);
  xTaskCreate([](void*) {
    // HTTP or HTTPS: pick the transport from the URL, so a plain
    // "python -m http.server" on the LAN can serve your own firmware.bin.
    bool isHttps = otaUrl.startsWith("https://");
    WiFiClient plainClient;
    WiFiClientSecure secureClient;
    WiFiClient* netClient = &plainClient;
    if (isHttps) {
      secureClient.setInsecure();
      netClient = &secureClient;
    }
    HTTPClient http;
    Serial.printf("[OTA] source: %s (%s)\n", otaUrl.c_str(),
                  isHttps ? "https" : "http");

    if (WiFi.status() != WL_CONNECTED || WiFi.localIP() == INADDR_NONE) {
      currentState = 'I';
      sendStatus(true);
      vTaskDelete(NULL);
    }

    otaInProgress = true;
    sendStatus(true);
    if (otaLedPulseTaskHandle == nullptr) {
      // 3072 instead of 1024: this task calls led.show(), which needs more
      // stack than a plain integer loop -- 1024 overflowed into a crash.
      xTaskCreate(otaLedPulseTask, "OtaLedPulse", 3072, NULL, 2, &otaLedPulseTaskHandle);
    }

    http.begin(*netClient, otaUrl);
    http.setTimeout(30000);      // 1.2 MB over a slow LAN needs more than 5 s
    http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
    int httpCode = http.GET();
    bool otaSuccess = false;

    if (httpCode == HTTP_CODE_OK) {
      int contentLength = http.getSize();
      if (Update.begin(contentLength)) {
        WiFiClient* stream = http.getStreamPtr();
        while (http.connected() && Update.writeStream(*stream) > 0) {
          delay(1);
        }
        if (Update.end() && Update.isFinished()) {
          otaSuccess = true;
          if (deviceConnected && pCharacteristic) {
            StaticJsonDocument<64> doc;
            doc["OTA_OK"] = true;
            delay(100);
            sendStatus(true);
            String jsonOut;
            serializeJson(doc, jsonOut);
            if (bleNotifyMutex) xSemaphoreTake(bleNotifyMutex, portMAX_DELAY);
            pCharacteristic->setValue(jsonOut.c_str());
            pCharacteristic->notify();
            if (bleNotifyMutex) xSemaphoreGive(bleNotifyMutex);
            NimBLEDevice::deinit(true);
            delay(1000);
          }
          otaInProgress = false;
          otaLedPulseTaskHandle = nullptr;
          ESP.restart();
        }
      } else {
        if (deviceConnected && pCharacteristic) {
          StaticJsonDocument<64> doc;
          doc["OTA_OK"] = false;
          String jsonOut;
          serializeJson(doc, jsonOut);
          if (bleNotifyMutex) xSemaphoreTake(bleNotifyMutex, portMAX_DELAY);
          pCharacteristic->setValue(jsonOut.c_str());
          pCharacteristic->notify();
          if (bleNotifyMutex) xSemaphoreGive(bleNotifyMutex);
        }
        otaInProgress = false;
        otaLedPulseTaskHandle = nullptr;
        currentState = 'I';
        sendStatus(true);
        vTaskDelete(NULL);
      }
    } else {
      if (deviceConnected && pCharacteristic) {
        StaticJsonDocument<64> doc;
        doc["OTA_OK"] = false;
        String jsonOut;
        serializeJson(doc, jsonOut);
        if (bleNotifyMutex) xSemaphoreTake(bleNotifyMutex, portMAX_DELAY);
        pCharacteristic->setValue(jsonOut.c_str());
        pCharacteristic->notify();
        if (bleNotifyMutex) xSemaphoreGive(bleNotifyMutex);
      }
      otaInProgress = false;
      otaLedPulseTaskHandle = nullptr;
      currentState = 'I';
      sendStatus(true);
      vTaskDelete(NULL);
    }
    http.end();
    if (!otaSuccess) {
      if (deviceConnected && pCharacteristic) {
        StaticJsonDocument<64> doc;
        doc["OTA_OK"] = false;
        String jsonOut;
        serializeJson(doc, jsonOut);
        if (bleNotifyMutex) xSemaphoreTake(bleNotifyMutex, portMAX_DELAY);
        pCharacteristic->setValue(jsonOut.c_str());
        pCharacteristic->notify();
        if (bleNotifyMutex) xSemaphoreGive(bleNotifyMutex);
      }
    }
    otaInProgress = false;
    otaLedPulseTaskHandle = nullptr;
    currentState = 'I';
    sendStatus(true);
    vTaskDelete(NULL);
  },
              "OTAUpdateTask", 8192, NULL, 1, NULL);
}

// ================= OTA over SoftAP (板子自己开热点) ==================
// Same idea as the badge project (ESP32-badge-wifi-bluetooth / lib/my_wifi):
// the board is its own AP + tiny HTTP server, so no router and no PC-side
// Python server is needed.
//   serial/BLE:  {"CMD":"OTA_AP"}     -> SSID "LTS-CB-OTA" / pass 12345678
//   browser:     http://192.168.4.1/  -> pick firmware.bin -> flashed
//   serial/BLE:  {"CMD":"OTA_AP_OFF"} -> back to normal STA mode
#define OTA_AP_SSID "LTS-CB-OTA2"      // renamed: forces phones to forget old cache
#define OTA_AP_PASS "12345678"          // WPA2 needs >= 8 chars
// 1 = open network (no password, only for diagnosing phone-side problems).
// Keep 0 for normal use: WPA2 with OTA_AP_PASS.
#define OTA_AP_OPEN 0
#define OTA_SLOT_SIZE 1310720UL         // app partition size (1.25 MB) -- hard cap
#define OTA_AP_TIMEOUT_MS (30UL * 60UL * 1000UL)   // auto-off after 30 min idle

static const char OTA_AP_PAGE[] =
    "<!DOCTYPE html><html><head><meta charset=utf-8>"
    "<meta name=viewport content='width=device-width,initial-scale=1'>"
    "<title>LTS Respooler OTA</title><style>"
    "body{background:#101725;color:#e8eefb;margin:0;padding:36px 18px;"
    "font:15px/1.6 -apple-system,'Segoe UI','Microsoft YaHei',sans-serif;"
    "display:flex;justify-content:center}"
    ".c{background:#18202f;border:1px solid #26324a;border-radius:14px;"
    "padding:22px;max-width:520px;width:100%}"
    "h1{font-size:18px;margin:0 0 6px}.dim{color:#8b9bb8;font-size:13px}"
    ".k{display:flex;justify-content:space-between;padding:6px 0;font-size:14px;"
    "border-bottom:1px dashed #26324a}.k b{font-family:Consolas,monospace;font-weight:600}"
    "#drop{border:2px dashed #26324a;border-radius:12px;padding:30px;text-align:center;"
    "color:#8b9bb8;margin:18px 0;cursor:pointer;display:block;position:relative}"
    "#drop.hot{border-color:#4da3ff;color:#e8eefb}"
    "#f{position:absolute;left:0;top:0;width:100%;height:100%;opacity:0;"
    "cursor:pointer;z-index:2}"
    "#pick{width:100%;margin-top:10px}"
    "button{background:#4da3ff;color:#06101f;border:0;border-radius:9px;"
    "padding:11px 18px;font-weight:700;font-size:14px}"
    "#bar{height:6px;background:#0d1522;border-radius:6px;overflow:hidden;"
    "margin-top:12px;display:none}#bar i{display:block;height:100%;width:0;"
    "background:#37d67a;transition:width .15s}#msg{margin-top:12px;font-size:13px}"
    "</style></head><body><div class=c>"
    "<h1>LTS Respooler · 无线刷机</h1>"
    "<div class=dim>选 .pio/build/esp32c3/firmware.bin 上传，约 1.2 MB</div>"
    "<div class=k><span>板子热点</span><b>LTS-CB-OTA2</b></div>"
    "<div class=k><span>本机地址</span><b>192.168.4.1</b></div>"
    "<label id=drop>点击这里选择 firmware.bin"
    "<div style='font-size:12px;margin-top:6px;color:#6b7c99'>"
    "手机看不到文件时，请在文件选择器里切到「文件 / 下载」分类</div>"
    "<input id=f type=file></label>"
    "<button type=button id=pick onclick=\"document.getElementById('f').click()\">"
    "选择 firmware.bin</button>"
    "<div id=bar><i></i></div><div id=msg></div>"
    "</div><script>"
    "var d=document.querySelector('#drop'),fi=document.querySelector('#f'),"
    "bar=document.querySelector('#bar'),msg=document.querySelector('#msg');"
    "fi.onchange=function(){up(fi.files[0])};"
    "['dragover','dragenter'].forEach(function(e){d.addEventListener(e,function(v){"
    "v.preventDefault();d.classList.add('hot')})});"
    "['dragleave','drop'].forEach(function(e){d.addEventListener(e,function(v){"
    "v.preventDefault();d.classList.remove('hot')})});"
    "d.addEventListener('drop',function(v){up(v.dataTransfer.files[0])});"
    "function up(f){if(!f)return;if(!/\\.bin$/i.test(f.name)){msg.textContent='只支持 .bin';return}"
    "if(f.size>1310720){msg.textContent='✗ '+f.size+' 字节，超过 1.25MB 分区，烧不进去';return}"
    "msg.textContent='上传中 '+f.name+' …';bar.style.display='block';"
    "var sent=false,x=new XMLHttpRequest();x.open('POST','/upload');"
    "x.upload.onprogress=function(e){if(e.lengthComputable)"
    "bar.firstElementChild.style.width=(e.loaded/e.total*100)+'%'};"
    "x.upload.onload=function(){sent=true;msg.textContent='数据已发完，板子写入中…'};"
    "x.onload=function(){msg.textContent=(x.status===200)?'✓ 成功，板子重启中…':('✗ '+x.responseText)};"
    "x.onerror=function(){msg.textContent=sent?"
    "'✓ 已发送完成，板子重启中（连接断开属正常）':'✗ 上传中断，请重试'};x.send(f)}"
    "</script></body></html>";

WiFiServer otaApServer(80);
WiFiUDP otaDnsServer;          // captive-portal DNS: every name -> 192.168.4.1
// Survives a software reboot (not a power cycle): prevents the idle-timeout
// restart from immediately re-opening the hotspot forever.
RTC_DATA_ATTR uint8_t otaAutoApDone = 0;
bool otaApOn = false;
bool otaApIsHotspot = false;
int otaApStations = 0;         // how many phones/PCs are on the hotspot
unsigned long otaApLastActivityMs = 0;

// Reply to any DNS query with 192.168.4.1 so Windows/Android/iOS think the
// hotspot needs a "login" and open the OTA page automatically.
void otaDnsProcess() {
  int len = otaDnsServer.parsePacket();
  if (len <= 0) return;
  uint8_t buf[512];
  int n = otaDnsServer.read(buf, sizeof(buf));
  if (n < 12) return;
  if (buf[2] & 0x80) return;                 // ignore responses

  int qEnd = 12;                             // skip header, walk QNAME
  while (qEnd < n) {
    uint8_t l = buf[qEnd];
    if (l == 0) { qEnd++; break; }
    if ((l & 0xC0) == 0xC0) { qEnd += 2; break; }
    qEnd += l + 1;
  }
  if (qEnd + 4 > n) return;
  qEnd += 4;                                 // QTYPE + QCLASS

  uint8_t resp[512];
  memcpy(resp, buf, qEnd);
  resp[2] = 0x81;                            // QR=1 RD=1
  resp[3] = 0x80;                            // RA=1
  resp[6] = 0x00; resp[7] = 0x01;            // ANCOUNT = 1
  resp[8] = 0x00; resp[9] = 0x00;
  resp[10] = 0x00; resp[11] = 0x00;
  int p = qEnd;
  resp[p++] = 0xC0; resp[p++] = 0x0C;        // name -> question
  resp[p++] = 0x00; resp[p++] = 0x01;        // TYPE A
  resp[p++] = 0x00; resp[p++] = 0x01;        // CLASS IN
  resp[p++] = 0x00; resp[p++] = 0x00;
  resp[p++] = 0x00; resp[p++] = 60;          // TTL 60s
  resp[p++] = 0x00; resp[p++] = 0x04;        // RDLENGTH
  resp[p++] = 192; resp[p++] = 168; resp[p++] = 4; resp[p++] = 1;

  otaDnsServer.beginPacket(otaDnsServer.remoteIP(), otaDnsServer.remotePort());
  otaDnsServer.write(resp, p);
  otaDnsServer.endPacket();
}

// Progress on the panel while the upload streams in.
void drawOtaProgress(int pct, size_t done, size_t total) {
  if (!oledOk) return;
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(1);
  oled.setCursor(0, 0);
  oled.println("OTA UPLOAD");
  oled.setCursor(0, 12);
  oled.printf("%u / %u KB", (unsigned)(done / 1024), (unsigned)(total / 1024));
  oled.setTextSize(2);
  oled.setCursor(0, 26);
  oled.printf("%3d%%", pct);
  oled.drawRect(0, 52, 128, 11, SSD1306_WHITE);
  int w = (int)(124L * pct / 100);
  if (w > 0) oled.fillRect(2, 54, w, 7, SSD1306_WHITE);
  oled.display();
}

void drawOtaDone(bool ok) {
  if (!oledOk) return;
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(2);
  oled.setCursor(0, 12);
  oled.println(ok ? "FLASHED" : "FAILED");
  oled.setTextSize(1);
  oled.setCursor(0, 40);
  oled.println(ok ? "rebooting..." : "check serial log");
  oled.display();
}

void stopOtaAp() {
  if (!otaApOn) return;
  if (otaApIsHotspot) otaDnsServer.stop();
  otaApServer.end();
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_STA);
  esp_coex_preference_set(ESP_COEX_PREFER_BT);   // hand the radio back to BLE
  otaApOn = false;
  otaApStations = 0;
  currentState = 'I';
  if (wifiSSID.length() > 0) WiFi.begin(wifiSSID.c_str(), wifiPassword.c_str());
  Serial.println("[OTA-AP] closed -- BLE is still up, no reboot needed");
  sendStatus(true);
}

void startOtaAp() {
  if (otaApOn) return;
  isMotorRunning = false;
  digitalWrite(STEP_PIN, LOW);
  stepperStop();
  digitalWrite(EN_PIN, HIGH);

  // Keep BLE alive! Turning the hotspot on must NOT kill Bluetooth, otherwise
  // the app can't connect while OTA mode is active. The C3 has one radio, so
  // just hand it to WiFi (AP beacons are the time-critical part); BLE keeps
  // advertising and reconnects whenever the radio is free.
  esp_coex_preference_set(ESP_COEX_PREFER_WIFI);
  WiFi.persistent(false);
  WiFi.disconnect(true);        // drop STA cleanly BEFORE switching mode
  WiFi.mode(WIFI_OFF);
  delay(300);
  WiFi.mode(WIFI_AP);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  delay(100);
  Serial.printf("[OTA-AP] AP up, BLE advertising=%d, heap=%u\n",
                NimBLEDevice::getAdvertising()->isAdvertising() ? 1 : 0,
                ESP.getFreeHeap());

  bool apOk;
#if OTA_AP_OPEN
  apOk = WiFi.softAP(OTA_AP_SSID, nullptr, 6, 0, 8);   // open, ch6, 8 clients
#else
  apOk = WiFi.softAP(OTA_AP_SSID, OTA_AP_PASS, 6, 0, 8);
#endif
  if (!apOk) {   // ch6, visible, 8 clients
    Serial.println("[OTA-AP] softAP failed, rebooting");
    delay(300);
    ESP.restart();
  }
  otaApServer.begin();
  otaApServer.setNoDelay(true);
  otaDnsServer.begin(53);        // captive portal: every DNS name -> us
  otaApOn = true;
  otaApIsHotspot = true;
  otaApLastActivityMs = millis();
  currentState = 'U';
  otaApStations = 0;
  updateOled();
  Serial.printf("[OTA-AP] SSID=%s PASS=%s ch=%d  open http://%s/  (30 min idle = auto off)\n",
                OTA_AP_SSID, OTA_AP_OPEN ? "(open)" : OTA_AP_PASS, 6,
                WiFi.softAPIP().toString().c_str());
}

// Serve the same page on the LAN when the board already joined a router:
// no hotspot involved, the PC keeps its internet connection.
void startOtaWeb() {
  if (otaApOn) return;
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[OTA-WEB] board is not on WiFi -- use OTA_AP instead");
    return;
  }
  isMotorRunning = false;
  digitalWrite(STEP_PIN, LOW);
  stepperStop();
  digitalWrite(EN_PIN, HIGH);
  otaApServer.begin();
  otaApServer.setNoDelay(true);
  otaApOn = true;
  otaApIsHotspot = false;
  otaApLastActivityMs = millis();
  currentState = 'U';
  otaApStations = 0;
  updateOled();
  Serial.printf("[OTA-WEB] open http://%s/  (30 min idle = auto off)\n",
                WiFi.localIP().toString().c_str());
}

// One pass of the tiny HTTP server: GET / -> page, POST /upload -> flash
void otaApHandleClient() {
  WiFiClient c = otaApServer.available();
  if (!c) return;
  otaApLastActivityMs = millis();

  unsigned long t0 = millis();
  while (!c.available() && millis() - t0 < 3000) delay(1);
  if (!c.available()) { c.stop(); return; }

  String reqLine = c.readStringUntil('\n');
  bool isUpload = reqLine.startsWith("POST /upload");
  int contentLen = -1;
  while (c.available() || millis() - t0 < 3000) {
    String h = c.readStringUntil('\n');
    h.trim();
    if (h.length() == 0) break;
    String hl = h;
    hl.toLowerCase();
    if (hl.startsWith("content-length:")) contentLen = hl.substring(15).toInt();
  }

  if (!isUpload) {
    c.print("HTTP/1.1 200 OK\r\nContent-Type: text/html; charset=utf-8\r\n");
    c.printf("Content-Length: %u\r\n", (unsigned)strlen(OTA_AP_PAGE));
    c.print("Cache-Control: no-store\r\nConnection: close\r\n\r\n");
    c.print(OTA_AP_PAGE);
    delay(5);
    c.stop();
    return;
  }

  Serial.printf("[OTA-AP] upload start, Content-Length=%d\n", contentLen);

  // ---- gate 1: size. The OTA slot is 1.25 MB; anything bigger can never fit.
  if (contentLen > 0 && (size_t)contentLen > OTA_SLOT_SIZE) {
    Serial.printf("[OTA-AP] rejected: %d B > slot %u B\n", contentLen,
                  (unsigned)OTA_SLOT_SIZE);
    c.printf("HTTP/1.1 413 Payload Too Large\r\nConnection: close\r\n\r\n"
             "file too big: %d B, max %u B", contentLen, (unsigned)OTA_SLOT_SIZE);
    drawOtaDone(false);
    delay(40);
    c.stop();
    return;
  }

  // ---- gate 2: image type. Read the first 64 bytes and check them BEFORE
  // touching flash, so a wrong file (partitions.bin / bootloader / zip) is
  // rejected instead of half-written and bricking the OTA slot.
  uint8_t head[64];
  size_t headLen = 0;
  unsigned long headT0 = millis();
  while (headLen < sizeof(head) && millis() - headT0 < 5000) {
    int n = c.read(head + headLen, sizeof(head) - headLen);
    if (n > 0) headLen += (size_t)n;
    else delay(1);
  }
  const char* reject = nullptr;
  if (headLen < 0x24) {
    reject = "file too short";
  } else if (head[0] != 0xE9) {
    reject = "not an ESP32 image (byte0 != 0xE9)";
  } else {
    uint32_t desc = (uint32_t)head[0x20] | ((uint32_t)head[0x21] << 8) |
                    ((uint32_t)head[0x22] << 16) | ((uint32_t)head[0x23] << 24);
    if (desc != 0xABCD5432u) reject = "not an app image (no app descriptor)";
  }
  if (reject) {
    Serial.printf("[OTA-AP] rejected: %s\n", reject);
    c.printf("HTTP/1.1 400 Bad Request\r\nConnection: close\r\n\r\n%s", reject);
    drawOtaDone(false);
    delay(40);
    c.stop();
    return;
  }
  Serial.printf("[OTA-AP] image OK: 0xE9 + app descriptor, %d B\n", contentLen);

  if (!Update.begin(contentLen > 0 ? (size_t)contentLen : UPDATE_SIZE_UNKNOWN)) {
    Update.printError(Serial);
    c.print("HTTP/1.1 500 Internal Server Error\r\nConnection: close\r\n\r\nUpdate.begin failed");
    delay(20);
    c.stop();
    return;
  }

  size_t written = 0;
  if (Update.write(head, headLen) != headLen) {   // flush the validated header
    Update.printError(Serial);
    c.print("HTTP/1.1 500 Internal Server Error\r\nConnection: close\r\n\r\nwrite failed");
    delay(20);
    c.stop();
    return;
  }
  written = headLen;

  unsigned long lastData = millis();
  uint8_t buf[1024];
  int lastPct = -1;
  drawOtaProgress(0, written, contentLen > 0 ? (size_t)contentLen : 0);
  while (contentLen < 0 || written < (size_t)contentLen) {
    int n = c.read(buf, sizeof(buf));
    if (n <= 0) {
      if (millis() - lastData > 8000) break;   // stalled client
      delay(1);
      continue;
    }
    lastData = millis();
    if (Update.write(buf, (size_t)n) != (size_t)n) {
      Update.printError(Serial);
      break;
    }
    written += (size_t)n;
    if (contentLen > 0) {
      // repaint at most every 250 ms: a full OLED frame costs ~90 ms on I2C,
      // repainting too often stretches the upload and starves the socket.
      static unsigned long lastDrawMs = 0;
      if (millis() - lastDrawMs > 250) {
        lastDrawMs = millis();
        int pct = (int)((written * 100ULL) / (unsigned long)contentLen);
        lastPct = pct;
        drawOtaProgress(pct, written, (size_t)contentLen);
      }
    }
  }

  bool ok = (written > 0) && Update.end(true);
  Serial.printf("[OTA-AP] upload %s, %u bytes\n", ok ? "OK" : "FAILED", (unsigned)written);
  drawOtaDone(ok);
  if (ok) {
    c.print("HTTP/1.1 200 OK\r\nContent-Type: text/plain\r\nConnection: close\r\n\r\nOK");
    c.flush();              // make sure the browser gets "OK" before we reset
    delay(1500);
    ESP.restart();
  } else {
    Update.printError(Serial);
    c.print("HTTP/1.1 500 Internal Server Error\r\nContent-Type: text/plain\r\n"
            "Connection: close\r\n\r\nflash write failed");
    delay(20);
    c.stop();
  }
}

void handleCommand(const std::string& cmd) {
  StaticJsonDocument<512> doc;
  DeserializationError error = deserializeJson(doc, cmd.c_str());
  if (error) return;

  if (doc.containsKey("CMD")) {
    String command = doc["CMD"].as<String>();
    if (command == "START") {
      if (!isMotorRunning) {
        shouldStartMotorNow = true;
        delayStartUntil = millis();
      }
    } else if (command == "STOP") {
      if (otaInProgress) {
        return;
      }
      if (currentState == 'A') {
        remHoldActive = true;
        remHoldExpiry = millis() + 200;
        progHoldActive = true;
        progHoldExpiry = millis() + 200;
        isMotorRunning = false;
        shouldStartMotorNow = false;
        pendingDirectionChange = false;
        triggerJingleNow = 0;
        progress = 0.0;
        totalEstimatedTime = 0;
        spoolingStartTime = 0;
        pausedElapsed = 0;
        filamentLostSince = 0;
        stepperStop();
        ledc_stop(LEDC_LOW_SPEED_MODE, STEP_LEDC_CHANNEL, 0);
        gpio_reset_pin((gpio_num_t)STEP_PIN);
        gpio_set_direction((gpio_num_t)STEP_PIN, GPIO_MODE_OUTPUT);
        digitalWrite(STEP_PIN, LOW);
        digitalWrite(EN_PIN, HIGH);
        {
          ledc_channel_config_t step_ledc_chan = {
            .gpio_num = STEP_PIN,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = STEP_LEDC_CHANNEL,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = STEP_LEDC_TIMER,
            .duty = 0,
            .hpoint = 0
          };
          ledc_channel_config(&step_ledc_chan);
        }
        currentState = 'I';
        sendStatus(true);
      } else {
        remHoldActive = true;
        remHoldExpiry = millis() + 200;
        progHoldActive = true;
        progHoldExpiry = millis() + 200;
        isMotorRunning = false;
        progress = 0.0;
        totalEstimatedTime = 0;
        spoolingStartTime = 0;
        pausedElapsed = 0;
        stepperStop();
        digitalWrite(EN_PIN, HIGH);
        digitalWrite(STEP_PIN, LOW);
        currentState = 'I';
        sendStatus(true);
      }
    } else if (command == "PAUSE") {
      if (isMotorRunning) {
        isMotorRunning = false;
        stepperStop();
        digitalWrite(EN_PIN, HIGH);
        digitalWrite(STEP_PIN, LOW);
        unsigned long elapsed = 0;
        if (spoolingStartTime > 0) {
          elapsed = millis() - spoolingStartTime;
        }
        pausedElapsed = elapsed;
        spoolingStartTime = 0;
        currentState = 'P';
        sendStatus(true);
      }
    } else if (command == "WIFI_SCAN") {
      if (wifiScanTaskHandle == nullptr) {
        xTaskCreate(wifiScanTask, "WiFiScan", 4096, NULL, 1, &wifiScanTaskHandle);
      }
    } else if (command == "OTA") {
      if (!otaInProgress) {
        sendOTAUpdate();
      }
    } else if (command == "OTA_AP") {
      startOtaAp();
    } else if (command == "OTA_WEB") {
      startOtaWeb();
    } else if (command == "OTA_AP_OFF") {
      stopOtaAp();
    } else if (command == "WIFI_CONNECT") {
      if (wifiSSID.length() > 0 && wifiPassword.length() > 0) {
        WiFi.disconnect(true);
        WiFi.begin(wifiSSID.c_str(), wifiPassword.c_str());
        wifiConnectInProgress = true;
        wifiConnectStartTime = millis();
      }
    }
  }
  if (doc.containsKey("SET")) {
    JsonObject set = doc["SET"].as<JsonObject>();
    bool changed = false;
    bool servoEndpointsChanged = false;
    if (set.containsKey("VAR")) {
      JsonVariant vj = set["VAR"];
      uint8_t newVar = boardVariant;
      if (vj.is<const char*>()) {
        const char* s = vj.as<const char*>();
        if (s && s[0]) {
          String up = String(s);
          up.toUpperCase();
          if (up == "PRO") newVar = 2;
          else if (up == "STD" || up == "STANDARD") newVar = 1;
          else if (up == "UNK" || up == "UNKNOWN") newVar = 0;
        }
      } else {
        int vi = vj.as<int>();
        if (vi < 0) vi = 0;
        if (vi > 2) vi = 2;
        newVar = (uint8_t)vi;
      }
      if (newVar != boardVariant) {
        boardVariant = newVar;
        changed = true;
      }
    }
    if (set.containsKey("DIR")) {
      int dir = set["DIR"];
      if (dir != motorDirection) {
        if (isMotorRunning) {
          pendingDirectionChange = true;
          newMotorDirection = dir;
          isMotorRunning = false;
          stepperStop();
          digitalWrite(EN_PIN, HIGH);
          digitalWrite(STEP_PIN, LOW);
        } else {
          motorDirection = dir;
          digitalWrite(DIR_PIN, motorDirection);
          prefs.begin("respooler", false);
          prefs.putUInt("dir", motorDirection);
          prefs.end();
        }
        changed = true;
      }
    }
    if (set.containsKey("LED")) {
      ledBrightness = constrain((int)set["LED"], 0, 100);
      pwmValue = calculatePWM(ledBrightness);
      prefs.begin("respooler", false);
      prefs.putUInt("led", ledBrightness);
      prefs.end();
      changed = true;
    }
    if (set.containsKey("USE_FIL")) {
      useFilamentSensor = (int)set["USE_FIL"] != 0;
      prefs.begin("respooler", false);
      prefs.putBool("filamentSensor", useFilamentSensor);
      prefs.end();
      changed = true;
    }
    if (set.containsKey("POW")) {
      motorStrength = constrain((int)set["POW"], 80, 120);
      setMotorCurrent(motorStrength);
      prefs.begin("respooler", false);
      prefs.putUInt("motorStrength", motorStrength);
      prefs.end();
      changed = true;
    }
    if (set.containsKey("TRQ")) {
      int newTrq = constrain((int)set["TRQ"], 0, 3);
      if (highSpeedMode) {
        savedTorqueLimitHS = newTrq;
        prefs.begin("respooler", false);
        prefs.putUInt("torqueLimit", savedTorqueLimitHS);
        prefs.end();
        torqueLimit = 0;
      } else {
        torqueLimit = newTrq;
        prefs.begin("respooler", false);
        prefs.putUInt("torqueLimit", torqueLimit);
        prefs.end();
      }
      changed = true;
    }
    if (set.containsKey("JIN")) {
      int newJingleStyle = constrain((int)set["JIN"], 0, 3);
      if (newJingleStyle != jingleStyle) {
        jingleStyle = newJingleStyle;
        prefs.begin("respooler", false);
        prefs.putUInt("jingle", jingleStyle);
        prefs.end();
        if (!isMotorRunning) triggerJingleNow = jingleStyle;
        changed = true;
      }
    }
    if (set.containsKey("DUR")) {
      unsigned long dur = (unsigned long)set["DUR"];
      if (dur >= 10 && dur <= 20000) {
        calibrationAt80Speed = dur * 1000UL;
        prefs.begin("respooler", false);
        prefs.putULong("cal80_093", calibrationAt80Speed);
        prefs.end();
        changed = true;
      }
    }
    if (set.containsKey("WGT")) {
      int newWgt = constrain((int)set["WGT"], 0, 3);
      targetWeight = newWgt;
      prefs.begin("respooler", false);
      prefs.putUInt("targetWeight", (unsigned int)targetWeight);
      prefs.end();
      changed = true;
    }
    if (set.containsKey("OTAURL")) {
      // Point OTA at your own build: {"set":{"OTAURL":"http://<pc-ip>:8000/firmware.bin"}}
      // Empty string restores the vendor default.
      String u = set["OTAURL"].as<String>();
      u.trim();
      if (u.length() == 0) {
        otaUrl = OTA_URL_DEFAULT;
      } else if (u.startsWith("http://") || u.startsWith("https://")) {
        otaUrl = u;
      }
      if (u.length() == 0 || u.startsWith("http")) {
        prefs.begin("respooler", false);
        prefs.putString("otaUrl", otaUrl);
        prefs.end();
        Serial.printf("[OTA] url now: %s\n", otaUrl.c_str());
        changed = true;
      }
    }
    if (set.containsKey("SPD")) {
      if (encoderHasPriority()) {
        // Knob was just turned: ignore the app's slider for a moment so the
        // hardware value wins. The phone still gets the knob value by notify.
        Serial.println("[BLE] SPD ignored (encoder has priority)");
      } else {
        int speed = constrain((int)set["SPD"], 50, 100);
        applySpeedPercent(speed);
        prefs.begin("respooler", false);
        prefs.putUInt("speed", baseTargetStepIntervalMicros);
        prefs.end();
        changed = true;
      }
    }
    if (set.containsKey("HS")) {
      bool newHS = (int)set["HS"] != 0;
      if (newHS != highSpeedMode) {
        if (newHS) {
          highSpeedMode = true;
          savedTorqueLimitHS = torqueLimit;
          torqueLimit = 0;
          applySpeedTarget();
          prefs.begin("respooler", false);
          prefs.putBool("hsMode", true);
          prefs.end();
        } else {
          highSpeedMode = false;
          torqueLimit = savedTorqueLimitHS;
          applySpeedTarget();
          prefs.begin("respooler", false);
          prefs.putBool("hsMode", false);
          prefs.putUInt("torqueLimit", torqueLimit);
          prefs.end();
        }
        changed = true;
      }
    }
    if (set.containsKey("FAN_SPD")) {
      fanSpeed = constrain((int)set["FAN_SPD"], 0, 100);
      prefs.begin("respooler", false);
      prefs.putUInt("fanSpeed", fanSpeed);
      prefs.end();
      changed = true;
    }
    if (set.containsKey("FAN_ALW")) {
      fanAlwaysOn = (int)set["FAN_ALW"] != 0;
      prefs.begin("respooler", false);
      prefs.putBool("fanAlways", fanAlwaysOn);
      prefs.end();
      changed = true;
    }

    if (set.containsKey("SV_R")) {
      int r = constrain((int)set["SV_R"], 0, 180);
      if (r != servoAngleR) {
        servoAngleR = r;
        prefs.begin("respooler", false);
        prefs.putUInt("servoR", (unsigned int)servoAngleR);
        prefs.end();
        servoEndpointsChanged = true;
        changed = true;
      }
      if (!isMotorRunning && (currentState == 'I' || currentState == 'D') && servoGotoSide == 'R') {
        servoGotoActive = true;
        servoHomingToHome = false;
        lastServoHomeStepTime = millis();
      }
    }
    if (set.containsKey("SV_L")) {
      int l = constrain((int)set["SV_L"], 0, 180);
      if (l != servoAngleL) {
        servoAngleL = l;
        prefs.begin("respooler", false);
        prefs.putUInt("servoL", (unsigned int)servoAngleL);
        prefs.end();
        servoEndpointsChanged = true;
        changed = true;
      }
      if (!isMotorRunning && (currentState == 'I' || currentState == 'D') && servoGotoSide == 'L') {
        servoGotoActive = true;
        servoHomingToHome = false;
        lastServoHomeStepTime = millis();
      }
    }
    if (set.containsKey("SV_STP")) {
      float s = (float)set["SV_STP"].as<float>();
      s = constrain(s, 0.05f, 20.0f);
      if (fabsf(s - servoStepMm) > 0.0001f) {
        servoStepMm = s;
        prefs.begin("respooler", false);
        prefs.putFloat("servoStep", servoStepMm);
        prefs.end();
        changed = true;
      }
    }
    if (set.containsKey("SV_HOME")) {
      const char* v = set["SV_HOME"].as<const char*>();
      char c = (v && v[0]) ? v[0] : 0;
      if (c >= 'a' && c <= 'z') c = (char)(c - 32);
      if (c == 'R' || c == 'L') {
        bool homeR = (c == 'R');
        if (homeR != servoHomeIsR) {
          servoHomeIsR = homeR;
          prefs.begin("respooler", false);
          prefs.putBool("servoHomeR", servoHomeIsR);
          prefs.end();
          servoEndpointsChanged = true;
          changed = true;
        }
      }
    }
    if (set.containsKey("SV_GOTO") && !isMotorRunning && (currentState == 'I' || currentState == 'D')) {
      const char* v = set["SV_GOTO"].as<const char*>();
      char c = (v && v[0]) ? v[0] : 0;
      if (c >= 'a' && c <= 'z') c = (char)(c - 32);

      if (c == 'L' || c == 'R' || c == 'H') {
        servoGotoSide = c;
        servoGotoActive = true;
        servoHomingToHome = false;
        lastServoHomeStepTime = millis();
      }
    }
    if (servoEndpointsChanged && !isMotorRunning && (currentState == 'I' || currentState == 'D') && !servoGotoActive && !(servoGotoSide == 'L' || servoGotoSide == 'R')) {
      guideServoTowardsL = servoHomeIsR;
      servoHomingToHome = true;
      lastServoHomeStepTime = millis();
    }
    if (set.containsKey("WIFI_SSID")) {
      wifiSSID = set["WIFI_SSID"].as<String>();
      changed = true;
    }
    if (set.containsKey("WIFI_PASS")) {
      wifiPassword = set["WIFI_PASS"].as<String>();
      changed = true;
    }
    if (changed) {
      prefs.begin("respooler", false);
      prefs.putString("ssid", wifiSSID);
      prefs.putString("pwd", wifiPassword);
      prefs.putUChar("variant", boardVariant);
      prefs.end();
      sendStatus(true);
    }
  }
}

// ------------------- BLE callback classes ------------------------
class MyServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* pServer) override {
    deviceConnected = true;
    if (pServer->getConnectedCount() < 2) {
      NimBLEDevice::startAdvertising();
    }
    xTaskCreate([](void*) {
      delay(1100);
      sendStatus(true);
      vTaskDelete(NULL);
    },
                "delayedSend", 2048, NULL, 1, NULL);
  }
  void onDisconnect(NimBLEServer* pServer) override {
    deviceConnected = pServer->getConnectedCount() > 0;
    if (pServer->getConnectedCount() < 2) {
      NimBLEDevice::startAdvertising();
    }
  }
};
class MyCharacteristicCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* pChar) override {
    std::string rx = pChar->getValue();
    if (!rx.empty()) handleCommand(rx);
  }
};

// -------------------------- Setup --------------------------------
void setup() {
  Serial.begin(115200);
  Serial.println("[BOOT] 1 setup start");
  // A flash/reset-button boot (ESP_RST_EXT) means "user wants to reach the
  // board again" -> re-arm the automatic OTA hotspot. Only the software reboot
  // after the hotspot idle-timeout keeps the flag, so we don't loop.
  if (esp_reset_reason() == ESP_RST_EXT || esp_reset_reason() == ESP_RST_POWERON) {
    otaAutoApDone = 0;
  }
  Serial.printf("[BOOT] reset=%d autoApDone=%d wifiSSID='%s'\n",
                (int)esp_reset_reason(), (int)otaAutoApDone, wifiSSID.c_str());
  // NOTE: esp_bt_controller_mem_release() must NOT be called before
  // NimBLEDevice::init() on Arduino core 2.x -- releasing the controller
  // memory first makes the stack fail to init and the board dies silently.
  esp_coex_preference_set(ESP_COEX_PREFER_BT);
  pinMode(FILAMENT_PIN, INPUT_PULLUP);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, HIGH);
  pinMode(LED_PIN, OUTPUT);
  pinMode(LED_CONN_PIN, OUTPUT);
  pinMode(FAN_PIN, OUTPUT);
  Serial.println("[BOOT] 2 gpio done");

  // ---- rotary encoder (speed knob) -------------------------------
  rotaryEncoder.begin();
  rotaryEncoder.setup(readEncoderISR);          // ISR reads A/B transitions
  rotaryEncoder.setBoundaries(-100000, 100000, false);  // no wrap-around
  rotaryEncoder.setAcceleration(ENC_ACCEL);     // fast spin = bigger jumps
  rotaryEncoder.setEncoderValue(0);
  lastEncoderValue = 0;
  Serial.println("[BOOT] 2c rotary encoder ok");
  // NOTE: OLED init moved to AFTER BLE startup (end of setup) -- it used to
  // sit here and BLE connect was unreliable. BLE comes up first now.

  // ---- OLED init: exact copy of the WORKING oled_diag sequence ------------
  // The diag firmware always finds the panel because it SCANS for the address
  // (some modules answer on 0x3D, not 0x3C) and it waits 2s for the panel POR.
  // Do the same here instead of trusting a hardcoded 0x3C.
  delay(1500);                                  // panel power-on-reset
  Wire.begin(OLED_SDA, OLED_SCL);
  Wire.setClock(100000);
  int nFound = 0;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.printf("[OLED] scan 0x%02X ACK\n", a);
      nFound++;
      if (a == 0x3C || a == 0x3D) oledAddr = a;   // remember display address
    }
  }
  if (nFound == 0) Serial.println("[OLED] scan: NOTHING on the bus!");
  Serial.printf("[OLED] using addr 0x%02X\n", oledAddr);

  oledOk = oled.begin(SSD1306_SWITCHCAPVCC, oledAddr, true, false);
  if (oledOk) {
    oled.clearDisplay();
    oled.setTextSize(1);
    oled.setTextColor(SSD1306_WHITE);
    oled.setCursor(0, 0);
    oled.println("LTS Respooler");
    oled.println("pre-BLE frame");
    oled.display();
    Serial.println("[BOOT] 2b oled ok (pre-BLE draw)");
  } else {
    Serial.println("[BOOT] 2b OLED init FAILED (even pre-BLE)");
  }

  // Probe helper: ACK check on whatever address we resolved
  auto probeI2c = [](const char* tag) {
    Wire.beginTransmission(oledAddr);
    bool ack = (Wire.endTransmission() == 0);
    Serial.printf("[PROBE] %s : 0x%02X %s\n", tag, oledAddr, ack ? "ACK" : "no-ack");
  };
  probeI2c("right after oled init");

  ledc_timer_config_t servo_timer = {
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .duty_resolution = LEDC_TIMER_14_BIT,
    .timer_num = SERVO_LEDC_TIMER,
    .freq_hz = 50,
    .clk_cfg = LEDC_AUTO_CLK
  };
  ledc_timer_config(&servo_timer);

  ledc_channel_config_t servo_channel = {
    .gpio_num = SERVO_PIN,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = SERVO_LEDC_CHANNEL,
    .intr_type = LEDC_INTR_DISABLE,
    .timer_sel = SERVO_LEDC_TIMER,
    .duty = 0,
    .hpoint = 0
  };
  ledc_channel_config(&servo_channel);

  ledc_timer_config_t timer_conf = {
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .duty_resolution = LEDC_TIMER_8_BIT,
    .timer_num = FAN_TIMER,
    .freq_hz = 35000,
    .clk_cfg = LEDC_AUTO_CLK
  };
  ledc_timer_config(&timer_conf);

  ledc_channel_config_t channel_conf = {
    .gpio_num = FAN_PIN,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = FAN_CHANNEL,
    .intr_type = LEDC_INTR_DISABLE,
    .timer_sel = FAN_TIMER,
    .duty = 0,
    .hpoint = 0
  };
  ledc_channel_config(&channel_conf);

  ledc_timer_config_t conn_led_timer = {
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .duty_resolution = LEDC_TIMER_8_BIT,
    .timer_num = LEDC_TIMER_0,
    .freq_hz = 1000,
    .clk_cfg = LEDC_AUTO_CLK
  };
  ledc_timer_config(&conn_led_timer);

  ledc_channel_config_t conn_led_conf = {
    .gpio_num = LED_CONN_PIN,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = LEDC_CHANNEL_2,
    .intr_type = LEDC_INTR_DISABLE,
    .timer_sel = LEDC_TIMER_0,
    .duty = 0,
    .hpoint = 0
  };
  ledc_channel_config(&conn_led_conf);

  ledc_timer_config_t step_ledc_timer = {
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .duty_resolution = LEDC_TIMER_10_BIT,
    .timer_num = STEP_LEDC_TIMER,
    .freq_hz = 1000,
    .clk_cfg = LEDC_AUTO_CLK
  };
  ledc_timer_config(&step_ledc_timer);

  ledc_channel_config_t step_ledc_chan = {
    .gpio_num = STEP_PIN,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = STEP_LEDC_CHANNEL,
    .intr_type = LEDC_INTR_DISABLE,
    .timer_sel = STEP_LEDC_TIMER,
    .duty = 0,
    .hpoint = 0
  };
  ledc_channel_config(&step_ledc_chan);

  Serial.println("[BOOT] 3 ledc done");
  led.begin();
  led.clear();
  led.show();
  Serial.println("[BOOT] 4 neopixel done");

  TMCSerial.begin(115200, SERIAL_8N1, TMC_UART_RX, TMC_UART_TX);
  Serial.println("[BOOT] 5 TMC uart done");

  driver.begin();
  driver.toff(3);
  driver.microsteps(MICROSTEPPING);
  driver.en_spreadCycle(false);
  driver.pwm_autoscale(true);
  driver.pwm_autograd(true);
  driver.SGTHRS(40);
  driver.semin(0);
  driver.TCOOLTHRS(0xFFFFF);
  uint16_t cool = driver.COOLCONF();
  driver.COOLCONF(cool | (1 << 13));
  Serial.println("[BOOT] 6 TMC driver configured");

  prefs.begin("respooler", true);
  wifiSSID = prefs.getString("ssid", "");
  wifiPassword = prefs.getString("pwd", "");
  prefs.end();
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true);

  if (wifiSSID.length() > 0 && wifiPassword.length() > 0) {
    WiFi.begin(wifiSSID.c_str(), wifiPassword.c_str());
  }
  bleNotifyMutex = xSemaphoreCreateMutex();
  loadSettings();
  setMotorCurrent(motorStrength);
  pwmValue = calculatePWM(ledBrightness);

  guideServoPos = (float)(servoHomeIsR ? servoAngleR : servoAngleL);
  guideServoTowardsL = servoHomeIsR;
  setServoAngle((int)lroundf(guideServoPos));

  Serial.println("[BOOT] wifi+settings done, init BLE...");
  NimBLEDevice::init(BOARD_NAME);
  Serial.println("[BOOT] NimBLEDevice::init ok");
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);
  // NOTE: do NOT call setMTU() here. On ESP32-C3 (400KB RAM) NimBLE 1.x
  // cannot allocate the 512-byte MTU buffers; the alloc fails, the stack ends
  // up half-initialised and the device never advertises. 247 is plenty here.
  NimBLEServer* pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  NimBLEService* pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY);
  pCharacteristic->setCallbacks(new MyCharacteristicCallbacks());
  pService->start();

  NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
  // Keep the advertising packet short. BLE limits the primary advert payload to
  // 31 bytes -- packing name + UUID + manufacturer data into ONE packet
  // overruns it, and NimBLE silently emits a malformed packet that phones
  // refuse to show. Standard practice: name in the advert, the rest in the
  // scan response.
  NimBLEAdvertisementData advData;
  advData.setName(BOARD_NAME);
  // The web app (control.lts-design.com) uses Web Bluetooth with
  // filters:[{services:[SERVICE_UUID]}] -- Chrome only lists devices whose
  // ADVERTISED payload contains the service UUID. nRF shows anything, but
  // the web chooser matches on UUID, so the UUID MUST be in the adv packet.
  // Budget: flags(3) + name(8) + 128-bit UUID(18) = 29 <= 31. No room for
  // addTxPower() (3 more bytes would overflow) -> omit it.
  advData.setCompleteServices(NimBLEUUID(SERVICE_UUID));

  // Manufacturer-specific data payload (board variant: 01=STD, 02=PRO).
  // NOTE: NimBLE 1.x prepends the 0xFF AD-type byte itself -- payload must
  // NOT include leading 0xFF or the packet is malformed.
  std::string manufacturerData;
  if (boardVariant == 2) {
    manufacturerData = "\x02";   // PRO
  } else {
    manufacturerData = "\x01";   // STD
  }

  NimBLEAdvertisementData scanData;
  scanData.setManufacturerData(manufacturerData);

  pAdvertising->setAdvertisementData(advData);
  pAdvertising->setScanResponseData(scanData);
  // Fast interval (30-60ms) so phone scanners latch on quickly.
  pAdvertising->setMinInterval(30);
  pAdvertising->setMaxInterval(60);
  bool advStarted = pAdvertising->start();
  if (!advStarted) {
    Serial.println("[BLE] start() failed, retrying once...");
    delay(200);
    advStarted = pAdvertising->start();
  }
  Serial.printf("[BLE] name=%s advStarted=%d variant=%u mfgByte=%02X\n",
                BOARD_NAME, advStarted ? 1 : 0,
                (unsigned)boardVariant,
                (unsigned char)manufacturerData[0]);
  Serial.printf("[BLE] advPayloadLen=%d scanPayloadLen=%d (BLE limit is 31)\n",
                (int)advData.getPayload().length(),
                (int)scanData.getPayload().length());
  // Print the BLE public identity so we can match it in a scanner.
  {
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_BT);
    Serial.printf("[BLE] BT MAC %02X:%02X:%02X:%02X:%02X:%02X  heap=%u\n",
                  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5],
                  (unsigned)ESP.getFreeHeap());
  }
  // (OLED init now at top of setup -- pre-BLE draw bisect; updateOled()
  //  re-runs begin() every refresh, so the panel self-heals if disturbed)

  ledc_set_duty(LEDC_LOW_SPEED_MODE, FAN_CHANNEL, 0);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, FAN_CHANNEL);
  setConnLed(150);
  currentState = 'I';

  // ---- Hold the button while powering on -> OTA hotspot ----
  // So you never need a serial cable or the app: power on with the knob/button
  // pressed for >1.2 s and the board boots straight into OTA mode.
  if (digitalRead(BUTTON_PIN) == LOW) {
    Serial.println("[BOOT] button held at power-on, waiting...");
    unsigned long held = millis();
    while (digitalRead(BUTTON_PIN) == LOW && millis() - held < 3000) delay(20);
    if (millis() - held >= 1200) {
      Serial.println("[BOOT] -> OTA AP mode (release the button now)");
      startOtaAp();
      return;
    }
    Serial.println("[BOOT] released early, normal boot");
  }
}

// ---------------- Rotary encoder -> speed -------------------------
// The knob is the authoritative speed input: it applies immediately, pushes
// the new value to the phone, and briefly blocks BLE "SPD" writes so the app
// slider cannot stomp what you just dialled in.
void applySpeedPercent(int pct) {
  speedPercent = constrain(pct, 50, 100);
  double span = 170.0 - 93.0;
  double baseInterval = 170.0 - ((double)(speedPercent - 50) * span / 50.0);
  baseTargetStepIntervalMicros = (unsigned long)lround(baseInterval);
  applySpeedTarget();
}

void pollRotaryEncoder() {
  if (!rotaryEncoder.encoderChanged()) return;
  long v = rotaryEncoder.readEncoder();
  long delta = v - lastEncoderValue;
  lastEncoderValue = v;
  if (delta == 0) return;

  // one detent = 1 %; spinning fast makes delta jump (library acceleration),
  // so scale the step but keep it inside +/-ENC_MAX_JUMP.
  long ad = delta < 0 ? -delta : delta;
  int inc = 1 + (int)(ad / 4);
  if (inc > ENC_MAX_JUMP) inc = ENC_MAX_JUMP;
  int newSpeed = constrain(speedPercent + (delta > 0 ? inc : -inc), 50, 100);
  if (newSpeed == speedPercent) return;

  applySpeedPercent(newSpeed);
  lastEncoderMoveMs = millis();
  encSpeedDirty = true;
  encNotifyPending = true;
}

// Debounced flash write + BLE push (called from loop)
void flushRotaryEncoder() {
  unsigned long now = millis();
  if (encNotifyPending && now - lastEncNotifyMs > 150) {
    encNotifyPending = false;
    lastEncNotifyMs = now;
    sendStatus(true);
  }
  if (encSpeedDirty && now - lastEncoderMoveMs > ENC_NVS_SETTLE_MS) {
    encSpeedDirty = false;
    prefs.begin("respooler", false);
    prefs.putUInt("speed", baseTargetStepIntervalMicros);
    prefs.end();
  }
}

// true while the knob outranks the phone: incoming BLE "SPD" is ignored
bool encoderHasPriority() {
  return lastEncoderMoveMs > 0 &&
         (millis() - lastEncoderMoveMs < ENC_BLE_LOCKOUT_MS);
}

// ------------------- Main loop -----------------------------------
// ---------------- OLED refresh (non-blocking, called from loop) ---------------
void updateOled() {
  if (!oledOk) return;
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);

  // Line 1: state + progress %
  oled.setTextSize(1);
  oled.setCursor(0, 0);
  const char* st = "IDLE";
  switch (currentState) {
    case 'R': st = "RUNNING";   break;
    case 'P': st = "PAUSED";    break;
    case 'A': st = "AUTO-STOP"; break;
    case 'D': st = "FIL-OUT";   break;
    case 'U': st = "OTA";       break;
  }
  if (isMotorRunning) st = "RUNNING";
  if (otaApOn) st = otaApIsHotspot ? "OTA-AP" : "OTA-WEB";
  oled.print(st);
  oled.setCursor(OLED_W - 42, 0);
  oled.printf("%3.0f%%", progress);

  // Line 2: remaining time (mirrors web REM) + speed
  unsigned long elapsed = 0;
  if (isMotorRunning && spoolingStartTime > 0) elapsed = millis() - spoolingStartTime;
  else elapsed = pausedElapsed;
  unsigned long rem = 0;
  if (totalEstimatedTime > elapsed) rem = (totalEstimatedTime - elapsed) / 1000;
  oled.setTextSize(2);
  oled.setCursor(0, 14);
  oled.printf("%02lu:%02lu", rem / 60, rem % 60);
  oled.setTextSize(1);
  oled.setCursor(OLED_W - 54, 20);
  oled.printf("SPD %d%%", speedPercent);

  // divider
  oled.drawLine(0, 32, OLED_W - 1, 32, SSD1306_WHITE);

  // Bottom: filament + wifi
  oled.setCursor(0, 38);
  oled.print("FIL ");
  if (!useFilamentSensor) oled.print("OFF");
  else oled.print(filamentDetected ? "OK" : "NONE");
  oled.setCursor(0, 50);
  // Bottom-left WiFi status: "WIFI --" when idle, IP when joined to a router,
  // "WIFI Connected" while the OTA hotspot has a client.
  oled.print("WIFI ");
  if (otaApOn) {
    oled.print(otaApStations > 0 ? "Connected" : "--");
  } else if (WiFi.status() == WL_CONNECTED) {
    oled.print(WiFi.localIP());
  } else {
    oled.print("--");
  }

  oled.display();
}

void loop() {
  unsigned long now = millis();

  // Not on a router? Open the OTA hotspot by itself 6 s after power-up, so a
  // freshly flashed board is always reachable over WiFi without a serial cable.
  // RTC flag stops an endless hot-spot loop after the idle timeout reboots us.
  static bool autoApChecked = false;
  if (!autoApChecked && now > 6000) {
    autoApChecked = true;
    if (!otaApOn && WiFi.status() != WL_CONNECTED && !otaAutoApDone) {
      otaAutoApDone = 1;
      Serial.println("[OTA] not connected to WiFi -> opening OTA hotspot");
      startOtaAp();
    }
  }
  // heartbeat: prove the main loop keeps running
  static uint32_t lastBeat = 0;
  if (now - lastBeat > 5000) {
    lastBeat = now;
    Serial.printf("[LOOP] alive state=%c heap=%u\n", currentState, (unsigned)ESP.getFreeHeap());
  }

  // OLED refresh every 1000ms (a full frame @100kHz blocks loop ~90ms,
  // so 1Hz keeps the loop free while the display stays lively enough)
  static uint32_t lastOled = 0;
  if (now - lastOled > 1000) {
    lastOled = now;
    updateOled();
  }
  pollSerialCommands();

  // rotary encoder: poll every loop (cheap) + debounced BLE/flash flush
  pollRotaryEncoder();
  flushRotaryEncoder();

  // OTA-over-SoftAP: serve the upload page while the hotspot is on
  if (otaApOn) {
    otaApHandleClient();
    if (otaApIsHotspot) {
      otaDnsProcess();   // captive portal: auto-open page
      // refresh "Connected" as soon as a client joins or leaves
      static unsigned long lastStaPoll = 0;
      if (millis() - lastStaPoll > 500) {
        lastStaPoll = millis();
        int n = WiFi.softAPgetStationNum();
        if (n != otaApStations) {
          otaApStations = n;
          Serial.printf("[OTA-AP] clients = %d\n", n);
          updateOled();
        }
      }
    }
    if (millis() - otaApLastActivityMs > OTA_AP_TIMEOUT_MS) {
      Serial.println("[OTA-AP] idle timeout");
      stopOtaAp();
    }
  }

  // (debug I2C probe removed -- root cause was SDA/SCL wired swapped)

  // ------------------ Wi-Fi connect async check -----------------
  if (wifiConnectInProgress) {
    if (WiFi.status() == WL_CONNECTED) {
      wifiConnectInProgress = false;
      if (deviceConnected && pCharacteristic) {
        StaticJsonDocument<64> doc;
        doc["WIFI_CONN_RESULT"] = true;
        String jsonOut;
        serializeJson(doc, jsonOut);
        if (bleNotifyMutex) xSemaphoreTake(bleNotifyMutex, portMAX_DELAY);
        pCharacteristic->setValue(jsonOut.c_str());
        pCharacteristic->notify();
        if (bleNotifyMutex) xSemaphoreGive(bleNotifyMutex);
      }
      prefs.begin("respooler", false);
      prefs.putString("ssid", wifiSSID);
      prefs.putString("pwd", wifiPassword);
      prefs.end();
    } else if (millis() - wifiConnectStartTime > 10000) {
      wifiConnectInProgress = false;
      if (deviceConnected && pCharacteristic) {
        StaticJsonDocument<64> doc;
        doc["WIFI_CONN_RESULT"] = false;
        String jsonOut;
        serializeJson(doc, jsonOut);
        if (bleNotifyMutex) xSemaphoreTake(bleNotifyMutex, portMAX_DELAY);
        pCharacteristic->setValue(jsonOut.c_str());
        pCharacteristic->notify();
        if (bleNotifyMutex) xSemaphoreGive(bleNotifyMutex);
      }
    }
  }

  // ------------------ Filament sensor -----------------
  bool rawFilamentPresent = digitalRead(FILAMENT_PIN) == LOW;
  if (rawFilamentPresent) {
    filamentLostSince = 0;
    filamentDetected = true;
  } else {
    if (filamentLostSince == 0) {
      filamentLostSince = now;
    }
    if (filamentDetected && (now - filamentLostSince >= FILAMENT_LOSS_CONFIRM_MS)) {
      filamentDetected = false;
    }
  }

  if (useFilamentSensor && (currentState == 'P' || currentState == 'A') && !filamentDetected) {
    isMotorRunning = false;
    shouldStartMotorNow = false;
    pendingDirectionChange = false;
    triggerJingleNow = 0;
    remHoldActive = true;
    remHoldExpiry = millis() + 200;
    progHoldActive = true;
    progHoldExpiry = millis() + 200;
    progress = 0.0;
    totalEstimatedTime = 0;
    spoolingStartTime = 0;
    pausedElapsed = 0;
    stepperStop();
    digitalWrite(EN_PIN, HIGH);
    digitalWrite(STEP_PIN, LOW);
    currentState = 'I';
    sendStatus(true);
  }

  // ------------------ Motor/LED state -----------------
  if (!otaInProgress) {
    if (isMotorRunning) {
      if (now - lastLedPulseTime > 5) {
        lastLedPulseTime = now;
        ledPulseValue += ledPulseDirection * 1;
        if (ledPulseValue >= 255) {
          ledPulseValue = 255;
          ledPulseDirection = -1;
        } else if (ledPulseValue <= 0) {
          ledPulseValue = 0;
          ledPulseDirection = 1;
        }
        uint16_t scaled16 = (uint16_t)ledPulseValue * (uint16_t)pwmValue;
        uint8_t baseVal = scaled16 / 255;
        uint8_t remainder = scaled16 % 255;
        greenDitherAcc = (uint16_t)(greenDitherAcc + remainder);
        if (greenDitherAcc >= 255) {
          baseVal++;
          greenDitherAcc -= 255;
        }

        led.setPixelColor(0, led.Color(0, baseVal, 0));
        led.show();
      }
    } else if (currentState == 'P') {
      if (now - lastLedPulseTime > 5) {
        lastLedPulseTime = now;
        ledPulseValue += ledPulseDirection * 1;
        if (ledPulseValue >= 255) {
          ledPulseValue = 255;
          ledPulseDirection = -1;
        } else if (ledPulseValue <= 0) {
          ledPulseValue = 0;
          ledPulseDirection = 1;
        }
        uint16_t scaled16 = (uint16_t)ledPulseValue * (uint16_t)pwmValue;
        uint8_t baseVal = scaled16 / 255;
        uint8_t remainder = scaled16 % 255;
        greenDitherAcc = (uint16_t)(greenDitherAcc + remainder);
        if (greenDitherAcc >= 255) {
          baseVal++;
          greenDitherAcc -= 255;
        }
        led.setPixelColor(0, led.Color(baseVal, baseVal / 2, 0));
        led.show();
      }
    } else if (currentState == 'A') {
      if (millis() - lastLedToggle > 400) {
        lastLedToggle = now;
        ledState = !ledState;
        led.setPixelColor(0, ledState ? led.Color(pwmValue, 0, 0) : 0);
        led.show();
      }
    } else if (useFilamentSensor && filamentDetected) {
      led.setPixelColor(0, led.Color(0, pwmValue, 0));
      led.show();
    } else {
      led.setPixelColor(0, led.Color(0, 0, pwmValue));
      led.show();
    }
  }

  // ----------------- Torque Auto-Stop -----------------
  static unsigned long lastTorqueCheck = 0;
  static unsigned long torqueBelowStartTime = 0;
  if (torqueLimit > 0 && isMotorRunning && stepIntervalMicros == targetStepIntervalMicros) {
    if (now - lastTorqueCheck >= TORQUE_CHECK_INTERVAL_MS) {
      lastTorqueCheck = now;
      uint16_t sg = driver.SG_RESULT();
      if (sg > TORQUE_SG_IGNORE) {

        float strengthScale = 1.0;
        int baseLimit = 350 * strengthScale;
        int limit = 999;
        if (torqueLimit == 1) limit = baseLimit * TORQUE_SG_LIMIT_LOW;
        else if (torqueLimit == 2) limit = baseLimit * TORQUE_SG_LIMIT_MED;
        else if (torqueLimit == 3) limit = baseLimit * TORQUE_SG_LIMIT_HIGH;

        int currSpeedPercent = getSpeedPercentFromInterval(stepIntervalMicros);
        if (currSpeedPercent > 94) {
          limit = limit * 0.95;
        } else if (currSpeedPercent > 85) {
          limit = limit * 0.97;
        } else if (currSpeedPercent < 65) {
          limit = limit * 0.98;
        }

        if (sg < limit) {
          if (torqueBelowStartTime == 0) torqueBelowStartTime = now;
          else if (now - torqueBelowStartTime >= TORQUE_BELOW_MS) {
            isMotorRunning = false;
            currentState = 'A';
            sendStatus(true);
            digitalWrite(STEP_PIN, LOW);
            delay(100);
            digitalWrite(EN_PIN, HIGH);
            unsigned long elapsed = millis() - spoolingStartTime;
            spoolingStartTime = 0;
            pausedElapsed = elapsed;
            torqueBelowStartTime = 0;
            stepperStop();
            ledc_stop(LEDC_LOW_SPEED_MODE, STEP_LEDC_CHANNEL, 0);
            gpio_reset_pin((gpio_num_t)STEP_PIN);
            gpio_set_direction((gpio_num_t)STEP_PIN, GPIO_MODE_OUTPUT);
            digitalWrite(EN_PIN, LOW);
            digitalWrite(STEP_PIN, LOW);
            delay(120);
            {
              led.setPixelColor(0, led.Color(pwmValue, 0, 0));
              led.show();
            }
            for (int i = 0; i < 3; i++) {
              playTone(850, 250);
              delay(100);
            }
            digitalWrite(EN_PIN, HIGH);
            {
              ledc_channel_config_t step_ledc_chan = {
                .gpio_num = STEP_PIN,
                .speed_mode = LEDC_LOW_SPEED_MODE,
                .channel = STEP_LEDC_CHANNEL,
                .intr_type = LEDC_INTR_DISABLE,
                .timer_sel = STEP_LEDC_TIMER,
                .duty = 0,
                .hpoint = 0
              };
              ledc_channel_config(&step_ledc_chan);
            }
            stepperStop();
          }
        } else {
          torqueBelowStartTime = 0;
        }
      }
    }
  } else {
    torqueBelowStartTime = 0;
  }

  static unsigned long wgtFinishStart = 0;
  if (isMotorRunning && targetWeight >= 1 && targetWeight <= 3 && totalEstimatedTime > 0) {
    unsigned long elapsed = 0;
    if (spoolingStartTime > 0) elapsed = now - spoolingStartTime;
    else if (pausedElapsed > 0) elapsed = pausedElapsed;
    else if (currentState == 'A') elapsed = pausedElapsed;

    unsigned long effectiveTotal = totalEstimatedTime;
    if (targetWeight == 2) effectiveTotal = (unsigned long)(totalEstimatedTime * TARGET_WEIGHT_FACTOR_2);
    else if (targetWeight == 3) effectiveTotal = (unsigned long)(totalEstimatedTime * TARGET_WEIGHT_FACTOR_3);

    if (effectiveTotal == 0) {
      wgtFinishStart = 0;
    } else if (elapsed >= effectiveTotal) {
      if (!(useFilamentSensor && !filamentDetected)) {
        if (wgtFinishStart == 0) wgtFinishStart = now;
        if (now - wgtFinishStart >= 700) {
          isMotorRunning = false;
          digitalWrite(STEP_PIN, LOW);
          stepperStop();
          digitalWrite(EN_PIN, HIGH);
          totalEstimatedTime = 0;
          spoolingStartTime = 0;
          currentState = 'D';
          sendStatus(true);
          progress = 100.0f;
          sendStatus(true);
          doneHoldActive = true;
          doneHoldStart = millis();
          playStepperJingle();
          wgtFinishStart = 0;
        }
      } else {
        wgtFinishStart = 0;
      }
    } else {
      wgtFinishStart = 0;
    }
  }


  if (isMotorRunning && useFilamentSensor && !filamentDetected && filamentLostSince > 0 && now - filamentLostSince > FILAMENT_LOSS_CONFIRM_MS) {
    isMotorRunning = false;
    digitalWrite(STEP_PIN, LOW);
    stepperStop();
    delay(40);
    digitalWrite(EN_PIN, HIGH);
    totalEstimatedTime = 0;
    spoolingStartTime = 0;
    currentState = 'D';
    sendStatus(true);
    progress = 100.0f;
    sendStatus(true);
    doneHoldActive = true;
    doneHoldStart = millis();
    delay(300);
    playStepperJingle();
  }
  if (doneHoldActive && (millis() - doneHoldStart >= 20000)) {
    progress = 0.0;
    currentState = 'I';
    sendStatus(true);
    doneHoldActive = false;
  }

  // ------- Direction change after stop -----------
  if (pendingDirectionChange && !isMotorRunning) {
    motorDirection = newMotorDirection;
    prefs.begin("respooler", false);
    prefs.putUInt("dir", motorDirection);
    prefs.end();
    digitalWrite(DIR_PIN, motorDirection);
    delayStartUntil = now + 1000;
    shouldStartMotorNow = true;
    pendingDirectionChange = false;
  }

  // ------------ Motor start logic ----------------
  if (shouldStartMotorNow && now >= delayStartUntil) {
    if (otaInProgress) {
      shouldStartMotorNow = false;
      return;
    }
    if (isMotorRunning) {
      shouldStartMotorNow = false;
    } else if (!useFilamentSensor || filamentDetected) {
      int currSpeedPercent = round((float)speedPercent * ((float)baseTargetStepIntervalMicros / (float)targetStepIntervalMicros));
      currSpeedPercent = constrain(currSpeedPercent, 50, 140);
      totalEstimatedTime = calibrationAt80Speed * (80.0 / currSpeedPercent);
      if (pausedElapsed > 0) {
        spoolingStartTime = millis() - pausedElapsed;
        pausedElapsed = 0;
      } else {
        spoolingStartTime = millis();
      }
      currentState = 'R';
      sendStatus(true);
      remStartHoldUntil = millis() + 800;
      doneHoldActive = false;
      stepperStop();
      reSanitizeDriver();
      digitalWrite(DIR_PIN, motorDirection);
      digitalWrite(EN_PIN, LOW);
      digitalWrite(STEP_PIN, LOW);
      delay(15);
      {
        uint16_t prevI = driver.rms_current();
        driver.en_spreadCycle(true);
        int boosted = prevI + prevI / 4;
        if (boosted > 1600) boosted = 1600;
        driver.rms_current(boosted);
        stepperStart(stepFreqFromIntervalUs(1800));
        delay(150);
        driver.rms_current(prevI);
        driver.en_spreadCycle(false);
      }
      stepIntervalMicros = START_INTERVAL_US;
      lastAccelUpdate = now;
      isMotorRunning = true;
      stepperStart(stepFreqFromIntervalUs(stepIntervalMicros));
    }
    shouldStartMotorNow = false;
  }

  // ------------ Button handling with debounce ------------
  bool reading = digitalRead(BUTTON_PIN);
  if (reading != lastStableButtonState && now - lastDebounceTime > BUTTON_DEBOUNCE_MS) {
    lastDebounceTime = now;

    if (reading == LOW) {
      if (!isMotorRunning) {
        if (currentState == 'P' || currentState == 'A') {
          buttonPressSince = now;
          buttonLongPressHandled = false;
          buttonAwaitingDecision = true;
        } else if (useFilamentSensor && !filamentDetected) {
          if (!otaInProgress) {
            for (int i = 0; i < 5; i++) {
              led.setPixelColor(0, led.Color(0, 0, pwmValue));
              led.show();
              delay(100);
              led.setPixelColor(0, 0);
              led.show();
              delay(100);
            }
          }
        } else {
          shouldStartMotorNow = true;
          delayStartUntil = now;
        }
      } else {
        isMotorRunning = false;
        stepperStop();
        digitalWrite(EN_PIN, HIGH);
        digitalWrite(STEP_PIN, LOW);
        unsigned long elapsed = 0;
        if (spoolingStartTime > 0) {
          elapsed = millis() - spoolingStartTime;
        }
        pausedElapsed = elapsed;
        spoolingStartTime = 0;
        currentState = 'P';
        sendStatus(true);
      }
    } else {
      if (buttonAwaitingDecision) {
        if (!buttonLongPressHandled) {
          if (!(useFilamentSensor && !filamentDetected)) {
            shouldStartMotorNow = true;
            delayStartUntil = now;
          }
        }
        buttonAwaitingDecision = false;
      }
    }
    lastStableButtonState = reading;
  }
  if (buttonAwaitingDecision && !buttonLongPressHandled && (digitalRead(BUTTON_PIN) == LOW)) {
    if (now - buttonPressSince >= 1000) {
      remHoldActive = true;
      remHoldExpiry = millis() + 200;
      progHoldActive = true;
      progHoldExpiry = millis() + 200;
      isMotorRunning = false;
      progress = 0.0;
      totalEstimatedTime = 0;
      spoolingStartTime = 0;
      pausedElapsed = 0;
      stepperStop();
      digitalWrite(EN_PIN, HIGH);
      digitalWrite(STEP_PIN, LOW);
      currentState = 'I';
      sendStatus(true);
      buttonLongPressHandled = true;
    }
  }

  // ----------- Motor acceleration (speed ramp) --------------
  if (isMotorRunning && now - lastAccelUpdate > ACCEL_UPDATE_INTERVAL) {
    lastAccelUpdate = now;
    if (stepIntervalMicros > targetStepIntervalMicros) {
      stepIntervalMicros -= ACCEL_STEP;
      if (stepIntervalMicros < targetStepIntervalMicros) stepIntervalMicros = targetStepIntervalMicros;
    } else if (stepIntervalMicros < targetStepIntervalMicros) {
      stepIntervalMicros += ACCEL_STEP;
      if (stepIntervalMicros > targetStepIntervalMicros) stepIntervalMicros = targetStepIntervalMicros;
    }
    if (stepIntervalMicros != lastIntervalMicros) {
      stepperSetFreq(stepFreqFromIntervalUs(stepIntervalMicros));
      lastIntervalMicros = stepIntervalMicros;
    }
    if (stepIntervalMicros == targetStepIntervalMicros) {
      int currSpeedPercent = round((float)speedPercent * ((float)baseTargetStepIntervalMicros / (float)stepIntervalMicros));
      currSpeedPercent = constrain(currSpeedPercent, 50, 140);
      unsigned long newTotal = calibrationAt80Speed * (80.0 / currSpeedPercent);
      unsigned long elapsed = millis() - spoolingStartTime;
      if (totalEstimatedTime > 0) {
        float progressRatio = (float)elapsed / totalEstimatedTime;
        spoolingStartTime = millis() - (newTotal * progressRatio);
      }
      totalEstimatedTime = newTotal;
    }
  }

  // ------------- Connection LED animation ------------------
  if (!otaInProgress) {
    static int connLedPulse = 0;
    static int connLedDir = 2;

    if (!deviceConnected) {
      if (now - lastConnLedTime > 25) {
        lastConnLedTime = now;
        connLedPulse += connLedDir;
        if (connLedPulse >= LED_CONN_PULSE_MAX) {
          connLedPulse = LED_CONN_PULSE_MAX;
          connLedDir = -2;
        } else if (connLedPulse <= 0) {
          connLedPulse = 0;
          connLedDir = 2;
        }
        setConnLed(connLedPulse);
      }
    } else {
      ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, LED_CONN_PULSE_MAX);
      ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
    }
  }

  // ---------------------- Jingle trigger ---------------------
  if (triggerJingleNow != 0) {
    playStepperJingle();
    triggerJingleNow = 0;
  }

  // ----------- Restart BLE advertising if disconnected --------------
  if (!deviceConnected && !NimBLEDevice::getAdvertising()->isAdvertising()) {
    NimBLEDevice::startAdvertising();
  }

  // ------------------ Guide servo movement -----------------
  if (isMotorRunning) {
    if (!servoWasRunning) {
      servoWasRunning = true;
      lastMotorStepAccTime = now;
      servoHomingToHome = false;
      servoGotoActive = false;
    } else {
      unsigned long dtMs = now - lastMotorStepAccTime;
      if (dtMs > 0) {
        lastMotorStepAccTime = now;

        float stepsPerSec = (float)stepFreqFromIntervalUs(stepIntervalMicros);
        motorStepAcc += stepsPerSec * ((float)dtMs / 1000.0f);
        float stepsPerSpoolRev = (200.0f * (float)MICROSTEPPING) * (52.0f / 18.0f);

        while (motorStepAcc >= stepsPerSpoolRev) {
          motorStepAcc -= stepsPerSpoolRev;

          float target = guideServoTowardsL ? (float)servoAngleL : (float)servoAngleR;
          float stepDeg = servoStepMm * (360.0f / (35.0f * 3.77f));
          if (stepDeg < 0.05f) stepDeg = 0.05f;

          if (fabsf(guideServoPos - target) <= stepDeg) {
            guideServoPos = target;
            setServoAngle((int)lroundf(guideServoPos));
            guideServoTowardsL = !guideServoTowardsL;
          } else {
            guideServoPos += (guideServoPos < target) ? stepDeg : -stepDeg;
            setServoAngle((int)lroundf(guideServoPos));
          }
        }
      }
    }
  } else {
    servoWasRunning = false;
  }

  if (currentState == 'I' && lastStateForServo != 'I') {
    guideServoTowardsL = servoHomeIsR;
    motorStepAcc = 0.0f;
    servoWasRunning = false;
    servoHomingToHome = true;
    lastServoHomeStepTime = now;
  }
  lastStateForServo = currentState;
  
  // ------------------ Respooler done ------------------
  if (currentState == 'D' && lastStateForDone != 'D') {
    servoHomeAfterDonePending = true;
  }
  if (currentState != 'D' && lastStateForDone == 'D') {
    servoHomeAfterDonePending = false;
  }
  lastStateForDone = currentState;

  if (servoHomeAfterDonePending && currentState == 'D' && !isMotorRunning) {
    guideServoTowardsL = servoHomeIsR;
    servoHomingToHome = true;
    lastServoHomeStepTime = now;
    servoHomeAfterDonePending = false;
  }

  // ------------------ Servo goto ------------------
  if (!isMotorRunning && servoGotoActive) {
    if (now - lastServoHomeStepTime >= 10) {
      lastServoHomeStepTime = now;

      float target;
      if (servoGotoSide == 'L') target = (float)servoAngleL;
      else if (servoGotoSide == 'R') target = (float)servoAngleR;
      else if (servoGotoSide == 'H') target = (float)(servoHomeIsR ? servoAngleR : servoAngleL);
      else {
        servoGotoActive = false;
        return;
      }

      float stepDeg = 1.0f;

      if (fabsf(guideServoPos - target) <= stepDeg) {
        guideServoPos = target;
        setServoAngle((int)lroundf(guideServoPos));
        servoGotoActive = false;
      } else {
        guideServoPos += (guideServoPos < target) ? stepDeg : -stepDeg;
        setServoAngle((int)lroundf(guideServoPos));
      }
    }
  }

  // ------------------ Servo homing ------------------
  if (!isMotorRunning && servoHomingToHome) {
    if (now - lastServoHomeStepTime >= 10) {
      lastServoHomeStepTime = now;

      float targetHome = (float)(servoHomeIsR ? servoAngleR : servoAngleL);
      float stepDeg = 1.0f;

      if (fabsf(guideServoPos - targetHome) <= stepDeg) {
        guideServoPos = targetHome;
        setServoAngle((int)lroundf(guideServoPos));
        servoHomingToHome = false;
      } else {
        guideServoPos += (guideServoPos < targetHome) ? stepDeg : -stepDeg;
        setServoAngle((int)lroundf(guideServoPos));
      }
    }
  }

  // ---------------- Fan control logic ----------------
  int fanPWM = (fanAlwaysOn || isMotorRunning || millis() < fanStopAfter) ? fanPWMFromSpeed(fanSpeed) : 0;
  ledc_set_duty(LEDC_LOW_SPEED_MODE, FAN_CHANNEL, fanPWM);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, FAN_CHANNEL);
  if (!isMotorRunning && fanPWM > 0 && fanStopAfter == 0) {
    fanStopAfter = millis() + 10000;
  }
  if (isMotorRunning) fanStopAfter = millis() + 10000;
  if (!fanAlwaysOn && !isMotorRunning && millis() >= fanStopAfter) {
    fanStopAfter = 0;
  }

  // ----------- Periodic Status-Update every 500 ms -----------
  static unsigned long lastStatusTime = 0;
  if (millis() - lastStatusTime > 500) {
    sendStatus();
    lastStatusTime = millis();
  }
}
