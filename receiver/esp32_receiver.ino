#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>
#include <ESP32Servo.h>

// Wi‑Fi credentials must match the CyberPi controller
const char *ssid = "cyberpi-rc";
const char *password = "drivefast";

// UDP settings
const uint16_t localPort = 4210;
WiFiUDP udp;

// Select your output mode:
// - Set USE_RC_OUTPUTS = true to drive standard hobby gear: a steering servo + an ESC/servo for throttle.
// - Set USE_RC_OUTPUTS = false to drive two DC motors with a dual H-bridge (original mode).
const bool USE_RC_OUTPUTS = true;

// RC outputs (used when USE_RC_OUTPUTS is true)
const int SERVO_PIN = 12;
const int ESC_PIN = 13;
const int SERVO_MIN_US = 1000;   // Typical servo range
const int SERVO_MAX_US = 2000;
const int ESC_MIN_US = 1000;     // Calibrate to your ESC if needed
const int ESC_MAX_US = 2000;
const int ESC_NEUTRAL_US = 1500; // Many car ESCs are bidirectional with 1500us as neutral

Servo steeringServo;
Servo escOutput;

// Motor pins (used when USE_RC_OUTPUTS is false)
const int LEFT_IN1 = 25;
const int LEFT_IN2 = 26;
const int RIGHT_IN1 = 27;
const int RIGHT_IN2 = 14;

// PWM configuration
const int PWM_FREQ = 20000;       // 20 kHz for silent drive
const int PWM_RES = 10;           // 10-bit resolution
const int CHANNEL_LEFT_FWD = 0;
const int CHANNEL_LEFT_REV = 1;
const int CHANNEL_RIGHT_FWD = 2;
const int CHANNEL_RIGHT_REV = 3;

// Failsafe
const unsigned long FAILSAFE_MS = 500;
unsigned long lastPacketMs = 0;

int mapFloatToUs(float value, int minUs, int maxUs) {
  // value is expected in [-1, 1]
  float halfRange = (maxUs - minUs) / 2.0f;
  return (int)(minUs + halfRange + (value * halfRange));
}

float clamp(float v, float minV, float maxV) {
  if (v < minV) return minV;
  if (v > maxV) return maxV;
  return v;
}

void stopMotors() {
  if (USE_RC_OUTPUTS) {
    escOutput.writeMicroseconds(ESC_NEUTRAL_US);
    steeringServo.writeMicroseconds(mapFloatToUs(0.0f, SERVO_MIN_US, SERVO_MAX_US));
  } else {
    ledcWrite(CHANNEL_LEFT_FWD, 0);
    ledcWrite(CHANNEL_LEFT_REV, 0);
    ledcWrite(CHANNEL_RIGHT_FWD, 0);
    ledcWrite(CHANNEL_RIGHT_REV, 0);
  }
}

void setupMotorPins() {
  if (USE_RC_OUTPUTS) {
    // 50 Hz is standard for hobby servos/ESCs
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    steeringServo.setPeriodHertz(50);
    escOutput.setPeriodHertz(50);
    steeringServo.attach(SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);
    escOutput.attach(ESC_PIN, ESC_MIN_US, ESC_MAX_US);
  } else {
    pinMode(LEFT_IN1, OUTPUT);
    pinMode(LEFT_IN2, OUTPUT);
    pinMode(RIGHT_IN1, OUTPUT);
    pinMode(RIGHT_IN2, OUTPUT);

    ledcSetup(CHANNEL_LEFT_FWD, PWM_FREQ, PWM_RES);
    ledcSetup(CHANNEL_LEFT_REV, PWM_FREQ, PWM_RES);
    ledcSetup(CHANNEL_RIGHT_FWD, PWM_FREQ, PWM_RES);
    ledcSetup(CHANNEL_RIGHT_REV, PWM_FREQ, PWM_RES);

    ledcAttachPin(LEFT_IN1, CHANNEL_LEFT_FWD);
    ledcAttachPin(LEFT_IN2, CHANNEL_LEFT_REV);
    ledcAttachPin(RIGHT_IN1, CHANNEL_RIGHT_FWD);
    ledcAttachPin(RIGHT_IN2, CHANNEL_RIGHT_REV);
  }

  stopMotors();
}

void connectWifi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  Serial.printf("Connecting to %s", ssid);
  int retries = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    if (++retries > 30) {
      Serial.println("\nRetrying...");
      WiFi.disconnect();
      WiFi.begin(ssid, password);
      retries = 0;
    }
  }
  WiFi.setAutoReconnect(true);
  WiFi.config(IPAddress(192, 168, 4, 2), IPAddress(192, 168, 4, 1), IPAddress(255, 255, 255, 0));
  Serial.printf("\nConnected! IP: %s\n", WiFi.localIP().toString().c_str());
}

void setup() {
  Serial.begin(115200);
  setupMotorPins();
  connectWifi();

  udp.begin(localPort);
  Serial.printf("Listening for UDP on port %d\n", localPort);
}

void driveRC(float throttle, float steering) {
  // Steering servo: -1 = full left, 1 = full right
  int steerUs = mapFloatToUs(clamp(steering, -1.0f, 1.0f), SERVO_MIN_US, SERVO_MAX_US);
  steeringServo.writeMicroseconds(steerUs);

  // ESC: -1 = full reverse, 0 = neutral, 1 = full forward
  int throttleUs = mapFloatToUs(clamp(throttle, -1.0f, 1.0f), ESC_MIN_US, ESC_MAX_US);
  escOutput.writeMicroseconds(throttleUs);
}

void driveDifferential(float throttle, float steering) {
  // Differential drive: left = throttle - steering, right = throttle + steering
  float left = clamp(throttle - steering, -1.0f, 1.0f);
  float right = clamp(throttle + steering, -1.0f, 1.0f);

  auto writeMotor = [](int fwdChan, int revChan, float value) {
    int duty = (int)(fabs(value) * ((1 << PWM_RES) - 1));
    if (value >= 0) {
      ledcWrite(fwdChan, duty);
      ledcWrite(revChan, 0);
    } else {
      ledcWrite(fwdChan, 0);
      ledcWrite(revChan, duty);
    }
  };

  writeMotor(CHANNEL_LEFT_FWD, CHANNEL_LEFT_REV, left);
  writeMotor(CHANNEL_RIGHT_FWD, CHANNEL_RIGHT_REV, right);
}

void loop() {
  int packetSize = udp.parsePacket();
  if (packetSize) {
    StaticJsonDocument<192> doc;
    char buffer[192];
    int len = udp.read(buffer, sizeof(buffer) - 1);
    if (len > 0) {
      buffer[len] = '\0';
      DeserializationError err = deserializeJson(doc, buffer);
      if (!err) {
        float throttle = doc["throttle"] | 0.0f;
        float steering = doc["steering"] | 0.0f;
        float mix = doc["mix"] | 1.0f;            // optional channel mixing coefficient
        int auxA = doc["aux_a"] | 0;               // optional aux channels from CyberPi buttons
        int auxB = doc["aux_b"] | 0;
        throttle = clamp(throttle, -1.0f, 1.0f);
        steering = clamp(steering * clamp(mix, 0.0f, 2.0f), -1.0f, 1.0f);
        if (USE_RC_OUTPUTS) {
          driveRC(throttle, steering);
        } else {
          driveDifferential(throttle, steering);
        }

        // Expose aux values for future expansion (e.g. spare PWM outputs)
        (void)auxA;
        (void)auxB;
        lastPacketMs = millis();
      }
    }
  }

  // Failsafe: stop if no packets recently
  if (millis() - lastPacketMs > FAILSAFE_MS) {
    stopMotors();
  }
}
