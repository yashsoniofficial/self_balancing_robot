// motor_controller.ino
// ESP32 / Arduino sketch
// Protocol (serial):
// - MCU -> PC: CSV line: t,ax,ay,az,gx,gy,gz,encL,encR
// - PC -> MCU: "M,<left_pwm>,<right_pwm>\n" (left/right -127..127 signed)

#include <Wire.h>
#include "MPU6050.h"
#include "ESP32Encoder.h"

// ----- CONFIG -----
const int PWM_LEFT_PIN = 18;   // PWM PWM channel pins
const int DIR_LEFT_PIN  = 19;
const int PWM_RIGHT_PIN = 21;
const int DIR_RIGHT_PIN = 22;

const int ENC_LEFT_A = 34;
const int ENC_LEFT_B = 35;
const int ENC_RIGHT_A = 32;
const int ENC_RIGHT_B = 33;

MPU6050 imu;
ESP32Encoder encL;
ESP32Encoder encR;

// PWM channels for ESP32 LedC
const int LEFT_CH = 0;
const int RIGHT_CH = 1;
const int PWM_FREQ = 20000;
const int PWM_RES = 8; // 8-bit

long lastPrint = 0;
char inBuf[64];

void setup() {
  Serial.begin(115200);
  delay(100);

  // IMU init
  Wire.begin();
  imu.initialize();
  if (!imu.testConnection()) {
    Serial.println("IMU FAIL");
  }

  // encoders
  encL.attachFullQuad(ENC_LEFT_A, ENC_LEFT_B);
  encR.attachFullQuad(ENC_RIGHT_A, ENC_RIGHT_B);

  // pwm
  ledcSetup(LEFT_CH, PWM_FREQ, PWM_RES);
  ledcSetup(RIGHT_CH, PWM_FREQ, PWM_RES);
  ledcAttachPin(PWM_LEFT_PIN, LEFT_CH);
  ledcAttachPin(PWM_RIGHT_PIN, RIGHT_CH);
  pinMode(DIR_LEFT_PIN, OUTPUT);
  pinMode(DIR_RIGHT_PIN, OUTPUT);
}

void applyMotor(int pwm, int ch, int dirPin) {
  if (pwm >= 0) { digitalWrite(dirPin, HIGH); ledcWrite(ch, pwm); }
  else { digitalWrite(dirPin, LOW); ledcWrite(ch, -pwm); }
}

void loop() {
  // read IMU
  int16_t ax, ay, az, gx, gy, gz;
  imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  long encLCount = encL.getCount();
  long encRCount = encR.getCount();

  unsigned long t = millis();
  // print CSV
  Serial.print(t); Serial.print(',');
  Serial.print(ax); Serial.print(',');
  Serial.print(ay); Serial.print(',');
  Serial.print(az); Serial.print(',');
  Serial.print(gx); Serial.print(',');
  Serial.print(gy); Serial.print(',');
  Serial.print(gz); Serial.print(',');
  Serial.print(encLCount); Serial.print(',');
  Serial.println(encRCount);

  // check incoming commands
  if (Serial.available()) {
    size_t len = Serial.readBytesUntil('\n', inBuf, sizeof(inBuf)-1);
    inBuf[len] = 0;
    if (len > 0) {
      if (inBuf[0] == 'M' && inBuf[1] == ',') {
        // parse "M,<l>,<r>"
        int l=0,r=0;
        char *p = inBuf+2;
        l = atoi(p);
        char *comma = strchr(p, ',');
        if (comma) r = atoi(comma+1);
        // constrain -127..127 -> map to 8-bit PWM 0..255
        l = max(-127, min(127, l));
        r = max(-127, min(127, r));
        int pwmL = map(abs(l), 0, 127, 0, 255);
        int pwmR = map(abs(r), 0, 127, 0, 255);
        applyMotor((l>=0)?pwmL:-pwmL, LEFT_CH, DIR_LEFT_PIN);
        applyMotor((r>=0)?pwmR:-pwmR, RIGHT_CH, DIR_RIGHT_PIN);
      }
    }
  }

  delay(5); // ~200 Hz telemetry
}
