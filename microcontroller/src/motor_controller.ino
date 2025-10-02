// motor_controller.ino
// ESP32 / Arduino sketch for Self-Balancing Robot
// Hardware: MPU6050 IMU + L298N Motor Driver
// Protocol (serial):
// - MCU -> PC: CSV line: t,ax,ay,az,gx,gy,gz,encL,encR
// - PC -> MCU: "M,<left_pwm>,<right_pwm>\n" (left/right -127..127 signed)

#include <Wire.h>
#include "MPU6050.h"
#include "Encoder.h"   // If you have encoders, otherwise comment out

// ----- PIN CONFIG (update based on your wiring) -----
const int ENA = 5;   // PWM pin for Left Motor (L298N ENA)
const int IN1 = 6;   // Left motor input 1
const int IN2 = 7;   // Left motor input 2

const int ENB = 9;   // PWM pin for Right Motor (L298N ENB)
const int IN3 = 10;  // Right motor input 1
const int IN4 = 11;  // Right motor input 2

const int ENC_LEFT_A  = 2;  // Encoder pins (optional)
const int ENC_LEFT_B  = 3;
const int ENC_RIGHT_A = 18;
const int ENC_RIGHT_B = 19;

MPU6050 imu;
Encoder encL(ENC_LEFT_A, ENC_LEFT_B);
Encoder encR(ENC_RIGHT_A, ENC_RIGHT_B);

char inBuf[64];

void setup() {
  Serial.begin(115200);
  delay(100);

  Wire.begin();
  imu.initialize();
  if (!imu.testConnection()) {
    Serial.println("IMU FAIL");
  }

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

void setMotor(int pwm, int in1, int in2, int enPin) {
  if (pwm > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(enPin, pwm);
  } else if (pwm < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(enPin, -pwm);
  } else {
    // stop
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(enPin, 0);
  }
}

void loop() {
  // --- Read IMU ---
  int16_t ax, ay, az, gx, gy, gz;
  imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  long encLCount = encL.read();
  long encRCount = encR.read();

  unsigned long t = millis();
  Serial.print(t); Serial.print(',');
  Serial.print(ax); Serial.print(',');
  Serial.print(ay); Serial.print(',');
  Serial.print(az); Serial.print(',');
  Serial.print(gx); Serial.print(',');
  Serial.print(gy); Serial.print(',');
  Serial.print(gz); Serial.print(',');
  Serial.print(encLCount); Serial.print(',');
  Serial.println(encRCount);

  // --- Check incoming motor command ---
  if (Serial.available()) {
    size_t len = Serial.readBytesUntil('\n', inBuf, sizeof(inBuf) - 1);
    inBuf[len] = 0;
    if (len > 0 && inBuf[0] == 'M' && inBuf[1] == ',') {
      int l = 0, r = 0;
      char *p = inBuf + 2;
      l = atoi(p);
      char *comma = strchr(p, ',');
      if (comma) r = atoi(comma + 1);

      l = constrain(l, -127, 127);
      r = constrain(r, -127, 127);

      // map -127..127 -> -255..255
      int pwmL = map(l, -127, 127, -255, 255);
      int pwmR = map(r, -127, 127, -255, 255);

      setMotor(pwmL, IN1, IN2, ENA);
      setMotor(pwmR, IN3, IN4, ENB);
    }
  }

  delay(5); // ~200Hz telemetry
}
