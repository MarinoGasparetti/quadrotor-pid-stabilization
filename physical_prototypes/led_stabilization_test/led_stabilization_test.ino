/*
 * QSCS - LED Stabilization Test
 * Target: Waveshare ESP32-S3-Zero (ESP32-S3FH4R2)
 * 4 LED sostituiscono motori: brightness = potenza motore
 * Sensore: MPU-6050 (I2C) per pitch/roll reale
 * PID dual-axis → mixer → PWM LED
 *
 * Wiring:
 *   MPU-6050 SDA → GPIO 8, SCL → GPIO 9, VCC → 3.3V, GND → GND
 *   LED_FL (Front-Left)  → GPIO 2  (PWM) + 220Ω → GND
 *   LED_FR (Front-Right) → GPIO 3  (PWM) + 220Ω → GND
 *   LED_RL (Rear-Left)   → GPIO 4  (PWM) + 220Ω → GND
 *   LED_RR (Rear-Right)  → GPIO 5  (PWM) + 220Ω → GND
 *
 * Board: "ESP32S3 Dev Module" in Arduino IDE
 * Upload speed: 921600, USB Mode: "USB-OTG (TinyUSB)"
 */

#include <Wire.h>

// ── I2C pins ESP32-S3 ─────────────────────────────────────────────────
#define I2C_SDA 8
#define I2C_SCL 9

// ── LED pins (PWM-capable GPIO) ───────────────────────────────────────
#define PIN_LED_FL 2
#define PIN_LED_FR 3
#define PIN_LED_RL 4
#define PIN_LED_RR 5

// ── PWM ESP32 (Arduino Core v3.x API) ────────────────────────────────
// ledcAttach(pin, freq, resolution) + ledcWrite(pin, duty) — no canali
#define PWM_FREQ       5000
#define PWM_RESOLUTION 8      // 8-bit → 0-255

// ── MPU-6050 ──────────────────────────────────────────────────────────
#define MPU_ADDR        0x68
#define MPU_PWR_MGMT_1  0x6B
#define MPU_ACCEL_XOUT  0x3B

// ── PID Params (profilo STATIC / hovering) ────────────────────────────
const float KP = 1.8f;
const float KI = 0.05f;
const float KD = 0.8f;

const float PID_MIN = -100.0f;
const float PID_MAX =  100.0f;

// Throttle base: punto di equilibrio virtuale (0-255)
const int BASE_THROTTLE = 140;

// ── Stato PID ─────────────────────────────────────────────────────────
struct PIDState {
  float integral;
  float last_error;
};

PIDState pid_pitch = {0, 0};
PIDState pid_roll  = {0, 0};

// ── Timing ────────────────────────────────────────────────────────────
unsigned long last_time_us = 0;

// ── Calibrazione accel ────────────────────────────────────────────────
float accel_offset_x = 0;
float accel_offset_y = 0;

// ── Angoli filtrati (complementary filter) ────────────────────────────
float angle_pitch = 0;
float angle_roll  = 0;

// ─────────────────────────────────────────────────────────────────────
void led_write(uint8_t pin, int val) {
  ledcWrite(pin, constrain(val, 0, 255));
}

// ─────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  // LEDC setup (ESP32 Arduino Core v3.x)
  ledcAttach(PIN_LED_FL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(PIN_LED_FR, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(PIN_LED_RL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(PIN_LED_RR, PWM_FREQ, PWM_RESOLUTION);

  // Boot signal: tutti al minimo
  led_write(PIN_LED_FL, 10);
  led_write(PIN_LED_FR, 10);
  led_write(PIN_LED_RL, 10);
  led_write(PIN_LED_RR, 10);

  Wire.begin(I2C_SDA, I2C_SCL);
  mpu_init();
  calibrate_accel();

  // LED al base throttle → "in volo virtuale"
  led_write(PIN_LED_FL, BASE_THROTTLE);
  led_write(PIN_LED_FR, BASE_THROTTLE);
  led_write(PIN_LED_RL, BASE_THROTTLE);
  led_write(PIN_LED_RR, BASE_THROTTLE);

  last_time_us = micros();

  Serial.println("QSCS LED Test - Ready [ESP32-S3]");
  Serial.println("t_ms,pitch,roll,pid_p,pid_r,FL,FR,RL,RR");
}

// ─────────────────────────────────────────────────────────────────────
void loop() {
  unsigned long now_us = micros();
  float dt = (now_us - last_time_us) / 1e6f;
  last_time_us = now_us;

  if (dt <= 0 || dt > 0.1f) dt = 0.004f;

  float ax, ay, az, gx, gy;
  read_mpu(&ax, &ay, &az, &gx, &gy);

  float accel_pitch = atan2f(ay - accel_offset_y, az) * 57.2958f;
  float accel_roll  = atan2f(ax - accel_offset_x, az) * 57.2958f;

  // Complementary filter: 98% gyro + 2% accel
  angle_pitch = 0.98f * (angle_pitch + gx * dt) + 0.02f * accel_pitch;
  angle_roll  = 0.98f * (angle_roll  + gy * dt) + 0.02f * accel_roll;

  float out_pitch = pid_compute(&pid_pitch, 0.0f, angle_pitch, dt);
  float out_roll  = pid_compute(&pid_roll,  0.0f, angle_roll,  dt);

  // Mixer quadrotor standard (+ config):
  //   pitch+ → FL/FR salgono, RL/RR scendono
  //   roll+  → FR/RR salgono, FL/RL scendono
  int fl = BASE_THROTTLE + (int)(out_pitch) - (int)(out_roll);
  int fr = BASE_THROTTLE + (int)(out_pitch) + (int)(out_roll);
  int rl = BASE_THROTTLE - (int)(out_pitch) - (int)(out_roll);
  int rr = BASE_THROTTLE - (int)(out_pitch) + (int)(out_roll);

  led_write(PIN_LED_FL, fl);
  led_write(PIN_LED_FR, fr);
  led_write(PIN_LED_RL, rl);
  led_write(PIN_LED_RR, rr);

  Serial.print(millis());        Serial.print(',');
  Serial.print(angle_pitch, 2);  Serial.print(',');
  Serial.print(angle_roll,  2);  Serial.print(',');
  Serial.print(out_pitch,   2);  Serial.print(',');
  Serial.print(out_roll,    2);  Serial.print(',');
  Serial.print(constrain(fl, 0, 255)); Serial.print(',');
  Serial.print(constrain(fr, 0, 255)); Serial.print(',');
  Serial.print(constrain(rl, 0, 255)); Serial.print(',');
  Serial.println(constrain(rr, 0, 255));

  delayMicroseconds(500); // ~250Hz loop
}

// ─────────────────────────────────────────────────────────────────────
float pid_compute(PIDState* s, float setpoint, float current, float dt) {
  float error = setpoint - current;

  float p_out = KP * error;

  s->integral += error * dt;
  s->integral = constrain(s->integral, PID_MIN / KI, PID_MAX / KI);
  float i_out = KI * s->integral;

  float derivative = (dt > 1e-6f) ? (error - s->last_error) / dt : 0;
  float d_out = KD * derivative;

  s->last_error = error;

  return constrain(p_out + i_out + d_out, PID_MIN, PID_MAX);
}

// ─────────────────────────────────────────────────────────────────────
void mpu_init() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_PWR_MGMT_1);
  Wire.write(0x00);
  Wire.endTransmission(true);
  delay(100);
}

void read_mpu(float* ax, float* ay, float* az, float* gx, float* gy) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_ACCEL_XOUT);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);

  int16_t raw_ax = (Wire.read() << 8) | Wire.read();
  int16_t raw_ay = (Wire.read() << 8) | Wire.read();
  int16_t raw_az = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read(); // temperatura
  int16_t raw_gx = (Wire.read() << 8) | Wire.read();
  int16_t raw_gy = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read(); // gz non usato

  *ax = raw_ax / 16384.0f; // ±2g → 16384 LSB/g
  *ay = raw_ay / 16384.0f;
  *az = raw_az / 16384.0f;
  *gx = raw_gx / 131.0f;   // ±250°/s → 131 LSB/°/s
  *gy = raw_gy / 131.0f;
}

void calibrate_accel() {
  Serial.println("Calibrazione... tieni fermo il board");
  delay(1000);

  const int N = 200;
  float sum_x = 0, sum_y = 0;
  for (int i = 0; i < N; i++) {
    float ax, ay, az, gx, gy;
    read_mpu(&ax, &ay, &az, &gx, &gy);
    sum_x += ax;
    sum_y += ay;
    delay(5);
  }
  accel_offset_x = sum_x / N;
  accel_offset_y = sum_y / N;

  Serial.print("Offset accel: x=");
  Serial.print(accel_offset_x, 4);
  Serial.print(" y=");
  Serial.println(accel_offset_y, 4);
}
