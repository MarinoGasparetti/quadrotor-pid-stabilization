/*
 * QSCS - LED Stabilization Test
 * 4 LED sostituiscono motori: brightness = potenza motore
 * Sensore: MPU-6050 (I2C) per pitch/roll reale
 * PID dual-axis → mixer → PWM LED
 *
 * Wiring:
 *   MPU-6050 SDA → A4, SCL → A5, VCC → 3.3V, GND → GND
 *   LED_FL (Front-Left)  → pin 9  (PWM)
 *   LED_FR (Front-Right) → pin 10 (PWM)
 *   LED_RL (Rear-Left)   → pin 5  (PWM)
 *   LED_RR (Rear-Right)  → pin 6  (PWM)
 *   Ogni LED con resistenza 220Ω verso GND
 */

#include <Wire.h>

// ── Pin ────────────────────────────────────────────────────────────────
#define PIN_LED_FL 9
#define PIN_LED_FR 10
#define PIN_LED_RL 5
#define PIN_LED_RR 6

// ── MPU-6050 ───────────────────────────────────────────────────────────
#define MPU_ADDR        0x68
#define MPU_PWR_MGMT_1  0x6B
#define MPU_ACCEL_XOUT  0x3B
#define MPU_GYRO_XOUT   0x43

// ── PID Params (profilo STATIC / hovering) ─────────────────────────────
// Tuning conservativo per test visivo con LED
const float KP = 1.8f;
const float KI = 0.05f;
const float KD = 0.8f;

// Output PID → [-100, +100], poi mappato su throttle base ± correzione
const float PID_MIN = -100.0f;
const float PID_MAX =  100.0f;

// Throttle base: potenza "hovering" virtuale (0-255 LED)
// Con LED non c'è fisica reale, ma simula punto di equilibrio
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

// ── Calibrazione offset gyro/accel ────────────────────────────────────
float accel_offset_x = 0;
float accel_offset_y = 0;

// ── Angoli filtrati (complementary filter) ────────────────────────────
float angle_pitch = 0;
float angle_roll  = 0;

// ─────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  pinMode(PIN_LED_FL, OUTPUT);
  pinMode(PIN_LED_FR, OUTPUT);
  pinMode(PIN_LED_RL, OUTPUT);
  pinMode(PIN_LED_RR, OUTPUT);

  // Accendi tutti al minimo per segnalare boot
  analogWrite(PIN_LED_FL, 10);
  analogWrite(PIN_LED_FR, 10);
  analogWrite(PIN_LED_RL, 10);
  analogWrite(PIN_LED_RR, 10);

  mpu_init();
  calibrate_accel();

  // LED al base throttle → "in volo virtuale"
  analogWrite(PIN_LED_FL, BASE_THROTTLE);
  analogWrite(PIN_LED_FR, BASE_THROTTLE);
  analogWrite(PIN_LED_RL, BASE_THROTTLE);
  analogWrite(PIN_LED_RR, BASE_THROTTLE);

  last_time_us = micros();

  Serial.println("QSCS LED Test - Ready");
  Serial.println("t_ms,pitch,roll,pid_p,pid_r,FL,FR,RL,RR");
}

// ─────────────────────────────────────────────────────────────────────
void loop() {
  unsigned long now_us = micros();
  float dt = (now_us - last_time_us) / 1e6f;
  last_time_us = now_us;

  if (dt <= 0 || dt > 0.1f) {
    dt = 0.004f; // fallback: 250Hz loop atteso
  }

  // Lettura sensore
  float ax, ay, az, gx, gy;
  read_mpu(&ax, &ay, &az, &gx, &gy);

  // Angoli da accelerometro (gradi)
  float accel_pitch = atan2f(ay - accel_offset_y, az) * 57.2958f;
  float accel_roll  = atan2f(ax - accel_offset_x, az) * 57.2958f;

  // Complementary filter: 98% gyro + 2% accel
  angle_pitch = 0.98f * (angle_pitch + gx * dt) + 0.02f * accel_pitch;
  angle_roll  = 0.98f * (angle_roll  + gy * dt) + 0.02f * accel_roll;

  // PID: setpoint = 0° (livello)
  float out_pitch = pid_compute(&pid_pitch, 0.0f, angle_pitch, dt);
  float out_roll  = pid_compute(&pid_roll,  0.0f, angle_roll,  dt);

  // Mixer quadrotor standard (+ config):
  //   pitch+  → FL/FR salgono, RL/RR scendono
  //   roll+   → FR/RR salgono, FL/RL scendono
  int fl = BASE_THROTTLE + (int)(out_pitch) - (int)(out_roll);
  int fr = BASE_THROTTLE + (int)(out_pitch) + (int)(out_roll);
  int rl = BASE_THROTTLE - (int)(out_pitch) - (int)(out_roll);
  int rr = BASE_THROTTLE - (int)(out_pitch) + (int)(out_roll);

  fl = clamp_pwm(fl);
  fr = clamp_pwm(fr);
  rl = clamp_pwm(rl);
  rr = clamp_pwm(rr);

  analogWrite(PIN_LED_FL, fl);
  analogWrite(PIN_LED_FR, fr);
  analogWrite(PIN_LED_RL, rl);
  analogWrite(PIN_LED_RR, rr);

  // Log CSV ogni ciclo (Serial a 115200 regge ~250Hz)
  Serial.print(millis());    Serial.print(',');
  Serial.print(angle_pitch, 2); Serial.print(',');
  Serial.print(angle_roll,  2); Serial.print(',');
  Serial.print(out_pitch,   2); Serial.print(',');
  Serial.print(out_roll,    2); Serial.print(',');
  Serial.print(fl);          Serial.print(',');
  Serial.print(fr);          Serial.print(',');
  Serial.print(rl);          Serial.print(',');
  Serial.println(rr);

  // Loop ~250Hz (4ms)
  delayMicroseconds(500);
}

// ─────────────────────────────────────────────────────────────────────
float pid_compute(PIDState* s, float setpoint, float current, float dt) {
  float error = setpoint - current;

  float p_out = KP * error;

  s->integral += error * dt;
  // Anti-windup: clamp integrale
  s->integral = constrain(s->integral, PID_MIN / KI, PID_MAX / KI);
  float i_out = KI * s->integral;

  float derivative = (dt > 1e-6f) ? (error - s->last_error) / dt : 0;
  float d_out = KD * derivative;

  s->last_error = error;

  float out = p_out + i_out + d_out;
  return constrain(out, PID_MIN, PID_MAX);
}

// ─────────────────────────────────────────────────────────────────────
void mpu_init() {
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_PWR_MGMT_1);
  Wire.write(0x00); // wake up
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
  Wire.read(); Wire.read(); // temperatura, scartata
  int16_t raw_gx = (Wire.read() << 8) | Wire.read();
  int16_t raw_gy = (Wire.read() << 8) | Wire.read();
  // gz non serve per pitch/roll
  Wire.read(); Wire.read();

  // ±2g range → 16384 LSB/g
  *ax = raw_ax / 16384.0f;
  *ay = raw_ay / 16384.0f;
  *az = raw_az / 16384.0f;

  // ±250°/s range → 131 LSB/°/s → converti in °/s
  *gx = raw_gx / 131.0f;
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

int clamp_pwm(int val) {
  return constrain(val, 0, 255);
}
