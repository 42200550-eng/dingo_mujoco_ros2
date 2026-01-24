#include <Arduino.h>
#include <Wire.h>

// Optional IMU (MPU6050). Install Adafruit_MPU6050 + Adafruit_Sensor libraries.
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// ---------------- User configuration ----------------
static const uint32_t SERIAL_BAUD = 115200;
static const uint32_t CMD_RATE_HZ = 100;
static const uint32_t TELEMETRY_RATE_HZ = 100;
static const uint32_t CMD_TIMEOUT_MS = 300;

static const uint8_t SERVO_COUNT = 12;

// GPIO pins for 12 servos (EDIT THESE)
static const int SERVO_PINS[SERVO_COUNT] = {
  2, 4, 5, 12, 13, 14, 15, 16, 17, 18, 19, 21
};

// PWM settings
static const uint32_t PWM_FREQ_HZ = 50;       // typical analog servo
static const uint8_t PWM_RES_BITS = 16;       // 16-bit resolution
static const int PWM_MIN_US = 500;
static const int PWM_MAX_US = 2500;

// Joint mapping
static const float ANGLE_SCALE = 1000.0f;     // int16 => rad = val / scale
static const float MAX_ANGLE_RAD = 1.6f;
static const float SAFE_POSE_RAD[SERVO_COUNT] = {
  0,0,0, 0,0,0, 0,0,0, 0,0,0
};

// Battery sensing
static const int VBAT_PIN = 34;               // ADC1 pin
static const float VBAT_DIV_RATIO = 2.0f;     // voltage divider ratio
static const float VBAT_REF_V = 3.3f;
static const int ADC_MAX = 4095;

// ----------------------------------------------------

static const uint8_t TX_HEADER[2] = {0x55, 0xAA}; // ESP -> PC
static const uint8_t RX_HEADER[2] = {0xAA, 0x55}; // PC -> ESP

static const size_t RX_PAYLOAD_LEN = 1 + (SERVO_COUNT * 2); // seq + 12*int16
static const size_t RX_FRAME_LEN = 2 + RX_PAYLOAD_LEN + 2;

static const size_t TX_PAYLOAD_LEN = 1 + (9 * 2) + 2 + 1; // seq + imu(9*int16) + vbat + fault
static const size_t TX_FRAME_LEN = 2 + TX_PAYLOAD_LEN + 2;

// IMU scale to telemetry (see ROS node params)
static const float ACCEL_G_PER_LSB = 0.001f;   // g/LSB
static const float GYRO_DPS_PER_LSB = 0.1f;    // deg/s/LSB
static const float MAG_UT_PER_LSB = 0.1f;      // uT/LSB (unused)

Adafruit_MPU6050 imu;
bool imu_ok = false;

uint8_t seq_tx = 0;
uint8_t seq_rx = 0;

float last_cmd[SERVO_COUNT];
uint32_t last_cmd_ms = 0;

// -------------- CRC16-CCITT ----------------
uint16_t crc16_ccitt(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; ++i) {
    crc ^= (uint16_t)data[i] << 8;
    for (int j = 0; j < 8; ++j) {
      if (crc & 0x8000) {
        crc = (crc << 1) ^ 0x1021;
      } else {
        crc = (crc << 1);
      }
    }
  }
  return crc;
}

uint16_t usToDuty(int us) {
  us = constrain(us, PWM_MIN_US, PWM_MAX_US);
  const uint32_t period_us = 1000000UL / PWM_FREQ_HZ;
  const uint32_t max_duty = (1UL << PWM_RES_BITS) - 1UL;
  return (uint16_t)((uint32_t)us * max_duty / period_us);
}

int angleToUs(float rad) {
  rad = constrain(rad, -MAX_ANGLE_RAD, MAX_ANGLE_RAD);
  const float t = (rad / MAX_ANGLE_RAD + 1.0f) * 0.5f; // 0..1
  return (int)(PWM_MIN_US + t * (PWM_MAX_US - PWM_MIN_US));
}

void setServo(int idx, float rad) {
  int us = angleToUs(rad);
  uint16_t duty = usToDuty(us);
  ledcWrite(idx, duty);
}

void applyCommand(const float *q) {
  for (int i = 0; i < SERVO_COUNT; ++i) {
    setServo(i, q[i]);
  }
}

void setupPWM() {
  for (int i = 0; i < SERVO_COUNT; ++i) {
    ledcSetup(i, PWM_FREQ_HZ, PWM_RES_BITS);
    ledcAttachPin(SERVO_PINS[i], i);
  }
}

void setupIMU() {
  imu_ok = imu.begin();
  if (!imu_ok) {
    // IMU not found, continue without it
    return;
  }
  imu.setAccelerometerRange(MPU6050_RANGE_8_G);
  imu.setGyroRange(MPU6050_RANGE_500_DEG);
  imu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

int16_t clamp_i16(int32_t v) {
  if (v > 32767) return 32767;
  if (v < -32768) return -32768;
  return (int16_t)v;
}

void sendTelemetry() {
  uint8_t payload[TX_PAYLOAD_LEN];
  payload[0] = seq_tx++;

  // IMU
  sensors_event_t accel, gyro, temp;
  float ax_g = 0, ay_g = 0, az_g = 0;
  float gx_dps = 0, gy_dps = 0, gz_dps = 0;

  if (imu_ok) {
    imu.getEvent(&accel, &gyro, &temp);
    ax_g = accel.acceleration.x / 9.80665f;
    ay_g = accel.acceleration.y / 9.80665f;
    az_g = accel.acceleration.z / 9.80665f;

    gx_dps = gyro.gyro.x * 57.2958f;
    gy_dps = gyro.gyro.y * 57.2958f;
    gz_dps = gyro.gyro.z * 57.2958f;
  }

  int16_t imu_raw[9];
  imu_raw[0] = clamp_i16((int32_t)round(ax_g / ACCEL_G_PER_LSB));
  imu_raw[1] = clamp_i16((int32_t)round(ay_g / ACCEL_G_PER_LSB));
  imu_raw[2] = clamp_i16((int32_t)round(az_g / ACCEL_G_PER_LSB));
  imu_raw[3] = clamp_i16((int32_t)round(gx_dps / GYRO_DPS_PER_LSB));
  imu_raw[4] = clamp_i16((int32_t)round(gy_dps / GYRO_DPS_PER_LSB));
  imu_raw[5] = clamp_i16((int32_t)round(gz_dps / GYRO_DPS_PER_LSB));
  imu_raw[6] = 0;
  imu_raw[7] = 0;
  imu_raw[8] = 0;

  memcpy(&payload[1], imu_raw, sizeof(imu_raw));

  // Battery (mV)
  int adc = analogRead(VBAT_PIN);
  float v = (adc / (float)ADC_MAX) * VBAT_REF_V * VBAT_DIV_RATIO;
  int16_t vbat_mv = (int16_t)round(v * 1000.0f);
  payload[19] = (uint8_t)(vbat_mv & 0xFF);
  payload[20] = (uint8_t)((vbat_mv >> 8) & 0xFF);

  // Fault flags (bitmask)
  uint8_t fault = 0;
  payload[21] = fault;

  uint16_t crc = crc16_ccitt(payload, TX_PAYLOAD_LEN);

  uint8_t frame[TX_FRAME_LEN];
  frame[0] = TX_HEADER[0];
  frame[1] = TX_HEADER[1];
  memcpy(&frame[2], payload, TX_PAYLOAD_LEN);
  frame[2 + TX_PAYLOAD_LEN] = (uint8_t)(crc & 0xFF);
  frame[3 + TX_PAYLOAD_LEN] = (uint8_t)((crc >> 8) & 0xFF);

  Serial.write(frame, TX_FRAME_LEN);
}

bool readFrame(uint8_t *out_payload) {
  static uint8_t buf[128];
  static size_t len = 0;

  while (Serial.available()) {
    uint8_t b = Serial.read();
    if (len < sizeof(buf)) {
      buf[len++] = b;
    } else {
      len = 0;
    }
  }

  for (;;) {
    if (len < RX_FRAME_LEN) return false;

    size_t i = 0;
    while (i + 1 < len && !(buf[i] == RX_HEADER[0] && buf[i + 1] == RX_HEADER[1])) {
      i++;
    }
    if (i > 0) {
      memmove(buf, buf + i, len - i);
      len -= i;
      if (len < RX_FRAME_LEN) return false;
    }

    uint8_t *frame = buf;
    if (frame[0] != RX_HEADER[0] || frame[1] != RX_HEADER[1]) {
      memmove(buf, buf + 1, len - 1);
      len -= 1;
      continue;
    }

    uint16_t crc_rx = (uint16_t)frame[2 + RX_PAYLOAD_LEN] | ((uint16_t)frame[3 + RX_PAYLOAD_LEN] << 8);
    uint16_t crc = crc16_ccitt(frame + 2, RX_PAYLOAD_LEN);
    if (crc != crc_rx) {
      memmove(buf, buf + 2, len - 2);
      len -= 2;
      continue;
    }

    memcpy(out_payload, frame + 2, RX_PAYLOAD_LEN);
    memmove(buf, buf + RX_FRAME_LEN, len - RX_FRAME_LEN);
    len -= RX_FRAME_LEN;
    return true;
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  Wire.begin();
  setupPWM();
  setupIMU();

  analogReadResolution(12);
  analogSetPinAttenuation(VBAT_PIN, ADC_11db);

  for (int i = 0; i < SERVO_COUNT; ++i) {
    last_cmd[i] = SAFE_POSE_RAD[i];
  }
  applyCommand(last_cmd);
  last_cmd_ms = millis();
}

void loop() {
  static uint32_t last_cmd_tick = 0;
  static uint32_t last_tel_tick = 0;

  uint8_t payload[RX_PAYLOAD_LEN];
  if (readFrame(payload)) {
    seq_rx = payload[0];
    for (int i = 0; i < SERVO_COUNT; ++i) {
      int16_t v = (int16_t)(payload[1 + i * 2] | (payload[2 + i * 2] << 8));
      float rad = (float)v / ANGLE_SCALE;
      last_cmd[i] = rad;
    }
    last_cmd_ms = millis();
  }

  uint32_t now = millis();
  if (now - last_cmd_ms > CMD_TIMEOUT_MS) {
    applyCommand(SAFE_POSE_RAD);
  } else {
    applyCommand(last_cmd);
  }

  if (now - last_cmd_tick >= (1000 / CMD_RATE_HZ)) {
    last_cmd_tick = now;
  }

  if (now - last_tel_tick >= (1000 / TELEMETRY_RATE_HZ)) {
    last_tel_tick = now;
    sendTelemetry();
  }
}
