#include <Wire.h>

// MPU-6050 I²C address and registers
#define MPU_ADDR       0x68
#define REG_PWR_MGMT   0x6B
#define REG_SMPLRT_DIV 0x19
#define REG_CONFIG     0x1A
#define REG_GYRO_CFG   0x1B
#define REG_ACCEL_CFG  0x1C
#define REG_DATA_START 0x3B  // ACCEL_XOUT_H

// Timing
const unsigned long INTERVAL_US = 20000; // 50 Hz
unsigned long lastUs = 0;

// Scale factors
const float ACCEL_SCALE = 16384.0f;  // LSB/g for ±2 g
const float GYRO_SCALE  = 131.0f;    // LSB/(°/s) for ±250 °/s

// Complementary filter gain
const float alpha = 0.98f;

float angleRoll = 0, anglePitch = 0, angleYaw = 0;

// Calibration
const int CAL_SAMPLES = 200;
float gyroBiasX, gyroBiasY, gyroBiasZ;
float initRoll, initPitch;

void writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

void readRegs(uint8_t reg, uint8_t *buf, int len) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, len);
  for (int i = 0; i < len; i++) buf[i] = Wire.read();
}

void calibrate() {
  int32_t sumGX=0, sumGY=0, sumGZ=0;
  int32_t sumAX=0, sumAY=0, sumAZ=0;
  uint8_t raw[14];
  for (int i = 0; i < CAL_SAMPLES; i++) {
    readRegs(REG_DATA_START, raw, 14);
    int16_t ax = (raw[0]<<8)|raw[1];
    int16_t ay = (raw[2]<<8)|raw[3];
    int16_t az = (raw[4]<<8)|raw[5];
    int16_t gx = (raw[8]<<8)|raw[9];
    int16_t gy = (raw[10]<<8)|raw[11];
    int16_t gz = (raw[12]<<8)|raw[13];
    sumAX += ax; sumAY += ay; sumAZ += az;
    sumGX += gx; sumGY += gy; sumGZ += gz;
    delay(5);
  }
  gyroBiasX = sumGX / float(CAL_SAMPLES) / GYRO_SCALE;
  gyroBiasY = sumGY / float(CAL_SAMPLES) / GYRO_SCALE;
  gyroBiasZ = sumGZ / float(CAL_SAMPLES) / GYRO_SCALE;
  float ax0 = sumAX/float(CAL_SAMPLES),
        ay0 = sumAY/float(CAL_SAMPLES),
        az0 = sumAZ/float(CAL_SAMPLES);
  initRoll  = atan2(ay0, az0) * RAD_TO_DEG;
  initPitch = atan2(-ax0, sqrt(ay0*ay0 + az0*az0)) * RAD_TO_DEG;
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Wire.begin();
  // Wake up, set sample rate to 50 Hz, DLPF=42Hz, gyro±250, accel±2g
  writeReg(REG_PWR_MGMT,   0x00);
  writeReg(REG_SMPLRT_DIV, 19);
  writeReg(REG_CONFIG,     0x03);
  writeReg(REG_GYRO_CFG,   0x00);
  writeReg(REG_ACCEL_CFG,  0x00);

  Serial.println("Calibrating... keep still");
  calibrate();
  Serial.print("Gyro bias (°/s): ");
  Serial.print(gyroBiasX,2); Serial.print(',');
  Serial.print(gyroBiasY,2); Serial.print(',');
  Serial.println(gyroBiasZ,2);
  Serial.print("Init roll,pitch (°): ");
  Serial.print(initRoll,2); Serial.print(',');
  Serial.println(initPitch,2);

  Serial.println("roll,pitch,yaw");
  lastUs = micros();
}

void loop() {
  unsigned long now = micros();
  if (now - lastUs < INTERVAL_US) return;
  lastUs += INTERVAL_US;

  uint8_t raw[14];
  readRegs(REG_DATA_START, raw, 14);
  int16_t ax = (raw[0]<<8)|raw[1],
          ay = (raw[2]<<8)|raw[3],
          az = (raw[4]<<8)|raw[5],
          gx = (raw[8]<<8)|raw[9],
          gy = (raw[10]<<8)|raw[11],
          gz = (raw[12]<<8)|raw[13];

  float aX = ax/ACCEL_SCALE,
        aY = ay/ACCEL_SCALE,
        aZ = az/ACCEL_SCALE;
  float gX = gx/GYRO_SCALE - gyroBiasX,
        gY = gy/GYRO_SCALE - gyroBiasY,
        gZ = gz/GYRO_SCALE - gyroBiasZ;

  // Accel-based angles (°), zeroed at startup
  float rollAcc  = atan2(aY, aZ) * RAD_TO_DEG - initRoll;
  float pitchAcc = atan2(-aX, sqrt(aY*aY + aZ*aZ)) * RAD_TO_DEG - initPitch;

  float dt = INTERVAL_US * 1e-6f;
  // Complementary fusion
  angleRoll  = alpha * (angleRoll  + gX * dt) + (1 - alpha) * rollAcc;
  anglePitch = alpha * (anglePitch + gY * dt) + (1 - alpha) * pitchAcc;
  angleYaw  += gZ * dt;  // no magnetometer, so yaw drifts

  Serial.print(angleRoll, 2);   Serial.print(',');
  Serial.print(anglePitch, 2);  Serial.print(',');
  Serial.println(angleYaw, 2);
}
