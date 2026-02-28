#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <MS5611.h>
#include <SPI.h>
#include <SdFat.h>
#include <math.h>

// ---------- SENSORS ----------
Adafruit_MPU6050 mpu;
MS5611 ms5611;

// ---------- TIMING ----------
const uint32_t OUTPUT_PERIOD_US = 10000;  // 100 Hz
uint32_t last_tick_us = 0;                // scheduler
float dt = 0.01f;                         // updated from micros()

// ---------- LOGGING WINDOW ----------
const uint32_t LOG_DURATION_MS = 300000;  // 10 s of data to SD
uint32_t logStartMs = 0;
bool loggingActive = false;

// ---------- TRIGGER & ARMING (robust) ----------
const float TRIGGER_AZ_MPS2 = 20.0f;   // go/no-go threshold (net vertical accel, m/s^2)
const uint32_t TRIGGER_HOLD_MS = 65.0;  // must exceed threshold for this long
const float LPF_AZ_ALPHA = 0.25f;     // low-pass on vertical accel (0..1, higher = less smoothing)

const float QUIET_AMAG_TOL = 0.8f;      // | |a| - 9.80665 | must be < this to be "quiet"
const float QUIET_GYRO_MAX_DPS = 8.0f;  // each axis < this (deg/s) to be "quiet"
const uint32_t ARM_QUIET_MS = 800;      // must be quiet this long to ARM

const float TRIGGER_RELEASE_HYST = 2.0f;  // hysteresis below threshold

bool armed = false;  // will ARM after pad is quiet
bool triggered = false;
float aZ_world_lpf = 0.0f;       // filtered world-frame vertical accel (with +g removed)
float aZ_bias_slow = 0.0f;       // slow bias, learned on pad
const float BIAS_ALPHA = 0.01f;  // bias learning rate while quiet

uint32_t quietSinceMs = 0;
uint32_t aboveSinceMs = 0;

// ---------- ORIENTATION (complementary) ----------
float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;  // degrees
const float alpha = 0.98f;                    // gyro weight

// ---------- KALMAN (alt/vel) FULL-FLIGHT (logged) ----------
float altitude_est = 0.0f;
float velocity_est = 0.0f;
float P[2][2] = { { 10.f, 0.f }, { 0.f, 10.f } };
const float Q[2][2] = { { 0.1f, 0.f }, { 0.f, 0.1f } };
const float R_meas = 1.0f;  // baro alt variance (m^2)

// ---------- COAST WINDOW + CONTROL ESTIMATOR (resets at T+3.2s) ----------
const uint32_t COAST_DELAY_MS = 3200;
bool coastWindowActive = false;
uint32_t launchMs = 0;

// Coast-only estimator states (use these for apogee prediction)
float alt_ctrl = 0.0f;
float vel_ctrl = 0.0f;
float Pc[2][2] = { { 10.f, 0.f }, { 0.f, 10.f } };

// Control KF tuning (start conservative; you can tune later)
const float Rc_meas = 3.0f;  // higher = trust baro less (reduces spikes)
const float Qc00 = 0.001f;   // process noise on altitude
const float Qc11 = 0.008f;   // process noise on velocity

// ---------- CALIBRATION OFFSETS (BODY FRAME) ----------
float accel_offset_x = 0.f, accel_offset_y = 0.f, accel_offset_z = 0.f;
float gyro_offset_x = 0.f, gyro_offset_y = 0.f, gyro_offset_z = 0.f;
float pressure_offset = 0.f;  // Pa baseline

// ---------- SD (SdFat, 18 MHz, 4KB buffer) ----------
SdFat sd;
FsFile logFile;
const uint8_t SD_CS = 10;
static char outBuf[4096];
static size_t outIdx = 0;

void sd_write_buffered(const char* data, size_t n) {
  if (n > sizeof(outBuf)) return;
  if (outIdx + n > sizeof(outBuf)) {  // flush full 4 KB block
    logFile.write(outBuf, outIdx);
    outIdx = 0;
  }
  memcpy(outBuf + outIdx, data, n);
  outIdx += n;
}

void sd_flush_close() {
  if (!logFile) return;
  if (outIdx) {
    logFile.write(outBuf, outIdx);
    outIdx = 0;
  }
  uint64_t actualSize = logFile.curPosition();
  logFile.truncate(actualSize);
  logFile.sync();
  logFile.close();
}

bool sd_open_log() {
  // Find next filename: FLIGHT###.CSV
  char fname[20];
  for (uint16_t i = 1; i < 1000; ++i) {
    snprintf(fname, sizeof(fname), "FLIGHT%u.CSV", i);
    if (!sd.exists(fname)) {
      logFile = sd.open(fname, O_WRONLY | O_CREAT | O_EXCL);
      if (!logFile) {
        Serial.println("SD open fail");
        return false;
      }
      Serial.print("Logging to ");
      Serial.println(fname);
      break;
    }
  }
  // Pre-allocate for contiguous writes (optional)
  logFile.preAllocate(4UL * 1024UL * 1024UL);

  // CSV header (added Alt_Ctrl and Vel_Ctrl)
  const char* header =
    "Time(ms),Roll(deg),Pitch(deg),Yaw(deg),"
    "Est_Alt(m),Est_Vel(m/s),Alt_Ctrl(m),Vel_Ctrl(m/s),"
    "Raw_Alt(m),Ax(mps2),Ay(mps2),Az(mps2),Gx(dps),Gy(dps),Gz(dps),Height(m),Temp(C)\n";
  sd_write_buffered(header, strlen(header));
  return true;
}
// ---------- SENSOR→BODY MAP (Y+ up) & gyro rad/s→deg/s ----------
static inline void map_to_body(const sensors_event_t& a, const sensors_event_t& g,
                               float& ax, float& ay, float& az,
                               float& gx, float& gy, float& gz) {
  // Body axes: X=fwd, Y=right, Z=up
  // Mount: sensor +Y is UP  => body Z = +sensor Y
  ax = a.acceleration.x;   // body X (assume sensor +X is forward)
  ay = -a.acceleration.z;  // body Y (right) = -sensor Z
  az = a.acceleration.y;   // body Z (up)    = +sensor Y

  const float RAD2DEG = 180.0f / PI;  // Adafruit gyro is rad/s
  gx = g.gyro.x * RAD2DEG;            // roll about body X
  gy = -g.gyro.z * RAD2DEG;           // pitch about body Y
  gz = g.gyro.y * RAD2DEG;            // yaw about body Z
}

// ---------- UTIL: body->world vertical accel (Z-up), ignore yaw ----------
static inline float verticalAccelWorld(float ax, float ay, float az, float roll_deg, float pitch_deg) {
  const float cphi = cosf(roll_deg * PI / 180.f);
  const float sphi = sinf(roll_deg * PI / 180.f);
  const float cth = cosf(pitch_deg * PI / 180.f);
  const float sth = sinf(pitch_deg * PI / 180.f);
  // Z row of Rz*Ry*Rx with yaw ignored: az_world = -sth*ax + cth*sphi*ay + cth*cphi*az
  return (-sth * ax) + (cth * sphi * ay) + (cth * cphi * az);
}

// ---------- COMPLEMENTARY FILTER (BODY FRAME INPUTS) ----------
void complementaryFilter(float ax, float ay, float az, float gx_dps, float gy_dps, float gz_dps, float dt_s) {
  ax -= accel_offset_x;
  ay -= accel_offset_y;
  az -= accel_offset_z;
  const float gx = gx_dps - gyro_offset_x;
  const float gy = gy_dps - gyro_offset_y;
  const float gz = gz_dps - gyro_offset_z;

  const float amag = sqrtf(ax * ax + ay * ay + az * az);
  const bool useAccel = (amag > 8.0f && amag < 12.5f);  // m/s^2

  const float accel_roll_deg = atan2f(ay, az) * 180.f / PI;
  const float accel_pitch_deg = atanf(-ax / sqrtf(ay * ay + az * az)) * 180.f / PI;

  if (useAccel) {
    roll = alpha * (roll + gx * dt_s) + (1.f - alpha) * accel_roll_deg;
    pitch = alpha * (pitch + gy * dt_s) + (1.f - alpha) * accel_pitch_deg;
  } else {
    roll += gx * dt_s;
    pitch += gy * dt_s;
  }
  yaw += gz * dt_s;
}

// ---------- KALMAN FULL-FLIGHT (altitude_est/velocity_est) ----------
void kalmanFilter(float measured_altitude_m, float ax, float ay, float az, float dt_s) {
  const float h0 = 44330.0f * (1.0f - powf(pressure_offset / 101325.0f, 0.1903f));
  const float z_meas = measured_altitude_m - h0;

  ax -= accel_offset_x;
  ay -= accel_offset_y;
  az -= accel_offset_z;
  const float a_world_z = verticalAccelWorld(ax, ay, az, roll, pitch);
  const float a_net = a_world_z - 9.80665f;  // m/s^2, Z-up

  const float altitude_pred = altitude_est + velocity_est * dt_s + 0.5f * a_net * dt_s * dt_s;
  const float velocity_pred = velocity_est + a_net * dt_s;

  float Pp00 = P[0][0] + 2.f * P[0][1] * dt_s + P[1][1] * dt_s * dt_s + Q[0][0];
  float Pp01 = P[0][1] + P[1][1] * dt_s;
  float Pp10 = P[1][0] + P[1][1] * dt_s;
  float Pp11 = P[1][1] + Q[1][1];

  const float innov = z_meas - altitude_pred;
  const float S = Pp00 + R_meas;
  const float K0 = Pp00 / S;
  const float K1 = Pp10 / S;

  altitude_est = altitude_pred + K0 * innov;
  velocity_est = velocity_pred + K1 * innov;

  P[0][0] = Pp00 - K0 * Pp00;
  P[0][1] = Pp01 - K0 * Pp01;
  P[1][0] = Pp10 - K1 * Pp00;
  P[1][1] = Pp11 - K1 * Pp01;
}

// ---------- CONTROL KALMAN (alt_ctrl/vel_ctrl) ----------
void kalmanControlUpdate(float measured_altitude_m, float ax, float ay, float az, float dt_s) {
  const float h0 = 44330.0f * (1.0f - powf(pressure_offset / 101325.0f, 0.1903f));
  const float z_meas = measured_altitude_m - h0;

  ax -= accel_offset_x;
  ay -= accel_offset_y;
  az -= accel_offset_z;
  const float a_world_z = verticalAccelWorld(ax, ay, az, roll, pitch);
  const float a_net = a_world_z - 9.80665f;

  const float alt_pred = alt_ctrl + vel_ctrl * dt_s + 0.5f * a_net * dt_s * dt_s;
  const float vel_pred = vel_ctrl + a_net * dt_s;

  float Pp00 = Pc[0][0] + 2.f * Pc[0][1] * dt_s + Pc[1][1] * dt_s * dt_s + Qc00;
  float Pp01 = Pc[0][1] + Pc[1][1] * dt_s;
  float Pp10 = Pc[1][0] + Pc[1][1] * dt_s;
  float Pp11 = Pc[1][1] + Qc11;

  const float innov = z_meas - alt_pred;
  const float S = Pp00 + Rc_meas;
  const float K0 = Pp00 / S;
  const float K1 = Pp10 / S;

  alt_ctrl = alt_pred + K0 * innov;
  vel_ctrl = vel_pred + K1 * innov;

  Pc[0][0] = Pp00 - K0 * Pp00;
  Pc[0][1] = Pp01 - K0 * Pp01;
  Pc[1][0] = Pp10 - K1 * Pp00;
  Pc[1][1] = Pp11 - K1 * Pp01;
}

void resetControlEstimator(float height_above_pad_now) {
  alt_ctrl = height_above_pad_now;

  vel_ctrl = velocity_est;  // Use current velocity estimate instead of 0.0

  Pc[0][0] = 50.f;
  Pc[0][1] = 0.f;
  Pc[1][0] = 0.f;
  Pc[1][1] = 50.f;
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}

  Wire.begin();
  Wire.setClock(400000);

  Serial.println("Init sensors...");

  if (!mpu.begin()) {
    Serial.println("MPU6050 not found");
    while (1) {}
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  if (!ms5611.begin()) {
    Serial.println("MS5611 not found");
    while (1) {}
  }

  // Init SD so file can be pre-opened (zero trigger latency)
  pinMode(10, OUTPUT);
  if (!sd.begin(SD_CS, SD_SCK_MHZ(18))) {
    Serial.println("SD init failed; will run serial-only.");
  }

  // Calibrate (stationary on table, BODY FRAME)
  Serial.println("Calibrating...");
  const int N = 200;
  double bx = 0, by = 0, bz = 0;
  double bgx = 0, bgy = 0, bgz = 0;
  double p_sum = 0;
  for (int i = 0; i < N; ++i) {
    sensors_event_t a, g, t;
    mpu.getEvent(&a, &g, &t);
    float ax, ay, az, gx, gy, gz;
    map_to_body(a, g, ax, ay, az, gx, gy, gz);
    bx += ax;
    by += ay;
    bz += az;
    bgx += gx;
    bgy += gy;
    bgz += gz;
    ms5611.read();
    p_sum += ms5611.getPressure();
    delay(5);
  }
  accel_offset_x = bx / N;
  accel_offset_y = by / N;
  accel_offset_z = (bz / N) - 9.80665f;  // remove gravity from UP (body Z)
  gyro_offset_x = bgx / N;
  gyro_offset_y = bgy / N;
  gyro_offset_z = bgz / N;
  pressure_offset = p_sum / N;

  Serial.println("Cal done.");

  // Pre-open log file to remove open() latency at trigger
  if (sd.card()) {
    if (sd_open_log()) {
      loggingActive = false;  // don't log until triggered
    }
  }

  // Serial header (matches SD header)
  Serial.println("Time(ms),Roll(deg),Pitch(deg),Yaw(deg),Est_Alt(m),Est_Vel(m/s),Alt_Ctrl(m),Vel_Ctrl(m/s),Raw_Alt(m),Ax(mps2),Ay(mps2),Az(mps2),Gx(dps),Gy(dps),Gz(dps),Height(m),Temp(C)");

  // Arm state
  Serial.println("Waiting for quiet pad to ARM...");
  armed = false;
  triggered = false;
  coastWindowActive = false;
  launchMs = 0;

  aZ_world_lpf = 0.0f;
  aZ_bias_slow = 0.0f;
  quietSinceMs = 0;
  aboveSinceMs = 0;

  Serial.println("Waiting 30s safety delay before arming enabled...");
  delay(30000);
  Serial.println("Arming enabled.");
  last_tick_us = micros();
}

void loop() {
  const uint32_t now_us = micros();
  if ((now_us - last_tick_us) < OUTPUT_PERIOD_US) return;

  dt = (now_us - last_tick_us) * 1e-6f;
  last_tick_us = now_us;

  // ----- Read sensors -----
  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);
  ms5611.read();
  const float pressure = ms5611.getPressure();        // Pa
  const float temperature = ms5611.getTemperature();  // °C
  const float raw_alt_m = 44330.0f * (1.0f - powf(pressure / 101325.0f, 0.1903f));

  // ---------- Time (ms) ----------
  const uint32_t now_ms = millis();

  // ----- Map to BODY frame -----
  float ax, ay, az, gx, gy, gz;
  map_to_body(a, g, ax, ay, az, gx, gy, gz);

  // ----- Orientation (DO NOT reset roll/pitch at coast start) -----
  complementaryFilter(ax, ay, az, gx, gy, gz, dt);

  // ----- KF (alt/vel) full-flight -----
  kalmanFilter(raw_alt_m, ax, ay, az, dt);

  // Height above pad (baro only)
  const float h0 = 44330.0f * (1.0f - powf(pressure_offset / 101325.0f, 0.1903f));
  const float height_above_pad = raw_alt_m - h0;

  // ---------- LAUNCH TRIGGER (world-frame vertical accel, robust) ----------

  // Body-frame accel with offsets removed (note: az_b ≈ +9.81 at rest)
  float ax_b = ax - accel_offset_x;
  float ay_b = ay - accel_offset_y;
  float az_b = az - accel_offset_z;

  // World-frame vertical accel (includes +g), then remove gravity
  float aZ_world = verticalAccelWorld(ax_b, ay_b, az_b, roll, pitch);
  float aZ_world_net = aZ_world - 9.80665f;

  // Low-pass filter vertical accel
  aZ_world_lpf = LPF_AZ_ALPHA * aZ_world_net + (1.0f - LPF_AZ_ALPHA) * aZ_world_lpf;

  // Quiet-pad detection for ARM
  float amag = sqrtf(ax_b * ax_b + ay_b * ay_b + az_b * az_b);  // includes +g
  bool accelQuiet = fabsf(amag - 9.80665f) < QUIET_AMAG_TOL;
  bool gyroQuiet = fabsf(gx - gyro_offset_x) < QUIET_GYRO_MAX_DPS && fabsf(gy - gyro_offset_y) < QUIET_GYRO_MAX_DPS && fabsf(gz - gyro_offset_z) < QUIET_GYRO_MAX_DPS;

  if (!armed && !triggered) {
    if (accelQuiet && gyroQuiet) {
      if (quietSinceMs == 0) quietSinceMs = now_ms;
      if ((now_ms - quietSinceMs) >= ARM_QUIET_MS) {
        armed = true;
        Serial.println("ARMED (pad quiet).");
      }
    } else {
      quietSinceMs = 0;
    }
  }

  // Auto-zero slow bias while quiet & not triggered
  if (!triggered && accelQuiet && gyroQuiet) {
    aZ_bias_slow = (1.0f - BIAS_ALPHA) * aZ_bias_slow + BIAS_ALPHA * aZ_world_lpf;
  }
  float aZ_for_trigger = aZ_world_lpf - aZ_bias_slow;

  // Trigger with debounce + hysteresis (logging still starts at launch)
  if (armed && !triggered) {
    if (aZ_for_trigger > TRIGGER_AZ_MPS2) {
      if (aboveSinceMs == 0) aboveSinceMs = now_ms;
      if ((now_ms - aboveSinceMs) >= TRIGGER_HOLD_MS) {
        triggered = true;
        armed = false;
        aboveSinceMs = 0;

        // Start logging immediately (UNCHANGED behavior)
        loggingActive = true;
        logStartMs = now_ms;

        // Coast-window timing
        launchMs = now_ms;
        coastWindowActive = false;

        Serial.println("TRIGGERED: launch detected. Logging started. Coast window at T+3.2s.");
      }
    } else {
      aboveSinceMs = 0;
    }
  }

  // ---------- START COAST WINDOW + RUN CONTROL ESTIMATOR ----------
  if (triggered && !coastWindowActive && (now_ms - launchMs) >= COAST_DELAY_MS) {
    coastWindowActive = true;
    resetControlEstimator(height_above_pad);
    Serial.println("COAST WINDOW ACTIVE: control estimator reset (roll/pitch kept).");
  }

  if (coastWindowActive) {
    kalmanControlUpdate(raw_alt_m, ax, ay, az, dt);
  }

  // ----- CSV line (mapped-minus-offset for accel/gyro outputs) -----
  char line[220];
  const int n = snprintf(line, sizeof(line),
                         "%lu,%.2f,%.2f,%.2f,"
                         "%.2f,%.2f,%.2f,%.2f,"
                         "%.2f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%.2f\n",
                         now_ms, roll, pitch, yaw,
                         altitude_est, velocity_est, alt_ctrl, vel_ctrl,
                         raw_alt_m,
                         ax - accel_offset_x,
                         ay - accel_offset_y,
                         az - accel_offset_z,
                         gx - gyro_offset_x,
                         gy - gyro_offset_y,
                         gz - gyro_offset_z,
                         height_above_pad, temperature);

  // Always print to serial
  Serial.write(line, n);

  // Write to SD only while loggingActive and file is open
  if (loggingActive && logFile) sd_write_buffered(line, (size_t)n);

  // ----- Auto-close after LOG_DURATION_MS -----
  if (loggingActive && (now_ms - logStartMs) >= LOG_DURATION_MS) {
    sd_flush_close();
    loggingActive = false;
    Serial.println("FLIGHT COMPLETE - 5 min logged to SD. File closed safely.");
  }
}


