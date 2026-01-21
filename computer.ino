// Test to check which libraries are installed
// Try uncommenting different include combinations to see what works

// Option 1: Adafruit Libraries (most common)
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <MS5611.h>

// Create sensor objects
Adafruit_MPU6050 mpu;
MS5611 ms5611;

/* 
// Option 2: If Adafruit doesn't work, try this instead:
#include <Wire.h>
#include <MPU6050_light.h>  // Different MPU6050 library
#include <MS5611.h>

MPU6050 mpu(Wire);
MS5611 ms5611;
*/

/* 
// Option 3: If MS5611 doesn't work, try:
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
// Comment out MS5611 for now

Adafruit_MPU6050 mpu;
*/

// Timing variables for 100Hz output
unsigned long lastPrintTime = 0;
const unsigned long printInterval = 10000; // 10ms = 100Hz

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);
  
  while (!Serial) {
    delay(10);
  }
  
  Serial.println("Testing sensor initialization...");
  
  // Test MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    Serial.println("Check wiring and library installation");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
  
  // Configure MPU6050
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  
  // Test MS5611
  if (!ms5611.begin()) {
    Serial.println("Could not find MS5611 sensor");
    Serial.println("Check wiring and library installation");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MS5611 Found!");
  
  delay(100);
  Serial.println("Time(ms),Ax(g),Ay(g),Az(g),Gx(deg/s),Gy(deg/s),Gz(deg/s),Pressure(Pa),Temp(C),Altitude(m)");
  
  lastPrintTime = micros();
}

void loop() {
  unsigned long currentTime = micros();
  
  if (currentTime - lastPrintTime >= printInterval) {
    
    // Read MPU6050 data
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    
    // Read MS5611 data
    int result = ms5611.read();  // Read both pressure and temperature
    float pressure = ms5611.getPressure();
    float temperature = ms5611.getTemperature();
    
    // Calculate altitude manually (standard atmosphere)
    float altitude = 44330.0 * (1.0 - pow(pressure / 101325.0, 0.1903));
    
    // Print all data
    Serial.print(millis());
    Serial.print(",");
    Serial.print(a.acceleration.x, 3);
    Serial.print(",");
    Serial.print(a.acceleration.y, 3);
    Serial.print(",");
    Serial.print(a.acceleration.z, 3);
    Serial.print(",");
    Serial.print(g.gyro.x, 3);
    Serial.print(",");
    Serial.print(g.gyro.y, 3);
    Serial.print(",");
    Serial.print(g.gyro.z, 3);
    Serial.print(",");
    Serial.print(pressure, 2);
    Serial.print(",");
    Serial.print(temperature, 2);
    Serial.print(",");
    Serial.println(altitude, 2);
    
    lastPrintTime = currentTime;
  }
}

