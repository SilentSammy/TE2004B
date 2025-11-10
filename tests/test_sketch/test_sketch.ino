// Minimal Gyro Integration (Calibration + Orientation)
#include <Wire.h>

#define SDA_PIN 6
#define SCL_PIN 7
#define MPU_ADDR 0x68
#define PWR_MGMT_1 0x6B
#define GYRO_XOUT_H 0x43

// Gyro offsets
float gx_offset = 0, gy_offset = 0, gz_offset = 0;

// Orientation (degrees)
float roll = 0, pitch = 0, yaw = 0;

// Timing
unsigned long lastTime = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  // Init I2C and wake MPU
  Wire.begin(SDA_PIN, SCL_PIN);
  delay(100);
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(PWR_MGMT_1);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(100);
  
  // Calibrate gyro
  Serial.println("Calibrating... Keep still!");
  float sum_gx = 0, sum_gy = 0, sum_gz = 0;
  
  for (int i = 0; i < 200; i++) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(GYRO_XOUT_H);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 6);
    
    int16_t gx = (Wire.read() << 8) | Wire.read();
    int16_t gy = (Wire.read() << 8) | Wire.read();
    int16_t gz = (Wire.read() << 8) | Wire.read();
    
    sum_gx += gx;
    sum_gy += gy;
    sum_gz += gz;
    delay(10);
  }
  
  gx_offset = sum_gx / 200.0;
  gy_offset = sum_gy / 200.0;
  gz_offset = sum_gz / 200.0;
  
  Serial.println("Ready");
  lastTime = millis();
}

void loop() {
  static unsigned long lastPrint = 0;
  
  // Read gyro
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(GYRO_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6);
  
  int16_t raw_gx = (Wire.read() << 8) | Wire.read();
  int16_t raw_gy = (Wire.read() << 8) | Wire.read();
  int16_t raw_gz = (Wire.read() << 8) | Wire.read();
  
  // Apply calibration and convert to °/s
  float gx = (raw_gx - gx_offset) / 131.0;
  float gy = (raw_gy - gy_offset) / 131.0;
  float gz = (raw_gz - gz_offset) / 131.0;
  
  // Calculate dt
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;
  
  // Integrate gyro
  roll += gx * dt;
  pitch -= gy * dt;
  yaw += gz * dt;
  
  // Print every 500ms
  if (currentTime - lastPrint >= 500) {
    Serial.print(roll, 1);
    Serial.print(" ");
    Serial.print(pitch, 1);
    Serial.print(" ");
    Serial.println(yaw, 1);
    lastPrint = currentTime;
  }
  
  delay(10);  // 100Hz
}
