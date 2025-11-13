/*
 * IMU Module - MPU-9250 Gyroscope Integration
 * Handles sensor initialization, calibration, and orientation tracking
 */

// MPU-9250 Register Definitions
#define MPU_ADDR 0x68
#define PWR_MGMT_1 0x6B
#define GYRO_XOUT_H 0x43

void initMPU() {
  Wire.begin(MPU_SDA, MPU_SCL);
  delay(100);
  
  // Wake up MPU-9250
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(PWR_MGMT_1);
  Wire.write(0x00);
  byte error = Wire.endTransmission();
  
  if (error != 0) {
    Serial.print("MPU init failed! Error: ");
    Serial.println(error);
  } else {
    Serial.println("MPU-9250 Ready");
  }
  delay(100);
}

void calibrateGyro() {
  Serial.println("Calibrating gyro... Keep still!");
  float sum_gx = 0, sum_gy = 0, sum_gz = 0;
  
  for (int i = 0; i < 200; i++) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(GYRO_XOUT_H);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 6);
    
    if (Wire.available() == 6) {
      int16_t gx = (Wire.read() << 8) | Wire.read();
      int16_t gy = (Wire.read() << 8) | Wire.read();
      int16_t gz = (Wire.read() << 8) | Wire.read();
      
      sum_gx += gx;
      sum_gy += gy;
      sum_gz += gz;
    }
    delay(10);
  }
  
  gx_offset = sum_gx / 200.0;
  gy_offset = sum_gy / 200.0;
  gz_offset = sum_gz / 200.0;
  
  Serial.println("Gyro calibration complete");
}

bool readGyro(float &gx, float &gy, float &gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(GYRO_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6);
  
  if (Wire.available() == 6) {
    int16_t raw_gx = (Wire.read() << 8) | Wire.read();
    int16_t raw_gy = (Wire.read() << 8) | Wire.read();
    int16_t raw_gz = (Wire.read() << 8) | Wire.read();
    
    // Apply calibration and convert to °/s
    gx = (raw_gx - gx_offset) / 131.0;
    gy = (raw_gy - gy_offset) / 131.0;
    gz = (raw_gz - gz_offset) / 131.0;
    
    return true;
  }
  return false;
}

void updateOrientation(float gx, float gy, float gz, float dt) {
  // Integrate gyro (all in degrees)
  roll += gx * dt;
  pitch -= gy * dt;
  yaw += gz * dt;
}

