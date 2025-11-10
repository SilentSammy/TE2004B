#include <SPI.h>
#include <mcp_can.h>
#include <Wire.h>

// Pin definitions
#define CAN0_INT  2
#define CAN0_CS   4
#define SPI_SCK   8
#define SPI_MISO  9
#define SPI_MOSI 10
#define ENC_A 0
#define ENC_B 1
#define MPU_SDA 6
#define MPU_SCL 7

// MPU-9250 definitions
#define MPU_ADDR 0x68
#define PWR_MGMT_1 0x6B
#define GYRO_XOUT_H 0x43

//  CAN configuration
MCP_CAN CAN0(CAN0_CS);                      // CAN object
const unsigned long TX_INTERVAL_MAX = 250;  // Always send every 250 ms
const unsigned long TX_INTERVAL_MIN = 1;    // Never send faster than 1 ms
const unsigned long ORIENT_INTERVAL = 20;   // Orientation CAN at 50Hz (20ms)
byte txData[8] = {0};                       // CAN message buffer (encoder)
byte orientData[8] = {0};                   // CAN message buffer (orientation)
bool canInitialized = false;

//  Encoder setup
volatile long encoderPosition = 0;
volatile int lastEncoded = 0;
void IRAM_ATTR updateEncoder() {
  int MSB = digitalRead(ENC_A);
  int LSB = digitalRead(ENC_B);
  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncoded << 2) | encoded;

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011)
    encoderPosition++;
  else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)
    encoderPosition--;

  lastEncoded = encoded;
}

// MPU-9250 variables
float gx_offset = 0, gy_offset = 0, gz_offset = 0;  // Gyro calibration offsets
float roll = 0, pitch = 0, yaw = 0;                 // Orientation (degrees, internal -180 to +180)
unsigned long lastMpuTime = 0;
unsigned long lastOrientTxTime = 0;

// MPU Functions
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

void sendOrientationCAN() {
  // Normalize angles to 0-360° range
  float roll_norm = roll;
  float pitch_norm = pitch;
  float yaw_norm = yaw;
  
  while (roll_norm < 0) roll_norm += 360;
  while (roll_norm >= 360) roll_norm -= 360;
  while (pitch_norm < 0) pitch_norm += 360;
  while (pitch_norm >= 360) pitch_norm -= 360;
  while (yaw_norm < 0) yaw_norm += 360;
  while (yaw_norm >= 360) yaw_norm -= 360;
  
  // Encode as uint16_t (0-36000 = 0.00-360.00°)
  uint16_t roll_enc = (uint16_t)(roll_norm * 100);
  uint16_t pitch_enc = (uint16_t)(pitch_norm * 100);
  uint16_t yaw_enc = (uint16_t)(yaw_norm * 100);
  
  // Pack into CAN message (little-endian)
  orientData[0] = (byte)(roll_enc);
  orientData[1] = (byte)(roll_enc >> 8);
  orientData[2] = (byte)(pitch_enc);
  orientData[3] = (byte)(pitch_enc >> 8);
  orientData[4] = (byte)(yaw_enc);
  orientData[5] = (byte)(yaw_enc >> 8);
  orientData[6] = 0;
  orientData[7] = 0;
  
  // Send on CAN ID 0x124
  CAN0.sendMsgBuf(0x124, 0, 8, orientData);
}

// Helpers
inline void fastStatusPrint(bool sent, long pos) {
  Serial.write(sent ? 'Y' : 'N');  // 1 char: Y or N
  Serial.print(pos);               // encoder position digits
  Serial.print("\r\n");            // CRLF line ending
}

// Main program
void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  Serial.println("\nESP32-C3 Sensor Hub (Encoder + IMU → CAN)");
  Serial.println("==========================================");

  // Encoder setup
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), updateEncoder, CHANGE);

  // SPI & CAN setup
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    Serial.println("MCP2515 Initialized Successfully!");
    canInitialized = true;
    CAN0.setMode(MCP_NORMAL); // Normal mode
  } else {
    Serial.println("Error Initializing MCP2515...");
  }
  pinMode(CAN0_INT, INPUT);

  // MPU-9250 setup
  initMPU();
  calibrateGyro();
  
  Serial.println("System Ready!");
  Serial.println();
  
  lastMpuTime = millis();
}

void loop() {
  if (!canInitialized) return;

  unsigned long now = millis();
  
  // ========== ORIENTATION UPDATE (Every loop, ~100Hz) ==========
  float dt = (now - lastMpuTime) / 1000.0;
  lastMpuTime = now;
  
  float gx, gy, gz;
  if (readGyro(gx, gy, gz)) {
    updateOrientation(gx, gy, gz, dt);
  }
  
  // ========== ORIENTATION CAN TX (Fixed 50Hz) ==========
  if (now - lastOrientTxTime >= ORIENT_INTERVAL) {
    lastOrientTxTime = now;
    sendOrientationCAN();
  }
  
  // ========== ENCODER CAN TX (Variable rate) ==========
  static unsigned long lastTxTime = 0;
  static long lastSentPos = 0;

  // Safely read encoder count
  long pos;
  noInterrupts();
  pos = encoderPosition;
  interrupts();

  // Timing and change detection
  bool changed = (pos != lastSentPos);
  bool tooSoon = (now - lastTxTime < TX_INTERVAL_MIN);
  bool dueByTimeout = (now - lastTxTime >= TX_INTERVAL_MAX);

  if ((changed && !tooSoon) || dueByTimeout) {
    lastTxTime = now;
    lastSentPos = pos;

    // Encode position as 4-byte integer (little-endian)
    txData[0] = (byte)(pos);
    txData[1] = (byte)(pos >> 8);
    txData[2] = (byte)(pos >> 16);
    txData[3] = (byte)(pos >> 24);
    for (int i = 4; i < 8; i++) txData[i] = 0;

    // Send CAN frame and print result
    byte sndStat = CAN0.sendMsgBuf(0x123, 0, 8, txData);
    fastStatusPrint(sndStat == CAN_OK, pos);
  }
  
  delay(10);  // 100Hz loop
}
