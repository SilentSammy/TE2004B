#include <SPI.h>
#include <mcp_can.h>
#include <Wire.h>
#include "ble_client.h"

// Debug configuration
#define DEBUG_BLE false  // Set to true for verbose BLE control messages

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
#define LED_PIN 3  // Debug LED (changed from 8 to avoid SPI conflict)

// BLE UUIDs
#define SERVICE_UUID        "12345678-1234-5678-1234-56789abcdef0"
#define CHARACTERISTIC_UUID_LED      "12345678-1234-5678-1234-56789abcdef1"
#define CHARACTERISTIC_UUID_THROTTLE "12345678-1234-5678-1234-56789abcdef2"
#define CHARACTERISTIC_UUID_STEERING "12345678-1234-5678-1234-56789abcdef3"
#define CHARACTERISTIC_UUID_OMEGA    "12345678-1234-5678-1234-56789abcdef4"
#define DEVICE_NAME "BLE_Sensor_Hub"

// CAN configuration
MCP_CAN CAN0(CAN0_CS);                      // CAN object
const unsigned long TX_INTERVAL_MAX = 250;  // Always send every 250 ms
const unsigned long TX_INTERVAL_MIN = 1;    // Never send faster than 1 ms
const unsigned long ORIENT_INTERVAL = 20;   // Orientation CAN at 50Hz (20ms)
const unsigned long CONTROL_INTERVAL = 50;  // Control CAN at 20Hz (50ms)
const unsigned long ANGVEL_INTERVAL = 20;   // Angular velocity CAN at 50Hz (20ms)
const unsigned long ACCEL_INTERVAL = 20;    // Acceleration CAN at 50Hz (20ms)
const unsigned long CAMERA_INTERVAL = 50;   // Camera data CAN at 20Hz (50ms)
byte txData[8] = {0};                       // CAN message buffer (encoder)
byte orientData[8] = {0};                   // CAN message buffer (orientation)
byte controlData[8] = {0};                  // CAN message buffer (motor control)
byte angvelData[8] = {0};                   // CAN message buffer (angular velocity)
byte accelData[8] = {0};                    // CAN message buffer (linear acceleration)
byte cameraData[8] = {0};                   // CAN message buffer (camera position)
bool canInitialized = false;

// Encoder variables (functions in encoder.ino)
volatile long encoderPosition = 0;
volatile int lastEncoded = 0;
void updateEncoder();  // Forward declaration

// IMU variables (functions in imu.ino)
float gx_offset = 0, gy_offset = 0, gz_offset = 0;  // Gyro calibration offsets
float roll = 0, pitch = 0, yaw = 0;                 // Orientation (degrees)
unsigned long lastMpuTime = 0;
unsigned long lastOrientTxTime = 0;
unsigned long lastAngvelTxTime = 0;
unsigned long lastAccelTxTime = 0;

// BLE Camera Client variables
unsigned long lastCameraTxTime = 0;

// BLE Control variables
float currentThrottle = 0.0;  // -1.0 to 1.0
float currentSteering = 0.0;  // -1.0 to 1.0
float currentOmega = 0.0;     // -1.0 to 1.0
unsigned long lastControlTxTime = 0;
bool controlChanged = false;

// BLE Callback functions
void onLedControl(uint8_t value) {
  if (value > 0) {
    digitalWrite(LED_PIN, HIGH);
    if (DEBUG_BLE) Serial.println("LED: ON");
  } else {
    digitalWrite(LED_PIN, LOW);
    if (DEBUG_BLE) Serial.println("LED: OFF");
  }
}

void onThrottleControl(uint8_t value) {
  currentThrottle = toBipolar(value);
  controlChanged = true;
  if (DEBUG_BLE) {
    Serial.print("Throttle: ");
    Serial.println(currentThrottle, 3);
  }
}

void onSteeringControl(uint8_t value) {
  currentSteering = toBipolar(value);
  controlChanged = true;
  if (DEBUG_BLE) {
    Serial.print("Steering: ");
    Serial.println(currentSteering, 3);
  }
}

void onOmegaControl(uint8_t value) {
  currentOmega = toBipolar(value);
  controlChanged = true;
  if (DEBUG_BLE) {
    Serial.print("Omega: ");
    Serial.println(currentOmega, 3);
  }
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
  // NOTE: we send pitch in bytes 0-1 and roll in bytes 2-3 (swapped by request)
  orientData[0] = (byte)(pitch_enc);
  orientData[1] = (byte)(pitch_enc >> 8);
  orientData[2] = (byte)(roll_enc);
  orientData[3] = (byte)(roll_enc >> 8);
  orientData[4] = (byte)(yaw_enc);
  orientData[5] = (byte)(yaw_enc >> 8);
  orientData[6] = 0;
  orientData[7] = 0;
  
  // Send on CAN ID 0x124
  CAN0.sendMsgBuf(0x124, 0, 8, orientData);
}

void sendControlCAN() {
  // Convert float (-1.0 to +1.0) to int16_t (-1000 to +1000)
  int16_t throttle_enc = (int16_t)(currentThrottle * 1000);
  int16_t steering_enc = (int16_t)(currentSteering * 1000);
  int16_t omega_enc = (int16_t)(currentOmega * 1000);
  
  // Pack into CAN message (little-endian)
  controlData[0] = (byte)(throttle_enc);
  controlData[1] = (byte)(throttle_enc >> 8);
  controlData[2] = (byte)(steering_enc);
  controlData[3] = (byte)(steering_enc >> 8);
  controlData[4] = (byte)(omega_enc);
  controlData[5] = (byte)(omega_enc >> 8);
  controlData[6] = 0;
  controlData[7] = 0;
  
  // Send on CAN ID 0x125
  byte sndStat = CAN0.sendMsgBuf(0x125, 0, 8, controlData);
  
  if (DEBUG_BLE && sndStat == CAN_OK) {
    Serial.print("CAN Control TX - T:");
    Serial.print(currentThrottle, 2);
    Serial.print(" S:");
    Serial.print(currentSteering, 2);
    Serial.print(" W:");
    Serial.println(currentOmega, 2);
  }
}

void sendAngularVelocityCAN(float gx, float gy, float gz) {
  // Convert °/s to int16_t (±1000 = ±1000 °/s)
  int16_t gx_enc = (int16_t)(gx * 10);
  int16_t gy_enc = (int16_t)(gy * 10);
  int16_t gz_enc = (int16_t)(gz * 10);
  
  // Pack into CAN message (little-endian)
  angvelData[0] = (byte)(gx_enc);
  angvelData[1] = (byte)(gx_enc >> 8);
  angvelData[2] = (byte)(gy_enc);
  angvelData[3] = (byte)(gy_enc >> 8);
  angvelData[4] = (byte)(gz_enc);
  angvelData[5] = (byte)(gz_enc >> 8);
  angvelData[6] = 0;
  angvelData[7] = 0;
  
  // Send on CAN ID 0x126
  CAN0.sendMsgBuf(0x126, 0, 8, angvelData);
}

void sendLinearAccelCAN(float ax, float ay, float az) {
  // Convert m/s² to int16_t (±100 = ±10.0 m/s²)
  int16_t ax_enc = (int16_t)(ax * 100);
  int16_t ay_enc = (int16_t)(ay * 100);
  int16_t az_enc = (int16_t)(az * 100);
  
  // Pack into CAN message (little-endian)
  accelData[0] = (byte)(ax_enc);
  accelData[1] = (byte)(ax_enc >> 8);
  accelData[2] = (byte)(ay_enc);
  accelData[3] = (byte)(ay_enc >> 8);
  accelData[4] = (byte)(az_enc);
  accelData[5] = (byte)(az_enc >> 8);
  accelData[6] = 0;
  accelData[7] = 0;
  
  // Send on CAN ID 0x127
  CAN0.sendMsgBuf(0x127, 0, 8, accelData);
}

void sendCameraPositionCAN(const uint8_t* data) {
  // Copy camera data to CAN buffer
  for (int i = 0; i < 8; i++) {
    cameraData[i] = data[i];
  }
  
  // Send on CAN ID 0x128
  byte sndStat = CAN0.sendMsgBuf(0x128, 0, 8, cameraData);
  
  if (sndStat == CAN_OK) {
    Serial.print("Camera CAN TX: ");
    for (int i = 0; i < 8; i++) {
      Serial.print("0x");
      if (cameraData[i] < 0x10) Serial.print("0");
      Serial.print(cameraData[i], HEX);
      if (i < 7) Serial.print(" ");
    }
    Serial.println();
  }
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
  Serial.println("\nESP32-C3 Sensor Hub (Encoder + IMU + BLE → CAN)");
  Serial.println("================================================");

  // LED setup
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

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
  
  // BLE setup
  Serial.println("Initializing BLE...");
  BLE_init(DEVICE_NAME, SERVICE_UUID);
  BLE_addCharacteristic(CHARACTERISTIC_UUID_LED, onLedControl);
  BLE_addCharacteristic(CHARACTERISTIC_UUID_THROTTLE, onThrottleControl);
  BLE_addCharacteristic(CHARACTERISTIC_UUID_STEERING, onSteeringControl);
  BLE_addCharacteristic(CHARACTERISTIC_UUID_OMEGA, onOmegaControl);
  BLE_start(DEVICE_NAME);
  
  // BLE Camera Client setup
  Serial.println("Initializing BLE Camera Client...");
  bleClientInit();
  
  Serial.println("System Ready!");
  Serial.println();
  
  lastMpuTime = millis();
}

void loop2() {
  float gx, gy, gz, ax, ay, az;
  
  if (readGyro(gx, gy, gz) && readAccel(ax, ay, az)) {
    Serial.print("Gyro: ");
    Serial.print(gx, 2); Serial.print(" ");
    Serial.print(gy, 2); Serial.print(" ");
    Serial.print(gz, 2); Serial.print(" | Accel: ");
    Serial.print(ax, 2); Serial.print(" ");
    Serial.print(ay, 2); Serial.print(" ");
    Serial.println(az, 2);
  }
  
  delay(100);
}

void loop() {
  if (!canInitialized) return;

  unsigned long now = millis();
  
  // ========== BLE CAMERA CLIENT UPDATE ==========
  bleClientUpdate();
  
  // ========== CAMERA DATA CAN TX (Event + Periodic) ==========
  if (bleClientIsConnected()) {
    bool cameraDueByTimeout = (now - lastCameraTxTime >= CAMERA_INTERVAL);
    
    if (bleClientHasNewData() || cameraDueByTimeout) {
      lastCameraTxTime = now;
      uint8_t cameraBuffer[8];
      if (bleClientGetData(cameraBuffer)) {
        sendCameraPositionCAN(cameraBuffer);
      }
    }
  }
  
  // ========== IMU READS (Every loop, ~100Hz) ==========
  float dt = (now - lastMpuTime) / 1000.0;
  lastMpuTime = now;
  
  float gx, gy, gz, ax, ay, az;
  bool hasGyro = readGyro(gx, gy, gz);
  bool hasAccel = readAccel(ax, ay, az);
  
  if (hasGyro) {
    updateOrientation(gx, gy, gz, dt);
  }
  
  // ========== ORIENTATION CAN TX (Fixed 50Hz) ==========
  if (now - lastOrientTxTime >= ORIENT_INTERVAL) {
    lastOrientTxTime = now;
    sendOrientationCAN();
  }
  
  // ========== ANGULAR VELOCITY CAN TX (Fixed 50Hz) ==========
  if (hasGyro && now - lastAngvelTxTime >= ANGVEL_INTERVAL) {
    lastAngvelTxTime = now;
    sendAngularVelocityCAN(gx, gy, gz);
  }
  
  // ========== LINEAR ACCEL CAN TX (Fixed 50Hz) ==========
  if (hasAccel && now - lastAccelTxTime >= ACCEL_INTERVAL) {
    lastAccelTxTime = now;
    sendLinearAccelCAN(ax, ay, az);
  }
  
  // ========== CONTROL CAN TX (Event + Periodic) ==========
  bool controlDueByTimeout = (now - lastControlTxTime >= CONTROL_INTERVAL);
  
  if (controlChanged || controlDueByTimeout) {
    lastControlTxTime = now;
    controlChanged = false;
    sendControlCAN();
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
