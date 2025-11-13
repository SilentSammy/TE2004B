#include <SPI.h>
#include <mcp_can.h>
#include <Wire.h>

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
#define DEVICE_NAME "BLE_Sensor_Hub"

// CAN configuration
MCP_CAN CAN0(CAN0_CS);                      // CAN object
const unsigned long TX_INTERVAL_MAX = 250;  // Always send every 250 ms
const unsigned long TX_INTERVAL_MIN = 1;    // Never send faster than 1 ms
const unsigned long ORIENT_INTERVAL = 20;   // Orientation CAN at 50Hz (20ms)
const unsigned long CONTROL_INTERVAL = 50;  // Control CAN at 20Hz (50ms)
byte txData[8] = {0};                       // CAN message buffer (encoder)
byte orientData[8] = {0};                   // CAN message buffer (orientation)
byte controlData[8] = {0};                  // CAN message buffer (motor control)
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

// BLE Control variables
float currentThrottle = 0.0;  // -1.0 to 1.0
float currentSteering = 0.0;  // -1.0 to 1.0
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
  
  // Pack into CAN message (little-endian)
  controlData[0] = (byte)(throttle_enc);
  controlData[1] = (byte)(throttle_enc >> 8);
  controlData[2] = (byte)(steering_enc);
  controlData[3] = (byte)(steering_enc >> 8);
  controlData[4] = 0;
  controlData[5] = 0;
  controlData[6] = 0;
  controlData[7] = 0;
  
  // Send on CAN ID 0x125
  byte sndStat = CAN0.sendMsgBuf(0x125, 0, 8, controlData);
  
  if (DEBUG_BLE && sndStat == CAN_OK) {
    Serial.print("CAN Control TX - T:");
    Serial.print(currentThrottle, 2);
    Serial.print(" S:");
    Serial.println(currentSteering, 2);
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
  BLE_start(DEVICE_NAME);
  
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
