#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <SPI.h>
#include <mcp_can.h>

// ========== CAN CONFIGURATION ==========
#define CAN0_INT  2
#define CAN0_CS   4
#define SPI_SCK   8
#define SPI_MISO  9
#define SPI_MOSI 10

MCP_CAN CAN0(CAN0_CS);
bool canInitialized = false;

// ========== FAKE DATA CONFIGURATION ==========
#define ENABLE_FAKE_DATA false 
unsigned long lastFakeDataTime = 0;
const unsigned long FAKE_DATA_INTERVAL = 100;  // Generate fake data every 100ms

// CAN message buffer for camera position data
byte cameraData[8] = {0};
const unsigned long CAN_TX_INTERVAL = 50;  // Send CAN every 50ms (20Hz)
unsigned long lastCanTxTime = 0;

// ========== BLE CONFIGURATION ==========
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

static BLEAddress *pServerAddress;
static bool doConnect = false;
static bool connected = false;
static BLERemoteCharacteristic* pRemoteCharacteristic;
static unsigned long lastDataReceivedTime = 0;

// Store latest received camera data
uint8_t latestCameraData[8] = {0};
bool newDataAvailable = false;

// ========== FAKE DATA FUNCTIONS ==========
void generateFakeData() {
  // Generate fake 8-byte camera position data for testing
  // Format: [x_high, x_low, y_high, y_low, width_high, width_low, height_high, height_low]
  static uint16_t x_pos = 160;      // Starting at center-ish (320/2)
  static uint16_t y_pos = 120;      // Starting at center-ish (240/2)
  static uint16_t width = 50;       // Box width
  static uint16_t height = 50;      // Box height
  static int8_t x_dir = 1;
  static int8_t y_dir = 1;
  
  // Simulate moving object (bouncing box)
  x_pos += x_dir * 5;
  y_pos += y_dir * 3;
  
  // Bounce at boundaries (assuming 320x240 camera)
  if (x_pos > 270 || x_pos < 10) x_dir *= -1;
  if (y_pos > 190 || y_pos < 10) y_dir *= -1;
  
  // Vary size slightly
  width = 40 + (millis() / 500) % 30;
  height = 40 + (millis() / 700) % 30;
  
  // Pack into 8-byte array (big-endian format)
  latestCameraData[0] = (x_pos >> 8) & 0xFF;   // x high byte
  latestCameraData[1] = x_pos & 0xFF;           // x low byte
  latestCameraData[2] = (y_pos >> 8) & 0xFF;   // y high byte
  latestCameraData[3] = y_pos & 0xFF;           // y low byte
  latestCameraData[4] = (width >> 8) & 0xFF;   // width high byte
  latestCameraData[5] = width & 0xFF;           // width low byte
  latestCameraData[6] = (height >> 8) & 0xFF;  // height high byte
  latestCameraData[7] = height & 0xFF;          // height low byte
  
  newDataAvailable = true;
  
  // Display fake data
  Serial.println("=== FAKE DATA ===");
  Serial.print("Position: (");
  Serial.print(x_pos);
  Serial.print(", ");
  Serial.print(y_pos);
  Serial.print(") Size: ");
  Serial.print(width);
  Serial.print("x");
  Serial.println(height);
  Serial.print("Hex: ");
  for (int i = 0; i < 8; i++) {
    Serial.print("0x");
    if (latestCameraData[i] < 0x10) Serial.print("0");
    Serial.print(latestCameraData[i], HEX);
    if (i < 7) Serial.print(" ");
  }
  Serial.println();
  Serial.println("=================");
}

// ========== CAN FUNCTIONS ==========
void sendCameraPositionCAN() {
  if (!canInitialized) return;
  
  // Copy latest camera data to CAN buffer
  for (int i = 0; i < 8; i++) {
    cameraData[i] = latestCameraData[i];
  }
  
  // Send CAN message on ID 0x128
  byte sndStat = CAN0.sendMsgBuf(0x128, 0, 8, cameraData);
  
  if (sndStat == CAN_OK) {
    Serial.print("CAN TX [0x128]: ");
    for (int i = 0; i < 8; i++) {
      Serial.print("0x");
      if (cameraData[i] < 0x10) Serial.print("0");
      Serial.print(cameraData[i], HEX);
      if (i < 7) Serial.print(" ");
    }
    Serial.println();
  } else {
    Serial.println("CAN TX FAILED");
  }
}

// ========== BLE CALLBACKS ==========
class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) {
    Serial.println("✓ Connected to BLE server");
  }

  void onDisconnect(BLEClient* pclient) {
    connected = false;
    Serial.println("✗ Disconnected from BLE server");
  }
};

// Notification Callback - Receives camera position data
static void notifyCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) {
  
  lastDataReceivedTime = millis();
  
  if (length == 8) {
    // Store received data
    for (int i = 0; i < 8; i++) {
      latestCameraData[i] = pData[i];
    }
    newDataAvailable = true;
    
    // Display received bytes
    Serial.println("=== BLE RX ===");
    Serial.print("Hex: ");
    for (int i = 0; i < 8; i++) {
      Serial.print("0x");
      if (pData[i] < 0x10) Serial.print("0");
      Serial.print(pData[i], HEX);
      if (i < 7) Serial.print(" ");
    }
    Serial.println();
    
    Serial.print("Dec: ");
    for (int i = 0; i < 8; i++) {
      Serial.print(pData[i]);
      if (i < 7) Serial.print(" ");
    }
    Serial.println();
    Serial.println("==============");
    
  } else {
    Serial.print("⚠ Unexpected length: ");
    Serial.print(length);
    Serial.println(" bytes (expected 8)");
  }
}

// Advertised Device Callback
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    Serial.print("Found device: ");
    Serial.println(advertisedDevice.toString().c_str());

    if (advertisedDevice.haveServiceUUID() &&
        advertisedDevice.getServiceUUID().equals(BLEUUID(SERVICE_UUID))) {
      
      advertisedDevice.getScan()->stop();
      pServerAddress = new BLEAddress(advertisedDevice.getAddress());
      doConnect = true;
      Serial.println("✓ Target camera server found!");
    }
  }
};

bool connectToServer() {
  Serial.print("Connecting to ");
  Serial.println(pServerAddress->toString().c_str());

  BLEClient* pClient = BLEDevice::createClient();
  pClient->setClientCallbacks(new MyClientCallback());

  if (!pClient->connect(*pServerAddress)) {
    Serial.println("Connection failed");
    return false;
  }
  Serial.println("Connected to server");

  BLERemoteService* pRemoteService = pClient->getService(SERVICE_UUID);
  if (pRemoteService == nullptr) {
    Serial.println("Failed to find service");
    pClient->disconnect();
    return false;
  }

  pRemoteCharacteristic = pRemoteService->getCharacteristic(CHARACTERISTIC_UUID);
  if (pRemoteCharacteristic == nullptr) {
    Serial.println("Failed to find characteristic");
    pClient->disconnect();
    return false;
  }

  if(pRemoteCharacteristic->canNotify()) {
    pRemoteCharacteristic->registerForNotify(notifyCallback);
    Serial.println("Registered for notifications");
  }

  connected = true;
  return true;
}

// ========== SETUP ==========
void setup() {
  Serial.begin(115200);
  delay(2000);  // Give USB CDC time to initialize
  
  Serial.println("\n================================");
  Serial.println("BLE Camera → CAN Bridge");
  Serial.println("CAN ID: 0x128");
  if (ENABLE_FAKE_DATA) {
    Serial.println("⚠ FAKE DATA MODE ENABLED");
  }
  Serial.println("================================\n");

  // Initialize CAN
  Serial.println("Initializing CAN...");
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  
  if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    Serial.println("✓ MCP2515 Initialized Successfully!");
    canInitialized = true;
    CAN0.setMode(MCP_NORMAL);
  } else {
    Serial.println("✗ Error Initializing MCP2515");
  }
  pinMode(CAN0_INT, INPUT);
  
  // Initialize BLE
  Serial.println("Initializing BLE...");
  BLEDevice::init("ESP32-C3-CAN-Bridge");

  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99);

  Serial.println("✓ System Ready!");
  Serial.println("Scanning for camera server...\n");
}

// ========== MAIN LOOP ==========
void loop() {
  unsigned long now = millis();
  
  // ===== BLE CONNECTION MANAGEMENT =====
  if (!connected && !doConnect) {
    BLEScan* pBLEScan = BLEDevice::getScan();
    pBLEScan->start(5, false);
    Serial.println("Scanning...");
  }

  if (doConnect && !connected) {
    if (connectToServer()) {
      Serial.println("✓ Ready to receive camera data\n");
    } else {
      Serial.println("✗ Connection failed");
    }
    doConnect = false;
  }

  // ===== FAKE DATA GENERATION (when not connected) =====
  if (!connected && ENABLE_FAKE_DATA && canInitialized) {
    if (now - lastFakeDataTime >= FAKE_DATA_INTERVAL) {
      lastFakeDataTime = now;
      generateFakeData();
    }
  }
  
  // ===== CAN TRANSMISSION =====
  if (canInitialized && (connected || ENABLE_FAKE_DATA)) {
    // Send CAN every 50ms OR immediately when new data arrives
    bool dueByTimeout = (now - lastCanTxTime >= CAN_TX_INTERVAL);
    
    if (newDataAvailable || dueByTimeout) {
      lastCanTxTime = now;
      newDataAvailable = false;
      sendCameraPositionCAN();
    }
    
    // No data warning (only when connected to real BLE)
    if (connected) {
      static unsigned long noDataWarningTime = 0;
      if (lastDataReceivedTime > 0 && 
          now - lastDataReceivedTime > 10000 && 
          now - noDataWarningTime > 10000) {
        Serial.println("⚠ No camera data in 10 seconds");
        noDataWarningTime = now;
      }
    }
    
  } else if (!connected && !ENABLE_FAKE_DATA) {
    delay(2000);
  }
  
  delay(10);
}