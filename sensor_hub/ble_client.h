// ========== BLE Client for Camera Position Data ==========
// This file provides BLE client functionality to receive camera
// position data and make it available to the main sketch.

#ifndef BLE_CLIENT_H
#define BLE_CLIENT_H

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

// ========== BLE CONFIGURATION ==========
#define CAMERA_SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CAMERA_CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// ========== STATE VARIABLES ==========
static BLEAddress *pServerAddress = nullptr;
static bool doConnect = false;
static bool bleClientConnected = false;
static bool scanCompleted = false;  // Scan only once at startup
static BLERemoteCharacteristic* pRemoteCharacteristic = nullptr;
static unsigned long lastDataReceivedTime = 0;

// Store latest received camera data
static uint8_t latestCameraData[8] = {0};
static bool newDataAvailable = false;

// ========== BLE CALLBACKS ==========
class CameraClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) {
    Serial.println("✓ BLE: Connected to camera server");
  }

  void onDisconnect(BLEClient* pclient) {
    bleClientConnected = false;
    Serial.println("✗ BLE: Disconnected from camera server");
  }
};

// Notification Callback - Receives camera position data
static void cameraNotifyCallback(
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
    Serial.print("⚠ BLE: Unexpected length: ");
    Serial.print(length);
    Serial.println(" bytes (expected 8)");
  }
}

// Advertised Device Callback
class CameraAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    Serial.print("BLE: Found device: ");
    Serial.println(advertisedDevice.toString().c_str());

    if (advertisedDevice.haveServiceUUID() &&
        advertisedDevice.getServiceUUID().equals(BLEUUID(CAMERA_SERVICE_UUID))) {
      
      advertisedDevice.getScan()->stop();
      pServerAddress = new BLEAddress(advertisedDevice.getAddress());
      doConnect = true;
      Serial.println("✓ BLE: Target camera server found!");
    }
  }
};

// ========== INTERNAL CONNECTION FUNCTION ==========
static bool connectToCameraServer() {
  Serial.print("BLE: Connecting to ");
  Serial.println(pServerAddress->toString().c_str());

  BLEClient* pClient = BLEDevice::createClient();
  pClient->setClientCallbacks(new CameraClientCallback());

  if (!pClient->connect(*pServerAddress)) {
    Serial.println("BLE: Connection failed");
    return false;
  }
  Serial.println("BLE: Connected to server");

  BLERemoteService* pRemoteService = pClient->getService(CAMERA_SERVICE_UUID);
  if (pRemoteService == nullptr) {
    Serial.println("BLE: Failed to find service");
    pClient->disconnect();
    return false;
  }

  pRemoteCharacteristic = pRemoteService->getCharacteristic(CAMERA_CHARACTERISTIC_UUID);
  if (pRemoteCharacteristic == nullptr) {
    Serial.println("BLE: Failed to find characteristic");
    pClient->disconnect();
    return false;
  }

  if(pRemoteCharacteristic->canNotify()) {
    pRemoteCharacteristic->registerForNotify(cameraNotifyCallback);
    Serial.println("BLE: Registered for notifications");
  }

  bleClientConnected = true;
  return true;
}

// ========== PUBLIC API FUNCTIONS ==========

/**
 * @brief Initialize BLE camera client (call after BLE server is initialized)
 */
void bleClientInit() {
  Serial.println("BLE Camera Client: Starting one-time scan...");

  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new CameraAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99);

  // Perform one-time scan (10 seconds)
  Serial.println("BLE: Scanning for 10 seconds...");
  pBLEScan->start(10, false);
  scanCompleted = true;
  
  if (doConnect) {
    Serial.println("✓ BLE Camera Client: Server found during scan");
  } else {
    Serial.println("⚠ BLE Camera Client: No server found (will not retry)");
  }

  Serial.println("✓ BLE Camera Client: Initialized");
}

/**
 * @brief Update BLE connection state (call in loop)
 */
void bleClientUpdate() {
  // Only attempt connection once if server was found during initial scan
  if (doConnect && !bleClientConnected) {
    if (connectToCameraServer()) {
      Serial.println("✓ BLE: Ready to receive camera data");
    } else {
      Serial.println("✗ BLE: Connection failed");
    }
    doConnect = false;
  }
  
  // No data warning
  if (bleClientConnected && lastDataReceivedTime > 0) {
    static unsigned long noDataWarningTime = 0;
    unsigned long now = millis();
    if (now - lastDataReceivedTime > 10000 && 
        now - noDataWarningTime > 10000) {
      Serial.println("⚠ BLE: No camera data in 10 seconds");
      noDataWarningTime = now;
    }
  }
}

/**
 * @brief Check if BLE is connected to camera server
 * @return true if connected
 */
bool bleClientIsConnected() {
  return bleClientConnected;
}

/**
 * @brief Check if new camera data is available
 * @return true if new data waiting
 */
bool bleClientHasNewData() {
  return newDataAvailable;
}

/**
 * @brief Get latest camera data and clear new data flag
 * @param data Buffer to copy 8 bytes of camera data
 * @return true if data was copied
 */
bool bleClientGetData(uint8_t* data) {
  if (data == nullptr) return false;
  
  for (int i = 0; i < 8; i++) {
    data[i] = latestCameraData[i];
  }
  
  newDataAvailable = false;
  return true;
}

/**
 * @brief Get time since last data received
 * @return milliseconds since last data (0 if never received)
 */
unsigned long bleClientTimeSinceLastData() {
  if (lastDataReceivedTime == 0) return 0;
  return millis() - lastDataReceivedTime;
}

#endif // BLE_CLIENT_H
