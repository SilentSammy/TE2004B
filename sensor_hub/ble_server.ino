/*
 * BLE Server Abstraction
 * Handles all BLE server setup and management
 */

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// Internal server variables
static BLEServer* pServer = NULL;
static BLEService* pService = NULL;
static bool deviceConnected = false;

// Callback storage
#define MAX_CALLBACKS 10
struct CallbackEntry {
  const char* charUUID;
  void (*callback)(uint8_t*, size_t);
  BLECharacteristic* pChar;
};
static CallbackEntry callbacks[MAX_CALLBACKS];
static int callbackCount = 0;

// Server connection callbacks
class ServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      Serial.println("Client connected");
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      Serial.println("Client disconnected");
      BLEDevice::startAdvertising();
      Serial.println("Advertising restarted");
    }
};

// Characteristic write callbacks
class CharCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      uint8_t* pData = pCharacteristic->getData();
      size_t len = pCharacteristic->getValue().length();
      
      if (len > 0) {
        // Find and call the registered callback with full data array
        for (int i = 0; i < callbackCount; i++) {
          if (callbacks[i].pChar == pCharacteristic && callbacks[i].callback != NULL) {
            callbacks[i].callback(pData, len);
            break;
          }
        }
      }
    }
};

void BLE_init(const char* deviceName, const char* serviceUUID) {
  Serial.println("Initializing BLE server...");
  
  // Create BLE Device
  BLEDevice::init(deviceName);
  
  // Create BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());
  
  // Create BLE Service
  pService = pServer->createService(serviceUUID);
  
  Serial.println("BLE server initialized");
}

void BLE_addCharacteristic(const char* charUUID, void (*callback)(uint8_t*, size_t)) {
  if (pService == NULL || callbackCount >= MAX_CALLBACKS) {
    Serial.println("Error: Cannot add characteristic");
    return;
  }
  
  // Create characteristic with read, write, and notify properties
  BLECharacteristic* pChar = pService->createCharacteristic(
                                charUUID,
                                BLECharacteristic::PROPERTY_READ |
                                BLECharacteristic::PROPERTY_WRITE |
                                BLECharacteristic::PROPERTY_NOTIFY
                              );
  
  // Add descriptor for notifications
  pChar->addDescriptor(new BLE2902());
  
  // Set callback
  pChar->setCallbacks(new CharCallbacks());
  
  // Store callback info
  callbacks[callbackCount].charUUID = charUUID;
  callbacks[callbackCount].callback = callback;
  callbacks[callbackCount].pChar = pChar;
  callbackCount++;
  
  Serial.print("Added characteristic: ");
  Serial.println(charUUID);
}

void BLE_start(const char* deviceName) {
  if (pService == NULL) {
    Serial.println("Error: No service created");
    return;
  }
  
  // Start the service
  pService->start();
  
  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(pService->getUUID());
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  
  Serial.println("BLE server started!");
  Serial.print("Device name: ");
  Serial.println(deviceName);
  Serial.println("Waiting for client connection...");
}

bool BLE_isConnected() {
  return deviceConnected;
}

// Conversion Helpers
float toNorm(uint8_t value) {
  // Convert 0-255 to 0.0-1.0 float
  return value / 256.0;
}

float toBipolar(uint8_t value) {
  // Convert 0-255 to -1.0-1.0 float
  return (value - 128) / 128.0;
}
