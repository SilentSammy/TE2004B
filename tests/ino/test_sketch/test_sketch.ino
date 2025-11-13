/*
 * Minimal BLE Server for ESP32
 * Mirrors the MicroPython BLE server functionality
 * Controls an LED via BLE characteristic
 */

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// BLE UUIDs (matching MicroPython server)
#define SERVICE_UUID        "12345678-1234-5678-1234-56789abcdef0"
#define CHARACTERISTIC_UUID "12345678-1234-5678-1234-56789abcdef1"

// Device name
#define DEVICE_NAME "BLE_Test"

// LED pin
#define LED_PIN 8

BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
bool deviceConnected = false;

// Callback for server connection events
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      Serial.println("Client connected");
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      Serial.println("Client disconnected");
      // Restart advertising
      BLEDevice::startAdvertising();
      Serial.println("Advertising restarted");
    }
};

// Callback for characteristic write events
class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      uint8_t* pData = pCharacteristic->getData();
      size_t len = pCharacteristic->getValue().length();
      
      if (len > 0) {
        uint8_t receivedValue = pData[0];
        
        // Control LED based on received value
        if (receivedValue > 0) {
          digitalWrite(LED_PIN, HIGH);
          Serial.println("LED: ON (received 1)");
        } else {
          digitalWrite(LED_PIN, LOW);
          Serial.println("LED: OFF (received 0)");
        }
      }
    }
};

void setup() {
  Serial.begin(115200);
  Serial.println("Starting BLE server...");
  
  // Initialize LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  // Create BLE Device
  BLEDevice::init(DEVICE_NAME);
  
  // Create BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  
  // Create BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);
  
  // Create BLE Characteristic
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ |
                      BLECharacteristic::PROPERTY_WRITE |
                      BLECharacteristic::PROPERTY_NOTIFY
                    );
  
  // Add descriptor for notifications
  pCharacteristic->addDescriptor(new BLE2902());
  
  // Set characteristic callback
  pCharacteristic->setCallbacks(new MyCallbacks());
  
  // Start the service
  pService->start();
  
  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  
  Serial.println("BLE server ready!");
  Serial.println("Device name: " + String(DEVICE_NAME));
  Serial.println("Waiting for client connection...");
}

void loop() {
  // Nothing needed in loop for basic BLE server
  delay(100);
}
