/*
 * BLE Test - Main Application
 * Simple BLE server with LED control
 */

// BLE UUIDs
#define SERVICE_UUID        "12345678-1234-5678-1234-56789abcdef0"
#define CHARACTERISTIC_UUID_LED      "12345678-1234-5678-1234-56789abcdef1"
#define CHARACTERISTIC_UUID_THROTTLE "12345678-1234-5678-1234-56789abcdef2"
#define CHARACTERISTIC_UUID_STEERING "12345678-1234-5678-1234-56789abcdef3"

// Device configuration
#define DEVICE_NAME "BLE_Test"
#define LED_PIN 8

// Global variables for control values
float throttleValue = 0.0;  // -1.0 to 1.0
float steeringValue = 0.0;  // -1.0 to 1.0

// LED control callback
void onLedControl(uint8_t value) {
  if (value > 0) {
    digitalWrite(LED_PIN, HIGH);
    Serial.println("LED: ON");
  } else {
    digitalWrite(LED_PIN, LOW);
    Serial.println("LED: OFF");
  }
}

// Throttle control callback
void onThrottleControl(uint8_t value) {
  throttleValue = toBipolar(value);
  Serial.print("Throttle: ");
  Serial.println(throttleValue, 3);
}

// Steering control callback
void onSteeringControl(uint8_t value) {
  steeringValue = toBipolar(value);
  Serial.print("Steering: ");
  Serial.println(steeringValue, 3);
}

void setup() {
  Serial.begin(115200);
  Serial.println("Starting BLE Test...");
  
  // Initialize LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  // Initialize BLE server
  BLE_init(DEVICE_NAME, SERVICE_UUID);
  
  // Add characteristics with callbacks
  BLE_addCharacteristic(CHARACTERISTIC_UUID_LED, onLedControl);
  BLE_addCharacteristic(CHARACTERISTIC_UUID_THROTTLE, onThrottleControl);
  BLE_addCharacteristic(CHARACTERISTIC_UUID_STEERING, onSteeringControl);
  
  // Start the BLE server
  BLE_start(DEVICE_NAME);
}

void loop() {
  // Application logic here
  delay(100);
}
