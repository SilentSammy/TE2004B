#include <SPI.h>
#include <mcp_can.h>

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

// ======================================================
//  MCP2515 wiring for ESP32-C3 Super Mini
// ======================================================
#define CAN0_INT  2
#define CAN0_CS   7
#define SPI_SCK   8
#define SPI_MISO  9
#define SPI_MOSI 10

// ======================================================
//  BLE UUIDs
// ======================================================
#define SERVICE_UUID           "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID_RX "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// ======================================================
//  Quadrature Encoder pins
// ======================================================
#define ENC_A 0
#define ENC_B 1

MCP_CAN CAN0(CAN0_CS);  // CAN object

// ======================================================
//  CAN configuration
// ======================================================
const unsigned long TX_INTERVAL_MAX = 250;  // Always send every 250 ms
const unsigned long TX_INTERVAL_MIN = 1;    // Never send faster than 1 ms
byte txData[8] = {0};
bool canInitialized = false;

// IDs
const uint32_t CANID_BLE = 0x321;
const uint32_t CANID_ENC = 0x123;

// ======================================================
//  Encoder variables
// ======================================================
volatile long encoderPosition = 0;
volatile int  lastEncoded = 0;

// ======================================================
//  BLE globals
// ======================================================
volatile bool deviceConnected = false;
BLEServer*         pServer = nullptr;
BLECharacteristic* pRx     = nullptr;

// ======================================================
//  Encoder ISR
// ======================================================
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

// ======================================================
//  BLE → CAN bridge
// ======================================================
void sendBleToCan(const uint8_t* data, size_t len) {
  if (!canInitialized) return;

  uint8_t buf[8];
  uint8_t seq = 0;
  size_t i = 0;

  while (i < len) {
    size_t chunk = (len - i > 7) ? 7 : (len - i);

    buf[0] = 0x80 | (seq & 0x7F);
    for (size_t j = 0; j < chunk; ++j) buf[1 + j] = data[i + j];
    for (size_t j = 1 + chunk; j < 8; ++j) buf[j] = 0;

    CAN0.sendMsgBuf(CANID_BLE, 0, (uint8_t)(1 + chunk), buf);

    i   += chunk;
    seq += 1;
  }
}

// ======================================================
//  BLE callbacks
// ======================================================
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* /*pSrv*/) override {
    deviceConnected = true;
    Serial.println("[BLE] Conectado");
  }
  void onDisconnect(BLEServer* /*pSrv*/) override {
    deviceConnected = false;
    Serial.println("[BLE] Desconectado");
    BLEDevice::startAdvertising();
  }
};

class MyCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic) override {
    String s = pCharacteristic->getValue();  // ← usa String de Arduino
    if (s.length() > 0) {
      Serial.println("**********");
      Serial.print("valor = ");
      Serial.println(s);

      // Puente BLE → CAN
      sendBleToCan((const uint8_t*)s.c_str(), s.length());
    }
  }
};

// ======================================================
//  Fast UART print helper
// ======================================================
inline void fastStatusPrint(bool sent, long pos) {
  Serial.write(sent ? 'Y' : 'N');
  Serial.print(pos);
  Serial.print("\r\n");
}

// ======================================================
//  Setup
// ======================================================
void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  Serial.println("\nESP32-C3 Encoder → CAN + BLE bridge");
  Serial.println("=====================================");

  // Encoder
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), updateEncoder, CHANGE);

  // SPI & CAN
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    Serial.println("MCP2515 Initialized Successfully!");
    canInitialized = true;
    CAN0.setMode(MCP_NORMAL);
  } else {
    Serial.println("Error Initializing MCP2515...");
  }
  pinMode(CAN0_INT, INPUT);

  // BLE
  BLEDevice::init("C3-Cuetita");

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService* pService = pServer->createService(SERVICE_UUID);

  pRx = pService->createCharacteristic(
    CHARACTERISTIC_UUID_RX,
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
  );
  pRx->setCallbacks(new MyCallbacks());

  pService->start();

  BLEAdvertising* adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(SERVICE_UUID);
  BLEDevice::startAdvertising();

  Serial.println("BLE listo. Conéctate desde la app y envía texto.");
}

// ======================================================
//  Main loop
// ======================================================
void loop() {
  if (!canInitialized) return;

  static unsigned long lastTxTime = 0;
  static long lastSentPos = 0;
  unsigned long now = millis();

  long pos;
  noInterrupts();
  pos = encoderPosition;
  interrupts();

  bool changed = (pos != lastSentPos);
  bool tooSoon = (now - lastTxTime < TX_INTERVAL_MIN);
  bool dueByTimeout = (now - lastTxTime >= TX_INTERVAL_MAX);

  if ((changed && !tooSoon) || dueByTimeout) {
    lastTxTime = now;
    lastSentPos = pos;

    txData[0] = (byte)(pos);
    txData[1] = (byte)(pos >> 8);
    txData[2] = (byte)(pos >> 16);
    txData[3] = (byte)(pos >> 24);
    for (int i = 4; i < 8; i++) txData[i] = 0;

    byte sndStat = CAN0.sendMsgBuf(CANID_ENC, 0, 8, txData);
    fastStatusPrint(sndStat == CAN_OK, pos);
  }
}
