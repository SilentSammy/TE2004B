#include <SPI.h>
#include <mcp_can.h>

// ======================================================
//  MCP2515 wiring for ESP32-C3 Super Mini
// ======================================================
#define CAN0_INT  2
#define CAN0_CS   7
#define SPI_SCK   8
#define SPI_MISO  9
#define SPI_MOSI 10

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
byte txData[8] = {0};                       // CAN message buffer
bool canInitialized = false;

// ======================================================
//  Encoder variables
// ======================================================
volatile long encoderPosition = 0;
volatile int lastEncoded = 0;

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
//  Setup
// ======================================================
void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  Serial.println("\nESP32-C3 Encoder → CAN Example");
  Serial.println("===================================");

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
}

// ======================================================
//  Fast UART print helper
// ======================================================
inline void fastStatusPrint(bool sent, long pos) {
  Serial.write(sent ? 'Y' : 'N');  // 1 char: Y or N
  Serial.print(pos);               // encoder position digits
  Serial.print("\r\n");            // CRLF line ending
}

// ======================================================
//  Main loop
// ======================================================
void loop() {
  if (!canInitialized) return;

  static unsigned long lastTxTime = 0;
  static long lastSentPos = 0;

  unsigned long now = millis();

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
}
