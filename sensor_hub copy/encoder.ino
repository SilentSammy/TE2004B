/*
 * Encoder Module - Rotary Encoder Quadrature Decoder
 * Handles encoder position tracking via interrupts
 */

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

