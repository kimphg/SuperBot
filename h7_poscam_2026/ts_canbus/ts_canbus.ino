// Teensy 4.0 - reads AprilTag UART packets from OpenMV H7
// Packet: AA 55 addr cmd tagID_hi tagID_lo cx cy rot_hi rot_lo crc_hi crc_lo (12 bytes)
// UART1 (Serial1) at 1000000 baud

#define UART_BAUD       1000000
#define HEADER0         0xAA
#define HEADER1         0x55
#define PACKET_LEN      12
#define ADDR_CAMERA     0x01
#define CMD_APRILTAG    0x11

struct AprilTagPacket {
  uint16_t tagId;
  uint8_t  cx;        // 0-255, normalized center X
  uint8_t  cy;        // 0-255, normalized center Y
  float    rotationDeg; // degrees (stored as deg*10 in packet)
};

AprilTagPacket lastTag;
bool newTagAvailable = false;

static uint16_t crc16(uint8_t *data, int offset, int length) {
  uint16_t crc = 0xFFFF;
  for (int i = 0; i < length; i++) {
    crc ^= (uint16_t)data[offset + i] << 8;
    for (int j = 0; j < 8; j++) {
      if (crc & 0x8000)
        crc = (crc << 1) ^ 0x1021;
      else
        crc <<= 1;
    }
  }
  return crc;
}

// Ring buffer for incoming bytes
static uint8_t rxBuf[256];
static int rxHead = 0;
static int rxTail = 0;

static void rxPush(uint8_t b) {
  int next = (rxHead + 1) % sizeof(rxBuf);
  if (next != rxTail) {
    rxBuf[rxHead] = b;
    rxHead = next;
  }
}

static int rxSize() {
  return (rxHead - rxTail + sizeof(rxBuf)) % sizeof(rxBuf);
}

static uint8_t rxPeek(int offset) {
  return rxBuf[(rxTail + offset) % sizeof(rxBuf)];
}

static void rxConsume(int n) {
  rxTail = (rxTail + n) % sizeof(rxBuf);
}

static void processPackets() {
  // Need at least PACKET_LEN bytes to attempt parsing
  while (rxSize() >= PACKET_LEN) {
    // Find header
    if (rxPeek(0) != HEADER0 || rxPeek(1) != HEADER1) {
      rxConsume(1);
      continue;
    }

    uint8_t pkt[PACKET_LEN];
    for (int i = 0; i < PACKET_LEN; i++)
      pkt[i] = rxPeek(i);

    // Verify address and command
    if (pkt[2] != ADDR_CAMERA || pkt[3] != CMD_APRILTAG) {
      rxConsume(1);
      continue;
    }

    // Verify CRC (computed over bytes 2..9, length 8)
    uint16_t calcCrc = crc16(pkt, 2, 8);
    uint16_t pktCrc  = ((uint16_t)pkt[10] << 8) | pkt[11];
    if (calcCrc != pktCrc) {
      rxConsume(1);
      continue;
    }

    // Parse fields
    lastTag.tagId        = ((uint16_t)pkt[4] << 8) | pkt[5];
    lastTag.cx           = pkt[6];
    lastTag.cy           = pkt[7];
    uint16_t rotRaw      = ((uint16_t)pkt[8] << 8) | pkt[9];
    lastTag.rotationDeg  = rotRaw / 10.0f;
    newTagAvailable      = true;
    rxConsume(PACKET_LEN);
  }
}

void setup() {
  Serial.begin(115200);   // USB debug
  Serial1.begin(UART_BAUD); // UART to OpenMV H7
}

void loop() {
  // Drain Serial1 into ring buffer
  while (Serial1.available()) {
    rxPush(Serial1.read());
  }

  processPackets();

  if (newTagAvailable) {
    newTagAvailable = false;

    Serial.print("Tag ID: ");   Serial.print(lastTag.tagId);
    Serial.print("  cx: ");     Serial.print(lastTag.cx);
    Serial.print("  cy: ");     Serial.print(lastTag.cy);
    Serial.print("  rot: ");    Serial.print(lastTag.rotationDeg, 1);
    Serial.println(" deg");

    // TODO: forward data over CAN bus or act on tag data here
  }
}
