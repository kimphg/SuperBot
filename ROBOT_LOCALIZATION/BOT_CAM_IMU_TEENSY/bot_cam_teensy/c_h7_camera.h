// Packet structure based on the definition
struct Packet {
    uint8_t header1;        // byte 0: 0xAA
    uint8_t header2;        // byte 1: 0x55
    uint8_t packet_len;     // byte 2: 0x01
    uint8_t packet_id;      // byte 3: 0x01
    uint8_t tag_id_msb;     // byte 4: tag id MSB
    uint8_t tag_id_lsb;     // byte 5: tag id LSB
    uint8_t tag_x_msb;      // byte 6: tag X MSB
    uint8_t tag_y_lsb;      // byte 7: tag Y LSB
    uint8_t tag_rotation_msb;  // byte 8: tag rotation MSB
    uint8_t tag_rotation_lsb;  // byte 9: tag rotation LSB
    uint8_t crc16_msb;      // byte 10: crc16 MSB
    uint8_t crc16_lsb;      // byte 11: crc16 LSB
};

class H7PacketReader {
public:
    // Read packet from raw bytes
    static bool readPacket(const uint8_t* data, size_t length, Packet& packet) {
        if (length < sizeof(Packet)) {
            std::cerr << "Error: Insufficient data length" << std::endl;
            return false;
        }

        // Copy data to packet structure
        memcpy(&packet, data, sizeof(Packet));

        // Validate header
        if (packet.header1 != 0xAA || packet.header2 != 0x55) {
            std::cerr << "Error: Invalid packet header" << std::endl;
            return false;
        }

        return true;
    }
static int packetBuffIndex = 0;
static unsigned char h7packetBuff[100];
    // Extract 16-bit tag ID from MSB and LSB
    static uint16_t getTagId(const Packet& packet) {
        return (static_cast<uint16_t>(packet.tag_id_msb) << 8) | packet.tag_id_lsb;
    }

    // Extract 16-bit tag X coordinate (assuming tag_x_msb at byte 6)
    static uint16_t getTagX(const Packet& packet) {
        return (static_cast<uint16_t>(packet.tag_x_msb) << 8) | packet.tag_y_lsb;
    }

    // Extract 16-bit tag rotation
    static uint16_t getTagRotation(const Packet& packet) {
        return (static_cast<uint16_t>(packet.tag_rotation_msb) << 8) | packet.tag_rotation_lsb;
    }

    // Extract 16-bit CRC
    static uint16_t getCRC16(const Packet& packet) {
        return (static_cast<uint16_t>(packet.crc16_msb) << 8) | packet.crc16_lsb;
    }

    // Display packet information
    static void displayPacket(const Packet& packet) {
        std::cout << "=== Packet Information ===" << std::endl;
        std::cout << "Header: 0x" << std::hex << static_cast<int>(packet.header1) 
                  << " 0x" << static_cast<int>(packet.header2) << std::endl;
        std::cout << "Packet Type: 0x" << static_cast<int>(packet.packet_type) << std::endl;
        std::cout << "Packet ID: 0x" << static_cast<int>(packet.packet_id) << std::endl;
        std::cout << "Tag ID: " << std::dec << getTagId(packet) << std::endl;
        std::cout << "Tag X/Y: " << getTagX(packet) << std::endl;
        std::cout << "Tag Rotation: " << getTagRotation(packet) << std::endl;
        std::cout << "CRC16: 0x" << std::hex << getCRC16(packet) << std::endl;
        std::cout << std::dec;
    }
};
