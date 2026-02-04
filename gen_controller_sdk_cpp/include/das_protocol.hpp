/**
 * @file das_protocol.hpp
 * @brief DAS protocol parsing module
 *
 * Packet parsing and creation for DAS protocol.
 */

#ifndef DAS_PROTOCOL_HPP
#define DAS_PROTOCOL_HPP

#include <vector>
#include <string>
#include <cstdint>
#include <tuple>
#include <chrono>
#include <optional>

namespace das {

/**
 * @brief DAS protocol class
 */
class DASProtocol {
public:
    // Protocol magic
    static constexpr char MAGIC[] = "das\r\n";
    static constexpr size_t MAGIC_LENGTH = 5;
    static constexpr size_t MAX_PACKET_SIZE = 4096;
    static constexpr size_t MAX_BUFFER_SIZE = 8192;

    /**
     * @brief Packet info struct
     */
    struct PacketInfo {
        uint8_t opcode;
        std::vector<uint8_t> data_section;
        size_t data_length;
        std::vector<uint8_t> raw_packet;
        size_t packet_length;
        std::chrono::system_clock::time_point timestamp;
    };

    /**
     * @brief Find complete packets in buffer
     * @param data Data buffer
     * @return Tuple of (packets found, remaining data)
     */
    static std::tuple<std::vector<std::vector<uint8_t>>, std::vector<uint8_t>> 
    findPacket(const std::vector<uint8_t>& data);

    /**
     * @brief Validate packet structure
     * @param packet Packet data
     * @return true if valid
     */
    static bool validatePacketStructure(const std::vector<uint8_t>& packet);

    /**
     * @brief Parse packet
     * @param packet Packet data
     * @return Parsed info or nullopt if invalid
     */
    static std::optional<PacketInfo> parsePacket(const std::vector<uint8_t>& packet);

    /**
     * @brief Create packet
     * @param opcode Opcode
     * @param data Payload
     * @return Created packet
     */
    static std::vector<uint8_t> createPacket(uint8_t opcode, const std::vector<uint8_t>& data = {});

private:
    /**
     * @brief Check if data starts with magic
     */
    static bool checkMagic(const uint8_t* data, size_t length);
};

} // namespace das

#endif // DAS_PROTOCOL_HPP
