/**
 * @file pack.hpp
 * @brief Packet packing module
 *
 * Command packing and message parsing.
 */

#ifndef PACK_HPP
#define PACK_HPP

#include <vector>
#include <string>
#include <cstdint>
#include <memory>
#include <optional>

namespace das {

/**
 * @brief Opcode enum
 */
enum class Opcode : uint8_t {
    ReadSingle = 0x01,
    ReadBatch = 0x02,
    WriteDrive = 0x03,
    Echo = 0x04,
    CalibEncoder = 0x05,
    DisableDrive = 0x06
};

/**
 * @brief Record type enum
 */
enum class RecordType : uint8_t {
    Tactile = 0x01,
    Encoder = 0x02,
    Drive = 0x03,
    Echo = 0x04
};

/**
 * @brief Protocol content constants
 */
struct PackContent {
    static constexpr char MAGIC[] = "das\r\n";
    static constexpr size_t MAGIC_LENGTH = 5;
};

/**
 * @brief Record struct
 */
struct Record {
    RecordType record_type;
    size_t record_content_length;
    std::vector<uint8_t> record_data;

    Record(RecordType type, const std::vector<uint8_t>& data)
        : record_type(type), record_content_length(data.size()), record_data(data) {}
    
    std::vector<uint8_t> pack() const;
};

/**
 * @brief Command pack class
 */
class CmdPack {
public:
    std::vector<uint8_t> data;
    std::optional<Opcode> opcode;
    std::optional<RecordType> record_type;
    std::vector<uint8_t> record_data;

    CmdPack() = default;
    CmdPack(const std::vector<uint8_t>& data, 
            std::optional<Opcode> opcode = std::nullopt,
            std::optional<RecordType> record_type = std::nullopt,
            const std::vector<uint8_t>& record_data = {});

    /**
     * @brief Pack command
     */
    static CmdPack pack(Opcode opcode, RecordType record_type, const std::vector<uint8_t>& record = {});

    /**
     * @brief Pack calibration command
     */
    static CmdPack packCalib(const std::vector<uint8_t>& record);

    /**
     * @brief Unpack command
     */
    static std::optional<CmdPack> unpack(const std::vector<uint8_t>& data);
};

/**
 * @brief Message unpack class
 */
class MessagePack {
public:
    std::vector<uint8_t> data;
    std::optional<Opcode> opcode;
    std::vector<Record> records;

    MessagePack() = default;
    MessagePack(const std::vector<uint8_t>& data,
                std::optional<Opcode> opcode = std::nullopt,
                const std::vector<Record>& records = {});

    /**
     * @brief Pack message
     */
    static std::vector<uint8_t> pack(Opcode opcode, const std::vector<Record>& records = {});

    /**
     * @brief Unpack message
     */
    static std::optional<MessagePack> unpack(const std::vector<uint8_t>& data);

    /**
     * @brief Unpack camera calibration data
     * @param yaml_filename Output YAML filename (empty = no file)
     * @param output_dir Output directory
     */
    static bool unpackCameraCalib(const std::vector<uint8_t>& data, 
                                   const std::string& yaml_filename = "",
                                   const std::string& output_dir = "");

    /**
     * @brief Calibration info struct
     */
    struct CalibInfo {
        uint32_t width = 0;
        uint32_t height = 0;
        std::string model;
        std::vector<double> distortion;
        std::vector<double> intrinsics;
        std::vector<double> extrinsics;
    };

private:
    static bool checkHead(const std::vector<uint8_t>& data);
    static bool checkTail(const std::vector<uint8_t>& data);
    static bool checkMagic(const uint8_t* data, size_t length);
    static std::pair<uint64_t, size_t> readVarint(const std::vector<uint8_t>& data, size_t pos);
    static CalibInfo parseProtobufCalib(const std::vector<uint8_t>& data);
    static std::string generateYaml(const CalibInfo& info);
};

/**
 * @brief Helper: float to big-endian bytes
 */
inline std::vector<uint8_t> floatToBigEndianBytes(float value) {
    std::vector<uint8_t> bytes(4);
    uint32_t* ptr = reinterpret_cast<uint32_t*>(&value);
    uint32_t val = *ptr;
    bytes[0] = (val >> 24) & 0xFF;
    bytes[1] = (val >> 16) & 0xFF;
    bytes[2] = (val >> 8) & 0xFF;
    bytes[3] = val & 0xFF;
    return bytes;
}

/**
 * @brief Helper: big-endian bytes to float
 */
inline float bigEndianBytesToFloat(const uint8_t* data) {
    uint32_t val = (static_cast<uint32_t>(data[0]) << 24) |
                   (static_cast<uint32_t>(data[1]) << 16) |
                   (static_cast<uint32_t>(data[2]) << 8) |
                   static_cast<uint32_t>(data[3]);
    return *reinterpret_cast<float*>(&val);
}

} // namespace das

#endif // PACK_HPP
