/**
 * @file pack.cpp
 * @brief Packet packing module implementation
 */

#include "pack.hpp"
#include <iostream>
#include <fstream>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <filesystem>

namespace das {

// Record implementation
std::vector<uint8_t> Record::pack() const {
    std::vector<uint8_t> packet;
    packet.push_back(static_cast<uint8_t>(record_type));
    
    // Little-endian 64-bit length
    uint64_t len = record_content_length;
    for (int i = 0; i < 8; i++) {
        packet.push_back(len & 0xFF);
        len >>= 8;
    }
    
    packet.insert(packet.end(), record_data.begin(), record_data.end());
    return packet;
}

// CmdPack implementation
CmdPack::CmdPack(const std::vector<uint8_t>& data,
                 std::optional<Opcode> opcode,
                 std::optional<RecordType> record_type,
                 const std::vector<uint8_t>& record_data)
    : data(data), opcode(opcode), record_type(record_type), record_data(record_data) {}

CmdPack CmdPack::pack(Opcode opcode, RecordType record_type, const std::vector<uint8_t>& record) {
    std::vector<uint8_t> packet;
    
    // Magic
    packet.insert(packet.end(), PackContent::MAGIC, PackContent::MAGIC + PackContent::MAGIC_LENGTH);
    
    // Opcode
    packet.push_back(static_cast<uint8_t>(opcode));
    
    // Record type
    packet.push_back(static_cast<uint8_t>(record_type));
    
    // Record content length (big-endian 32-bit)
    uint32_t len = static_cast<uint32_t>(record.size());
    packet.push_back((len >> 24) & 0xFF);
    packet.push_back((len >> 16) & 0xFF);
    packet.push_back((len >> 8) & 0xFF);
    packet.push_back(len & 0xFF);
    
    // Padding bytes
    packet.push_back(0);
    packet.push_back(0);
    
    // Torque value (big-endian 16-bit, fixed 80)
    uint16_t torque = 80;
    packet.push_back((torque >> 8) & 0xFF);
    packet.push_back(torque & 0xFF);
    
    // Record data
    packet.insert(packet.end(), record.begin(), record.end());
    
    // Tail padding
    packet.push_back(0);
    packet.push_back(1);
    
    // Magic
    packet.insert(packet.end(), PackContent::MAGIC, PackContent::MAGIC + PackContent::MAGIC_LENGTH);
    
    return CmdPack(packet, opcode, record_type, record);
}

CmdPack CmdPack::packCalib(const std::vector<uint8_t>& record) {
    return CmdPack(record, std::nullopt, std::nullopt, record);
}

std::optional<CmdPack> CmdPack::unpack(const std::vector<uint8_t>& data) {
    if (data.size() < PackContent::MAGIC_LENGTH * 2) {
        return std::nullopt;
    }

    // Check head/tail magic
    if (std::memcmp(data.data(), PackContent::MAGIC, PackContent::MAGIC_LENGTH) != 0) {
        return std::nullopt;
    }
    if (std::memcmp(data.data() + data.size() - PackContent::MAGIC_LENGTH, 
                    PackContent::MAGIC, PackContent::MAGIC_LENGTH) != 0) {
        return std::nullopt;
    }

    size_t opcode_pos = PackContent::MAGIC_LENGTH;
    size_t record_type_pos = opcode_pos + 1;
    size_t length_pos = record_type_pos + 1;
    size_t record_start = length_pos + 8;
    size_t record_end = data.size() - PackContent::MAGIC_LENGTH;

    if (data.size() < record_start) {
        return std::nullopt;
    }

    uint8_t opcode_value = data[opcode_pos];
    uint8_t record_type_value = data[record_type_pos];

    // Read length (little-endian 64-bit)
    uint64_t record_content_length = 0;
    for (int i = 0; i < 8; i++) {
        record_content_length |= static_cast<uint64_t>(data[length_pos + i]) << (i * 8);
    }

    std::vector<uint8_t> record_data(data.begin() + record_start, data.begin() + record_end);

    if (record_data.size() != record_content_length) {
        std::cerr << "ERROR! record data length not match!" << std::endl;
        return std::nullopt;
    }

    return CmdPack(data, static_cast<Opcode>(opcode_value), 
                   static_cast<RecordType>(record_type_value), record_data);
}

// MessagePack implementation
MessagePack::MessagePack(const std::vector<uint8_t>& data,
                         std::optional<Opcode> opcode,
                         const std::vector<Record>& records)
    : data(data), opcode(opcode), records(records) {}

bool MessagePack::checkMagic(const uint8_t* data, size_t length) {
    if (length < PackContent::MAGIC_LENGTH) return false;
    return std::memcmp(data, PackContent::MAGIC, PackContent::MAGIC_LENGTH) == 0;
}

bool MessagePack::checkHead(const std::vector<uint8_t>& data) {
    return checkMagic(data.data(), data.size());
}

bool MessagePack::checkTail(const std::vector<uint8_t>& data) {
    if (data.size() < PackContent::MAGIC_LENGTH) return false;
    return checkMagic(data.data() + data.size() - PackContent::MAGIC_LENGTH, PackContent::MAGIC_LENGTH);
}

std::vector<uint8_t> MessagePack::pack(Opcode opcode, const std::vector<Record>& records) {
    std::vector<uint8_t> packet;
    
    // Magic
    packet.insert(packet.end(), PackContent::MAGIC, PackContent::MAGIC + PackContent::MAGIC_LENGTH);
    
    // Opcode
    packet.push_back(static_cast<uint8_t>(opcode));
    
    // Records
    for (const auto& record : records) {
        auto record_bytes = record.pack();
        packet.insert(packet.end(), record_bytes.begin(), record_bytes.end());
    }
    
    // Magic
    packet.insert(packet.end(), PackContent::MAGIC, PackContent::MAGIC + PackContent::MAGIC_LENGTH);
    
    return packet;
}

std::optional<MessagePack> MessagePack::unpack(const std::vector<uint8_t>& data) {
    try {
        if (!checkHead(data) || !checkTail(data)) {
            return std::nullopt;
        }

        std::vector<Record> records;

        size_t opcode_pos = PackContent::MAGIC_LENGTH;
        size_t i = opcode_pos + 1;
        uint8_t opcode_value = data[opcode_pos];

        size_t data_end = data.size() - PackContent::MAGIC_LENGTH;
        
        while (i < data_end) {
            if (i >= data_end) break;
            
            size_t record_type_pos = i;
            uint8_t record_type_value = data[record_type_pos];

            size_t length_pos = record_type_pos + 1;
            if (length_pos + 8 > data_end) {
                return std::nullopt;
            }

            // Read length (big-endian 64-bit)
            uint64_t record_content_length = 0;
            for (int j = 0; j < 8; j++) {
                record_content_length = (record_content_length << 8) | data[length_pos + j];
            }

            size_t record_start = length_pos + 8;
            size_t record_end_pos = record_start + record_content_length;

            if (record_end_pos > data_end) {
                return std::nullopt;
            }

            std::vector<uint8_t> record_data(data.begin() + record_start, data.begin() + record_end_pos);

            if (record_data.size() != record_content_length) {
                return std::nullopt;
            }

            records.emplace_back(static_cast<RecordType>(record_type_value), record_data);
            i = record_end_pos;
        }

        // Print parsed data (debug)
        // std::cout << "unpack data: ";
        // for (auto b : data) {
        //     std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(b);
        // }
        // std::cout << std::dec << std::endl;

        return MessagePack(data, static_cast<Opcode>(opcode_value), records);
    } catch (...) {
        return std::nullopt;
    }
}

std::pair<uint64_t, size_t> MessagePack::readVarint(const std::vector<uint8_t>& data, size_t pos) {
    uint64_t result = 0;
    int shift = 0;
    while (true) {
        if (pos >= data.size()) {
            std::cerr << "pos: " << pos << ", data size: " << data.size() << std::endl;
            throw std::runtime_error("Varint overflow");
        }
        uint8_t b = data[pos];
        pos++;
        result |= static_cast<uint64_t>(b & 0x7f) << shift;
        if (!(b & 0x80)) {
            return {result, pos};
        }
        shift += 7;
        // Prevent infinite loop, varint max 10 bytes
        if (shift >= 70) {
            throw std::runtime_error("Varint too long");
        }
    }
}

MessagePack::CalibInfo MessagePack::parseProtobufCalib(const std::vector<uint8_t>& data) {
    CalibInfo info;
    size_t pos = 0;

    try {
        while (pos < data.size()) {
            // Check enough data for tag
            if (pos >= data.size()) break;
            
            auto [tag, new_pos] = readVarint(data, pos);
            pos = new_pos;
            uint64_t field_num = tag >> 3;
            uint64_t wire_type = tag & 0x07;

            if (field_num == 2 && wire_type == 5) {  // Width (Fixed32)
                if (pos + 4 > data.size()) {
                    std::cerr << "Width: insufficient data, need 4 bytes, pos: " << pos << ", size: " << data.size() << std::endl;
                    break;
                }
                std::memcpy(&info.width, &data[pos], 4);
                std::cout << "info width: " << info.width << std::endl;
                pos += 4;
            } else if (field_num == 3 && wire_type == 5) {  // Height (Fixed32)
                if (pos + 4 > data.size()) {
                    std::cerr << "Height: insufficient data" << std::endl;
                    break;
                }
                std::memcpy(&info.height, &data[pos], 4);
                std::cout << "info height: " << info.height << std::endl;
                pos += 4;
            } else if (field_num == 4 && wire_type == 2) {  // Model (String)
                auto [length, next_pos] = readVarint(data, pos);
                pos = next_pos;
                if (pos + length > data.size()) {
                    std::cerr << "Model: insufficient data, need " << length << " bytes" << std::endl;
                    break;
                }
                info.model = std::string(data.begin() + pos, data.begin() + pos + length);
                std::cout << "info model: " << info.model << std::endl;
                pos += length;
            } else if (field_num == 5 && wire_type == 2) {  // Distortion Coeffs (Packed Doubles)
                auto [length, next_pos] = readVarint(data, pos);
                pos = next_pos;
                if (pos + length > data.size()) {
                    std::cerr << "Distortion: insufficient data, need " << length << " bytes" << std::endl;
                    break;
                }
                size_t count = length / 8;
                info.distortion.clear();
                for (size_t i = 0; i < count; i++) {
                    double val;
                    std::memcpy(&val, &data[pos + i * 8], 8);
                    info.distortion.push_back(val);
                }
                std::cout << "info distortion: ";
                for (auto v : info.distortion) std::cout << v << " ";
                std::cout << std::endl;
                pos += length;
            } else if (field_num == 6 && wire_type == 2) {  // Intrinsics Matrix (Packed Doubles)
                auto [length, next_pos] = readVarint(data, pos);
                pos = next_pos;
                if (pos + length > data.size()) {
                    std::cerr << "Intrinsics: insufficient data, need " << length << " bytes" << std::endl;
                    break;
                }
                size_t count = length / 8;
                info.intrinsics.clear();
                for (size_t i = 0; i < count; i++) {
                    double val;
                    std::memcpy(&val, &data[pos + i * 8], 8);
                    info.intrinsics.push_back(val);
                }
                std::cout << "info intrinsics: ";
                for (auto v : info.intrinsics) std::cout << v << " ";
                std::cout << std::endl;
                pos += length;
            } else if (field_num == 10 && wire_type == 2) {  // Extrinsics (Packed Doubles)
                auto [length, next_pos] = readVarint(data, pos);
                pos = next_pos;
                if (pos + length > data.size()) {
                    std::cerr << "Extrinsics: insufficient data, need " << length << " bytes" << std::endl;
                    break;
                }
                size_t count = length / 8;
                info.extrinsics.clear();
                for (size_t i = 0; i < count; i++) {
                    double val;
                    std::memcpy(&val, &data[pos + i * 8], 8);
                    info.extrinsics.push_back(val);
                }
                std::cout << "info extrinsics: ";
                for (auto v : info.extrinsics) std::cout << v << " ";
                std::cout << std::endl;
                pos += length;
            } else {
                // Skip unknown fields
                if (wire_type == 0) {  // Varint
                    auto [_, next_pos] = readVarint(data, pos);
                    pos = next_pos;
                } else if (wire_type == 1) {  // Fixed64
                    if (pos + 8 > data.size()) break;
                    pos += 8;
                } else if (wire_type == 2) {  // Length Delimited
                    auto [length, next_pos] = readVarint(data, pos);
                    pos = next_pos;
                    if (pos + length > data.size()) break;
                    pos += length;
                } else if (wire_type == 3) {  // Start Group (Deprecated)
                    // Just skip the tag
                } else if (wire_type == 4) {  // End Group (Deprecated)
                    // Just skip the tag
                } else if (wire_type == 5) {  // Fixed32
                    if (pos + 4 > data.size()) break;
                    pos += 4;
                } else {
                    // Unknown wire type, break
                    std::cerr << "Unknown wire type " << wire_type << " at pos " << pos << std::endl;
                    break;
                }
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "Parse protobuf error: " << e.what() << std::endl;
    }

    return info;
}

std::string MessagePack::generateYaml(const CalibInfo& info) {
    std::ostringstream yaml;

    // Map model name
    std::string model = (info.model == "equidistant") ? "kb4" : info.model;

    // Extract intrinsics: [fx, 0, cx, 0, fy, cy, 0, 0, 1] -> [fx, fy, cx, cy]
    std::vector<double> intrinsics;
    if (info.intrinsics.size() >= 6) {
        intrinsics = {info.intrinsics[0], info.intrinsics[4], info.intrinsics[2], info.intrinsics[5]};
    }

    // Calculate Extrinsics Matrix T_BS
    std::vector<double> t_bs_data;
    if (info.extrinsics.size() >= 7) {
        double tx = info.extrinsics[0], ty = info.extrinsics[1], tz = info.extrinsics[2];
        double qx = info.extrinsics[3], qy = info.extrinsics[4];
        double qz = info.extrinsics[5], qw = info.extrinsics[6];

        // Quaternion to Rotation Matrix
        double xx = qx * qx, yy = qy * qy, zz = qz * qz;
        double xy = qx * qy, xz = qx * qz, yz = qy * qz;
        double wx = qw * qx, wy = qw * qy, wz = qw * qz;

        double r00 = 1 - 2 * (yy + zz);
        double r01 = 2 * (xy - wz);
        double r02 = 2 * (xz + wy);
        double r10 = 2 * (xy + wz);
        double r11 = 1 - 2 * (xx + zz);
        double r12 = 2 * (yz - wx);
        double r20 = 2 * (xz - wy);
        double r21 = 2 * (yz + wx);
        double r22 = 1 - 2 * (xx + yy);

        t_bs_data = {r00, r01, r02, tx, r10, r11, r12, ty, r20, r21, r22, tz, 0.0, 0.0, 0.0, 1.0};
    } else {
        t_bs_data = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
    }

    // Format YAML string
    yaml << "# General sensor definitions.\n";
    yaml << "sensor_type: camera\n";
    yaml << "comment: DAS Camera cam0\n\n";

    yaml << "# Sensor extrinsics wrt. the body-frame.\n";
    yaml << "T_BS:\n";
    yaml << "  cols: 4\n";
    yaml << "  rows: 4\n";
    yaml << "  data: [";
    for (size_t i = 0; i < t_bs_data.size(); i++) {
        yaml << t_bs_data[i];
        if (i < t_bs_data.size() - 1) yaml << ", ";
    }
    yaml << "]\n\n";

    yaml << "# Camera specific definitions.\n";
    yaml << "rate_hz: 30\n";
    yaml << "resolution: [" << info.width << ", " << info.height << "]\n";
    yaml << "camera_model: " << model << "\n";
    yaml << "intrinsics: [";
    for (size_t i = 0; i < intrinsics.size(); i++) {
        yaml << intrinsics[i];
        if (i < intrinsics.size() - 1) yaml << ", ";
    }
    yaml << "] #fu, fv, cu, cv\n";
    yaml << "distortion_model: " << model << "\n";
    yaml << "distortion_coefficients: [";
    for (size_t i = 0; i < info.distortion.size(); i++) {
        yaml << info.distortion[i];
        if (i < info.distortion.size() - 1) yaml << ", ";
    }
    yaml << "] #k1, k2, k3, k4\n";

    return yaml.str();
}

std::optional<std::string> MessagePack::extractDasFramedPayload(const std::vector<uint8_t>& data) {
    const std::string magic(PackContent::MAGIC, PackContent::MAGIC + PackContent::MAGIC_LENGTH);
    std::string s(data.begin(), data.end());
    const size_t first = s.find(magic);
    if (first == std::string::npos) {
        return std::nullopt;
    }
    const size_t content_start = first + magic.size();
    const size_t second = s.find(magic, content_start);
    if (second == std::string::npos) {
        return std::nullopt;
    }
    return s.substr(content_start, second - content_start);
}

bool MessagePack::unpackCameraCalib(const std::vector<uint8_t>& data,
                                     const std::string& yaml_filename,
                                     const std::string& output_dir) {
    try {
        // Print data
        std::cout << "unpack data: ";
        for (auto b : data) {
            std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(b);
        }
        std::cout << std::dec << std::endl;

        if (!checkHead(data) || !checkTail(data)) {
            return false;
        }

        // Read length and opcode
        size_t length_pos = PackContent::MAGIC_LENGTH;
        uint16_t payload_length = (static_cast<uint16_t>(data[length_pos]) << 8) |
                                   static_cast<uint16_t>(data[length_pos + 1]);

        size_t opcode_pos = length_pos + 2;
        uint8_t opcode_value = data[opcode_pos];

        size_t payload_start = opcode_pos + 1;
        std::vector<uint8_t> payload(data.begin() + payload_start, 
                                      data.begin() + data.size() - PackContent::MAGIC_LENGTH);

        // Parse protobuf data
        CalibInfo calib_info = parseProtobufCalib(payload);

        // Generate YAML file if requested
        if (!yaml_filename.empty()) {
            std::string yaml_content = generateYaml(calib_info);
            std::cout << "Generated YAML content: " << yaml_content << std::endl;

            try {
                std::string result_dir = output_dir;
                if (result_dir.empty()) {
                    result_dir = "calib_result";
                }

                std::filesystem::create_directories(result_dir);

                std::string file_path = result_dir + "/" + yaml_filename;
                std::ofstream file(file_path);
                if (file.is_open()) {
                    file << yaml_content;
                    file.close();
                    std::cout << "Camera calibration file saved: " << file_path << std::endl;
                    return true;
                } else {
                    std::cerr << "Cannot open file: " << file_path << std::endl;
                    return false;
                }
            } catch (const std::exception& e) {
                std::cerr << "Save calibration file error: " << e.what() << std::endl;
                return false;
            }
        } else {
            std::cout << "Calibration data parsed (no YAML file generated)" << std::endl;
            return true;
        }
    } catch (const std::exception& e) {
        std::cerr << "Parse calibration data error: " << e.what() << std::endl;
        return false;
    }
}

} // namespace das
