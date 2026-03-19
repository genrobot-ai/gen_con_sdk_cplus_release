/**
 * @file das_protocol.cpp
 * @brief DAS protocol parsing module implementation
 */

#include "das_protocol.hpp"
#include <iostream>
#include <cstring>
#include <algorithm>

namespace das {

bool DASProtocol::checkMagic(const uint8_t* data, size_t length) {
    if (length < MAGIC_LENGTH) return false;
    return std::memcmp(data, MAGIC, MAGIC_LENGTH) == 0;
}

std::tuple<std::vector<std::vector<uint8_t>>, std::vector<uint8_t>> 
DASProtocol::findPacket(const std::vector<uint8_t>& data) {
    std::vector<std::vector<uint8_t>> packets;
    std::vector<uint8_t> buffer = data;

    if (buffer.size() > MAX_BUFFER_SIZE) {
        std::cerr << "[DASProtocol] Buffer too large: " << buffer.size() << " bytes, clearing" << std::endl;
        return {packets, {}};
    }

    size_t search_start = 0;
    size_t processed_count = 0;

    while (buffer.size() - search_start >= MAGIC_LENGTH * 2) {
        processed_count++;
        if (processed_count > 1000) {
            std::cerr << "[DASProtocol] Possible infinite loop detected, exiting" << std::endl;
            break;
        }

        // Find packet header
        auto header_it = std::search(
            buffer.begin() + search_start, buffer.end(),
            MAGIC, MAGIC + MAGIC_LENGTH
        );
        
        if (header_it == buffer.end()) {
            break;
        }
        
        size_t header_pos = header_it - buffer.begin();

        // Check for consecutive magic
        if (header_pos + MAGIC_LENGTH < buffer.size()) {
            auto next_magic_it = std::search(
                buffer.begin() + header_pos + MAGIC_LENGTH, buffer.end(),
                MAGIC, MAGIC + MAGIC_LENGTH
            );
            if (next_magic_it == buffer.begin() + header_pos + MAGIC_LENGTH) {
                search_start = header_pos + MAGIC_LENGTH;
                continue;
            }
        }

        // Check header position
        if (header_pos > buffer.size() - MAGIC_LENGTH * 2) {
            break;
        }

        // Find packet footer
        size_t footer_search_start = header_pos + MAGIC_LENGTH;
        auto footer_it = std::search(
            buffer.begin() + footer_search_start, buffer.end(),
            MAGIC, MAGIC + MAGIC_LENGTH
        );

        if (footer_it == buffer.end()) {
            break;
        }

        size_t footer_pos = footer_it - buffer.begin();

        if (footer_pos > buffer.size() - MAGIC_LENGTH) {
            break;
        }

        size_t packet_end = footer_pos + MAGIC_LENGTH;
        std::vector<uint8_t> full_packet(buffer.begin() + header_pos, buffer.begin() + packet_end);

        if (full_packet.size() > MAX_PACKET_SIZE) {
            std::cerr << "[DASProtocol] Packet too large: " << full_packet.size() << " bytes, skip" << std::endl;
            search_start = header_pos + MAGIC_LENGTH;
            continue;
        }

        if (validatePacketStructure(full_packet)) {
            packets.push_back(full_packet);
            search_start = packet_end;
        } else {
            search_start = header_pos + MAGIC_LENGTH;
        }
    }

    std::vector<uint8_t> remaining_data(buffer.begin() + search_start, buffer.end());
    return {packets, remaining_data};
}

bool DASProtocol::validatePacketStructure(const std::vector<uint8_t>& packet) {
    try {
        // Check length
        if (packet.size() < MAGIC_LENGTH * 2) {
            return false;
        }

        // Check head/tail magic
        if (!checkMagic(packet.data(), MAGIC_LENGTH)) {
            return false;
        }

        if (!checkMagic(packet.data() + packet.size() - MAGIC_LENGTH, MAGIC_LENGTH)) {
            return false;
        }

        // Check content length
        size_t content_length = packet.size() - MAGIC_LENGTH * 2;
        if (content_length < 1) {
            return false;
        }

        return true;
    } catch (...) {
        return false;
    }
}

std::optional<DASProtocol::PacketInfo> DASProtocol::parsePacket(const std::vector<uint8_t>& packet) {
    try {
        if (!validatePacketStructure(packet)) {
            return std::nullopt;
        }

        PacketInfo info;
        info.raw_packet = packet;
        info.packet_length = packet.size();
        info.timestamp = std::chrono::system_clock::now();

        // Extract payload (strip head/tail magic)
        size_t content_start = MAGIC_LENGTH;
        size_t content_end = packet.size() - MAGIC_LENGTH;

        info.opcode = packet[content_start];
        
        if (content_end > content_start + 1) {
            info.data_section.assign(
                packet.begin() + content_start + 1,
                packet.begin() + content_end
            );
        }
        info.data_length = info.data_section.size();

        return info;
    } catch (...) {
        return std::nullopt;
    }
}

std::vector<uint8_t> DASProtocol::createPacket(uint8_t opcode, const std::vector<uint8_t>& data) {
    if (data.size() > 1024) {
        throw std::runtime_error("Data length exceeds limit");
    }

    std::vector<uint8_t> packet;
    packet.reserve(MAGIC_LENGTH * 2 + 1 + data.size());

    // Add header magic
    packet.insert(packet.end(), MAGIC, MAGIC + MAGIC_LENGTH);
    
    // Add opcode
    packet.push_back(opcode);
    
    // Add data
    packet.insert(packet.end(), data.begin(), data.end());
    
    // Add footer magic
    packet.insert(packet.end(), MAGIC, MAGIC + MAGIC_LENGTH);

    return packet;
}

} // namespace das
