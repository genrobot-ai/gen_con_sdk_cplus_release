/**
 * @file tactile_processing.hpp
 * @brief 448-byte tactile packet -> 1000-value grid; grid text format matches
 *        gen_controller_sdk_python/tactile_processing.py
 */

#ifndef TACTILE_PROCESSING_HPP
#define TACTILE_PROCESSING_HPP

#include <cstdint>
#include <cstddef>
#include <string>
#include <vector>

namespace das {

/**
 * Convert one 448-byte tactile packet to 1000 signed cell values (left 500 + right 500).
 * Mirrors tactile_processing.convert_tactile_448_to_1000.
 */
std::vector<int> convert_tactile_448_to_1000(const std::vector<uint8_t>& record_data);

/**
 * Format 1000 values as 50 lines: row i = left row i (10 cells) + 10 spaces + right row i (10 cells).
 * Consecutive calls insert a blank line before the next frame (same as Python _build_tactile_1000_grid_text).
 */
std::string build_tactile_1000_grid_text(const std::vector<int>& all_tactile);

void print_tactile_1000_grid(const std::vector<int>& all_tactile);

}  // namespace das

#endif
