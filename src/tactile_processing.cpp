/**
 * @file tactile_processing.cpp
 * @brief Same semantics as gen_controller_sdk_python/tactile_processing.py
 */

#include "tactile_processing.hpp"

#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <stdexcept>
#include <utility>

namespace das {

namespace {

// Mirrors LEFT_NEG_COORDS / RIGHT_NEG_COORDS in tactile_processing.py
static const std::pair<int, int> kLeftNegCoords[] = {
    {0, 0},   {0, 1},   {0, 2},   {1, 0},   {1, 1},   {2, 0},
    {0, 7},   {0, 8},   {0, 9},   {1, 8},   {1, 9},   {2, 9},
    {49, 0},  {49, 1},  {49, 2},  {49, 3},
    {48, 0},  {48, 1},  {48, 2},  {48, 3},
    {47, 0},  {47, 1},  {47, 2},
    {46, 0},  {46, 1},  {46, 2},
    {45, 0},  {45, 1},  {45, 2},
    {44, 0},  {44, 1},
    {43, 0},
    {49, 6},  {49, 7},  {49, 8},  {49, 9},
    {48, 6},  {48, 7},  {48, 8},  {48, 9},
    {47, 7},  {47, 8},  {47, 9},
    {46, 7},  {46, 8},  {46, 9},
    {45, 7},  {45, 8},  {45, 9},
    {44, 8},  {44, 9},
    {43, 9},
};

static const std::pair<int, int> kRightNegCoords[] = {
    {50, 0},  {50, 1},  {50, 2},  {51, 0},  {51, 1},  {52, 0},
    {50, 7},  {50, 8},  {50, 9},  {51, 8},  {51, 9},  {52, 9},
    {99, 0},  {99, 1},  {99, 2},  {99, 3},
    {98, 0},  {98, 1},  {98, 2},  {98, 3},
    {97, 0},  {97, 1},  {97, 2},
    {96, 0},  {96, 1},  {96, 2},
    {95, 0},  {95, 1},  {95, 2},
    {94, 0},  {94, 1},
    {93, 0},
    {99, 6},  {99, 7},  {99, 8},  {99, 9},
    {98, 6},  {98, 7},  {98, 8},  {98, 9},
    {97, 7},  {97, 8},  {97, 9},
    {96, 7},  {96, 8},  {96, 9},
    {95, 7},  {95, 8},  {95, 9},
    {94, 8},  {94, 9},
    {93, 9},
};

static int to_signed_int8(int value) {
    if (value == -1) {
        return -1;
    }
    if (value < 128) {
        return value;
    }
    return value - 256;
}

}  // namespace

std::vector<int> convert_tactile_448_to_1000(const std::vector<uint8_t>& record_data) {
    if (record_data.size() != 448) {
        throw std::invalid_argument("Expected 448 tactile bytes, got " + std::to_string(record_data.size()));
    }

    std::vector<int> left_expanded;
    left_expanded.reserve(448);
    for (size_t i = 0; i < 224; ++i) {
        int v = static_cast<int>(record_data[i]);
        left_expanded.push_back(v);
        left_expanded.push_back(v);
    }

    std::vector<int> right_expanded;
    right_expanded.reserve(448);
    for (size_t i = 224; i < 448; ++i) {
        int v = static_cast<int>(record_data[i]);
        right_expanded.push_back(v);
        right_expanded.push_back(v);
    }

    int total_grid[100][10];
    for (int r = 0; r < 100; ++r) {
        for (int c = 0; c < 10; ++c) {
            total_grid[r][c] = 0;
        }
    }

    for (const auto& p : kLeftNegCoords) {
        total_grid[p.first][p.second] = -1;
    }
    for (const auto& p : kRightNegCoords) {
        total_grid[p.first][p.second] = -1;
    }

    size_t left_idx = 0;
    for (int row = 0; row < 50; ++row) {
        for (int col = 0; col < 10; ++col) {
            if (total_grid[row][col] != -1 && left_idx < left_expanded.size()) {
                total_grid[row][col] = left_expanded[left_idx];
                ++left_idx;
            }
        }
    }

    size_t right_idx = 0;
    for (int row = 50; row < 100; ++row) {
        for (int col = 0; col < 10; ++col) {
            if (total_grid[row][col] != -1 && right_idx < right_expanded.size()) {
                total_grid[row][col] = right_expanded[right_idx];
                ++right_idx;
            }
        }
    }

    std::vector<int> left_flat;
    left_flat.reserve(500);
    for (int row = 0; row < 50; ++row) {
        for (int col = 0; col < 10; ++col) {
            left_flat.push_back(to_signed_int8(total_grid[row][col]));
        }
    }

    std::vector<int> right_flat;
    right_flat.reserve(500);
    for (int row = 50; row < 100; ++row) {
        for (int col = 0; col < 10; ++col) {
            right_flat.push_back(to_signed_int8(total_grid[row][col]));
        }
    }

    std::vector<int> out;
    out.reserve(1000);
    out.insert(out.end(), left_flat.begin(), left_flat.end());
    out.insert(out.end(), right_flat.begin(), right_flat.end());
    return out;
}

std::string build_tactile_1000_grid_text(const std::vector<int>& all_tactile) {
    static bool tactile_grid_blank_before_next_frame = false;

    if (all_tactile.size() != 1000) {
        throw std::invalid_argument("Expected 1000 tactile values, got " + std::to_string(all_tactile.size()));
    }

    std::ostringstream parts;
    if (tactile_grid_blank_before_next_frame) {
        parts << '\n';
    }

    const char* gap = "          ";
    for (int row = 0; row < 50; ++row) {
        const int lo = row * 10;
        for (int i = 0; i < 10; ++i) {
            if (i > 0) {
                parts << ' ';
            }
            parts << std::setw(3) << all_tactile[lo + i];
        }
        parts << gap;
        for (int i = 0; i < 10; ++i) {
            if (i > 0) {
                parts << ' ';
            }
            parts << std::setw(3) << all_tactile[500 + lo + i];
        }
        if (row < 49) {
            parts << '\n';
        }
    }
    tactile_grid_blank_before_next_frame = true;
    return parts.str();
}

void print_tactile_1000_grid(const std::vector<int>& all_tactile) {
    std::cout << build_tactile_1000_grid_text(all_tactile) << std::endl;
}

}  // namespace das
