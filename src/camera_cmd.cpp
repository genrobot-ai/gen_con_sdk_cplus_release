/**
 * @file camera_cmd.cpp
 * @brief Camera calibration command tool
 *
 * Sends camera calibration commands and retrieves device information.
 */

#include <iostream>
#include <string>
#include <cstdlib>
#include <cstring>
#include <thread>
#include <chrono>
#include <filesystem>
#include "databus.hpp"
#include "pack.hpp"

using namespace das;

void printUsage(const char* program) {
    std::cout << "Usage:" << std::endl;
    std::cout << "  " << program << " [left|right] <command>" << std::endl;
    std::cout << std::endl;
    std::cout << "  Optional first argument left|right selects /dev/ttyDeviceLeft or /dev/ttyDeviceRight" << std::endl;
    std::cout << "  (default: left). Ignored if env SERIAL_PORT is set." << std::endl;
    std::cout << std::endl;
    std::cout << "  Optional env: SERIAL_PORT=/dev/... to override device path" << std::endl;
    std::cout << std::endl;
    std::cout << "  Commands:" << std::endl;
    std::cout << "    1234      - Calibration complete confirmation" << std::endl;
    std::cout << "    camerarc  - Center camera calibration (generates cam0_sensor_<left|right>.yaml)" << std::endl;
    std::cout << "    camerarl  - Left camera calibration (generates cam1_sensor_<left|right>.yaml)" << std::endl;
    std::cout << "    camerarr  - Right camera calibration (generates cam2_sensor_<left|right>.yaml)" << std::endl;
    std::cout << "    MCUID     - Query device MCUID" << std::endl;
    std::cout << std::endl;
    std::cout << "Examples:" << std::endl;
    std::cout << "  " << program << " MCUID              # left device, query MCUID" << std::endl;
    std::cout << "  " << program << " right MCUID        # right device" << std::endl;
    std::cout << "  " << program << " camerarc           # left device, center calib" << std::endl;
    std::cout << "  " << program << " right camerarl     # right device, left camera calib" << std::endl;
    std::cout << "  " << program << " 1234               # left device, calib OK" << std::endl;
}

void cameraCalibCallback(const std::vector<uint8_t>& data) {
    (void)data;
    std::cout << "Camera calibration data received" << std::endl;
}

int main(int argc, char* argv[]) {
    if (argc < 2) {
        printUsage(argv[0]);
        return 1;
    }

    std::string side = "left";
    std::string record_value;
    if (std::string(argv[1]) == "left" || std::string(argv[1]) == "right") {
        if (argc < 3) {
            std::cerr << "Error: missing <command> after \"" << argv[1] << "\"" << std::endl;
            printUsage(argv[0]);
            return 1;
        }
        side = argv[1];
        record_value = argv[2];
    } else {
        record_value = argv[1];
    }

    // Check help
    if (record_value == "-h" || record_value == "--help") {
        printUsage(argv[0]);
        return 0;
    }

    // Validate command
    std::vector<std::string> valid_commands = {"1234", "camerarc", "camerarl", "camerarr", "MCUID"};
    bool valid = false;
    for (const auto& cmd : valid_commands) {
        if (record_value == cmd) {
            valid = true;
            break;
        }
    }

    if (!valid) {
        std::cerr << "Error: command must be one of 1234, camerarc, camerarl, camerarr, MCUID" << std::endl;
        printUsage(argv[0]);
        return 1;
    }

    // Determine YAML filename from command (suffix left|right matches CLI device side)
    std::string yaml_filename;
    if (record_value == "camerarc") {
        yaml_filename = "cam0_sensor_" + side + ".yaml";
    } else if (record_value == "camerarl") {
        yaml_filename = "cam1_sensor_" + side + ".yaml";
    } else if (record_value == "camerarr") {
        yaml_filename = "cam2_sensor_" + side + ".yaml";
    }

    // Get output directory
    std::string output_dir;
    std::filesystem::path exe_path = std::filesystem::canonical("/proc/self/exe");
    std::filesystem::path exe_dir = exe_path.parent_path();
    output_dir = (exe_dir / ".." / "calib_result").string();

    if (!yaml_filename.empty()) {
        std::cout << "Will generate YAML file: " << yaml_filename << std::endl;
        std::cout << "Save path: " << output_dir << "/" << yaml_filename << std::endl;
    }

    const bool is_mcuid = (record_value == "MCUID");

    // Serial port: prefer env SERIAL_PORT, else find configured device
    std::string serial_port;
    const char* env_port = std::getenv("SERIAL_PORT");
    if (env_port && strlen(env_port) > 0) {
        serial_port = env_port;
    } else {
        serial_port = findGripperSerialBySide(side, !is_mcuid);
    }

    if (serial_port.empty()) {
        std::cerr << "No configured serial port found" << std::endl;
        return 1;
    }

    if (!is_mcuid) {
        std::cout << "Using serial port: " << serial_port << std::endl;
        std::cout << "Sending camera calibration command: " << record_value << std::endl;
    }

    // Create bus and send command
    try {
        DataBus bus(
            serial_port,
            921600,
            0.5,
            true,  // is_calib_cmd
            0,     // encoder_freq
            0,     // tactile_freq
            nullptr,
            nullptr,
            is_mcuid ? nullptr : cameraCalibCallback,
            yaml_filename,
            output_dir,
            is_mcuid,  // quiet_console
            is_mcuid   // mcuid_ascii_only
        );

        std::this_thread::sleep_for(std::chrono::seconds(1));

        // Send calibration command
        std::vector<uint8_t> record(record_value.begin(), record_value.end());
        bus.addCmd(CmdPack::packCalib(record));

        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        if (!is_mcuid) {
            std::cout << "Waiting for device response..." << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::seconds(2));

        bus.stop();

        // Special command handling
        if (record_value == "1234") {
            std::cout << "Calibration OK!" << std::endl;
        } else if (record_value == "MCUID") {
            // MCUID value printed in DataBus parsing (MCUID: ...)
        } else {
            std::cout << "Command sent: " << record_value << std::endl;
            if (!yaml_filename.empty()) {
                std::string yaml_path = output_dir + "/" + yaml_filename;
                if (std::filesystem::exists(yaml_path)) {
                    std::cout << "YAML file generated: " << yaml_path << std::endl;
                } else {
                    std::cout << "YAML file not generated, check device response" << std::endl;
                }
            }
        }

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
