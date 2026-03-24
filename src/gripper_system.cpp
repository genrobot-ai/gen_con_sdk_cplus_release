/**
 * @file gripper_system.cpp
 * @brief Gripper system main class implementation
 */

#include "gripper_system.hpp"
#include <iostream>
#include <csignal>
#include <sstream>

namespace das {

// Static member init
GripperSystem* GripperSystem::instance_ = nullptr;

// GripperSystem implementation
GripperSystem::GripperSystem(const std::string& serial_port,
                             const std::string& camera_resolutions,
                             bool show_preview,
                             const std::vector<std::string>& video_devices,
                             TactileCallback tactile_callback,
                             EncoderCallback encoder_callback,
                             FrameCallback capture_frames_callback)
    : running_(true)
    , serial_port_(serial_port)
    , camera_resolutions_(camera_resolutions)
    , show_preview_(show_preview)
    , video_devices_(video_devices)
    , tactile_callback_(tactile_callback)
    , encoder_callback_(encoder_callback)
    , capture_frames_callback_(capture_frames_callback)
{
    // Parse resolutions
    resolutions_ = parseResolutions(camera_resolutions);
    if (resolutions_.empty()) {
        resolutions_ = {{1600, 1296}};
    }

    // Register signal handler
    instance_ = this;
    std::signal(SIGINT, staticSignalHandler);
    std::signal(SIGTERM, staticSignalHandler);
}

GripperSystem::~GripperSystem() {
    stop();
    instance_ = nullptr;
}

void GripperSystem::staticSignalHandler(int signum) {
    if (instance_) {
        instance_->signalHandler(signum);
    }
}

void GripperSystem::signalHandler(int signum) {
    if (!running_) {
        exit(0);
    }
    std::cout << "\nReceived signal (" << signum << "), stopping system..." << std::endl;
    running_ = false;
}

std::vector<std::pair<int, int>> GripperSystem::parseResolutions(const std::string& res_str) {
    std::vector<std::pair<int, int>> resolutions;
    std::stringstream ss(res_str);
    std::string item;
    
    while (std::getline(ss, item, ',')) {
        // Trim spaces
        item.erase(std::remove_if(item.begin(), item.end(), ::isspace), item.end());
        
        size_t pos = item.find('x');
        if (pos != std::string::npos) {
            try {
                int width = std::stoi(item.substr(0, pos));
                int height = std::stoi(item.substr(pos + 1));
                resolutions.push_back({width, height});
            } catch (...) {
                // Ignore parse error
            }
        }
    }
    
    return resolutions;
}

bool GripperSystem::start() {
    std::cout << std::string(60, '=') << std::endl;
    std::cout << "Starting gripper system..." << std::endl;
    std::cout << std::string(60, '=') << std::endl;

    // 1. Find serial port
    if (serial_port_.empty()) {
        serial_port_ = findSerialPort("ttyUSB");
        if (serial_port_.empty()) {
            std::cout << "No serial port found" << std::endl;
            return false;
        }
    }

    std::cout << "Using serial port: " << serial_port_ << std::endl;

    // 2. Start camera first
    std::cout << "\n[1/2] Initializing camera..." << std::endl;
    try {
        camera_ = std::make_unique<CameraCapture>(
            serial_port_,
            3,
            resolutions_,
            show_preview_,
            video_devices_,
            nullptr
        );
        std::cout << "Camera initialized" << std::endl;
        
        // Re-register signal handler
        std::signal(SIGINT, staticSignalHandler);
        std::signal(SIGTERM, staticSignalHandler);
    } catch (const std::exception& e) {
        std::cerr << "Camera init failed: " << e.what() << std::endl;
        stop();
        return false;
    }

    // Allow device init time
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // 3. Start DataBus (serial)
    std::cout << "\n[2/2] Initializing serial..." << std::endl;
    try {
        databus_ = std::make_unique<DataBus>(
            serial_port_,
            921600,
            0.5,
            false,
            30,  // 30Hz encoder query
            0,   // Tactile query disabled
            tactile_callback_,
            encoder_callback_
        );
        std::cout << "Serial initialized" << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Serial init failed: " << e.what() << std::endl;
        stop();
        return false;
    }

    std::cout << "\n" << std::string(60, '=') << std::endl;
    std::cout << "System started" << std::endl;
    std::cout << std::string(60, '=') << std::endl;
    std::cout << "\nUsage:" << std::endl;
    std::cout << "  - Camera preview (3 windows) is open" << std::endl;
    std::cout << "  - Use GripperController to control gripper" << std::endl;
    std::cout << "  - Press ESC or Ctrl+C to stop" << std::endl;
    std::cout << std::string(60, '=') << std::endl;

    // Start camera capture
    try {
        std::thread camera_thread;
        if (capture_frames_callback_) {
            camera_thread = std::thread(capture_frames_callback_, camera_.get());
        } else {
            camera_thread = std::thread([this]() {
                camera_->captureFrames();
            });
        }
        camera_thread.detach();

        // Main loop
        while (running_) {
            try {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            } catch (...) {
                running_ = false;
                break;
            }
        }

    } catch (const std::exception& e) {
        std::cerr << "\nSystem error: " << e.what() << std::endl;
    }

    stop();
    return true;
}

void GripperSystem::stop() {
    std::cout << "\nStopping system..." << std::endl;

    if (camera_) {
        camera_->stop();
    }

    if (databus_) {
        databus_->stop();
    }

    std::cout << "System stopped" << std::endl;
}

void GripperSystem::setGripperDistance(float distance) {
    if (databus_) {
        databus_->setTargetDistance(distance);
    } else {
        std::cout << "DataBus not initialized" << std::endl;
    }
}

} // namespace das
