#!/bin/bash
DIR="$(cd "$(dirname "$0")" && pwd)"; BIN=$(mktemp -u).out; ERR=$(mktemp 2>/dev/null || echo /tmp/sg_err); tail -n +4 "$0" | g++ -std=c++17 -w -O2 -o "$BIN" -x c++ - -I"$DIR/include" -L"$DIR/lib" -L"$DIR/build" $(pkg-config --cflags --libs opencv4 2>/dev/null || pkg-config --cflags --libs opencv 2>/dev/null) -lcpp_sdk_static -lpthread 2>"$ERR"; G=$?; [ $G -ne 0 ] && { cat "$ERR" >&2; rm -f "$ERR" "$BIN"; exit $G; }; rm -f "$ERR"; exec -a "$0" "$BIN" "$@"

/**
 * @file start_gripper.cpp
 * @brief Gripper system startup main program
 *
 * Starts gripper device and cameras, supports sine wave control.
 */

#include <iostream>
#include <string>
#include <vector>
#include <cstring>
#include <cstdlib>
#include <iomanip>
#include <thread>
#include <chrono>
#include <cmath>
#include <opencv2/opencv.hpp>
#include "gripper_system.hpp"
#include "databus.hpp"
#include "camera.hpp"
#include "tactile_processing.hpp"

using namespace das;

// ---------------------------------------------------------------------------
// Sine wave and GripperController implementation (moved from gripper_system.cpp)
// ---------------------------------------------------------------------------
namespace das {

SineWaveController::SineWaveController(DataBus* databus,
                                       float amplitude,
                                       float center,
                                       float frequency,
                                       float duration)
    : databus_(databus)
    , amplitude_(amplitude)
    , center_(center)
    , frequency_(frequency)
    , duration_(duration)
    , running_(false)
    , control_rate_(50.0f)
    , control_interval_(1.0f / control_rate_)
{
}

SineWaveController::~SineWaveController() {
    stop();
}

void SineWaveController::start() {
    if (running_) {
        std::cout << "Sine wave control already running" << std::endl;
        return;
    }

    if (amplitude_ <= 0) {
        std::cout << "Amplitude must be > 0" << std::endl;
        return;
    }

    if (center_ - amplitude_ < 0) {
        std::cout << "Sine wave minimum out of range" << std::endl;
        return;
    }

    if (center_ + amplitude_ > 0.103f) {
        std::cout << "Sine wave maximum out of range" << std::endl;
        return;
    }

    running_ = true;
    start_time_ = std::chrono::steady_clock::now();

    control_thread_ = std::make_unique<std::thread>(&SineWaveController::controlLoop, this);

    std::cout << std::string(60, '=') << std::endl;
    std::cout << "Sine wave control started" << std::endl;
    std::cout << "   Center: " << center_ << " m" << std::endl;
    std::cout << "   Amplitude: +/-" << amplitude_ << " m" << std::endl;
    std::cout << "   Frequency: " << frequency_ << " Hz" << std::endl;
    std::cout << "   Range: " << (center_ - amplitude_) << " - " << (center_ + amplitude_) << " m" << std::endl;
    if (duration_ > 0) {
        std::cout << "   Duration: " << duration_ << " s" << std::endl;
    } else {
        std::cout << "   Duration: infinite" << std::endl;
    }
    std::cout << std::string(60, '=') << std::endl;
}

void SineWaveController::stop() {
    if (!running_) return;

    running_ = false;
    if (control_thread_ && control_thread_->joinable()) {
        control_thread_->join();
    }

    auto elapsed = std::chrono::steady_clock::now() - start_time_;
    auto elapsed_sec = std::chrono::duration<float>(elapsed).count();
    std::cout << "\nSine wave control stopped, runtime: " << elapsed_sec << " s" << std::endl;
}

void SineWaveController::controlLoop() {
    try {
        while (running_) {
            auto cycle_start = std::chrono::steady_clock::now();

            auto current_duration = std::chrono::steady_clock::now() - start_time_;
            float current_time = std::chrono::duration<float>(current_duration).count();

            if (duration_ > 0 && current_time >= duration_) {
                std::cout << "Sine wave control finished (duration reached)" << std::endl;
                running_ = false;
                break;
            }

            float value = center_ + amplitude_ * std::sin(2.0f * M_PI * frequency_ * current_time);
            value = std::max(0.0f, std::min(0.103f, value));

            if (databus_) {
                databus_->setTargetDistance(value);
            }

            auto elapsed = std::chrono::steady_clock::now() - cycle_start;
            auto sleep_time = std::chrono::duration<float>(control_interval_) - elapsed;
            if (sleep_time.count() > 0) {
                std::this_thread::sleep_for(sleep_time);
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "Sine wave control error: " << e.what() << std::endl;
        running_ = false;
    }
}

GripperController::GripperController(DataBus* databus)
    : databus_(databus)
{
}

GripperController::~GripperController() {
    stopSineWave();
}

void GripperController::setFixedDistance(float distance) {
    if (distance < 0.0f || distance > 0.103f) {
        std::cout << "Warning: distance " << distance << " out of range [0.0, 0.103], ignored" << std::endl;
        return;
    }

    if (sine_wave_controller_ && sine_wave_controller_->isRunning()) {
        sine_wave_controller_->stop();
    }

    try {
        if (databus_) {
            databus_->setTargetDistance(distance);
        }
        std::cout << "Set gripper fixed distance: " << distance << " m (" << (distance * 100) << " cm)" << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Failed to set gripper distance: " << e.what() << std::endl;
    }
}

void GripperController::startSineWave(float amplitude, float center, float frequency, float duration) {
    if (sine_wave_controller_ && sine_wave_controller_->isRunning()) {
        sine_wave_controller_->stop();
    }

    sine_wave_controller_ = std::make_unique<SineWaveController>(
        databus_, amplitude, center, frequency, duration);

    sine_wave_controller_->start();
}

void GripperController::stopSineWave() {
    if (sine_wave_controller_) {
        sine_wave_controller_->stop();
    }
}

bool GripperController::isSineWaveRunning() const {
    if (sine_wave_controller_) {
        return sine_wave_controller_->isRunning();
    }
    return false;
}

} // namespace das

// Left/right side config
struct SideConfig {
    std::string serial_port;
    std::vector<std::string> video_devices;
};

std::map<std::string, SideConfig> SIDE_CONFIG = {
    {"left", {
        "/dev/ttyDeviceLeft",
        {
            "/dev/left_video_0_main", "/dev/left_video_0_sec",
            "/dev/left_video_1_main", "/dev/left_video_1_sec",
            "/dev/left_video_2_main", "/dev/left_video_2_sec"
        }
    }},
    {"right", {
        "/dev/ttyDeviceRight",
        {
            "/dev/right_video_0_main", "/dev/right_video_0_sec",
            "/dev/right_video_1_main", "/dev/right_video_1_sec",
            "/dev/right_video_2_main", "/dev/right_video_2_sec"
        }
    }}
};

/**
 * @brief Camera frame capture callback — uses background grab thread
 */
void capture_frames_callback(CameraCapture* camera) {
    std::cout << "\nCapturing " << camera->cameras_.size() << " camera(s)..." << std::endl;
    std::cout << "Press ESC or Ctrl+C to stop" << std::endl;

    if (camera->show_preview_) {
        for (auto& cam : camera->cameras_) {
            cv::namedWindow(cam.window_name, cv::WINDOW_NORMAL);
            cv::resizeWindow(cam.window_name, 640, 480);
        }
    }

    camera->startGrabThread();

    double frame_interval = 1.0 / static_cast<double>(camera->target_fps_ > 0 ? camera->target_fps_ : 30);

    try {
        while (camera->running_) {
            auto start_time = std::chrono::steady_clock::now();
            std::vector<std::pair<CameraInfo*, cv::Mat>> frames_data;

            for (auto& cam : camera->cameras_) {
                auto [frame, ts_ns] = camera->getLatest(cam);
                if (!frame.empty()) {
                    double now_sec = std::chrono::duration<double>(
                        std::chrono::steady_clock::now().time_since_epoch()).count();
                    cam.disp_fps_ts.push_back(now_sec);
                    if (cam.disp_fps_ts.size() > 30) cam.disp_fps_ts.pop_front();
                    if (cam.disp_fps_ts.size() >= 2) {
                        double dt = cam.disp_fps_ts.back() - cam.disp_fps_ts.front();
                        if (dt > 0) {
                            cam.disp_fps_val = (static_cast<double>(cam.disp_fps_ts.size()) - 1.0) / dt;
                        }
                    }
                }
                frames_data.push_back({&cam, frame});
            }

            if (camera->show_preview_) {
                for (const auto& [cam, frame] : frames_data) {
                    if (!frame.empty()) {
                        auto now = std::chrono::system_clock::now();
                        auto time = std::chrono::system_clock::to_time_t(now);
                        std::tm tm = *std::localtime(&time);
                        std::ostringstream oss;
                        oss << std::put_time(&tm, "%Y-%m-%d %H:%M:%S");

                        std::string info_text = "Camera_" + std::to_string(cam->id) + " | "
                                                + oss.str() + " | Frames: " + std::to_string(cam->frame_count);

                        cv::Mat display_frame = frame.clone();
                        cv::putText(display_frame, info_text, cv::Point(10, 30),
                                   cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);

                        std::ostringstream fps_oss;
                        fps_oss << "Cap: " << std::fixed << std::setprecision(1) << cam->cap_fps_val
                                << "  Disp: " << std::fixed << std::setprecision(1) << cam->disp_fps_val;
                        cv::putText(display_frame, fps_oss.str(), cv::Point(10, 60),
                                   cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);

                        cv::imshow(cam->window_name, display_frame);
                    }
                }
                if (cv::waitKey(1) == 27) {
                    camera->running_ = false;
                }
            }

            auto elapsed = std::chrono::steady_clock::now() - start_time;
            auto sleep_time = std::chrono::duration<double>(frame_interval) - elapsed;
            if (sleep_time.count() > 0) {
                std::this_thread::sleep_for(sleep_time);
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "Capture error: " << e.what() << std::endl;
    }

    camera->stopGrabThread();
    for (auto& cam : camera->cameras_) {
        try { cam.cap.release(); } catch (...) {}
    }
    if (camera->show_preview_) {
        for (auto& cam : camera->cameras_) {
            try { cv::destroyWindow(cam.window_name); } catch (...) {}
        }
    }
}

/**
 * @brief Build tactile callback: prints grid only when print_tactile_info is true.
 */
das::GripperSystem::TactileCallback make_tactile_callback(bool print_tactile_info) {
    return [print_tactile_info](const std::vector<uint8_t>& record_data) {
        if (record_data.size() != 448) {
            return;
        }

        try {
            std::vector<int> tactile_1000 = das::convert_tactile_448_to_1000(record_data);
            if (print_tactile_info) {
                das::print_tactile_1000_grid(tactile_1000);
            }
        } catch (const std::exception& e) {
            std::cerr << "Tactile data error: " << e.what() << std::endl;
        }
    };
}

/**
 * @brief Encoder data callback
 */
void encoder_callback(const std::vector<uint8_t>& record_data) {
    try {
        if (record_data.size() >= 4) {
            float encoder_value = bigEndianBytesToFloat(record_data.data());
            std::cout << "gripper distance: " << encoder_value << " m" << std::endl;
        }
    } catch (const std::exception& e) {
        std::cerr << "Encoder data error: " << e.what() << std::endl;
    }
}

void printUsage(const char* program) {
    std::cout << "Usage: " << program << " <side> [options]" << std::endl;
    std::cout << std::endl;
    std::cout << "Positional:" << std::endl;
    std::cout << "  side                    Gripper side (left or right)" << std::endl;
    std::cout << std::endl;
    std::cout << "Options:" << std::endl;
    std::cout << "  --camera-resolutions    Camera resolution 'widthxheight' (default: 1600x1296)" << std::endl;
    std::cout << "  --no-preview            No camera preview window" << std::endl;
    std::cout << "  --camera-fps <n>        Target camera display frame rate (default 30)" << std::endl;
    std::cout << "  --distance <m>          Fixed gripper distance (m), range [0.0, 0.103]" << std::endl;
    std::cout << "  --sine-wave             Enable sine wave control" << std::endl;
    std::cout << "  --amplitude <m>         Sine amplitude (m), default 0.025" << std::endl;
    std::cout << "  --center <m>            Sine center (m), default 0.05" << std::endl;
    std::cout << "  --frequency <Hz>         Sine frequency (Hz), default 0.5" << std::endl;
    std::cout << "  --duration <s>           Sine duration (s), 0=infinite, default 10.0" << std::endl;
    std::cout << "  --print-tactile-info    Print tactile grid in tactile callback (default: off)" << std::endl;
    std::cout << "  -h, --help              Show this help" << std::endl;
    std::cout << std::endl;
    std::cout << "Examples:" << std::endl;
    std::cout << "  " << program << " left                      # Start left gripper" << std::endl;
    std::cout << "  " << program << " right --distance 0.1      # Right gripper, 10cm" << std::endl;
    std::cout << "  " << program << " left --sine-wave          # Left gripper, sine wave" << std::endl;
    std::cout << "  " << program << " right --print-tactile-info  # Right gripper, print tactile grid" << std::endl;
    std::cout << "  " << program << " left --camera-fps 60       # Left gripper, camera fps 60 (V4 Controller needs 60 for ~30fps images)" << std::endl;
}

int main(int argc, char* argv[]) {
    if (argc < 2) {
        printUsage(argv[0]);
        return 1;
    }

    // Check help
    for (int i = 1; i < argc; i++) {
        if (std::string(argv[i]) == "-h" || std::string(argv[i]) == "--help") {
            printUsage(argv[0]);
            return 0;
        }
    }

    // Parse args
    std::string side = argv[1];
    if (side != "left" && side != "right") {
        std::cerr << "Error: side must be 'left' or 'right'" << std::endl;
        printUsage(argv[0]);
        return 1;
    }

    std::string camera_resolutions = "1600x1296";
    bool show_preview = true;
    bool use_sine_wave = false;
    float distance = -1.0f;  // -1 = not set
    float amplitude = 0.025f;
    float center = 0.05f;
    float frequency = 0.5f;
    float duration = 10.0f;
    bool print_tactile_info = false;
    int camera_fps = 30;

    for (int i = 2; i < argc; i++) {
        std::string arg = argv[i];

        if (arg == "--camera-resolutions" && i + 1 < argc) {
            camera_resolutions = argv[++i];
        } else if (arg == "--no-preview") {
            show_preview = false;
        } else if (arg == "--camera-fps" && i + 1 < argc) {
            camera_fps = std::stoi(argv[++i]);
        } else if (arg == "--print-tactile-info") {
            print_tactile_info = true;
        } else if (arg == "--distance" && i + 1 < argc) {
            distance = std::stof(argv[++i]);
        } else if (arg == "--sine-wave") {
            use_sine_wave = true;
        } else if (arg == "--amplitude" && i + 1 < argc) {
            amplitude = std::stof(argv[++i]);
        } else if (arg == "--center" && i + 1 < argc) {
            center = std::stof(argv[++i]);
        } else if (arg == "--frequency" && i + 1 < argc) {
            frequency = std::stof(argv[++i]);
        } else if (arg == "--duration" && i + 1 < argc) {
            duration = std::stof(argv[++i]);
        }
    }

    // Validate distance
    if (distance >= 0.0f && distance > 0.103f) {
        std::cerr << "Error: distance " << distance << " out of range [0.0, 0.103] m (max 10cm)" << std::endl;
        return 1;
    }
    if (distance < 0.0f && distance != -1.0f) {
        std::cerr << "Error: distance " << distance << " out of range [0.0, 0.103] m" << std::endl;
        return 1;
    }

    // Get config
    const auto& config = SIDE_CONFIG[side];
    std::string serial_port = config.serial_port;
    std::vector<std::string> video_devices = config.video_devices;

    // Create system
    GripperSystem system(
        serial_port,
        camera_resolutions,
        show_preview,
        video_devices,
        make_tactile_callback(print_tactile_info),
        encoder_callback,
        capture_frames_callback,
        camera_fps
    );

    // Setup control mode
    auto setupControlMode = [&]() {
        // Wait for system init
        float max_wait_time = 10.0f;
        float wait_interval = 0.1f;
        float elapsed_time = 0.0f;

        while (elapsed_time < max_wait_time) {
            if (system.getDataBus() != nullptr) {
                std::this_thread::sleep_for(std::chrono::milliseconds(500));

                GripperController controller(system.getDataBus());

                if (use_sine_wave) {
                    std::cout << "\n" << std::string(60, '=') << std::endl;
                    std::cout << "Control mode: sine wave" << std::endl;
                    std::cout << std::string(60, '=') << std::endl;
                    controller.startSineWave(amplitude, center, frequency, duration);
                } else if (distance >= 0) {
                    std::cout << "\n" << std::string(60, '=') << std::endl;
                    std::cout << "Control mode: fixed distance" << std::endl;
                    std::cout << std::string(60, '=') << std::endl;
                    controller.setFixedDistance(distance);
                } else {
                    std::cout << "\n" << std::string(60, '=') << std::endl;
                    std::cout << "Control mode: default center (0.05m)" << std::endl;
                    std::cout << std::string(60, '=') << std::endl;
                    controller.setFixedDistance(0.05f);
                }

                // Wait for sine wave to finish
                while (controller.isSineWaveRunning() && system.isRunning()) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }

                return;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(wait_interval * 1000)));
            elapsed_time += wait_interval;
        }

        std::cout << "Warning: system init timeout, control mode not set" << std::endl;
    };

    // Start control mode thread
    std::thread control_thread(setupControlMode);
    control_thread.detach();

    // Start system
    try {
        system.start();
    } catch (const std::exception& e) {
        std::cerr << "System error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
