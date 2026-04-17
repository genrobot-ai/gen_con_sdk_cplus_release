/**
 * @file camera.cpp
 * @brief Camera capture module implementation
 */

#include "camera.hpp"
#include <iostream>
#include <algorithm>
#include <regex>
#include <filesystem>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <chrono>
#include <thread>
#include <cstdlib>
#include <csignal>
#include <fstream>
#include <mutex>

namespace das {

CameraCapture::CameraCapture(const std::string& serial_port,
                             int camera_count,
                             const std::vector<std::pair<int, int>>& resolutions,
                             bool show_preview,
                             const std::vector<std::string>& video_devices,
                             FrameCallback frame_callback,
                             int target_fps)
    : running_(true)
    , show_preview_(show_preview)
    , frame_callback_(frame_callback)
    , target_fps_(target_fps)
    , serial_port_(serial_port)
    , camera_count_(camera_count)
    , resolutions_(resolutions)
    , video_devices_(video_devices)
{
    if (resolutions_.empty()) {
        resolutions_ = {{1600, 1296}};
    }
    initCameras();
}

CameraCapture::~CameraCapture() {
    stop();
}

std::vector<std::string> CameraCapture::getPhysicalDevices() {
    std::vector<std::string> devices;
    
    try {
        // List devices with v4l2-ctl
        FILE* pipe = popen("v4l2-ctl --list-devices 2>/dev/null", "r");
        if (pipe) {
            char buffer[256];
            while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
                std::string line(buffer);
                // Strip newline
                line.erase(std::remove(line.begin(), line.end(), '\n'), line.end());
                line.erase(std::remove(line.begin(), line.end(), '\r'), line.end());
                
                // Check video device
                if (line.find("/dev/video") != std::string::npos) {
                    size_t start = line.find("/dev/video");
                    if (start != std::string::npos) {
                        std::string dev_path = line.substr(start);
                        // Trim trailing space
                        dev_path.erase(dev_path.find_last_not_of(" \t") + 1);
                        if (std::filesystem::exists(dev_path)) {
                            devices.push_back(dev_path);
                        }
                    }
                }
            }
            pclose(pipe);
        }

        std::cout << "Detected video devices: ";
        for (const auto& dev : devices) {
            std::cout << dev << " ";
        }
        std::cout << std::endl;

        // Use specified video devices if given
        if (!video_devices_.empty()) {
            std::vector<std::string> filtered_devices;
            for (const auto& dev : video_devices_) {
                if (std::filesystem::exists(dev)) {
                    filtered_devices.push_back(dev);
                }
            }
            if (!filtered_devices.empty()) {
                devices = filtered_devices;
                std::cout << "Using specified video devices: ";
                for (const auto& dev : devices) {
                    std::cout << dev << " ";
                }
                std::cout << std::endl;
            }
        }

        // Sort and dedup
        std::sort(devices.begin(), devices.end());
        devices.erase(std::unique(devices.begin(), devices.end()), devices.end());

    } catch (const std::exception& e) {
        std::cerr << "Error getting video devices: " << e.what() << std::endl;
    }

    // Fallback: enumerate /dev/video*
    if (devices.empty()) {
        for (int i = 0; i < 20; i++) {
            std::string dev = "/dev/video" + std::to_string(i);
            if (std::filesystem::exists(dev)) {
                devices.push_back(dev);
            }
        }
    }

    return devices;
}

bool CameraCapture::tryResetDevice(const std::string& dev_path) {
    try {
        // Get udev info
        std::string cmd = "udevadm info -q path -n " + dev_path + " 2>/dev/null";
        FILE* pipe = popen(cmd.c_str(), "r");
        if (pipe) {
            char buffer[256];
            if (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
                std::string udev_info(buffer);
                udev_info.erase(std::remove(udev_info.begin(), udev_info.end(), '\n'), udev_info.end());
                
                if (!udev_info.empty()) {
                    std::string usb_path = "/sys" + udev_info + "/../reset";
                    if (std::filesystem::exists(usb_path)) {
                        std::ofstream reset_file(usb_path);
                        if (reset_file.is_open()) {
                            reset_file << "1";
                            reset_file.close();
                            std::this_thread::sleep_for(std::chrono::seconds(2));
                            pclose(pipe);
                            return true;
                        }
                    }
                }
            }
            pclose(pipe);
        }
    } catch (...) {
        // Ignore
    }
    return false;
}

bool CameraCapture::initCamera(const std::string& dev_path, int cam_id) {
    for (int attempt = 0; attempt < 3; attempt++) {
        try {
            if (!std::filesystem::exists(dev_path)) {
                std::cout << "Device " << dev_path << " does not exist" << std::endl;
                continue;
            }

            if (attempt > 0) {
                tryResetDevice(dev_path);
                std::string cmd = "sudo chmod 666 " + dev_path + " 2>/dev/null";
                system(cmd.c_str());
                cmd = "sudo fuser -k " + dev_path + " 2>/dev/null";
                system(cmd.c_str());
            }

            cv::VideoCapture cap(dev_path, cv::CAP_V4L2);
            if (!cap.isOpened()) {
                std::cout << "OpenCV cannot open device" << std::endl;
                continue;
            }

            // MJPG format
            cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
            cap.set(cv::CAP_PROP_FPS, static_cast<double>(target_fps_));

            // Try requested resolution
            bool success = false;
            int actual_width = 0;
            int actual_height = 0;

            for (const auto& res : resolutions_) {
                cap.set(cv::CAP_PROP_FRAME_WIDTH, res.first);
                cap.set(cv::CAP_PROP_FRAME_HEIGHT, res.second);
                actual_width = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH));
                actual_height = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT));

                if (actual_width == res.first && actual_height == res.second) {
                    std::cout << "Camera " << cam_id << " set to " 
                              << actual_width << "x" << actual_height << std::endl;
                    success = true;
                    break;
                } else {
                    std::cout << "Camera " << cam_id << " could not set " 
                              << res.first << "x" << res.second 
                              << ", got " << actual_width << "x" << actual_height << std::endl;
                }
            }

            if (!success) {
                std::cout << "Camera " << cam_id << " could not set any requested resolution, using default " 
                          << actual_width << "x" << actual_height << std::endl;
            }

            // Pre-read a few frames
            for (int i = 0; i < 5; i++) {
                cap.grab();
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }

            // Get actual resolution again
            actual_width = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH));
            actual_height = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT));

            std::string window_name = "Camera_" + std::to_string(cam_id) + "_" 
                                      + std::to_string(actual_width) + "x" + std::to_string(actual_height);

            CameraInfo info;
            info.id = cam_id;
            info.cap = std::move(cap);
            info.dev = dev_path;
            info.frame_count = 0;
            info.width = actual_width;
            info.height = actual_height;
            info.window_name = window_name;

            cameras_.push_back(std::move(info));
            return true;

        } catch (const std::exception& e) {
            std::cout << "Attempt #" << (attempt + 1) << " init " << dev_path << " failed: " << e.what() << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }
    return false;
}

bool CameraCapture::initMainOrSecondCamera(const std::string& dev_main, const std::string& dev_sec, int cam_id) {
    if (!dev_main.empty() && std::filesystem::exists(dev_main)) {
        if (initCamera(dev_main, cam_id)) {
            std::cout << "Camera initialized: " << dev_main << " as camera_" << cam_id << std::endl;
            return true;
        }
    } else if (!dev_main.empty()) {
        std::cout << "Device " << dev_main << " not found, trying alternate" << std::endl;
    }

    if (!dev_sec.empty() && std::filesystem::exists(dev_sec)) {
        if (initCamera(dev_sec, cam_id)) {
            std::cout << "Camera initialized: " << dev_sec << " as camera_" << cam_id << std::endl;
            return true;
        }
    }

    std::cout << "Cannot init camera " << cam_id << " (main: " << dev_main << ", sec: " << dev_sec << ")" << std::endl;
    return false;
}

std::map<int, std::pair<std::string, std::string>> CameraCapture::findConfiguredCameraDevices() {
    std::map<int, std::pair<std::string, std::string>> camera_configs;

    // Find all matching symlinks
    std::vector<std::string> main_devices;
    std::vector<std::string> sec_devices;

    try {
        for (const auto& entry : std::filesystem::directory_iterator("/dev")) {
            std::string name = entry.path().filename().string();
            if (name.find("_video_") != std::string::npos && name.find("_main") != std::string::npos) {
                main_devices.push_back(entry.path().string());
            }
            if (name.find("_video_") != std::string::npos && name.find("_sec") != std::string::npos) {
                sec_devices.push_back(entry.path().string());
            }
        }
    } catch (...) {
        // Ignore error
    }

    // Process main devices
    std::regex main_regex("video_(\\d+)_main");
    for (const auto& dev : main_devices) {
        std::smatch match;
        std::string filename = std::filesystem::path(dev).filename().string();
        if (std::regex_search(filename, match, main_regex)) {
            int cam_num = std::stoi(match[1].str());
            if (camera_configs.find(cam_num) == camera_configs.end()) {
                camera_configs[cam_num] = {dev, ""};
            } else {
                camera_configs[cam_num].first = dev;
            }
        }
    }

    // Process sec devices
    std::regex sec_regex("video_(\\d+)_sec");
    for (const auto& dev : sec_devices) {
        std::smatch match;
        std::string filename = std::filesystem::path(dev).filename().string();
        if (std::regex_search(filename, match, sec_regex)) {
            int cam_num = std::stoi(match[1].str());
            if (camera_configs.find(cam_num) == camera_configs.end()) {
                camera_configs[cam_num] = {"", dev};
            } else {
                camera_configs[cam_num].second = dev;
            }
        }
    }

    return camera_configs;
}

void CameraCapture::initCameras() {
    // Use specified video devices if given
    if (!video_devices_.empty()) {
        std::vector<std::pair<std::string, std::string>> video_configs;
        
        if (video_devices_.size() >= 6) {
            // 6 devices: main+sec per camera
            video_configs = {
                {video_devices_[0], video_devices_[1]},  // cam0: main, sec
                {video_devices_[2], video_devices_[3]},  // cam1: main, sec
                {video_devices_[4], video_devices_[5]}   // cam2: main, sec
            };
        } else if (video_devices_.size() == 3) {
            // 3 devices: main only
            video_configs = {
                {video_devices_[0], ""},
                {video_devices_[1], ""},
                {video_devices_[2], ""}
            };
        } else {
            std::cout << "video_devices count wrong (" << video_devices_.size() << "), expect 3 or 6" << std::endl;
        }

        if (!video_configs.empty()) {
            // Use specified config (cam1/cam2 swapped)
            std::vector<std::pair<std::string, std::string>> video_mapping = {
                video_configs[0],  // cam0: video_0
                video_configs[2],  // cam1: video_2 (swap)
                video_configs[1]   // cam2: video_1 (swap)
            };

            for (size_t cam_id = 0; cam_id < video_mapping.size(); cam_id++) {
                initMainOrSecondCamera(video_mapping[cam_id].first, video_mapping[cam_id].second, cam_id);
            }
        }
    } else {
        // Find configured camera devices (symlinks)
        auto camera_configs = findConfiguredCameraDevices();

        if (camera_configs.empty()) {
            std::cout << "\n" << std::string(60, '=') << std::endl;
            std::cout << "Error: No configured camera devices found" << std::endl;
            std::cout << std::string(60, '=') << std::endl;
            std::cout << "\nSee README.md to configure cameras." << std::endl;
            std::cout << "Create udev rules with SYMLINK+=\"left_video_0_main\", etc." << std::endl;
            std::cout << "Copy to /etc/udev/rules.d/ and run udevadm control --reload-rules; udevadm trigger" << std::endl;
            std::cout << std::string(60, '=') << "\n" << std::endl;
            return;
        }

        // Need at least 3 cameras
        std::vector<int> required_cams = {0, 1, 2};
        std::vector<int> missing_cams;
        for (int cam_num : required_cams) {
            if (camera_configs.find(cam_num) == camera_configs.end() || 
                camera_configs[cam_num].first.empty()) {
                missing_cams.push_back(cam_num);
            }
        }

        if (!missing_cams.empty()) {
            std::cout << "\nError: Missing configured cameras: ";
            for (int cam : missing_cams) {
                std::cout << cam << " ";
            }
            std::cout << std::endl;
            std::cout << "Configure video_0, video_1, video_2 symlinks. See README.md" << std::endl;
            return;
        }

        // Use configured devices (cam1/cam2 swapped)
        std::vector<std::pair<std::string, std::string>> video_mapping = {
            camera_configs[0],  // cam0: video_0
            camera_configs[2],  // cam1: video_2 (swap)
            camera_configs[1]   // cam2: video_1 (swap)
        };

        std::cout << "Configured camera devices:" << std::endl;
        for (size_t cam_id = 0; cam_id < video_mapping.size(); cam_id++) {
            std::cout << "  Camera_" << cam_id << ": main=" << video_mapping[cam_id].first 
                      << ", sec=" << video_mapping[cam_id].second << std::endl;
        }

        for (size_t cam_id = 0; cam_id < video_mapping.size(); cam_id++) {
            initMainOrSecondCamera(video_mapping[cam_id].first, video_mapping[cam_id].second, cam_id);
        }
    }

    if (cameras_.empty()) {
        std::cout << "\nError: No cameras available" << std::endl;
        std::cout << "Diagnose: ls /dev/video*; v4l2-ctl --list-devices; ls -l /dev/*_video_*" << std::endl;
        std::cout << "Fix permission: sudo chmod 666 /dev/*_video_*" << std::endl;
        std::cout << "See README.md to configure cameras" << std::endl;
        return;
    }

    std::cout << "\nInitialized " << cameras_.size() << " camera(s)" << std::endl;
}

void CameraCapture::syncGrabLoop() {
    while (running_) {
        // Grab all cameras as close together as possible
        std::map<int, bool> grab_results;
        for (auto& cam : cameras_) {
            grab_results[cam.id] = cam.cap.grab();
        }

        auto now = std::chrono::steady_clock::now();
        double now_sec = std::chrono::duration<double>(now.time_since_epoch()).count();
        auto ts_ns = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count());

        for (auto& cam : cameras_) {
            if (!grab_results[cam.id]) continue;

            cv::Mat frame;
            bool ret = cam.cap.retrieve(frame);
            if (!ret || frame.empty()) continue;

            if (frame_callback_) {
                try {
                    frame_callback_(cam.id, frame, ts_ns);
                } catch (const std::exception& e) {
                    std::cerr << "Frame callback error: " << e.what() << std::endl;
                }
            }
            cam.frame_count++;

            {
                std::lock_guard<std::mutex> lk(*cam.lock);
                cam.latest_frame = frame;
                cam.latest_ts_ns = ts_ns;
            }

            cam.cap_fps_ts.push_back(now_sec);
            if (cam.cap_fps_ts.size() > 30) cam.cap_fps_ts.pop_front();
            if (cam.cap_fps_ts.size() >= 2) {
                double dt = cam.cap_fps_ts.back() - cam.cap_fps_ts.front();
                if (dt > 0) {
                    cam.cap_fps_val = (static_cast<double>(cam.cap_fps_ts.size()) - 1.0) / dt;
                }
            }
        }
    }
}

void CameraCapture::startGrabThread() {
    grab_thread_ = std::make_unique<std::thread>(&CameraCapture::syncGrabLoop, this);
}

void CameraCapture::stopGrabThread() {
    running_ = false;
    if (grab_thread_ && grab_thread_->joinable()) {
        grab_thread_->join();
    }
    grab_thread_.reset();
}

std::pair<cv::Mat, uint64_t> CameraCapture::getLatest(CameraInfo& cam) {
    std::lock_guard<std::mutex> lk(*cam.lock);
    cv::Mat frame = cam.latest_frame;
    uint64_t ts_ns = cam.latest_ts_ns;
    cam.latest_frame = cv::Mat();
    return {frame, ts_ns};
}

void CameraCapture::displayFrames(const std::vector<std::pair<CameraInfo*, cv::Mat>>& frames_data) {
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
        running_ = false;
    }
}

void CameraCapture::captureFrames() {
    std::cout << "\nCapturing " << cameras_.size() << " camera(s)..." << std::endl;
    std::cout << "Press ESC or Ctrl+C to stop" << std::endl;

    if (show_preview_) {
        for (auto& cam : cameras_) {
            cv::namedWindow(cam.window_name, cv::WINDOW_NORMAL);
            cv::resizeWindow(cam.window_name, 640, 480);
        }
    }

    startGrabThread();

    double frame_interval = 1.0 / static_cast<double>(target_fps_ > 0 ? target_fps_ : 30);

    try {
        while (running_) {
            auto start_time = std::chrono::steady_clock::now();
            std::vector<std::pair<CameraInfo*, cv::Mat>> frames_data;

            for (auto& cam : cameras_) {
                auto [frame, ts_ns] = getLatest(cam);
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

            if (show_preview_) {
                displayFrames(frames_data);
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

    releaseResources();
}

void CameraCapture::releaseResources() {
    stopGrabThread();
    for (auto& cam : cameras_) {
        try {
            cam.cap.release();
        } catch (...) {}
    }
    if (show_preview_) {
        for (auto& cam : cameras_) {
            try {
                cv::destroyWindow(cam.window_name);
            } catch (...) {}
        }
    }
}

void CameraCapture::stop() {
    running_ = false;
    stopGrabThread();
}

} // namespace das
