/**
 * @file camera.hpp
 * @brief Camera capture module
 *
 * Multi-camera capture and preview.
 */

#ifndef CAMERA_HPP
#define CAMERA_HPP

#include <string>
#include <vector>
#include <atomic>
#include <functional>
#include <mutex>
#include <memory>
#include <thread>
#include <deque>
#include <opencv2/opencv.hpp>

namespace das {

/**
 * @brief Camera info struct
 */
struct CameraInfo {
    int id;
    cv::VideoCapture cap;
    std::string dev;
    int frame_count;
    int width;
    int height;
    std::string window_name;

    // Thread-safe latest frame cache (unique_ptr keeps CameraInfo movable)
    std::unique_ptr<std::mutex> lock = std::make_unique<std::mutex>();
    cv::Mat latest_frame;
    uint64_t latest_ts_ns = 0;

    // FPS tracking (sliding window)
    std::deque<double> cap_fps_ts;
    double cap_fps_val = 0.0;
    std::deque<double> disp_fps_ts;
    double disp_fps_val = 0.0;
};

/**
 * @brief Camera capture class
 */
class CameraCapture {
public:
    using FrameCallback = std::function<void(int camera_id, const cv::Mat& frame, uint64_t timestamp)>;

    CameraCapture(const std::string& serial_port = "",
                  int camera_count = 3,
                  const std::vector<std::pair<int, int>>& resolutions = {{1600, 1296}},
                  bool show_preview = true,
                  const std::vector<std::string>& video_devices = {},
                  FrameCallback frame_callback = nullptr,
                  int target_fps = 30);

    ~CameraCapture();

    void captureFrames();
    void stop();

    void startGrabThread();
    void stopGrabThread();
    std::pair<cv::Mat, uint64_t> getLatest(CameraInfo& cam);

    bool isRunning() const { return running_; }
    std::vector<CameraInfo>& getCameras() { return cameras_; }
    void setFrameCallback(FrameCallback callback) { frame_callback_ = callback; }

    // Public for external callbacks
    std::atomic<bool> running_;
    bool show_preview_;
    FrameCallback frame_callback_;
    std::vector<CameraInfo> cameras_;
    int target_fps_;

private:
    std::vector<std::string> getPhysicalDevices();
    bool tryResetDevice(const std::string& dev_path);
    bool initCamera(const std::string& dev_path, int cam_id);
    bool initMainOrSecondCamera(const std::string& dev_main, const std::string& dev_sec, int cam_id);
    std::map<int, std::pair<std::string, std::string>> findConfiguredCameraDevices();
    void initCameras();
    void syncGrabLoop();
    void displayFrames(const std::vector<std::pair<CameraInfo*, cv::Mat>>& frames_data);
    void releaseResources();

    std::string serial_port_;
    int camera_count_;
    std::vector<std::pair<int, int>> resolutions_;
    std::vector<std::string> video_devices_;
    std::unique_ptr<std::thread> grab_thread_;
};

} // namespace das

#endif // CAMERA_HPP
