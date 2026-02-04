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
};

/**
 * @brief Camera capture class
 */
class CameraCapture {
public:
    using FrameCallback = std::function<void(int camera_id, const cv::Mat& frame, uint64_t timestamp)>;

    /**
     * @brief Constructor
     * @param serial_port USB serial path (for filtering video devices)
     * @param camera_count Number of cameras
     * @param resolutions Resolution list
     * @param show_preview Show preview window
     * @param video_devices Video device paths
     * @param frame_callback Frame callback
     */
    CameraCapture(const std::string& serial_port = "",
                  int camera_count = 3,
                  const std::vector<std::pair<int, int>>& resolutions = {{1600, 1296}},
                  bool show_preview = true,
                  const std::vector<std::string>& video_devices = {},
                  FrameCallback frame_callback = nullptr);

    ~CameraCapture();

    /**
     * @brief Start frame capture
     */
    void captureFrames();

    /**
     * @brief Stop capture
     */
    void stop();

    /**
     * @brief Check if running
     */
    bool isRunning() const { return running_; }

    /**
     * @brief Get camera list
     */
    std::vector<CameraInfo>& getCameras() { return cameras_; }

    /**
     * @brief Set frame callback
     */
    void setFrameCallback(FrameCallback callback) { frame_callback_ = callback; }

    // Public for external callbacks
    std::atomic<bool> running_;
    bool show_preview_;
    FrameCallback frame_callback_;
    std::vector<CameraInfo> cameras_;

private:
    std::vector<std::string> getPhysicalDevices();
    bool tryResetDevice(const std::string& dev_path);
    bool initCamera(const std::string& dev_path, int cam_id);
    bool initMainOrSecondCamera(const std::string& dev_main, const std::string& dev_sec, int cam_id);
    std::map<int, std::pair<std::string, std::string>> findConfiguredCameraDevices();
    void initCameras();
    void displayFrames(const std::vector<std::pair<CameraInfo*, cv::Mat>>& frames_data);
    void releaseResources();

    std::string serial_port_;
    int camera_count_;
    std::vector<std::pair<int, int>> resolutions_;
    std::vector<std::string> video_devices_;
};

} // namespace das

#endif // CAMERA_HPP
