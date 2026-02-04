/**
 * @file gripper_system.hpp
 * @brief Gripper system main class
 *
 * Starts and manages the gripper system (serial and cameras).
 */

#ifndef GRIPPER_SYSTEM_HPP
#define GRIPPER_SYSTEM_HPP

#include <string>
#include <vector>
#include <memory>
#include <atomic>
#include <functional>
#include <thread>
#include "databus.hpp"
#include "camera.hpp"

namespace das {

/**
 * @brief Sine wave controller
 */
class SineWaveController {
public:
    /**
     * @brief Constructor
     * @param databus DataBus pointer
     * @param amplitude Amplitude (m)
     * @param center Center position (m)
     * @param frequency Frequency (Hz)
     * @param duration Duration (s), 0 = infinite
     */
    SineWaveController(DataBus* databus,
                       float amplitude = 0.05f,
                       float center = 0.05f,
                       float frequency = 0.5f,
                       float duration = 1000.0f);

    ~SineWaveController();

    /**
     * @brief Start sine wave control
     */
    void start();

    /**
     * @brief Stop sine wave control
     */
    void stop();

    /**
     * @brief Check if running
     */
    bool isRunning() const { return running_; }

private:
    void controlLoop();

    DataBus* databus_;
    float amplitude_;
    float center_;
    float frequency_;
    float duration_;
    std::atomic<bool> running_;
    std::unique_ptr<std::thread> control_thread_;
    std::chrono::steady_clock::time_point start_time_;
    float control_rate_;
    float control_interval_;
};

/**
 * @brief Gripper controller
 */
class GripperController {
public:
    /**
     * @brief Constructor
     * @param databus DataBus pointer
     */
    explicit GripperController(DataBus* databus);

    ~GripperController();

    /**
     * @brief Set fixed gripper distance
     * @param distance Distance (m), range [0.0, 0.103]
     */
    void setFixedDistance(float distance);

    /**
     * @brief Start sine wave control
     */
    void startSineWave(float amplitude = 0.05f,
                       float center = 0.05f,
                       float frequency = 0.5f,
                       float duration = 60.0f);

    /**
     * @brief Stop sine wave control
     */
    void stopSineWave();

    /**
     * @brief Check if sine wave control is running
     */
    bool isSineWaveRunning() const;

private:
    DataBus* databus_;
    std::unique_ptr<SineWaveController> sine_wave_controller_;
};

/**
 * @brief Gripper system main class
 */
class GripperSystem {
public:
    using TactileCallback = std::function<void(const std::vector<uint8_t>&)>;
    using EncoderCallback = std::function<void(const std::vector<uint8_t>&)>;
    using FrameCallback = std::function<void(CameraCapture*)>;

    /**
     * @brief Constructor
     * @param serial_port Serial device path, empty = auto-detect
     * @param camera_resolutions Resolution string "widthxheight"
     * @param show_preview Show camera preview
     * @param video_devices Video device list
     * @param tactile_callback Tactile data callback
     * @param encoder_callback Encoder data callback
     * @param capture_frames_callback Frame capture callback
     */
    GripperSystem(const std::string& serial_port = "",
                  const std::string& camera_resolutions = "1600x1296",
                  bool show_preview = true,
                  const std::vector<std::string>& video_devices = {},
                  TactileCallback tactile_callback = nullptr,
                  EncoderCallback encoder_callback = nullptr,
                  FrameCallback capture_frames_callback = nullptr);

    ~GripperSystem();

    /**
     * @brief Start system
     * @return true on success
     */
    bool start();

    /**
     * @brief Stop system
     */
    void stop();

    /**
     * @brief Set gripper open distance
     * @param distance Target distance (m), range [0.0, 0.103] (max 10cm)
     */
    void setGripperDistance(float distance);

    /**
     * @brief Get DataBus pointer
     */
    DataBus* getDataBus() { return databus_.get(); }

    /**
     * @brief Get Camera pointer
     */
    CameraCapture* getCamera() { return camera_.get(); }

    /**
     * @brief Check if running
     */
    bool isRunning() const { return running_; }

private:
    void signalHandler(int signum);
    static void staticSignalHandler(int signum);
    std::vector<std::pair<int, int>> parseResolutions(const std::string& res_str);

    std::atomic<bool> running_;
    std::string serial_port_;
    std::string camera_resolutions_;
    bool show_preview_;
    std::vector<std::string> video_devices_;
    std::vector<std::pair<int, int>> resolutions_;
    
    TactileCallback tactile_callback_;
    EncoderCallback encoder_callback_;
    FrameCallback capture_frames_callback_;

    std::unique_ptr<DataBus> databus_;
    std::unique_ptr<CameraCapture> camera_;

    static GripperSystem* instance_;
};

} // namespace das

#endif // GRIPPER_SYSTEM_HPP
