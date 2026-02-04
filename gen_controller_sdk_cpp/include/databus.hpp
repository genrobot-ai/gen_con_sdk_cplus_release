/**
 * @file databus.hpp
 * @brief Serial communication module
 *
 * Serial communication with gripper device.
 */

#ifndef DATABUS_HPP
#define DATABUS_HPP

#include <string>
#include <vector>
#include <queue>
#include <mutex>
#include <thread>
#include <atomic>
#include <functional>
#include <optional>
#include <condition_variable>
#include "pack.hpp"
#include "das_protocol.hpp"

namespace das {

/**
 * @brief Serial communication class
 */
class DataBus {
public:
    using TactileCallback = std::function<void(const std::vector<uint8_t>&)>;
    using EncoderCallback = std::function<void(const std::vector<uint8_t>&)>;
    using CameraCalibCallback = std::function<void(const std::vector<uint8_t>&)>;

    /**
     * @brief Constructor
     * @param tty_port Serial device path
     * @param baudrate Baud rate
     * @param timeout Timeout (s)
     * @param is_calib_cmd Calibration command mode
     * @param encoder_freq Encoder query freq (Hz), 0 = disabled
     * @param tactile_freq Tactile query freq (Hz), 0 = disabled
     * @param tactile_callback Tactile callback
     * @param encoder_callback Encoder callback
     * @param camera_calib_callback Camera calib callback
     * @param yaml_filename YAML output filename for calib
     * @param output_dir Output dir for calib
     */
    DataBus(const std::string& tty_port = "/dev/ttyUSB0",
            int baudrate = 921600,
            double timeout = 0.5,
            bool is_calib_cmd = false,
            double encoder_freq = 0,
            double tactile_freq = 0,
            TactileCallback tactile_callback = nullptr,
            EncoderCallback encoder_callback = nullptr,
            CameraCalibCallback camera_calib_callback = nullptr,
            const std::string& yaml_filename = "",
            const std::string& output_dir = "");

    ~DataBus();

    /**
     * @brief Set target distance (gripper open)
     * @param distance Target (m), range [0.0, 0.103] (max 10cm)
     */
    void setTargetDistance(float distance);

    /**
     * @brief Get current target distance
     */
    float getTargetDistance();

    /**
     * @brief Drive motor
     * @param angle_degree Angle
     */
    void driveMotor(float angle_degree);

    /**
     * @brief Disable motor
     */
    void disableMotor();

    /**
     * @brief Calibrate encoder
     */
    void calibEncoder();

    /**
     * @brief Send camera calibration command
     */
    bool sendCameraCalibCmd(const std::string& camera_cmd);

    /**
     * @brief Add command to queue
     */
    bool addCmd(const CmdPack& cmd);

    /**
     * @brief Check if serial is open
     */
    bool isOpened() const { return open_serial_success_; }

    void registerTactileCallback(TactileCallback callback);
    void registerEncoderCallback(EncoderCallback callback);
    void registerCameraCalibCallback(CameraCalibCallback callback);

    /**
     * @brief Stop all threads
     */
    void stop();

private:
    void openSerial();
    void startReading();
    void startParsing();
    void startSending();
    void startEncoderLoop();
    void startTactileLoop();

    void readingLoop();
    void parsingLoop();
    void sendingLoop();
    void encoderLoop();
    void tactileLoop();

    std::string tty_port_;
    int baudrate_;
    double timeout_;
    int serial_fd_;
    std::atomic<bool> is_running_;
    std::atomic<bool> open_serial_success_;

    std::vector<uint8_t> data_buffer_;
    std::mutex data_buffer_lock_;
    std::mutex serial_lock_;

    std::queue<CmdPack> cmd_queue_;
    std::mutex cmd_queue_lock_;
    std::condition_variable cmd_queue_cv_;

    double encoder_freq_;
    double tactile_freq_;
    float gripper_dis_;
    std::mutex angle_lock_;
    std::atomic<bool> is_calib_cmd_;

    TactileCallback tactile_callback_;
    EncoderCallback encoder_callback_;
    CameraCalibCallback camera_calib_callback_;
    std::string yaml_filename_;
    std::string output_dir_;

    std::unique_ptr<std::thread> read_thread_;
    std::unique_ptr<std::thread> parse_thread_;
    std::unique_ptr<std::thread> send_thread_;
    std::unique_ptr<std::thread> encoder_thread_;
    std::unique_ptr<std::thread> tactile_thread_;
};

/**
 * @brief Check and fix serial device permission
 */
bool checkAndFixPermission(const std::string& port);

/**
 * @brief Find configured USB serial (symlink)
 * @return Serial path or empty if not found
 */
std::string findConfiguredSerialPort();

/**
 * @brief Find USB serial device
 * @param pattern Device name pattern (deprecated, for compatibility)
 */
std::string findSerialPort(const std::string& pattern = "ttyUSB");

} // namespace das

#endif // DATABUS_HPP
