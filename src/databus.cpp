/**
 * @file databus.cpp
 * @brief Serial communication module implementation
 */

#include "databus.hpp"
#include <iostream>
#include <iomanip>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <dirent.h>
#include <cstring>
#include <chrono>
#include <filesystem>

namespace das {

DataBus::DataBus(const std::string& tty_port,
                 int baudrate,
                 double timeout,
                 bool is_calib_cmd,
                 double encoder_freq,
                 double tactile_freq,
                 TactileCallback tactile_callback,
                 EncoderCallback encoder_callback,
                 CameraCalibCallback camera_calib_callback,
                 const std::string& yaml_filename,
                 const std::string& output_dir,
                 bool quiet_console,
                 bool mcuid_ascii_only)
    : tty_port_(tty_port)
    , baudrate_(baudrate)
    , timeout_(timeout)
    , serial_fd_(-1)
    , is_running_(false)
    , open_serial_success_(false)
    , encoder_freq_(encoder_freq)
    , tactile_freq_(tactile_freq)
    , gripper_dis_(0.0f)
    , is_calib_cmd_(is_calib_cmd)
    , tactile_callback_(tactile_callback)
    , encoder_callback_(encoder_callback)
    , camera_calib_callback_(camera_calib_callback)
    , yaml_filename_(yaml_filename)
    , output_dir_(output_dir)
    , quiet_console_(quiet_console)
    , mcuid_ascii_only_(mcuid_ascii_only)
{
    openSerial();
    if (!open_serial_success_) {
        throw std::runtime_error("Cannot open serial port: " + tty_port);
    }

    is_running_ = true;
    startReading();
    startParsing();
    startSending();

    if (encoder_freq_ > 0) {
        startEncoderLoop();
    }
    if (tactile_freq_ > 0) {
        startTactileLoop();
    }
}

DataBus::~DataBus() {
    stop();
}

void DataBus::openSerial() {
    try {
        serial_fd_ = open(tty_port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (serial_fd_ < 0) {
            std::cerr << "Failed to open serial port: " << tty_port_ << " - " << strerror(errno) << std::endl;
            open_serial_success_ = false;
            return;
        }

        // Configure serial port
        struct termios tty;
        memset(&tty, 0, sizeof(tty));

        if (tcgetattr(serial_fd_, &tty) != 0) {
            std::cerr << "Failed to get serial attributes: " << strerror(errno) << std::endl;
            close(serial_fd_);
            open_serial_success_ = false;
            return;
        }

        // Set baud rate
        speed_t baud;
        switch (baudrate_) {
            case 9600: baud = B9600; break;
            case 19200: baud = B19200; break;
            case 38400: baud = B38400; break;
            case 57600: baud = B57600; break;
            case 115200: baud = B115200; break;
            case 230400: baud = B230400; break;
            case 460800: baud = B460800; break;
            case 921600: baud = B921600; break;
            default:
                std::cerr << "Unsupported baud rate: " << baudrate_ << std::endl;
                baud = B921600;
        }

        cfsetispeed(&tty, baud);
        cfsetospeed(&tty, baud);

        // 8N1
        tty.c_cflag &= ~PARENB;        // No parity
        tty.c_cflag &= ~CSTOPB;        // 1 stop bit
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;            // 8 data bits
        tty.c_cflag &= ~CRTSCTS;       // No hardware flow control
        tty.c_cflag |= CREAD | CLOCAL; // Enable receiver, ignore modem lines

        // Raw input
        tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

        // Raw output
        tty.c_oflag &= ~OPOST;

        // Timeout
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = static_cast<cc_t>(timeout_ * 10);

        if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
            std::cerr << "Failed to set serial attributes: " << strerror(errno) << std::endl;
            close(serial_fd_);
            open_serial_success_ = false;
            return;
        }

        // Flush buffers
        tcflush(serial_fd_, TCIOFLUSH);

        if (!quiet_console_) {
            std::cout << "Serial port opened: " << tty_port_ << ", baud rate: " << baudrate_ << std::endl;
        }
        open_serial_success_ = true;

    } catch (const std::exception& e) {
        std::cerr << "Error opening serial port: " << e.what() << std::endl;
        open_serial_success_ = false;
    }
}

void DataBus::setTargetDistance(float distance) {
    if (distance < 0.0f || distance > 0.103f) {
        throw std::invalid_argument("Distance must be in [0.0, 0.103], got: " + std::to_string(distance));
    }
    
    std::lock_guard<std::mutex> lock(angle_lock_);
    gripper_dis_ = distance;
    std::cout << "Set target distance: " << distance << " m" << std::endl;
}

float DataBus::getTargetDistance() {
    std::lock_guard<std::mutex> lock(angle_lock_);
    return gripper_dis_;
}

void DataBus::driveMotor(float angle_degree) {
    addCmd(CmdPack::pack(Opcode::WriteDrive, RecordType::Drive, floatToBigEndianBytes(angle_degree)));
}

void DataBus::disableMotor() {
    addCmd(CmdPack::pack(Opcode::DisableDrive, RecordType::Drive));
}

void DataBus::calibEncoder() {
    addCmd(CmdPack::pack(Opcode::CalibEncoder, RecordType::Drive));
}

bool DataBus::sendCameraCalibCmd(const std::string& camera_cmd) {
    try {
        is_calib_cmd_ = true;
        std::vector<uint8_t> record(camera_cmd.begin(), camera_cmd.end());
        CmdPack cmd = CmdPack::packCalib(record);
        bool success = addCmd(cmd);
        if (success) {
            std::cout << "Send camera calibration command: " << camera_cmd << std::endl;
        } else {
            std::cout << "Failed to send camera calibration command: " << camera_cmd << std::endl;
        }
        return success;
    } catch (const std::exception& e) {
        std::cerr << "Error sending camera calibration command: " << e.what() << std::endl;
        return false;
    }
}

bool DataBus::addCmd(const CmdPack& cmd) {
    try {
        std::lock_guard<std::mutex> lock(cmd_queue_lock_);
        if (cmd_queue_.size() >= 1000) {
            std::cerr << "Command queue full, add failed" << std::endl;
            return false;
        }
        cmd_queue_.push(cmd);
        cmd_queue_cv_.notify_one();
        return true;
    } catch (...) {
        return false;
    }
}

void DataBus::registerTactileCallback(TactileCallback callback) {
    tactile_callback_ = callback;
}

void DataBus::registerEncoderCallback(EncoderCallback callback) {
    encoder_callback_ = callback;
}

void DataBus::registerCameraCalibCallback(CameraCalibCallback callback) {
    camera_calib_callback_ = callback;
}

void DataBus::startReading() {
    read_thread_ = std::make_unique<std::thread>(&DataBus::readingLoop, this);
    if (!quiet_console_) {
        std::cout << "Read thread started" << std::endl;
    }
}

void DataBus::startParsing() {
    parse_thread_ = std::make_unique<std::thread>(&DataBus::parsingLoop, this);
    if (!quiet_console_) {
        std::cout << "Parse thread started" << std::endl;
    }
}

void DataBus::startSending() {
    send_thread_ = std::make_unique<std::thread>(&DataBus::sendingLoop, this);
    if (!quiet_console_) {
        std::cout << "Send thread started" << std::endl;
    }
}

void DataBus::startEncoderLoop() {
    encoder_thread_ = std::make_unique<std::thread>(&DataBus::encoderLoop, this);
    if (!quiet_console_) {
        std::cout << "Encoder loop thread started" << std::endl;
    }
}

void DataBus::startTactileLoop() {
    tactile_thread_ = std::make_unique<std::thread>(&DataBus::tactileLoop, this);
    if (!quiet_console_) {
        std::cout << "Tactile loop thread started" << std::endl;
    }
}

void DataBus::readingLoop() {
    uint8_t buffer[16384];
    
    while (is_running_) {
        try {
            std::lock_guard<std::mutex> lock(serial_lock_);
            if (serial_fd_ >= 0) {
                int bytes_available = 0;
                ioctl(serial_fd_, FIONREAD, &bytes_available);
                
                if (bytes_available > 0) {
                    int read_size = std::min(bytes_available, 16384);
                    ssize_t n = read(serial_fd_, buffer, read_size);
                    if (n > 0) {
                        std::lock_guard<std::mutex> data_lock(data_buffer_lock_);
                        data_buffer_.insert(data_buffer_.end(), buffer, buffer + n);
                    }
                }
            }
        } catch (const std::exception& e) {
            std::cerr << "Read loop error: " << e.what() << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    if (!quiet_console_) {
        std::cout << "Read thread exited" << std::endl;
    }
}

void DataBus::parsingLoop() {
    while (is_running_) {
        std::vector<std::vector<uint8_t>> packets_to_process;

        // Get packet list
        {
            std::lock_guard<std::mutex> lock(data_buffer_lock_);
            if (!data_buffer_.empty()) {
                auto [packets, remain] = DASProtocol::findPacket(data_buffer_);
                data_buffer_ = remain;
                packets_to_process = packets;
            }
        }

        // Process packets
        for (const auto& packet : packets_to_process) {
            try {
                if (is_calib_cmd_) {
                    if (mcuid_ascii_only_) {
                        auto mid = MessagePack::extractDasFramedPayload(packet);
                        if (mid && !mid->empty()) {
                            std::cout << "MCUID: " << *mid << std::endl;
                        }
                        is_calib_cmd_ = false;
                    } else {
                        if (!quiet_console_) {
                            // Print packet (Python bytes-style)
                            std::cout << "packet: b'";
                            for (auto b : packet) {
                                if (b == '\r') {
                                    std::cout << "\\r";
                                } else if (b == '\n') {
                                    std::cout << "\\n";
                                } else if (b == '\\') {
                                    std::cout << "\\\\";
                                } else if (b == '\'') {
                                    std::cout << "\\'";
                                } else if (b >= 32 && b <= 126) {
                                    // Printable ASCII
                                    std::cout << static_cast<char>(b);
                                } else {
                                    // Non-printable as \xHH
                                    std::cout << "\\x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(b);
                                }
                            }
                            std::cout << "'" << std::dec << std::endl;
                        }

                        bool success = MessagePack::unpackCameraCalib(packet, yaml_filename_, output_dir_);
                        if (success) {
                            if (camera_calib_callback_) {
                                camera_calib_callback_(packet);
                            }
                            is_calib_cmd_ = false;
                        }
                    }
                } else {
                    auto pack_opt = MessagePack::unpack(packet);
                    if (!pack_opt) {
                        continue;
                    }

                    const auto& pack = *pack_opt;
                    for (const auto& record : pack.records) {
                        try {
                            if (record.record_type == RecordType::Tactile) {
                                if (tactile_callback_) {
                                    tactile_callback_(record.record_data);
                                }
                            } else if (record.record_type == RecordType::Encoder) {
                                if (encoder_callback_) {
                                    encoder_callback_(record.record_data);
                                }
                            }
                        } catch (const std::exception& e) {
                            std::cerr << "Callback error: " << e.what() << std::endl;
                        }
                    }
                }
            } catch (const std::exception& e) {
                std::cerr << "Packet processing error: " << e.what() << std::endl;
            }
        }

        if (packets_to_process.empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
    if (!quiet_console_) {
        std::cout << "Parse thread exited" << std::endl;
    }
}

void DataBus::sendingLoop() {
    while (is_running_) {
        CmdPack cmd;
        bool has_cmd = false;

        {
            std::unique_lock<std::mutex> lock(cmd_queue_lock_);
            if (cmd_queue_cv_.wait_for(lock, std::chrono::milliseconds(100), 
                                        [this] { return !cmd_queue_.empty() || !is_running_; })) {
                if (!cmd_queue_.empty()) {
                    cmd = cmd_queue_.front();
                    cmd_queue_.pop();
                    has_cmd = true;
                }
            }
        }

        if (has_cmd) {
            try {
                std::lock_guard<std::mutex> lock(serial_lock_);
                if (serial_fd_ >= 0) {
                    ssize_t written = write(serial_fd_, cmd.data.data(), cmd.data.size());
                    if (written < 0) {
                        std::cerr << "Failed to write to serial: " << strerror(errno) << std::endl;
                    }
                    tcdrain(serial_fd_);
                }
            } catch (const std::exception& e) {
                std::cerr << "Send error: " << e.what() << std::endl;
            }
        }
    }
    if (!quiet_console_) {
        std::cout << "Send thread exited" << std::endl;
    }
}

void DataBus::encoderLoop() {
    if (encoder_freq_ <= 0) return;

    double interval = 1.0 / encoder_freq_;
    if (!quiet_console_) {
        std::cout << "Encoder loop started, freq: " << encoder_freq_ << "Hz, interval: " << interval << "s" << std::endl;
    }

    while (is_running_) {
        auto start_time = std::chrono::steady_clock::now();

        float dis_target;
        {
            std::lock_guard<std::mutex> lock(angle_lock_);
            dis_target = gripper_dis_;
        }

        addCmd(CmdPack::pack(Opcode::ReadBatch, RecordType::Encoder, floatToBigEndianBytes(dis_target)));

        auto elapsed = std::chrono::steady_clock::now() - start_time;
        auto sleep_time = std::chrono::duration<double>(interval) - elapsed;
        if (sleep_time.count() > 0) {
            std::this_thread::sleep_for(sleep_time);
        }
    }
    if (!quiet_console_) {
        std::cout << "Encoder loop thread exited" << std::endl;
    }
}

void DataBus::tactileLoop() {
    if (tactile_freq_ <= 0) return;

    double interval = 1.0 / tactile_freq_;
    if (!quiet_console_) {
        std::cout << "Tactile loop started, freq: " << tactile_freq_ << "Hz, interval: " << interval << "s" << std::endl;
    }

    while (is_running_) {
        auto start_time = std::chrono::steady_clock::now();

        addCmd(CmdPack::pack(Opcode::ReadSingle, RecordType::Tactile, floatToBigEndianBytes(0.0f)));

        auto elapsed = std::chrono::steady_clock::now() - start_time;
        auto sleep_time = std::chrono::duration<double>(interval) - elapsed;
        if (sleep_time.count() > 0) {
            std::this_thread::sleep_for(sleep_time);
        }
    }
    if (!quiet_console_) {
        std::cout << "Tactile loop thread exited" << std::endl;
    }
}

void DataBus::stop() {
    if (!quiet_console_) {
        std::cout << "Stopping all threads..." << std::endl;
    }
    is_running_ = false;

    cmd_queue_cv_.notify_all();

    if (read_thread_ && read_thread_->joinable()) {
        read_thread_->join();
    }
    if (send_thread_ && send_thread_->joinable()) {
        send_thread_->join();
    }
    if (parse_thread_ && parse_thread_->joinable()) {
        parse_thread_->join();
    }
    if (encoder_thread_ && encoder_thread_->joinable()) {
        encoder_thread_->join();
    }
    if (tactile_thread_ && tactile_thread_->joinable()) {
        tactile_thread_->join();
    }

    if (serial_fd_ >= 0) {
        close(serial_fd_);
        serial_fd_ = -1;
        if (!quiet_console_) {
            std::cout << "Serial port closed" << std::endl;
        }
    }

    if (!quiet_console_) {
        std::cout << "All threads stopped" << std::endl;
    }
}

// Helper functions
bool checkAndFixPermission(const std::string& port, bool verbose) {
    if (!std::filesystem::exists(port)) {
        return false;
    }

    // Check if current user has rw permission
    if (access(port.c_str(), R_OK | W_OK) == 0) {
        return true;
    }

    if (verbose) {
        std::cout << "Attempting to fix permission for " << port << "..." << std::endl;
    }
    try {
        std::string cmd = "sudo chmod 666 " + port;
        int result = system(cmd.c_str());
        if (result == 0) {
            if (verbose) {
                std::cout << "Permission fixed: " << port << std::endl;
            }
            return true;
        }
    } catch (...) {
        // Ignore error
    }
    if (verbose) {
        std::cout << "Permission fix failed, run manually: sudo chmod 666 " << port << std::endl;
    }
    return false;
}

std::string findConfiguredSerialPort(bool verbose) {
    // Find all /dev/ttyDevice* devices
    std::vector<std::string> configured_ports;
    
    DIR* dir = opendir("/dev");
    if (dir) {
        struct dirent* entry;
        while ((entry = readdir(dir)) != nullptr) {
            std::string name = entry->d_name;
            if (name.find("ttyDevice") == 0) {
                configured_ports.push_back("/dev/" + name);
            }
        }
        closedir(dir);
    }

    if (configured_ports.empty()) {
        return "";
    }

    // Sort devices
    std::sort(configured_ports.begin(), configured_ports.end());

    // Return first device with permission
    for (const auto& port : configured_ports) {
        if (std::filesystem::exists(port) && checkAndFixPermission(port, verbose)) {
            return port;
        }
    }

    // If all have permission issues, return first
    return configured_ports.empty() ? "" : configured_ports[0];
}

std::string findSerialPort(const std::string& pattern, bool verbose) {
    (void)pattern;
    // Only look for configured symlink devices
    std::string configured_port = findConfiguredSerialPort(verbose);

    if (!configured_port.empty()) {
        if (verbose) {
            std::cout << "Using configured serial port: " << configured_port << std::endl;
        }
        return configured_port;
    }

    // No configured device found
    if (verbose) {
        std::cout << "\n" << std::string(60, '=') << std::endl;
        std::cout << "Error: No configured USB serial port found" << std::endl;
        std::cout << std::string(60, '=') << std::endl;
        std::cout << "\nTo configure USB device:" << std::endl;
        std::cout << "1. See config guide in 配置方法/README_CN.md" << std::endl;
        std::cout << "2. Create udev rules file (e.g. 99-usb-serial.rules)" << std::endl;
        std::cout << "3. Copy rules to /etc/udev/rules.d/" << std::endl;
        std::cout << "4. Reload udev:" << std::endl;
        std::cout << "   sudo udevadm control --reload-rules" << std::endl;
        std::cout << "   sudo udevadm trigger" << std::endl;
        std::cout << "\nAfter setup, ttyDevice* symlinks should appear under /dev/" << std::endl;
        std::cout << "e.g. /dev/ttyDeviceLeft or your custom name" << std::endl;
        std::cout << std::string(60, '=') << "\n" << std::endl;
    } else {
        std::cerr << "No configured serial port found" << std::endl;
    }
    return "";
}

std::string findGripperSerialBySide(const std::string& side, bool verbose) {
    if (side != "left" && side != "right") {
        if (verbose) {
            std::cerr << "Error: side must be \"left\" or \"right\", got: " << side << std::endl;
        }
        return "";
    }

    const std::string dev = (side == "right") ? "/dev/ttyDeviceRight" : "/dev/ttyDeviceLeft";

    if (!std::filesystem::exists(dev)) {
        if (verbose) {
            std::cerr << "Serial device not found: " << dev << std::endl;
            std::cerr << "Check udev rules (e.g. ttyDeviceLeft / ttyDeviceRight under /dev/)" << std::endl;
        } else {
            std::cerr << "Serial device not found: " << dev << std::endl;
        }
        return "";
    }

    if (!checkAndFixPermission(dev, verbose)) {
        if (verbose) {
            std::cerr << "No read/write access to " << dev << std::endl;
        } else {
            std::cerr << "No access: " << dev << std::endl;
        }
        return "";
    }

    return dev;
}

} // namespace das
