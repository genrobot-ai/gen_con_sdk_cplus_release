# Gen Gripper Controller C++ SDK

> C++ SDK for Gen Controller gripper — camera streaming, tactile sensing, encoder feedback, and distance control.

[中文](README(ZH).MD)

[GitHub repository](https://github.com/genrobot-ai/gen_con_sdk_cplus_release)

License: [MIT License](LICENSE)

## 1 Features

- C++ SDK (`cpp_sdk` / `cpp_sdk_static`) for direct integration without ROS
- Three-camera image capture with optional live preview
- Tactile sensor data via callback (left / right)
- Encoder feedback for gripper opening distance
- Gripper distance control via `DataBus::setTargetDistance()`
- `start_gripper.cpp` wrapper script — compile-on-run demo launcher
- CMake build with `camera_cmd` calibration CLI and `camera_cmd.sh` helper
- Single-gripper and dual-gripper demo workflows



## 2 Requirements


| Item     | Requirement                   |
| -------- | ----------------------------- |
| OS       | Ubuntu 20.04 (recommended)    |
| C++      | C++17                         |
| CMake    | 3.16+                         |
| USB      | USB 3.0 port                  |
| Hardware | Gen Controller gripper device |




## 3 Quick Start

> First-time users must complete [USB configuration](docs/usb-setup.md) before running the SDK.

```shell
git clone https://github.com/genrobot-ai/gen_con_sdk_cplus_release.git
cd gen_con_sdk_cplus_release
mkdir -p build && cd build
cmake ..
make
cd ..
./start_gripper.cpp left
```

Verify feedback in the console:

- Three camera preview windows (`camera_0`, `camera_1`, `camera_2`)
- Encoder output: `gripper distance: <value> m`
- Optional tactile grid with `--print-tactile-info`

Set a fixed opening distance:

```shell
./start_gripper.cpp left --distance 0.05   # 5 cm
```

Select the gripper type with `--gripper-type`. The maximum opening distance depends on the gripper:

| Gripper type | Maximum opening distance |
| --- | ---: |
| `default_gripper` | `0.103` m |
| `tactile_gripper` | `0.0998` m |
| `soft_gripper` | `0.103` m |



## 4 Program Interface

Integrate the SDK by registering callbacks when constructing `GripperSystem` and sending control commands through `DataBus`.

### 4.1 Sensor Callbacks


| Callback                  | Type                                | Description                           |
| ------------------------- | ----------------------------------- | ------------------------------------- |
| `capture_frames_callback` | `void(CameraCapture*)`              | Camera frame capture and preview loop |
| `tactile_callback`        | `void(const std::vector<uint8_t>&)` | Tactile sensor raw data (448 bytes)   |
| `encoder_callback`        | `void(const std::vector<uint8_t>&)` | Gripper opening distance feedback (m) |


Example (from `start_gripper.cpp`):

```cpp
GripperSystem system(
    serial_port,
    camera_resolutions,
    show_preview,
    video_devices,
    tactile_callback,
    encoder_callback,
    capture_frames_callback,
    camera_fps
);
```



### 4.2 Gripper Control

Send target opening distance through `DataBus`:

```cpp
if (databus_) {
    databus_->setTargetDistance(distance);
}
```

Or use the higher-level `GripperController` helper:

```cpp
GripperController controller(system.getDataBus());
controller.setFixedDistance(0.05f);
controller.startSineWave(amplitude, center, frequency, duration);
```



### 4.3 Libraries and Headers


| Target           | Description                             |
| ---------------- | --------------------------------------- |
| `cpp_sdk_static` | Static library (used by demo script)    |
| `cpp_sdk`        | Shared library                          |
| Headers          | `include/` (`gripper_system.hpp`, etc.) |




## 5 Installation



### 5.1 Install system dependencies

```shell
sudo apt update
sudo apt install build-essential cmake pkg-config libopencv-dev v4l-utils
```

`v4l-utils` provides `v4l2-ctl`, required for USB configuration.

### 5.2 Clone and build

```shell
git clone https://github.com/genrobot-ai/gen_con_sdk_cplus_release.git
cd gen_con_sdk_cplus_release
mkdir -p build && cd build
cmake ..
make
cd ..
```

Build outputs:


| Artifact               | Path                        |
| ---------------------- | --------------------------- |
| Static SDK library     | `build/libcpp_sdk_static.a` |
| Shared SDK library     | `build/libcpp_sdk.so`       |
| Camera calibration CLI | `build/camera_cmd`          |


The `start_gripper.cpp` script links against `libcpp_sdk_static.a` and recompiles on each run.

## 6 USB Configuration

Configure udev rules once per USB port before first use. The template is at [config/99-usb-serial.rules](./config/99-usb-serial.rules).

Summary:

1. Query serial and camera `KERNELS` values with `udevadm` and `v4l2-ctl`
2. Edit `config/99-usb-serial.rules`
3. Copy to `/etc/udev/rules.d/` and reload rules

For step-by-step instructions with screenshots, see:

- [USB 配置指南 (ZH)](docs/usb-setup_CN.md)
- [USB Configuration Guide (EN)](docs/usb-setup.md)

Default serial symlinks after dual-gripper setup: `/dev/ttyDeviceLeft`, `/dev/ttyDeviceRight`.

Default camera symlinks: `/dev/left_video_*`, `/dev/right_video_*` (see `start_gripper.cpp` `SIDE_CONFIG`).

## 7 Usage



### 7.1 Single Gripper Demo

```shell
cd gen_con_sdk_cplus_release
./start_gripper.cpp left                              # Default: open to 5 cm
./start_gripper.cpp left --distance 0.08              # Fixed 8 cm
./start_gripper.cpp left --gripper-type tactile_gripper
./start_gripper.cpp left --gripper-type soft_gripper
./start_gripper.cpp left --sine-wave                  # Sine wave for 10 s
./start_gripper.cpp left --print-tactile-info         # Print tactile grid
./start_gripper.cpp left --no-preview                 # Disable preview windows
./start_gripper.cpp left --camera-fps 60              # V4 Controller: set 60 for ~30 fps images
```

After startup, three image windows appear:

```
camera_0   # Center camera
camera_1   # Left-side camera
camera_2   # Right-side camera
```

Console output includes tactile data (when enabled) and gripper distance.

### 7.2 Dual Gripper Demo

Run one instance per gripper in separate terminals:

```shell
# Terminal 1 — left gripper
./start_gripper.cpp left

# Terminal 2 — right gripper
./start_gripper.cpp right
```

After startup, six image windows appear (three per gripper).

For V4 Controller units shipped after April 2026:

```shell
./start_gripper.cpp left --camera-fps 60
./start_gripper.cpp right --camera-fps 60
```



### 7.3 Device Utilities

Do **not** run `start_gripper.cpp` or other control programs while using these commands.

Build the project first (`cmake .. && make`), then use `./camera_cmd.sh`:

**Single device:**

```shell
./camera_cmd.sh camerarc   # Center camera calibration
./camera_cmd.sh camerarl   # Left camera calibration
./camera_cmd.sh camerarr   # Right camera calibration
./camera_cmd.sh MCUID      # Device ID
```

**Dual device (left / right):**

```shell
./camera_cmd.sh left camerarc
./camera_cmd.sh left camerarl
./camera_cmd.sh left camerarr
./camera_cmd.sh left MCUID

./camera_cmd.sh right camerarc
./camera_cmd.sh right camerarl
./camera_cmd.sh right camerarr
./camera_cmd.sh right MCUID
```

Calibration YAML files are saved to `calib_result/` (e.g. `cam0_sensor_left.yaml`).

## 8 Troubleshooting


| Problem                         | Solution                                                          |
| ------------------------------- | ----------------------------------------------------------------- |
| Serial port not found           | Run `sudo apt remove brltty`, then replug the device              |
| Camera or serial has wrong path | Re-check udev rules; see [docs/usb-setup.md](docs/usb-setup.md)   |
| V4 Controller frame rate halved | Run with `--camera-fps 60` (units shipped after April 2026)       |
| `camera_cmd.sh` not found       | Build first: `mkdir -p build && cd build && cmake .. && make`     |
| `start_gripper.cpp` link error  | Ensure `make` completed and `build/libcpp_sdk_static.a` exists    |
| OpenCV not found during build   | Install: `sudo apt install libopencv-dev pkg-config`              |
| No camera preview               | Check udev camera symlinks; verify with `v4l2-ctl --list-devices` |




## 9 Documentation


| Description           | Link                                                                                     |
| --------------------- | ---------------------------------------------------------------------------------------- |
| User Manual           | [DAS Controller User Manual](https://docs.genrobot.ai/en/products/das-controller)        |
| URDF                  | [URDF Model](https://zcnma1sv5kma.feishu.cn/wiki/CKpbwye45iOlrckukIPc5hKCndh)            |
| Troubleshooting Guide | [Problem Solving Guide](https://zcnma1sv5kma.feishu.cn/docx/E6VJdNqIfoGrSDxKIHKck7tLn3b) |
| USB 配置 (ZH)           | [docs/usb-setup_CN.md](docs/usb-setup_CN.md)                                             |
| USB setup (EN)        | [docs/usb-setup.md](docs/usb-setup.md)                                                   |
| udev rules template   | [config/99-usb-serial.rules](config/99-usb-serial.rules)                                 |
| Demo launcher         | [start_gripper.cpp](start_gripper.cpp)                                                   |
| Calibration helper    | [camera_cmd.sh](camera_cmd.sh)                                                           |
| SDK headers           | [include/](include/)                                                                     |


