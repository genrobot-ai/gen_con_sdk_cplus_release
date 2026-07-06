# Gen Controller C++ SDK

> 用于 Gen Controller 夹爪的 C++ SDK，支持相机图像、触觉传感、编码器反馈及开合距离控制。

[English](README.MD)

[GitHub代码](https://github.com/genrobot-ai/gen_con_sdk_cplus_release)

License: [MIT License](LICENSE)

## 1 功能特性

- C++ SDK（`cpp_sdk` / `cpp_sdk_static`），无需 ROS 即可直接集成
- 三相机图像采集，支持实时预览
- 触觉数据回调（左 / 右）
- 夹爪开合距离编码器反馈
- 通过 `DataBus::setTargetDistance()` 控制夹爪开合
- `start_gripper.cpp` 包装脚本 — 按需编译并运行 demo
- CMake 构建，含 `camera_cmd` 标定 CLI 与 `camera_cmd.sh` 辅助脚本
- 单爪 / 双爪 demo 工作流

## 2 环境要求


| 项目   | 要求                  |
| ---- | ------------------- |
| 系统   | Ubuntu 20.04（推荐）    |
| C++  | C++17               |
| CMake | 3.16+               |
| USB  | USB 3.0 接口          |
| 硬件   | Gen Controller 夹爪设备 |


## 3 快速开始

> 首次使用请先完成 [USB 配置](docs/usb-setup_CN.md)。

```shell
git clone https://github.com/genrobot-ai/gen_con_sdk_cplus_release.git
cd gen_con_sdk_cplus_release
mkdir -p build && cd build
cmake ..
make
cd ..
./start_gripper.cpp left
```

在控制台验证反馈：

- 三个相机预览窗口（`camera_0`、`camera_1`、`camera_2`）
- 编码器输出：`gripper distance: <value> m`
- 可选触觉网格：`--print-tactile-info`

设置固定开合距离：

```shell
./start_gripper.cpp left --distance 0.05   # 5 cm，范围 [0.0, 0.103] m
```

## 4 程序接口

通过构造 `GripperSystem` 时注册回调获取传感器数据，通过 `DataBus` 下发控制指令。

### 4.1 传感器回调

| 回调                        | 类型                                        | 说明                |
| ------------------------- | ----------------------------------------- | ----------------- |
| `capture_frames_callback` | `void(CameraCapture*)`                    | 相机帧采集与预览循环        |
| `tactile_callback`        | `void(const std::vector<uint8_t>&)`       | 触觉传感器原始数据（448 字节） |
| `encoder_callback`        | `void(const std::vector<uint8_t>&)`       | 夹爪开合距离反馈（m）       |

示例（来自 `start_gripper.cpp`）：

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

### 4.2 夹爪控制

通过 `DataBus` 下发目标开合距离：

```cpp
if (databus_) {
    databus_->setTargetDistance(distance);  // 范围 [0.0, 0.103] m
}
```

或使用更高级的 `GripperController` 辅助类：

```cpp
GripperController controller(system.getDataBus());
controller.setFixedDistance(0.05f);
controller.startSineWave(amplitude, center, frequency, duration);
```

### 4.3 库与头文件

| 目标               | 说明              |
| ---------------- | --------------- |
| `cpp_sdk_static` | 静态库（demo 脚本使用） |
| `cpp_sdk`        | 动态库             |
| 头文件              | `include/`（`gripper_system.hpp` 等） |


## 5 安装

### 5.1 安装系统依赖

```shell
sudo apt update
sudo apt install build-essential cmake pkg-config libopencv-dev v4l-utils
```

`v4l-utils` 提供 USB 配置所需的 `v4l2-ctl` 命令。

### 5.2 拉取仓库并编译

```shell
git clone https://github.com/genrobot-ai/gen_con_sdk_cplus_release.git
cd gen_con_sdk_cplus_release
mkdir -p build && cd build
cmake ..
make
cd ..
```

编译产物：

| 产物       | 路径                          |
| -------- | --------------------------- |
| 静态 SDK 库 | `build/libcpp_sdk_static.a` |
| 动态 SDK 库 | `build/libcpp_sdk.so`       |
| 相机标定 CLI | `build/camera_cmd`          |

`start_gripper.cpp` 脚本链接 `libcpp_sdk_static.a`，每次运行时会按需重新编译。

## 6 USB 配置

首次使用前需为每个 USB 口配置 udev 规则，模板见 [config/99-usb-serial.rules](./config/99-usb-serial.rules)。

简要步骤：

1. 用 `udevadm` 和 `v4l2-ctl` 查询串口与相机的 `KERNELS` 值
2. 编辑 `config/99-usb-serial.rules`
3. 复制到 `/etc/udev/rules.d/` 并 reload

详细图文步骤见：

- [USB 配置指南 (ZH)](docs/usb-setup_CN.md)
- [USB Configuration Guide (EN)](docs/usb-setup.md)

双爪配置后的默认串口软链接：`/dev/ttyDeviceLeft`、`/dev/ttyDeviceRight`。

默认相机软链接：`/dev/left_video_*`、`/dev/right_video_*`（见 `start_gripper.cpp` 中 `SIDE_CONFIG`）。

## 7 使用方法

### 7.1 单爪 Demo

```shell
cd gen_con_sdk_cplus_release
./start_gripper.cpp left                              # 默认：打开 5 cm
./start_gripper.cpp left --distance 0.08              # 固定打开 8 cm
./start_gripper.cpp left --sine-wave                  # 正弦波开合 10 s
./start_gripper.cpp left --print-tactile-info         # 打印触觉网格
./start_gripper.cpp left --no-preview                 # 关闭预览窗口
./start_gripper.cpp left --camera-fps 60              # V4 Controller：设为 60 以获得约 30 fps 图像
```

启动后弹出三个图像窗口：

```
camera_0   # 中间相机
camera_1   # 左侧相机
camera_2   # 右侧相机
```

控制台输出触觉数据（启用时）及夹爪开合距离。

### 7.2 双爪 Demo

在独立终端中分别启动左右夹爪：

```shell
# 终端 1 — 左夹爪
./start_gripper.cpp left

# 终端 2 — 右夹爪
./start_gripper.cpp right
```

启动后弹出六个图像窗口（每爪三个）。

2026 年 4 月之后发货的 V4 Controller：

```shell
./start_gripper.cpp left --camera-fps 60
./start_gripper.cpp right --camera-fps 60
```

### 7.3 设备工具命令

运行以下命令时**不要**同时启动 `start_gripper.cpp` 或其他控制程序。

需先完成编译（`cmake .. && make`），再使用 `./camera_cmd.sh`：

**单设备：**

```shell
./camera_cmd.sh camerarc   # 中间相机标定
./camera_cmd.sh camerarl   # 左侧相机标定
./camera_cmd.sh camerarr   # 右侧相机标定
./camera_cmd.sh MCUID      # 设备 ID
```

**双设备（左 / 右）：**

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

标定 YAML 文件保存至 `calib_result/`（如 `cam0_sensor_left.yaml`）。

## 8 常见问题


| 问题                | 解决方法                                                      |
| ----------------- | --------------------------------------------------------- |
| 找不到串口             | 执行 `sudo apt remove brltty`，重新插拔设备                        |
| 相机或串口路径不对         | 检查 udev 规则，见 [docs/usb-setup_CN.md](docs/usb-setup_CN.md) |
| V4 Controller 帧率减半 | 使用 `--camera-fps 60`（2026/04 之后发货的设备）                    |
| `camera_cmd.sh` 报错 | 先编译：`mkdir -p build && cd build && cmake .. && make`     |
| `start_gripper.cpp` 链接失败 | 确认 `make` 完成且存在 `build/libcpp_sdk_static.a`              |
| 编译找不到 OpenCV      | 安装：`sudo apt install libopencv-dev pkg-config`             |
| 无相机预览             | 检查 udev 相机软链接；用 `v4l2-ctl --list-devices` 验证              |


## 9 文档索引


| 说明           | 链接                                                                               |
| ------------ | -------------------------------------------------------------------------------- |
| 用户手册         | [DAS Controller 用户手册](https://docs.genrobot.ai/zh/products/das-controller)       |
| URDF         | [URDF 模型](https://zcnma1sv5kma.feishu.cn/wiki/CKpbwye45iOlrckukIPc5hKCndh)       |
| 问题解决指南       | [Gripper Controller 问题指南](https://zcnma1sv5kma.feishu.cn/docx/E6VJdNqIfoGrSDxKIHKck7tLn3b) |
| USB 配置 (ZH)  | [docs/usb-setup_CN.md](docs/usb-setup_CN.md)                                     |
| USB setup (EN) | [docs/usb-setup.md](docs/usb-setup.md)                                           |
| udev 规则模板    | [config/99-usb-serial.rules](config/99-usb-serial.rules)                         |
| Demo 启动脚本    | [start_gripper.cpp](start_gripper.cpp)                                           |
| 标定辅助脚本       | [camera_cmd.sh](camera_cmd.sh)                                                   |
| SDK 头文件      | [include/](include/)                                                             |

