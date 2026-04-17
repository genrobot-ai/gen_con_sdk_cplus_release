# genrobot_controller_sdk_cpp
## Environment Setup
```
USB interface must be USB 3.0
```

## USB Interface Configuration

### Single Gripper USB Port Configuration
The final configuration looks as shown in the figures. After configuration, this USB port can recognize any Gen Controller; no further configuration is needed. The template file is stored at:
```
config/99-usb-serial.rules
```
![image/image_1.png](image/image_1.png)

What you need to modify:
![image/image_2.png](image/image_2.png)

**Parameter 1** — run:

```
cd /dev && ls | grep ttyUSB
udevadm info -a -n /dev/ttyUSB* | grep -E "KERNELS|DRIVERS"
```

Set the **second** `KERNELS` value from the output to position 1:
![image/image_3.png](image/image_3.png)

**Parameter 2** — run:
```
v4l2-ctl --list-devices
```
Example output:
![image/image_4.png](image/image_4.png)

Then, for the **first** camera on that USB, run:
```
udevadm info -a -n /dev/video* | grep -E "KERNELS|SUBSYSTEMS"
```
Set the **first** `KERNELS` value from the output to position 2:
![image/image_5.png](image/image_5.png)

Then copy the template file to:
```
sudo cp config/99-usb-serial.rules /etc/udev/rules.d/
```
Reload rules:
```
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### Dual Gripper USB Port Configuration
The final configuration looks as shown in the figures.
![image/image_6.png](image/image_6.png)

Fields to modify:
![image/image_7.png](image/image_7.png)

First plug in the **left** gripper and configure it using the single-gripper steps above; then unplug the left gripper, plug in the **right** gripper, and repeat the single-gripper configuration; finally reload the rules as above.

### Multi-Gripper USB Port Configuration
Add more entries to `99-usb-serial.rules` in the same way.

## Running the SDK
Run the `start_gripper.cpp` script directly (it compiles on demand each run; edit the script and run again for changes to take effect).

### Single Gripper Demo

```
cd gen_con_sdk_cplus_release

mkdir -p build && cd build   # Remove the build folder first if it already exists

cmake ..

make

cd ..

./start_gripper.cpp left   # With current config, gripper opens to a fixed 5 cm

./start_gripper.cpp left --distance 0.08   # Gripper opens to a fixed 8 cm; valid range [0.0, 0.103], i.e. up to ~10 cm

./start_gripper.cpp left --sine-wave   # Gripper opens and closes continuously for 10 s

./start_gripper.cpp left --print-tactile-info  #Tactile Data Reading

Camera frame rate (The V4 Controller needs to change the frame rate to 60 to obtain 30fps image data. For units shipped after April 2026)
./start_gripper.cpp left --camera-fps 60

```

After startup, three image windows appear:
```
/camera_0   # Center camera
/camera_1   # Left camera
/camera_2   # Right camera

```
Console output includes:
```
Tactile data
Gripper distance data
```

### Dual Gripper Demo
```
cd gen_con_sdk_cplus_release
Terminal 1:
./start_gripper.cpp left
Terminal 2:
./start_gripper.cpp right
Camera frame rate (The V4 Controller needs to change the frame rate to 60 to obtain 30fps image data. For units shipped after April 2026)
./start_gripper.cpp left --camera-fps 60
```

After startup, six image windows appear.

## Program Usage
### Reading Sensor Data
Use the following callbacks:
```

capture_frames_callback  // Camera frame capture callback
tactile_callback         // Tactile data callback
encoder_callback         // Gripper opening distance callback
```

### Sending Gripper Open/Close Commands
Use:
```
if (databus_) {
        databus_->setTargetDistance(distance);
```

## Device Parameter Retrieval (do not run other control programs)

### Run the Script Directly
```
./camera_cmd.sh [left|right]  <command>
```

**Parameters:**

| Parameter   | Description |
|------------|-------------|
| `camerarc` | Calibrate center camera (writes `cam0_sensor.yaml`) |
| `camerarl` | Calibrate left camera (writes `cam1_sensor.yaml`) |
| `camerarr` | Calibrate right camera (writes `cam2_sensor.yaml`) |
| `MCUID`    | Query device MCUID |

**Calibration YAML files** are saved under `gen_controller_sdk_cpp/calib_result`.

### Examples

### Single Gripper
#### Obtain Camera Calibration Files
```
Center camera
./camera_cmd.sh camerarc
Left camera
./camera_cmd.sh camerarl
Right camera
./camera_cmd.sh camerarr
```
#### Query Device ID
```
./camera_cmd.sh MCUID
```
### Dual Gripper
#### Obtain Camera Calibration Files
```
Center camera
./camera_cmd.sh left camerarc
./camera_cmd.sh right camerarc
Left camera
./camera_cmd.sh left camerarl
./camera_cmd.sh right camerarl
Right camera
./camera_cmd.sh left camerarr
./camera_cmd.sh right camerarr
```

#### Query Device ID
```
./camera_cmd.sh left MCUID
./camera_cmd.sh right MCUID
```
