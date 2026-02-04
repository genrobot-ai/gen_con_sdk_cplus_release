# genrobot_controller_sdk_cpp
## Environment Setup
```
USB interface must be 3.0
```

## USB Interface Configuration

### Single Gripper USB Configuration
The final configuration is as shown in the figure. After configuration, this USB port can recognize any Gen Controller, and no further configuration is required. The template file is located at:
```
config/99-usb-serial.rules
```
![image/image_1.png](image/image_1.png)

You need to modify the following:
![image/image_2.png](image/image_2.png)

To modify parameter 1, run:
```
cd /dev && ls | grep ttyUSB
udevadm info -a -n /dev/ttyUSB* | grep -E "KERNELS|DRIVERS"
```

Configure the second KERNELS value from the output to position 1:
![image/image_3.png](image/image_3.png)

To modify parameter 2, run:
```
v4l2-ctl --list-devices
```
Output:
![image/image_4.png](image/image_4.png)

Then for the first camera on that USB, run:
```
udevadm info -a -n /dev/video* | grep -E "KERNELS|SUBSYSTEMS"
```
Configure the first KERNELS value from the output to position 2
![image/image_5.png](image/image_5.png)

Then copy the template file to the following location:
```
sudo cp config/99-usb-serial.rules /etc/udev/rules.d/
```
Then reload the configuration:
```
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### Dual Gripper USB Configuration
The final configuration is as shown in the figure.
![image/image_6.png](image/image_6.png)

Modifications required:
![image/image_7.png](image/image_7.png)

First plug in the left gripper and configure it using the single gripper method; then unplug the left gripper, plug in the right gripper, and configure again using the single gripper method; finally reload the configuration.

### Multi-Gripper USB Configuration
Add configurations to 99-usb-serial.rules in the same way.

## Running the SDK
Run the `start_gripper.cpp` script directly (it will compile on demand each run; changes to the script take effect on the next execution).

### Single Gripper Demo

```
cd gen_controller_sdk_cpp

./start_gripper.cpp left   # Current config: gripper opens to fixed 5cm

./start_gripper.cpp left --distance 0.08  # Gripper opens to fixed 8cm; distance range is [0.0, 0.103], i.e. max 10cm

./start_gripper.cpp left --sine-wave  # Gripper opens and closes continuously for 10s

```

After startup, three image windows will open:
```
/camera_0   # Center camera
/camera_1   # Left camera
/camera_2   # Right camera

```
Printed data includes:
```
Tactile data
Gripper distance data
```

### Dual Gripper Demo
```
cd gen_controller_sdk_cpp
Start:
./start_gripper.cpp left
In another terminal, start:
./start_gripper.cpp right
```

After startup, six image windows will open.

## Program Usage
### Sensor Data Reading
Use the following callbacks to obtain data:
```
capture_frames_callback  // Camera frame capture callback
tactile_callback         // Tactile data callback
encoder_callback         // Gripper opening/closing (encoder) data callback
```

### Gripper Open/Close Control Command
Use the following commands:
```
if (databus_) {
        databus_->setTargetDistance(distance);
```
