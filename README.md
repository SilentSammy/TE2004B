# TE2004B - Embedded Systems Robot Project

An autonomous robot control system built on STM32H7 microcontroller with camera-based and encoder-based navigation, and sensor fusion.

## Project Structure

### 📁 `stm32_h7_reto/` - **Main Firmware (STM32H7)**
The active production firmware for the STM32H745 microcontroller running on the Cortex-M7 core.

**Key Features:**
- CAN-based sensor data consumption
- Camera-based waypoint navigation system
- Encoder-based odometry and position tracking
- PWM motor control
- UART command interface

**Structure:**
- `CM7/Core/Src/main.c` - Main control loop, navigation logic, sensor processing
- `Drivers/` - STM32 HAL drivers

**CAN Message IDs:**
- `0x125` - Motor control commands
- `0x126` - Angular velocity (IMU gyroscope)
- `0x127` - Linear acceleration (IMU accelerometer)
- `0x128` - Camera position data (x, y, width)

---

### 📁 `sensor_hub/` - **ESP32-C3 Sensor Bridge**
ESP32-C3 based sensor aggregation and communication hub.

**Purpose:** Collects data from multiple sensors and forwards it to the STM32H7 via CAN bus.

**Components:**
- `sensor_hub.ino` - Main sketch integrating all sensors
- `ble_server.ino` - BLE server for remote control (extra feature)
- `ble_client.h` - BLE client for camera position data
- `imu.ino` - MPU6050 IMU data acquisition
- `encoder.ino` - Wheel encoder reading

**Hardware:**
- ESP32-C3 microcontroller
- MCP2515 CAN controller
- MPU6050 IMU sensor
- Optical encoders

---

### 📁 `ble_controller/` - **Python Remote Control (Extra Feature)**
Optional PC-based wireless control interface using BLE.

**Files:**
- `ble_client.py` - Main controller connecting to sensor hub
  - WASD keyboard control
  - Gamepad support (Xbox/PS controllers)
  - Real-time throttle/steering commands
  - Configurable scaling factors

- `combined_input.py` - Unified keyboard + gamepad input handler
  - Button mapping and axis normalization
  - Rising/falling edge detection
  - Toggle states

- `keybrd.py` - Legacy keyboard-only handler

- `scales.txt` - Control sensitivity configuration
  ```
  0.7  # Throttle scale
  0.6  # Steering scale
  ```

**Usage:**
```bash
cd ble_controller
python3 ble_client.py
# Use WASD or gamepad to control
# Press X or A button to toggle LED
# ESC to exit
```

---

### 📁 `tests/` - **Development Tools**
Testing and development utilities.

**Contents:**
- `ble_client/ble_client.ino` - Standalone BLE-to-CAN camera bridge
  - Used for testing camera integration independently
  - Has fake data mode for STM32 testing without hardware
  - Simpler alternative to full sensor_hub for debugging

**When to use:**
- Testing camera-based navigation without full system
- Debugging CAN message format (0x128)
- Simulating camera input with `ENABLE_FAKE_DATA = true`

---

## Quick Start

### Building the STM32 Firmware
1. Open `stm32_h7_reto/` in STM32CubeIDE
2. Build both CM7 and CM4 cores
3. Flash to STM32H745 Nucleo board

### Running the Sensor Hub
1. Open `sensor_hub/sensor_hub.ino` in Arduino IDE
2. Select ESP32-C3 board
3. Install libraries: BLEDevice, Adafruit_MPU6050, mcp_can
4. Upload to ESP32-C3

### Remote Control Setup (Optional Extra Feature)
1. Ensure sensor hub is running with BLE server enabled
2. Install Python dependencies:
   ```bash
   pip install bleak pynput inputs
   ```
3. Run the controller:
   ```bash
   cd py
   python3 ble_client.py
   ```

## Current Features

### Navigation Modes
- **Camera-based waypoint navigation** - Tracks objects via BLE camera feed
- **Encoder-based waypoint system** - Odometry-based path following

### Sensor Integration
- IMU orientation and motion tracking
- Wheel encoder position estimation
- Camera object tracking and position

### Communication
- CAN bus for sensor data (500kbps)
- BLE for camera data transmission
- UART for debugging and commands

### Extra Features
- **Manual remote control** - Direct keyboard/gamepad control via BLE (optional)

## Git History

Recent development timeline:
```
377d7cb - camera-based waypoint event system working
20ea45f - camera data being received, encoder-based waypoint system working
4d67993 - added encoder-based waypointLoop
0ceab1d - sending raw IMU values via CAN
3d2152a - remote control working perfectly!
```

## Hardware Requirements

- **STM32H745ZI-Q** Nucleo board (dual-core)
- **ESP32-C3** microcontroller
- **MCP2515** CAN controller module
- **MPU6050** IMU sensor
- Optical encoders
- Motor drivers (ESC compatible)
- BLE camera (phone or ESP32-CAM)

## Development Branch

Current branch: `feature/thirdDelivery`

Main branch: `main`

---

**Last Updated:** November 24, 2025
