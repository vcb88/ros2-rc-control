# ROS 2 RC Control Package

A ROS 2 package designed to interface Radio Control (RC) receivers with ROS 2 based robots running on Raspberry Pi. It supports reading RC signals via PPM (Pulse Position Modulation) and publishing them as ROS topics.

> **Note:** This package is configured for the **Radiomaster TX16S** transmitter layout but can be adapted for other controllers.

## 🚀 Features

*   **PPM Signal Reading:** Reads raw PPM signals from a single GPIO pin on Raspberry Pi using `RPi.GPIO`.
*   **Protocol Conversion:** Internally normalizes PPM pulses to SBUS-compatible ranges for consistency.
*   **Controller Support:**
    *   **PPM Receiver:** For standard RC receivers.
    *   **Joystick:** Use a gamepad connected to the Pi.
    *   **Keyboard:** Control via keyboard arrow keys.
*   **Failsafe:** Detects signal loss and publishes emergency stop flags.
*   **Debug Topics:** Publishes diagnostic information about signal quality and frame errors.

## 🛠 Hardware Requirements

*   **Computer:** Raspberry Pi (3B+, 4, or 5) running Ubuntu/Debian.
*   **RC Receiver:** Any receiver capable of outputting a PPM signal (e.g., FrSky, FlySky).
*   **RC Transmitter:** 8+ channel transmitter (e.g., Radiomaster TX16S).
*   **Wiring:** Connect the Receiver's PPM pin to a GPIO pin (Default: GPIO 17) on the Raspberry Pi. **Ensure common ground!**

## 📦 Dependencies

### ROS 2 Dependencies
*   `rclpy`
*   `std_msgs`
*   **`hexapod_msgs`**: This package relies on a custom message type `RCCommand`. You must have the `hexapod_msgs` package built in your workspace.

### Python Dependencies
```bash
pip3 install RPi.GPIO pyserial numpy
```

## 📥 Installation

1.  **Clone the repository:**
    ```bash
    cd ~/ros2_ws/src
    git clone https://github.com/vcb88/ros2-rc-control.git
    ```

2.  **Fix Package Name (Important):**
    The current `setup.py` incorrectly names the package `hexapod_robot`. You may need to rename it to `rc-control` to match `package.xml` before building.

3.  **Build the package:**
    ```bash
    cd ~/ros2_ws
    colcon build --packages-select rc-control
    source install/setup.bash
    ```

## 🚀 Usage

### Launching the PPM Controller
The main launch file starts the controller node.

```bash
ros2 launch rc-control rccontrol.launch.py
```

### Configuration
Parameters can be modified in `rc-control/controller_config.py` or passed via launch arguments:

*   `ppm_pin`: GPIO pin number (BCM numbering) connected to PPM signal (Default: 17).

## 📡 Topics

### Published Topics
*   `/rc_commands` (`hexapod_msgs/RCCommand`): The parsed RC stick and switch positions.
*   `/ppm_debug` (`std_msgs/String`): Diagnostics info (frame counts, errors).

## 🎮 Controls (TX16S Mapping)

| Stick/Switch | Function |
|--------------|----------|
| **Right Stick X/Y** | Movement / Body rotation |
| **Left Stick X/Y** | Movement / Height / Stride |
| **Switch SE** | Emergency Stop |
| **Potentiometer S1**| Speed Limiter |

## ⚠️ Known Issues
*   The `setup.py` file currently lists the package name as `hexapod_robot`, which conflicts with `package.xml` (`rc-control`).
*   Depends on external `hexapod_msgs` which are not included in this repo.

## 📄 License
Apache License 2.0
