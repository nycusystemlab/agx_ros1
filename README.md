# 🚀 AGX ROS 1 Dashboard User Guide

This is a **zero-dependency** Web dashboard designed specifically for AGX robot development. It allows you to control Docker services, launch ROS tasks directly via the web, and even drive the robot remotely using a virtual joystick.

## 🛠️ Quick Start

1. **Enter the directory**:
```bash
cd agx_ros1/docker

```


2. **Start the dashboard**:
```bash
python3 dashboard.py --port 8090

```


3. **Access the webpage**:
Open your browser and enter `http://localhost:8090` (or the robot's IP address).

---

## 📖 Features Overview

### 1. Core Services (Docker Services)

Displays the current status of the Docker containers.

* **▶ Start**: Start a specific service.
* **⏹ Stop**: Stop and remove the container.
* **📋 Logs**: View the background logs of the container.
* **Start All / Stop All**: Manage the entire ROS 1 stack with a single click.

### 2. Automated Tasks (Tmux Tasks)

These are pre-configured ROS commands that run within Tmux windows inside the container.

* **▶ Start**: Execute the ROS command in the background.
* **📋 Log (Important)**: Click to **view terminal output in real-time**. If a program crashes or throws an error, this is your ultimate debugging tool.
* **⏹ Stop**: Close the Tmux session for that task.

### 3. 🎮 Virtual Joystick

No physical gamepad required; drive the robot directly from the webpage.

* **Controls**: Click the buttons with your mouse or use the **W/A/S/D** keys on your keyboard.
* **E-Stop**: Press the **Spacebar** or the **⏹ Stop** button.
* **Speed Control**: Use the slider to adjust the movement speed in real-time.

---

## 🛠️ Development & Customization

If you want to modify task commands or add new buttons, simply edit the `SERVICES` and `TASKS` dictionaries at the beginning of `dashboard.py`. No compilation is needed after modifying—just restart the script for the changes to take effect immediately!

---