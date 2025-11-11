# Running the Robot GUI (Ubuntu)

This document describes how to install dependencies and run the `robot_gui` application
on an Ubuntu-based system with ROS 2 available.

## Prerequisites

- A supported ROS 2 (Humble) distribution installed
  Follow the ROS 2 installation guide for your Ubuntu version: https://docs.ros.org/
- Python 3.10 (this workspace uses 3.10 in its install layout; adjust if needed).
- Recommended: create a Python virtual environment for local Python-only packages.

## Dependencies

Project Python dependencies (non-ROS):

- PySide6 (Qt for Python)
- pyqtgraph

The ROS 2 Python packages (rclpy, std_msgs, etc.) come from the ROS 2 distribution and are not normally installed via pip.

## Install Python packages (example)

From the workspace root (eg: `/home/akash/ros2_gui_ws`):

```bash
# (optional) create and activate a venv
python3 -m venv .venv
source .venv/bin/activate

# install pip deps listed in requirements.txt
pip install -r requirements.txt
```

Note: `requirements.txt` in this workspace should list PySide6 and pyqtgraph. If PySide6 installation fails (system Qt headers required), consider installing PySide6 via your package manager or consult the PySide6 docs.

## Build and source the workspace (ROS 2 / colcon)

If you modified the package or are running from source, build and source the ROS2 workspace:

```bash
# from workspace root
colcon build --packages-select robot_gui

# source the workspace overlay in the current shell
source install/setup.bash
```

## Running the GUI

Assuming the package entrypoint is set (the package already provides an entrypoint script), you can run the GUI with:

```bash
# ensure your ROS 2 environment is sourced
source /opt/ros/humble/setup.bash
# then source the workspace overlay if built
source install/setup.bash

# run via ros2 run (preferred)
ros2 run robot_gui robot_gui
```

Alternatively, you can run the module directly (useful for debugging):

```bash
python3 -m robot_gui.main
```

## Notes about environment and common issues

- rclpy and ROS 2 libraries must match your OS and be sourced before running. If you see import errors for rclpy, make sure you have sourced the ROS 2 setup script.
- If PySide6 imports fail, verify you installed PySide6 in the active Python environment. On some systems installing PySide6 via pip may require compilation; using the system package (if available) may be easier.
- If the GUI does not appear or freezes, check that the ROS spin is running in a background thread (the provided entrypoint starts a background thread). If you changed the threading model, ensure the Qt main thread remains the one running the Qt event loop.

## Logging files and download

- Log files created by the in-app DataLogger are written to the current working directory by default with filenames like `robot_log_YYYYmmdd-HHMMSS.csv`.
- Use the "Download Log" button in the GUI to copy session log files to a destination folder. The GUI maintains a session-local history which is valid while the application runs.

## Troubleshooting tips

- "ImportError: No module named rclpy": source ROS 2 and ensure your Python uses the ROS-installed rclpy (sourcing `install/setup.bash` from your workspace or `/opt/ros/humble/setup.bash`).
- UI slow or choppy: reduce plotting symbol usage or buffer size; enable OpenGL for pyqtgraph if supported.
- Logging fails to create files: check file permissions and current working directory.

## Quick checklist to run

1. Install ROS 2 (see ROS docs).
2. From workspace root, install Python deps: `pip install -r requirements.txt`.
3. Build the ROS workspace: `colcon build`.
4. Source the setup files and run: `ros2 run robot_gui robot_gui`.
