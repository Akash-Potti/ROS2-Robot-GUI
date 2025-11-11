# Design decisions for robot_gui

## Overview

This project provides a small desktop GUI for simulating and controlling a robot via ROS 2. The main goals are:

- Provide Start/Stop controls and a simple status panel (battery, velocity).
- Visualize a live data stream (Here I have simulated a flow sensor) and velocity over time.
- Allow recording telemetry to CSV for later analysis.
- Keep the code modular and suitable for constrained/embedded deployments.

## High-level architecture

The code is organized into clear components:

- Entry point: `main.py` — creates the Qt application, starts the ROS interface spinning thread, and shows the main window.
- GUI coordinator: `gui/main_window.py` — composes the left control/status panel and the right plotting area. It wires signals between GUI widgets, the ROS adapter, and the DataLogger.
- Widgets: `gui/widgets/*.py` — `ControlPanel`, `StatusPanel`, and `PlotPanel`. These are presentation-focused and emit or accept signals/data but do not perform ROS operations directly.
- ROS adapter: `ros_interface/ros_node.py` — a small `rclpy` node that simulates telemetry and publishes control messages. It emits Qt signals (status, sensor samples, log-ready dictionaries) consumed by the GUI.
- Logger: `data_logger/logger.py` — simple CSV writer that records incoming dicts to a CSV file and maintains a session history of created files.

## Design choices and rationale

1. Qt widgets + pyqtgraph for the GUI and plotting

- PySide6 (Qt for Python) gives a native-feeling desktop GUI on Ubuntu and embedded Linux.
- pyqtgraph is lightweight and efficient for real-time plotting; it supports many optimizations (decimation, OpenGL) if needed.

2. Separation of concerns

- The GUI only handles presentation and user interactions. The ROS-specific behaviors live in a small adapter node. This keeps the UI modular and simplifies testing.
- DataLogger is independent and designed to be connected to any source emitting the expected dict structure.

3. Signals-based integration

- The GUI uses Qt signals/slots to decouple widgets and to allow easy unit testing of components.
- ROSInterface emits Qt signals for status and sensor updates; MainWindow connects those signals to widget update methods and to the logger.

4. Threading model (chosen for simplicity)

- The application runs `rclpy.spin()` in a background Python thread while keeping the Qt event loop in the main thread. This keeps the GUI responsive without requiring advanced thread orchestration.
- Note: Emitting Qt signals from a different thread is supported via queued connections, but mixing `QObject` usage across threads is brittle. For production stability it's recommended to migrate to a safer pattern (see "Thread-safety notes").

5. Simplicity and embedded suitability

- Buffers are bounded (`deque(maxlen=100)`), keeping memory usage predictable on embedded devices.
- CSV logging is synchronous and simple; for low sample rates (5 Hz) this is acceptable. For higher throughput, an async/batched writer is recommended.

## Key implementation details

- Control commands: `std_msgs/String` published on topic `control_command` when Start/Stop are pressed.
- Status updates: simulated battery and velocity are emitted periodically by ROSInterface via Qt signal `status_updated`.
- Sensor stream: simulated flow values emitted as `sensor_updated` and used for plotting and logging.
- Logging: `DataLogger.write_entry` expects a dict with keys `battery`, `velocity`, `sensor` and writes a timestamped CSV row.

## Logging

- The current CSV logger flushes after each write to minimize data loss.
- For long recording sessions or higher rates, I think i would shift to batch write.
