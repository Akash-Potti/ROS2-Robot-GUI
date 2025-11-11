"""ROS2 interface adapter that simulates robot telemetry.

This module implements :class:`ROSInterface`, a small helper that combines a
ROS2 ``rclpy`` node with Qt signal emission for use by the GUI. The node
publishes simple control commands (``std_msgs/String`` on ``control_command``)
and simulates periodic status and sensor updates which are emitted as
Qt signals so the GUI can remain decoupled from ROS internals.

Important public signals
- ``status_updated`` (dict): periodic status updates containing keys
  ``battery`` and ``velocity``.
- ``sensor_updated`` (float): a single flow sensor float value emitted
  as samples arrive.
- ``log_data`` (dict): a dict suitable for CSV logging; contains ``battery``,
  ``velocity``, and ``sensor`` keys.

Notes on threading
------------------
The code currently runs ``rclpy.spin`` in a background Python thread (see
``main.py``). The timers created by this node will run in that thread and
emit Qt signals. Emitting Qt signals from another thread is supported via
queued connections, but care must be taken if you later modify the class
to access QObject internals from the ROS thread. See the project README or
the MainWindow design notes for recommended safer alternatives (move the
ROS logic into a QThread-owned object or route updates through a thread-
safe queue polled by the GUI thread).
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from PySide6.QtCore import QObject, Signal
import random
import time
import math


class ROSInterface(Node, QObject):
    """A small ROS2 node that also emits Qt signals for GUI consumption.

    The class intentionally keeps simulation logic simple. It publishes
    ``start`` and ``stop`` commands on a ``control_command`` topic and uses
    ROS timers to periodically emit simulated status and sensor samples.
    """

    status_updated = Signal(object)
    sensor_updated = Signal(object)
    log_data = Signal(object)

    def __init__(self):

        try:
            if not rclpy.ok():
                rclpy.init()
        except Exception:

            try:
                rclpy.init()
            except Exception:
                pass

        Node.__init__(self, 'robot_gui_node')
        QObject.__init__(self)

        self.command_pub = self.create_publisher(String, 'control_command', 10)
        self.sim_timer = None
        self.sensor_timer = None
        self.battery = 100.0
        self.velocity = 0.0
        self.sensor_value = 0.0
        # single update period (seconds) used for all periodic updates
        # change this value to alter frequency of both status and sensor updates
        self.update_period = 0.2  # 5 Hz sampling rate

        self.get_logger().info("ROSInterface initialized")

    def publish_start(self):
        msg = String()
        msg.data = "start"
        self.command_pub.publish(msg)
        self.get_logger().info("Published START command")
        # create both timers with the same update period so GUI and logger
        # receive synchronized updates
        if self.sim_timer is None:
            self.sim_timer = self.create_timer(self.update_period, self.simulate_status)
        if self.sensor_timer is None:
            self.sensor_timer = self.create_timer(self.update_period, self.simulate_sensor_data)

    def publish_stop(self):
        msg = String()
        msg.data = "stop"
        self.command_pub.publish(msg)
        self.get_logger().info("Published STOP command")

        # Stop timers
        if self.sim_timer:
            self.sim_timer.cancel()
            self.sim_timer = None
        if self.sensor_timer:
            self.sensor_timer.cancel()
            self.sensor_timer = None

    def simulate_status(self):
        self.velocity = round(random.uniform(0.0, 1.0), 2)
        self.battery = max(0.0, self.battery - random.uniform(0.0001, 0.005))

        status = {
            "battery": round(self.battery, 1),
            "velocity": self.velocity,
        }

        try:
            self.status_updated.emit(status)
        except Exception:
            pass
        self.get_logger().info(f"Status → {status}")

    def simulate_sensor_data(self):
        """Simulate a realistic flow sensor signal (L/min)."""
        # Time-based waveform for smooth variation
        t = time.time() % 60  # seconds within minute (looping pattern)
        base_flow = 5.0 + 2.0 * math.sin(t / 6.0)  # periodic variation
        noise = random.uniform(-0.2, 0.2)          # random noise
        flow_value = round(base_flow + noise, 2)

        # Clip to safe range (no negative flow)
        flow_value = max(0.0, flow_value)

        self.sensor_value = flow_value

        # Prepare data dictionary for GUI + logging
        log_data = {
            "battery": round(self.battery, 1),
            "velocity": self.velocity,
            "sensor": self.sensor_value,   # now "flow sensor" value
        }

        # Emit to GUI and logger
        try:
            self.sensor_updated.emit(self.sensor_value)
            self.log_data.emit(log_data)
        except Exception as e:
            self.get_logger().warn(f"Flow signal emit error: {e}")

    def spin(self):
        try:
            rclpy.spin(self)
        except KeyboardInterrupt:
            pass
        finally:
            self.get_logger().info("Shutting down ROSInterface.")
            self.destroy_node()
            rclpy.shutdown()