"""Entry point for the Robot GUI application.

This module creates the Qt application, starts the ROS2 interface in a
background thread, and shows the main window. It is intentionally small;
the implementation delegates behavior to the :class:`robot_gui.ros_interface.ros_node.ROSInterface`
and :class:`robot_gui.gui.main_window.MainWindow` classes.

Usage
-----
Typically invoked via the package entry point (e.g. `ros2 run robot_gui robot_gui`)
or by calling ``main()`` from a small launcher script. The module does not parse
command-line arguments and expects ROS2 to be available in the runtime.

Notes
-----
- ROS spinning is run in a plain Python ``threading.Thread`` so that the
    Qt event loop remains responsive. See :mod:`robot_gui.ros_interface.ros_node`
    for details about thread-safety and signal emission.
- Keep this file minimal to make testing and packaging easier.
"""

import sys
import threading
from PySide6.QtWidgets import QApplication
from robot_gui.gui.main_window import MainWindow
from robot_gui.ros_interface.ros_node import ROSInterface


def main():
        """Create and run the GUI application.

        This function performs three steps:
        1. create the QApplication instance
        2. create and start a background thread to spin the ROS2 node
        3. instantiate and show the MainWindow and run the Qt event loop
        """
        app = QApplication(sys.argv)

        # Create the ROS/Qt interface object. This object implements both the
        # ROS node behavior and emits Qt signals which the GUI subscribes to.
        ros_interface = ROSInterface()

        # Run rclpy.spin in a background thread so the Qt main thread stays
        # responsive. The ROSInterface.spin method handles shutdown.
        ros_thread = threading.Thread(target=ros_interface.spin, daemon=True)
        ros_thread.start()

        # Create and show the main GUI window
        window = MainWindow(ros_interface)
        window.show()

        sys.exit(app.exec())
