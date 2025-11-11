"""Main window for the Robot GUI dashboard.

This module implements :class:`MainWindow`, the top-level Qt widget that
composes the control panel, status panel, and plotting widgets. The
MainWindow wires together the GUI widgets with a :class:`robot_gui.ros_interface.ros_node.ROSInterface`
instance and a :class:`robot_gui.data_logger.logger.DataLogger` instance.

Public responsibilities
- Layout and styling of the application
- Connecting GUI widget signals (start/stop/logging) to ROS and logger
- Receiving Qt signals from the ROS interface and updating widgets
- Managing log lifecycle and exposing a simple download flow

Design notes
- The window keeps the application logic at a coordinator level and delegates
    rendering/controls to the sub-widgets in ``gui.widgets``. This keeps the UI
    modular and testable.
"""
from PySide6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, 
                               QLabel, QFrame, QScrollArea, QSizePolicy,
                               QFileDialog, QMessageBox, QDialog, QListWidget,
                               QListWidgetItem, QPushButton, QAbstractItemView)
from PySide6.QtCore import Qt
from PySide6.QtGui import QFont
import os
import shutil
import time
from robot_gui.data_logger.logger import DataLogger
from robot_gui.gui.widgets import ControlPanel, StatusPanel, PlotPanel


class MainWindow(QWidget):
    """Main application window with modular widget design"""
    
    def __init__(self, ros_interface):
        super().__init__()
        self.ros_interface = ros_interface
        self.logging_active = False
        
        # Initialize logger
        self.logger = DataLogger()
        
        # Setup UI
        self._init_ui()
        self._connect_signals()
        
    def _init_ui(self):
        """Initialize the main window UI with left/right panel layout"""
        self.setWindowTitle("🤖 ROS2 Robot Control Dashboard")
        self.setMinimumSize(900, 600)
        self.resize(1400, 800)
        
        # Main layout
        main_layout = QVBoxLayout()
        main_layout.setSpacing(0)
        main_layout.setContentsMargins(0, 0, 0, 0)
        
        # Header
        header = self._create_header()
        main_layout.addWidget(header)
        
        # Content area with left and right panels
        content_layout = QHBoxLayout()
        content_layout.setSpacing(10)
        content_layout.setContentsMargins(10, 10, 10, 10)
        
        # LEFT PANEL - Controls and Status (fixed width, expandable height)
        left_panel = self._create_left_panel()
        left_panel.setMinimumWidth(280)
        left_panel.setMaximumWidth(400)
        content_layout.addWidget(left_panel)
        
        # RIGHT PANEL - Live Data Plot (expandable)
        right_panel = self._create_right_panel()
        right_panel.setMinimumWidth(400)
        content_layout.addWidget(right_panel, stretch=1)
        
        main_layout.addLayout(content_layout)
        
        self.setLayout(main_layout)
        
        # Set window background
        self.setStyleSheet("""
            QWidget {
                background-color: #121212;
                font-family: Arial, sans-serif;
                color: #E0E0E0;
            }
        """)
        
    def _create_header(self):
        """Create the header bar"""
        header = QFrame()
        header.setMinimumHeight(60)
        header.setMaximumHeight(70)
        header.setStyleSheet("""
            QFrame {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                                            stop:0 #1E1E1E, stop:1 #2C2C2C);
                border-bottom: 2px solid #424242;
            }
        """)
        
        header_layout = QHBoxLayout()
        header_layout.setContentsMargins(15, 10, 15, 10)
        
        # Title
        title = QLabel("🤖 ROS2 Robot Control Dashboard")
        title.setFont(QFont("Arial", 16, QFont.Weight.Bold))
        title.setStyleSheet("color: #E0E0E0;")
        header_layout.addWidget(title)
        
        header_layout.addStretch()
        
        # Version label
        version = QLabel("v1.0")
        version.setFont(QFont("Arial", 9))
        version.setStyleSheet("color: #9E9E9E;")
        header_layout.addWidget(version)
        
        header.setLayout(header_layout)
        return header
        
    def _create_left_panel(self):
        """Create the left panel with controls and status"""
        # Scroll area for left panel
        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_area.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        scroll_area.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded)
        scroll_area.setFrameShape(QFrame.Shape.NoFrame)
        
        left_container = QFrame()
        left_container.setStyleSheet("""
            QFrame {
                background-color: #1E1E1E;
                border-radius: 8px;
            }
        """)
        
        left_layout = QVBoxLayout()
        left_layout.setSpacing(12)
        left_layout.setContentsMargins(0, 0, 0, 0)
        
        # Control Panel
        self.control_panel = ControlPanel()
        left_layout.addWidget(self.control_panel)
        
        # Status Panel
        self.status_panel = StatusPanel()
        left_layout.addWidget(self.status_panel)
        
        left_container.setLayout(left_layout)
        scroll_area.setWidget(left_container)
        
        # Style the scroll area
        scroll_area.setStyleSheet("""
            QScrollArea {
                background-color: transparent;
                border: none;
            }
            QScrollBar:vertical {
                background-color: #1E1E1E;
                width: 10px;
                border-radius: 5px;
            }
            QScrollBar::handle:vertical {
                background-color: #424242;
                border-radius: 5px;
                min-height: 20px;
            }
            QScrollBar::handle:vertical:hover {
                background-color: #616161;
            }
            QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {
                height: 0px;
            }
        """)
        
        return scroll_area
        
    def _create_right_panel(self):
        """Create the right panel with live plot"""
        right_container = QFrame()
        right_container.setStyleSheet("""
            QFrame {
                background-color: #1E1E1E;
                border-radius: 8px;
            }
        """)
        
        right_layout = QVBoxLayout()
        right_layout.setContentsMargins(0, 0, 0, 0)
        
        # Plot Panel wrapped in scroll area for vertical scrolling
        self.plot_panel = PlotPanel()
        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_area.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        scroll_area.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded)
        scroll_area.setFrameShape(QFrame.Shape.NoFrame)
        scroll_area.setWidget(self.plot_panel)
        scroll_area.setStyleSheet("""
            QScrollArea {
                background-color: transparent;
                border: none;
            }
            QScrollBar:vertical {
                background-color: #1E1E1E;
                width: 10px;
                border-radius: 5px;
            }
            QScrollBar::handle:vertical {
                background-color: #424242;
                border-radius: 5px;
                min-height: 20px;
            }
            QScrollBar::handle:vertical:hover {
                background-color: #616161;
            }
            QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {
                height: 0px;
            }
        """)
        right_layout.addWidget(scroll_area)

        self.plot_scroll_area = scroll_area
        self.plot_panel.interaction_toggled.connect(self._on_plot_interaction_toggled)  # type: ignore[attr-defined]

        right_container.setLayout(right_layout)
        return right_container
        
    def _connect_signals(self):
        """Connect all signals and slots"""
        # Control panel signals
        self.control_panel.start_clicked.connect(self.on_start_clicked)
        self.control_panel.stop_clicked.connect(self.on_stop_clicked)
        self.control_panel.logging_toggled.connect(self.toggle_logging)
        self.control_panel.download_requested.connect(self._on_download_log)  # type: ignore[attr-defined]
        
        # ROS interface signals
        self.ros_interface.status_updated.connect(self.update_status_panel)
        self.ros_interface.sensor_updated.connect(self.update_plot)
        self.ros_interface.log_data.connect(self.logger.write_entry)
        
    # --- Button handlers ---
    def on_start_clicked(self):
        """Handle start button click"""
        self.status_panel.update_operation_status("Running")
        self.ros_interface.publish_start()
        
    def on_stop_clicked(self):
        """Handle stop button click"""
        self.status_panel.update_operation_status("Stopped")
        self.ros_interface.publish_stop()
        
        # Auto-stop logging when robot stops
        if self.logging_active:
            self.toggle_logging()
            
    def toggle_logging(self):
        """Toggle data logging on/off"""
        if not self.logging_active:
            # Start logging
            filename = f"robot_log_{time.strftime('%Y%m%d-%H%M%S')}.csv"
            filepath = os.path.join(os.getcwd(), filename)
            self.logger.start_logging(filepath)
            self.control_panel.update_logging_status(True, filename)
            self.logging_active = True
        else:
            # Stop logging
            self.logger.stop_logging()
            self.control_panel.update_logging_status(False)
            self.logging_active = False
            
    # --- Data update handlers ---
    def update_status_panel(self, status):
        """Update status panel with new data"""
        self.status_panel.update_battery(status['battery'])
        self.status_panel.update_velocity(status['velocity'])
        self.plot_panel.update_velocity(status['velocity'])  # type: ignore[attr-defined]
        
    def update_plot(self, value):
        """Update live plot with new sensor data"""
        self.plot_panel.update_plot(value)

    def _on_download_log(self):
        """Let the user pick one or more logs from this session and copy them to a folder.

        Uses an in-app selection dialog that lists files recorded in
        DataLogger.get_log_history().
        """
        history = self.logger.get_log_history()  # type: ignore[attr-defined]
        if not history:
            QMessageBox.warning(self, "No log files", "No log files have been created in this session.")
            return

        # Build simple multi-select dialog
        dlg = QDialog(self)
        dlg.setWindowTitle("Select logs to download")
        dlg_layout = QVBoxLayout(dlg)

        listw = QListWidget()
        listw.setSelectionMode(QAbstractItemView.MultiSelection)  # type: ignore[attr-defined]
        for path in history:
            item = QListWidgetItem(os.path.basename(path))
            item.setData(Qt.UserRole, path)  # type: ignore[attr-defined]
            listw.addItem(item)
        dlg_layout.addWidget(listw)

        btn_layout = QHBoxLayout()
        btn_layout.addStretch()
        ok_btn = QPushButton("Download")
        cancel_btn = QPushButton("Cancel")
        btn_layout.addWidget(ok_btn)
        btn_layout.addWidget(cancel_btn)
        dlg_layout.addLayout(btn_layout)

        ok_btn.clicked.connect(dlg.accept)
        cancel_btn.clicked.connect(dlg.reject)

        if dlg.exec() != QDialog.Accepted:  # type: ignore[attr-defined]
            return

        selected = [it.data(Qt.UserRole) for it in listw.selectedItems()]  # type: ignore[attr-defined]
        if not selected:
            QMessageBox.information(self, "No selection", "No logs selected for download.")
            return

        dest_dir = QFileDialog.getExistingDirectory(self, "Select destination folder", os.path.expanduser("~"))
        if not dest_dir:
            return

        # flush open file if any (best-effort)
        try:
            if getattr(self.logger, 'file', None):
                try:
                    self.logger.file.flush()  # type: ignore[attr-defined]
                except Exception:
                    pass
        except Exception:
            pass

        copied = []
        errors = []
        for src in selected:
            try:
                dest_path = os.path.join(dest_dir, os.path.basename(src))
                shutil.copy(src, dest_path)
                copied.append(dest_path)
            except Exception as e:
                errors.append((src, str(e)))

        if copied:
            QMessageBox.information(self, "Download complete", f"Copied {len(copied)} file(s) to: {dest_dir}")
        if errors:
            err_text = "\n".join([f"{s}: {m}" for s, m in errors])
            QMessageBox.warning(self, "Some files failed", f"Some files could not be copied:\n{err_text}")

    def _on_plot_interaction_toggled(self, enabled: bool):
        """Synchronize scroll behavior with plot interaction state."""
        if enabled:
            self.plot_scroll_area.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        else:
            self.plot_scroll_area.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded)
