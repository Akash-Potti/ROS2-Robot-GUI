"""Status panel widget displaying robot telemetry and operation state.

The :class:`StatusPanel` is a simple informative widget that shows:
- operation state (Idle / Running / Stopped)
- battery percentage (with a progress bar)
- current velocity

Public methods
- ``update_operation_status(status_text)`` — update the operation label.
- ``update_battery(battery_percent)`` — update the battery label and progress.
- ``update_velocity(velocity_value)`` — update the velocity label.

This widget is intentionally presentation-focused and does not perform any
data acquisition. It expects to be updated by a coordinator (for example,
the :class:`robot_gui.gui.main_window.MainWindow`) when new telemetry arrives.
"""
from PySide6.QtWidgets import QWidget, QVBoxLayout, QLabel, QFrame, QProgressBar
from PySide6.QtCore import Qt
from PySide6.QtGui import QFont


class StatusPanel(QWidget):
    """Widget displaying robot status information"""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self._init_ui()
        
    def _init_ui(self):
        """Initialize the status panel UI"""
        layout = QVBoxLayout()
        layout.setSpacing(10)
        layout.setContentsMargins(12, 12, 12, 12)
        
        # Title
        title = QLabel("📡 Robot Status")
        title.setFont(QFont("Arial", 13, QFont.Bold))
        title.setStyleSheet("color: #E0E0E0; padding: 5px;")
        layout.addWidget(title)
        
        # Separator
        separator = QFrame()
        separator.setFrameShape(QFrame.HLine)
        separator.setStyleSheet("background-color: #424242;")
        layout.addWidget(separator)
        
        # Operation Status
        status_container = QFrame()
        status_container.setStyleSheet("""
            QFrame {
                background-color: #2C2C2C;
                border-radius: 4px;
                padding: 10px;
            }
        """)
        status_layout = QVBoxLayout()
        status_layout.setSpacing(5)
        
        status_header = QLabel("Operation Status")
        status_header.setFont(QFont("Arial", 10, QFont.Bold))
        status_header.setStyleSheet("color: #B0B0B0;")
        status_layout.addWidget(status_header)
        
        self.status_label = QLabel("⚪ Idle")
        self.status_label.setFont(QFont("Arial", 12, QFont.Bold))
        self.status_label.setStyleSheet("color: #9E9E9E; padding: 5px;")
        status_layout.addWidget(self.status_label)
        
        status_container.setLayout(status_layout)
        layout.addWidget(status_container)
        
        # Battery Section
        battery_container = QFrame()
        battery_container.setStyleSheet("""
            QFrame {
                background-color: #2C2C2C;
                border-radius: 4px;
                padding: 10px;
            }
        """)
        battery_layout = QVBoxLayout()
        battery_layout.setSpacing(8)
        
        battery_header = QLabel("🔋 Battery Level")
        battery_header.setFont(QFont("Arial", 10, QFont.Bold))
        battery_header.setStyleSheet("color: #B0B0B0;")
        battery_layout.addWidget(battery_header)
        
        self.battery_label = QLabel("-- %")
        self.battery_label.setFont(QFont("Arial", 18, QFont.Bold))
        self.battery_label.setStyleSheet("color: #66BB6A; padding: 5px;")
        self.battery_label.setAlignment(Qt.AlignCenter)
        battery_layout.addWidget(self.battery_label)
        
        # Battery Progress Bar
        self.battery_progress = QProgressBar()
        self.battery_progress.setRange(0, 100)
        self.battery_progress.setValue(0)
        self.battery_progress.setTextVisible(False)
        self.battery_progress.setMinimumHeight(16)
        self.battery_progress.setMaximumHeight(20)
        self.battery_progress.setStyleSheet("""
            QProgressBar {
                border: 2px solid #424242;
                border-radius: 4px;
                background-color: #1E1E1E;
            }
            QProgressBar::chunk {
                background-color: #66BB6A;
                border-radius: 2px;
            }
        """)
        battery_layout.addWidget(self.battery_progress)
        
        battery_container.setLayout(battery_layout)
        layout.addWidget(battery_container)
        
        # Velocity Section
        velocity_container = QFrame()
        velocity_container.setStyleSheet("""
            QFrame {
                background-color: #2C2C2C;
                border-radius: 4px;
                padding: 10px;
            }
        """)
        velocity_layout = QVBoxLayout()
        velocity_layout.setSpacing(8)
        
        velocity_header = QLabel("⚡ Velocity")
        velocity_header.setFont(QFont("Arial", 10, QFont.Bold))
        velocity_header.setStyleSheet("color: #B0B0B0;")
        velocity_layout.addWidget(velocity_header)
        
        self.velocity_label = QLabel("-- m/s")
        self.velocity_label.setFont(QFont("Arial", 18, QFont.Bold))
        self.velocity_label.setStyleSheet("color: #42A5F5; padding: 5px;")
        self.velocity_label.setAlignment(Qt.AlignCenter)
        velocity_layout.addWidget(self.velocity_label)
        
        velocity_container.setLayout(velocity_layout)
        layout.addWidget(velocity_container)
        
        # Add stretch to push everything to the top
        layout.addStretch()
        
        self.setLayout(layout)
        
        # Set panel background
        self.setStyleSheet("""
            StatusPanel {
                background-color: #1E1E1E;
                border-radius: 8px;
            }
        """)
        
    def update_operation_status(self, status_text):
        """Update the operation status display"""
        status_map = {
            "Idle": ("⚪", "#9E9E9E"),
            "Started": ("🟢", "#66BB6A"),
            "Running": ("🟢", "#66BB6A"),
            "Stopped": ("🔴", "#EF5350")
        }
        
        icon, color = status_map.get(status_text, ("⚪", "#9E9E9E"))
        self.status_label.setText(f"{icon} {status_text}")
        self.status_label.setStyleSheet(f"color: {color}; padding: 5px; font-weight: bold;")
        
    def update_battery(self, battery_percent):
        """Update battery display"""
        battery_value = round(battery_percent, 1)
        self.battery_label.setText(f"{battery_value} %")
        self.battery_progress.setValue(int(battery_value))
        
        # Change color based on battery level
        if battery_value > 50:
            color = "#66BB6A"  # Green
        elif battery_value > 20:
            color = "#FFA726"  # Orange
        else:
            color = "#EF5350"  # Red
            
        self.battery_label.setStyleSheet(f"color: {color}; padding: 5px; font-weight: bold;")
        self.battery_progress.setStyleSheet(f"""
            QProgressBar {{
                border: 2px solid #424242;
                border-radius: 4px;
                background-color: #1E1E1E;
            }}
            QProgressBar::chunk {{
                background-color: {color};
                border-radius: 2px;
            }}
        """)
        
    def update_velocity(self, velocity_value):
        """Update velocity display"""
        self.velocity_label.setText(f"{velocity_value} m/s")
