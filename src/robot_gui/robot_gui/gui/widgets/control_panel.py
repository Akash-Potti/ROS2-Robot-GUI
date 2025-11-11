"""Control panel widget for robot operation controls.

This module exposes :class:`ControlPanel`, a reusable Qt widget containing
the Start/Stop buttons, logging controls, and a small status area. It emits
Qt signals that the parent window listens to:

- ``start_clicked``: emitted when the "Start" button is pressed.
- ``stop_clicked``: emitted when the "Stop" button is pressed.
- ``logging_toggled``: emitted when the logging button is toggled.
- ``download_requested``: emitted when the user requests to download logs.

Usage
-----
Create an instance and connect to the signals::

        panel = ControlPanel()
        panel.start_clicked.connect(on_start)

Implementation details
- Styling is done inline with Qt style sheets for portability.
- The widget exposes ``update_logging_status(active, filename)`` to reflect
    the current logger state in the UI.
"""
from PySide6.QtWidgets import (QWidget, QVBoxLayout, QPushButton, 
                               QLabel, QFrame, QHBoxLayout)
from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QFont, QIcon


class ControlPanel(QWidget):
    """Widget containing control buttons for robot operation"""
    
    start_clicked = Signal()
    stop_clicked = Signal()
    logging_toggled = Signal()
    download_requested = Signal()
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.logging_active = False
        self._init_ui()
        
    def _init_ui(self):
        """Initialize the control panel UI"""
        layout = QVBoxLayout()
        layout.setSpacing(12)
        layout.setContentsMargins(12, 12, 12, 12)
        
        # Title
        title = QLabel("🎮 Robot Controls")
        title.setFont(QFont("Arial", 13, QFont.Bold))
        title.setStyleSheet("color: #E0E0E0; padding: 5px;")
        layout.addWidget(title)
        
        # Separator
        separator = QFrame()
        separator.setFrameShape(QFrame.HLine)
        separator.setStyleSheet("background-color: #424242;")
        layout.addWidget(separator)
        
        # Start Button
        self.start_btn = QPushButton("▶ Start Robot")
        self.start_btn.setMinimumHeight(45)
        self.start_btn.setSizePolicy(self.start_btn.sizePolicy().horizontalPolicy(), 
                                      self.start_btn.sizePolicy().verticalPolicy())
        self.start_btn.setStyleSheet("""
            QPushButton {
                background-color: #66BB6A;
                color: #1E1E1E;
                border: none;
                border-radius: 4px;
                font-size: 13px;
                font-weight: bold;
                padding: 10px;
            }
            QPushButton:hover {
                background-color: #81C784;
            }
            QPushButton:pressed {
                background-color: #4CAF50;
            }
        """)
        self.start_btn.clicked.connect(self._on_start)
        layout.addWidget(self.start_btn)
        
        # Stop Button
        self.stop_btn = QPushButton("⏹ Stop Robot")
        self.stop_btn.setMinimumHeight(45)
        self.stop_btn.setSizePolicy(self.stop_btn.sizePolicy().horizontalPolicy(), 
                                     self.stop_btn.sizePolicy().verticalPolicy())
        self.stop_btn.setStyleSheet("""
            QPushButton {
                background-color: #EF5350;
                color: #1E1E1E;
                border: none;
                border-radius: 4px;
                font-size: 13px;
                font-weight: bold;
                padding: 10px;
            }
            QPushButton:hover {
                background-color: #E57373;
            }
            QPushButton:pressed {
                background-color: #F44336;
            }
        """)
        self.stop_btn.clicked.connect(self._on_stop)
        layout.addWidget(self.stop_btn)
        
        # Logging Section
        logging_title = QLabel("📊 Data Logging")
        logging_title.setFont(QFont("Arial", 12, QFont.Bold))
        logging_title.setStyleSheet("color: #E0E0E0; margin-top: 10px; padding: 5px;")
        layout.addWidget(logging_title)
        
        # Logging Button
        self.log_btn = QPushButton("● Start Logging")
        self.log_btn.setMinimumHeight(40)
        self.log_btn.setSizePolicy(self.log_btn.sizePolicy().horizontalPolicy(), 
                                    self.log_btn.sizePolicy().verticalPolicy())
        self.log_btn.setStyleSheet("""
            QPushButton {
                background-color: #42A5F5;
                color: #1E1E1E;
                border: none;
                border-radius: 4px;
                font-size: 12px;
                font-weight: bold;
                padding: 8px;
            }
            QPushButton:hover {
                background-color: #64B5F6;
            }
            QPushButton:pressed {
                background-color: #2196F3;
            }
        """)
        self.log_btn.clicked.connect(self._on_logging_toggle)
        layout.addWidget(self.log_btn)

        # Download Log Button
        self.download_btn = QPushButton("⬇️ Download Log")
        self.download_btn.setMinimumHeight(36)
        self.download_btn.setStyleSheet("""
            QPushButton {
                background-color: #3A3A3A;
                color: #E0E0E0;
                border: 1px solid #424242;
                border-radius: 4px;
                font-size: 12px;
                padding: 8px;
            }
            QPushButton:hover {
                background-color: #4A4A4A;
            }
        """)
        self.download_btn.clicked.connect(self._on_download)
        layout.addWidget(self.download_btn)
        
        # Logging Status
        self.log_status_label = QLabel("Status: Not Logging")
        self.log_status_label.setWordWrap(True)
        self.log_status_label.setStyleSheet("""
            QLabel {
                background-color: #2C2C2C;
                color: #9E9E9E;
                border-radius: 4px;
                padding: 8px;
                font-size: 10px;
            }
        """)
        # Use AlignmentFlag for newer PySide6 versions
        try:
            self.log_status_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        except Exception:
            self.log_status_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.log_status_label)
        
        # Add stretch to push everything to the top
        layout.addStretch()
        
        self.setLayout(layout)
        
        # Set panel background
        self.setStyleSheet("""
            ControlPanel {
                background-color: #1E1E1E;
                border-radius: 8px;
            }
        """)
        
    def _on_start(self):
        """Handle start button click"""
        self.start_clicked.emit()
        
    def _on_stop(self):
        """Handle stop button click"""
        self.stop_clicked.emit()
        
    def _on_logging_toggle(self):
        """Handle logging toggle"""
        self.logging_toggled.emit()

    def _on_download(self):
        """Emit a request to download the current log file."""
        self.download_requested.emit()
        
    def update_logging_status(self, active, filename=""):
        """Update logging status display"""
        self.logging_active = active
        if active:
            self.log_btn.setText("⏹ Stop Logging")
            self.log_btn.setStyleSheet("""
                QPushButton {
                    background-color: #FF9800;
                    color: #1E1E1E;
                    border: none;
                    border-radius: 4px;
                    font-size: 13px;
                    font-weight: bold;
                    padding: 10px;
                }
                QPushButton:hover {
                    background-color: #FFB74D;
                }
                QPushButton:pressed {
                    background-color: #F57C00;
                }
            """)
            self.log_status_label.setText(f"📝 Recording: {filename}")
            self.log_status_label.setStyleSheet("""
                QLabel {
                    background-color: #2C2C2C;
                    color: #66BB6A;
                    border-radius: 4px;
                    padding: 8px;
                    font-size: 11px;
                    font-weight: bold;
                }
            """)
        else:
            self.log_btn.setText("● Start Logging")
            self.log_btn.setStyleSheet("""
                QPushButton {
                    background-color: #42A5F5;
                    color: #1E1E1E;
                    border: none;
                    border-radius: 4px;
                    font-size: 13px;
                    font-weight: bold;
                    padding: 10px;
                }
                QPushButton:hover {
                    background-color: #64B5F6;
                }
                QPushButton:pressed {
                    background-color: #2196F3;
                }
            """)
            self.log_status_label.setText("Status: Not Logging")
            self.log_status_label.setStyleSheet("""
                QLabel {
                    background-color: #2C2C2C;
                    color: #9E9E9E;
                    border-radius: 4px;
                    padding: 8px;
                    font-size: 11px;
                }
            """)
