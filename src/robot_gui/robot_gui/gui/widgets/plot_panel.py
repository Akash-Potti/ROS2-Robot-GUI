"""Live plotting panel for sensor and velocity data.

The :class:`PlotPanel` widget provides two live plots (flow sensor and
velocity) using :mod:`pyqtgraph`. It exposes the following signal:

- ``interaction_toggled`` (bool): emitted when the user toggles plot
    interactivity (pan/zoom) via the UI control.

API and behavior
- ``update_plot(value)`` appends a new flow sensor sample and updates the
    flow plot and associated statistics.
- ``update_velocity(value)`` appends a new velocity sample and updates the
    velocity plot and its statistics.
- The widget uses small fixed-size deques (``maxlen=100``) to bound memory
    usage and keep rendering efficient for typical embedded systems.

Performance notes
- Point symbols are enabled in the current UI which look nice for sparse
    data but can be disabled for higher sample rates to improve rendering
    performance. See :meth:`_set_plot_interactivity` for interaction controls.
"""
from PySide6.QtWidgets import QWidget, QVBoxLayout, QLabel, QFrame, QPushButton
from PySide6.QtGui import QFont
from PySide6.QtCore import Signal
import pyqtgraph as pg
from collections import deque
from typing import cast


class PlotPanel(QWidget):
    """Widget containing the live data plot"""

    interaction_toggled = Signal(bool)
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.flow_buffer = deque(maxlen=100)
        self.velocity_buffer = deque(maxlen=100)
        self._interaction_enabled = False
        self._init_ui()
        self._set_plot_interactivity(False)
        
    def _init_ui(self):
        """Initialize the plot panel UI"""
        layout = QVBoxLayout()
        layout.setSpacing(8)
        layout.setContentsMargins(12, 12, 12, 12)

    # Flow plot title
        title = QLabel("📈 Flow Sensor Data")
        title.setFont(QFont("Arial", 13, QFont.Weight.Bold))
        title.setStyleSheet("color: #E0E0E0; padding: 5px;")
        layout.addWidget(title)

        # Flow section separator
        separator = QFrame()
        separator.setFrameShape(QFrame.Shape.HLine)
        separator.setStyleSheet("background-color: #424242;")
        layout.addWidget(separator)

        # Interaction toggle
        self.interaction_btn = QPushButton("Enable Plot Interaction")
        self.interaction_btn.setCheckable(True)
        self.interaction_btn.setStyleSheet("""
            QPushButton {
                background-color: #2C2C2C;
                color: #E0E0E0;
                border: 1px solid #424242;
                border-radius: 4px;
                padding: 8px;
                font-size: 11px;
            }
            QPushButton:checked {
                background-color: #42A5F5;
                color: #121212;
                border: 1px solid #64B5F6;
            }
        """)
        self.interaction_btn.toggled.connect(self._on_interaction_toggled)
        layout.addWidget(self.interaction_btn)

        # Flow plot widget
        self.flow_plot = pg.PlotWidget()
        self.flow_plot.setBackground('#121212')
        self.flow_plot.setMinimumHeight(240)
        self.flow_plot.setLabel(
            'left', 'Flow (L/min)', color='#B0B0B0', **{'font-size': '10pt'}
        )
        self.flow_plot.setLabel(
            'bottom', 'Time (samples)', color='#B0B0B0', **{'font-size': '10pt'}
        )
        self.flow_plot.showGrid(x=True, y=True, alpha=0.2)

        flow_axis_pen = pg.mkPen(color='#424242', width=1)
        self.flow_plot.getAxis('left').setPen(flow_axis_pen)
        self.flow_plot.getAxis('bottom').setPen(flow_axis_pen)
        self.flow_plot.getAxis('left').setTextPen('#B0B0B0')
        self.flow_plot.getAxis('bottom').setTextPen('#B0B0B0')

        flow_pen = pg.mkPen(color='#42A5F5', width=2)
        self.flow_curve = self.flow_plot.plot(pen=flow_pen)
        self.flow_curve.setSymbol('o')
        self.flow_curve.setSymbolSize(5)
        self.flow_curve.setSymbolBrush('#42A5F5')
        self.flow_curve.setSymbolPen(None)
        self.flow_plot.setYRange(0, 12)
        layout.addWidget(cast(QWidget, self.flow_plot))

        # Flow stats panel
        flow_stats_container = QFrame()
        flow_stats_container.setStyleSheet("""
            QFrame {
                background-color: #2C2C2C;
                border-radius: 4px;
                padding: 10px;
            }
        """)
        flow_stats_layout = QVBoxLayout()
        flow_stats_layout.setSpacing(5)

        flow_stats_header = QLabel("📊 Flow Statistics")
        flow_stats_header.setFont(QFont("Arial", 10, QFont.Weight.Bold))
        flow_stats_header.setStyleSheet("color: #B0B0B0;")
        flow_stats_layout.addWidget(flow_stats_header)

        self.flow_stats_label = QLabel(
            "Current: -- L/min | Min: -- L/min | Max: -- L/min | Avg: -- L/min"
        )
        self.flow_stats_label.setWordWrap(True)
        self.flow_stats_label.setStyleSheet("""
            QLabel {
                color: #E0E0E0;
                font-size: 10px;
                padding: 5px;
            }
        """)
        flow_stats_layout.addWidget(self.flow_stats_label)

        flow_stats_container.setLayout(flow_stats_layout)
        layout.addWidget(flow_stats_container)

        # Velocity section title
        velocity_separator = QFrame()
        velocity_separator.setFrameShape(QFrame.Shape.HLine)
        velocity_separator.setStyleSheet("background-color: #424242;")
        layout.addWidget(velocity_separator)

        velocity_title = QLabel("🚀 Velocity Data")
        velocity_title.setFont(QFont("Arial", 13, QFont.Weight.Bold))
        velocity_title.setStyleSheet("color: #E0E0E0; padding: 5px;")
        layout.addWidget(velocity_title)

        # Velocity plot widget
        self.velocity_plot = pg.PlotWidget()
        self.velocity_plot.setBackground('#121212')
        self.velocity_plot.setMinimumHeight(240)
        self.velocity_plot.setLabel(
            'left', 'Velocity (m/s)', color='#B0B0B0', **{'font-size': '10pt'}
        )
        self.velocity_plot.setLabel(
            'bottom', 'Time (samples)', color='#B0B0B0', **{'font-size': '10pt'}
        )
        self.velocity_plot.showGrid(x=True, y=True, alpha=0.2)

        velocity_axis_pen = pg.mkPen(color='#424242', width=1)
        self.velocity_plot.getAxis('left').setPen(velocity_axis_pen)
        self.velocity_plot.getAxis('bottom').setPen(velocity_axis_pen)
        self.velocity_plot.getAxis('left').setTextPen('#B0B0B0')
        self.velocity_plot.getAxis('bottom').setTextPen('#B0B0B0')

        velocity_pen = pg.mkPen(color='#FF7043', width=2)
        self.velocity_curve = self.velocity_plot.plot(pen=velocity_pen)
        self.velocity_curve.setSymbol('o')
        self.velocity_curve.setSymbolSize(5)
        self.velocity_curve.setSymbolBrush('#FF7043')
        self.velocity_curve.setSymbolPen(None)
        self.velocity_plot.setYRange(0, 1.5)
        layout.addWidget(cast(QWidget, self.velocity_plot))

        # Velocity stats panel
        velocity_stats_container = QFrame()
        velocity_stats_container.setStyleSheet("""
            QFrame {
                background-color: #2C2C2C;
                border-radius: 4px;
                padding: 10px;
            }
        """)
        velocity_stats_layout = QVBoxLayout()
        velocity_stats_layout.setSpacing(5)

        velocity_stats_header = QLabel("📊 Velocity Statistics")
        velocity_stats_header.setFont(QFont("Arial", 10, QFont.Weight.Bold))
        velocity_stats_header.setStyleSheet("color: #B0B0B0;")
        velocity_stats_layout.addWidget(velocity_stats_header)

        self.velocity_stats_label = QLabel(
            "Current: -- m/s | Min: -- m/s | Max: -- m/s | Avg: -- m/s"
        )
        self.velocity_stats_label.setWordWrap(True)
        self.velocity_stats_label.setStyleSheet("""
            QLabel {
                color: #E0E0E0;
                font-size: 10px;
                padding: 5px;
            }
        """)
        velocity_stats_layout.addWidget(self.velocity_stats_label)

        velocity_stats_container.setLayout(velocity_stats_layout)
        layout.addWidget(velocity_stats_container)
        
        self.setLayout(layout)
        
        # Set panel background
        self.setStyleSheet("""
            PlotPanel {
                background-color: #1E1E1E;
                border-radius: 8px;
            }
        """)
        
    def update_plot(self, value):
        """Update the plot with new sensor data"""
        self.flow_buffer.append(value)
        data_list = list(self.flow_buffer)
        self.flow_curve.setData(data_list)
        
        # Update statistics
        if data_list:
            current = data_list[-1]
            min_val = min(data_list)
            max_val = max(data_list)
            avg_val = sum(data_list) / len(data_list)
            
            self.flow_stats_label.setText(
                f"Current: {current:.2f} L/min | "
                f"Min: {min_val:.2f} L/min | "
                f"Max: {max_val:.2f} L/min | "
                f"Avg: {avg_val:.2f} L/min"
            )
    
    def update_velocity(self, value):
        """Update the velocity plot with new status data."""
        self.velocity_buffer.append(value)
        data_list = list(self.velocity_buffer)
        self.velocity_curve.setData(data_list)

        if data_list:
            current = data_list[-1]
            min_val = min(data_list)
            max_val = max(data_list)
            avg_val = sum(data_list) / len(data_list)

            self.velocity_stats_label.setText(
                f"Current: {current:.2f} m/s | "
                f"Min: {min_val:.2f} m/s | "
                f"Max: {max_val:.2f} m/s | "
                f"Avg: {avg_val:.2f} m/s"
            )

    def clear_plot(self):
        """Clear all data from the plot"""
        self.flow_buffer.clear()
        self.velocity_buffer.clear()
        self.flow_curve.setData([])
        self.velocity_curve.setData([])
        self.flow_stats_label.setText(
            "Current: -- L/min | Min: -- L/min | Max: -- L/min | Avg: -- L/min"
        )
        self.velocity_stats_label.setText(
            "Current: -- m/s | Min: -- m/s | Max: -- m/s | Avg: -- m/s"
        )

    def _on_interaction_toggled(self, enabled: bool):
        """Handle interaction mode toggling for plots."""
        self._interaction_enabled = enabled
        self._set_plot_interactivity(enabled)
        if enabled:
            self.interaction_btn.setText("Disable Plot Interaction")
        else:
            self.interaction_btn.setText("Enable Plot Interaction")
        self.interaction_toggled.emit(enabled)

    def _set_plot_interactivity(self, enabled: bool):
        """Enable or disable mouse-driven plot interaction."""
        self.flow_plot.setInteractive(enabled)
        self.velocity_plot.setInteractive(enabled)
        self.flow_plot.setMouseEnabled(enabled, enabled)
        self.velocity_plot.setMouseEnabled(enabled, enabled)
        self.flow_plot.setMenuEnabled(enabled)
        self.velocity_plot.setMenuEnabled(enabled)
