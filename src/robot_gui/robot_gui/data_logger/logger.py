"""Simple CSV data logger used by the GUI.

This module provides :class:`DataLogger`, a minimal helper that records
incoming telemetry dictionaries to a CSV file. The class is intentionally
lightweight and synchronous (writes occur on the thread calling
``write_entry``). For the target sample rate (~5 Hz) this is acceptable,
but for higher rates consider moving writes to a background thread or
batching rows.

Public API
- ``start_logging(filepath: str)``: open a CSV file and begin recording.
- ``stop_logging()``: close any open file and stop recording.
- ``write_entry(data: dict)``: append a row to the CSV. Expected keys: ``battery``,
  ``velocity``, ``sensor``.
- ``get_last_log_path()`` / ``get_log_history()``: helpers for UI download and
  management.

Design notes
- The class is a :class:`PySide6.QtCore.QObject` so its methods can be used
  as Qt slots. The GUI connects the ROSInterface.log_data signal directly to
  :meth:`write_entry` and controls lifecycle via :meth:`start_logging`/
  :meth:`stop_logging`.
"""

import csv
import time
import os
from PySide6.QtCore import QObject, Slot


class DataLogger(QObject):
    """Minimal CSV logger for telemetry data.

    Instances track the current open file and maintain a session-local
    history of created log files (useful for the GUI "Download" flow).
    """

    def __init__(self):
        super().__init__()
        self.writer = None
        self.file = None
        self.logging = False
        self.current_filepath = None
        # keep a history of all log files created during this app session
        self.log_history = []

    @Slot(str)
    def start_logging(self, filename: str = ""):
        """Start logging to the given filepath.

        If ``filename`` is empty a timestamped CSV file will be created in the
        current working directory. The function records the absolute path in
        ``log_history`` for later retrieval by the UI.
        """
        if not self.logging:
            if filename:
                # honor the provided path
                filepath = filename
            else:
                timestamp = time.strftime("%Y%m%d-%H%M%S")
                filepath = os.path.join(os.getcwd(), f"robot_log_{timestamp}.csv")

            # ensure directory exists
            dirpath = os.path.dirname(os.path.abspath(filepath))
            if dirpath and not os.path.exists(dirpath):
                try:
                    os.makedirs(dirpath, exist_ok=True)
                except Exception:
                    pass

            # Open the file and write the CSV header
            self.file = open(filepath, 'w', newline='')
            self.current_filepath = os.path.abspath(filepath)
            # record in session history
            try:
                self.log_history.append(self.current_filepath)
            except Exception:
                pass
            self.writer = csv.writer(self.file)
            self.writer.writerow(['Timestamp', 'Battery', 'Velocity', 'FlowSensor'])
            self.logging = True
            print(f"[Logger] Logging started: {self.current_filepath}")

    @Slot()
    def stop_logging(self):
        """Close the CSV file safely and update internal state."""
        if self.file:
            self.file.close()
            print("[Logger] Logging stopped and file closed.")
        self.file = None
        self.writer = None
        self.logging = False
        self.current_filepath = None

    @Slot(dict)
    def write_entry(self, data: dict):
        """Write a new data row to CSV.

        Expected ``data`` keys: ``battery``, ``velocity``, ``sensor``.
        The method flushes the file after each write to minimize data loss in
        case of a crash. This synchronous behavior is acceptable at low
        sample rates but should be swapped to an asynchronous writer for
        high-throughput logging.
        """
        if not self.logging or not self.writer:
            return
        timestamp = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime())
        row = [timestamp, data['battery'], data['velocity'], data['sensor']]
        try:
            self.writer.writerow(row)
            if self.file:
                try:
                    self.file.flush()
                except Exception:
                    pass
        except Exception:
            # Keep logger robust: ignore  write errors to avoid crashing the UI
            # In a production system, forward this to a logger or error handler
            pass

    def get_last_log_path(self):
        """Return the absolute path to the active log file (or last created file),
        or None if none exists.
        """
        return self.current_filepath

    def get_log_history(self):
        """Return a list of absolute file paths for logs created during this session.
        The list is ordered by creation time (oldest first).
        """
        return list(self.log_history)