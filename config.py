"""
Bifrost Configuration
All constants and configurable parameters in one place
"""

# ========== Serial Communication ==========
SERIAL_TIMEOUT = 0.1  # seconds - SHORT timeout to prevent blocking on busy firmware
SERIAL_BAUDRATE_DEFAULT = 115200

# Serial thread timing (seconds) - OPTIMIZED to prevent GUI lockups
SERIAL_STATUS_REQUEST_INTERVAL = 0.2  # 200ms between status requests (M114/?) - reduced from 100ms
SERIAL_ENDSTOP_REQUEST_INTERVAL = 0.5  # 500ms between endstop requests (M119) - reduced from 100ms (endstops don't change that fast)
SERIAL_THREAD_SLEEP = 0.005  # 5ms sleep to prevent busy-waiting - reduced for more responsive reads
SERIAL_THREAD_SHUTDOWN_TIMEOUT = 2000  # milliseconds to wait for thread shutdown

# Blocking command handling (seconds)
BLOCKING_COMMAND_MIN_PAUSE = 3.0  # Minimum pause duration after blocking command (G28, etc)
BLOCKING_COMMAND_MAX_PAUSE = 30.0  # Maximum pause - force resume even without "ok" response

# ========== Position Feedback & Validation ==========
# Position limits for each axis (degrees)
POSITION_LIMITS = {
    'X': (-180, 180),   # Art1 base rotation
    'Y': (-180, 180),   # Art2 shoulder (coupled motors)
    'Z': (-180, 180),   # Art3 elbow
    'U': (-180, 180),   # Art4 wrist roll
    'V': (-360, 360),   # Differential motor V
    'W': (-360, 360),   # Differential motor W
}

# Maximum position change per 100ms update (degrees)
# Used to detect encoder errors or impossible movements
MAX_POSITION_CHANGE_PER_UPDATE = 50  # degrees

# GUI update throttling - OPTIMIZED for performance
GUI_UPDATE_INTERVAL = 0.1  # seconds (10Hz max) - faster visual feedback
LOGGING_INTERVAL_POSITIONS = 20  # Log position every Nth update - reduced logging spam

# ========== Sequence Playback ==========
SEQUENCE_TIMER_INTERVAL = 100  # milliseconds between playback updates
SEQUENCE_SPEED_MIN = 0.1
SEQUENCE_SPEED_MAX = 10.0

# ========== GUI Layout (pixels) ==========
# Sequence Programmer panel
SEQUENCE_PANEL_X = 910
SEQUENCE_PANEL_Y = 155
SEQUENCE_PANEL_WIDTH = 280
SEQUENCE_PANEL_HEIGHT = 703

# Endstop Status panel
ENDSTOP_PANEL_X = 910
ENDSTOP_PANEL_Y = 20
ENDSTOP_PANEL_WIDTH = 280
ENDSTOP_PANEL_HEIGHT = 125

# ========== Firmware Types ==========
FIRMWARE_GRBL = "GRBL"
FIRMWARE_RRF = "RRF"

# ========== Logging ==========
LOG_LEVEL = "DEBUG"
LOG_FILE = "bifrost_debug.log"
LOG_FORMAT = '%(asctime)s - %(levelname)s - [%(funcName)s] %(message)s'

# ========== Robot Kinematics ==========
# See inverse_kinematics.py for link lengths (L1, L2, L3, L4)
# These are defined there to keep kinematics self-contained

# ========== Differential Kinematics ==========
DIFFERENTIAL_VALIDATION_TOLERANCE = 0.1  # degrees

# ========== Increment/Decrement Steps ==========
INCREMENT_LARGE = 10.0   # degrees
INCREMENT_MEDIUM = 1.0   # degrees
INCREMENT_SMALL = 0.1    # degrees

# ========== Gripper ==========
GRIPPER_PWM_MAX = 255
GRIPPER_PERCENT_MAX = 100

# ========== Position History ==========
POSITION_HISTORY_MAX_SIZE = 5000  # Maximum snapshots to keep in memory
POSITION_HISTORY_SAMPLE_RATE = 5  # Record every Nth position update (1=all, 5=every 5th)
POSITION_HISTORY_AUTO_SAVE_INTERVAL = 300  # seconds (5 minutes, 0=disabled)
POSITION_HISTORY_PLOT_UPDATE_INTERVAL = 1000  # milliseconds
POSITION_HISTORY_PLOT_WINDOW_SIZE = 100  # Number of recent points to show in real-time plot
