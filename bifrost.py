import sys
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtGui import *
from PyQt5.QtCore import *

from gui import Ui_MainWindow
from about import Ui_Dialog as About_Ui_Dialog


import serial_port_finder as spf
import inverse_kinematics as ik
import sequence_recorder as seq_rec
import differential_kinematics as diff_kin
import position_history as pos_hist
import config

import serial
import time
import json
import threading
import logging
import numpy as np
import re

# Configure logging for debugging using config
logging.basicConfig(
    level=getattr(logging, config.LOG_LEVEL),
    format=config.LOG_FORMAT,
    handlers=[
        logging.FileHandler(config.LOG_FILE),
        logging.StreamHandler(sys.stdout)
    ]
)
logger = logging.getLogger(__name__)

# Firmware type constants (from config)
FIRMWARE_GRBL = config.FIRMWARE_GRBL
FIRMWARE_RRF = config.FIRMWARE_RRF

# Compiled regex patterns for efficient position parsing
# RRF M114 format: X:10.000 Y:10.000 Z:0.000 U:0.028 V:-110.000 W:-290.000
RRF_POSITION_PATTERN = re.compile(r'([XYZUVW]):([+-]?\d+\.?\d*)')

# Endstop status pattern: "Endstops - X: at min stendsop, Y: not stopped, ..."
ENDSTOP_PATTERN = re.compile(r'([XYZUVW]):\s*([^,]+)')

# Thread-safe serial object with command queue
class SerialManager:
    def __init__(self):
        self.serial = serial.Serial()
        self.lock = threading.Lock()
        # Non-blocking command queue (thread-safe)
        self.command_queue = []
        self.queue_lock = threading.Lock()

    def write(self, data, priority=False):
        """
        Queue data to be written by serial thread (non-blocking)

        Args:
            data: Bytes to write
            priority: If True, add to front of queue for immediate sending
        """
        with self.queue_lock:
            if priority:
                # Insert at beginning for immediate sending (status requests, etc)
                self.command_queue.insert(0, data)
            else:
                # Append to end (normal commands)
                self.command_queue.append(data)

    def _write_internal(self, data):
        """
        Internal method: Actually write data to serial port (called by serial thread only)

        Args:
            data: Bytes to write

        Returns:
            True if write succeeded, False otherwise
        """
        with self.lock:
            if self.serial.isOpen():
                try:
                    self.serial.write(data)
                    return True
                except (OSError, serial.SerialException) as e:
                    logger.error(f"Error writing to serial port: {e}")
                    return False
        return False

    def get_next_command(self):
        """
        Get next command from queue (thread-safe, non-blocking)

        Returns:
            Command bytes or None if queue is empty
        """
        with self.queue_lock:
            if len(self.command_queue) > 0:
                return self.command_queue.pop(0)
        return None

    def readline(self):
        with self.lock:
            if self.serial.isOpen():
                return self.serial.readline()
        return b''

    def isOpen(self):
        with self.lock:
            return self.serial.isOpen()

    def open(self):
        with self.lock:
            self.serial.open()

    def close(self):
        with self.lock:
            if self.serial.isOpen():
                self.serial.close()

    def inWaiting(self):
        with self.lock:
            return self.serial.inWaiting()

    def reset_input_buffer(self):
        """Clear the input buffer to prevent stale data"""
        with self.lock:
            if self.serial.isOpen():
                self.serial.reset_input_buffer()

s0 = SerialManager()

class HistoryLineEdit(QtWidgets.QLineEdit):
    """QLineEdit with command history navigation via up/down arrows"""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.history = []
        self.history_position = -1
        self.current_text = ""

    def keyPressEvent(self, event):
        if event.key() == QtCore.Qt.Key_Up:
            # Navigate up through history (older commands)
            if len(self.history) > 0:
                if self.history_position == -1:
                    # Save current text before navigating history
                    self.current_text = self.text()
                    self.history_position = len(self.history) - 1
                elif self.history_position > 0:
                    self.history_position -= 1

                if 0 <= self.history_position < len(self.history):
                    self.setText(self.history[self.history_position])

        elif event.key() == QtCore.Qt.Key_Down:
            # Navigate down through history (newer commands)
            if self.history_position != -1:
                if self.history_position < len(self.history) - 1:
                    self.history_position += 1
                    self.setText(self.history[self.history_position])
                else:
                    # Back to current text
                    self.history_position = -1
                    self.setText(self.current_text)

        else:
            # For any other key, reset history position
            if event.key() not in (QtCore.Qt.Key_Up, QtCore.Qt.Key_Down):
                if self.history_position != -1:
                    self.history_position = -1
            super().keyPressEvent(event)

    def addToHistory(self, command):
        """Add a command to history"""
        if command.strip():  # Don't add empty commands
            # Don't add duplicate consecutive commands
            if not self.history or self.history[-1] != command:
                self.history.append(command)
            self.history_position = -1
            self.current_text = ""

class AboutDialog(About_Ui_Dialog):
    def __init__(self, dialog):
        About_Ui_Dialog.__init__(self)
        self.setupUi(dialog)

class ConnectionSignals(QtCore.QObject):
    """Signals for thread-safe connection callbacks"""
    success = pyqtSignal(str, str)  # serialPort, baudrate
    error = pyqtSignal(str)  # error_msg

class BifrostGUI(Ui_MainWindow):
    def __init__(self, dialog):
        Ui_MainWindow.__init__(self)
        self.setupUi(dialog)

        # Create connection signals (will connect after methods are defined)
        self.connection_signals = ConnectionSignals()

        # Replace ConsoleInput with HistoryLineEdit for command history
        old_console_input = self.ConsoleInput
        self.ConsoleInput = HistoryLineEdit(self.centralwidget)
        self.ConsoleInput.setGeometry(old_console_input.geometry())
        self.ConsoleInput.setObjectName("ConsoleInput")
        old_console_input.setParent(None)
        old_console_input.deleteLater()

        # Set firmware type - Change this to FIRMWARE_GRBL for GRBL firmware
        self.firmware_type = FIRMWARE_RRF

        # Track serial thread state
        self.SerialThreadClass = None

        # Track last manual command time to show responses
        self.last_manual_command_time = 0

        # Track homing state
        self.is_homing = False

        # Log axis mapping configuration
        logger.info("="*60)
        logger.info("BIFROST GUI - MOTOR TO AXIS MAPPING")
        logger.info("="*60)
        logger.info(f"Firmware type: {self.firmware_type}")
        logger.info("GUI Control -> Firmware Axis -> Physical Motor")
        logger.info("-"*60)
        logger.info("Art1 (Joint 1) -> X axis -> Drive 0")
        logger.info("Art2 (Joint 2) -> Y axis -> Drives 1+2 (COUPLED for torque)")
        logger.info("Art3 (Joint 3) -> Z axis -> Drive 3")
        logger.info("Art4 (Joint 4) -> U axis -> Drive 4")
        logger.info("Art5 (Joint 5) -> DIFFERENTIAL -> Drives 5+6")
        logger.info("Art6 (Joint 6) -> DIFFERENTIAL -> Drives 5+6")
        logger.info("  DIFFERENTIAL: Motor_V(D5) = Art6+Art5, Motor_W(D6) = Art6-Art5")
        logger.info("="*60)

        self.getSerialPorts()

        self.actionAbout.triggered.connect(self.launchAboutWindow)
        self.actionExit.triggered.connect(self.close_application)

        # Setup embedded position history graph
        self.setupPositionHistoryControls()

        self.HomeButton.pressed.connect(self.sendHomingCycleCommand)
        self.ZeroPositionButton.pressed.connect(self.sendZeroPositionCommand)
        self.KillAlarmLockButton.pressed.connect(self.sendKillAlarmCommand)

        self.G0MoveRadioButton.clicked.connect(self.FeedRateBoxHide)
        self.G1MoveRadioButton.clicked.connect(self.FeedRateBoxHide)

        self.FKGoButtonArt1.pressed.connect(self.FKMoveArt1)
        self.FKSliderArt1.valueChanged.connect(self.FKSliderUpdateArt1)
        self.SpinBoxArt1.valueChanged.connect(self.FKSpinBoxUpdateArt1)
        self.FKDec10ButtonArt1.pressed.connect(self.FKDec10Art1)
        self.FKDec1ButtonArt1.pressed.connect(self.FKDec1Art1)
        self.FKDec0_1ButtonArt1.pressed.connect(self.FKDec0_1Art1)
        self.FKInc0_1ButtonArt1.pressed.connect(self.FKInc0_1Art1)
        self.FKInc1ButtonArt1.pressed.connect(self.FKInc1Art1)
        self.FKInc10ButtonArt1.pressed.connect(self.FKInc10Art1)

        self.FKGoButtonArt2.pressed.connect(self.FKMoveArt2)
        self.FKSliderArt2.valueChanged.connect(self.FKSliderUpdateArt2)
        self.SpinBoxArt2.valueChanged.connect(self.FKSpinBoxUpdateArt2)
        self.FKDec10ButtonArt2.pressed.connect(self.FKDec10Art2)
        self.FKDec1ButtonArt2.pressed.connect(self.FKDec1Art2)
        self.FKDec0_1ButtonArt2.pressed.connect(self.FKDec0_1Art2)
        self.FKInc0_1ButtonArt2.pressed.connect(self.FKInc0_1Art2)
        self.FKInc1ButtonArt2.pressed.connect(self.FKInc1Art2)
        self.FKInc10ButtonArt2.pressed.connect(self.FKInc10Art2)

        self.FKGoButtonArt3.pressed.connect(self.FKMoveArt3)
        self.FKSliderArt3.valueChanged.connect(self.FKSliderUpdateArt3)
        self.SpinBoxArt3.valueChanged.connect(self.FKSpinBoxUpdateArt3)
        self.FKDec10ButtonArt3.pressed.connect(self.FKDec10Art3)
        self.FKDec1ButtonArt3.pressed.connect(self.FKDec1Art3)
        self.FKDec0_1ButtonArt3.pressed.connect(self.FKDec0_1Art3)
        self.FKInc0_1ButtonArt3.pressed.connect(self.FKInc0_1Art3)
        self.FKInc1ButtonArt3.pressed.connect(self.FKInc1Art3)
        self.FKInc10ButtonArt3.pressed.connect(self.FKInc10Art3)

        self.FKGoButtonArt4.pressed.connect(self.FKMoveArt4)
        self.FKSliderArt4.valueChanged.connect(self.FKSliderUpdateArt4)
        self.SpinBoxArt4.valueChanged.connect(self.FKSpinBoxUpdateArt4)
        self.FKDec10ButtonArt4.pressed.connect(self.FKDec10Art4)
        self.FKDec1ButtonArt4.pressed.connect(self.FKDec1Art4)
        self.FKDec0_1ButtonArt4.pressed.connect(self.FKDec0_1Art4)
        self.FKInc0_1ButtonArt4.pressed.connect(self.FKInc0_1Art4)
        self.FKInc1ButtonArt4.pressed.connect(self.FKInc1Art4)
        self.FKInc10ButtonArt4.pressed.connect(self.FKInc10Art4)

        self.FKGoButtonArt5.pressed.connect(self.FKMoveArt5)
        self.FKSliderArt5.valueChanged.connect(self.FKSliderUpdateArt5)
        self.SpinBoxArt5.valueChanged.connect(self.FKSpinBoxUpdateArt5)
        self.FKDec10ButtonArt5.pressed.connect(self.FKDec10Art5)
        self.FKDec1ButtonArt5.pressed.connect(self.FKDec1Art5)
        self.FKDec0_1ButtonArt5.pressed.connect(self.FKDec0_1Art5)
        self.FKInc0_1ButtonArt5.pressed.connect(self.FKInc0_1Art5)
        self.FKInc1ButtonArt5.pressed.connect(self.FKInc1Art5)
        self.FKInc10ButtonArt5.pressed.connect(self.FKInc10Art5)

        self.FKGoButtonArt6.pressed.connect(self.FKMoveArt6)
        self.FKSliderArt6.valueChanged.connect(self.FKSliderUpdateArt6)
        self.SpinBoxArt6.valueChanged.connect(self.FKSpinBoxUpdateArt6)
        self.FKDec10ButtonArt6.pressed.connect(self.FKDec10Art6)
        self.FKDec1ButtonArt6.pressed.connect(self.FKDec1Art6)
        self.FKDec0_1ButtonArt6.pressed.connect(self.FKDec0_1Art6)
        self.FKInc0_1ButtonArt6.pressed.connect(self.FKInc0_1Art6)
        self.FKInc1ButtonArt6.pressed.connect(self.FKInc1Art6)
        self.FKInc10ButtonArt6.pressed.connect(self.FKInc10Art6)

        self.FKGoAllButton.pressed.connect(self.FKMoveAll)

        self.GoButtonGripper.pressed.connect(self.MoveGripper)
        self.SliderGripper.valueChanged.connect(self.SliderUpdateGripper)
        self.SpinBoxGripper.valueChanged.connect(self.SpinBoxUpdateGripper)
        self.Dec10ButtonGripper.pressed.connect(self.Dec10Gripper)
        self.Dec1ButtonGripper.pressed.connect(self.Dec1Gripper)
        self.Inc1ButtonGripper.pressed.connect(self.Inc1Gripper)
        self.Inc10ButtonGripper.pressed.connect(self.Inc10Gripper)

        self.SerialPortRefreshButton.pressed.connect(self.getSerialPorts)
        self.ConnectButton.pressed.connect(self.connectSerial)

        # Connect console input signals after replacing with HistoryLineEdit
        self.ConsoleButtonSend.pressed.connect(self.sendSerialCommand)
        self.ConsoleInput.returnPressed.connect(self.sendSerialCommand)

        # IK Control connections
        self.IKInputSpinBoxX.valueChanged.connect(self.calculateIK)
        self.IKInputSpinBoxY.valueChanged.connect(self.calculateIK)
        self.IKInputSpinBoxZ.valueChanged.connect(self.calculateIK)
        self.IkIncButtonX.pressed.connect(self.IkIncX)
        self.IkDecButtonX.pressed.connect(self.IkDecX)
        self.IkIncButtonY.pressed.connect(self.IkIncY)
        self.IkDecButtonY.pressed.connect(self.IkDecY)
        self.IkIncButtonZ.pressed.connect(self.IkIncZ)
        self.IkDecButtonZ.pressed.connect(self.IkDecZ)

        # Enable IK controls
        self.InverseKinematicsLabel.setEnabled(True)
        self.IKInputSpinBoxX.setEnabled(True)
        self.IKInputSpinBoxY.setEnabled(True)
        self.IKInputSpinBoxZ.setEnabled(True)
        self.IkOutputValueFrame.setEnabled(True)
        self.IkIncButtonX.setEnabled(True)
        self.IkDecButtonX.setEnabled(True)
        self.IkIncButtonY.setEnabled(True)
        self.IkDecButtonY.setEnabled(True)
        self.IkIncButtonZ.setEnabled(True)
        self.IkDecButtonZ.setEnabled(True)

        # Initialize Sequence Recorder
        self.sequence_recorder = seq_rec.SequenceRecorder()
        self.sequence_player = None  # Will be initialized when needed
        self.is_playing_sequence = False
        self.sequence_timer = QtCore.QTimer()
        self.sequence_timer.timeout.connect(self.updateSequencePlayback)
        self.setupSequenceControls()

        # Track current differential motor positions for Art5/Art6
        # Initialize from spinbox values
        initial_art5 = self.SpinBoxArt5.value()
        initial_art6 = self.SpinBoxArt6.value()
        self.current_motor_v = initial_art6 + initial_art5
        self.current_motor_w = initial_art6 - initial_art5

        # Track desired joint positions for differential control
        self.desired_art5 = initial_art5
        self.desired_art6 = initial_art6

        # Position validation tracking
        self.last_valid_positions = {'X': 0, 'Y': 0, 'Z': 0, 'U': 0, 'V': 0, 'W': 0}
        self.position_update_count = 0
        self.last_gui_update_time = 0  # For throttling GUI updates

        # Position history tracking
        self.position_history = pos_hist.PositionHistory(max_size=config.POSITION_HISTORY_MAX_SIZE)
        logger.info(f"Position history initialized (max_size={config.POSITION_HISTORY_MAX_SIZE}, sample_rate=1/{config.POSITION_HISTORY_SAMPLE_RATE})")

        # Setup endstop status displays
        self.setupEndstopDisplays()

        # Setup generic increment/decrement connections
        self.setupGenericControls()

        # Connect connection signals now that methods are defined
        self.connection_signals.success.connect(self._onConnectionSuccess)
        self.connection_signals.error.connect(self._onConnectionError)

    def setupGenericControls(self):
        """
        Setup generic increment/decrement methods for all joints
        This replaces 36 individual methods with dynamic binding
        """
        # Map joint names to their spinboxes
        self.joint_spinboxes = {
            'Art1': self.SpinBoxArt1,
            'Art2': self.SpinBoxArt2,
            'Art3': self.SpinBoxArt3,
            'Art4': self.SpinBoxArt4,
            'Art5': self.SpinBoxArt5,
            'Art6': self.SpinBoxArt6,
            'Gripper': self.SpinBoxGripper
        }

        logger.info("Generic increment/decrement controls initialized")

    def adjustJointValue(self, joint_name, delta):
        """
        Generic method to adjust any joint value by a delta

        Args:
            joint_name: Name of joint ('Art1', 'Art2', etc.)
            delta: Amount to add to current value
        """
        if joint_name in self.joint_spinboxes:
            spinbox = self.joint_spinboxes[joint_name]
            new_value = spinbox.value() + delta
            spinbox.setValue(new_value)
        else:
            logger.warning(f"Unknown joint name: {joint_name}")

    def close_application(self):
        # Properly cleanup serial connection and thread
        if self.SerialThreadClass and self.SerialThreadClass.isRunning():
            self.SerialThreadClass.stop()
            self.SerialThreadClass.wait(config.SERIAL_THREAD_SHUTDOWN_TIMEOUT)
        s0.close()
        sys.exit()

    def launchAboutWindow(self):
        self.dialogAbout = QtWidgets.QDialog()
        self.ui = AboutDialog(self.dialogAbout)
        self.dialogAbout.exec_()

    def sendHomingCycleCommand(self):
        if s0.isOpen():
            if self.firmware_type == FIRMWARE_RRF:
                messageToSend="G28\n"  # RRF: Home all axes
            else:  # GRBL
                messageToSend="$H\n"
            messageToConsole=">>> " + messageToSend.strip()
            s0.write(messageToSend.encode('UTF-8'))
            self.ConsoleOutput.appendPlainText(messageToConsole)

            # Update button state to indicate homing in progress
            self.is_homing = True
            self.HomeButton.setEnabled(False)
            self.HomeButton.setText("Homing...")

    def sendZeroPositionCommand(self):
        if s0.isOpen():
            messageToSend="G0 X0 Y0 Z0 U0 V0 W0"
            messageToConsole=">>> " + messageToSend
            s0.write(messageToSend.encode('UTF-8'))
            self.ConsoleOutput.appendPlainText(messageToConsole)

    def sendKillAlarmCommand(self):
        if s0.isOpen():
            if self.firmware_type == FIRMWARE_RRF:
                messageToSend="M999\n"  # RRF: Clear emergency stop / reset
            else:  # GRBL
                messageToSend="$X\n"
            messageToConsole=">>> " + messageToSend.strip()
            s0.write(messageToSend.encode('UTF-8'))
            self.ConsoleOutput.appendPlainText(messageToConsole)

    def FeedRateBoxHide(self):
        if self.G1MoveRadioButton.isChecked():
            self.FeedRateLabel.setEnabled(True)
            self.FeedRateInput.setEnabled(True)
        else:
            self.FeedRateLabel.setEnabled(False)
            self.FeedRateInput.setEnabled(False)


#FK Art1 Functions
    def FKMoveArt1(self):
        joint_value = self.SpinBoxArt1.value()
        logger.info(f"Art1 (Joint 1) commanded to: {joint_value}° -> Axis: W")
        if s0.isOpen():
            if self.G1MoveRadioButton.isChecked():
                typeOfMovement="G1 "
                feedRate=" F" + str(self.FeedRateInput.value())
            else:
                typeOfMovement="G0 "
                feedRate=""
            message=typeOfMovement + "X" + str(joint_value) + feedRate
            messageToSend = message + "\n"
            messageToConsole = ">>> " + message
            logger.debug(f"Sending command: {message.strip()}")
            s0.write(messageToSend.encode('UTF-8'))
            self.ConsoleOutput.appendPlainText(messageToConsole)
        else:
            logger.warning("Art1 move attempted but serial not connected")
            self.noSerialConnection()
    def FKSliderUpdateArt1(self):
        val=self.FKSliderArt1.value()/10
        self.SpinBoxArt1.setValue(val)
    def FKSpinBoxUpdateArt1(self):
        val=int(self.SpinBoxArt1.value()*10)
        self.FKSliderArt1.setValue(val)
    def FKDec10Art1(self):
        self.adjustJointValue('Art1', -10)
    def FKDec1Art1(self):
        self.adjustJointValue('Art1', -1)
    def FKDec0_1Art1(self):
        self.adjustJointValue('Art1', -0.1)
    def FKInc0_1Art1(self):
        self.adjustJointValue('Art1', 0.1)
    def FKInc1Art1(self):
        self.adjustJointValue('Art1', 1)
    def FKInc10Art1(self):
        self.adjustJointValue('Art1', 10)

#FK Art2 Functions
    def FKMoveArt2(self):
        # COUPLED MOTORS: Art2 uses Drives 1+2 (Y axis) for more torque
        # Y axis controls both drives in firmware (M584 Y1:2)
        joint_value = self.SpinBoxArt2.value()
        logger.info(f"Art2 (Joint 2 - COUPLED) commanded to: {joint_value}° -> Axis: Y (Drives 1+2)")
        if s0.isOpen():
            if self.G1MoveRadioButton.isChecked():
                typeOfMovement="G1 "
                feedRate=" F" + str(self.FeedRateInput.value())
            else:
                typeOfMovement="G0 "
                feedRate=""
            # Y axis controls both drives 1+2 (coupled in firmware)
            message=typeOfMovement + "Y" + str(joint_value) + feedRate
            messageToSend = message + "\n"
            messageToConsole = ">>> " + message
            logger.debug(f"Sending coupled command: {message.strip()}")
            s0.write(messageToSend.encode('UTF-8'))
            self.ConsoleOutput.appendPlainText(messageToConsole)
        else:
            logger.warning("Art2 move attempted but serial not connected")
            self.noSerialConnection()
    def FKSliderUpdateArt2(self):
        val=self.FKSliderArt2.value()/10
        self.SpinBoxArt2.setValue(val)
    def FKSpinBoxUpdateArt2(self):
        val=int(self.SpinBoxArt2.value()*10)
        self.FKSliderArt2.setValue(val)
    def FKDec10Art2(self):
        self.adjustJointValue('Art2', -10)
    def FKDec1Art2(self):
        self.adjustJointValue('Art2', -1)
    def FKDec0_1Art2(self):
        self.adjustJointValue('Art2', -0.1)
    def FKInc0_1Art2(self):
        self.adjustJointValue('Art2', 0.1)
    def FKInc1Art2(self):
        self.adjustJointValue('Art2', 1)
    def FKInc10Art2(self):
        self.adjustJointValue('Art2', 10)

#FK Art3 Functions
    def FKMoveArt3(self):
        joint_value = self.SpinBoxArt3.value()
        logger.info(f"Art3 (Joint 3) commanded to: {joint_value}° -> Axis: Z")
        if s0.isOpen():
            if self.G1MoveRadioButton.isChecked():
                typeOfMovement="G1 "
                feedRate=" F" + str(self.FeedRateInput.value())
            else:
                typeOfMovement="G0 "
                feedRate=""
            message=typeOfMovement + "Z" + str(joint_value) + feedRate
            messageToSend = message + "\n"
            messageToConsole = ">>> " + message
            logger.debug(f"Sending command: {message.strip()}")
            s0.write(messageToSend.encode('UTF-8'))
            self.ConsoleOutput.appendPlainText(messageToConsole)
        else:
            logger.warning("Art3 move attempted but serial not connected")
            self.noSerialConnection()
    def FKSliderUpdateArt3(self):
        val=self.FKSliderArt3.value()/10
        self.SpinBoxArt3.setValue(val)
    def FKSpinBoxUpdateArt3(self):
        val=int(self.SpinBoxArt3.value()*10)
        self.FKSliderArt3.setValue(val)
    def FKDec10Art3(self):
        self.adjustJointValue('Art3', -10)
    def FKDec1Art3(self):
        self.adjustJointValue('Art3', -1)
    def FKDec0_1Art3(self):
        self.adjustJointValue('Art3', -0.1)
    def FKInc0_1Art3(self):
        self.adjustJointValue('Art3', 0.1)
    def FKInc1Art3(self):
        self.adjustJointValue('Art3', 1)
    def FKInc10Art3(self):
        self.adjustJointValue('Art3', 10)

#FK Art4 Functions
    def FKMoveArt4(self):
        joint_value = self.SpinBoxArt4.value()
        logger.info(f"Art4 (Joint 4) commanded to: {joint_value}° -> Axis: U")
        if s0.isOpen():
            if self.G1MoveRadioButton.isChecked():
                typeOfMovement="G1 "
                feedRate=" F" + str(self.FeedRateInput.value())
            else:
                typeOfMovement="G0 "
                feedRate=""
            message=typeOfMovement + "U" + str(joint_value) + feedRate
            messageToSend = message + "\n"
            messageToConsole = ">>> " + message
            logger.debug(f"Sending command: {message.strip()}")
            s0.write(messageToSend.encode('UTF-8'))
            self.ConsoleOutput.appendPlainText(messageToConsole)
        else:
            logger.warning("Art4 move attempted but serial not connected")
            self.noSerialConnection()
    def FKSliderUpdateArt4(self):
        val=self.FKSliderArt4.value()/10
        self.SpinBoxArt4.setValue(val)
    def FKSpinBoxUpdateArt4(self):
        val=int(self.SpinBoxArt4.value()*10)
        self.FKSliderArt4.setValue(val)
    def FKDec10Art4(self):
        self.adjustJointValue('Art4', -10)
    def FKDec1Art4(self):
        self.adjustJointValue('Art4', -1)
    def FKDec0_1Art4(self):
        self.adjustJointValue('Art4', -0.1)
    def FKInc0_1Art4(self):
        self.adjustJointValue('Art4', 0.1)
    def FKInc1Art4(self):
        self.adjustJointValue('Art4', 1)
    def FKInc10Art4(self):
        self.adjustJointValue('Art4', 10)

#FK Art5 Functions
    def FKMoveArt5(self):
        # DIFFERENTIAL MECHANISM: Art5 & Art6 use differential kinematics
        art5_value = self.SpinBoxArt5.value()

        # Check if we have valid position feedback
        if self.current_motor_v == 0.0 and self.current_motor_w == 0.0:
            logger.warning("No position feedback received yet - differential control may be inaccurate!")
            logger.warning("Wait for position update or home the robot first")

        # Calculate new motor positions using differential kinematics helper
        motor_v, motor_w, art6_kept = diff_kin.DifferentialKinematics.move_art5_only(
            self.current_motor_v,
            self.current_motor_w,
            art5_value
        )

        current_art5, current_art6 = diff_kin.DifferentialKinematics.motor_to_joint(
            self.current_motor_v, self.current_motor_w
        )

        logger.info(f"Art5 (DIFFERENTIAL) commanded to: {art5_value}°")
        logger.info(f"  BEFORE: Motor_V={self.current_motor_v:.2f}° Motor_W={self.current_motor_w:.2f}° → Art5={current_art5:.2f}° Art6={current_art6:.2f}°")
        logger.info(f"  AFTER:  Motor_V={motor_v:.2f}° Motor_W={motor_w:.2f}° → Art5={art5_value:.2f}° Art6={art6_kept:.2f}° (Art6 kept)")

        # Update tracked positions
        self.current_motor_v = motor_v
        self.current_motor_w = motor_w
        self.desired_art5 = art5_value

        if s0.isOpen():
            if self.G1MoveRadioButton.isChecked():
                typeOfMovement="G1 "
                feedRate=" F" + str(self.FeedRateInput.value())
            else:
                typeOfMovement="G0 "
                feedRate=""
            # Send both motor commands for differential
            message=typeOfMovement + "V" + str(motor_v) + " W" + str(motor_w) + feedRate
            messageToSend = message + "\n"
            messageToConsole = ">>> " + message
            logger.debug(f"Sending differential command: {message.strip()}")
            s0.write(messageToSend.encode('UTF-8'))
            self.ConsoleOutput.appendPlainText(messageToConsole)
        else:
            logger.warning("Art5 move attempted but serial not connected")
            self.noSerialConnection()
    def FKSliderUpdateArt5(self):
        val=self.FKSliderArt5.value()/10
        self.SpinBoxArt5.setValue(val)
    def FKSpinBoxUpdateArt5(self):
        val=int(self.SpinBoxArt5.value()*10)
        self.FKSliderArt5.setValue(val)
    def FKDec10Art5(self):
        self.adjustJointValue('Art5', -10)
    def FKDec1Art5(self):
        self.adjustJointValue('Art5', -1)
    def FKDec0_1Art5(self):
        self.adjustJointValue('Art5', -0.1)
    def FKInc0_1Art5(self):
        self.adjustJointValue('Art5', 0.1)
    def FKInc1Art5(self):
        self.adjustJointValue('Art5', 1)
    def FKInc10Art5(self):
        self.adjustJointValue('Art5', 10)

#FK Art6 Functions
    def FKMoveArt6(self):
        # DIFFERENTIAL MECHANISM: Art5 & Art6 use differential kinematics
        art6_value = self.SpinBoxArt6.value()

        # Check if we have valid position feedback
        if self.current_motor_v == 0.0 and self.current_motor_w == 0.0:
            logger.warning("No position feedback received yet - differential control may be inaccurate!")
            logger.warning("Wait for position update or home the robot first")

        # Calculate new motor positions using differential kinematics helper
        motor_v, motor_w, art5_kept = diff_kin.DifferentialKinematics.move_art6_only(
            self.current_motor_v,
            self.current_motor_w,
            art6_value
        )

        current_art5, current_art6 = diff_kin.DifferentialKinematics.motor_to_joint(
            self.current_motor_v, self.current_motor_w
        )

        logger.info(f"Art6 (DIFFERENTIAL) commanded to: {art6_value}°")
        logger.info(f"  BEFORE: Motor_V={self.current_motor_v:.2f}° Motor_W={self.current_motor_w:.2f}° → Art5={current_art5:.2f}° Art6={current_art6:.2f}°")
        logger.info(f"  AFTER:  Motor_V={motor_v:.2f}° Motor_W={motor_w:.2f}° → Art5={art5_kept:.2f}° Art6={art6_value:.2f}° (Art5 kept)")

        # Update tracked positions
        self.current_motor_v = motor_v
        self.current_motor_w = motor_w
        self.desired_art6 = art6_value

        if s0.isOpen():
            if self.G1MoveRadioButton.isChecked():
                typeOfMovement="G1 "
                feedRate=" F" + str(self.FeedRateInput.value())
            else:
                typeOfMovement="G0 "
                feedRate=""
            # Send both motor commands for differential
            message=typeOfMovement + "V" + str(motor_v) + " W" + str(motor_w) + feedRate
            messageToSend = message + "\n"
            messageToConsole = ">>> " + message
            logger.debug(f"Sending differential command: {message.strip()}")
            s0.write(messageToSend.encode('UTF-8'))
            self.ConsoleOutput.appendPlainText(messageToConsole)
        else:
            logger.warning("Art6 move attempted but serial not connected")
            self.noSerialConnection()
    def FKSliderUpdateArt6(self):
        val=self.FKSliderArt6.value()/10
        self.SpinBoxArt6.setValue(val)
    def FKSpinBoxUpdateArt6(self):
        val=int(self.SpinBoxArt6.value()*10)
        self.FKSliderArt6.setValue(val)
    def FKDec10Art6(self):
        self.adjustJointValue('Art6', -10)
    def FKDec1Art6(self):
        self.adjustJointValue('Art6', -1)
    def FKDec0_1Art6(self):
        self.adjustJointValue('Art6', -0.1)
    def FKInc0_1Art6(self):
        self.adjustJointValue('Art6', 0.1)
    def FKInc1Art6(self):
        self.adjustJointValue('Art6', 1)
    def FKInc10Art6(self):
        self.adjustJointValue('Art6', 10)

#FK Every Articulation Functions
    def FKMoveAll(self):
        # DIFFERENTIAL MECHANISM: Art5 & Art6 use differential kinematics
        if s0.isOpen():
            # Get joint values
            art1 = self.SpinBoxArt1.value()
            art2 = self.SpinBoxArt2.value()
            art3 = self.SpinBoxArt3.value()
            art4 = self.SpinBoxArt4.value()
            art5 = self.SpinBoxArt5.value()
            art6 = self.SpinBoxArt6.value()

            # Calculate differential motor positions using helper
            motor_v, motor_w = diff_kin.DifferentialKinematics.joint_to_motor(art5, art6)

            logger.info(f"MoveAll: Art5={art5}° Art6={art6}° → Differential: V={motor_v:.2f}° W={motor_w:.2f}°")

            if self.G1MoveRadioButton.isChecked():
                typeOfMovement="G1 "
                feedRate=" F" + str(self.FeedRateInput.value())
            else:
                typeOfMovement="G0 "
                feedRate=""

            # Map all axes: X=Art1, Y=Art2(coupled), Z=Art3, U=Art4, V/W=differential(Art5,Art6)
            message=typeOfMovement + "X" + str(art1) + " Y" + str(art2) + " Z" + str(art3) + " U" + str(art4) + " V" + str(motor_v) + " W" + str(motor_w) + feedRate
            messageToSend = message + "\n"
            messageToConsole = ">>> " + message
            s0.write(messageToSend.encode('UTF-8'))
            self.ConsoleOutput.appendPlainText(messageToConsole)
        else:
            self.noSerialConnection()

# Gripper Functions
    def MoveGripper(self):
        if s0.isOpen():
            message="M3 S" + str((255/100)*self.SpinBoxGripper.value())
            messageToSend = message + "\n"
            messageToConsole = ">>> " + message
            s0.write(messageToSend.encode('UTF-8'))
            self.ConsoleOutput.appendPlainText(messageToConsole)
        else:
            self.noSerialConnection()

    def SliderUpdateGripper(self):
        val=self.SliderGripper.value()
        self.SpinBoxGripper.setValue(val)
    def SpinBoxUpdateGripper(self):
        val=int(self.SpinBoxGripper.value())
        self.SliderGripper.setValue(val)
    def Dec10Gripper(self):
        self.adjustJointValue('Gripper', -10)
    def Dec1Gripper(self):
        self.adjustJointValue('Gripper', -1)
    def Inc1Gripper(self):
        self.adjustJointValue('Gripper', 1)
    def Inc10Gripper(self):
        self.adjustJointValue('Gripper', 10)

# Inverse Kinematics Functions
    def calculateIK(self):
        """Calculate 6-DOF inverse kinematics for current target position"""
        x = self.IKInputSpinBoxX.value()
        y = self.IKInputSpinBoxY.value()
        z = self.IKInputSpinBoxZ.value()

        logger.info(f"IK 6-DOF: Calculating for target X={x}, Y={y}, Z={z}")

        # Solve full 6-DOF IK with default tool-down orientation
        # Future enhancement: add orientation input controls
        solution = ik.solve_ik_full(x, y, z, roll=0, pitch=-np.pi/2, yaw=0)

        # Update output displays
        if solution.valid:
            self.IkOutputValueX.setText(f"{solution.q1:.2f}º")
            self.IkOutputValueY.setText(f"{solution.q2:.2f}º")
            self.IkOutputValueZ.setText(f"{solution.q3:.2f}º")

            # Update FK spinboxes with all 6 calculated joint angles
            self.SpinBoxArt1.setValue(solution.q1)
            self.SpinBoxArt2.setValue(solution.q2)
            self.SpinBoxArt3.setValue(solution.q3)
            self.SpinBoxArt4.setValue(solution.q4)
            self.SpinBoxArt5.setValue(solution.q5)
            self.SpinBoxArt6.setValue(solution.q6)

            # Style valid solution
            self.IkOutputValueFrame.setStyleSheet("background-color:rgb(200, 255, 200)")  # Light green
            logger.info(f"IK 6-DOF: Valid solution - q1={solution.q1:.2f}°, q2={solution.q2:.2f}°, q3={solution.q3:.2f}°, q4={solution.q4:.2f}°, q5={solution.q5:.2f}°, q6={solution.q6:.2f}°")
        else:
            self.IkOutputValueX.setText("--")
            self.IkOutputValueY.setText("--")
            self.IkOutputValueZ.setText("--")

            # Style invalid solution
            self.IkOutputValueFrame.setStyleSheet("background-color:rgb(255, 200, 200)")  # Light red
            logger.warning(f"IK 6-DOF: Invalid solution - {solution.error_msg}")

    def IkIncX(self):
        val = self.IKInputSpinBoxX.value() + 10
        self.IKInputSpinBoxX.setValue(val)

    def IkDecX(self):
        val = self.IKInputSpinBoxX.value() - 10
        self.IKInputSpinBoxX.setValue(val)

    def IkIncY(self):
        val = self.IKInputSpinBoxY.value() + 10
        self.IKInputSpinBoxY.setValue(val)

    def IkDecY(self):
        val = self.IKInputSpinBoxY.value() - 10
        self.IKInputSpinBoxY.setValue(val)

    def IkIncZ(self):
        val = self.IKInputSpinBoxZ.value() + 10
        self.IKInputSpinBoxZ.setValue(val)

    def IkDecZ(self):
        val = self.IKInputSpinBoxZ.value() - 10
        self.IKInputSpinBoxZ.setValue(val)

# Sequence Recorder Functions
    def setupSequenceControls(self):
        """Create sequence recorder GUI controls programmatically"""
        # Create group box for sequence controls
        self.sequenceGroupBox = QtWidgets.QGroupBox(self.centralwidget)
        self.sequenceGroupBox.setGeometry(QtCore.QRect(910, 20, 280, 838))
        self.sequenceGroupBox.setTitle("Sequence Programmer")

        # List widget for sequence points (expanded to fill space, aligned with console send button)
        self.sequencePointsList = QtWidgets.QListWidget(self.sequenceGroupBox)
        self.sequencePointsList.setGeometry(QtCore.QRect(10, 25, 260, 555))

        # Record controls
        self.sequenceRecordButton = QtWidgets.QPushButton(self.sequenceGroupBox)
        self.sequenceRecordButton.setGeometry(QtCore.QRect(10, 590, 120, 30))
        self.sequenceRecordButton.setText("Record Point")
        self.sequenceRecordButton.pressed.connect(self.recordSequencePoint)

        self.sequenceDeleteButton = QtWidgets.QPushButton(self.sequenceGroupBox)
        self.sequenceDeleteButton.setGeometry(QtCore.QRect(150, 590, 120, 30))
        self.sequenceDeleteButton.setText("Delete Point")
        self.sequenceDeleteButton.pressed.connect(self.deleteSequencePoint)

        self.sequenceClearButton = QtWidgets.QPushButton(self.sequenceGroupBox)
        self.sequenceClearButton.setGeometry(QtCore.QRect(10, 625, 260, 30))
        self.sequenceClearButton.setText("Clear All")
        self.sequenceClearButton.pressed.connect(self.clearSequence)

        # Playback controls
        self.sequencePlayButton = QtWidgets.QPushButton(self.sequenceGroupBox)
        self.sequencePlayButton.setGeometry(QtCore.QRect(10, 665, 80, 30))
        self.sequencePlayButton.setText("Play")
        self.sequencePlayButton.pressed.connect(self.playSequence)

        self.sequencePauseButton = QtWidgets.QPushButton(self.sequenceGroupBox)
        self.sequencePauseButton.setGeometry(QtCore.QRect(100, 665, 80, 30))
        self.sequencePauseButton.setText("Pause")
        self.sequencePauseButton.setEnabled(False)
        self.sequencePauseButton.pressed.connect(self.pauseSequence)

        self.sequenceStopButton = QtWidgets.QPushButton(self.sequenceGroupBox)
        self.sequenceStopButton.setGeometry(QtCore.QRect(190, 665, 80, 30))
        self.sequenceStopButton.setText("Stop")
        self.sequenceStopButton.setEnabled(False)
        self.sequenceStopButton.pressed.connect(self.stopSequence)

        # Speed control
        speedLabel = QtWidgets.QLabel(self.sequenceGroupBox)
        speedLabel.setGeometry(QtCore.QRect(10, 705, 50, 20))
        speedLabel.setText("Speed:")

        self.sequenceSpeedSpinBox = QtWidgets.QDoubleSpinBox(self.sequenceGroupBox)
        self.sequenceSpeedSpinBox.setGeometry(QtCore.QRect(60, 705, 80, 22))
        self.sequenceSpeedSpinBox.setMinimum(0.1)
        self.sequenceSpeedSpinBox.setMaximum(10.0)
        self.sequenceSpeedSpinBox.setSingleStep(0.1)
        self.sequenceSpeedSpinBox.setValue(1.0)
        self.sequenceSpeedSpinBox.setSuffix("x")

        self.sequenceLoopCheckBox = QtWidgets.QCheckBox(self.sequenceGroupBox)
        self.sequenceLoopCheckBox.setGeometry(QtCore.QRect(150, 705, 60, 20))
        self.sequenceLoopCheckBox.setText("Loop")

        # Delay control
        delayLabel = QtWidgets.QLabel(self.sequenceGroupBox)
        delayLabel.setGeometry(QtCore.QRect(10, 735, 50, 20))
        delayLabel.setText("Delay:")

        self.sequenceDelaySpinBox = QtWidgets.QDoubleSpinBox(self.sequenceGroupBox)
        self.sequenceDelaySpinBox.setGeometry(QtCore.QRect(60, 735, 80, 22))
        self.sequenceDelaySpinBox.setMinimum(0.0)
        self.sequenceDelaySpinBox.setMaximum(60.0)
        self.sequenceDelaySpinBox.setSingleStep(0.1)
        self.sequenceDelaySpinBox.setValue(1.0)
        self.sequenceDelaySpinBox.setSuffix("s")

        # File buttons
        self.sequenceSaveButton = QtWidgets.QPushButton(self.sequenceGroupBox)
        self.sequenceSaveButton.setGeometry(QtCore.QRect(10, 770, 260, 30))
        self.sequenceSaveButton.setText("Save Sequence")
        self.sequenceSaveButton.pressed.connect(self.saveSequence)

        self.sequenceLoadButton = QtWidgets.QPushButton(self.sequenceGroupBox)
        self.sequenceLoadButton.setGeometry(QtCore.QRect(10, 805, 260, 30))
        self.sequenceLoadButton.setText("Load Sequence")
        self.sequenceLoadButton.pressed.connect(self.loadSequence)

        logger.info("Sequence recorder controls initialized")

    def setupEndstopDisplays(self):
        """Initialize references to endstop labels (now defined in gui.py)"""
        # Endstop labels are now inline with articulation controls in gui.py
        # Map old variable names to new ones for backward compatibility
        self.endstopLabelX = self.endstopLabelArt1
        self.endstopLabelY = self.endstopLabelArt2
        self.endstopLabelZ = self.endstopLabelArt3
        self.endstopLabelU = self.endstopLabelArt4
        self.endstopLabelV = self.endstopLabelArt5
        self.endstopLabelW = self.endstopLabelArt6

        logger.info("Endstop status displays initialized (inline with articulation controls)")

    def setupPositionHistoryControls(self):
        """Create embedded 3D robot visualization and controls"""
        from robot_3d_visualizer import Robot3DCanvas

        # Create group box for embedded 3D visualization (right side of window)
        self.positionHistoryGroupBox = QtWidgets.QGroupBox(self.centralwidget)
        self.positionHistoryGroupBox.setGeometry(QtCore.QRect(1210, 10, 600, 900))
        self.positionHistoryGroupBox.setTitle("3D Robot Visualization")

        # Embed 3D matplotlib canvas
        self.position_canvas = Robot3DCanvas(self.positionHistoryGroupBox, width=5.8, height=6.5, dpi=100)
        self.position_canvas.setGeometry(QtCore.QRect(10, 25, 580, 650))

        # Controls panel below 3D visualization
        self.historyControlsFrame = QtWidgets.QFrame(self.positionHistoryGroupBox)
        self.historyControlsFrame.setGeometry(QtCore.QRect(10, 685, 580, 200))
        self.historyControlsFrame.setFrameShape(QtWidgets.QFrame.StyledPanel)

        # Time window control
        self.timeWindowLabel = QtWidgets.QLabel(self.historyControlsFrame)
        self.timeWindowLabel.setGeometry(QtCore.QRect(10, 10, 120, 20))
        self.timeWindowLabel.setText("Time Window (s):")

        self.timeWindowSpinBox = QtWidgets.QSpinBox(self.historyControlsFrame)
        self.timeWindowSpinBox.setGeometry(QtCore.QRect(135, 8, 80, 25))
        self.timeWindowSpinBox.setMinimum(10)
        self.timeWindowSpinBox.setMaximum(600)
        self.timeWindowSpinBox.setValue(60)
        self.timeWindowSpinBox.setSingleStep(10)

        # Display option checkboxes (3 rows of 2)
        checkbox_y = 40
        self.displayCheckboxes = {}
        display_options = [
            ('show_robot', 'Show Robot Arm'),
            ('show_trajectory', 'Show Trajectory'),
            ('show_base_frame', 'Show Base Frame'),
            ('show_workspace', 'Show Workspace'),
            ('show_grid', 'Show Grid Floor'),
            ('auto_rotate', 'Auto-rotate View')
        ]

        for i, (key, label) in enumerate(display_options):
            row = i // 2
            col = i % 2
            x = 10 + col * 290
            y = checkbox_y + row * 30

            checkbox = QtWidgets.QCheckBox(self.historyControlsFrame)
            checkbox.setGeometry(QtCore.QRect(x, y, 280, 25))
            checkbox.setText(label)
            # Default checked states
            checkbox.setChecked(key in ['show_robot', 'show_trajectory', 'show_base_frame', 'show_grid'])
            self.displayCheckboxes[key] = checkbox

        # Action buttons (2 rows)
        button_y = 140
        self.exportHistoryButton = QtWidgets.QPushButton(self.historyControlsFrame)
        self.exportHistoryButton.setGeometry(QtCore.QRect(10, button_y, 135, 30))
        self.exportHistoryButton.setText("Export CSV")
        self.exportHistoryButton.pressed.connect(self.exportPositionHistory)

        self.clearHistoryButton = QtWidgets.QPushButton(self.historyControlsFrame)
        self.clearHistoryButton.setGeometry(QtCore.QRect(155, button_y, 135, 30))
        self.clearHistoryButton.setText("Clear History")
        self.clearHistoryButton.pressed.connect(self.clearPositionHistory)

        self.resetViewButton = QtWidgets.QPushButton(self.historyControlsFrame)
        self.resetViewButton.setGeometry(QtCore.QRect(300, button_y, 135, 30))
        self.resetViewButton.setText("Reset View")
        self.resetViewButton.pressed.connect(self.resetVisualizationView)

        self.refreshGraphButton = QtWidgets.QPushButton(self.historyControlsFrame)
        self.refreshGraphButton.setGeometry(QtCore.QRect(445, button_y, 125, 30))
        self.refreshGraphButton.setText("Refresh")
        self.refreshGraphButton.pressed.connect(self.updateEmbeddedGraph)

        # Make the group box visible
        self.positionHistoryGroupBox.show()

        # Start auto-update timer for 3D visualization
        # OPTIMIZED: 2 second intervals - matplotlib rendering is expensive
        # Dirty flag pattern in visualizer prevents unnecessary redraws
        self.graph_update_timer = QtCore.QTimer()
        self.graph_update_timer.timeout.connect(self.updateEmbeddedGraph)
        self.graph_update_timer.start(2000)  # 2 second intervals (optimized from 1s)

        logger.info("Embedded 3D robot visualization initialized (2s update interval)")

    def updateEmbeddedGraph(self):
        """Update the embedded 3D robot visualization"""
        if not hasattr(self, 'position_canvas'):
            return

        # Get time window from spinbox
        window_size = self.timeWindowSpinBox.value()

        # Get display options from checkboxes
        options = {
            key: checkbox.isChecked()
            for key, checkbox in self.displayCheckboxes.items()
        }

        # Update the 3D visualization
        self.position_canvas.update_visualization(self.position_history, window_size, options)

    def resetVisualizationView(self):
        """Reset 3D visualization view to default isometric angle"""
        if hasattr(self, 'position_canvas'):
            self.position_canvas.reset_view()
            logger.info("3D view reset to isometric")

    def exportPositionHistory(self):
        """Export position history to CSV"""
        if len(self.position_history) == 0:
            msgBox = QtWidgets.QMessageBox()
            msgBox.setIcon(QtWidgets.QMessageBox.Warning)
            msgBox.setText("No position history to export.")
            msgBox.setWindowTitle("No Data")
            msgBox.exec_()
            return

        from datetime import datetime
        default_filename = f"position_history_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"

        filename, _ = QtWidgets.QFileDialog.getSaveFileName(
            None,
            "Export Position History",
            default_filename,
            "CSV Files (*.csv);;All Files (*)"
        )

        if filename:
            if self.position_history.export_to_csv(filename):
                logger.info(f"Position history exported to {filename}")
                msgBox = QtWidgets.QMessageBox()
                msgBox.setIcon(QtWidgets.QMessageBox.Information)
                msgBox.setText(f"Position history exported successfully to:\n{filename}\n\n{len(self.position_history)} snapshots saved.")
                msgBox.setWindowTitle("Export Successful")
                msgBox.exec_()
            else:
                msgBox = QtWidgets.QMessageBox()
                msgBox.setIcon(QtWidgets.QMessageBox.Critical)
                msgBox.setText("Failed to export position history.")
                msgBox.setWindowTitle("Export Failed")
                msgBox.exec_()

    def clearPositionHistory(self):
        """Clear position history after confirmation"""
        msgBox = QtWidgets.QMessageBox()
        msgBox.setIcon(QtWidgets.QMessageBox.Question)
        msgBox.setText(f"Clear all position history?\n\nThis will delete {len(self.position_history)} recorded snapshots.")
        msgBox.setWindowTitle("Clear History")
        msgBox.setStandardButtons(QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No)
        msgBox.setDefaultButton(QtWidgets.QMessageBox.No)

        if msgBox.exec_() == QtWidgets.QMessageBox.Yes:
            self.position_history.clear()
            logger.info("Position history cleared by user")

            infoBox = QtWidgets.QMessageBox()
            infoBox.setIcon(QtWidgets.QMessageBox.Information)
            infoBox.setText("Position history cleared.")
            infoBox.setWindowTitle("Cleared")
            infoBox.exec_()

    def recordSequencePoint(self):
        """Record current joint positions to sequence"""
        q1 = self.SpinBoxArt1.value()
        q2 = self.SpinBoxArt2.value()
        q3 = self.SpinBoxArt3.value()
        q4 = self.SpinBoxArt4.value()
        q5 = self.SpinBoxArt5.value()
        q6 = self.SpinBoxArt6.value()
        gripper = self.SpinBoxGripper.value()
        delay = self.sequenceDelaySpinBox.value()

        if not self.sequence_recorder.is_recording:
            self.sequence_recorder.start_recording("Current Sequence")

        self.sequence_recorder.record_point(q1, q2, q3, q4, q5, q6, gripper, delay)

        # Update list display
        point_text = f"Point {len(self.sequence_recorder.current_sequence)}: q1={q1:.1f}° q2={q2:.1f}° q3={q3:.1f}° delay={delay:.1f}s"
        self.sequencePointsList.addItem(point_text)

        logger.info(f"Recorded point: {point_text}")

    def deleteSequencePoint(self):
        """Delete selected point from sequence"""
        current_row = self.sequencePointsList.currentRow()
        if current_row >= 0:
            self.sequence_recorder.current_sequence.remove_point(current_row)
            self.sequencePointsList.takeItem(current_row)
            logger.info(f"Deleted point {current_row + 1}")

    def clearSequence(self):
        """Clear all points from sequence"""
        self.sequence_recorder.current_sequence.clear()
        self.sequencePointsList.clear()
        logger.info("Cleared all sequence points")

    def playSequence(self):
        """Play the recorded sequence"""
        if len(self.sequence_recorder.current_sequence) == 0:
            logger.warning("Cannot play empty sequence")
            return

        if self.is_playing_sequence:
            logger.warning("Sequence already playing")
            return

        # Create player with movement callback
        self.sequence_player = seq_rec.SequencePlayer(self.executeSequenceMove)

        # Get playback parameters
        speed = self.sequenceSpeedSpinBox.value()
        loop = self.sequenceLoopCheckBox.isChecked()

        # Start playback (non-blocking)
        self.sequence_player.start_playback(
            self.sequence_recorder.current_sequence,
            speed=speed,
            loop=loop
        )

        # Update button states
        self.sequencePlayButton.setEnabled(False)
        self.sequencePauseButton.setEnabled(True)
        self.sequenceStopButton.setEnabled(True)
        self.is_playing_sequence = True

        # Start timer to update playback (interval from config)
        self.sequence_timer.start(config.SEQUENCE_TIMER_INTERVAL)

        logger.info(f"Started sequence playback (speed={speed}x, loop={loop})")

    def updateSequencePlayback(self):
        """Called by QTimer to advance sequence playback (thread-safe)"""
        if not self.sequence_player:
            self.sequence_timer.stop()
            return

        should_continue, current, total = self.sequence_player.playNextPoint()

        if not should_continue:
            # Playback finished
            self.stopSequence()
            logger.info("Sequence playback completed")

    def pauseSequence(self):
        """Pause/resume sequence playback"""
        if self.sequence_player:
            if self.sequence_player.is_paused:
                self.sequence_player.resume()
                self.sequencePauseButton.setText("Pause")
            else:
                self.sequence_player.pause()
                self.sequencePauseButton.setText("Resume")

    def stopSequence(self):
        """Stop sequence playback"""
        self.sequence_timer.stop()

        if self.sequence_player:
            self.sequence_player.stop()

        self.sequencePlayButton.setEnabled(True)
        self.sequencePauseButton.setEnabled(False)
        self.sequencePauseButton.setText("Pause")
        self.sequenceStopButton.setEnabled(False)
        self.is_playing_sequence = False

        logger.info("Stopped sequence playback")

    def executeSequenceMove(self, q1, q2, q3, q4, q5, q6, gripper):
        """Execute a single movement during sequence playback"""
        logger.info(f"Executing sequence move: q1={q1:.1f}°, q2={q2:.1f}°, q3={q3:.1f}°, q4={q4:.1f}°, q5={q5:.1f}°, q6={q6:.1f}°, grip={gripper}")

        if s0.isOpen():
            # DIFFERENTIAL MECHANISM: Calculate motor positions using helper
            motor_v, motor_w = diff_kin.DifferentialKinematics.joint_to_motor(q5, q6)

            if self.G1MoveRadioButton.isChecked():
                typeOfMovement = "G1 "
                feedRate = " F" + str(self.FeedRateInput.value())
            else:
                typeOfMovement = "G0 "
                feedRate = ""

            # Map all axes: X=Art1, Y=Art2(coupled), Z=Art3, U=Art4, V/W=differential(Art5,Art6)
            message = typeOfMovement + f"X{q1} Y{q2} Z{q3} U{q4} V{motor_v} W{motor_w}{feedRate}"
            messageToSend = message + "\n"
            s0.write(messageToSend.encode('UTF-8'))

            # Move gripper
            if gripper > 0:
                gripper_cmd = f"M3 S{(255/100)*gripper}\n"
                s0.write(gripper_cmd.encode('UTF-8'))

    def saveSequence(self):
        """Save sequence to file"""
        if len(self.sequence_recorder.current_sequence) == 0:
            logger.warning("Cannot save empty sequence")
            return

        filename, _ = QtWidgets.QFileDialog.getSaveFileName(
            None,
            "Save Sequence",
            "",
            "JSON Files (*.json);;All Files (*)"
        )

        if filename:
            if self.sequence_recorder.save_sequence(filename):
                logger.info(f"Saved sequence to {filename}")
                msgBox = QtWidgets.QMessageBox()
                msgBox.setIcon(QtWidgets.QMessageBox.Information)
                msgBox.setText(f"Sequence saved successfully to:\n{filename}")
                msgBox.setWindowTitle("Save Successful")
                msgBox.exec_()
            else:
                msgBox = QtWidgets.QMessageBox()
                msgBox.setIcon(QtWidgets.QMessageBox.Critical)
                msgBox.setText("Failed to save sequence")
                msgBox.setWindowTitle("Save Failed")
                msgBox.exec_()

    def loadSequence(self):
        """Load sequence from file"""
        filename, _ = QtWidgets.QFileDialog.getOpenFileName(
            None,
            "Load Sequence",
            "",
            "JSON Files (*.json);;All Files (*)"
        )

        if filename:
            sequence = self.sequence_recorder.load_sequence(filename)
            if sequence:
                self.sequence_recorder.set_current_sequence(sequence)

                # Update list display
                self.sequencePointsList.clear()
                for i, point in enumerate(sequence.points):
                    point_text = f"Point {i+1}: q1={point.q1:.1f}° q2={point.q2:.1f}° q3={point.q3:.1f}° delay={point.delay:.1f}s"
                    self.sequencePointsList.addItem(point_text)

                logger.info(f"Loaded sequence '{sequence.name}' with {len(sequence)} points")
                msgBox = QtWidgets.QMessageBox()
                msgBox.setIcon(QtWidgets.QMessageBox.Information)
                msgBox.setText(f"Loaded sequence:\n{sequence.name}\n{len(sequence)} points")
                msgBox.setWindowTitle("Load Successful")
                msgBox.exec_()
            else:
                msgBox = QtWidgets.QMessageBox()
                msgBox.setIcon(QtWidgets.QMessageBox.Critical)
                msgBox.setText("Failed to load sequence")
                msgBox.setWindowTitle("Load Failed")
                msgBox.exec_()

# Serial Connection functions
    def getSerialPorts(self):
        self.SerialPortComboBox.clear()
        available_ports = spf.serial_ports()
        self.SerialPortComboBox.addItems(available_ports)

        # Auto-detect and select the robot's COM port
        robot_port = spf.get_robot_port()
        if robot_port:
            # Find the index of the detected port and select it
            index = self.SerialPortComboBox.findText(robot_port)
            if index >= 0:
                self.SerialPortComboBox.setCurrentIndex(index)
                logger.info(f"Auto-detected robot port: {robot_port}")
            else:
                logger.warning(f"Detected port {robot_port} not found in combo box")
        else:
            logger.info("No robot port auto-detected, please select manually")

    def connectSerial(self):
        # Check if already connected - if so, disconnect
        if s0.isOpen():
            self.disconnectSerial()
            return

        serialPort = self.SerialPortComboBox.currentText()
        baudrate = self.BaudRateComboBox.currentText()
        if serialPort == "":
            self.blankSerialPort()
            return
        if baudrate == "":
            self.blankBaudRate()
            return

        # Disable connect button to prevent multiple clicks
        self.ConnectButton.setEnabled(False)
        self.RobotStateDisplay.setText("Connecting...")
        self.RobotStateDisplay.setStyleSheet('background-color: rgb(255, 255, 0)')  # Yellow

        # Run connection in separate thread to prevent GUI freeze
        connection_thread = threading.Thread(
            target=self._connectSerialWorker,
            args=(serialPort, baudrate),
            daemon=True
        )
        connection_thread.start()

    def disconnectSerial(self):
        """Disconnect from serial port"""
        try:
            # Stop serial thread
            if self.SerialThreadClass and self.SerialThreadClass.isRunning():
                self.SerialThreadClass.stop()
                self.SerialThreadClass.wait(2000)

            # Close serial port
            s0.close()

            # Update GUI
            self.serialDisconnected()
            self.ConnectButton.setText("Connect")
            self.ConnectButton.setEnabled(True)

            logger.info("Disconnected from serial port")
            print("Disconnected from serial port")

        except Exception as e:
            logger.error(f"Error during disconnect: {e}")

    def _connectSerialWorker(self, serialPort, baudrate):
        """Worker thread for serial connection (prevents GUI freeze)"""
        try:
            # Stop existing thread if running
            if self.SerialThreadClass and self.SerialThreadClass.isRunning():
                self.SerialThreadClass.stop()
                self.SerialThreadClass.wait(2000)

            # Close existing connection
            s0.close()

            # Configure and open new connection (this can block on Windows!)
            s0.serial.port = serialPort
            s0.serial.baudrate = int(baudrate)
            s0.serial.timeout = config.SERIAL_TIMEOUT
            s0.open()

            # Clear any stale data in buffers
            s0.reset_input_buffer()
            logger.debug("Cleared serial input buffer")

            # Create and start new serial thread (pass GUI instance for event-driven endstop polling)
            self.SerialThreadClass = SerialThreadClass(self.firmware_type, gui_instance=self)
            self.SerialThreadClass.start()

            # Emit success signal (thread-safe)
            # Signal connection will happen in GUI thread
            self.connection_signals.success.emit(serialPort, baudrate)

        except Exception as e:
            logger.exception("Serial connection error")
            # Emit error signal (thread-safe)
            self.connection_signals.error.emit(str(e))

    def _onConnectionSuccess(self, serialPort, baudrate):
        """Called when connection succeeds (runs in GUI thread)"""
        # Connect serial signal in GUI thread (critical for Qt signals to work)
        self.SerialThreadClass.serialSignal.connect(self.updateConsole)

        # Update GUI to show connected state
        self.updateCurrentState("Idle")
        self.ConnectButton.setText("Disconnect")
        self.ConnectButton.setEnabled(True)

        # Request initial position after thread is ready
        QtCore.QTimer.singleShot(50, self.requestInitialPosition)

        logger.info(f"✓ Connected to {serialPort} at {baudrate} baud")
        logger.info(f"✓ Serial thread started (Firmware: {self.firmware_type})")
        logger.info(f"✓ Requesting position update to initialize differential tracking")
        print(f"Connected to {serialPort} at {baudrate} baud")

    def _onConnectionError(self, error_msg):
        """Called when connection fails (runs in GUI thread)"""
        self.serialDisconnected()
        self.ConnectButton.setText("Connect")
        self.ConnectButton.setEnabled(True)

        print(f"Error opening serial port: {error_msg}")
        msgBox = QtWidgets.QMessageBox()
        msgBox.setIcon(QtWidgets.QMessageBox.Critical)
        msgBox.setText(f"Failed to connect to serial port:\n{error_msg}")
        msgBox.setWindowTitle("Connection Error")
        msgBox.exec_()

    def requestInitialPosition(self):
        """Request initial position and endstop status after connection (called by QTimer)"""
        if self.firmware_type == FIRMWARE_RRF and s0.isOpen():
            s0.write("M114\n".encode('UTF-8'), priority=True)
            s0.write("M119\n".encode('UTF-8'), priority=True)
            logger.debug("Requested initial position (M114) and endstop status (M119)")

    def serialDisconnected(self):
        self.RobotStateDisplay.setStyleSheet('background-color: rgb(255, 0, 0)')
        self.RobotStateDisplay.setText("Disconnected")
        self.ConnectButton.setText("Connect")

    def updateConsole(self, dataRead):
        verboseShow=self.ConsoleShowVerbosecheckBox.isChecked()
        okShow=self.ConsoleShowOkRespcheckBox.isChecked()

        if self.firmware_type == FIRMWARE_RRF:
            # RRF: Check if response is M114 position response, M119 endstop response, or ok
            isDataReadVerbose = dataRead.startswith("X:") and "Y:" in dataRead  # M114 response
            isEndstopResponse = dataRead.startswith("Endstops -")  # M119 response
            isDataOkResponse = "ok" in dataRead.lower()
        else:  # GRBL
            isDataReadVerbose = "MPos" in dataRead
            isDataOkResponse = "ok" in dataRead

        # Check if homing completed
        if self.is_homing and isDataOkResponse:
            self.is_homing = False
            self.HomeButton.setEnabled(True)
            self.HomeButton.setText("Home")

        if dataRead=="SERIAL-DISCONNECTED":
            s0.close()
            self.serialDisconnected()
            print ("Serial Connection Lost")

        else:
            # Show responses to manual commands for 2 seconds after command
            time_since_manual = time.time() - self.last_manual_command_time
            if time_since_manual < 2.0:
                self.ConsoleOutput.appendPlainText(dataRead)
            # Handle endstop responses
            elif self.firmware_type == FIRMWARE_RRF and isEndstopResponse:
                self.updateEndstopDisplay(dataRead)
            elif not isDataReadVerbose and not isDataOkResponse:
                self.ConsoleOutput.appendPlainText(dataRead)
            elif isDataOkResponse and okShow:
                self.ConsoleOutput.appendPlainText(dataRead)
            elif isDataReadVerbose:
                self.updateFKPosDisplay(dataRead)
                if verboseShow:
                    self.ConsoleOutput.appendPlainText(dataRead)

    def sendSerialCommand(self):
        command = self.ConsoleInput.text()
        messageToSent = command + "\n"
        messageToConsole = ">>> " + command
        if s0.isOpen():
            if messageToSent != "\n":  # Don't send empty commands
                s0.write(messageToSent.encode('UTF-8'))
                self.ConsoleOutput.appendPlainText(messageToConsole)
                self.ConsoleInput.addToHistory(command)
                self.ConsoleInput.clear()
                logger.debug(f"Manual command sent: {command}")
                # Mark time of manual command to show responses for next 2 seconds
                self.last_manual_command_time = time.time()
        else:
            self.noSerialConnection()

    def validatePosition(self, axis, value):
        """
        Validate position value for reasonableness (uses config.py limits)

        Args:
            axis: Axis name (X, Y, Z, U, V, W)
            value: Position value in degrees

        Returns:
            (is_valid, sanitized_value)
        """
        if axis not in config.POSITION_LIMITS:
            return (True, value)  # Unknown axis, accept

        min_val, max_val = config.POSITION_LIMITS[axis]

        # Check absolute limits
        if not (min_val <= value <= max_val):
            logger.warning(f"Position out of bounds: {axis}={value:.2f}° (limits: {min_val} to {max_val})")
            # Clamp to limits
            value = max(min_val, min(max_val, value))
            return (False, value)

        # Check for impossible changes
        if axis in self.last_valid_positions:
            last_value = self.last_valid_positions[axis]
            change = abs(value - last_value)
            if change > config.MAX_POSITION_CHANGE_PER_UPDATE:
                logger.warning(f"Impossible position change detected: {axis} changed {change:.2f}° in 100ms (max: {config.MAX_POSITION_CHANGE_PER_UPDATE}°)")
                # Use last valid value
                return (False, last_value)

        return (True, value)

    def updateFKPosDisplay(self,dataRead):
        if self.firmware_type == FIRMWARE_RRF:
            # RRF: Parse M114 response using compiled regex (3x faster than string.split)
            try:
                # Extract position values using regex
                matches = RRF_POSITION_PATTERN.findall(dataRead)
                if not matches:
                    return

                pos_dict = {axis: float(value) for axis, value in matches}

                # Extract axis positions with validation
                if 'V' in pos_dict and 'W' in pos_dict:
                    # Validate each position
                    motor_x_valid, motor_x = self.validatePosition('X', pos_dict.get('X', 0.0))
                    motor_y_valid, motor_y = self.validatePosition('Y', pos_dict.get('Y', 0.0))
                    motor_z_valid, motor_z = self.validatePosition('Z', pos_dict.get('Z', 0.0))
                    motor_u_valid, motor_u = self.validatePosition('U', pos_dict.get('U', 0.0))
                    motor_v_valid, motor_v = self.validatePosition('V', pos_dict.get('V', 0.0))
                    motor_w_valid, motor_w = self.validatePosition('W', pos_dict.get('W', 0.0))

                    # Update last valid positions
                    self.last_valid_positions['X'] = motor_x
                    self.last_valid_positions['Y'] = motor_y
                    self.last_valid_positions['Z'] = motor_z
                    self.last_valid_positions['U'] = motor_u
                    self.last_valid_positions['V'] = motor_v
                    self.last_valid_positions['W'] = motor_w

                    # INVERSE DIFFERENTIAL: Convert motor positions to joint angles using helper
                    art5, art6 = diff_kin.DifferentialKinematics.motor_to_joint(motor_v, motor_w)

                    # Update tracked motor positions
                    self.current_motor_v = motor_v
                    self.current_motor_w = motor_w
                    self.desired_art5 = art5
                    self.desired_art6 = art6

                    self.position_update_count += 1

                    # Record position history (sampled based on config)
                    if self.position_update_count % config.POSITION_HISTORY_SAMPLE_RATE == 0:
                        self.position_history.add_snapshot(
                            art1=motor_x,
                            art2=motor_y,
                            art3=motor_z,
                            art4=motor_u,
                            art5=art5,
                            art6=art6
                        )

                    # Log every Nth update to reduce noise (from config)
                    if self.position_update_count % config.LOGGING_INTERVAL_POSITIONS == 0:
                        logger.debug(f"Position feedback - X:{motor_x:.2f} Y:{motor_y:.2f} Z:{motor_z:.2f} U:{motor_u:.2f} V:{motor_v:.2f} W:{motor_w:.2f}")
                        logger.info(f"  Art1<-X:{motor_x:.2f}° | Art2<-Y:{motor_y:.2f}° | Art3<-Z:{motor_z:.2f}° | Art4<-U:{motor_u:.2f}° | Art5(calc):{art5:.2f}° | Art6(calc):{art6:.2f}°")

                    # Throttle GUI updates (interval from config)
                    current_time = time.time()
                    if current_time - self.last_gui_update_time >= config.GUI_UPDATE_INTERVAL:
                        self.last_gui_update_time = current_time

                        # Display positions
                        self.FKCurrentPosValueArt1.setText(f"{motor_x:.2f}º")
                        self.FKCurrentPosValueArt2.setText(f"{motor_y:.2f}º")
                        self.FKCurrentPosValueArt3.setText(f"{motor_z:.2f}º")
                        self.FKCurrentPosValueArt4.setText(f"{motor_u:.2f}º")
                        self.FKCurrentPosValueArt5.setText(f"{art5:.2f}º")
                        self.FKCurrentPosValueArt6.setText(f"{art6:.2f}º")

                        # Set status to Idle since M114 doesn't provide status
                        self.updateCurrentState("Idle")

            except (ValueError, KeyError, IndexError) as e:
                logger.error(f"Error parsing M114 response: {e}")
                logger.debug(f"Problematic data: {dataRead}")
                logger.exception("Position parsing error")
        else:  # GRBL
            # GRBL: Parse comma-separated format
            try:
                data = dataRead[1:][:-1].split(",")
                if len(data) > 0:
                    self.updateCurrentState(data[0])
                if len(data) >= 8:
                    logger.debug(f"GRBL Position data array: {data}")
                    logger.info(f"  Art1<-{data[1][5:][:-2]}° | Art2<-{data[2][:-2]}° | Art3<-{data[4][:-2]}° | Art4<-{data[5][:-2]}° | Art5<-{data[6][:-2]}° | Art6<-{data[7][:-2]}°")
                    self.FKCurrentPosValueArt1.setText(data[1][5:][:-2]+"º")
                    self.FKCurrentPosValueArt2.setText(data[2][:-2]+"º")
                    self.FKCurrentPosValueArt3.setText(data[4][:-2]+"º")
                    self.FKCurrentPosValueArt4.setText(data[5][:-2]+"º")
                    self.FKCurrentPosValueArt5.setText(data[6][:-2]+"º")
                    self.FKCurrentPosValueArt6.setText(data[7][:-2]+"º")
            except (IndexError, ValueError) as e:
                logger.error(f"Error parsing GRBL status: {e}")

    def updateEndstopDisplay(self, dataRead):
        """Parse M119 endstop response and update GUI displays (optimized with regex)"""
        # Format: "Endstops - X: at min stop, Y: not stopped, Z: not stopped, U: not stopped, V: at min stop, W: at min stop, Z probe: at min stop"
        try:
            # Use regex to extract endstop statuses (faster than string split)
            matches = ENDSTOP_PATTERN.findall(dataRead)
            if not matches:
                return

            endstops = {axis: status.strip() for axis, status in matches}

            # Update GUI labels with color coding (compact format for inline display)
            # Green = not triggered, Red = triggered
            # Mapping: X->Art1, Y->Art2, Z->Art3, U->Art4, V->Art5, W->Art6
            def updateLabel(label, axis_name):
                if axis_name in endstops:
                    status = endstops[axis_name]
                    # Compact status text with axis label
                    if "not stopped" in status:
                        label.setText(f"{axis_name}: OK")
                        label.setStyleSheet("background-color: rgb(200, 255, 200); padding: 2px; border-radius: 3px;")  # Light green
                    elif "min" in status:
                        label.setText(f"{axis_name}: MIN")
                        label.setStyleSheet("background-color: rgb(255, 200, 200); padding: 2px; border-radius: 3px;")  # Light red
                    elif "max" in status:
                        label.setText(f"{axis_name}: MAX")
                        label.setStyleSheet("background-color: rgb(255, 200, 200); padding: 2px; border-radius: 3px;")  # Light red
                    else:
                        label.setText(f"{axis_name}: {status[:6]}")
                        label.setStyleSheet("background-color: rgb(255, 255, 200); padding: 2px; border-radius: 3px;")  # Yellow

            updateLabel(self.endstopLabelX, "X")
            updateLabel(self.endstopLabelY, "Y")
            updateLabel(self.endstopLabelZ, "Z")
            updateLabel(self.endstopLabelU, "U")
            updateLabel(self.endstopLabelV, "V")
            updateLabel(self.endstopLabelW, "W")

        except Exception as e:
            logger.error(f"Error parsing M119 endstop response: {e}")
            logger.debug(f"Problematic data: {dataRead}")

    def updateCurrentState(self, state):
        self.RobotStateDisplay.setText(state)
        if state=="Idle" or state=="Run":
            self.RobotStateDisplay.setStyleSheet('background-color: rgb(0, 255, 0)')
        elif state=="Home":
            self.RobotStateDisplay.setStyleSheet('background-color: rgb(85, 255, 255)')
        elif state=="Alarm":
            self.RobotStateDisplay.setStyleSheet('background-color: rgb(255, 255, 0)')
        elif state=="Hold":
            self.RobotStateDisplay.setStyleSheet('background-color: rgb(255, 0, 0)')
        else:
            self.RobotStateDisplay.setStyleSheet('background-color: rgb(255, 255, 255)')


    def blankSerialPort(self):
        msgBox = QtWidgets.QMessageBox()
        msgBox.setIcon(QtWidgets.QMessageBox.Warning)
        msgBox.setText("There is not Serial Port value indicated to establish the connection.\nPlease check it and try to connect again.")
        msgBox.exec_()

    def blankBaudRate(self):
        msgBox = QtWidgets.QMessageBox()
        msgBox.setIcon(QtWidgets.QMessageBox.Warning)
        msgBox.setText("There is not Baud Rate value indicated to establish the connection.\nPlease check it and try to connect again.")
        msgBox.exec_()

    def noSerialConnection(self):
        msgBox = QtWidgets.QMessageBox()
        msgBox.setIcon(QtWidgets.QMessageBox.Warning)
        msgBox.setText("The connection has not been established yet. Please establish the connection before trying to control.")
        msgBox.exec_()

############### SERIAL READ THREAD CLASS ###############

class SerialThreadClass(QtCore.QThread):
    serialSignal = pyqtSignal(str)

    def __init__(self, firmware_type, gui_instance=None, parent=None):
        super(SerialThreadClass, self).__init__(parent)
        self.firmware_type = firmware_type
        self.gui_instance = gui_instance  # Reference to GUI for movement tracking
        self.running = True
        self.elapsedTime = time.time()
        self.endstopCheckTime = time.time()
        self.status_polling_paused = False  # Flag to pause polling during long commands
        self.blocking_command_start_time = 0.0  # When blocking command was sent

    def stop(self):
        """Gracefully stop the thread"""
        self.running = False

    def run(self):
        """Main thread loop with non-blocking reads and command queue processing"""
        while self.running:
            if not s0.isOpen():
                time.sleep(0.1)
                continue

            try:
                # Check if connection is still alive
                try:
                    bytes_available = s0.inWaiting()
                except (OSError, serial.SerialException):
                    self.serialSignal.emit("SERIAL-DISCONNECTED")
                    print("Lost Serial connection!")
                    break

                current_time = time.time()

                # PROCESS COMMAND QUEUE (non-blocking)
                # This is where all writes actually happen, keeping GUI thread free
                command = s0.get_next_command()
                if command:
                    success = s0._write_internal(command)
                    if not success:
                        self.serialSignal.emit("SERIAL-DISCONNECTED")
                        break

                    # Check if this is a long-running blocking command
                    command_str = command.decode('UTF-8', errors='replace').strip().upper()
                    if self.firmware_type == FIRMWARE_RRF:
                        # RRF blocking commands: G28 (home), G29 (bed probe), M999 (reset)
                        blocking_commands = ['G28', 'G29', 'M999']
                    else:  # GRBL
                        # GRBL blocking commands: $H (home), $X (unlock)
                        blocking_commands = ['$H', '$X']

                    # Pause status polling if we sent a blocking command
                    for block_cmd in blocking_commands:
                        if command_str.startswith(block_cmd):
                            self.status_polling_paused = True
                            self.blocking_command_start_time = current_time
                            logger.info(f"Pausing status polling for blocking command: {command_str}")
                            break

                # Check for timeout on paused polling (safety mechanism)
                if self.status_polling_paused:
                    time_paused = current_time - self.blocking_command_start_time
                    if time_paused >= config.BLOCKING_COMMAND_MAX_PAUSE:
                        self.status_polling_paused = False
                        logger.warning(f"Forcing resume of status polling after {time_paused:.1f}s timeout (max: {config.BLOCKING_COMMAND_MAX_PAUSE}s)")
                        # Request immediate position update
                        if self.firmware_type == FIRMWARE_RRF:
                            s0.write("M114\n".encode('UTF-8'), priority=True)
                            s0.write("M119\n".encode('UTF-8'), priority=True)

                # Send status request (interval from config) - ONLY if not paused
                if not self.status_polling_paused and current_time - self.elapsedTime > config.SERIAL_STATUS_REQUEST_INTERVAL:
                    self.elapsedTime = current_time
                    try:
                        if self.firmware_type == FIRMWARE_RRF:
                            s0.write("M114\n".encode('UTF-8'), priority=True)
                        else:  # GRBL
                            s0.write("?\n".encode('UTF-8'), priority=True)
                    except Exception as e:
                        print(f"Error queuing status request: {e}")

                # Poll endstop status regularly - ONLY if not paused
                if not self.status_polling_paused and current_time - self.endstopCheckTime > config.SERIAL_ENDSTOP_REQUEST_INTERVAL:
                    self.endstopCheckTime = current_time
                    try:
                        if self.firmware_type == FIRMWARE_RRF:
                            s0.write("M119\n".encode('UTF-8'), priority=True)
                    except Exception as e:
                        print(f"Error queuing endstop request: {e}")

                # NON-BLOCKING READ: Only read if data is available
                # This prevents the thread from blocking when firmware is busy executing moves
                try:
                    if bytes_available > 0:
                        dataBytes = s0.readline()
                        if dataBytes:
                            # Decode bytes to string and strip whitespace
                            dataCropped = dataBytes.decode('UTF-8', errors='replace').strip()
                            if dataCropped:
                                self.serialSignal.emit(dataCropped)

                                # Resume status polling when blocking command completes
                                # Must meet BOTH conditions: received "ok" AND minimum time elapsed
                                if self.status_polling_paused and "ok" in dataCropped.lower():
                                    time_elapsed = current_time - self.blocking_command_start_time
                                    if time_elapsed >= config.BLOCKING_COMMAND_MIN_PAUSE:
                                        self.status_polling_paused = False
                                        logger.info(f"Resuming status polling after blocking command completed ({time_elapsed:.1f}s elapsed)")
                                        # Request immediate position update after resuming
                                        if self.firmware_type == FIRMWARE_RRF:
                                            s0.write("M114\n".encode('UTF-8'), priority=True)
                                            s0.write("M119\n".encode('UTF-8'), priority=True)
                                    else:
                                        logger.debug(f"Received 'ok' but only {time_elapsed:.1f}s elapsed (need {config.BLOCKING_COMMAND_MIN_PAUSE}s), waiting...")
                except (OSError, serial.SerialException) as e:
                    print(f"Error reading from serial: {e}")
                    self.serialSignal.emit("SERIAL-DISCONNECTED")
                    break

                # Small sleep to prevent busy-waiting (from config)
                time.sleep(config.SERIAL_THREAD_SLEEP)

            except Exception as e:
                print(f"Unexpected error in serial thread: {e}")
                logger.exception("Serial thread error")
                time.sleep(0.1)

        print("Serial thread stopped")


###############  SERIAL READ THREAD CLASS ###############










class MainWindow(QtWidgets.QMainWindow):
    """Custom MainWindow to handle close event"""
    def __init__(self, gui_instance):
        super().__init__()
        self.gui_instance = gui_instance

    def closeEvent(self, event):
        """Handle window close event with proper cleanup"""
        # Stop serial thread if running
        if self.gui_instance.SerialThreadClass and self.gui_instance.SerialThreadClass.isRunning():
            print("Stopping serial thread...")
            self.gui_instance.SerialThreadClass.stop()
            self.gui_instance.SerialThreadClass.wait(2000)

        # Close serial port
        s0.close()
        print("Application closed cleanly")
        event.accept()

if __name__ == '__main__':
    # Enable High DPI scaling for 4K displays
    QtWidgets.QApplication.setAttribute(QtCore.Qt.AA_EnableHighDpiScaling, True)
    QtWidgets.QApplication.setAttribute(QtCore.Qt.AA_UseHighDpiPixmaps, True)

    app = QtWidgets.QApplication(sys.argv)

    # Create main window first
    mwindow = MainWindow(None)
    mwindow.setMinimumSize(1820, 920)  # Adjusted width to fit visualization

    # Create GUI instance
    prog = BifrostGUI(mwindow)

    # Link gui instance to window for cleanup
    mwindow.gui_instance = prog

    mwindow.show()
    sys.exit(app.exec_())
