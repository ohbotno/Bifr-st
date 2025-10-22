import sys
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtGui import *
from PyQt5.QtCore import *

from gui import Ui_MainWindow
from about import Ui_Dialog as About_Ui_Dialog


import serial_port_finder as spf
import inverse_kinematics as ik
import sequence_recorder as seq_rec

import serial
import time
import json
import threading
import logging
import numpy as np

# Configure logging for debugging
logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s - %(levelname)s - [%(funcName)s] %(message)s',
    handlers=[
        logging.FileHandler('bifrost_debug.log'),
        logging.StreamHandler(sys.stdout)
    ]
)
logger = logging.getLogger(__name__)

# Firmware type constants
FIRMWARE_GRBL = "GRBL"
FIRMWARE_RRF = "RRF"

# Thread-safe serial object with lock
class SerialManager:
    def __init__(self):
        self.serial = serial.Serial()
        self.lock = threading.Lock()

    def write(self, data):
        with self.lock:
            if self.serial.isOpen():
                self.serial.write(data)

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

class BifrostGUI(Ui_MainWindow):
    def __init__(self, dialog):
        Ui_MainWindow.__init__(self)
        self.setupUi(dialog)

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

        # Setup endstop status displays
        self.setupEndstopDisplays()

    def close_application(self):
        # Properly cleanup serial connection and thread
        if self.SerialThreadClass and self.SerialThreadClass.isRunning():
            self.SerialThreadClass.stop()
            self.SerialThreadClass.wait(2000)  # Wait up to 2 seconds
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
        val=self.SpinBoxArt1.value()-10
        self.SpinBoxArt1.setValue(val)
    def FKDec1Art1(self):
        val=self.SpinBoxArt1.value()-1
        self.SpinBoxArt1.setValue(val)
    def FKDec0_1Art1(self):
        val=self.SpinBoxArt1.value()-0.1
        self.SpinBoxArt1.setValue(val)
    def FKInc0_1Art1(self):
        val=self.SpinBoxArt1.value()+0.1
        self.SpinBoxArt1.setValue(val)
    def FKInc1Art1(self):
        val=self.SpinBoxArt1.value()+1
        self.SpinBoxArt1.setValue(val)
    def FKInc10Art1(self):
        val=self.SpinBoxArt1.value()+10
        self.SpinBoxArt1.setValue(val)

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
        val=self.SpinBoxArt2.value()-10
        self.SpinBoxArt2.setValue(val)
    def FKDec1Art2(self):
        val=self.SpinBoxArt2.value()-1
        self.SpinBoxArt2.setValue(val)
    def FKDec0_1Art2(self):
        val=self.SpinBoxArt2.value()-0.1
        self.SpinBoxArt2.setValue(val)
    def FKInc0_1Art2(self):
        val=self.SpinBoxArt2.value()+0.1
        self.SpinBoxArt2.setValue(val)
    def FKInc1Art2(self):
        val=self.SpinBoxArt2.value()+1
        self.SpinBoxArt2.setValue(val)
    def FKInc10Art2(self):
        val=self.SpinBoxArt2.value()+10
        self.SpinBoxArt2.setValue(val)

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
        val=self.SpinBoxArt3.value()-10
        self.SpinBoxArt3.setValue(val)
    def FKDec1Art3(self):
        val=self.SpinBoxArt3.value()-1
        self.SpinBoxArt3.setValue(val)
    def FKDec0_1Art3(self):
        val=self.SpinBoxArt3.value()-0.1
        self.SpinBoxArt3.setValue(val)
    def FKInc0_1Art3(self):
        val=self.SpinBoxArt3.value()+0.1
        self.SpinBoxArt3.setValue(val)
    def FKInc1Art3(self):
        val=self.SpinBoxArt3.value()+1
        self.SpinBoxArt3.setValue(val)
    def FKInc10Art3(self):
        val=self.SpinBoxArt3.value()+10
        self.SpinBoxArt3.setValue(val)

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
        val=self.SpinBoxArt4.value()-10
        self.SpinBoxArt4.setValue(val)
    def FKDec1Art4(self):
        val=self.SpinBoxArt4.value()-1
        self.SpinBoxArt4.setValue(val)
    def FKDec0_1Art4(self):
        val=self.SpinBoxArt4.value()-0.1
        self.SpinBoxArt4.setValue(val)
    def FKInc0_1Art4(self):
        val=self.SpinBoxArt4.value()+0.1
        self.SpinBoxArt4.setValue(val)
    def FKInc1Art4(self):
        val=self.SpinBoxArt4.value()+1
        self.SpinBoxArt4.setValue(val)
    def FKInc10Art4(self):
        val=self.SpinBoxArt4.value()+10
        self.SpinBoxArt4.setValue(val)

#FK Art5 Functions
    def FKMoveArt5(self):
        # DIFFERENTIAL MECHANISM: Art5 & Art6 are coupled via bevel gear differential
        # Motors: Drive5 (V axis) and Drive6 (W axis)
        # To move only Art5 while keeping Art6 stationary:
        # - Calculate Art6 from current motor positions (most recent robot state)
        # - Use new Art5 value from spinbox

        art5_value = self.SpinBoxArt5.value()

        # Check if we have valid position feedback
        if self.current_motor_v == 0.0 and self.current_motor_w == 0.0:
            logger.warning("No position feedback received yet - differential control may be inaccurate!")
            logger.warning("Wait for position update or home the robot first")

        # Calculate current Art6 from motor positions to keep it stationary
        art6_value = (self.current_motor_v + self.current_motor_w) / 2.0

        # Calculate new motor positions
        motor_v = art6_value + art5_value  # Drive 5 (V axis)
        motor_w = art6_value - art5_value  # Drive 6 (W axis)

        logger.info(f"Art5 (DIFFERENTIAL) commanded to: {art5_value}°")
        logger.info(f"  BEFORE: Motor_V={self.current_motor_v:.2f}° Motor_W={self.current_motor_w:.2f}° → Art5={(self.current_motor_v - self.current_motor_w)/2:.2f}° Art6={art6_value:.2f}°")
        logger.info(f"  AFTER:  Motor_V={motor_v:.2f}° Motor_W={motor_w:.2f}° → Art5={art5_value:.2f}° Art6={art6_value:.2f}° (Art6 should stay same)")

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
        val=self.SpinBoxArt5.value()-10
        self.SpinBoxArt5.setValue(val)
    def FKDec1Art5(self):
        val=self.SpinBoxArt5.value()-1
        self.SpinBoxArt5.setValue(val)
    def FKDec0_1Art5(self):
        val=self.SpinBoxArt5.value()-0.1
        self.SpinBoxArt5.setValue(val)
    def FKInc0_1Art5(self):
        val=self.SpinBoxArt5.value()+0.1
        self.SpinBoxArt5.setValue(val)
    def FKInc1Art5(self):
        val=self.SpinBoxArt5.value()+1
        self.SpinBoxArt5.setValue(val)
    def FKInc10Art5(self):
        val=self.SpinBoxArt5.value()+10
        self.SpinBoxArt5.setValue(val)

#FK Art6 Functions
    def FKMoveArt6(self):
        # DIFFERENTIAL MECHANISM: Art5 & Art6 are coupled via bevel gear differential
        # Motors: Drive5 (V axis) and Drive6 (W axis)
        # To move only Art6 while keeping Art5 stationary:
        # - Calculate Art5 from current motor positions (most recent robot state)
        # - Use new Art6 value from spinbox

        art6_value = self.SpinBoxArt6.value()

        # Check if we have valid position feedback
        if self.current_motor_v == 0.0 and self.current_motor_w == 0.0:
            logger.warning("No position feedback received yet - differential control may be inaccurate!")
            logger.warning("Wait for position update or home the robot first")

        # Calculate current Art5 from motor positions to keep it stationary
        art5_value = (self.current_motor_v - self.current_motor_w) / 2.0

        # Calculate new motor positions
        motor_v = art6_value + art5_value  # Drive 5 (V axis)
        motor_w = art6_value - art5_value  # Drive 6 (W axis)

        logger.info(f"Art6 (DIFFERENTIAL) commanded to: {art6_value}°")
        logger.info(f"  BEFORE: Motor_V={self.current_motor_v:.2f}° Motor_W={self.current_motor_w:.2f}° → Art5={art5_value:.2f}° Art6={(self.current_motor_v + self.current_motor_w)/2:.2f}°")
        logger.info(f"  AFTER:  Motor_V={motor_v:.2f}° Motor_W={motor_w:.2f}° → Art5={art5_value:.2f}° Art6={art6_value:.2f}° (Art5 should stay same)")

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
        val=self.SpinBoxArt6.value()-10
        self.SpinBoxArt6.setValue(val)
    def FKDec1Art6(self):
        val=self.SpinBoxArt6.value()-1
        self.SpinBoxArt6.setValue(val)
    def FKDec0_1Art6(self):
        val=self.SpinBoxArt6.value()-0.1
        self.SpinBoxArt6.setValue(val)
    def FKInc0_1Art6(self):
        val=self.SpinBoxArt6.value()+0.1
        self.SpinBoxArt6.setValue(val)
    def FKInc1Art6(self):
        val=self.SpinBoxArt6.value()+1
        self.SpinBoxArt6.setValue(val)
    def FKInc10Art6(self):
        val=self.SpinBoxArt6.value()+10
        self.SpinBoxArt6.setValue(val)

#FK Every Articulation Functions
    def FKMoveAll(self):
        # DIFFERENTIAL MECHANISM: Art5 & Art6 use differential, need to calculate motor positions
        # Other axes map directly to motors

        if s0.isOpen():
            # Get joint values
            art1 = self.SpinBoxArt1.value()
            art2 = self.SpinBoxArt2.value()
            art3 = self.SpinBoxArt3.value()
            art4 = self.SpinBoxArt4.value()
            art5 = self.SpinBoxArt5.value()
            art6 = self.SpinBoxArt6.value()

            # Calculate differential motor positions for Art5/Art6
            motor_v = art6 + art5  # Drive 5 (V axis) for differential
            motor_w = art6 - art5  # Drive 6 (W axis) for differential

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
        val=self.SpinBoxGripper.value()-10
        self.SpinBoxGripper.setValue(val)
    def Dec1Gripper(self):
        val=self.SpinBoxGripper.value()-1
        self.SpinBoxGripper.setValue(val)
    def Inc1Gripper(self):
        val=self.SpinBoxGripper.value()+1
        self.SpinBoxGripper.setValue(val)
    def Inc10Gripper(self):
        val=self.SpinBoxGripper.value()+10
        self.SpinBoxGripper.setValue(val)

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
        self.sequenceGroupBox.setGeometry(QtCore.QRect(910, 155, 280, 703))
        self.sequenceGroupBox.setTitle("Sequence Programmer")

        # List widget for sequence points (expanded to fill space, aligned with console send button)
        self.sequencePointsList = QtWidgets.QListWidget(self.sequenceGroupBox)
        self.sequencePointsList.setGeometry(QtCore.QRect(10, 25, 260, 420))

        # Record controls
        self.sequenceRecordButton = QtWidgets.QPushButton(self.sequenceGroupBox)
        self.sequenceRecordButton.setGeometry(QtCore.QRect(10, 455, 120, 30))
        self.sequenceRecordButton.setText("Record Point")
        self.sequenceRecordButton.pressed.connect(self.recordSequencePoint)

        self.sequenceDeleteButton = QtWidgets.QPushButton(self.sequenceGroupBox)
        self.sequenceDeleteButton.setGeometry(QtCore.QRect(150, 455, 120, 30))
        self.sequenceDeleteButton.setText("Delete Point")
        self.sequenceDeleteButton.pressed.connect(self.deleteSequencePoint)

        self.sequenceClearButton = QtWidgets.QPushButton(self.sequenceGroupBox)
        self.sequenceClearButton.setGeometry(QtCore.QRect(10, 490, 260, 30))
        self.sequenceClearButton.setText("Clear All")
        self.sequenceClearButton.pressed.connect(self.clearSequence)

        # Playback controls
        self.sequencePlayButton = QtWidgets.QPushButton(self.sequenceGroupBox)
        self.sequencePlayButton.setGeometry(QtCore.QRect(10, 530, 80, 30))
        self.sequencePlayButton.setText("Play")
        self.sequencePlayButton.pressed.connect(self.playSequence)

        self.sequencePauseButton = QtWidgets.QPushButton(self.sequenceGroupBox)
        self.sequencePauseButton.setGeometry(QtCore.QRect(100, 530, 80, 30))
        self.sequencePauseButton.setText("Pause")
        self.sequencePauseButton.setEnabled(False)
        self.sequencePauseButton.pressed.connect(self.pauseSequence)

        self.sequenceStopButton = QtWidgets.QPushButton(self.sequenceGroupBox)
        self.sequenceStopButton.setGeometry(QtCore.QRect(190, 530, 80, 30))
        self.sequenceStopButton.setText("Stop")
        self.sequenceStopButton.setEnabled(False)
        self.sequenceStopButton.pressed.connect(self.stopSequence)

        # Speed control
        speedLabel = QtWidgets.QLabel(self.sequenceGroupBox)
        speedLabel.setGeometry(QtCore.QRect(10, 570, 50, 20))
        speedLabel.setText("Speed:")

        self.sequenceSpeedSpinBox = QtWidgets.QDoubleSpinBox(self.sequenceGroupBox)
        self.sequenceSpeedSpinBox.setGeometry(QtCore.QRect(60, 570, 80, 22))
        self.sequenceSpeedSpinBox.setMinimum(0.1)
        self.sequenceSpeedSpinBox.setMaximum(10.0)
        self.sequenceSpeedSpinBox.setSingleStep(0.1)
        self.sequenceSpeedSpinBox.setValue(1.0)
        self.sequenceSpeedSpinBox.setSuffix("x")

        self.sequenceLoopCheckBox = QtWidgets.QCheckBox(self.sequenceGroupBox)
        self.sequenceLoopCheckBox.setGeometry(QtCore.QRect(150, 570, 60, 20))
        self.sequenceLoopCheckBox.setText("Loop")

        # Delay control
        delayLabel = QtWidgets.QLabel(self.sequenceGroupBox)
        delayLabel.setGeometry(QtCore.QRect(10, 600, 50, 20))
        delayLabel.setText("Delay:")

        self.sequenceDelaySpinBox = QtWidgets.QDoubleSpinBox(self.sequenceGroupBox)
        self.sequenceDelaySpinBox.setGeometry(QtCore.QRect(60, 600, 80, 22))
        self.sequenceDelaySpinBox.setMinimum(0.0)
        self.sequenceDelaySpinBox.setMaximum(60.0)
        self.sequenceDelaySpinBox.setSingleStep(0.1)
        self.sequenceDelaySpinBox.setValue(1.0)
        self.sequenceDelaySpinBox.setSuffix("s")

        # File buttons
        self.sequenceSaveButton = QtWidgets.QPushButton(self.sequenceGroupBox)
        self.sequenceSaveButton.setGeometry(QtCore.QRect(10, 635, 260, 30))
        self.sequenceSaveButton.setText("Save Sequence")
        self.sequenceSaveButton.pressed.connect(self.saveSequence)

        self.sequenceLoadButton = QtWidgets.QPushButton(self.sequenceGroupBox)
        self.sequenceLoadButton.setGeometry(QtCore.QRect(10, 670, 260, 30))
        self.sequenceLoadButton.setText("Load Sequence")
        self.sequenceLoadButton.pressed.connect(self.loadSequence)

        logger.info("Sequence recorder controls initialized")

    def setupEndstopDisplays(self):
        """Create endstop status display GUI controls programmatically"""
        # Create group box for endstop displays
        self.endstopGroupBox = QtWidgets.QGroupBox(self.centralwidget)
        self.endstopGroupBox.setGeometry(QtCore.QRect(910, 20, 280, 125))
        self.endstopGroupBox.setTitle("Endstop Status")

        # Create labels for each endstop (positioned in 2 columns)
        y_pos = 25
        col1_x = 10
        col2_x = 145
        label_height = 25

        # Column 1: Art1, Art2, Art3
        self.endstopLabelX = QtWidgets.QLabel(self.endstopGroupBox)
        self.endstopLabelX.setGeometry(QtCore.QRect(col1_x, y_pos, 120, label_height))
        self.endstopLabelX.setText("Art1 (X): ---")
        self.endstopLabelX.setStyleSheet("background-color: rgb(200, 200, 200); padding: 3px;")

        self.endstopLabelY = QtWidgets.QLabel(self.endstopGroupBox)
        self.endstopLabelY.setGeometry(QtCore.QRect(col1_x, y_pos + 30, 120, label_height))
        self.endstopLabelY.setText("Art2 (Y): ---")
        self.endstopLabelY.setStyleSheet("background-color: rgb(200, 200, 200); padding: 3px;")

        self.endstopLabelZ = QtWidgets.QLabel(self.endstopGroupBox)
        self.endstopLabelZ.setGeometry(QtCore.QRect(col1_x, y_pos + 60, 120, label_height))
        self.endstopLabelZ.setText("Art3 (Z): ---")
        self.endstopLabelZ.setStyleSheet("background-color: rgb(200, 200, 200); padding: 3px;")

        # Column 2: Art4, Art5, Art6
        self.endstopLabelU = QtWidgets.QLabel(self.endstopGroupBox)
        self.endstopLabelU.setGeometry(QtCore.QRect(col2_x, y_pos, 120, label_height))
        self.endstopLabelU.setText("Art4 (U): ---")
        self.endstopLabelU.setStyleSheet("background-color: rgb(200, 200, 200); padding: 3px;")

        self.endstopLabelV = QtWidgets.QLabel(self.endstopGroupBox)
        self.endstopLabelV.setGeometry(QtCore.QRect(col2_x, y_pos + 30, 120, label_height))
        self.endstopLabelV.setText("Art5 (V): ---")
        self.endstopLabelV.setStyleSheet("background-color: rgb(200, 200, 200); padding: 3px;")

        self.endstopLabelW = QtWidgets.QLabel(self.endstopGroupBox)
        self.endstopLabelW.setGeometry(QtCore.QRect(col2_x, y_pos + 60, 120, label_height))
        self.endstopLabelW.setText("Art6 (W): ---")
        self.endstopLabelW.setStyleSheet("background-color: rgb(200, 200, 200); padding: 3px;")

        # Make the group box visible
        self.endstopGroupBox.show()

        logger.info("Endstop status displays initialized")

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

        # Update button states
        self.sequencePlayButton.setEnabled(False)
        self.sequencePauseButton.setEnabled(True)
        self.sequenceStopButton.setEnabled(True)
        self.is_playing_sequence = True

        # Start playback in a separate thread
        self.playback_thread = threading.Thread(
            target=self.runSequencePlayback,
            args=(speed, loop),
            daemon=True
        )
        self.playback_thread.start()

        logger.info(f"Started sequence playback (speed={speed}x, loop={loop})")

    def runSequencePlayback(self, speed, loop):
        """Run sequence playback in thread"""
        try:
            for current, total in self.sequence_player.play(
                self.sequence_recorder.current_sequence,
                speed=speed,
                loop=loop
            ):
                # Update UI progress (would need signal/slot for thread safety)
                pass
        except Exception as e:
            logger.error(f"Error during sequence playback: {e}")
        finally:
            self.is_playing_sequence = False
            # Reset button states (would need signal/slot for thread safety)

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
            # DIFFERENTIAL MECHANISM: Art5 & Art6 use differential, calculate motor positions
            motor_v = q6 + q5  # Drive 5 (V axis) for differential
            motor_w = q6 - q5  # Drive 6 (W axis) for differential

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
        self.SerialPortComboBox.addItems(spf.serial_ports())

    def connectSerial(self):
        serialPort = self.SerialPortComboBox.currentText()
        baudrate = self.BaudRateComboBox.currentText()
        if serialPort == "":
            self.blankSerialPort()
            return
        if baudrate == "":
            self.blankBaudRate()
            return

        # Stop existing thread if running
        if self.SerialThreadClass and self.SerialThreadClass.isRunning():
            self.SerialThreadClass.stop()
            self.SerialThreadClass.wait(2000)

        try:
            # Close existing connection
            s0.close()

            # Configure and open new connection
            s0.serial.port = serialPort
            s0.serial.baudrate = int(baudrate)
            s0.serial.timeout = 1
            s0.open()

            # Create and start new serial thread
            self.SerialThreadClass = SerialThreadClass(self.firmware_type)
            self.SerialThreadClass.serialSignal.connect(self.updateConsole)
            self.SerialThreadClass.start()

            # Request immediate position update to initialize tracking variables
            if self.firmware_type == FIRMWARE_RRF:
                s0.write("M114\n".encode('UTF-8'))

            logger.info(f"✓ Connected to {serialPort} at {baudrate} baud")
            logger.info(f"✓ Serial thread started (Firmware: {self.firmware_type})")
            logger.info(f"✓ Requesting position update to initialize differential tracking")
            print(f"Connected to {serialPort} at {baudrate} baud")
        except Exception as e:
            print(f"Error opening serial port: {e}")
            msgBox = QtWidgets.QMessageBox()
            msgBox.setIcon(QtWidgets.QMessageBox.Critical)
            msgBox.setText(f"Failed to connect to serial port:\n{str(e)}")
            msgBox.setWindowTitle("Connection Error")
            msgBox.exec_()

    def serialDisconnected(self):
        self.RobotStateDisplay.setStyleSheet('background-color: rgb(255, 0, 0)')
        self.RobotStateDisplay.setText("Disconnected")

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

        if dataRead=="SERIAL-DISCONNECTED":
            s0.close()
            self.serialDisconnected()
            print ("Serial Connection Lost")

        else:
            # Handle endstop responses
            if self.firmware_type == FIRMWARE_RRF and isEndstopResponse:
                self.updateEndstopDisplay(dataRead)
                if verboseShow:
                    self.ConsoleOutput.appendPlainText(dataRead)
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
        else:
            self.noSerialConnection()

    def updateFKPosDisplay(self,dataRead):
        if self.firmware_type == FIRMWARE_RRF:
            # RRF: Parse M114 response format: X:10.000 Y:10.000 Z:0.000 U:0.028 V:-110.000 W:-290.000 E:0.000 ...
            try:
                # Extract position values using simple string parsing
                parts = dataRead.split()
                pos_dict = {}
                for part in parts:
                    if ':' in part:
                        axis, value = part.split(':')
                        try:
                            pos_dict[axis] = float(value)
                        except ValueError:
                            continue

                # Extract axis positions
                if 'V' in pos_dict and 'W' in pos_dict:
                    motor_x = pos_dict.get('X', 0.0)  # Drive 0 (X axis) - Art1
                    motor_y = pos_dict.get('Y', 0.0)  # Drives 1+2 (Y axis) - Art2 (coupled)
                    motor_z = pos_dict.get('Z', 0.0)  # Drive 3 (Z axis) - Art3
                    motor_u = pos_dict.get('U', 0.0)  # Drive 4 (U axis) - Art4
                    motor_v = pos_dict.get('V', 0.0)  # Drive 5 (V axis) - Differential
                    motor_w = pos_dict.get('W', 0.0)  # Drive 6 (W axis) - Differential

                    # INVERSE DIFFERENTIAL: Convert motor positions to joint angles
                    # Art5 = (Motor_V - Motor_W) / 2
                    # Art6 = (Motor_V + Motor_W) / 2
                    art5 = (motor_v - motor_w) / 2.0
                    art6 = (motor_v + motor_w) / 2.0

                    # Update tracked motor positions and desired joint positions for differential control
                    self.current_motor_v = motor_v
                    self.current_motor_w = motor_w
                    self.desired_art5 = art5
                    self.desired_art6 = art6

                    logger.debug(f"Position feedback - X:{motor_x:.2f} Y:{motor_y:.2f} Z:{motor_z:.2f} U:{motor_u:.2f} V:{motor_v:.2f} W:{motor_w:.2f}")
                    logger.info(f"  Art1<-X:{motor_x:.2f}° | Art2<-Y:{motor_y:.2f}° | Art3<-Z:{motor_z:.2f}° | Art4<-U:{motor_u:.2f}° | Art5(calc):{art5:.2f}° | Art6(calc):{art6:.2f}°")

                    # Display positions
                    self.FKCurrentPosValueArt1.setText(f"{motor_x:.2f}º")  # X → Art1
                    self.FKCurrentPosValueArt2.setText(f"{motor_y:.2f}º")  # Y → Art2
                    self.FKCurrentPosValueArt3.setText(f"{motor_z:.2f}º")  # Z → Art3
                    self.FKCurrentPosValueArt4.setText(f"{motor_u:.2f}º")  # U → Art4
                    self.FKCurrentPosValueArt5.setText(f"{art5:.2f}º")     # Differential inverse → Art5
                    self.FKCurrentPosValueArt6.setText(f"{art6:.2f}º")     # Differential inverse → Art6

                    # Set status to Idle since M114 doesn't provide status
                    self.updateCurrentState("Idle")
            except (ValueError, KeyError, IndexError) as e:
                logger.error(f"Error parsing M114 response: {e}")
                logger.debug(f"Problematic data: {dataRead}")
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
        """Parse M119 endstop response and update GUI displays"""
        # Format: "Endstops - X: at min stop, Y: not stopped, Z: not stopped, U: not stopped, V: at min stop, W: at min stop, Z probe: at min stop"
        try:
            # Remove "Endstops - " prefix
            endstop_data = dataRead.replace("Endstops - ", "")

            # Split by comma and parse each endstop
            endstops = {}
            for part in endstop_data.split(','):
                part = part.strip()
                if ':' in part:
                    axis, status = part.split(':', 1)
                    axis = axis.strip()
                    status = status.strip()
                    endstops[axis] = status

            # Update GUI labels with color coding
            # Green = not triggered, Red = triggered
            # Mapping: X->Art1, Y->Art2, Z->Art3, U->Art4, V->Art5, W->Art6
            def updateLabel(label, axis_name, art_name):
                if axis_name in endstops:
                    status = endstops[axis_name]
                    label.setText(f"{art_name} ({axis_name}): {status}")
                    if "not stopped" in status:
                        label.setStyleSheet("background-color: rgb(0, 255, 0); padding: 3px;")  # Green
                    else:
                        label.setStyleSheet("background-color: rgb(255, 0, 0); padding: 3px;")  # Red

            updateLabel(self.endstopLabelX, "X", "Art1")
            updateLabel(self.endstopLabelY, "Y", "Art2")
            updateLabel(self.endstopLabelZ, "Z", "Art3")
            updateLabel(self.endstopLabelU, "U", "Art4")
            updateLabel(self.endstopLabelV, "V", "Art5")
            updateLabel(self.endstopLabelW, "W", "Art6")

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

    def __init__(self, firmware_type, parent=None):
        super(SerialThreadClass, self).__init__(parent)
        self.firmware_type = firmware_type
        self.running = True
        self.elapsedTime = time.time()
        self.endstopCheckTime = time.time()

    def stop(self):
        """Gracefully stop the thread"""
        self.running = False

    def run(self):
        """Main thread loop with proper error handling and exit condition"""
        while self.running:
            if not s0.isOpen():
                time.sleep(0.1)
                continue

            try:
                # Check if connection is still alive
                try:
                    s0.inWaiting()
                except (OSError, serial.SerialException):
                    self.serialSignal.emit("SERIAL-DISCONNECTED")
                    print("Lost Serial connection!")
                    break

                # Send status request every 100ms
                current_time = time.time()
                if current_time - self.elapsedTime > 0.1:
                    self.elapsedTime = current_time
                    try:
                        if self.firmware_type == FIRMWARE_RRF:
                            s0.write("M114\n".encode('UTF-8'))  # Use M114 instead of M408 for position feedback
                        else:  # GRBL
                            s0.write("?\n".encode('UTF-8'))
                    except (OSError, serial.SerialException) as e:
                        print(f"Error sending status request: {e}")
                        self.serialSignal.emit("SERIAL-DISCONNECTED")
                        break

                # Send endstop status request every 500ms
                if current_time - self.endstopCheckTime > 0.5:
                    self.endstopCheckTime = current_time
                    try:
                        if self.firmware_type == FIRMWARE_RRF:
                            s0.write("M119\n".encode('UTF-8'))  # Request endstop status
                    except (OSError, serial.SerialException) as e:
                        print(f"Error sending endstop request: {e}")
                        self.serialSignal.emit("SERIAL-DISCONNECTED")
                        break

                # Read response
                try:
                    dataBytes = s0.readline()
                    if dataBytes:
                        # Decode bytes to string and strip whitespace
                        dataCropped = dataBytes.decode('UTF-8', errors='replace').strip()
                        if dataCropped:
                            self.serialSignal.emit(dataCropped)
                except (OSError, serial.SerialException) as e:
                    print(f"Error reading from serial: {e}")
                    self.serialSignal.emit("SERIAL-DISCONNECTED")
                    break

            except Exception as e:
                print(f"Unexpected error in serial thread: {e}")
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

    # Create GUI instance
    prog = BifrostGUI(mwindow)

    # Link gui instance to window for cleanup
    mwindow.gui_instance = prog

    mwindow.show()
    sys.exit(app.exec_())
