import sys
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtGui import *
from PyQt5.QtCore import *

from gui import Ui_MainWindow
from about import Ui_Dialog as About_Ui_Dialog


import serial_port_finder as spf
import inverse_kinematics as ik

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

class AboutDialog(About_Ui_Dialog):
    def __init__(self, dialog):
        About_Ui_Dialog.__init__(self)
        self.setupUi(dialog)

class BifrostGUI(Ui_MainWindow):
    def __init__(self, dialog):
        Ui_MainWindow.__init__(self)
        self.setupUi(dialog)

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
        logger.info("Art1 (Joint 1) -> W axis -> Drive 6")
        logger.info("Art2 (Joint 2) -> U+V axes -> Drives 4+5 (COUPLED for torque)")
        logger.info("Art3 (Joint 3) -> Z axis -> Drive 3")
        logger.info("Art4 (Joint 4) -> A axis -> Drive 2")
        logger.info("Art5 (Joint 5) -> DIFFERENTIAL -> Drives 0+1")
        logger.info("Art6 (Joint 6) -> DIFFERENTIAL -> Drives 0+1")
        logger.info("  DIFFERENTIAL: Motor_X(D0) = Art6+Art5, Motor_Y(D1) = Art6-Art5")
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
            message=typeOfMovement + "W" + str(joint_value) + feedRate
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
        # COUPLED MOTORS: Art2 uses Drives 4+5 (U+V axes) for more torque
        # Both motors receive the same command
        joint_value = self.SpinBoxArt2.value()
        logger.info(f"Art2 (Joint 2 - COUPLED) commanded to: {joint_value}° -> Axes: U+V (Drives 4+5)")
        if s0.isOpen():
            if self.G1MoveRadioButton.isChecked():
                typeOfMovement="G1 "
                feedRate=" F" + str(self.FeedRateInput.value())
            else:
                typeOfMovement="G0 "
                feedRate=""
            # Send same command to both U and V axes
            message=typeOfMovement + "U" + str(joint_value) + " V" + str(joint_value) + feedRate
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
        logger.info(f"Art4 (Joint 4) commanded to: {joint_value}° -> Axis: A")
        if s0.isOpen():
            if self.G1MoveRadioButton.isChecked():
                typeOfMovement="G1 "
                feedRate=" F" + str(self.FeedRateInput.value())
            else:
                typeOfMovement="G0 "
                feedRate=""
            message=typeOfMovement + "A" + str(joint_value) + feedRate
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
        # Motors: Drive0 (X axis) and Drive1 (Y axis)
        # Forward kinematics: Motor_X = Art6 + Art5, Motor_Y = Art6 - Art5

        art5_value = self.SpinBoxArt5.value()
        art6_value = self.SpinBoxArt6.value()

        # Calculate differential motor positions
        motor_x = art6_value + art5_value  # Drive 0 (X axis)
        motor_y = art6_value - art5_value  # Drive 1 (Y axis)

        logger.info(f"Art5 (DIFFERENTIAL) commanded to: {art5_value}°")
        logger.info(f"  Differential calculation: Motor_X={motor_x:.2f}° Motor_Y={motor_y:.2f}°")

        if s0.isOpen():
            if self.G1MoveRadioButton.isChecked():
                typeOfMovement="G1 "
                feedRate=" F" + str(self.FeedRateInput.value())
            else:
                typeOfMovement="G0 "
                feedRate=""
            # Send both motor commands for differential
            message=typeOfMovement + "X" + str(motor_x) + " Y" + str(motor_y) + feedRate
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
        # Motors: Drive0 (X axis) and Drive1 (Y axis)
        # Forward kinematics: Motor_X = Art6 + Art5, Motor_Y = Art6 - Art5

        art5_value = self.SpinBoxArt5.value()
        art6_value = self.SpinBoxArt6.value()

        # Calculate differential motor positions
        motor_x = art6_value + art5_value  # Drive 0 (X axis)
        motor_y = art6_value - art5_value  # Drive 1 (Y axis)

        logger.info(f"Art6 (DIFFERENTIAL) commanded to: {art6_value}°")
        logger.info(f"  Differential calculation: Motor_X={motor_x:.2f}° Motor_Y={motor_y:.2f}°")

        if s0.isOpen():
            if self.G1MoveRadioButton.isChecked():
                typeOfMovement="G1 "
                feedRate=" F" + str(self.FeedRateInput.value())
            else:
                typeOfMovement="G0 "
                feedRate=""
            # Send both motor commands for differential
            message=typeOfMovement + "X" + str(motor_x) + " Y" + str(motor_y) + feedRate
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
            motor_x = art6 + art5  # Drive 0 (X axis) for differential
            motor_y = art6 - art5  # Drive 1 (Y axis) for differential

            logger.info(f"MoveAll: Art5={art5}° Art6={art6}° → Differential: X={motor_x:.2f}° Y={motor_y:.2f}°")

            if self.G1MoveRadioButton.isChecked():
                typeOfMovement="G1 "
                feedRate=" F" + str(self.FeedRateInput.value())
            else:
                typeOfMovement="G0 "
                feedRate=""

            # Map all axes: W=Art1, U+V=Art2(coupled), Z=Art3, A=Art4, X/Y=differential(Art5,Art6)
            message=typeOfMovement + "W" + str(art1) + " U" + str(art2) + " V" + str(art2) + " Z" + str(art3) + " A" + str(art4) + " X" + str(motor_x) + " Y" + str(motor_y) + feedRate
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

            logger.info(f"✓ Connected to {serialPort} at {baudrate} baud")
            logger.info(f"✓ Serial thread started (Firmware: {self.firmware_type})")
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
            # RRF: Check if response is JSON status
            isDataReadVerbose = dataRead.startswith("{") and "status" in dataRead
            isDataOkResponse = "ok" in dataRead.lower()
        else:  # GRBL
            isDataReadVerbose = "MPos" in dataRead
            isDataOkResponse = "ok" in dataRead

        if dataRead=="SERIAL-DISCONNECTED":
            s0.close()
            self.serialDisconnected()
            print ("Serial Connection Lost")

        else:
            if not isDataReadVerbose and not isDataOkResponse:
                self.ConsoleOutput.appendPlainText(dataRead)
            elif isDataOkResponse and okShow:
                self.ConsoleOutput.appendPlainText(dataRead)
            elif isDataReadVerbose:
                self.updateFKPosDisplay(dataRead)
                if verboseShow:
                    self.ConsoleOutput.appendPlainText(dataRead)

    def sendSerialCommand(self):
        messageToSent=self.ConsoleInput.text()+"\n"
        messageToConsole= ">>> "+self.ConsoleInput.text()
        if s0.isOpen():
            if messageToSent!="":
                s0.write(messageToSent.encode('UTF-8'))
                self.ConsoleOutput.appendPlainText(messageToConsole)
                self.ConsoleInput.clear()
        else:
            self.noSerialConnection()

    def updateFKPosDisplay(self,dataRead):
        if self.firmware_type == FIRMWARE_RRF:
            # RRF: Parse JSON response
            try:
                data = json.loads(dataRead)
                # RRF status: I=idle, P=printing, S=stopped, C=config, A=paused, D=pausing, R=resuming, B=busy
                status_map = {'I': 'Idle', 'P': 'Run', 'S': 'Stopped', 'C': 'Config',
                             'A': 'Paused', 'D': 'Pausing', 'R': 'Resuming', 'B': 'Busy'}
                status = status_map.get(data.get('status', ''), 'Unknown')
                self.updateCurrentState(status)

                # Get axis positions from coords.xyz array
                # RRF returns axes in order: X, Y, Z, U, V, W, A, B, C (as many as configured)
                if 'coords' in data and 'xyz' in data['coords']:
                    pos = data['coords']['xyz']
                    if len(pos) >= 7:  # Need at least 7 axes (X,Y,Z,U,V,W,A)
                        # Raw motor positions from firmware
                        motor_x = pos[0]  # Drive 0 (X axis) - Differential
                        motor_y = pos[1]  # Drive 1 (Y axis) - Differential
                        motor_z = pos[2]  # Drive 3 (Z axis) - Art3
                        motor_u = pos[3]  # Drive 4 (U axis) - Art2 (coupled)
                        motor_v = pos[4]  # Drive 5 (V axis) - Art2 (coupled)
                        motor_w = pos[5]  # Drive 6 (W axis) - Art1
                        motor_a = pos[6]  # Drive 2 (A axis) - Art4

                        # INVERSE DIFFERENTIAL: Convert motor positions to joint angles
                        # Art5 = (Motor_X - Motor_Y) / 2
                        # Art6 = (Motor_X + Motor_Y) / 2
                        art5 = (motor_x - motor_y) / 2.0
                        art6 = (motor_x + motor_y) / 2.0

                        # Art2 uses coupled motors (U+V), take average for display
                        art2 = (motor_u + motor_v) / 2.0

                        logger.debug(f"Position feedback - X:{motor_x:.2f} Y:{motor_y:.2f} Z:{motor_z:.2f} U:{motor_u:.2f} V:{motor_v:.2f} W:{motor_w:.2f} A:{motor_a:.2f}")
                        logger.info(f"  Art1<-W:{motor_w:.2f}° | Art2<-U+V(avg):{art2:.2f}° | Art3<-Z:{motor_z:.2f}° | Art4<-A:{motor_a:.2f}° | Art5(calc):{art5:.2f}° | Art6(calc):{art6:.2f}°")

                        # Display positions
                        self.FKCurrentPosValueArt1.setText(f"{motor_w:.2f}º")  # W → Art1
                        self.FKCurrentPosValueArt2.setText(f"{art2:.2f}º")     # U+V average → Art2
                        self.FKCurrentPosValueArt3.setText(f"{motor_z:.2f}º")  # Z → Art3
                        self.FKCurrentPosValueArt4.setText(f"{motor_a:.2f}º")  # A → Art4
                        self.FKCurrentPosValueArt5.setText(f"{art5:.2f}º")     # Differential inverse → Art5
                        self.FKCurrentPosValueArt6.setText(f"{art6:.2f}º")     # Differential inverse → Art6
            except (json.JSONDecodeError, KeyError, IndexError) as e:
                logger.error(f"Error parsing RRF status: {e}")
                logger.debug(f"Problematic data (first 500 chars): {dataRead[:500]}")
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
                            s0.write("M408 S0\n".encode('UTF-8'))
                        else:  # GRBL
                            s0.write("?\n".encode('UTF-8'))
                    except (OSError, serial.SerialException) as e:
                        print(f"Error sending status request: {e}")
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
