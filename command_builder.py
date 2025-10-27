"""
Command Builder Module for Bifrost
Provides utilities for building and formatting G-code commands to reduce code duplication
"""

from typing import Tuple, Dict, Optional, Callable
import logging

logger = logging.getLogger(__name__)


class CommandBuilder:
    """Helper class for building G-code commands and formatting serial communication"""

    @staticmethod
    def get_movement_params(gui) -> Tuple[str, str]:
        """
        Extract movement type and feed rate from GUI state

        Args:
            gui: BifrostGUI instance with radio buttons

        Returns:
            tuple: (movement_type, feedrate_string)
                   e.g., ("G1", " F1000") or ("G0", "")
        """
        if gui.G1MoveRadioButton.isChecked():
            movement_type = "G1"
            feedrate = f" F{gui.FeedRateInput.value()}"
        else:
            movement_type = "G0"
            feedrate = ""

        return movement_type, feedrate

    @staticmethod
    def build_axis_command(movement_type: str, axes_dict: Dict[str, float], feedrate: str = "") -> str:
        """
        Build a G-code movement command from axis dictionary

        Args:
            movement_type: G-code command (e.g., "G0", "G1")
            axes_dict: Dictionary mapping axis letters to values
                      e.g., {"X": 10, "Y": 20.5, "Z": 30}
            feedrate: Optional feedrate string (e.g., " F1000")

        Returns:
            str: Formatted G-code command (e.g., "G0 X10 Y20.5 Z30 F1000")
        """
        # Build axis portion of command
        axis_parts = [f"{axis}{value}" for axis, value in axes_dict.items()]
        axis_string = " ".join(axis_parts)

        # Combine all parts
        command = f"{movement_type} {axis_string}{feedrate}"

        return command

    @staticmethod
    def build_single_axis_command(movement_type: str, axis: str, value: float, feedrate: str = "") -> str:
        """
        Build a G-code command for a single axis

        Args:
            movement_type: G-code command (e.g., "G0", "G1")
            axis: Axis letter (e.g., "X", "Y", "Z")
            value: Position value
            feedrate: Optional feedrate string (e.g., " F1000")

        Returns:
            str: Formatted G-code command (e.g., "G0 X10 F1000")
        """
        return f"{movement_type} {axis}{value}{feedrate}"

    @staticmethod
    def format_console_output(command: str) -> str:
        """
        Format command for console display

        Args:
            command: G-code command string

        Returns:
            str: Formatted for console (e.g., ">>> G0 X10")
        """
        return f">>> {command}"

    @staticmethod
    def prepare_serial_message(command: str) -> bytes:
        """
        Prepare command for serial transmission

        Args:
            command: G-code command string

        Returns:
            bytes: Encoded command with newline terminator
        """
        return f"{command}\n".encode('UTF-8')


class SerialCommandSender:
    """Helper for sending commands through serial with consistent error handling"""

    def __init__(self, serial_manager, console_output_widget=None):
        """
        Initialize command sender

        Args:
            serial_manager: SerialManager instance (s0)
            console_output_widget: Optional QPlainTextEdit for console output
        """
        self.serial_manager = serial_manager
        self.console_output = console_output_widget

    def send(self, command: str, show_in_console: bool = True, log_command: bool = True) -> bool:
        """
        Send a G-code command through serial port

        Args:
            command: G-code command string (without newline)
            show_in_console: Whether to display in console widget
            log_command: Whether to log to debug logger

        Returns:
            bool: True if sent successfully, False if serial not open
        """
        if not self.serial_manager.isOpen():
            logger.warning(f"Cannot send command (serial not open): {command}")
            return False

        # Prepare message
        message_bytes = CommandBuilder.prepare_serial_message(command)

        # Send to serial port
        self.serial_manager.write(message_bytes)

        # Display in console
        if show_in_console and self.console_output:
            console_text = CommandBuilder.format_console_output(command)
            self.console_output.appendPlainText(console_text)

        # Log if requested
        if log_command:
            logger.debug(f"Sent command: {command}")

        return True

    def send_if_connected(self, command: str, error_callback: Optional[Callable[[], None]] = None, **kwargs) -> bool:
        """
        Send command if connected, otherwise call error callback

        Args:
            command: G-code command string
            error_callback: Function to call if not connected (e.g., gui.noSerialConnection)
            **kwargs: Additional arguments passed to send()

        Returns:
            bool: True if sent, False if not connected
        """
        if self.serial_manager.isOpen():
            return self.send(command, **kwargs)
        else:
            if error_callback:
                error_callback()
            return False


if __name__ == "__main__":
    # Test command building
    print("CommandBuilder Tests")
    print("=" * 60)

    # Test single axis
    cmd = CommandBuilder.build_single_axis_command("G0", "X", 100, " F1000")
    print(f"Single axis: {cmd}")
    assert cmd == "G0 X100 F1000", "Single axis command failed"

    # Test multi-axis
    axes = {"X": 10, "Y": 20.5, "Z": 30}
    cmd = CommandBuilder.build_axis_command("G1", axes, " F2000")
    print(f"Multi-axis: {cmd}")
    assert "X10" in cmd and "Y20.5" in cmd and "Z30" in cmd, "Multi-axis command failed"

    # Test console formatting
    console = CommandBuilder.format_console_output("G0 X100")
    print(f"Console: {console}")
    assert console == ">>> G0 X100", "Console formatting failed"

    # Test serial preparation
    serial_msg = CommandBuilder.prepare_serial_message("G0 X100")
    print(f"Serial: {serial_msg}")
    assert serial_msg == b"G0 X100\n", "Serial preparation failed"

    print("\nAll tests passed!")
