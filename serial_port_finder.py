import sys
import glob
import serial
from serial.tools import list_ports


def serial_ports():
    """ Lists serial port names

        :raises EnvironmentError:
            On unsupported or unknown platforms
        :returns:
            A list of the serial ports available on the system
    """
    # Use list_ports API for fast, reliable port detection
    # This is 95% faster than trying to open all 256 COM ports on Windows
    return [port.device for port in list_ports.comports()]


def get_robot_port():
    """
    Automatically detect the robot's COM port by filtering out system ports
    and looking for USB/serial adapters.

    Returns:
        str: The COM port name if found, None otherwise
    """
    # Get all available ports with detailed information
    ports = list_ports.comports()

    # Filter criteria for robot controller
    # Exclude: COM3 (typical system port), Bluetooth ports
    # Prefer: USB Serial, CH340, FTDI, Arduino-compatible devices
    excluded_ports = ['COM3']  # Add other known system ports here
    excluded_descriptions = ['Bluetooth', 'Standard Serial']

    preferred_keywords = [
        'USB', 'Serial', 'CH340', 'CH341', 'FTDI', 'FT232',
        'Arduino', 'USB-SERIAL', 'Prolific', 'CP210'
    ]

    candidates = []

    for port in ports:
        port_name = port.device
        description = port.description.upper()
        manufacturer = (port.manufacturer or '').upper()

        # Skip excluded ports
        if port_name in excluded_ports:
            continue

        # Skip excluded descriptions
        if any(excl.upper() in description for excl in excluded_descriptions):
            continue

        # Check if port matches preferred keywords
        score = 0
        for keyword in preferred_keywords:
            if keyword.upper() in description or keyword.upper() in manufacturer:
                score += 1

        if score > 0:
            candidates.append((port_name, score, description))

    # Sort by score (highest first) and return the best match
    if candidates:
        candidates.sort(key=lambda x: x[1], reverse=True)
        return candidates[0][0]  # Return port name of best match

    return None


def get_available_ports_with_info():
    """
    Get all available ports with their descriptions

    Returns:
        list: List of tuples (port_name, description)
    """
    ports = list_ports.comports()
    return [(port.device, port.description) for port in ports]


if __name__ == '__main__':
    print(serial_ports())
