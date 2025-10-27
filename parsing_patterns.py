"""
Parsing Patterns Module
Centralized regex patterns for parsing RepRapFirmware responses
All patterns are compiled once at module load for optimal performance
"""

import re
import logging

logger = logging.getLogger(__name__)

# ========== RepRapFirmware (RRF) Response Patterns ==========

# M114 Position Response Pattern
# Format example: "X:10.000 Y:10.000 Z:0.000 U:0.028 V:-110.000 W:-290.000"
# Captures axis letter and signed floating point value
RRF_POSITION_PATTERN = re.compile(r'([XYZUVW]):([+-]?\d+\.?\d*)')

# M119 Endstop Status Pattern
# Format example: "Endstops - X: at min stop, Y: not stopped, Z: not stopped, ..."
# Captures axis letter and status text
ENDSTOP_PATTERN = re.compile(r'([XYZUVW]):\s*([^,]+)')

# OK Response Pattern (simple lowercase check is faster than regex)
# Used to detect command completion
# Note: We don't use a regex for this since "ok" in dataRead.lower() is faster


def parse_m114_response(response_string: str) -> dict:
    """
    Parse M114 position response string

    Args:
        response_string: Raw M114 response from firmware

    Returns:
        Dictionary mapping axis letters to float values, or empty dict if parse failed
        Example: {'X': 10.0, 'Y': 20.5, 'Z': 0.0, 'U': 0.028, 'V': -110.0, 'W': -290.0}
    """
    matches = RRF_POSITION_PATTERN.findall(response_string)
    if not matches:
        logger.debug(f"Failed to parse M114 response: {response_string}")
        return {}

    return {axis: float(value) for axis, value in matches}


def parse_m119_response(response_string: str) -> dict:
    """
    Parse M119 endstop status response string

    Args:
        response_string: Raw M119 response from firmware

    Returns:
        Dictionary mapping axis letters to status strings, or empty dict if parse failed
        Example: {'X': 'at min stop', 'Y': 'not stopped', 'Z': 'not stopped', ...}
    """
    matches = ENDSTOP_PATTERN.findall(response_string)
    if not matches:
        logger.debug(f"Failed to parse M119 response: {response_string}")
        return {}

    return {axis: status.strip() for axis, status in matches}


def is_m114_response(response_string: str) -> bool:
    """
    Check if response string is an M114 position response

    Args:
        response_string: Raw response from firmware

    Returns:
        True if this looks like an M114 response
    """
    return response_string.startswith("X:") and "Y:" in response_string


def is_m119_response(response_string: str) -> bool:
    """
    Check if response string is an M119 endstop status response

    Args:
        response_string: Raw response from firmware

    Returns:
        True if this looks like an M119 response
    """
    return response_string.startswith("Endstops -")


def is_ok_response(response_string: str) -> bool:
    """
    Check if response string is an 'ok' confirmation

    Args:
        response_string: Raw response from firmware

    Returns:
        True if response contains 'ok'
    """
    return "ok" in response_string.lower()


if __name__ == "__main__":
    # Test the parsing functions
    import sys
    logging.basicConfig(level=logging.DEBUG, stream=sys.stdout)

    print("Parsing Patterns Test")
    print("=" * 60)

    # Test M114 parsing
    print("\nTest 1: M114 Position Response")
    test_m114 = "X:10.500 Y:20.000 Z:-5.250 U:0.028 V:-110.000 W:-290.000"
    positions = parse_m114_response(test_m114)
    print(f"  Input:  {test_m114}")
    print(f"  Output: {positions}")
    assert positions == {'X': 10.5, 'Y': 20.0, 'Z': -5.25, 'U': 0.028, 'V': -110.0, 'W': -290.0}
    assert is_m114_response(test_m114)

    # Test M119 parsing
    print("\nTest 2: M119 Endstop Status Response")
    test_m119 = "Endstops - X: at min stop, Y: not stopped, Z: not stopped, U: not stopped, V: at min stop, W: at min stop"
    endstops = parse_m119_response(test_m119)
    print(f"  Input:  {test_m119}")
    print(f"  Output: {endstops}")
    assert endstops == {'X': 'at min stop', 'Y': 'not stopped', 'Z': 'not stopped', 'U': 'not stopped', 'V': 'at min stop', 'W': 'at min stop'}
    assert is_m119_response(test_m119)

    # Test ok response
    print("\nTest 3: OK Response")
    test_ok = "ok"
    print(f"  Input:  {test_ok}")
    print(f"  Is OK:  {is_ok_response(test_ok)}")
    assert is_ok_response(test_ok)

    # Test invalid responses
    print("\nTest 4: Invalid Responses")
    invalid = "Some random text"
    print(f"  Input:      {invalid}")
    print(f"  Is M114:    {is_m114_response(invalid)}")
    print(f"  Is M119:    {is_m119_response(invalid)}")
    print(f"  Parsed M114: {parse_m114_response(invalid)}")
    print(f"  Parsed M119: {parse_m119_response(invalid)}")
    assert not is_m114_response(invalid)
    assert not is_m119_response(invalid)
    assert parse_m114_response(invalid) == {}
    assert parse_m119_response(invalid) == {}

    print("\n" + "=" * 60)
    print("All tests passed!")
