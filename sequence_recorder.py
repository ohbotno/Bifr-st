"""
Sequence Recorder module for Thor Robot Arm
Records, saves, loads, and plays back robot movement sequences
"""

import json
import time
import logging
from datetime import datetime
from typing import List, Dict, Optional, Callable, Tuple

logger = logging.getLogger(__name__)


class SequencePoint:
    """Represents a single point in a robot movement sequence"""

    def __init__(self, q1: float, q2: float, q3: float, q4: float, q5: float, q6: float, gripper: float = 0, timestamp: Optional[float] = None, delay: float = 0):
        """
        Args:
            q1-q6: Joint angles in degrees
            gripper: Gripper position (0-100)
            timestamp: Time when point was recorded (auto-generated if None)
            delay: Delay in seconds before moving to this point during playback
        """
        self.q1 = q1
        self.q2 = q2
        self.q3 = q3
        self.q4 = q4
        self.q5 = q5
        self.q6 = q6
        self.gripper = gripper
        self.timestamp = timestamp if timestamp else time.time()
        self.delay = delay

    def to_dict(self) -> Dict[str, float]:
        """Convert to dictionary for JSON serialization"""
        return {
            'q1': self.q1,
            'q2': self.q2,
            'q3': self.q3,
            'q4': self.q4,
            'q5': self.q5,
            'q6': self.q6,
            'gripper': self.gripper,
            'timestamp': self.timestamp,
            'delay': self.delay
        }

    @classmethod
    def from_dict(cls, data: Dict[str, float]) -> 'SequencePoint':
        """Create from dictionary"""
        return cls(
            q1=data['q1'],
            q2=data['q2'],
            q3=data['q3'],
            q4=data['q4'],
            q5=data['q5'],
            q6=data['q6'],
            gripper=data.get('gripper', 0),
            timestamp=data.get('timestamp'),
            delay=data.get('delay', 0)
        )

    def __str__(self) -> str:
        return f"Point[q1={self.q1:.1f}°, q2={self.q2:.1f}°, q3={self.q3:.1f}°, q4={self.q4:.1f}°, q5={self.q5:.1f}°, q6={self.q6:.1f}°, grip={self.gripper}]"


class Sequence:
    """Represents a complete movement sequence"""

    def __init__(self, name: str = "Untitled Sequence", description: str = ""):
        self.name = name
        self.description = description
        self.points: List[SequencePoint] = []
        self.created_at = datetime.now().isoformat()
        self.modified_at = self.created_at

    def add_point(self, point: SequencePoint):
        """Add a point to the sequence"""
        self.points.append(point)
        self.modified_at = datetime.now().isoformat()
        logger.info(f"Added point {len(self.points)} to sequence '{self.name}': {point}")

    def remove_point(self, index: int) -> bool:
        """Remove a point by index"""
        if 0 <= index < len(self.points):
            removed = self.points.pop(index)
            self.modified_at = datetime.now().isoformat()
            logger.info(f"Removed point {index+1} from sequence '{self.name}': {removed}")
            return True
        return False

    def clear(self) -> None:
        """Clear all points"""
        self.points.clear()
        self.modified_at = datetime.now().isoformat()
        logger.info(f"Cleared all points from sequence '{self.name}'")

    def get_point(self, index: int) -> Optional[SequencePoint]:
        """Get a point by index"""
        if 0 <= index < len(self.points):
            return self.points[index]
        return None

    def update_point(self, index: int, point: SequencePoint) -> bool:
        """Update a point at index"""
        if 0 <= index < len(self.points):
            self.points[index] = point
            self.modified_at = datetime.now().isoformat()
            logger.info(f"Updated point {index+1} in sequence '{self.name}': {point}")
            return True
        return False

    def get_duration(self) -> float:
        """Calculate total sequence duration in seconds"""
        if len(self.points) < 2:
            return 0
        return sum(p.delay for p in self.points)

    def to_dict(self) -> Dict[str, any]:
        """Convert to dictionary for JSON serialization"""
        return {
            'name': self.name,
            'description': self.description,
            'created_at': self.created_at,
            'modified_at': self.modified_at,
            'points': [p.to_dict() for p in self.points]
        }

    @classmethod
    def from_dict(cls, data: Dict[str, any]) -> 'Sequence':
        """Create from dictionary"""
        seq = cls(
            name=data.get('name', 'Untitled Sequence'),
            description=data.get('description', '')
        )
        seq.created_at = data.get('created_at', seq.created_at)
        seq.modified_at = data.get('modified_at', seq.modified_at)
        seq.points = [SequencePoint.from_dict(p) for p in data.get('points', [])]
        return seq

    def __len__(self) -> int:
        return len(self.points)

    def __str__(self) -> str:
        return f"Sequence '{self.name}' ({len(self.points)} points, {self.get_duration():.1f}s)"


class SequenceRecorder:
    """Manages sequence recording and playback"""

    def __init__(self) -> None:
        self.current_sequence = Sequence()
        self.is_recording = False
        self.recording_start_time = None

    def start_recording(self, sequence_name: str = "New Sequence") -> None:
        """Start recording a new sequence"""
        self.current_sequence = Sequence(name=sequence_name)
        self.is_recording = True
        self.recording_start_time = time.time()
        logger.info(f"Started recording sequence: {sequence_name}")

    def stop_recording(self) -> Sequence:
        """Stop recording"""
        self.is_recording = False
        logger.info(f"Stopped recording. Sequence has {len(self.current_sequence)} points")
        return self.current_sequence

    def record_point(self, q1: float, q2: float, q3: float, q4: float, q5: float, q6: float, gripper: float = 0, delay: float = 0) -> bool:
        """Record current robot position"""
        if not self.is_recording:
            logger.warning("Attempted to record point while not recording")
            return False

        point = SequencePoint(q1, q2, q3, q4, q5, q6, gripper, delay=delay)
        self.current_sequence.add_point(point)
        return True

    def save_sequence(self, filepath: str, sequence: Optional[Sequence] = None) -> bool:
        """Save sequence to JSON file"""
        if sequence is None:
            sequence = self.current_sequence

        try:
            with open(filepath, 'w') as f:
                json.dump(sequence.to_dict(), f, indent=2)
            logger.info(f"Saved sequence '{sequence.name}' to {filepath}")
            return True
        except Exception as e:
            logger.error(f"Failed to save sequence: {e}")
            return False

    def load_sequence(self, filepath: str) -> Optional[Sequence]:
        """Load sequence from JSON file"""
        try:
            with open(filepath, 'r') as f:
                data = json.load(f)
            sequence = Sequence.from_dict(data)
            logger.info(f"Loaded sequence '{sequence.name}' from {filepath} ({len(sequence)} points)")
            return sequence
        except Exception as e:
            logger.error(f"Failed to load sequence: {e}")
            return None

    def get_current_sequence(self) -> Sequence:
        """Get the current sequence"""
        return self.current_sequence

    def set_current_sequence(self, sequence: Sequence) -> None:
        """Set the current sequence"""
        self.current_sequence = sequence
        logger.info(f"Set current sequence to: {sequence}")


class SequencePlayer:
    """Handles playback of sequences with speed control and looping

    This class is designed to work with Qt's event loop via QTimer.
    Do NOT use threading - use playNextPoint() with QTimer instead.
    """

    def __init__(self, move_callback: Callable[[float, float, float, float, float, float, float], None]) -> None:
        """
        Args:
            move_callback: Function to call for each movement point
                          Signature: callback(q1, q2, q3, q4, q5, q6, gripper) -> None
        """
        self.move_callback = move_callback
        self.is_playing = False
        self.is_paused = False
        self.speed_multiplier = 1.0
        self.loop_enabled = False
        self.current_point_index = 0
        self.current_sequence = None
        self.last_move_time = 0

    def start_playback(self, sequence: Sequence, speed: float = 1.0, loop: bool = False) -> None:
        """
        Start playback of a sequence

        NOTE: This does NOT block. Use with QTimer to call playNextPoint() periodically.

        Args:
            sequence: Sequence to play
            speed: Speed multiplier (1.0 = normal, 2.0 = double speed, 0.5 = half speed)
            loop: Whether to loop the sequence
        """
        self.is_playing = True
        self.is_paused = False
        self.speed_multiplier = speed
        self.loop_enabled = loop
        self.current_point_index = 0
        self.current_sequence = sequence
        self.last_move_time = time.time()

        logger.info(f"Starting playback of '{sequence.name}' (speed={speed}x, loop={loop})")

    def playNextPoint(self) -> Tuple[bool, int, int]:
        """
        Play the next point in the sequence (called by QTimer)

        Returns:
            tuple: (should_continue, point_index, total_points) or (False, 0, 0) if done
        """
        if not self.is_playing or self.current_sequence is None:
            return (False, 0, 0)

        if self.is_paused:
            return (True, self.current_point_index, len(self.current_sequence))

        # Check if we're done with the sequence
        if self.current_point_index >= len(self.current_sequence):
            if self.loop_enabled:
                logger.info(f"Looping sequence '{self.current_sequence.name}'")
                self.current_point_index = 0
                self.last_move_time = time.time()
            else:
                self.is_playing = False
                logger.info(f"Playback complete: '{self.current_sequence.name}'")
                return (False, len(self.current_sequence), len(self.current_sequence))

        # Get current point
        point = self.current_sequence.points[self.current_point_index]

        # Check if delay has elapsed
        current_time = time.time()
        adjusted_delay = point.delay / self.speed_multiplier
        time_since_last_move = current_time - self.last_move_time

        if time_since_last_move < adjusted_delay:
            # Still waiting for delay
            return (True, self.current_point_index, len(self.current_sequence))

        # Execute movement
        logger.info(f"Moving to point {self.current_point_index + 1}/{len(self.current_sequence)}: {point}")
        self.move_callback(
            point.q1, point.q2, point.q3,
            point.q4, point.q5, point.q6,
            point.gripper
        )

        # Move to next point
        self.last_move_time = current_time
        self.current_point_index += 1

        return (True, self.current_point_index, len(self.current_sequence))

    def stop(self) -> None:
        """Stop playback"""
        self.is_playing = False
        self.is_paused = False
        logger.info("Playback stopped by user")

    def pause(self) -> None:
        """Pause playback"""
        if self.is_playing:
            self.is_paused = True
            logger.info("Playback paused")

    def resume(self) -> None:
        """Resume playback"""
        if self.is_playing and self.is_paused:
            self.is_paused = False
            logger.info("Playback resumed")

    def set_speed(self, speed: float):
        """Set playback speed multiplier"""
        self.speed_multiplier = max(0.1, min(10.0, speed))  # Clamp between 0.1x and 10x
        logger.info(f"Playback speed set to {self.speed_multiplier}x")


if __name__ == "__main__":
    # Test the sequence recorder
    logging.basicConfig(level=logging.INFO)

    print("Testing Sequence Recorder\n")

    # Create recorder
    recorder = SequenceRecorder()

    # Start recording
    recorder.start_recording("Test Sequence")

    # Record some points
    recorder.record_point(0, 90, 45, 0, 0, 0, gripper=50, delay=1.0)
    recorder.record_point(45, 80, 60, 10, 20, 30, gripper=75, delay=1.5)
    recorder.record_point(90, 70, 75, -10, -20, 45, gripper=100, delay=2.0)

    # Stop recording
    sequence = recorder.stop_recording()
    print(f"\n{sequence}")
    print(f"Duration: {sequence.get_duration()}s\n")

    # Print all points
    for i, point in enumerate(sequence.points):
        print(f"  {i+1}. {point}")

    # Save to file
    test_file = "test_sequence.json"
    if recorder.save_sequence(test_file):
        print(f"\nSaved to {test_file}")

    # Load from file
    loaded = recorder.load_sequence(test_file)
    if loaded:
        print(f"\nLoaded: {loaded}")

    # Test editing
    sequence.remove_point(1)
    print(f"\nAfter removing point 2: {len(sequence)} points")

    print("\nSequence recorder test complete!")
