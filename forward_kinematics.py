"""
Forward Kinematics module for Thor Robot Arm
Computes all joint positions and TCP position from joint angles using DH parameters

DH Parameters (from Hackaday Thor project):
Link | θ        | d  | a  | α
-----|----------|----|----|------
1    | q1       | L1 | 0  | 90°
2    | q2-90    | 0  | L2 | 0°
3    | q3       | 0  | 0  | 90°
4    | q4       | L3 | 0  | -90°
5    | q5       | 0  | 0  | 90°
6    | q6       | 0  | 0  | 0°
TCP  | 0        | L4 | 0  | 0°

Reference: https://hackaday.io/project/12989-thor/log/43941-forward-kinematics
"""

from typing import List, Tuple
import numpy as np
import numpy.typing as npt
import logging

logger = logging.getLogger(__name__)

# Thor Robot Link Lengths (in mm)
L1 = 202.00  # Base height
L2 = 160.00  # Upper arm length
L3 = 195.00  # Forearm length
L4 = 67.15   # Wrist to TCP length

# DH Parameters Table
# Each row: [theta_offset, d, a, alpha]
# theta_offset is added to joint angle, already in radians
DH_PARAMS = [
    # Link 1: Base rotation
    [0, L1, 0, np.pi/2],
    # Link 2: Shoulder (note: q2-90 in DH convention)
    [-np.pi/2, 0, L2, 0],
    # Link 3: Elbow
    [0, 0, 0, np.pi/2],
    # Link 4: Wrist roll
    [0, L3, 0, -np.pi/2],
    # Link 5: Wrist pitch
    [0, 0, 0, np.pi/2],
    # Link 6: Wrist yaw
    [0, 0, 0, 0],
    # TCP: Tool center point
    [0, L4, 0, 0]
]


def dh_transform(theta: float, d: float, a: float, alpha: float) -> npt.NDArray[np.float64]:
    """
    Calculate DH transformation matrix

    Args:
        theta: Joint angle (radians)
        d: Link offset
        a: Link length
        alpha: Link twist (radians)

    Returns:
        4x4 homogeneous transformation matrix
    """
    ct = np.cos(theta)
    st = np.sin(theta)
    ca = np.cos(alpha)
    sa = np.sin(alpha)

    return np.array([
        [ct, -st*ca,  st*sa, a*ct],
        [st,  ct*ca, -ct*sa, a*st],
        [0,      sa,     ca,    d],
        [0,       0,      0,    1]
    ])


def compute_all_joint_positions(q1: float, q2: float, q3: float, q4: float, q5: float, q6: float) -> List[Tuple[float, float, float]]:
    """
    Compute positions of all joints and TCP using forward kinematics

    Args:
        q1, q2, q3, q4, q5, q6: Joint angles in degrees

    Returns:
        List of 8 tuples [(x, y, z), ...] representing:
        [Base, J1, J2, J3, J4, J5, J6, TCP]

    Note:
        Base is always at (0, 0, 0)
        All positions in mm
    """
    # Convert degrees to radians
    joint_angles_rad = [
        np.radians(q1),
        np.radians(q2),
        np.radians(q3),
        np.radians(q4),
        np.radians(q5),
        np.radians(q6),
        0  # TCP has no rotation
    ]

    # Initialize position list with base at origin
    positions = [(0.0, 0.0, 0.0)]

    # Current transformation matrix (starts at identity)
    T_current = np.eye(4)

    # Compute transformation for each link
    for i, (theta_offset, d, a, alpha) in enumerate(DH_PARAMS):
        # Get joint angle (0 for TCP)
        theta = joint_angles_rad[i] + theta_offset

        # Compute DH transformation for this link
        T_link = dh_transform(theta, d, a, alpha)

        # Accumulate transformation
        T_current = T_current @ T_link

        # Extract position from transformation matrix
        x, y, z = T_current[0:3, 3]
        positions.append((x, y, z))

    logger.debug(f"FK: Computed {len(positions)} joint positions")

    return positions


def compute_tcp_position_only(q1: float, q2: float, q3: float, q4: float, q5: float, q6: float) -> Tuple[float, float, float]:
    """
    Compute only TCP position (faster than computing all joints)

    Args:
        q1, q2, q3, q4, q5, q6: Joint angles in degrees

    Returns:
        Tuple (x, y, z) representing TCP position in mm
    """
    # Convert degrees to radians
    joint_angles_rad = [
        np.radians(q1),
        np.radians(q2),
        np.radians(q3),
        np.radians(q4),
        np.radians(q5),
        np.radians(q6),
        0
    ]

    # Compute full transformation to TCP
    T = np.eye(4)
    for i, (theta_offset, d, a, alpha) in enumerate(DH_PARAMS):
        theta = joint_angles_rad[i] + theta_offset
        T = T @ dh_transform(theta, d, a, alpha)

    # Extract TCP position
    x, y, z = T[0:3, 3]

    return (x, y, z)


def compute_workspace_envelope() -> dict:
    """
    Compute workspace envelope parameters for visualization

    Returns:
        Dictionary with workspace parameters:
        {
            'type': 'cylinder',
            'radius': maximum horizontal reach (mm),
            'height': vertical reach range (mm),
            'z_min': minimum Z position (mm),
            'z_max': maximum Z position (mm)
        }
    """
    # Maximum horizontal reach (fully extended in XY plane)
    # This occurs when q2 and q3 are aligned
    max_horizontal_reach = L2 + L3 + L4

    # Vertical reach
    # Maximum Z: arm pointing straight up
    max_z = L1 + L2 + L3 + L4

    # Minimum Z: arm pointing straight down
    # (limited by self-collision, but theoretically)
    min_z = L1 - L2 - L3 - L4

    # More realistic minimum (arm can't fold completely)
    min_z = max(min_z, 0)

    return {
        'type': 'cylinder',
        'radius': max_horizontal_reach,
        'height': max_z - min_z,
        'z_min': min_z,
        'z_max': max_z,
        'center_x': 0,
        'center_y': 0
    }


def get_home_position():
    """
    Get robot positions for home configuration (all joints at 0°)

    Returns:
        List of 8 tuples [(x, y, z), ...] for home position
    """
    return compute_all_joint_positions(0, 0, 0, 0, 0, 0)


def get_joint_names():
    """
    Get descriptive names for each joint

    Returns:
        List of joint names
    """
    return [
        'Base',
        'J1 (Shoulder Rotation)',
        'J2 (Shoulder Pitch)',
        'J3 (Elbow)',
        'J4 (Wrist Roll)',
        'J5 (Wrist Pitch)',
        'J6 (Wrist Yaw)',
        'TCP'
    ]


# Example usage and testing
if __name__ == '__main__':
    logging.basicConfig(level=logging.DEBUG)

    print("Thor Robot Forward Kinematics Test")
    print("=" * 50)

    # Test home position
    print("\nHome Position (all joints at 0°):")
    home_pos = get_home_position()
    joint_names = get_joint_names()
    for name, (x, y, z) in zip(joint_names, home_pos):
        print(f"  {name:25s}: ({x:7.2f}, {y:7.2f}, {z:7.2f}) mm")

    # Test TCP position only
    print("\nTCP Position (fast calculation):")
    tcp = compute_tcp_position_only(0, 0, 0, 0, 0, 0)
    print(f"  TCP: ({tcp[0]:.2f}, {tcp[1]:.2f}, {tcp[2]:.2f}) mm")

    # Test with some joint angles
    print("\nTest Position (q1=45°, q2=30°, q3=-45°):")
    test_pos = compute_all_joint_positions(45, 30, -45, 0, 0, 0)
    for name, (x, y, z) in zip(joint_names, test_pos):
        print(f"  {name:25s}: ({x:7.2f}, {y:7.2f}, {z:7.2f}) mm")

    # Workspace envelope
    print("\nWorkspace Envelope:")
    workspace = compute_workspace_envelope()
    print(f"  Type: {workspace['type']}")
    print(f"  Radius: {workspace['radius']:.2f} mm")
    print(f"  Height: {workspace['height']:.2f} mm")
    print(f"  Z range: [{workspace['z_min']:.2f}, {workspace['z_max']:.2f}] mm")
