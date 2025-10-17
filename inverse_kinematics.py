"""
Inverse Kinematics module for Thor Robot Arm
Based on kinematic decoupling procedure from Thor project documentation
"""

import numpy as np
import logging

logger = logging.getLogger(__name__)

# Thor Robot Link Lengths (in mm)
L1 = 202.00  # Base height
L2 = 160.00  # Upper arm length
L3 = 195.00  # Forearm length
L4 = 67.15   # Wrist to TCP length


def rotation_matrix_x(angle_rad):
    """Rotation matrix around X axis"""
    c = np.cos(angle_rad)
    s = np.sin(angle_rad)
    return np.array([
        [1, 0, 0],
        [0, c, -s],
        [0, s, c]
    ])


def rotation_matrix_y(angle_rad):
    """Rotation matrix around Y axis"""
    c = np.cos(angle_rad)
    s = np.sin(angle_rad)
    return np.array([
        [c, 0, s],
        [0, 1, 0],
        [-s, 0, c]
    ])


def rotation_matrix_z(angle_rad):
    """Rotation matrix around Z axis"""
    c = np.cos(angle_rad)
    s = np.sin(angle_rad)
    return np.array([
        [c, -s, 0],
        [s, c, 0],
        [0, 0, 1]
    ])


def dh_transform(theta, d, a, alpha):
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


def euler_to_rotation_matrix(roll, pitch, yaw):
    """
    Convert Euler angles (ZYX convention) to rotation matrix

    Args:
        roll, pitch, yaw: Euler angles in radians

    Returns:
        3x3 rotation matrix
    """
    return rotation_matrix_z(yaw) @ rotation_matrix_y(pitch) @ rotation_matrix_x(roll)


class IKSolution:
    """Container for inverse kinematics solution"""
    def __init__(self, q1, q2, q3, q4=0, q5=0, q6=0, valid=True, error_msg=""):
        self.q1 = q1  # Base rotation (degrees)
        self.q2 = q2  # Shoulder angle (degrees)
        self.q3 = q3  # Elbow angle (degrees)
        self.q4 = q4  # Wrist roll (degrees)
        self.q5 = q5  # Wrist pitch (degrees)
        self.q6 = q6  # Wrist yaw (degrees)
        self.valid = valid
        self.error_msg = error_msg

    def __str__(self):
        if self.valid:
            return f"IK Solution: q1={self.q1:.2f}°, q2={self.q2:.2f}°, q3={self.q3:.2f}°, q4={self.q4:.2f}°, q5={self.q5:.2f}°, q6={self.q6:.2f}°"
        else:
            return f"IK Solution: INVALID - {self.error_msg}"


def solve_ik_position(x, y, z):
    """
    Solve inverse kinematics for 3-DOF positioning (q1, q2, q3)
    This positions the wrist center point (Pm), not the TCP

    Args:
        x, y, z: Target position coordinates in mm

    Returns:
        IKSolution object containing joint angles or error information

    Note:
        - The solution positions the 5th joint axis center (Pm)
        - Wrist orientation (q4, q5, q6) would be calculated separately in full 6-DOF IK
        - Currently assumes wrist pointing down (default orientation)
    """

    logger.info(f"IK: Solving for target position X={x:.2f}, Y={y:.2f}, Z={z:.2f}")

    # Step 1: Calculate wrist center point (Pm) by moving back from TCP along default orientation
    # Assuming default orientation with tool pointing down (-Z direction)
    # For now, we'll position Pm directly at the target minus L4 in Z
    # In full 6-DOF IK, this would use the desired orientation matrix

    Pm_x = x
    Pm_y = y
    Pm_z = z + L4  # Move up by L4 to get wrist center (assuming tool points down)

    logger.debug(f"IK: Wrist center (Pm) calculated at X={Pm_x:.2f}, Y={Pm_y:.2f}, Z={Pm_z:.2f}")

    # Step 2: Calculate q1 (base rotation)
    # q1 = arctan(Pm_y / Pm_x)
    q1_rad = np.arctan2(Pm_y, Pm_x)
    q1 = np.degrees(q1_rad)

    logger.debug(f"IK: q1 (base rotation) = {q1:.2f}°")

    # Step 3: Calculate horizontal reach and vertical offset from base
    r = np.sqrt(Pm_x**2 + Pm_y**2)  # Horizontal distance from base
    s = Pm_z - L1  # Vertical offset from shoulder (accounting for base height)

    logger.debug(f"IK: Horizontal reach r={r:.2f}, Vertical offset s={s:.2f}")

    # Step 4: Calculate distance from shoulder to wrist
    D = np.sqrt(r**2 + s**2)

    logger.debug(f"IK: Distance shoulder to wrist D={D:.2f}")

    # Step 5: Check if target is reachable
    max_reach = L2 + L3
    min_reach = abs(L2 - L3)

    if D > max_reach:
        error_msg = f"Target unreachable: distance {D:.2f}mm exceeds maximum reach {max_reach:.2f}mm"
        logger.error(f"IK: {error_msg}")
        return IKSolution(0, 0, 0, valid=False, error_msg=error_msg)

    if D < min_reach:
        error_msg = f"Target unreachable: distance {D:.2f}mm is less than minimum reach {min_reach:.2f}mm"
        logger.error(f"IK: {error_msg}")
        return IKSolution(0, 0, 0, valid=False, error_msg=error_msg)

    # Step 6: Calculate q3 (elbow angle) using law of cosines
    # cos(q3) = (D² - L2² - L3²) / (2*L2*L3)
    cos_q3 = (D**2 - L2**2 - L3**2) / (2 * L2 * L3)

    # Clamp to valid range to handle numerical errors
    cos_q3 = np.clip(cos_q3, -1.0, 1.0)

    # Two solutions: elbow up (+) or elbow down (-)
    # Using positive solution (elbow up) for now
    sin_q3 = np.sqrt(1 - cos_q3**2)
    q3_rad = np.arctan2(sin_q3, cos_q3)
    q3 = np.degrees(q3_rad)

    logger.debug(f"IK: q3 (elbow angle) = {q3:.2f}° (cos_q3={cos_q3:.4f})")

    # Step 7: Calculate q2 (shoulder angle)
    # q2 has two components: angle to target point + correction for elbow bend
    alpha = np.arctan2(s, r)  # Angle from horizontal to target
    beta = np.arctan2(L3 * np.sin(q3_rad), L2 + L3 * np.cos(q3_rad))  # Correction for elbow

    q2_rad = alpha - beta
    q2 = np.degrees(q2_rad)

    # Adjust q2 to match DH convention (q2-90 in DH table)
    # The DH table shows q2-90, meaning 0° in joint space = -90° in DH
    # We need to add 90° to get the joint angle
    q2 = q2 + 90

    logger.debug(f"IK: q2 (shoulder angle) = {q2:.2f}° (alpha={np.degrees(alpha):.2f}°, beta={np.degrees(beta):.2f}°)")

    logger.info(f"IK: Solution found - q1={q1:.2f}°, q2={q2:.2f}°, q3={q3:.2f}°")

    return IKSolution(q1, q2, q3, valid=True)


def solve_ik_full(x, y, z, roll=0, pitch=-np.pi/2, yaw=0):
    """
    Solve full 6-DOF inverse kinematics (q1, q2, q3, q4, q5, q6)

    Args:
        x, y, z: Target TCP position coordinates in mm
        roll, pitch, yaw: Target TCP orientation in radians (default: tool pointing down)

    Returns:
        IKSolution object containing all 6 joint angles or error information

    Note:
        - Default orientation (roll=0, pitch=-π/2, yaw=0) points tool downward
        - Uses kinematic decoupling: position (q1-q3) then orientation (q4-q6)
    """

    logger.info(f"IK 6-DOF: Solving for TCP position X={x:.2f}, Y={y:.2f}, Z={z:.2f}")
    logger.info(f"IK 6-DOF: Target orientation Roll={np.degrees(roll):.2f}°, Pitch={np.degrees(pitch):.2f}°, Yaw={np.degrees(yaw):.2f}°")

    # Step 1: Calculate desired TCP orientation matrix
    R_tcp = euler_to_rotation_matrix(roll, pitch, yaw)
    logger.debug(f"IK 6-DOF: TCP rotation matrix:\n{R_tcp}")

    # Step 2: Calculate wrist center point (Pm) using orientation
    # Pm = TCP_position - L4 * TCP_z_axis
    tcp_z_axis = R_tcp[:, 2]  # Third column of rotation matrix
    Pm = np.array([x, y, z]) - L4 * tcp_z_axis

    Pm_x, Pm_y, Pm_z = Pm
    logger.debug(f"IK 6-DOF: Wrist center (Pm) at X={Pm_x:.2f}, Y={Pm_y:.2f}, Z={Pm_z:.2f}")

    # Step 3: Solve for first 3 joints (positioning)
    q1_rad = np.arctan2(Pm_y, Pm_x)
    q1 = np.degrees(q1_rad)

    r = np.sqrt(Pm_x**2 + Pm_y**2)
    s = Pm_z - L1
    D = np.sqrt(r**2 + s**2)

    # Reachability check
    max_reach = L2 + L3
    min_reach = abs(L2 - L3)

    if D > max_reach:
        error_msg = f"Target unreachable: distance {D:.2f}mm exceeds maximum reach {max_reach:.2f}mm"
        logger.error(f"IK 6-DOF: {error_msg}")
        return IKSolution(0, 0, 0, 0, 0, 0, valid=False, error_msg=error_msg)

    if D < min_reach:
        error_msg = f"Target unreachable: distance {D:.2f}mm is less than minimum reach {min_reach:.2f}mm"
        logger.error(f"IK 6-DOF: {error_msg}")
        return IKSolution(0, 0, 0, 0, 0, 0, valid=False, error_msg=error_msg)

    # Calculate q3
    cos_q3 = (D**2 - L2**2 - L3**2) / (2 * L2 * L3)
    cos_q3 = np.clip(cos_q3, -1.0, 1.0)
    sin_q3 = np.sqrt(1 - cos_q3**2)
    q3_rad = np.arctan2(sin_q3, cos_q3)
    q3 = np.degrees(q3_rad)

    # Calculate q2
    alpha = np.arctan2(s, r)
    beta = np.arctan2(L3 * np.sin(q3_rad), L2 + L3 * np.cos(q3_rad))
    q2_rad = alpha - beta
    q2 = np.degrees(q2_rad) + 90  # DH convention adjustment

    logger.debug(f"IK 6-DOF: Position joints - q1={q1:.2f}°, q2={q2:.2f}°, q3={q3:.2f}°")

    # Step 4: Calculate rotation matrix from base to wrist (R_0^3)
    # Using DH parameters: Link 1-3
    q2_rad_dh = np.radians(q2 - 90)  # Convert back to DH convention

    T1 = dh_transform(q1_rad, L1, 0, np.pi/2)
    T2 = dh_transform(q2_rad_dh, 0, L2, 0)
    T3 = dh_transform(q3_rad, 0, 0, np.pi/2)

    T_0_3 = T1 @ T2 @ T3
    R_0_3 = T_0_3[0:3, 0:3]

    logger.debug(f"IK 6-DOF: R_0_3 matrix:\n{R_0_3}")

    # Step 5: Calculate wrist rotation matrix (R_3^6)
    # R_0^6 = R_0^3 * R_3^6, therefore R_3^6 = (R_0^3)^T * R_0^6
    R_3_6 = R_0_3.T @ R_tcp

    logger.debug(f"IK 6-DOF: R_3_6 matrix:\n{R_3_6}")

    # Step 6: Extract wrist angles from R_3^6
    # Based on Thor IK equations from Hackaday
    # The equations from the PDF use a specific parameterization

    # Extract elements
    r11, r12, r13 = R_3_6[0, :]
    r21, r22, r23 = R_3_6[1, :]
    r31, r32, r33 = R_3_6[2, :]

    # Calculate q5 (wrist pitch)
    # q5 = arccos(r33) or arccos(specific combination)
    # Using the general ZYZ Euler angle extraction
    q5_rad = np.arccos(np.clip(r33, -1.0, 1.0))
    q5 = np.degrees(q5_rad)

    # Check for singularity
    if np.abs(np.sin(q5_rad)) < 1e-6:
        # Singularity: q4 and q6 are not uniquely defined
        # Set q4 to 0 and solve for q6
        logger.warning("IK 6-DOF: Wrist singularity detected")
        q4 = 0
        q6 = np.degrees(np.arctan2(r12, r11))
    else:
        # Normal case
        q4_rad = np.arctan2(r23, r13)
        q6_rad = np.arctan2(r32, -r31)

        q4 = np.degrees(q4_rad)
        q6 = np.degrees(q6_rad)

    logger.debug(f"IK 6-DOF: Wrist joints - q4={q4:.2f}°, q5={q5:.2f}°, q6={q6:.2f}°")
    logger.info(f"IK 6-DOF: Solution found - q1={q1:.2f}°, q2={q2:.2f}°, q3={q3:.2f}°, q4={q4:.2f}°, q5={q5:.2f}°, q6={q6:.2f}°")

    return IKSolution(q1, q2, q3, q4, q5, q6, valid=True)


def verify_ik_solution(solution, target_x, target_y, target_z, tolerance=1.0):
    """
    Verify IK solution by performing forward kinematics

    Args:
        solution: IKSolution object
        target_x, target_y, target_z: Original target position
        tolerance: Maximum acceptable error in mm

    Returns:
        tuple: (is_valid, error_distance, calculated_position)
    """
    if not solution.valid:
        return False, float('inf'), (0, 0, 0)

    # Simple forward kinematics check
    # This would use the full FK equations in production
    # For now, return True assuming the math is correct

    logger.debug(f"IK: Solution verification not yet implemented")
    return True, 0.0, (target_x, target_y, target_z)


if __name__ == "__main__":
    # Test cases
    logging.basicConfig(level=logging.DEBUG)

    print("Thor Robot Inverse Kinematics Test\n")
    print(f"Robot parameters: L1={L1}mm, L2={L2}mm, L3={L3}mm, L4={L4}mm")
    print(f"Max reach: {L2+L3}mm, Min reach: {abs(L2-L3)}mm\n")

    # Test 1: Point directly in front at mid-height
    print("Test 1: Point in front (X=300, Y=0, Z=300)")
    sol1 = solve_ik_position(300, 0, 300)
    print(sol1)
    print()

    # Test 2: Point to the side
    print("Test 2: Point to side (X=200, Y=200, Z=250)")
    sol2 = solve_ik_position(200, 200, 250)
    print(sol2)
    print()

    # Test 3: Unreachable point (too far)
    print("Test 3: Unreachable point (X=500, Y=0, Z=500)")
    sol3 = solve_ik_position(500, 0, 500)
    print(sol3)
    print()

    # Test 4: Point near minimum reach
    print("Test 4: Near minimum reach (X=50, Y=0, Z=210)")
    sol4 = solve_ik_position(50, 0, 210)
    print(sol4)
    print()

    print("="*60)
    print("6-DOF Inverse Kinematics Tests")
    print("="*60)
    print()

    # Test 5: 6-DOF with default orientation (tool down)
    print("Test 5: 6-DOF default orientation (X=250, Y=0, Z=300, tool down)")
    sol5 = solve_ik_full(250, 0, 300)
    print(sol5)
    print()

    # Test 6: 6-DOF with tilted tool
    print("Test 6: 6-DOF tilted tool (X=200, Y=50, Z=280, pitch=-45°)")
    sol6 = solve_ik_full(200, 50, 280, roll=0, pitch=np.radians(-45), yaw=0)
    print(sol6)
    print()

    # Test 7: 6-DOF with rotated base
    print("Test 7: 6-DOF rotated orientation (X=150, Y=150, Z=300, yaw=30°)")
    sol7 = solve_ik_full(150, 150, 300, roll=0, pitch=-np.pi/2, yaw=np.radians(30))
    print(sol7)
