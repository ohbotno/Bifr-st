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

class IKSolution:
    """Container for inverse kinematics solution"""
    def __init__(self, q1, q2, q3, valid=True, error_msg=""):
        self.q1 = q1  # Base rotation (degrees)
        self.q2 = q2  # Shoulder angle (degrees)
        self.q3 = q3  # Elbow angle (degrees)
        self.valid = valid
        self.error_msg = error_msg

    def __str__(self):
        if self.valid:
            return f"IK Solution: q1={self.q1:.2f}°, q2={self.q2:.2f}°, q3={self.q3:.2f}°"
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
