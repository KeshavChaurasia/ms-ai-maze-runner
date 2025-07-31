"""
Mecanum Wheel Kinematics Module

This module implements the mathematical foundations for Mecanum wheel kinematics,
enabling omnidirectional movement in a four-wheel robot system. Mecanum wheels
feature angled rollers that allow lateral movement without changing the robot's
orientation, making them ideal for maze navigation in confined spaces.

Mathematical Foundation:
The kinematics are based on the principle that each Mecanum wheel contributes
to both longitudinal and lateral motion through vector decomposition of forces.
The 45-degree roller configuration creates specific velocity relationships
between desired robot motion and individual wheel speeds.

References:
- Muir, P.F. & Neuman, C.P. (1987). "Kinematic modeling of wheeled mobile robots"
- Taheri, H. et al. (2015). "Omnidirectional mobile robots, mechanisms and navigation approaches"

Author: Keshav Chaurasia, Mark, Chris, David
Date: July 2025
Academic Project: Maze Navigation with Four-Wheel Robot
"""

import math
from typing import Tuple


class MecanumKinematics:
    """
    Mecanum Wheel Kinematics Calculator
    
    This class provides static methods for converting desired robot velocities
    into individual wheel speeds for a four-wheel Mecanum drive system.
    
    Wheel Configuration:
    The robot uses a standard Mecanum wheel arrangement:
    - Front Left (FL): Rollers angled at +45° (exterior wheel)
    - Front Right (FR): Rollers angled at -45° (interior wheel)
    - Rear Left (RL): Rollers angled at -45° (interior wheel)
    - Rear Right (RR): Rollers angled at +45° (exterior wheel)
    
    Coordinate System:
    - X-axis: Forward/backward motion (positive = forward)
    - Y-axis: Left/right motion (positive = left)
    - Origin: Robot's geometric center
    
    Mathematical Model:
    The kinematic equations are derived from the constraint that each wheel's
    velocity vector must align with the combined effect of robot translation
    and the wheel's roller orientation.
    """
    
    @staticmethod
    def calculate_wheel_speeds(vx: float, vy: float) -> Tuple[float, float, float, float]:
        """
        Calculate individual wheel speeds for omnidirectional movement
        
        This method implements the inverse kinematics for a Mecanum wheel system,
        converting desired robot velocities into the required wheel angular velocities.
        
        Mathematical Derivation:
        For Mecanum wheels with 45° rollers, the relationship between robot
        velocity (vx, vy) and wheel speeds is:
        
        v_wheel = v_robot · cos(θ_roller) + v_lateral · sin(θ_roller)
        
        Where θ_roller is the roller angle relative to the wheel's rotation axis.
        
        Kinematic Equations:
        - FL_speed = vx + vy  (exterior wheel: +45° rollers)
        - FR_speed = vx - vy  (interior wheel: -45° rollers)
        - RL_speed = vx - vy  (interior wheel: -45° rollers)
        - RR_speed = vx + vy  (exterior wheel: +45° rollers)
        
        Args:
            vx (float): Desired velocity in X direction (m/s)
                       Positive values move robot forward
                       Negative values move robot backward
                       
            vy (float): Desired velocity in Y direction (m/s)
                       Positive values move robot left
                       Negative values move robot right
            
        Returns:
            Tuple[float, float, float, float]: Individual wheel speeds in rad/s
                (front_left, front_right, rear_left, rear_right)
                
        Note:
            The returned speeds are in angular velocity units (rad/s) assuming
            unit wheel radius. For actual implementation, multiply by the
            inverse of the wheel radius to get true angular velocities.
            
        Example:
            >>> # Move forward at 1 m/s
            >>> fl, fr, rl, rr = MecanumKinematics.calculate_wheel_speeds(1.0, 0.0)
            >>> print(f"All wheels: {fl}, {fr}, {rl}, {rr}")  # (1.0, 1.0, 1.0, 1.0)
            
            >>> # Strafe left at 0.5 m/s
            >>> fl, fr, rl, rr = MecanumKinematics.calculate_wheel_speeds(0.0, 0.5)
            >>> print(f"Wheels: {fl}, {fr}, {rl}, {rr}")  # (0.5, -0.5, -0.5, 0.5)
        """
        # Apply standard Mecanum wheel kinematic equations
        # These equations are derived from the geometric constraints of
        # the 45-degree roller configuration
        
        front_left_speed = vx + vy    # EXTERIOR wheel (+45° roller configuration)
                                      # Positive vy (leftward) adds to forward motion
                                      
        front_right_speed = vx - vy   # INTERIOR wheel (-45° roller configuration)
                                      # Positive vy (leftward) subtracts from forward motion
                                      
        rear_left_speed = vx - vy     # INTERIOR wheel (-45° roller configuration)
                                      # Same configuration as front right
                                      
        rear_right_speed = vx + vy    # EXTERIOR wheel (+45° roller configuration)
                                      # Same configuration as front left
        
        # Return wheel speeds in standard order: FL, FR, RL, RR
        # This ordering matches common robotics conventions and motor
        # driver indexing schemes
        return front_left_speed, front_right_speed, rear_left_speed, rear_right_speed
