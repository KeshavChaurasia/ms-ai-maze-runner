"""
Mecanum Motor Controller Module

This module provides a high-level interface for controlling the four motors
of a Mecanum wheel drive system. It abstracts the low-level motor control
operations and provides a unified API for setting wheel velocities in a
robotics navigation system.

The controller manages the interface between kinematic calculations and
actual motor hardware, ensuring proper initialization and synchronized
control of all four wheels in the drive system.

Author: [Your Name]
Date: July 2025
Academic Project: AI-Powered Maze Navigation with Four-Wheel Robot
"""


class MecanumMotorController:
    """
    Motor Controller for Four-Wheel Mecanum Drive System
    
    This class provides a comprehensive interface for controlling the motors
    of a Mecanum wheel robot. It manages four independent motors arranged
    in a standard Mecanum configuration for omnidirectional movement.
    
    Motor Arrangement (Top View):
        Front Left (FL) ---- Front Right (FR)
              |                    |
              |      Robot         |
              |      Center        |
              |                    |
        Rear Left (RL)  ---- Rear Right (RR)
    
    The controller operates in velocity control mode, where each motor
    maintains a specified angular velocity for smooth continuous motion.
    """
    
    
    def __init__(self, front_left, front_right, rear_left, rear_right):
        """
        Initialize the Mecanum Motor Controller with four motor objects.
        
        Sets up the four-motor system with proper naming conventions and
        initializes all motors for velocity control operation. The motors
        should be obtained from the robot's device list in the main controller.
        
        Args:
            front_left: Webots Motor object for front-left wheel
            front_right: Webots Motor object for front-right wheel  
            rear_left: Webots Motor object for rear-left wheel
            rear_right: Webots Motor object for rear-right wheel
        """
        # Store motor references in a dictionary for organized access
        # Dictionary structure enables easy iteration and systematic control
        self.motors = {
            'front_left': front_left,      # Motor controlling front-left Mecanum wheel
            'front_right': front_right,    # Motor controlling front-right Mecanum wheel
            'rear_left': rear_left,        # Motor controlling rear-left Mecanum wheel
            'rear_right': rear_right       # Motor controlling rear-right Mecanum wheel
        }
        # Initialize all motors with appropriate control settings
        self._initialize_motors()
    
    def _initialize_motors(self):
        """
        Initialize all motors for velocity control operation.
        
        This private method configures each motor for velocity control mode,
        which is essential for smooth Mecanum wheel operation. In velocity
        control mode, motors maintain a specified angular velocity rather
        than moving to specific positions.
        
        Technical Implementation:
        - setPosition(float('inf')) enables velocity control mode
        - setVelocity(0.0) ensures motors start stationary
        """
        for motor in self.motors.values():
            # Set position to infinity to enable continuous velocity control
            # This disables position-based control and allows free rotation
            motor.setPosition(float('inf'))
            # Initialize with zero velocity for controlled startup
            # Prevents unexpected movement during system initialization
            motor.setVelocity(0.0)
    
    def set_speeds(self, front_left, front_right, rear_left, rear_right):
        """
        Set individual wheel speeds for coordinated robot movement.
        
        This method applies calculated wheel speeds from the kinematics
        module to the actual motor hardware. The speeds are applied
        simultaneously to all motors to ensure synchronized movement
        and prevent unwanted robot drift or rotation.
        
        Args:
            front_left (float): Angular velocity for front-left wheel (rad/s)
            front_right (float): Angular velocity for front-right wheel (rad/s)
            rear_left (float): Angular velocity for rear-left wheel (rad/s)
            rear_right (float): Angular velocity for rear-right wheel (rad/s)
        
        Note: Positive velocities correspond to forward wheel rotation.
        """
        # Apply speeds to individual motors in systematic order
        # Simultaneous application ensures coordinated movement
        self.motors['front_left'].setVelocity(front_left)
        self.motors['front_right'].setVelocity(front_right)
        self.motors['rear_left'].setVelocity(rear_left)
        self.motors['rear_right'].setVelocity(rear_right)
    
    def stop(self):
        """
        Emergency stop - immediately halt all wheel movement.
        
        This method provides a safe way to stop the robot by setting all
        motor velocities to zero. Essential for emergency situations,
        end-of-program cleanup, and controlled stops during navigation.
        
        Safety Implementation:
        Uses the existing set_speeds method to ensure consistent behavior
        and maintain code modularity. Called during obstacle detection,
        navigation completion, or manual intervention scenarios.
        """
        # Set all wheel velocities to zero for immediate stop
        self.set_speeds(0, 0, 0, 0)
    
    def get_velocities(self):
        """
        Retrieve current motor velocities for monitoring and debugging.
        
        This method provides real-time feedback on actual motor speeds,
        valuable for debugging motor control issues, monitoring system
        performance, and validating that commanded speeds are achieved.
        
        Returns:
            dict: Dictionary mapping motor names to current velocities
                  Keys: 'front_left', 'front_right', 'rear_left', 'rear_right'
                  Values: Current angular velocities in rad/s
        
        Note: Returned velocities may differ from commanded values due to
        motor dynamics, load variations, and control system characteristics.
        """
        # Query each motor for current velocity and return organized data
        # Dictionary comprehension provides clean, systematic data collection
        return {
            name: motor.getVelocity() 
            for name, motor in self.motors.items()
        }