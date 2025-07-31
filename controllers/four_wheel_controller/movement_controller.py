"""
Movement Controller Module for Mecanum Wheel Robot Navigation

This module implements high-level movement control algorithms for precise
robot navigation in a maze environment. It integrates kinematic calculations,
position tracking, and motor control to achieve accurate cell-to-cell movement
using a Mecanum wheel drive system.

Key Features:
- Adaptive speed control based on distance to target
- Precision movement with drift compensation
- Real-time position feedback and error correction
- Debug monitoring for performance analysis

The controller employs advanced algorithms for:
1. Trajectory planning and execution
2. Speed adaptation for precision vs. efficiency
3. Movement direction enforcement for grid-based navigation
4. Motor speed limiting to prevent wheel slip

Author: Keshav Chaurasia, Mark, Chris, David
Date: July 2025
Academic Project: Maze Navigation with Four-Wheel Robot
"""

from config import Config
from mecanum_kinematics import MecanumKinematics
import math


class MovementController:
    """
    High-Level Movement Controller for Cell-Based Robot Navigation
    
    This class provides sophisticated movement control capabilities for a
    Mecanum wheel robot operating in a grid-based maze environment. It
    coordinates between position tracking, kinematic calculations, and
    motor control to achieve precise autonomous navigation.
    
    Architecture:
    The controller operates as the central coordination layer between:
    - Position tracking system (GPS/odometry)
    - Kinematic calculations (velocity to wheel speed conversion)
    - Motor control system (hardware interface)
    
    Navigation Strategy:
    Uses a precision-focused approach optimized for maze navigation:
    - Grid-aligned movement with drift prevention
    - Adaptive speed control for accuracy near targets
    - Real-time feedback correction
    """
    
    
    def __init__(self, robot, motor_controller, position_tracker):
        """
        Initialize the Movement Controller with required system components.
        
        Establishes connections to all subsystems required for autonomous
        navigation and initializes internal state variables for movement
        control and debugging.
        
        Args:
            robot: Webots Robot instance for simulation interface
            motor_controller: MecanumMotorController for wheel speed control
            position_tracker: PositionTracker for real-time location data
            
        Attributes:
            robot: Reference to the main robot controller
            motor_controller: Interface to the four-wheel motor system
            position_tracker: GPS/odometry-based position tracking system
            kinematics: Mecanum wheel kinematic calculation engine
            _debug_counter: Internal counter for periodic debug output
        """
        # Store references to essential subsystems
        self.robot = robot                          # Main robot controller interface
        self.motor_controller = motor_controller    # Motor control subsystem
        self.position_tracker = position_tracker   # Position tracking subsystem
        
        # Initialize kinematic calculation engine for Mecanum wheels
        self.kinematics = MecanumKinematics()
        
        # Initialize debug counter for periodic information output
        # Used to control frequency of debug messages during navigation
        self._debug_counter = 0
    
    def move_to_position(self, target_x, target_y, verbose=True):
        """
        Move robot to specific GPS coordinates with precision control for Mecanum wheels.
        
        This method implements a sophisticated movement algorithm that combines
        real-time position feedback, adaptive speed control, and drift prevention
        to achieve accurate navigation in a grid-based maze environment.
        
        Algorithm Overview:
        1. Position Error Calculation: Compute displacement vector to target
        2. Convergence Check: Verify if target is within tolerance
        3. Movement Direction Analysis: Determine if movement is grid-aligned
        4. Speed Adaptation: Adjust velocity based on proximity to target
        5. Kinematic Transformation: Convert to individual wheel speeds
        6. Safety Limiting: Ensure speeds don't exceed hardware limits
        
        Args:
            target_x (float): Target X coordinate in meters (global frame)
            target_y (float): Target Y coordinate in meters (global frame)
            verbose (bool): Enable detailed debug output for analysis
            
        Returns:
            bool: True when target is reached within tolerance, False otherwise
            
        Navigation Features:
        - Precision movement with configurable tolerance
        - Drift prevention for grid-aligned navigation
        - Adaptive speed control (slow near target, fast for long distances)
        - Real-time feedback correction
        - Hardware safety limits enforcement
        """
        # === STEP 1: POSITION ACQUISITION AND VALIDATION ===
        current_pos = self.position_tracker.get_current_position()
        if current_pos is None:
            # Position tracking failure - abort movement for safety
            return False
        
        current_x, current_y = current_pos
        
        # === STEP 2: ERROR VECTOR CALCULATION ===
        # Calculate displacement vector from current position to target
        dx = target_x - current_x  # X-component of position error
        dy = target_y - current_y  # Y-component of position error
        
        # Calculate Euclidean distance to target for convergence check
        distance = math.sqrt(dx*dx + dy*dy)
        
        # === STEP 3: CONVERGENCE CHECK ===
        # Check if robot has reached target within specified tolerance
        if distance < Config.MOVEMENT_TOLERANCE:
            # Target reached - stop all motors and report success
            self.motor_controller.stop()
            if verbose:
                print(f"Target reached! Final: ({current_x:.3f}, {current_y:.3f}), Distance: {distance:.4f}")
            return True
        
        # === STEP 4: MOVEMENT DIRECTION ANALYSIS AND DRIFT PREVENTION ===
        # For straight line movements, enforce pure directional movement
        # This prevents drift and ensures precise cell-to-cell movement in maze
        abs_dx = abs(dx)  # Magnitude of X displacement
        abs_dy = abs(dy)  # Magnitude of Y displacement
        
        # Determine if this is primarily a horizontal or vertical movement
        # Use 2:1 ratio threshold to distinguish between diagonal and straight movement
        if abs_dx > abs_dy * 2:  # Primarily horizontal movement
            # Force pure horizontal movement to prevent Y-axis drift
            # Critical for maintaining grid alignment in maze navigation
            dy = 0  # Eliminate vertical component
            distance = abs_dx  # Recalculate distance for speed control
        elif abs_dy > abs_dx * 2:  # Primarily vertical movement
            # Force pure vertical movement to prevent X-axis drift
            # Ensures straight-line movement between maze cells
            dx = 0  # Eliminate horizontal component
            distance = abs_dy  # Recalculate distance for speed control
        
        # === STEP 5: ADAPTIVE SPEED CALCULATION ===
        # Calculate movement speed based on distance to target
        # Implements multi-tier speed control for precision vs. efficiency
        speed = self._calculate_adaptive_speed(distance)
        
        # === STEP 6: VELOCITY VECTOR NORMALIZATION ===
        # Calculate normalized movement direction with applied speed
        if distance > 0:
            # Normalize displacement vector and scale by desired speed
            vx = (dx / distance) * speed  # X-component of velocity vector
            vy = (dy / distance) * speed  # Y-component of velocity vector
        else:
            # Zero distance case - no movement required
            vx = vy = 0
        
        # === STEP 7: MICRO-MOVEMENT SMOOTHING ===
        # Apply additional smoothing for very small movements
        # Prevents oscillation and overshoot near target
        if distance < 0.05:
            # Reduce velocity for micro-adjustments near target
            vx *= 0.5
            vy *= 0.5
        
        # === STEP 8: KINEMATIC TRANSFORMATION ===
        # Apply Mecanum kinematics to convert velocity to wheel speeds
        wheel_speeds = self.kinematics.calculate_wheel_speeds(vx, vy)
        
        # === STEP 9: HARDWARE SAFETY LIMITING ===
        # Apply speed limiting to prevent wheel slip and hardware damage
        max_wheel_speed = max(abs(speed) for speed in wheel_speeds)
        if max_wheel_speed > Config.MAX_SPEED:
            # Scale down all wheel speeds proportionally to stay within limits
            scale_factor = Config.MAX_SPEED / max_wheel_speed
            wheel_speeds = tuple(speed * scale_factor for speed in wheel_speeds)
        
        # === STEP 10: MOTOR COMMAND EXECUTION ===
        # Send calculated wheel speeds to motor controller
        self.motor_controller.set_speeds(*wheel_speeds)
        
        # === STEP 11: DEBUG OUTPUT AND MONITORING ===
        # Provide detailed feedback for system monitoring and debugging
        if verbose:
            self._debug_movement(current_pos, (target_x, target_y), distance, vx, vy, wheel_speeds)
        
        # Return False to indicate movement is still in progress
        return False

    def _calculate_adaptive_speed(self, distance):
        """
        Calculate optimal movement speed based on distance to target.
        
        Implements a multi-tier speed control algorithm that balances
        navigation efficiency with precision requirements. Uses distance-based
        speed zones to ensure smooth approach to targets while maintaining
        reasonable travel times for longer distances.
        
        Speed Control Strategy:
        - Close range (< 0.1m): Slow speed for precision approach
        - Medium range (0.1-0.3m): Moderate speed for controlled movement  
        - Long range (> 0.3m): Higher speed for efficient travel
        
        Args:
            distance (float): Current distance to target in meters
            
        Returns:
            float: Calculated speed as fraction of maximum speed
            
        Note: Speed values are tuned for maze navigation where precision
        is more critical than maximum velocity.
        """
        # return Config.MAX_SPEED  # Default speed
        
        # Multi-tier adaptive speed control based on proximity to target
        if distance < 0.1:
            # Precision zone - slow speed for accurate final approach
            # Critical for achieving tight tolerance requirements
            return Config.MAX_SPEED * 0.3  # 30% of maximum speed
        elif distance < 0.3:
            # Transition zone - medium speed for controlled movement
            # Balances speed and control for intermediate distances
            return Config.MAX_SPEED * 0.5  # 50% of maximum speed
        else:
            # Travel zone - higher speed for efficient long-distance movement
            # Optimizes travel time while maintaining control
            return Config.MAX_SPEED * 0.7  # 70% of maximum speed
    
    
    def _debug_movement(self, current_pos, target_pos, distance, vx, vy, wheel_speeds):
        """
        Print comprehensive debug information for movement analysis.
        
        Provides periodic detailed output of robot navigation state for
        performance monitoring, debugging, and academic analysis. Output
        frequency is controlled by Config.DEBUG_FREQUENCY to prevent
        console flooding while maintaining useful feedback.
        
        Debug Information Includes:
        - Current and target positions with precision
        - Distance to target for convergence monitoring  
        - Velocity components for trajectory analysis
        - Individual wheel speeds for hardware verification
        
        Args:
            current_pos (tuple): Current robot position (x, y)
            target_pos (tuple): Target position (x, y) 
            distance (float): Current distance to target
            vx (float): X-component of velocity vector
            vy (float): Y-component of velocity vector
            wheel_speeds (tuple): Individual wheel speeds (FL, FR, RL, RR)
            
        Output Format:
        Structured multi-line output showing:
        Line 1: Position data and distance metrics
        Line 2: Velocity vector components  
        Line 3: Individual wheel speed assignments
        """
        # Increment debug counter for periodic output control
        self._debug_counter += 1
        
        # Output debug information at specified frequency
        # Prevents excessive console output while maintaining visibility
        if self._debug_counter % Config.DEBUG_FREQUENCY == 0:
            # Extract position coordinates for formatted output
            current_x, current_y = current_pos
            target_x, target_y = target_pos
            
            # Extract individual wheel speeds for detailed monitoring
            fl, fr, rl, rr = wheel_speeds
            
            # Formatted output for position and navigation status
            print(f"  → Moving... Current: ({current_x:.3f}, {current_y:.3f}), "
                  f"Target: ({target_x:.3f}, {target_y:.3f}), Distance: {distance:.4f}")
            
            # Velocity vector information for trajectory analysis
            print(f"      Velocity: vx={vx:.3f}, vy={vy:.3f}")
            
            # Individual wheel speed monitoring for hardware verification
            print(f"      Wheels: FL={fl:.2f}, FR={fr:.2f}, RL={rl:.2f}, RR={rr:.2f}")

