"""
Consolidated Maze Robot Hardware Interface

This module provides a simplified, unified interface for controlling a Mecanum wheel
robot in maze navigation scenarios. It consolidates hardware management, kinematics,
position tracking, and basic movement functionality into a single, cohesive class.

Key Features:
- Integrated hardware abstraction (motors, sensors, GPS)
- Built-in Mecanum wheel kinematics
- GPS-based position tracking with coordinate conversion
- Cell-based movement commands
- Sensor reading and wall detection

Authors: Christopher Hunter-Bennett
         David Rasheeld Watler
         Keshav Chaurasia 
         Mark Arthur Gabiana
         Rainer Knapp
Date: August 2025
Academic Project: Maze Navigation with Four-Wheel Robot
"""

from controller import Robot, DistanceSensor, Motor, GPS, Compass
from config import Config
import math


class MazeRobot:
    """
    Unified Maze Robot Interface
    
    This class consolidates all hardware-related functionality into a single
    interface, making it easier to understand and maintain the robot control
    system. It handles motor control, position tracking, sensor management,
    and basic movement operations.
    """
    
    def __init__(self):
        """Initialize the maze robot with default state."""
        # Core system components
        self.robot = None
        self.timestep = None
        
        # Hardware components
        self.motors = {}
        self.sensors = {}
        self.gps = None
        
        # System state
        self._initialized = False
    
    def initialize(self):
        """
        Initialize the robot and all subsystems.
        
        Returns:
            bool: True if initialization successful, False otherwise
        """
        if self._initialized:
            print("Robot already initialized!")
            return True
        
        try:
            # Initialize robot
            self.robot = Robot()
            self.timestep = int(self.robot.getBasicTimeStep())
            
            # Initialize motors
            self._initialize_motors()
            
            # Initialize GPS
            self._initialize_gps()
            
            # Initialize sensors
            self._initialize_sensors()
            
            self._initialized = True
            print("Maze Robot initialized successfully!")
            return True
            
        except Exception as e:
            print(f"ERROR: Failed to initialize robot: {e}")
            return False
    
    def _initialize_motors(self):
        """Initialize the four Mecanum wheel motors."""
        motor_names = ['front_left_motor', 'front_right_motor', 
                      'rear_left_motor', 'rear_right_motor']
        
        for name in motor_names:
            motor = self.robot.getDevice(name)
            motor.setPosition(float('inf'))  # Enable velocity control
            motor.setVelocity(0.0)  # Start stationary
            self.motors[name] = motor
    
    def _initialize_gps(self):
        """Initialize GPS for position tracking."""
        self.gps = self.robot.getDevice('gps')
        self.gps.enable(self.timestep)
    
    def _initialize_sensors(self):
        """Initialize distance sensors."""
        sensor_names = ['front_distance_sensor', 'right_distance_sensor', 
                       'back_distance_sensor', 'left_distance_sensor']
        
        for name in sensor_names:
            try:
                sensor = self.robot.getDevice(name)
                sensor.enable(self.timestep)
                self.sensors[name] = sensor
            except:
                print(f"Warning: Could not initialize sensor {name}")
    
    def wait_for_gps(self):
        """
        Wait for GPS to provide valid position data.
        
        Returns:
            tuple or None: (x, y) GPS coordinates if successful, None if failed
        """
        if not self._initialized:
            print("ERROR: Robot not initialized! Call initialize() first.")
            return None
        
        print("Waiting for GPS to initialize...")
        while self.robot.step(self.timestep) != -1:
            position = self.get_current_position()
            if position is not None:
                print(f"GPS ready! Starting position: ({position[0]:.2f}, {position[1]:.2f})")
                return position
        return None
    
    # =================== POSITION TRACKING ===================
    
    def get_current_position(self):
        """
        Get current GPS position with validation.
        
        Returns:
            tuple or None: (x, y) coordinates in meters if valid, None otherwise
        """
        position = self.gps.getValues()[:2]
        if math.isnan(position[0]) or math.isnan(position[1]):
            return None
        return position
    
    def get_current_cell(self):
        """
        Get current cell coordinates.
        
        Returns:
            tuple or None: (cell_x, cell_y) if GPS available, None otherwise
        """
        position = self.get_current_position()
        if position is None:
            return None
        return self.gps_to_cell(*position)
    
    def gps_to_cell(self, gps_x, gps_y):
        """Convert GPS coordinates to cell coordinates."""
        return round(gps_x - 0.5), round(gps_y - 0.5)
    
    def cell_to_gps(self, cell_x, cell_y):
        """Convert cell coordinates to GPS coordinates."""
        return cell_x + 0.5, cell_y + 0.5
    
    # =================== KINEMATICS ===================
    
    def calculate_wheel_speeds(self, vx, vy):
        """
        Calculate individual wheel speeds for Mecanum drive.
        
        Args:
            vx (float): Desired velocity in X direction (m/s)
            vy (float): Desired velocity in Y direction (m/s)
            
        Returns:
            tuple: (front_left, front_right, rear_left, rear_right) speeds
        """
        front_left_speed = vx + vy
        front_right_speed = vx - vy
        rear_left_speed = vx - vy
        rear_right_speed = vx + vy
        
        return front_left_speed, front_right_speed, rear_left_speed, rear_right_speed
    
    def set_wheel_speeds(self, front_left, front_right, rear_left, rear_right):
        """Set individual wheel speeds."""
        self.motors['front_left_motor'].setVelocity(front_left)
        self.motors['front_right_motor'].setVelocity(front_right)
        self.motors['rear_left_motor'].setVelocity(rear_left)
        self.motors['rear_right_motor'].setVelocity(rear_right)
    
    def stop(self):
        """Stop all motors immediately."""
        self.set_wheel_speeds(0, 0, 0, 0)
    
    # =================== MOVEMENT CONTROL ===================
    
    def move_to_position(self, target_x, target_y, verbose=True):
        """
        Move robot to specific GPS coordinates.
        
        Args:
            target_x (float): Target GPS X coordinate
            target_y (float): Target GPS Y coordinate
            verbose (bool): Enable debug output
            
        Returns:
            bool: True if target reached, False otherwise
        """
        current_pos = self.get_current_position()
        if current_pos is None:
            return False
        
        current_x, current_y = current_pos
        dx = target_x - current_x
        dy = target_y - current_y
        distance = math.sqrt(dx*dx + dy*dy)
        
        # Check if already at target
        if distance < Config.MOVEMENT_TOLERANCE:
            if verbose:
                print(f"Target reached! Distance: {distance:.4f}")
            return True
        
        # Force straight-line movement for grid navigation
        abs_dx = abs(dx)
        abs_dy = abs(dy)
        
        if abs_dx > abs_dy * 2:  # Primarily horizontal
            dy = 0
            distance = abs_dx
        elif abs_dy > abs_dx * 2:  # Primarily vertical
            dx = 0
            distance = abs_dy
        
        # Calculate adaptive speed
        speed = self._calculate_adaptive_speed(distance)
        
        # Calculate normalized direction
        if distance > 0:
            vx = (dx / distance) * speed
            vy = (dy / distance) * speed
        else:
            vx = vy = 0
        
        # Apply additional smoothing for small movements
        if distance < 0.05:
            vx *= 0.5
            vy *= 0.5
        
        # Calculate wheel speeds and apply limits
        wheel_speeds = self.calculate_wheel_speeds(vx, vy)
        max_wheel_speed = max(abs(speed) for speed in wheel_speeds)
        
        if max_wheel_speed > Config.MAX_SPEED:
            scale_factor = Config.MAX_SPEED / max_wheel_speed
            wheel_speeds = tuple(speed * scale_factor for speed in wheel_speeds)
        
        self.set_wheel_speeds(*wheel_speeds)
        return False  # Still moving
    
    def _calculate_adaptive_speed(self, distance):
        """Calculate speed based on distance to target."""
        if distance > 0.5:
            return Config.MAX_SPEED * 0.8  # High speed for long distances
        elif distance > 0.2:
            return Config.MAX_SPEED * 0.6  # Medium speed
        elif distance > 0.1:
            return Config.MAX_SPEED * 0.4  # Slow speed
        else:
            return Config.MAX_SPEED * 0.2  # Very slow for precision
    
    def move_to_cell(self, target_cell_x, target_cell_y, verbose=True):
        """
        Move robot to specific cell coordinates.
        
        Args:
            target_cell_x (int): Target cell X coordinate
            target_cell_y (int): Target cell Y coordinate
            verbose (bool): Enable debug output
            
        Returns:
            bool: True if target cell reached, False otherwise
        """
        if not self._initialized:
            print("ERROR: Robot not initialized!")
            return False
        
        target_gps_x, target_gps_y = self.cell_to_gps(target_cell_x, target_cell_y)
        
        if verbose:
            current_pos = self.get_current_position()
            if current_pos:
                current_cell = self.gps_to_cell(*current_pos)
                print(f"Moving from cell {current_cell} to cell ({target_cell_x}, {target_cell_y})")
        
        step_count = 0
        while self.robot.step(self.timestep) != -1:
            step_count += 1
            
            if self.move_to_position(target_gps_x, target_gps_y, verbose):
                if verbose:
                    final_pos = self.get_current_position()
                    if final_pos:
                        final_cell = self.gps_to_cell(*final_pos)
                        distance_error = math.sqrt((final_pos[0] - target_gps_x)**2 + 
                                                 (final_pos[1] - target_gps_y)**2)
                        print(f"✓ Movement complete! Steps: {step_count}")
                        print(f"  Final GPS: ({final_pos[0]:.3f}, {final_pos[1]:.3f})")
                        print(f"  Final cell: {final_cell}")
                        print(f"  Distance error: {distance_error:.4f} units")
                return True
        return False
    
    # =================== DIRECTIONAL MOVEMENT ===================
    
    def move_left(self, verbose=True):
        """Move left by one cell (-X direction)."""
        current_pos = self.get_current_position()
        if current_pos is None:
            print("GPS not ready, cannot move")
            return False
        
        current_cell_x, current_cell_y = self.gps_to_cell(*current_pos)
        if verbose:
            print("=== MOVE LEFT ===")
        return self.move_to_cell(current_cell_x - 1, current_cell_y, verbose)
    
    def move_right(self, verbose=True):
        """Move right by one cell (+X direction)."""
        current_pos = self.get_current_position()
        if current_pos is None:
            print("GPS not ready, cannot move")
            return False
        
        current_cell_x, current_cell_y = self.gps_to_cell(*current_pos)
        if verbose:
            print("=== MOVE RIGHT ===")
        return self.move_to_cell(current_cell_x + 1, current_cell_y, verbose)
    
    def move_top(self, verbose=True):
        """Move top by one cell (+Y direction)."""
        current_pos = self.get_current_position()
        if current_pos is None:
            print("GPS not ready, cannot move")
            return False
        
        current_cell_x, current_cell_y = self.gps_to_cell(*current_pos)
        if verbose:
            print("=== MOVE TOP ===")
        return self.move_to_cell(current_cell_x, current_cell_y + 1, verbose)
    
    def move_bottom(self, verbose=True):
        """Move bottom by one cell (-Y direction)."""
        current_pos = self.get_current_position()
        if current_pos is None:
            print("GPS not ready, cannot move")
            return False
        
        current_cell_x, current_cell_y = self.gps_to_cell(*current_pos)
        if verbose:
            print("=== MOVE BOTTOM ===")
        return self.move_to_cell(current_cell_x, current_cell_y - 1, verbose)
    
    # =================== SENSOR OPERATIONS ===================
    
    def get_sensor_readings(self):
        """Get all distance sensor readings."""
        readings = {}
        for name, sensor in self.sensors.items():
            try:
                readings[name.replace('_distance_sensor', '')] = sensor.getValue()
            except:
                readings[name.replace('_distance_sensor', '')] = None
        return readings
    
    def is_wall_detected(self, direction):
        """
        Check if wall is detected in specified direction.
        
        Args:
            direction (str): 'front', 'right', 'back', or 'left'
            
        Returns:
            bool: True if wall detected, False otherwise
        """
        sensor_name = f"{direction}_distance_sensor"
        if sensor_name not in self.sensors:
            return True  # Assume wall if sensor not available
        
        try:
            sensor_value = self.sensors[sensor_name].getValue()
            return sensor_value < Config.SENSOR_THRESHOLD  # Wall detection threshold
        except:
            return True  # Assume wall if sensor read fails
