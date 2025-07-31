"""
Four Wheel Controller - Backward compatibility wrapper
This file maintains compatibility with the original controller name
while using the new modular architecture.
"""
from controller import Robot, DistanceSensor, Motor, GPS, Compass
import math
import time
from collections import deque

# Configuration Constants
class Config:
    TIME_STEP = 64
    MAX_SPEED = 6.0
    CELL_SIZE = 1.0
    MOVEMENT_TOLERANCE = 0.05
    DEBUG_FREQUENCY = 25  # Print debug info every N steps
    SENSOR_THRESHOLD = 400

class MecanumKinematics:
    """Handles Mecanum wheel kinematics calculations"""
    
    @staticmethod
    def calculate_wheel_speeds(vx, vy):
        """
        Calculate individual wheel speeds for omnidirectional movement
        
        Args:
            vx (float): Velocity in X direction (horizontal)
            vy (float): Velocity in Y direction (vertical)
            
        Returns:
            tuple: (front_left, front_right, rear_left, rear_right) speeds
        """
        # Standard Mecanum wheel equations
        front_left_speed = vx + vy    # EXTERIOR wheel
        front_right_speed = vx - vy   # INTERIOR wheel  
        rear_left_speed = vx - vy     # INTERIOR wheel
        rear_right_speed = vx + vy    # EXTERIOR wheel
                 
        return front_left_speed, front_right_speed, rear_left_speed, rear_right_speed


class PositionTracker:
    """Handles GPS position tracking and coordinate conversion"""
    
    def __init__(self, gps_device):
        self.gps = gps_device
    
    def get_current_position(self):
        """Get current GPS position (x, y)"""
        position = self.gps.getValues()[:2]
        if math.isnan(position[0]) or math.isnan(position[1]):
            return None
        return position
    
    def gps_to_cell(self, gps_x, gps_y):
        """Convert GPS coordinates to cell coordinates"""
        return round(gps_x - 0.5), round(gps_y - 0.5)
    
    def cell_to_gps(self, cell_x, cell_y):
        """Convert cell coordinates to GPS coordinates"""
        return cell_x + 0.5, cell_y + 0.5


class MecanumMotorController:
    """Handles motor control for Mecanum wheels"""
    
    def __init__(self, front_left, front_right, rear_left, rear_right):
        self.motors = {
            'front_left': front_left,
            'front_right': front_right,
            'rear_left': rear_left,
            'rear_right': rear_right
        }
        self._initialize_motors()
    
    def _initialize_motors(self):
        """Initialize all motors for velocity control"""
        for motor in self.motors.values():
            motor.setPosition(float('inf'))
            motor.setVelocity(0.0)
    
    def set_speeds(self, front_left, front_right, rear_left, rear_right):
        """Set individual wheel speeds"""
        self.motors['front_left'].setVelocity(front_left)
        self.motors['front_right'].setVelocity(front_right)
        self.motors['rear_left'].setVelocity(rear_left)
        self.motors['rear_right'].setVelocity(rear_right)
    
    def stop(self):
        """Stop all wheels"""
        self.set_speeds(0, 0, 0, 0)
    
    def get_velocities(self):
        """Get current motor velocities for debugging"""
        return {
            name: motor.getVelocity() 
            for name, motor in self.motors.items()
        }


class MovementController:
    """High-level movement controller for cell-based navigation"""
    
    def __init__(self, robot, motor_controller, position_tracker):
        self.robot = robot
        self.motor_controller = motor_controller
        self.position_tracker = position_tracker
        self.kinematics = MecanumKinematics()
        self._debug_counter = 0
    

    def move_to_position(self, target_x, target_y, verbose=True):
        """
        Move robot to specific GPS coordinates with improved precision for Mecanum wheels
        
        Args:
            target_x (float): Target X coordinate
            target_y (float): Target Y coordinate
            verbose (bool): Enable debug output
            
        Returns:
            bool: True when target is reached
        """
        current_pos = self.position_tracker.get_current_position()
        if current_pos is None:
            return False
        
        current_x, current_y = current_pos
        
        # Calculate distance to target
        dx = target_x - current_x
        dy = target_y - current_y
        distance = math.sqrt(dx*dx + dy*dy)
        
        # Check if target reached
        if distance < Config.MOVEMENT_TOLERANCE:
            self.motor_controller.stop()
            if verbose:
                print(f"Target reached! Final: ({current_x:.3f}, {current_y:.3f}), Distance: {distance:.4f}")
            return True
        
        # For straight line movements, enforce pure directional movement
        # This prevents drift and ensures precise cell-to-cell movement
        abs_dx = abs(dx)
        abs_dy = abs(dy)
        
        # Determine if this is primarily a horizontal or vertical movement
        if abs_dx > abs_dy * 2:  # Primarily horizontal movement
            dy = 0  # Force pure horizontal movement
            distance = abs_dx
        elif abs_dy > abs_dx * 2:  # Primarily vertical movement
            dx = 0  # Force pure vertical movement
            distance = abs_dy
        
        # Calculate adaptive speed with more conservative approach
        speed = self._calculate_adaptive_speed(distance)
        
        # Calculate normalized movement direction
        if distance > 0:
            vx = (dx / distance) * speed
            vy = (dy / distance) * speed
        else:
            vx = vy = 0
        
        # Apply additional smoothing for very small movements
        if distance < 0.05:
            vx *= 0.5
            vy *= 0.5
        
        # Apply Mecanum kinematics
        wheel_speeds = self.kinematics.calculate_wheel_speeds(vx, vy)
        
        # Apply speed limiting to prevent wheel slip
        max_wheel_speed = max(abs(speed) for speed in wheel_speeds)
        if max_wheel_speed > Config.MAX_SPEED:
            scale_factor = Config.MAX_SPEED / max_wheel_speed
            wheel_speeds = tuple(speed * scale_factor for speed in wheel_speeds)
        
        self.motor_controller.set_speeds(*wheel_speeds)
        
        # Debug output
        if verbose:
            self._debug_movement(current_pos, (target_x, target_y), distance, vx, vy, wheel_speeds)
        return False

    def _calculate_adaptive_speed(self, distance):
        """Calculate speed based on distance to target"""
        # return Config.MAX_SPEED  # Default speed
        if distance < 0.1:
            return Config.MAX_SPEED * 0.3  # Slow for precision
        elif distance < 0.3:
            return Config.MAX_SPEED * 0.5  # Medium speed
        else:
            return Config.MAX_SPEED * 0.7  # Normal speed
    
    def _debug_movement(self, current_pos, target_pos, distance, vx, vy, wheel_speeds):
        """Print debug information periodically"""
        self._debug_counter += 1
        if self._debug_counter % Config.DEBUG_FREQUENCY == 0:
            current_x, current_y = current_pos
            target_x, target_y = target_pos
            fl, fr, rl, rr = wheel_speeds
            
            print(f"  → Moving... Current: ({current_x:.3f}, {current_y:.3f}), "
                  f"Target: ({target_x:.3f}, {target_y:.3f}), Distance: {distance:.4f}")
            print(f"      Velocity: vx={vx:.3f}, vy={vy:.3f}")
            print(f"      Wheels: FL={fl:.2f}, FR={fr:.2f}, RL={rl:.2f}, RR={rr:.2f}")


class MecanumCellController:
    """Main controller class for cell-based mecanum robot navigation"""
    
    def __init__(self):
        self.robot = None
        self.timestep = None
        self.motor_controller = None
        self.position_tracker = None
        self.movement_controller = None
        self.sensors = {}
        self._initialized = False
    
    def initialize(self):
        """Initialize the robot and all subsystems"""
        if self._initialized:
            print("Controller already initialized!")
            return True
        
        try:
            # Initialize robot
            self.robot = Robot()
            self.timestep = int(self.robot.getBasicTimeStep())
            
            # Initialize motors
            motor_devices = self._get_motor_devices()
            self.motor_controller = MecanumMotorController(*motor_devices)
            
            # Initialize GPS
            gps = self.robot.getDevice('gps')
            gps.enable(self.timestep)
            self.position_tracker = PositionTracker(gps)
            
            # Initialize movement controller
            self.movement_controller = MovementController(
                self.robot, self.motor_controller, self.position_tracker
            )
            
            # Initialize sensors
            self._initialize_sensors()
            
            self._initialized = True
            print("Mecanum Cell Controller initialized successfully!")
            print("Available functions: move_left(), move_right(), move_top(), move_bottom()")
            return True
            
        except Exception as e:
            print(f"ERROR: Failed to initialize controller: {e}")
            return False
    
    def _get_motor_devices(self):
        """Get motor devices in correct order"""
        return (
            self.robot.getDevice('front_left_motor'),
            self.robot.getDevice('front_right_motor'),
            self.robot.getDevice('rear_left_motor'),
            self.robot.getDevice('rear_right_motor')
        )
    
    def _initialize_sensors(self):
        """Initialize distance sensors"""
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
        """Wait for GPS to provide valid position data"""
        if not self._initialized:
            print("ERROR: Controller not initialized! Call initialize() first.")
            return None
        
        print("Waiting for GPS to initialize...")
        while self.robot.step(self.timestep) != -1:
            position = self.position_tracker.get_current_position()
            if position is not None:
                print(f"GPS ready! Starting position: ({position[0]:.2f}, {position[1]:.2f})")
                return position
        return None
    
    def move_to_cell(self, target_cell_x, target_cell_y, verbose=True):
        """
        Move to specific cell coordinates
        
        Args:
            target_cell_x (int): Target cell X coordinate
            target_cell_y (int): Target cell Y coordinate
            verbose (bool): Enable debug output
        """
        if not self._initialized:
            print("ERROR: Controller not initialized!")
            return False
        
        target_gps_x, target_gps_y = self.position_tracker.cell_to_gps(target_cell_x, target_cell_y)
        
        if verbose:
            current_pos = self.position_tracker.get_current_position()
            if current_pos:
                current_cell = self.position_tracker.gps_to_cell(*current_pos)
                print(f"Moving from cell {current_cell} to cell ({target_cell_x}, {target_cell_y})")
        
        step_count = 0
        while self.robot.step(self.timestep) != -1:
            step_count += 1
            if self.movement_controller.move_to_position(target_gps_x, target_gps_y, verbose):
                if verbose:
                    final_pos = self.position_tracker.get_current_position()
                    if final_pos:
                        final_cell = self.position_tracker.gps_to_cell(*final_pos)
                        distance_error = math.sqrt((final_pos[0] - target_gps_x)**2 + 
                                                 (final_pos[1] - target_gps_y)**2)
                        print(f"✓ Movement complete! Steps: {step_count}")
                        print(f"  Final GPS: ({final_pos[0]:.3f}, {final_pos[1]:.3f})")
                        print(f"  Final cell: {final_cell}")
                        print(f"  Distance error: {distance_error:.4f} units")
                return True
        return False
    
    def move_left(self, verbose=True):
        """Move left by one cell (-X direction)"""
        current_pos = self.position_tracker.get_current_position()
        if current_pos is None:
            print("GPS not ready, cannot move")
            return False
        
        current_cell_x, current_cell_y = self.position_tracker.gps_to_cell(*current_pos)
        if verbose:
            print("=== MOVE LEFT ===")
        return self.move_to_cell(current_cell_x - 1, current_cell_y, verbose)
    
    def move_right(self, verbose=True):
        """Move right by one cell (+X direction)"""
        current_pos = self.position_tracker.get_current_position()
        if current_pos is None:
            print("GPS not ready, cannot move")
            return False
        
        current_cell_x, current_cell_y = self.position_tracker.gps_to_cell(*current_pos)
        if verbose:
            print("=== MOVE RIGHT ===")
        return self.move_to_cell(current_cell_x + 1, current_cell_y, verbose)
    
    def move_top(self, verbose=True):
        """Move top by one cell (+Y direction)"""
        current_pos = self.position_tracker.get_current_position()
        if current_pos is None:
            print("GPS not ready, cannot move")
            return False
        
        current_cell_x, current_cell_y = self.position_tracker.gps_to_cell(*current_pos)
        if verbose:
            print("=== MOVE TOP ===")
        return self.move_to_cell(current_cell_x, current_cell_y + 1, verbose)
    
    def move_bottom(self, verbose=True):
        """Move bottom by one cell (-Y direction)"""
        current_pos = self.position_tracker.get_current_position()
        if current_pos is None:
            print("GPS not ready, cannot move")
            return False
        
        current_cell_x, current_cell_y = self.position_tracker.gps_to_cell(*current_pos)
        if verbose:
            print("=== MOVE BOTTOM ===")
        return self.move_to_cell(current_cell_x, current_cell_y - 1, verbose)
    
    def get_current_cell(self):
        """Get current cell coordinates"""
        current_pos = self.position_tracker.get_current_position()
        if current_pos is None:
            return None
        return self.position_tracker.gps_to_cell(*current_pos)
    
    def get_current_position(self):
        """Get current GPS coordinates"""
        return self.position_tracker.get_current_position()
    
    def get_sensor_readings(self):
        """Get all distance sensor readings"""
        readings = {}
        for name, sensor in self.sensors.items():
            try:
                readings[name.replace('_distance_sensor', '')] = sensor.getValue()
            except:
                readings[name.replace('_distance_sensor', '')] = None
        return readings

    def _detect_wall_between_cells(self, from_cell, to_cell):
        """
        Alternative method: Use sensor readings to detect walls between cells
        This is more efficient than physical testing but requires accurate sensor positioning
        """
        # Calculate the direction from from_cell to to_cell
        dx = to_cell[0] - from_cell[0]
        dy = to_cell[1] - from_cell[1]
        
        # Move to the from_cell and orient towards to_cell
        if not self.move_to_cell(from_cell[0], from_cell[1], verbose=False):
            return True  # Assume wall if can't reach starting position
        
        # Get sensor readings
        sensors = self.get_sensor_readings()
        
        # Check the appropriate sensor based on movement direction
        sensor_threshold = Config.SENSOR_THRESHOLD
        
        if dx == 1:  # Moving right
            return sensors.get('right', float('inf')) < sensor_threshold
        elif dx == -1:  # Moving left
            return sensors.get('left', float('inf')) < sensor_threshold
        elif dy == 1:  # Moving up
            return sensors.get('front', float('inf')) < sensor_threshold
        elif dy == -1:  # Moving down
            return sensors.get('back', float('inf')) < sensor_threshold
        return True  # Default to wall if uncertain

    def get_front_sensor(self):
        """Get front distance sensor reading"""
        return self.sensors.get('front_distance_sensor', None).getValue() if 'front_distance_sensor' in self.sensors else None
     
    def stop(self):
        """Stop the robot"""
        if self.motor_controller:
            self.motor_controller.stop()
        
    def print_visited_cells(self, visited_cells):
        """Print all visited cells with (0,0) at bottom left"""
        for row in range(11, -1, -1):  # Start from top row (11) and go down to 0
            line = ""
            for col in range(12):  # Go from left (0) to right (11)
                if (col, row) in visited_cells:
                    line += "X "
                else:
                    line += ". "
            print(line)

    def run_left_wall_following(self):
        """
        Run Left Wall Following algorithm to navigate through the maze
        The robot follows the left wall until it reaches the end cell (11, 11)
        """
        print("=== Starting Left Wall Following Navigation ===")
        print("Goal: Navigate from start cell to end cell (11,11) by following the left wall")
        
        # Target end cell
        end_cell = (11, 11)
        
        # Get starting position
        start_pos = self.get_current_cell()
        if start_pos is None:
            print("ERROR: Cannot get starting position!")
            return False
        
        current_cell = start_pos
        visited = set()
        visited.add(current_cell)
        
        print(f"Starting Left Wall Following from cell: {current_cell}")
        print(f"Target cell: {end_cell}")
        
        # Direction mappings: 0=North(up), 1=East(right), 2=South(down), 3=West(left)
        directions = ['up', 'right', 'down', 'left']
        direction_vectors = [(0, 1), (1, 0), (0, -1), (-1, 0)]
        sensor_mapping = ['front', 'right', 'back', 'left']
        
        # Start facing North (up)
        current_direction = 0
        
        step_count = 0
        max_steps = 1000  # Safety limit
        
        while current_cell != end_cell and step_count < max_steps:
            step_count += 1
            visited.add(current_cell)
            self.print_visited_cells(visited)
            print(f"\n--- Step {step_count} ---")
            print(f"Current cell: {current_cell}")
            print(f"Current direction: {directions[current_direction]}")
            
            # Get sensor readings
            sensors = self.get_sensor_readings()
            print(f"Sensor readings: {sensors}")
            
            # Left wall following algorithm:
            # 1. Try to turn left and move
            # 2. If can't turn left, try to go straight
            # 3. If can't go straight, turn right
            # 4. If can't turn right, turn around (180 degrees)
            
            moved = False
            
            # Step 1: Try to turn left and move
            left_direction = (current_direction - 1) % 4
            left_dx, left_dy = direction_vectors[left_direction]
            left_cell = (current_cell[0] + left_dx, current_cell[1] + left_dy)
            left_sensor = sensor_mapping[left_direction]
            
            if self._is_valid_move(left_cell, sensors.get(left_sensor, 0)):
                print(f"  Left turn possible: turning {directions[left_direction]} and moving to {left_cell}")
                current_direction = left_direction
                if self._execute_move(directions[current_direction]):
                    current_cell = self.get_current_cell() or left_cell
                    moved = True
                    print(f"  Successfully moved to: {current_cell}")
            
            # Step 2: If can't turn left, try to go straight
            if not moved:
                straight_dx, straight_dy = direction_vectors[current_direction]
                straight_cell = (current_cell[0] + straight_dx, current_cell[1] + straight_dy)
                straight_sensor = sensor_mapping[current_direction]
                
                if self._is_valid_move(straight_cell, sensors.get(straight_sensor, 0)):
                    print(f"  Going straight: moving {directions[current_direction]} to {straight_cell}")
                    if self._execute_move(directions[current_direction]):
                        current_cell = self.get_current_cell() or straight_cell
                        moved = True
                        print(f"  Successfully moved to: {current_cell}")
            
            # Step 3: If can't go straight, turn right
            if not moved:
                right_direction = (current_direction + 1) % 4
                right_dx, right_dy = direction_vectors[right_direction]
                right_cell = (current_cell[0] + right_dx, current_cell[1] + right_dy)
                right_sensor = sensor_mapping[right_direction]
                
                if self._is_valid_move(right_cell, sensors.get(right_sensor, 0)):
                    print(f"  Right turn: turning {directions[right_direction]} and moving to {right_cell}")
                    current_direction = right_direction
                    if self._execute_move(directions[current_direction]):
                        current_cell = self.get_current_cell() or right_cell
                        moved = True
                        print(f"  Successfully moved to: {current_cell}")
                else:
                    # Just turn right without moving
                    print(f"  Turning right to face {directions[right_direction]} (no movement)")
                    current_direction = right_direction
                    moved = True
            
            # Step 4: If can't turn right, turn around (180 degrees)
            if not moved:
                print("  Dead end: turning around 180 degrees")
                current_direction = (current_direction + 2) % 4
                print(f"  Now facing: {directions[current_direction]}")
                moved = True
            
            # Small delay for visualization
            time.sleep(0.1)
        
        # Check if we reached the goal
        if current_cell == end_cell:
            print(f"\n🎉 SUCCESS! Reached end cell {end_cell} in {step_count} steps using Left Wall Following!")
            print(f"Total cells visited: {len(visited)}")
            return True
        else:
            print(f"\n❌ FAILED to reach end cell after {step_count} steps")
            print(f"Final position: {current_cell}")
            return False

    def run_right_wall_following(self):
        """
        Run Right Wall Following algorithm to navigate through the maze
        The robot follows the right wall until it reaches the end cell (11, 11)
        """
        print("=== Starting Right Wall Following Navigation ===")
        print("Goal: Navigate from start cell to end cell (11,11) by following the right wall")
        
        # Target end cell
        end_cell = (11, 11)
        
        # Get starting position
        start_pos = self.get_current_cell()
        if start_pos is None:
            print("ERROR: Cannot get starting position!")
            return False
        
        current_cell = start_pos
        visited = set()
        visited.add(current_cell)
        
        print(f"Starting Right Wall Following from cell: {current_cell}")
        print(f"Target cell: {end_cell}")
        
        # Direction mappings: 0=North(up), 1=East(right), 2=South(down), 3=West(left)
        directions = ['up', 'right', 'down', 'left']
        direction_vectors = [(0, 1), (1, 0), (0, -1), (-1, 0)]
        sensor_mapping = ['front', 'right', 'back', 'left']
        
        # Start facing North (up)
        current_direction = 0
        
        step_count = 0
        max_steps = 1000  # Safety limit
        
        while current_cell != end_cell and step_count < max_steps:
            step_count += 1
            visited.add(current_cell)
            self.print_visited_cells(visited)
            print(f"\n--- Step {step_count} ---")
            print(f"Current cell: {current_cell}")
            print(f"Current direction: {directions[current_direction]}")
            
            # Get sensor readings
            sensors = self.get_sensor_readings()
            print(f"Sensor readings: {sensors}")
            
            # Right wall following algorithm:
            # 1. Try to turn right and move
            # 2. If can't turn right, try to go straight
            # 3. If can't go straight, turn left
            # 4. If can't turn left, turn around (180 degrees)
            
            moved = False
            
            # Step 1: Try to turn right and move
            right_direction = (current_direction + 1) % 4
            right_dx, right_dy = direction_vectors[right_direction]
            right_cell = (current_cell[0] + right_dx, current_cell[1] + right_dy)
            right_sensor = sensor_mapping[right_direction]
            
            if self._is_valid_move(right_cell, sensors.get(right_sensor, 0)):
                print(f"  Right turn possible: turning {directions[right_direction]} and moving to {right_cell}")
                current_direction = right_direction
                if self._execute_move(directions[current_direction]):
                    current_cell = self.get_current_cell() or right_cell
                    moved = True
                    print(f"  Successfully moved to: {current_cell}")
            
            # Step 2: If can't turn right, try to go straight
            if not moved:
                straight_dx, straight_dy = direction_vectors[current_direction]
                straight_cell = (current_cell[0] + straight_dx, current_cell[1] + straight_dy)
                straight_sensor = sensor_mapping[current_direction]
                
                if self._is_valid_move(straight_cell, sensors.get(straight_sensor, 0)):
                    print(f"  Going straight: moving {directions[current_direction]} to {straight_cell}")
                    if self._execute_move(directions[current_direction]):
                        current_cell = self.get_current_cell() or straight_cell
                        moved = True
                        print(f"  Successfully moved to: {current_cell}")
            
            # Step 3: If can't go straight, turn left
            if not moved:
                left_direction = (current_direction - 1) % 4
                left_dx, left_dy = direction_vectors[left_direction]
                left_cell = (current_cell[0] + left_dx, current_cell[1] + left_dy)
                left_sensor = sensor_mapping[left_direction]
                
                if self._is_valid_move(left_cell, sensors.get(left_sensor, 0)):
                    print(f"  Left turn: turning {directions[left_direction]} and moving to {left_cell}")
                    current_direction = left_direction
                    if self._execute_move(directions[current_direction]):
                        current_cell = self.get_current_cell() or left_cell
                        moved = True
                        print(f"  Successfully moved to: {current_cell}")
                else:
                    # Just turn left without moving
                    print(f"  Turning left to face {directions[left_direction]} (no movement)")
                    current_direction = left_direction
                    moved = True
            
            # Step 4: If can't turn left, turn around (180 degrees)
            if not moved:
                print("  Dead end: turning around 180 degrees")
                current_direction = (current_direction + 2) % 4
                print(f"  Now facing: {directions[current_direction]}")
                moved = True
            
            # Small delay for visualization
            time.sleep(0.1)
        
        # Check if we reached the goal
        if current_cell == end_cell:
            print(f"\n🎉 SUCCESS! Reached end cell {end_cell} in {step_count} steps using Right Wall Following!")
            print(f"Total cells visited: {len(visited)}")
            return True
        else:
            print(f"\n❌ FAILED to reach end cell after {step_count} steps")
            print(f"Final position: {current_cell}")
            return False
    
    def _is_valid_move(self, target_cell, sensor_value):
        """Check if a move to target_cell is valid (within bounds and no wall)"""
        # Check bounds
        if not (0 <= target_cell[0] <= 11 and 0 <= target_cell[1] <= 11):
            return False
        
        # Check for wall using sensor reading
        if sensor_value is not None and sensor_value < Config.SENSOR_THRESHOLD:  # Wall detected
            return False
        
        return True
    
    def _execute_move(self, direction):
        """Execute a move in the specified direction"""
        if direction == 'up':
            return self.move_top(verbose=False)
        elif direction == 'right':
            return self.move_right(verbose=False)
        elif direction == 'down':
            return self.move_bottom(verbose=False)
        elif direction == 'left':
            return self.move_left(verbose=False)
        return False

    def run_dfs(self):
        """
        Run DFS (Depth-First Search) algorithm to navigate to the end cell
        End cell is at position (11, 11) - top right corner of the maze
        """
        print("=== Starting DFS Maze Navigation ===")
        print("Goal: Navigate from start cell (0,0) to end cell (11,11)")
        
        # DFS algorithm parameters
        end_cell = (11, 11)  # Top right corner
        visited = set()
        path_stack = []
        
        # Get starting position
        start_pos = self.get_current_cell()
        if start_pos is None:
            print("ERROR: Cannot get starting position!")
            return False
        
        current_cell = start_pos
        visited.add(current_cell)
        path_stack.append(current_cell)
        
        print(f"Starting DFS from cell: {current_cell}")
        print(f"Target cell: {end_cell}")
        
        step_count = 0
        max_steps = 1000  # Safety limit
        
        while current_cell != end_cell and step_count < max_steps:
            step_count += 1
            self.print_visited_cells(visited)
            print(f"\n--- Step {step_count} ---")
            print(f"Current cell: {current_cell}")
            
            # Get sensor readings to detect walls
            sensors = self.get_sensor_readings()
            print(f"Sensor readings: {sensors}")
            
            # Define possible moves (right, up, left, down) - DFS order
            moves = [
                ('right', (1, 0), 'right'),
                ('up', (0, 1), 'front'), 
                ('left', (-1, 0), 'left'),
                ('down', (0, -1), 'back')
            ]
            
            found_move = False
            
            # Try each direction in DFS order
            for direction, (dx, dy), sensor_key in moves:
                next_cell = (current_cell[0] + dx, current_cell[1] + dy)
                
                # Check bounds
                if not (0 <= next_cell[0] <= 11 and 0 <= next_cell[1] <= 11):
                    continue
                
                # Check if already visited
                if next_cell in visited:
                    continue
                
                # Check for wall using sensors
                sensor_value = sensors.get(sensor_key, None)
                print("Sensor value for", sensor_key, ":", sensor_value)
                if sensor_value is not None and sensor_value < 380:  # Wall detected
                    print(f"  Wall detected {direction} (sensor: {sensor_value:.3f})")
                    continue
                
                # Valid move found - execute it
                print(f"  Moving {direction} to cell: {next_cell}")
                
                success = False
                if direction == 'right':
                    success = self.move_right(verbose=True)
                elif direction == 'up':
                    success = self.move_top(verbose=True)
                elif direction == 'left':
                    success = self.move_left(verbose=True)
                elif direction == 'down':
                    success = self.move_bottom(verbose=True)
                
                if success:
                    current_cell = self.get_current_cell()
                    if current_cell is None:
                        current_cell = next_cell  # Fallback
                    
                    visited.add(current_cell)
                    path_stack.append(current_cell)
                    found_move = True
                    print(f"  Successfully moved to: {current_cell}")
                    break
                else:
                    print(f"  Failed to move {direction}")
            
            # If no valid move found, backtrack
            if not found_move:
                if len(path_stack) <= 1:
                    print("ERROR: No more moves available and cannot backtrack!")
                    break
                
                # Remove current cell from stack
                path_stack.pop()
                if not path_stack:
                    print("ERROR: Path stack is empty!")
                    break
                
                # Get previous cell to backtrack to
                target_cell = path_stack[-1]
                print(f"  Backtracking to cell: {target_cell}")
                
                # Move to target cell
                if self.move_to_cell(target_cell[0], target_cell[1], verbose=True):
                    current_cell = target_cell
                    print(f"  Backtracked to: {current_cell}")
                else:
                    print(f"  Failed to backtrack to: {target_cell}")
                    break
            
            # Small delay for visualization
            time.sleep(0.1)
        
        # Check if we reached the goal
        if current_cell == end_cell:
            print(f"\n🎉 SUCCESS! Reached end cell {end_cell} in {step_count} steps!")
            print(f"Final path length: {len(path_stack)} cells")
            print(f"Path taken: {' -> '.join(map(str, path_stack))}")
            return True
        else:
            print(f"\n❌ FAILED to reach end cell after {step_count} steps")
            print(f"Final position: {current_cell}")
            return False

    def run_smart_wall_following(self):
        """
        Enhanced wall following that builds a map as it explores
        Combines wall following with memory to avoid revisiting areas
        """
        print("=== Starting Smart Wall Following ===")
        print("Goal: Enhanced wall following with mapping")
        
        end_cell = (11, 11)
        start_pos = self.get_current_cell()
        
        if start_pos is None:
            print("ERROR: Cannot get starting position!")
            return False
        
        # Build a wall map as we explore
        wall_map = {}  # (from_cell, to_cell) -> is_blocked
        visited = set()
        path_taken = []
        
        current_cell = start_pos
        visited.add(current_cell)
        path_taken.append(current_cell)
        
        # Direction tracking for wall following
        directions = ['up', 'right', 'down', 'left']
        direction_vectors = [(0, 1), (1, 0), (0, -1), (-1, 0)]
        sensor_mapping = ['front', 'right', 'back', 'left']
        current_direction = 0  # Start facing up
        
        step_count = 0
        max_steps = 500
        
        print(f"Starting from {start_pos}, target: {end_cell}")
        
        while current_cell != end_cell and step_count < max_steps:
            step_count += 1
            visited.add(current_cell)
            
            if step_count % 20 == 0:
                self.print_visited_cells(visited)
            
            print(f"\n--- Step {step_count} ---")
            print(f"At {current_cell}, facing {directions[current_direction]}")
            
            # Update wall map with sensor readings
            self._update_wall_map(current_cell, wall_map)
            
            # Smart wall following with backtracking prevention
            next_direction, next_cell = self._choose_smart_direction(
                current_cell, current_direction, wall_map, visited, directions, direction_vectors
            )
            
            if next_cell is None:
                print("No valid moves available!")
                break
            
            print(f"Choosing to go {directions[next_direction]} to {next_cell}")
            
            # Execute the move
            if self._execute_move(directions[next_direction]):
                current_direction = next_direction
                current_cell = self.get_current_cell() or next_cell
                path_taken.append(current_cell)
                print(f"Successfully moved to: {current_cell}")
            else:
                print(f"Failed to move {directions[next_direction]}")
                # Mark as wall and try again
                wall_map[(current_cell, next_cell)] = True
        
        # Check success
        if current_cell == end_cell:
            print(f"\n🎉 SUCCESS! Reached goal using Smart Wall Following!")
            print(f"Steps taken: {step_count}")
            print(f"Cells visited: {len(visited)}")
            print(f"Path efficiency: {len(visited)/step_count*100:.1f}%")
            return True
        else:
            print(f"\n❌ FAILED to reach goal after {step_count} steps")
            print(f"Final position: {current_cell}")
            return False

    def _update_wall_map(self, current_cell, wall_map):
        """Update wall map based on current sensor readings"""
        sensors = self.get_sensor_readings()
        
        moves = [
            ('up', (0, 1), 'front'),
            ('right', (1, 0), 'right'), 
            ('down', (0, -1), 'back'),
            ('left', (-1, 0), 'left')
        ]
        
        for direction, (dx, dy), sensor_key in moves:
            neighbor = (current_cell[0] + dx, current_cell[1] + dy)
            
            # Check bounds
            if not (0 <= neighbor[0] <= 11 and 0 <= neighbor[1] <= 11):
                wall_map[(current_cell, neighbor)] = True
                continue
            
            sensor_value = sensors.get(sensor_key, float('inf'))
            wall_map[(current_cell, neighbor)] = sensor_value < Config.SENSOR_THRESHOLD
            
        # Debug output
        walls = [direction for direction, (dx, dy), sensor_key in moves 
                if wall_map.get((current_cell, (current_cell[0] + dx, current_cell[1] + dy)), False)]
        print(f"  Walls detected: {walls}")

    def _choose_smart_direction(self, current_cell, current_direction, wall_map, visited, directions, direction_vectors):
        """Choose the best direction using smart wall following logic"""
        
        # Priority order: left turn, straight, right turn, u-turn
        direction_priorities = [
            (current_direction - 1) % 4,  # Left turn
            current_direction,              # Straight
            (current_direction + 1) % 4,   # Right turn
            (current_direction + 2) % 4    # U-turn
        ]
        
        for next_direction in direction_priorities:
            dx, dy = direction_vectors[next_direction]
            next_cell = (current_cell[0] + dx, current_cell[1] + dy)
            
            # Check bounds
            if not (0 <= next_cell[0] <= 11 and 0 <= next_cell[1] <= 11):
                continue
            
            # Check if there's a wall
            if wall_map.get((current_cell, next_cell), False):
                continue
            
            # Prefer unvisited cells, but allow revisiting if necessary
            if next_cell not in visited:
                return next_direction, next_cell
        
        # If all preferred moves lead to visited cells, pick the least recently visited
        for next_direction in direction_priorities:
            dx, dy = direction_vectors[next_direction]
            next_cell = (current_cell[0] + dx, current_cell[1] + dy)
            
            # Check bounds and walls again
            if (0 <= next_cell[0] <= 11 and 0 <= next_cell[1] <= 11 and 
                not wall_map.get((current_cell, next_cell), False)):
                return next_direction, next_cell
        
        return None, None

def run_robot():
    """Main function to run the robot controller"""
    controller = MecanumCellController()
    
    # Initialize the controller
    if not controller.initialize():
        return
    
    # Wait for GPS
    if controller.wait_for_gps() is None:
        print("ERROR: Could not initialize GPS!")
        return
    
    controller.run_dfs()


if __name__ == "__main__":
    run_robot()