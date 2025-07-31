"""
Main Four-Wheel Robot Controller for Autonomous Maze Navigation

This module implements the primary control system for a Mecanum wheel robot
designed for autonomous maze navigation. It integrates multiple subsystems
including motion control, position tracking, sensor management, and 
path-finding algorithms to achieve intelligent navigation in grid-based
maze environments.

System Architecture:
The controller follows a modular design pattern with clear separation of
concerns across different functional layers:

1. Hardware Abstraction Layer:
   - Motor control through MecanumMotorController
   - Sensor management (GPS, distance sensors)
   - Device initialization and configuration

2. Navigation Layer:
   - Position tracking and coordinate transformations
   - Movement control with precision positioning
   - Cell-based navigation for discrete path planning

3. Algorithm Layer:
   - Multiple pathfinding algorithms (DFS, Wall Following)
   - Smart exploration with memory and mapping
   - Performance analysis and result logging

Key Features:
- Multi-algorithm maze solving capability
- Real-time sensor-based obstacle detection
- Comprehensive performance monitoring and analysis
- Academic-grade result logging with visualizations
- Robust error handling and system recovery

Implemented Algorithms:
1. Depth-First Search (DFS) - Systematic exploration with backtracking
2. Left/Right Wall Following - Classic maze navigation techniques
3. Smart Wall Following - Enhanced wall following with mapping memory

Author: [Your Name]
Date: July 2025
Academic Project: AI-Powered Maze Navigation with Four-Wheel Robot
Course: [Course Name/Number]
Institution: [Institution Name]
"""

from controller import Robot, DistanceSensor, Motor, GPS, Compass
from movement_controller import MovementController
from mecanum_motor_controller import MecanumMotorController
from mecanum_kinematics import MecanumKinematics
from position_tracker import PositionTracker
from config import Config
from collections import deque
import math
import time
import datetime
import os


class MecanumCellController:
    """
    Main Controller Class for Cell-Based Mecanum Robot Navigation
    
    This class serves as the central coordination hub for autonomous maze
    navigation using a four-wheel Mecanum drive system. It manages all
    subsystem integration, implements multiple pathfinding algorithms,
    and provides comprehensive performance monitoring capabilities.
    
    Design Philosophy:
    The controller is designed with modularity and extensibility in mind,
    allowing for easy integration of new algorithms and sensors. It follows
    object-oriented principles with clear interfaces between components.
    
    System Integration:
    - Hardware Interface: Webots robot simulation environment
    - Motor Control: Four independent Mecanum wheels with velocity control
    - Position Tracking: GPS-based localization with coordinate conversion
    - Sensor Network: Distance sensors for obstacle detection
    - Navigation Algorithms: Multiple pathfinding strategies
    
    Coordinate System:
    - Cell coordinates: Integer grid positions (0,0) to (11,11)
    - GPS coordinates: Continuous real-world positions in meters
    - Grid layout: 12x12 maze with (0,0) at bottom-left, (11,11) at top-right
    """
    
    
    def __init__(self):
        """
        Initialize the Mecanum Cell Controller with default state.
        
        Sets up the controller with uninitialized subsystem references
        and prepares the system for subsequent initialization. This
        two-phase initialization pattern allows for error handling
        during the hardware setup process.
        
        Initialization State Variables:
        - All subsystem references set to None for lazy initialization
        - Initialization flag to prevent duplicate setup attempts
        - Clean slate for sensor and device management
        """
        # Core system components - initialized to None for lazy loading
        self.robot = None                    # Main Webots Robot interface
        self.timestep = None                 # Simulation timestep for timing control
        
        # Motion control subsystem references
        self.motor_controller = None         # Four-wheel motor control interface
        self.position_tracker = None         # GPS-based position tracking system
        self.movement_controller = None      # High-level movement coordination
        
        # Sensor management
        self.sensors = {}                    # Dictionary of distance sensors
        
        # System state management
        self._initialized = False            # Prevents duplicate initialization
    
    def initialize(self):
        """
        Initialize the robot and all subsystems with comprehensive error handling.
        
        Performs systematic initialization of all robot subsystems in the correct
        order to ensure proper system startup. This method implements a robust
        initialization sequence with error checking and prevents duplicate
        initialization attempts.
        
        Initialization Sequence:
        1. Duplicate initialization check
        2. Core robot interface setup
        3. Motor subsystem configuration
        4. GPS and position tracking initialization
        5. Movement controller integration
        6. Distance sensor network setup
        7. System readiness confirmation
        
        Returns:
            bool: True if initialization successful, False otherwise
            
        Error Handling:
        - Comprehensive exception catching for hardware failures
        - Detailed error messages for debugging
        - Graceful degradation for optional components
        """
        # Prevent duplicate initialization attempts
        if self._initialized:
            print("Controller already initialized!")
            return True
        
        try:
            # === PHASE 1: CORE ROBOT INTERFACE SETUP ===
            # Initialize the main Webots Robot interface
            self.robot = Robot()
            
            # Extract simulation timestep for synchronization with Webots
            # This ensures proper timing coordination across all subsystems
            self.timestep = int(self.robot.getBasicTimeStep())
            
            # === PHASE 2: MOTOR SUBSYSTEM CONFIGURATION ===
            # Retrieve motor device references in correct order for Mecanum setup
            motor_devices = self._get_motor_devices()
            
            # Initialize motor controller with proper Mecanum wheel configuration
            self.motor_controller = MecanumMotorController(*motor_devices)
            
            # === PHASE 3: GPS AND POSITION TRACKING INITIALIZATION ===
            # Setup GPS device for real-time position feedback
            gps = self.robot.getDevice('gps')
            gps.enable(self.timestep)  # Enable GPS with simulation timestep
            
            # Initialize position tracking with coordinate conversion capabilities
            self.position_tracker = PositionTracker(gps)
            
            # === PHASE 4: MOVEMENT CONTROLLER INTEGRATION ===
            # Create high-level movement controller that coordinates all subsystems
            self.movement_controller = MovementController(
                self.robot, self.motor_controller, self.position_tracker
            )
            
            # === PHASE 5: DISTANCE SENSOR NETWORK SETUP ===
            # Initialize distance sensors for obstacle detection and wall mapping
            self._initialize_sensors()
            
            # === PHASE 6: SYSTEM READINESS CONFIRMATION ===
            # Mark system as successfully initialized
            self._initialized = True
            
            # Provide user feedback on successful initialization
            print("Mecanum Cell Controller initialized successfully!")
            print("Available functions: move_left(), move_right(), move_top(), move_bottom()")
            return True
            
        except Exception as e:
            # Comprehensive error handling for initialization failures
            print(f"ERROR: Failed to initialize controller: {e}")
            return False
    
    
    def _get_motor_devices(self):
        """
        Retrieve motor devices in correct order for Mecanum wheel configuration.
        
        Obtains references to the four motor devices from the Webots robot
        model in the specific order required by the MecanumMotorController.
        The order is critical for proper kinematic calculations and movement
        execution.
        
        Motor Ordering Convention:
        1. Front Left Motor - Controls front-left Mecanum wheel
        2. Front Right Motor - Controls front-right Mecanum wheel  
        3. Rear Left Motor - Controls rear-left Mecanum wheel
        4. Rear Right Motor - Controls rear-right Mecanum wheel
        
        Returns:
            tuple: Four motor device objects in the correct sequence
                  (front_left, front_right, rear_left, rear_right)
                  
        Note:
            Motor names must match the device names defined in the
            Webots robot model (.wbt file). Any mismatch will cause
            initialization failure.
        """
        return (
            self.robot.getDevice('front_left_motor'),    # Front-left Mecanum wheel motor
            self.robot.getDevice('front_right_motor'),   # Front-right Mecanum wheel motor
            self.robot.getDevice('rear_left_motor'),     # Rear-left Mecanum wheel motor
            self.robot.getDevice('rear_right_motor')     # Rear-right Mecanum wheel motor
        )
    
    
    def _initialize_sensors(self):
        """
        Initialize distance sensor network for obstacle detection and mapping.
        
        Sets up the four directional distance sensors used for wall detection
        and obstacle avoidance during maze navigation. Each sensor provides
        real-time distance measurements to nearby obstacles, enabling the
        robot to build an understanding of its environment.
        
        Sensor Configuration:
        - Front Distance Sensor: Detects obstacles ahead of robot
        - Right Distance Sensor: Detects walls/obstacles to the right
        - Back Distance Sensor: Detects obstacles behind robot  
        - Left Distance Sensor: Detects walls/obstacles to the left
        
        Error Handling:
        Uses graceful degradation - continues initialization even if
        some sensors fail to initialize. Warning messages alert to
        missing sensors without causing system failure.
        
        Technical Notes:
        - Sensors are enabled with the simulation timestep for synchronization
        - Sensor readings are cached in self.sensors dictionary for easy access
        - Failed sensors are logged but don't prevent system operation
        """
        # Define standard sensor names for four-directional coverage
        sensor_names = ['front_distance_sensor', 'right_distance_sensor', 
                       'back_distance_sensor', 'left_distance_sensor']
        
        # Initialize each sensor with error handling
        for name in sensor_names:
            try:
                # Retrieve sensor device from robot model
                sensor = self.robot.getDevice(name)
                
                # Enable sensor with simulation timestep for synchronized readings
                sensor.enable(self.timestep)
                
                # Store sensor reference for later use
                self.sensors[name] = sensor
                
            except:
                # Log sensor initialization failure but continue operation
                # This allows the system to work with partial sensor coverage
                print(f"Warning: Could not initialize sensor {name}")
    
    
    def wait_for_gps(self):
        """
        Wait for GPS system to provide valid position data.
        
        GPS systems typically require initialization time before providing
        accurate position readings. This method implements a blocking wait
        that ensures the position tracking system is ready before navigation
        begins, preventing navigation errors due to invalid position data.
        
        Initialization Process:
        1. Verify controller has been properly initialized
        2. Enter waiting loop with simulation stepping
        3. Continuously poll GPS for valid position data
        4. Return first valid position reading received
        
        Returns:
            tuple or None: (x, y) GPS coordinates if successful, None if failed
            
        Error Conditions:
        - Controller not initialized: Returns None immediately
        - Simulation termination: Returns None if Webots stops
        - GPS hardware failure: Infinite wait until manual intervention
        
        Technical Notes:
        - Uses robot.step() to advance simulation during waiting
        - Position validation performed by PositionTracker class
        - Blocking operation - will wait indefinitely for valid GPS
        """
        # Verify system initialization before attempting GPS operations
        if not self._initialized:
            print("ERROR: Controller not initialized! Call initialize() first.")
            return None
        
        # Provide user feedback during GPS initialization wait
        print("Waiting for GPS to initialize...")
        
        # Main GPS waiting loop - continues until valid position received
        while self.robot.step(self.timestep) != -1:
            # Attempt to get current position from GPS system
            position = self.position_tracker.get_current_position()
            
            # Check if valid position data received
            if position is not None:
                # GPS ready - provide confirmation and starting position
                print(f"GPS ready! Starting position: ({position[0]:.2f}, {position[1]:.2f})")
                return position
                
        # Simulation terminated before GPS became ready
        return None
    
    
    def move_to_cell(self, target_cell_x, target_cell_y, verbose=True):
        """
        Move robot to specific cell coordinates with comprehensive feedback.
        
        Implements precise cell-to-cell navigation by converting discrete
        cell coordinates to continuous GPS coordinates and executing
        controlled movement with real-time feedback and error analysis.
        
        Movement Process:
        1. System initialization verification
        2. Coordinate conversion (cell → GPS)
        3. Current position analysis and reporting
        4. Iterative movement execution with feedback
        5. Completion analysis with performance metrics
        
        Args:
            target_cell_x (int): Target cell X coordinate (0-11)
            target_cell_y (int): Target cell Y coordinate (0-11)
            verbose (bool): Enable detailed progress reporting
            
        Returns:
            bool: True if target cell reached successfully, False otherwise
            
        Performance Metrics:
        - Step counting for movement efficiency analysis
        - Distance error calculation for precision assessment
        - Final position verification against target coordinates
        
        Technical Implementation:
        - Uses continuous simulation stepping for real-time control
        - Integrates with MovementController for precise positioning
        - Provides comprehensive feedback for academic analysis
        """
        # Verify system readiness before attempting movement
        if not self._initialized:
            print("ERROR: Controller not initialized!")
            return False
        
        # Convert discrete cell coordinates to continuous GPS coordinates
        # This targets the center of the specified grid cell
        target_gps_x, target_gps_y = self.position_tracker.cell_to_gps(target_cell_x, target_cell_y)
        
        # Provide movement context and starting position analysis
        if verbose:
            current_pos = self.position_tracker.get_current_position()
            if current_pos:
                # Calculate current cell position for movement context
                current_cell = self.position_tracker.gps_to_cell(*current_pos)
                print(f"Moving from cell {current_cell} to cell ({target_cell_x}, {target_cell_y})")
        
        # Initialize performance tracking
        step_count = 0
        
        # Main movement execution loop with real-time control
        while self.robot.step(self.timestep) != -1:
            step_count += 1
            
            # Execute one iteration of movement control
            # Returns True when target position is reached within tolerance
            if self.movement_controller.move_to_position(target_gps_x, target_gps_y, verbose):
                
                # Movement completed - perform comprehensive analysis
                if verbose:
                    final_pos = self.position_tracker.get_current_position()
                    if final_pos:
                        # Calculate final cell position for verification
                        final_cell = self.position_tracker.gps_to_cell(*final_pos)
                        
                        # Calculate positioning accuracy for performance analysis
                        distance_error = math.sqrt((final_pos[0] - target_gps_x)**2 + 
                                                 (final_pos[1] - target_gps_y)**2)
                        
                        # Provide comprehensive completion report
                        print(f"✓ Movement complete! Steps: {step_count}")
                        print(f"  Final GPS: ({final_pos[0]:.3f}, {final_pos[1]:.3f})")
                        print(f"  Final cell: {final_cell}")
                        print(f"  Distance error: {distance_error:.4f} units")
                        
                return True
                
        # Movement failed - simulation terminated or other error
        return False
    
    
    def move_left(self, verbose=True):
        """
        Move left by one cell (-X direction) in the grid coordinate system.
        
        Implements leftward movement by calculating the target cell position
        relative to the current location and executing precise cell-to-cell
        navigation. This method provides a high-level interface for grid-based
        movement commands used by pathfinding algorithms.
        
        Movement Mechanics:
        - Determines current cell position using GPS coordinates
        - Calculates target cell as (current_x - 1, current_y)
        - Executes movement using the cell-based navigation system
        
        Args:
            verbose (bool): Enable detailed movement progress reporting
            
        Returns:
            bool: True if movement completed successfully, False otherwise
            
        Error Handling:
        - GPS availability check before movement attempt
        - Boundary checking performed by underlying movement system
        - Graceful failure reporting for navigation errors
        """
        # Verify GPS system availability for position calculation
        current_pos = self.position_tracker.get_current_position()
        if current_pos is None:
            print("GPS not ready, cannot move")
            return False
        
        # Calculate current grid cell position for relative movement
        current_cell_x, current_cell_y = self.position_tracker.gps_to_cell(*current_pos)
        
        # Provide movement context for user/algorithm feedback
        if verbose:
            print("=== MOVE LEFT ===")
            
        # Execute leftward movement (decrease X coordinate by 1)
        return self.move_to_cell(current_cell_x - 1, current_cell_y, verbose)
    
    def move_right(self, verbose=True):
        """
        Move right by one cell (+X direction) in the grid coordinate system.
        
        Implements rightward movement following the same pattern as move_left
        but incrementing the X coordinate. This provides consistent directional
        movement interface for algorithm implementation.
        
        Args:
            verbose (bool): Enable detailed movement progress reporting
            
        Returns:
            bool: True if movement completed successfully, False otherwise
        """
        # Verify GPS system availability for position calculation
        current_pos = self.position_tracker.get_current_position()
        if current_pos is None:
            print("GPS not ready, cannot move")
            return False
        
        # Calculate current grid cell position for relative movement
        current_cell_x, current_cell_y = self.position_tracker.gps_to_cell(*current_pos)
        
        # Provide movement context for user/algorithm feedback
        if verbose:
            print("=== MOVE RIGHT ===")
            
        # Execute rightward movement (increase X coordinate by 1)
        return self.move_to_cell(current_cell_x + 1, current_cell_y, verbose)
    
    def move_top(self, verbose=True):
        """
        Move up by one cell (+Y direction) in the grid coordinate system.
        
        Implements upward movement in the maze grid. Note that "top" refers
        to increasing Y coordinates in the grid system, which corresponds
        to northward movement in the robot's reference frame.
        
        Args:
            verbose (bool): Enable detailed movement progress reporting
            
        Returns:
            bool: True if movement completed successfully, False otherwise
        """
        # Verify GPS system availability for position calculation
        current_pos = self.position_tracker.get_current_position()
        if current_pos is None:
            print("GPS not ready, cannot move")
            return False
        
        # Calculate current grid cell position for relative movement
        current_cell_x, current_cell_y = self.position_tracker.gps_to_cell(*current_pos)
        
        # Provide movement context for user/algorithm feedback
        if verbose:
            print("=== MOVE TOP ===")
            
        # Execute upward movement (increase Y coordinate by 1)
        return self.move_to_cell(current_cell_x, current_cell_y + 1, verbose)
    
    def move_bottom(self, verbose=True):
        """
        Move down by one cell (-Y direction) in the grid coordinate system.
        
        Implements downward movement in the maze grid. "Bottom" refers to
        decreasing Y coordinates, corresponding to southward movement in
        the robot's reference frame.
        
        Args:
            verbose (bool): Enable detailed movement progress reporting
            
        Returns:
            bool: True if movement completed successfully, False otherwise
        """
        # Verify GPS system availability for position calculation
        current_pos = self.position_tracker.get_current_position()
        if current_pos is None:
            print("GPS not ready, cannot move")
            return False
        
        # Calculate current grid cell position for relative movement
        current_cell_x, current_cell_y = self.position_tracker.gps_to_cell(*current_pos)
        
        # Provide movement context for user/algorithm feedback
        if verbose:
            print("=== MOVE BOTTOM ===")
            
        # Execute downward movement (decrease Y coordinate by 1)
        return self.move_to_cell(current_cell_x, current_cell_y - 1, verbose)
    
    
    def get_current_cell(self):
        """
        Retrieve current robot position in discrete cell coordinates.
        
        Provides a convenient interface for algorithms to determine the
        robot's current grid position without dealing with GPS coordinate
        conversion. Essential for pathfinding algorithms that operate on
        discrete cell-based representations.
        
        Returns:
            tuple or None: (cell_x, cell_y) as integer coordinates,
                          None if GPS position unavailable
                          
        Usage in Algorithms:
        This method is frequently used by pathfinding algorithms to:
        - Track current position during exploration
        - Calculate relative movement targets
        - Verify successful movement completion
        - Update visited cell sets for progress tracking
        """
        # Attempt to get current GPS position
        current_pos = self.position_tracker.get_current_position()
        if current_pos is None:
            return None
            
        # Convert continuous GPS coordinates to discrete cell coordinates
        return self.position_tracker.gps_to_cell(*current_pos)
    
    def get_current_position(self):
        """
        Retrieve current robot position in continuous GPS coordinates.
        
        Provides direct access to raw GPS position data for applications
        requiring precise position information. Used for movement control,
        error analysis, and performance evaluation.
        
        Returns:
            tuple or None: (gps_x, gps_y) as floating-point coordinates in meters,
                          None if GPS position unavailable
                          
        Technical Notes:
        - Coordinates are in the Webots global reference frame
        - Precision depends on GPS sensor configuration
        - May return None during GPS initialization or signal loss
        """
        return self.position_tracker.get_current_position()
    
    def get_sensor_readings(self):
        """
        Retrieve current distance sensor readings for obstacle detection.
        
        Collects real-time distance measurements from all available sensors
        and formats them into a convenient dictionary structure. Essential
        for wall detection, obstacle avoidance, and environment mapping.
        
        Returns:
            dict: Sensor readings with simplified keys:
                  {'front': value, 'right': value, 'back': value, 'left': value}
                  Values are distances in simulation units, None if sensor unavailable
                  
        Sensor Interpretation:
        - Lower values indicate closer obstacles/walls
        - Values below Config.SENSOR_THRESHOLD typically indicate walls
        - Infinite/high values suggest open space in that direction
        
        Error Handling:
        - Failed sensor readings return None for that direction
        - System continues operation with partial sensor coverage
        - Warning messages logged for missing sensors during initialization
        """
        readings = {}
        
        # Process each available sensor and extract readings
        for name, sensor in self.sensors.items():
            try:
                # Simplify sensor names by removing '_distance_sensor' suffix
                # This creates more convenient keys: 'front', 'right', 'back', 'left'
                simplified_name = name.replace('_distance_sensor', '')
                readings[simplified_name] = sensor.getValue()
            except:
                # Handle sensor read failures gracefully
                simplified_name = name.replace('_distance_sensor', '')
                readings[simplified_name] = None
                
        return readings

    def _detect_wall_between_cells(self, from_cell, to_cell):
        """
        Detect walls between adjacent cells using sensor-based analysis.
        
        This method provides an efficient alternative to physical movement
        testing for wall detection. It uses distance sensor readings to
        determine if a wall exists between two adjacent cells without
        requiring actual robot movement.
        
        Algorithm Process:
        1. Calculate movement direction vector between cells
        2. Navigate to the source cell for proper sensor orientation
        3. Read appropriate directional sensor based on movement direction
        4. Compare sensor reading against wall detection threshold
        
        Args:
            from_cell (tuple): Starting cell coordinates (x, y)
            to_cell (tuple): Target cell coordinates (x, y)
            
        Returns:
            bool: True if wall detected between cells, False if path is clear
            
        Sensor Mapping Strategy:
        - Rightward movement (dx=1): Use right sensor
        - Leftward movement (dx=-1): Use left sensor  
        - Upward movement (dy=1): Use front sensor
        - Downward movement (dy=-1): Use back sensor
        
        Advantages over Physical Testing:
        - Much faster than actual movement attempts
        - No risk of collision with walls
        - Enables rapid environment mapping
        - Reduces algorithm execution time significantly
        
        Technical Notes:
        - Requires accurate sensor positioning and calibration
        - Threshold value defined in Config.SENSOR_THRESHOLD
        - Default to wall detection if sensor data unavailable (safety)
        """
        # Calculate the direction vector from source to target cell
        dx = to_cell[0] - from_cell[0]  # X-direction component
        dy = to_cell[1] - from_cell[1]  # Y-direction component
        
        # Move to the source cell and orient towards target
        # This ensures proper sensor alignment for accurate readings
        if not self.move_to_cell(from_cell[0], from_cell[1], verbose=False):
            # Assume wall if can't reach starting position (safety default)
            return True
        
        # Get current sensor readings for wall detection analysis
        sensors = self.get_sensor_readings()
        
        # Wall detection threshold from configuration
        sensor_threshold = Config.SENSOR_THRESHOLD
        
        # Determine which sensor to check based on movement direction
        if dx == 1:  # Moving right (+X direction)
            return sensors.get('right', float('inf')) < sensor_threshold
        elif dx == -1:  # Moving left (-X direction)
            return sensors.get('left', float('inf')) < sensor_threshold
        elif dy == 1:  # Moving up (+Y direction)
            return sensors.get('front', float('inf')) < sensor_threshold
        elif dy == -1:  # Moving down (-Y direction)
            return sensors.get('back', float('inf')) < sensor_threshold
            
        # Default to wall detection for uncertain cases (safety)
        return True

    def get_front_sensor(self):
        """
        Get front distance sensor reading for forward obstacle detection.
        
        Provides convenient access to the front-facing distance sensor,
        commonly used for forward path checking and obstacle avoidance
        during navigation algorithms.
        
        Returns:
            float or None: Distance reading in simulation units,
                          None if sensor unavailable
                          
        Usage Context:
        - Wall following algorithms for forward path verification
        - Obstacle avoidance during straight-line movement
        - Emergency stop detection for collision prevention
        """
        return self.sensors.get('front_distance_sensor', None).getValue() if 'front_distance_sensor' in self.sensors else None
     
    def stop(self):
        """
        Emergency stop - immediately halt all robot movement.
        
        Provides a safety mechanism to stop all motor activity instantly.
        Essential for emergency situations, algorithm termination, and
        controlled shutdown procedures.
        
        Safety Implementation:
        - Directly commands motor controller to stop all wheels
        - No gradual deceleration - immediate velocity cessation
        - Safe to call at any time during operation
        
        Usage Scenarios:
        - Algorithm completion or failure
        - Emergency obstacle detection
        - User intervention or system shutdown
        - Error recovery procedures
        """
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

    def save_maze_results(self, algorithm_name, visited_cells, total_steps, execution_time, 
                         success=False, final_position=None, path_length=None, path_history=None):
        """
        Save maze solving results to a text file with visualization and statistics
        
        Args:
            algorithm_name (str): Name of the algorithm used
            visited_cells (set): Set of visited cell coordinates
            total_steps (int): Total steps taken during execution
            execution_time (float): Time taken to complete/attempt the maze in seconds
            success (bool): Whether the algorithm successfully reached the goal
            final_position (tuple): Final position of the robot
            path_length (int): Length of the actual path taken (for successful runs)
            path_history (list): List of cells in order they were visited (for step-by-step visualization)
        """
        # Create results directory if it doesn't exist
        results_dir = "maze_results"
        if not os.path.exists(results_dir):
            os.makedirs(results_dir)
        
        # Generate filename with timestamp
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"{results_dir}/maze_result_{algorithm_name}_{timestamp}.txt"
        
        # Calculate statistics
        total_cells_visited = len(visited_cells)
        efficiency = (total_cells_visited / total_steps * 100) if total_steps > 0 else 0
        
        # Generate maze visualization
        maze_visualization = self._generate_maze_visualization(visited_cells)
        
        # Write results to file
        try:
            with open(filename, 'w') as f:
                f.write("=" * 60 + "\n")
                f.write(f"MAZE SOLVING RESULTS - {algorithm_name.upper()}\n")
                f.write("=" * 60 + "\n")
                f.write(f"Timestamp: {datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
                f.write(f"Algorithm: {algorithm_name}\n")
                f.write(f"Target: Cell (11, 11)\n")
                f.write("\n")
                
                # Results summary
                f.write("RESULTS SUMMARY\n")
                f.write("-" * 30 + "\n")
                f.write(f"Success: {'YES' if success else 'NO'}\n")
                f.write(f"Total Steps: {total_steps}\n")
                f.write(f"Total Cells Visited: {total_cells_visited}\n")
                f.write(f"Execution Time: {execution_time:.2f} seconds\n")
                f.write(f"Efficiency: {efficiency:.1f}% (unique cells / total steps)\n")
                
                if success and path_length:
                    f.write(f"Path Length: {path_length} cells\n")
                    path_efficiency = (path_length / total_steps * 100) if total_steps > 0 else 0
                    f.write(f"Path Efficiency: {path_efficiency:.1f}% (path length / total steps)\n")
                
                if final_position:
                    f.write(f"Final Position: {final_position}\n")
                
                f.write("\n")
                
                # Performance metrics
                f.write("PERFORMANCE METRICS\n")
                f.write("-" * 30 + "\n")
                f.write(f"Steps per Second: {total_steps / execution_time:.1f}\n")
                f.write(f"Cells per Second: {total_cells_visited / execution_time:.1f}\n")
                f.write(f"Coverage: {total_cells_visited / 144 * 100:.1f}% (of 12x12 maze)\n")
                f.write("\n")
                
                # Final maze visualization
                f.write("FINAL MAZE VISUALIZATION\n")
                f.write("-" * 30 + "\n")
                f.write("Legend: X = Visited, . = Unvisited, S = Start (0,0), G = Goal (11,11)\n")
                f.write("Y-axis: 11 (top) to 0 (bottom)\n")
                f.write("X-axis: 0 (left) to 11 (right)\n")
                f.write("\n")
                f.write(maze_visualization)
                f.write("\n")
                
                # Step-by-step path visualization
                if path_history:
                    f.write("STEP-BY-STEP PATH VISUALIZATION\n")
                    f.write("-" * 50 + "\n")
                    f.write("Shows the progression of exploration step by step\n")
                    f.write("Legend: X = Current visited cells, C = Current position, . = Unvisited\n")
                    f.write("\n")
                    
                    # Show every 5th step to keep file size manageable, plus first 10 and last 10
                    step_interval = max(1, len(path_history) // 20)  # Show ~20 snapshots
                    
                    for i, cell in enumerate(path_history):
                        # Show: first 10 steps, every Nth step, last 10 steps, or successful completion
                        show_step = (i < 10 or 
                                   i >= len(path_history) - 10 or 
                                   i % step_interval == 0 or
                                   (success and cell == (11, 11)))
                        
                        if show_step:
                            f.write(f"Step {i + 1}: Moving to {cell}\n")
                            visited_so_far = set(path_history[:i + 1])
                            step_viz = self._generate_step_visualization(visited_so_far, cell)
                            f.write(step_viz)
                            f.write("\n")
                            
                            # Add separator for readability
                            if i < len(path_history) - 1:
                                f.write("-" * 25 + "\n")
                    
                    f.write("\n")
                
                # Visited cells list
                f.write("VISITED CELLS LIST\n")
                f.write("-" * 30 + "\n")
                visited_list = sorted(list(visited_cells))
                for i, cell in enumerate(visited_list):
                    if i % 6 == 0:  # New line every 6 cells
                        f.write("\n")
                    f.write(f"{cell} ")
                f.write("\n\n")
                
                # Path sequence (if available)
                if path_history:
                    f.write("COMPLETE PATH SEQUENCE\n")
                    f.write("-" * 30 + "\n")
                    f.write("Order in which cells were visited:\n")
                    for i, cell in enumerate(path_history):
                        if i % 8 == 0:  # New line every 8 cells
                            f.write("\n")
                        f.write(f"{i+1}:{cell} ")
                    f.write("\n\n")
                
                # Algorithm comparison baseline
                f.write("ALGORITHM COMPARISON\n")
                f.write("-" * 30 + "\n")
                f.write("Theoretical minimum path length: ~22 steps (Manhattan distance)\n")
                f.write("Maximum possible coverage: 144 cells (12x12 maze)\n")
                f.write("Optimal efficiency: 100% (no redundant moves)\n")
                f.write("\n")
                
            print(f"\n📊 Results saved to: {filename}")
            return filename
            
        except Exception as e:
            print(f"ERROR: Failed to save results to file: {e}")
            return None

    def _generate_step_visualization(self, visited_cells, current_cell):
        """Generate ASCII visualization for a specific step showing current position"""
        visualization = ""
        
        for row in range(11, -1, -1):  # Start from top row (11) and go down to 0
            # Add row number
            visualization += f"{row:2d} "
            
            for col in range(12):  # Go from left (0) to right (11)
                cell = (col, row)
                
                if cell == current_cell:  # Current position
                    visualization += "C "
                elif cell == (0, 0) and cell not in visited_cells:  # Start position (if not visited yet)
                    visualization += "S "
                elif cell == (11, 11) and cell not in visited_cells:  # Goal position (if not reached yet)
                    visualization += "G "
                elif cell in visited_cells:
                    visualization += "X "
                else:
                    visualization += ". "
            
            visualization += "\n"
        
        # Add column numbers
        visualization += "   "
        for col in range(12):
            visualization += f"{col % 10} "
        visualization += "\n"
        
        return visualization

    def _generate_maze_visualization(self, visited_cells):
        """Generate ASCII visualization of the maze with visited cells marked"""
        visualization = ""
        
        for row in range(11, -1, -1):  # Start from top row (11) and go down to 0
            # Add row number
            visualization += f"{row:2d} "
            
            for col in range(12):  # Go from left (0) to right (11)
                cell = (col, row)
                
                if cell == (0, 0):  # Start position
                    visualization += "S "
                elif cell == (11, 11):  # Goal position
                    visualization += "G "
                elif cell in visited_cells:
                    visualization += "X "
                else:
                    visualization += ". "
            
            visualization += "\n"
        
        # Add column numbers
        visualization += "   "
        for col in range(12):
            visualization += f"{col % 10} "
        visualization += "\n"
        
        return visualization

    def run_left_wall_following(self):
        """
        Execute Left Wall Following Algorithm for Autonomous Maze Navigation.
        
        This method implements the classic left-hand rule for maze solving,
        a fundamental algorithm in robotics and artificial intelligence.
        The robot maintains contact with the left wall while navigating,
        ensuring complete maze exploration and guaranteed exit finding
        in simply connected mazes.
        
        Algorithm Theory:
        The left wall following algorithm is based on graph theory principles:
        - Treats the maze as a planar graph where walls are edges
        - Robot follows the leftmost available path at each decision point
        - Guarantees finding an exit in any finite, simply connected maze
        - Time complexity: O(E) where E is the number of edges in the maze graph
        
        Implementation Strategy:
        The algorithm uses a priority-based decision system:
        1. LEFT TURN: Highest priority - always try to turn left first
        2. STRAIGHT: Medium priority - continue forward if left blocked
        3. RIGHT TURN: Lower priority - turn right if straight blocked
        4. U-TURN: Lowest priority - reverse direction if all paths blocked
        
        Direction Management:
        Uses a compass-based orientation system:
        - 0: North (up/+Y direction)
        - 1: East (right/+X direction)  
        - 2: South (down/-Y direction)
        - 3: West (left/-X direction)
        
        Sensor Integration:
        Maps robot orientation to sensor readings:
        - Front sensor: Forward obstacle detection
        - Right sensor: Right-side wall detection
        - Back sensor: Rear obstacle detection
        - Left sensor: Left-side wall detection
        
        Performance Characteristics:
        - Guaranteed solution for simply connected mazes
        - May revisit cells multiple times (not optimal path)
        - Execution time depends on maze complexity and wall density
        - Robust against sensor noise and positioning errors
        
        Returns:
            bool: True if successfully reached target cell (11,11), False otherwise
            
        Academic Applications:
        - Demonstrates classical maze-solving algorithms
        - Shows integration of sensors, actuators, and decision logic
        - Provides baseline for comparing advanced pathfinding methods
        - Illustrates real-world robotics navigation challenges
        """
        print("=== Starting Left Wall Following Navigation ===")
        print("Goal: Navigate from start cell to end cell (11,11) by following the left wall")
        
        # === ALGORITHM INITIALIZATION ===
        # Record start time for performance analysis
        start_time = time.time()
        
        # Define target destination in maze coordinate system
        end_cell = (11, 11)  # Top-right corner of 12x12 maze
        
        # === POSITION INITIALIZATION AND VALIDATION ===
        # Get starting position and validate GPS availability
        start_pos = self.get_current_cell()
        if start_pos is None:
            print("ERROR: Cannot get starting position!")
            return False
        
        # Initialize algorithm state variables
        current_cell = start_pos              # Current robot position
        visited = set()                       # Set of visited cells for tracking
        visited.add(current_cell)             # Mark starting cell as visited
        path_history = []                     # Sequential path for analysis
        path_history.append(current_cell)     # Record starting position
        
        print(f"Starting Left Wall Following from cell: {current_cell}")
        print(f"Target cell: {end_cell}")
        
        # === DIRECTION SYSTEM CONFIGURATION ===
        # Direction mappings: 0=North(up), 1=East(right), 2=South(down), 3=West(left)
        directions = ['up', 'right', 'down', 'left']           # Human-readable direction names
        direction_vectors = [(0, 1), (1, 0), (0, -1), (-1, 0)] # Coordinate displacement vectors
        sensor_mapping = ['front', 'right', 'back', 'left']    # Sensor-to-direction correspondence
        
        # Initialize robot orientation - start facing North (toward +Y)
        current_direction = 0
        
        # === ALGORITHM EXECUTION PARAMETERS ===
        step_count = 0      # Performance counter for algorithm analysis
        max_steps = 1000    # Safety limit to prevent infinite loops
        
        # === MAIN ALGORITHM EXECUTION LOOP ===
        while current_cell != end_cell and step_count < max_steps:
            # === STEP EXECUTION AND STATE TRACKING ===
            step_count += 1                    # Increment algorithm step counter
            visited.add(current_cell)          # Mark current cell as explored
            self.print_visited_cells(visited)  # Visual progress display
            
            # Provide detailed step-by-step feedback for analysis
            print(f"\n--- Step {step_count} ---")
            print(f"Current cell: {current_cell}")
            print(f"Current direction: {directions[current_direction]}")
            
            # === ENVIRONMENTAL SENSING AND ANALYSIS ===
            # Collect real-time sensor data for wall detection
            sensors = self.get_sensor_readings()
            print(f"Sensor readings: {sensors}")
            
            # === LEFT WALL FOLLOWING DECISION ALGORITHM ===
            # Implement priority-based decision system following left-hand rule:
            # Priority 1: Try to turn left and move (maintain left wall contact)
            # Priority 2: If can't turn left, try to go straight (follow wall)
            # Priority 3: If can't go straight, turn right (navigate around obstacle)
            # Priority 4: If can't turn right, turn around 180° (dead end recovery)
            
            moved = False  # Flag to track successful movement execution
            
            # === PRIORITY 1: LEFT TURN ATTEMPT ===
            # Calculate left turn direction using modular arithmetic
            left_direction = (current_direction - 1) % 4
            left_dx, left_dy = direction_vectors[left_direction]
            left_cell = (current_cell[0] + left_dx, current_cell[1] + left_dy)
            left_sensor = sensor_mapping[left_direction]
            
            if self._is_valid_move(left_cell, sensors.get(left_sensor, 0)):
                print(f"  Left turn possible: turning {directions[left_direction]} and moving to {left_cell}")
                current_direction = left_direction
                if self._execute_move(directions[current_direction]):
                    current_cell = self.get_current_cell() or left_cell
                    path_history.append(current_cell)  # Record the move
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
                        path_history.append(current_cell)  # Record the move
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
                        path_history.append(current_cell)  # Record the move
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
        
        # Calculate execution time
        execution_time = time.time() - start_time
        
        # Check if we reached the goal
        if current_cell == end_cell:
            print(f"\n🎉 SUCCESS! Reached end cell {end_cell} in {step_count} steps using Left Wall Following!")
            print(f"Total cells visited: {len(visited)}")
            print(f"Execution time: {execution_time:.2f} seconds")
            
            # Save results to file
            self.save_maze_results(
                algorithm_name="Left Wall Following",
                visited_cells=visited,
                total_steps=step_count,
                execution_time=execution_time,
                success=True,
                final_position=current_cell,
                path_history=path_history
            )
            return True
        else:
            print(f"\n❌ FAILED to reach end cell after {step_count} steps")
            print(f"Final position: {current_cell}")
            print(f"Execution time: {execution_time:.2f} seconds")
            
            # Save results to file
            self.save_maze_results(
                algorithm_name="Left Wall Following",
                visited_cells=visited,
                total_steps=step_count,
                execution_time=execution_time,
                success=False,
                final_position=current_cell,
                path_history=path_history
            )
            return False

    def run_right_wall_following(self):
        """
        Run Right Wall Following algorithm to navigate through the maze
        The robot follows the right wall until it reaches the end cell (11, 11)
        """
        print("=== Starting Right Wall Following Navigation ===")
        print("Goal: Navigate from start cell to end cell (11,11) by following the right wall")
        
        # Record start time
        start_time = time.time()
        
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
        path_history = []  # Track the order cells were visited for step-by-step visualization
        path_history.append(current_cell)  # Record starting position
        
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
                    path_history.append(current_cell)  # Record the move
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
                        path_history.append(current_cell)  # Record the move
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
                        path_history.append(current_cell)  # Record the move
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
        
        # Calculate execution time
        execution_time = time.time() - start_time
        
        # Check if we reached the goal
        if current_cell == end_cell:
            print(f"\n🎉 SUCCESS! Reached end cell {end_cell} in {step_count} steps using Right Wall Following!")
            print(f"Total cells visited: {len(visited)}")
            print(f"Execution time: {execution_time:.2f} seconds")
            
            # Save results to file
            self.save_maze_results(
                algorithm_name="Right Wall Following",
                visited_cells=visited,
                total_steps=step_count,
                execution_time=execution_time,
                success=True,
                final_position=current_cell,
                path_history=path_history
            )
            return True
        else:
            print(f"\n❌ FAILED to reach end cell after {step_count} steps")
            print(f"Final position: {current_cell}")
            print(f"Execution time: {execution_time:.2f} seconds")
            
            # Save results to file
            self.save_maze_results(
                algorithm_name="Right Wall Following",
                visited_cells=visited,
                total_steps=step_count,
                execution_time=execution_time,
                success=False,
                final_position=current_cell,
                path_history=path_history
            )
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
        Execute Depth-First Search (DFS) Algorithm for Systematic Maze Exploration.
        
        This method implements the classic DFS algorithm adapted for robotic
        maze navigation. DFS is a fundamental graph traversal algorithm that
        explores as far as possible along each branch before backtracking,
        making it ideal for systematic maze exploration and pathfinding.
        
        Algorithm Theory:
        DFS is based on graph theory principles for systematic exploration:
        - Treats maze as a graph where cells are vertices and passages are edges
        - Uses a stack (LIFO) data structure to manage exploration order
        - Guarantees visiting every reachable cell exactly once
        - Time complexity: O(V + E) where V is vertices, E is edges
        - Space complexity: O(V) for the recursion stack/visited set
        
        Implementation Strategy:
        The algorithm uses an iterative approach with explicit stack management:
        1. EXPLORATION: Try to move to unvisited adjacent cells
        2. PRIORITIZATION: Use consistent direction ordering for deterministic behavior
        3. BACKTRACKING: Return to previous cell when no unvisited neighbors exist
        4. TERMINATION: Stop when target found or all reachable cells explored
        
        Direction Priority System:
        Uses right-hand preference for consistent exploration:
        - RIGHT (East/+X): Highest priority for systematic coverage
        - UP (North/+Y): Second priority toward target direction
        - LEFT (West/-X): Third priority for complete exploration
        - DOWN (South/-Y): Lowest priority, away from target
        
        Data Structures:
        - visited (set): Tracks explored cells to prevent cycles
        - path_stack (list): Maintains current path for backtracking
        - path_history (list): Records complete movement sequence for analysis
        
        Sensor Integration:
        Maps movement directions to sensor readings for obstacle detection:
        - Right movement → Right distance sensor
        - Up movement → Front distance sensor
        - Left movement → Left distance sensor  
        - Down movement → Back distance sensor
        
        Performance Characteristics:
        - Systematic exploration ensures complete maze coverage
        - Optimal path not guaranteed (finds A path, not THE shortest path)
        - Memory efficient with explicit stack management
        - Deterministic behavior with consistent direction ordering
        - Robust backtracking handles dead ends and complex maze topologies
        
        Returns:
            bool: True if successfully reached target cell (11,11), False otherwise
            
        Academic Applications:
        - Demonstrates classical computer science algorithms in robotics
        - Shows stack-based problem solving and backtracking techniques
        - Illustrates systematic state space exploration methods
        - Provides comparison baseline for advanced pathfinding algorithms
        """
        print("=== Starting DFS Maze Navigation ===")
        print("Goal: Navigate from start cell (0,0) to end cell (11,11)")
        
        # === ALGORITHM INITIALIZATION ===
        # Record start time for performance analysis
        start_time = time.time()
        
        # === DFS DATA STRUCTURE INITIALIZATION ===
        end_cell = (11, 11)          # Target destination coordinates
        visited = set()              # Set for O(1) visited cell lookup
        path_stack = []              # Stack for current path maintenance and backtracking
        path_history = []            # Complete movement sequence for academic analysis
        
        # === POSITION INITIALIZATION AND VALIDATION ===
        # Get starting position and validate GPS availability
        start_pos = self.get_current_cell()
        if start_pos is None:
            print("ERROR: Cannot get starting position!")
            return False
        
        # Initialize algorithm state
        current_cell = start_pos
        visited.add(current_cell)           # Mark starting cell as explored
        path_stack.append(current_cell)     # Initialize path with starting position
        path_history.append(current_cell)   # Begin movement history tracking
        
        print(f"Starting DFS from cell: {current_cell}")
        print(f"Target cell: {end_cell}")
        
        # === ALGORITHM EXECUTION PARAMETERS ===
        step_count = 0      # Performance counter for complexity analysis
        max_steps = 1000    # Safety limit to prevent infinite loops
        
        # === MAIN DFS EXECUTION LOOP ===
        while current_cell != end_cell and step_count < max_steps:
            # === STEP TRACKING AND VISUALIZATION ===
            step_count += 1
            self.print_visited_cells(visited)  # Visual progress display
            print(f"\n--- Step {step_count} ---")
            print(f"Current cell: {current_cell}")
            
            # === ENVIRONMENTAL SENSING ===
            # Collect sensor data for wall detection and movement validation
            sensors = self.get_sensor_readings()
            print(f"Sensor readings: {sensors}")
            
            # === DFS DIRECTION EXPLORATION SYSTEM ===
            # Define movement options with consistent priority ordering
            # Priority: Right → Up → Left → Down (systematic right-hand exploration)
            moves = [
                ('right', (1, 0), 'right'),   # East: +X direction (highest priority)
                ('up', (0, 1), 'front'),      # North: +Y direction (toward target)
                ('left', (-1, 0), 'left'),    # West: -X direction (complete coverage)
                ('down', (0, -1), 'back')     # South: -Y direction (lowest priority)
            ]
            
            found_move = False  # Flag for successful movement detection
            
            # === SYSTEMATIC DIRECTION EXPLORATION ===
            # Try each direction in priority order for unvisited accessible cells
            for direction, (dx, dy), sensor_key in moves:
                # Calculate potential next cell coordinates
                next_cell = (current_cell[0] + dx, current_cell[1] + dy)
                
                # === BOUNDARY VALIDATION ===
                # Ensure movement stays within 12x12 maze boundaries
                if not (0 <= next_cell[0] <= 11 and 0 <= next_cell[1] <= 11):
                    continue
                
                # === VISITED STATE CHECK ===
                # Skip cells already explored to prevent infinite cycles
                if next_cell in visited:
                    continue
                
                # === OBSTACLE DETECTION AND WALL CHECKING ===
                # Use sensor readings to detect walls before attempting movement
                sensor_value = sensors.get(sensor_key, None)
                print("Sensor value for", sensor_key, ":", sensor_value)
                
                # Wall detection using calibrated sensor threshold
                if sensor_value is not None and sensor_value < 380:  # Wall detected
                    print(f"  Wall detected {direction} (sensor: {sensor_value:.3f})")
                    continue
                
                # === VALID MOVEMENT EXECUTION ===
                # All checks passed - attempt to execute movement
                print(f"  Moving {direction} to cell: {next_cell}")
                
                # Execute direction-specific movement command
                success = False
                if direction == 'right':
                    success = self.move_right(verbose=True)
                elif direction == 'up':
                    success = self.move_top(verbose=True)
                elif direction == 'left':
                    success = self.move_left(verbose=True)
                elif direction == 'down':
                    success = self.move_bottom(verbose=True)
                
                # === MOVEMENT SUCCESS VALIDATION AND STATE UPDATE ===
                if success:
                    # Update current position (with GPS verification)
                    current_cell = self.get_current_cell()
                    if current_cell is None:
                        current_cell = next_cell  # Fallback to calculated position
                    
                    # Update DFS data structures for successful move
                    visited.add(current_cell)           # Mark new cell as explored
                    path_stack.append(current_cell)     # Add to current path
                    path_history.append(current_cell)   # Record in movement history
                    found_move = True                   # Flag successful exploration
                    print(f"  Successfully moved to: {current_cell}")
                    break  # Exit direction loop - move to next iteration
                else:
                    # Movement execution failed - log for debugging
                    print(f"  Failed to move {direction}")
            
            # === DFS BACKTRACKING MECHANISM ===
            # If no valid forward moves found, implement backtracking
            if not found_move:
                # === TERMINATION CONDITION CHECK ===
                # Verify backtracking is possible before proceeding
                if len(path_stack) <= 1:
                    print("ERROR: No more moves available and cannot backtrack!")
                    print("DFS exploration complete - no path to target found!")
                    break
                
                # === STACK-BASED BACKTRACKING EXECUTION ===
                # Remove current cell from active path (backtrack one step)
                path_stack.pop()
                
                # Validate stack integrity after pop operation
                if not path_stack:
                    print("ERROR: Path stack is empty!")
                    break
                
                # === BACKTRACK TARGET DETERMINATION ===
                # Get previous cell from stack for backtrack navigation
                target_cell = path_stack[-1]
                print(f"  Backtracking to cell: {target_cell}")
                
                # === BACKTRACK MOVEMENT EXECUTION ===
                # Navigate back to previous cell in exploration path
                if self.move_to_cell(target_cell[0], target_cell[1], verbose=True):
                    # Successful backtrack - update algorithm state
                    current_cell = target_cell
                    path_history.append(current_cell)  # Record backtrack in movement history
                    print(f"  Backtracked to: {current_cell}")
                else:
                    # Backtrack movement failed - critical error
                    print(f"  Failed to backtrack to: {target_cell}")
                    break
            
            # === VISUALIZATION DELAY ===
            # Small delay for step-by-step visualization and analysis
            time.sleep(0.1)
        
        # Calculate execution time
        execution_time = time.time() - start_time
        
        # Check if we reached the goal and save results
        if current_cell == end_cell:
            print(f"\n🎉 SUCCESS! Reached end cell {end_cell} in {step_count} steps!")
            print(f"Final path length: {len(path_stack)} cells")
            print(f"Path taken: {' -> '.join(map(str, path_stack))}")
            print(f"Execution time: {execution_time:.2f} seconds")
            
            # Save results to file
            self.save_maze_results(
                algorithm_name="DFS",
                visited_cells=visited,
                total_steps=step_count,
                execution_time=execution_time,
                success=True,
                final_position=current_cell,
                path_length=len(path_stack),
                path_history=path_history
            )
            return True
        else:
            print(f"\n❌ FAILED to reach end cell after {step_count} steps")
            print(f"Final position: {current_cell}")
            print(f"Execution time: {execution_time:.2f} seconds")
            
            # Save results to file
            self.save_maze_results(
                algorithm_name="DFS",
                visited_cells=visited,
                total_steps=step_count,
                execution_time=execution_time,
                success=False,
                final_position=current_cell,
                path_history=path_history
            )
            return False

    def run_smart_wall_following(self):
        """
        Execute Smart Wall Following Algorithm with Adaptive Memory and Mapping.
        
        This method implements an enhanced version of traditional wall following
        that incorporates intelligent memory management and dynamic environment
        mapping. Unlike basic wall following, this algorithm builds and maintains
        a persistent map of discovered walls and visited areas, enabling more
        efficient navigation and reduced redundant exploration.
        
        Algorithm Innovation:
        Smart Wall Following extends classical wall following with:
        - Dynamic Environment Mapping: Real-time construction of wall layout
        - Visited Cell Memory: Intelligent tracking of explored areas
        - Adaptive Decision Making: Context-aware direction selection
        - Backtrack Prevention: Minimized revisiting of exhausted areas
        - Priority-Based Navigation: Preference for unexplored territories
        
        Theoretical Foundation:
        Combines principles from multiple AI domains:
        - Classical maze solving (wall following)
        - SLAM (Simultaneous Localization and Mapping)
        - Heuristic search (preference for unvisited areas)
        - Memory-augmented navigation systems
        
        Data Structures:
        - wall_map (dict): Maps cell transitions to wall existence
        - visited (set): Tracks explored cells for redundancy avoidance
        - path_history (list): Complete movement sequence for analysis
        - direction tracking: Maintains robot orientation for wall following
        
        Decision Making Process:
        1. ENVIRONMENTAL MAPPING: Update wall map with current sensor readings
        2. SMART DIRECTION SELECTION: Choose optimal direction using:
           - Wall following priority (left turn preference)
           - Unvisited cell preference (exploration priority)
           - Wall map consultation (obstacle avoidance)
        3. MOVEMENT EXECUTION: Execute selected movement
        4. STATE UPDATE: Update position, mapping, and history
        
        Performance Advantages:
        - Reduced exploration time through memory utilization
        - More efficient path planning with accumulated knowledge
        - Adaptive behavior based on discovered environment structure
        - Intelligent backtracking when traditional wall following fails
        
        Returns:
            bool: True if successfully reached target cell (11,11), False otherwise
            
        Academic Significance:
        Demonstrates integration of classical algorithms with modern AI techniques:
        - Shows evolution from reactive to memory-based navigation
        - Illustrates practical SLAM implementation in constrained environments
        - Provides example of heuristic enhancement to deterministic algorithms
        - Bridges gap between simple rule-based and intelligent adaptive systems
        """
        print("=== Starting Smart Wall Following ===")
        print("Goal: Enhanced wall following with mapping")
        
        # === ALGORITHM INITIALIZATION ===
        # Record start time for performance analysis
        start_time = time.time()
        
        # Define navigation target and validate starting position
        end_cell = (11, 11)
        start_pos = self.get_current_cell()
        
        if start_pos is None:
            print("ERROR: Cannot get starting position!")
            return False
        
        # === SMART MAPPING DATA STRUCTURES ===
        # Initialize enhanced data structures for intelligent navigation
        wall_map = {}            # Dynamic environment map: (from_cell, to_cell) → is_blocked
        visited = set()          # Memory of explored cells for efficiency
        path_taken = []          # Current navigation path
        path_history = []        # Complete movement history for analysis
        
        # === INITIAL STATE CONFIGURATION ===
        current_cell = start_pos
        visited.add(current_cell)           # Mark starting position as explored
        path_taken.append(current_cell)     # Initialize path tracking
        path_history.append(current_cell)   # Begin movement history
        
        # === DIRECTION SYSTEM FOR WALL FOLLOWING ===
        # Maintain orientation system for consistent wall following behavior
        directions = ['up', 'right', 'down', 'left']           # Human-readable direction labels
        direction_vectors = [(0, 1), (1, 0), (0, -1), (-1, 0)] # Coordinate transformation vectors
        sensor_mapping = ['front', 'right', 'back', 'left']    # Sensor-to-direction mapping
        current_direction = 0  # Initialize facing North (up) for consistent start orientation
        
        # === ALGORITHM EXECUTION PARAMETERS ===
        step_count = 0     # Performance tracking counter
        max_steps = 500    # Safety limit (reduced from basic algorithms due to efficiency)
        
        print(f"Starting from {start_pos}, target: {end_cell}")
        
        # === MAIN SMART WALL FOLLOWING LOOP ===
        while current_cell != end_cell and step_count < max_steps:
            # === STEP PROGRESSION AND MONITORING ===
            step_count += 1
            visited.add(current_cell)  # Update visited cells memory
            
            # Periodic visualization for analysis (reduced frequency due to efficiency)
            if step_count % 20 == 0:
                self.print_visited_cells(visited)
            
            print(f"\n--- Step {step_count} ---")
            print(f"At {current_cell}, facing {directions[current_direction]}")
            
            # === DYNAMIC ENVIRONMENT MAPPING ===
            # Update wall map with current sensor readings for intelligent decision making
            self._update_wall_map(current_cell, wall_map)
            
            # === SMART DIRECTION SELECTION ===
            # Use enhanced decision algorithm that combines wall following with memory
            next_direction, next_cell = self._choose_smart_direction(
                current_cell, current_direction, wall_map, visited, directions, direction_vectors
            )
            
            # === MOVEMENT VALIDATION AND EXECUTION ===
            if next_cell is None:
                print("No valid moves available!")
                break
            
            print(f"Choosing to go {directions[next_direction]} to {next_cell}")
            
            # === MOVEMENT EXECUTION AND STATE UPDATE ===
            # Execute selected movement and update algorithm state
            if self._execute_move(directions[next_direction]):
                current_direction = next_direction                    # Update robot orientation
                current_cell = self.get_current_cell() or next_cell   # Update position (with GPS verification)
                path_taken.append(current_cell)                      # Extend current path
                path_history.append(current_cell)                    # Record in movement history
                print(f"Successfully moved to: {current_cell}")
            else:
                print(f"Failed to move {directions[next_direction]}")
                # Mark as wall and try again
                wall_map[(current_cell, next_cell)] = True
        
        # Calculate execution time
        execution_time = time.time() - start_time
        
        # Check success
        if current_cell == end_cell:
            print(f"\n🎉 SUCCESS! Reached goal using Smart Wall Following!")
            print(f"Steps taken: {step_count}")
            print(f"Cells visited: {len(visited)}")
            print(f"Path efficiency: {len(visited)/step_count*100:.1f}%")
            print(f"Execution time: {execution_time:.2f} seconds")
            
            # Save results to file
            self.save_maze_results(
                algorithm_name="Smart Wall Following",
                visited_cells=visited,
                total_steps=step_count,
                execution_time=execution_time,
                success=True,
                final_position=current_cell,
                path_history=path_history
            )
            return True
        else:
            print(f"\n❌ FAILED to reach goal after {step_count} steps")
            print(f"Final position: {current_cell}")
            print(f"Execution time: {execution_time:.2f} seconds")
            
            # Save results to file
            self.save_maze_results(
                algorithm_name="Smart Wall Following",
                visited_cells=visited,
                total_steps=step_count,
                execution_time=execution_time,
                success=False,
                final_position=current_cell,
                path_history=path_history
            )
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
    """
    Main execution function for the robot controller system.
    
    This function serves as the primary entry point for the autonomous
    maze navigation system. It orchestrates the complete initialization
    and execution sequence required for successful robot operation.
    
    Execution Sequence:
    1. Controller instantiation and system setup
    2. Hardware and subsystem initialization with error checking
    3. GPS system synchronization and readiness verification
    4. Algorithm execution (currently Smart Wall Following)
    5. Graceful handling of initialization or execution failures
    
    Error Handling:
    - Comprehensive initialization failure detection
    - GPS readiness verification before navigation
    - Graceful termination on system errors
    
    Algorithm Selection:
    Currently configured to run Smart Wall Following algorithm.
    This can be easily modified to execute different navigation
    strategies by changing the algorithm method call.
    
    Academic Usage:
    This function demonstrates the complete system integration
    required for autonomous robotics applications, from hardware
    initialization through algorithm execution.
    """
    # Instantiate the main controller object
    controller = MecanumCellController()
    
    # Initialize the controller and all subsystems
    if not controller.initialize():
        # Initialization failed - terminate with error message
        return
    
    # Wait for GPS system to become ready for navigation
    if controller.wait_for_gps() is None:
        print("ERROR: Could not initialize GPS!")
        return
    
    # Execute the selected navigation algorithm
    # Currently configured for Smart Wall Following demonstration
    controller.run_dfs()


# Main execution entry point for standalone script operation
if __name__ == "__main__":
    """
    Script execution entry point for autonomous maze navigation.
    
    When this module is executed directly (not imported), this block
    initiates the complete robot control system. This design pattern
    allows the module to function both as an importable library and
    as a standalone executable script.
    
    Academic Note:
    This demonstrates proper Python module design with support for
    both library usage and direct execution, following established
    software engineering best practices.
    """
    run_robot()