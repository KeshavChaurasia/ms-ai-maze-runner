"""
Navigation Algorithms for Maze Robot

This module contains all the pathfinding and navigation algorithms for autonomous
maze navigation. It provides a clean separation between the robot hardware
interface and the high-level navigation strategies.

Implemented Algorithms:
1. Depth-First Search (DFS) - Systematic exploration with backtracking
2. Left Wall Following - Classic left-hand maze navigation
3. Right Wall Following - Classic right-hand maze navigation  
4. Smart Wall Following - Enhanced wall following with mapping memory

Author: Keshav Chaurasia, Mark, Chris, David
Date: July 2025
Academic Project: Maze Navigation with Four-Wheel Robot
"""

from config import Config
from maze_visualizer import MazeVisualizer
import time


class NavigationAlgorithms:
    """
    Collection of maze navigation algorithms.
    
    This class provides various pathfinding strategies that can be used
    with any robot implementing the basic movement interface.
    """
    
    def __init__(self, robot, visualizer=None):
        """
        Initialize navigation algorithms.
        
        Args:
            robot: Robot instance with movement capabilities
            visualizer: Optional visualizer for progress tracking
        """
        self.robot = robot
        self.visualizer = visualizer or MazeVisualizer()
    
    def run_dfs(self, start_cell=None, end_cell=(11, 11)):
        """
        Run Depth-First Search algorithm.
        
        Args:
            start_cell (tuple): Starting cell coordinates, None for current position
            end_cell (tuple): Target cell coordinates
            
        Returns:
            bool: True if target reached, False otherwise
        """
        print("=== Starting Depth-First Search ===")
        print("Goal: Systematic exploration with backtracking")
        
        # Initialize starting position
        if start_cell is None:
            start_cell = self.robot.get_current_cell()
        
        if start_cell is None:
            print("ERROR: Cannot get starting position!")
            return False
        
        # DFS data structures
        visited = set()
        path_stack = [start_cell]
        path_history = []
        
        current_cell = start_cell
        visited.add(current_cell)
        path_history.append(current_cell)
        
        step_count = 0
        max_steps = 1000
        
        print(f"Starting from {start_cell}, target: {end_cell}")
        
        while current_cell != end_cell and step_count < max_steps:
            step_count += 1
            
            if step_count % 50 == 0:
                self.visualizer.print_progress(visited, current_cell, step_count)
            
            print(f"\n--- Step {step_count} ---")
            print(f"Current cell: {current_cell}")
            
            # Get sensor readings
            sensors = self.robot.get_sensor_readings()
            print(f"Sensor readings: {sensors}")
            
            # Define possible moves in DFS priority order
            moves = [
                ('right', (1, 0), 'right'),
                ('up', (0, 1), 'front'),
                ('left', (-1, 0), 'left'),
                ('down', (0, -1), 'back')
            ]
            
            found_move = False
            
            # Try each direction
            for direction, (dx, dy), sensor_key in moves:
                next_cell = (current_cell[0] + dx, current_cell[1] + dy)
                
                # Check bounds
                if not self._is_valid_cell(next_cell):
                    continue
                
                # Check if already visited
                if next_cell in visited:
                    continue
                
                # Check for wall
                if self._is_wall_detected(sensor_key, sensors):
                    continue
                
                # Valid move found - execute it
                print(f"  Moving {direction} to cell: {next_cell}")
                
                success = self._execute_directional_move(direction)
                
                if success:
                    current_cell = self.robot.get_current_cell() or next_cell
                    visited.add(current_cell)
                    path_stack.append(current_cell)
                    path_history.append(current_cell)
                    found_move = True
                    print(f"  Successfully moved to: {current_cell}")
                    break
                else:
                    print(f"  Failed to move {direction}")
            
            # Backtrack if no valid move found
            if not found_move:
                if len(path_stack) <= 1:
                    print("ERROR: No more moves available!")
                    break
                
                path_stack.pop()  # Remove current cell
                if not path_stack:
                    break
                
                target_cell = path_stack[-1]
                print(f"  Backtracking to cell: {target_cell}")
                
                if self.robot.move_to_cell(target_cell[0], target_cell[1], verbose=True):
                    current_cell = target_cell
                    path_history.append(current_cell)
                    print(f"  Backtracked to: {current_cell}")
                else:
                    print(f"  Failed to backtrack to: {target_cell}")
                    break
            
            time.sleep(0.1)  # Visualization delay
        
        # Results
        success = current_cell == end_cell
        result = {
            'algorithm': 'DFS',
            'success': success,
            'steps': step_count,
            'visited_count': len(visited),
            'path_length': len(path_stack),
            'final_position': current_cell
        }
        
        self.visualizer.print_results(result, visited, path_stack)
        return success
    
    def run_left_wall_following(self, start_cell=None, end_cell=(11, 11)):
        """
        Run Left Wall Following algorithm.
        
        Args:
            start_cell (tuple): Starting cell coordinates
            end_cell (tuple): Target cell coordinates
            
        Returns:
            bool: True if target reached, False otherwise
        """
        print("=== Starting Left Wall Following ===")
        print("Goal: Follow left wall to navigate maze")
        
        if start_cell is None:
            start_cell = self.robot.get_current_cell()
        
        if start_cell is None:
            print("ERROR: Cannot get starting position!")
            return False
        
        visited = set()
        current_cell = start_cell
        
        # Direction mappings: 0=North(up), 1=East(right), 2=South(down), 3=West(left)
        directions = ['up', 'right', 'down', 'left']
        direction_vectors = [(0, 1), (1, 0), (0, -1), (-1, 0)]
        sensor_mapping = ['front', 'right', 'back', 'left']
        
        current_direction = 0  # Start facing North
        step_count = 0
        max_steps = 1000
        
        while current_cell != end_cell and step_count < max_steps:
            step_count += 1
            visited.add(current_cell)
            
            if step_count % 20 == 0:
                self.visualizer.print_progress(visited, current_cell, step_count)
            
            print(f"\n--- Step {step_count} ---")
            print(f"Current cell: {current_cell}")
            print(f"Current direction: {directions[current_direction]}")
            
            sensors = self.robot.get_sensor_readings()
            moved = False
            
            # Left wall following algorithm:
            # 1. Try to turn left and move
            # 2. If can't turn left, try to go straight
            # 3. If can't go straight, turn right
            # 4. If can't turn right, turn around
            
            # Step 1: Try left turn
            left_direction = (current_direction - 1) % 4
            left_dx, left_dy = direction_vectors[left_direction]
            left_cell = (current_cell[0] + left_dx, current_cell[1] + left_dy)
            left_sensor = sensor_mapping[left_direction]
            
            if self._is_valid_move(left_cell, sensors.get(left_sensor, 0)):
                print(f"  Left turn: turning {directions[left_direction]} and moving to {left_cell}")
                current_direction = left_direction
                if self._execute_directional_move(directions[current_direction]):
                    current_cell = self.robot.get_current_cell() or left_cell
                    moved = True
                    print(f"  Successfully moved to: {current_cell}")
            
            # Step 2: Try straight
            if not moved:
                straight_dx, straight_dy = direction_vectors[current_direction]
                straight_cell = (current_cell[0] + straight_dx, current_cell[1] + straight_dy)
                straight_sensor = sensor_mapping[current_direction]
                
                if self._is_valid_move(straight_cell, sensors.get(straight_sensor, 0)):
                    print(f"  Going straight: moving {directions[current_direction]} to {straight_cell}")
                    if self._execute_directional_move(directions[current_direction]):
                        current_cell = self.robot.get_current_cell() or straight_cell
                        moved = True
                        print(f"  Successfully moved to: {current_cell}")
            
            # Step 3: Try right turn
            if not moved:
                right_direction = (current_direction + 1) % 4
                right_dx, right_dy = direction_vectors[right_direction]
                right_cell = (current_cell[0] + right_dx, current_cell[1] + right_dy)
                right_sensor = sensor_mapping[right_direction]
                
                if self._is_valid_move(right_cell, sensors.get(right_sensor, 0)):
                    print(f"  Right turn: turning {directions[right_direction]} and moving to {right_cell}")
                    current_direction = right_direction
                    if self._execute_directional_move(directions[current_direction]):
                        current_cell = self.robot.get_current_cell() or right_cell
                        moved = True
                        print(f"  Successfully moved to: {current_cell}")
                else:
                    print(f"  Turning right to face {directions[right_direction]} (no movement)")
                    current_direction = right_direction
                    moved = True
            
            # Step 4: Turn around if stuck
            if not moved:
                print("  Dead end: turning around 180 degrees")
                current_direction = (current_direction + 2) % 4
                print(f"  Now facing: {directions[current_direction]}")
                moved = True
            
            time.sleep(0.1)
        
        success = current_cell == end_cell
        result = {
            'algorithm': 'Left Wall Following',
            'success': success,
            'steps': step_count,
            'visited_count': len(visited),
            'final_position': current_cell
        }
        
        self.visualizer.print_results(result, visited)
        return success
    
    def run_right_wall_following(self, start_cell=None, end_cell=(11, 11)):
        """
        Run Right Wall Following algorithm.
        
        Args:
            start_cell (tuple): Starting cell coordinates
            end_cell (tuple): Target cell coordinates
            
        Returns:
            bool: True if target reached, False otherwise
        """
        print("=== Starting Right Wall Following ===")
        print("Goal: Follow right wall to navigate maze")
        
        if start_cell is None:
            start_cell = self.robot.get_current_cell()
        
        if start_cell is None:
            print("ERROR: Cannot get starting position!")
            return False
        
        visited = set()
        current_cell = start_cell
        
        directions = ['up', 'right', 'down', 'left']
        direction_vectors = [(0, 1), (1, 0), (0, -1), (-1, 0)]
        sensor_mapping = ['front', 'right', 'back', 'left']
        
        current_direction = 0  # Start facing North
        step_count = 0
        max_steps = 1000
        
        while current_cell != end_cell and step_count < max_steps:
            step_count += 1
            visited.add(current_cell)
            
            if step_count % 20 == 0:
                self.visualizer.print_progress(visited, current_cell, step_count)
            
            print(f"\n--- Step {step_count} ---")
            print(f"Current cell: {current_cell}")
            print(f"Current direction: {directions[current_direction]}")
            
            sensors = self.robot.get_sensor_readings()
            moved = False
            
            # Right wall following algorithm (mirror of left wall following)
            
            # Step 1: Try right turn
            right_direction = (current_direction + 1) % 4
            right_dx, right_dy = direction_vectors[right_direction]
            right_cell = (current_cell[0] + right_dx, current_cell[1] + right_dy)
            right_sensor = sensor_mapping[right_direction]
            
            if self._is_valid_move(right_cell, sensors.get(right_sensor, 0)):
                print(f"  Right turn: turning {directions[right_direction]} and moving to {right_cell}")
                current_direction = right_direction
                if self._execute_directional_move(directions[current_direction]):
                    current_cell = self.robot.get_current_cell() or right_cell
                    moved = True
                    print(f"  Successfully moved to: {current_cell}")
            
            # Step 2: Try straight
            if not moved:
                straight_dx, straight_dy = direction_vectors[current_direction]
                straight_cell = (current_cell[0] + straight_dx, current_cell[1] + straight_dy)
                straight_sensor = sensor_mapping[current_direction]
                
                if self._is_valid_move(straight_cell, sensors.get(straight_sensor, 0)):
                    print(f"  Going straight: moving {directions[current_direction]} to {straight_cell}")
                    if self._execute_directional_move(directions[current_direction]):
                        current_cell = self.robot.get_current_cell() or straight_cell
                        moved = True
                        print(f"  Successfully moved to: {current_cell}")
            
            # Step 3: Try left turn
            if not moved:
                left_direction = (current_direction - 1) % 4
                left_dx, left_dy = direction_vectors[left_direction]
                left_cell = (current_cell[0] + left_dx, current_cell[1] + left_dy)
                left_sensor = sensor_mapping[left_direction]
                
                if self._is_valid_move(left_cell, sensors.get(left_sensor, 0)):
                    print(f"  Left turn: turning {directions[left_direction]} and moving to {left_cell}")
                    current_direction = left_direction
                    if self._execute_directional_move(directions[current_direction]):
                        current_cell = self.robot.get_current_cell() or left_cell
                        moved = True
                        print(f"  Successfully moved to: {current_cell}")
                else:
                    print(f"  Turning left to face {directions[left_direction]} (no movement)")
                    current_direction = left_direction
                    moved = True
            
            # Step 4: Turn around if stuck
            if not moved:
                print("  Dead end: turning around 180 degrees")
                current_direction = (current_direction + 2) % 4
                print(f"  Now facing: {directions[current_direction]}")
                moved = True
            
            time.sleep(0.1)
        
        success = current_cell == end_cell
        result = {
            'algorithm': 'Right Wall Following',
            'success': success,
            'steps': step_count,
            'visited_count': len(visited),
            'final_position': current_cell
        }
        
        self.visualizer.print_results(result, visited)
        return success
    
    def run_smart_wall_following(self, start_cell=None, end_cell=(11, 11)):
        """
        Run Smart Wall Following with mapping memory.
        
        Args:
            start_cell (tuple): Starting cell coordinates
            end_cell (tuple): Target cell coordinates
            
        Returns:
            bool: True if target reached, False otherwise
        """
        print("=== Starting Smart Wall Following ===")
        print("Goal: Enhanced wall following with mapping")
        
        if start_cell is None:
            start_cell = self.robot.get_current_cell()
        
        if start_cell is None:
            print("ERROR: Cannot get starting position!")
            return False
        
        wall_map = {}  # (from_cell, to_cell) -> is_blocked
        visited = set()
        path_taken = []
        
        current_cell = start_cell
        visited.add(current_cell)
        path_taken.append(current_cell)
        
        directions = ['up', 'right', 'down', 'left']
        direction_vectors = [(0, 1), (1, 0), (0, -1), (-1, 0)]
        current_direction = 0  # Start facing up
        
        step_count = 0
        max_steps = 500
        
        while current_cell != end_cell and step_count < max_steps:
            step_count += 1
            visited.add(current_cell)
            
            if step_count % 20 == 0:
                self.visualizer.print_progress(visited, current_cell, step_count)
            
            print(f"\n--- Step {step_count} ---")
            print(f"At {current_cell}, facing {directions[current_direction]}")
            
            # Update wall map with sensor readings
            self._update_wall_map(current_cell, wall_map)
            
            # Choose next direction using smart logic
            next_direction, next_cell = self._choose_smart_direction(
                current_cell, current_direction, wall_map, visited, directions, direction_vectors
            )
            
            if next_cell is None:
                print("No valid moves available!")
                break
            
            print(f"Choosing to go {directions[next_direction]} to {next_cell}")
            
            # Execute the move
            if self._execute_directional_move(directions[next_direction]):
                current_direction = next_direction
                current_cell = self.robot.get_current_cell() or next_cell
                path_taken.append(current_cell)
                print(f"Successfully moved to: {current_cell}")
            else:
                print(f"Failed to move {directions[next_direction]}")
                wall_map[(current_cell, next_cell)] = True
        
        success = current_cell == end_cell
        result = {
            'algorithm': 'Smart Wall Following',
            'success': success,
            'steps': step_count,
            'visited_count': len(visited),
            'path_efficiency': len(visited)/step_count*100 if step_count > 0 else 0,
            'final_position': current_cell
        }
        
        self.visualizer.print_results(result, visited)
        return success
    
    # =================== HELPER METHODS ===================
    
    def _is_valid_cell(self, cell):
        """Check if cell coordinates are within maze bounds."""
        return 0 <= cell[0] <= 11 and 0 <= cell[1] <= 11
    
    def _is_wall_detected(self, sensor_key, sensors):
        """Check if wall is detected using sensor reading."""
        sensor_value = sensors.get(sensor_key, None)
        if sensor_value is None:
            return True  # Assume wall if sensor unavailable
        return sensor_value < Config.SENSOR_THRESHOLD  # Wall detection threshold
    
    def _is_valid_move(self, target_cell, sensor_value):
        """Check if move to target cell is valid."""
        if not self._is_valid_cell(target_cell):
            return False
        if sensor_value is not None and sensor_value < Config.SENSOR_THRESHOLD:
            return False
        return True
    
    def _execute_directional_move(self, direction):
        """Execute movement in specified direction."""
        if direction == 'up':
            return self.robot.move_top(verbose=False)
        elif direction == 'right':
            return self.robot.move_right(verbose=False)
        elif direction == 'down':
            return self.robot.move_bottom(verbose=False)
        elif direction == 'left':
            return self.robot.move_left(verbose=False)
        return False
    
    def _update_wall_map(self, current_cell, wall_map):
        """Update wall map based on current sensor readings."""
        sensors = self.robot.get_sensor_readings()
        
        moves = [
            ('up', (0, 1), 'front'),
            ('right', (1, 0), 'right'),
            ('down', (0, -1), 'back'),
            ('left', (-1, 0), 'left')
        ]
        
        for direction, (dx, dy), sensor_key in moves:
            neighbor = (current_cell[0] + dx, current_cell[1] + dy)
            
            if not self._is_valid_cell(neighbor):
                wall_map[(current_cell, neighbor)] = True
                continue
            
            sensor_value = sensors.get(sensor_key, float('inf'))
            wall_map[(current_cell, neighbor)] = sensor_value < Config.SENSOR_THRESHOLD
    
    def _choose_smart_direction(self, current_cell, current_direction, wall_map, visited, directions, direction_vectors):
        """Choose best direction using smart wall following logic."""
        # Priority: left turn, straight, right turn, u-turn
        direction_priorities = [
            (current_direction - 1) % 4,  # Left turn
            current_direction,              # Straight
            (current_direction + 1) % 4,   # Right turn
            (current_direction + 2) % 4    # U-turn
        ]
        
        for next_direction in direction_priorities:
            dx, dy = direction_vectors[next_direction]
            next_cell = (current_cell[0] + dx, current_cell[1] + dy)
            
            if not self._is_valid_cell(next_cell):
                continue
            
            if wall_map.get((current_cell, next_cell), False):
                continue
            
            # Prefer unvisited cells
            if next_cell not in visited:
                return next_direction, next_cell
        
        # If all preferred moves lead to visited cells, pick any valid one
        for next_direction in direction_priorities:
            dx, dy = direction_vectors[next_direction]
            next_cell = (current_cell[0] + dx, current_cell[1] + dy)
            
            if (self._is_valid_cell(next_cell) and 
                not wall_map.get((current_cell, next_cell), False)):
                return next_direction, next_cell
        
        return None, None
