"""
Navigation Algorithms for Maze Robot

This module contains all the pathfinding and navigation algorithms for autonomous
maze navigation. It provides a clean separation between the robot hardware
interface and the high-level navigation strategies.

Implemented Algorithms:
1. Depth-First Search (DFS) - Systematic exploration with backtracking
2. Breadth-First Search (BFS) - Level-by-level exploration for shortest path
3. A* Search - Heuristic-guided optimal pathfinding
4. Flood Fill - Distance-based exploration with optimal path reconstruction
5. Left Wall Following - Classic left-hand maze navigation
6. Right Wall Following - Classic right-hand maze navigation  
7. Smart Wall Following - Enhanced wall following with mapping memory

Authors: Christopher Hunter-Bennett
         David Rasheeld Watler
         Keshav Chaurasia 
         Mark Arthur Gabiana
         Rainer Knapp
Date: August 2025
Academic Project: Maze Navigation with Four-Wheel Robot
"""

from config import Config
from maze_visualizer import MazeVisualizer
import time
import logging
import heapq
import os
from datetime import datetime


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
        self._setup_logging()
    
    def _setup_logging(self):
        """Setup detailed logging for navigation algorithms."""
        # Create logs directory if it doesn't exist
        if not os.path.exists("logs"):
            os.makedirs("logs")
        
        # Create unique log filename with timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_filename = f"logs/navigation_detailed_{timestamp}.log"
        
        # Setup logger
        self.logger = logging.getLogger(f'NavigationAlgorithms_{timestamp}')
        self.logger.setLevel(logging.DEBUG)
        
        # Remove any existing handlers
        for handler in self.logger.handlers[:]:
            self.logger.removeHandler(handler)
        
        # Create file handler
        file_handler = logging.FileHandler(log_filename)
        file_handler.setLevel(logging.DEBUG)
        
        # Create console handler
        console_handler = logging.StreamHandler()
        console_handler.setLevel(logging.INFO)
        
        # Create formatter
        formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
        file_handler.setFormatter(formatter)
        console_handler.setFormatter(formatter)
        
        # Add handlers to logger
        self.logger.addHandler(file_handler)
        self.logger.addHandler(console_handler)
        
        self.logger.info(f"Logging initialized. Log file: {log_filename}")
    
    def _log_data_structures(self, step_count, current_cell, **kwargs):
        """Log all data structures at current step."""
        log_msg = f"\n=== STEP {step_count} DATA STRUCTURES ===\n"
        log_msg += f"Current Cell: {current_cell}\n"
        
        for name, data in kwargs.items():
            if data is not None:
                if isinstance(data, set):
                    log_msg += f"{name}: {sorted(list(data))}\n"
                elif isinstance(data, dict):
                    if name == 'accessible_from':
                        log_msg += f"{name}:\n"
                        for cell, neighbors in sorted(data.items()):
                            log_msg += f"  {cell} -> {sorted(list(neighbors))}\n"
                    elif name == 'distance_map':
                        log_msg += f"{name}:\n"
                        for cell, distance in sorted(data.items()):
                            log_msg += f"  {cell}: {distance}\n"
                    else:
                        log_msg += f"{name}: {dict(sorted(data.items()))}\n"
                elif isinstance(data, list):
                    log_msg += f"{name}: {data}\n"
                else:
                    log_msg += f"{name}: {data}\n"
        
        self.logger.debug(log_msg)
    
    def _log_visual_board(self, visited_cells, current_cell, path=None, step_count=0, algorithm=""):
        """Log visual representation of the maze board."""
        board_str = f"\n=== VISUAL BOARD - STEP {step_count} - {algorithm} ===\n"
        board_str += "   0  1  2  3  4  5  6  7  8  9 10 11\n"
        
        for y in range(11, -1, -1):  # Start from top (y=11) and go down
            board_str += f"{y:2d} "
            for x in range(12):
                cell = (x, y)
                if cell == current_cell:
                    board_str += "R  "  # Robot position
                elif path and cell in path:
                    board_str += "P  "  # Path
                elif cell in visited_cells:
                    board_str += "V  "  # Visited
                else:
                    board_str += ".  "  # Unvisited
            board_str += f" {y:2d}\n"
        
        board_str += "   0  1  2  3  4  5  6  7  8  9 10 11\n"
        board_str += "Legend: R=Robot, V=Visited, P=Path, .=Unvisited\n"
        
        # Log to file
        self.logger.debug(board_str)
        
        # Print to console (simplified version)
        print(f"\n=== STEP {step_count} - {algorithm} ===")
        print(f"Current Position: {current_cell}")
        print(f"Visited Cells: {len(visited_cells)}")
        if path:
            print(f"Path Length: {len(path)}")
    
    def _comprehensive_log(self, step_count, algorithm, current_cell, **data_structures):
        """Comprehensive logging combining data structures and visual board."""
        # Log data structures
        self._log_data_structures(step_count, current_cell, **data_structures)
        
        # Extract visited cells and path for visual board
        visited = data_structures.get('visited', set()) or data_structures.get('explored_cells', set())
        path = data_structures.get('path_stack') or data_structures.get('path_history') or data_structures.get('optimal_path')
        
        # Log visual board
        self._log_visual_board(visited, current_cell, path, step_count, algorithm)
    
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
        max_steps = 10000
        
        print(f"Starting from {start_cell}, target: {end_cell}")
        self.logger.info(f"DFS Algorithm Starting: {start_cell} -> {end_cell}")
        
        # Initial comprehensive log
        self._comprehensive_log(0, "DFS", current_cell, 
                               visited=visited, 
                               path_stack=path_stack, 
                               path_history=path_history)
        
        while current_cell != end_cell and step_count < max_steps:
            step_count += 1
            
            # Regular progress updates
            if step_count % 50 == 0:
                self.visualizer.print_progress(visited, current_cell, step_count)
            
            print(f"\n--- Step {step_count} ---")
            print(f"Current cell: {current_cell}")
            self.logger.info(f"DFS Step {step_count}: Current cell {current_cell}")
            
            # Get sensor readings
            sensors = self.robot.get_sensor_readings()
            print(f"Sensor readings: {sensors}")
            self.logger.debug(f"Sensor readings: {sensors}")
            
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
                self.logger.debug(f"  Attempting move {direction} to {next_cell}")
                
                success = self._execute_directional_move(direction)
                
                if success:
                    current_cell = self.robot.get_current_cell() or next_cell
                    visited.add(current_cell)
                    path_stack.append(current_cell)
                    path_history.append(current_cell)
                    found_move = True
                    print(f"  Successfully moved to: {current_cell}")
                    self.logger.info(f"  Successfully moved to: {current_cell}")
                    break
                else:
                    print(f"  Failed to move {direction}")
                    self.logger.warning(f"  Failed to move {direction}")
            
            # Backtrack if no valid move found
            if not found_move:
                if len(path_stack) <= 1:
                    print("ERROR: No more moves available!")
                    self.logger.error("No more moves available - terminating DFS")
                    break
                
                path_stack.pop()  # Remove current cell
                if not path_stack:
                    break
                
                target_cell = path_stack[-1]
                print(f"  Backtracking to cell: {target_cell}")
                self.logger.info(f"  Backtracking to cell: {target_cell}")
                
                if self.robot.move_to_cell(target_cell[0], target_cell[1], verbose=True):
                    current_cell = target_cell
                    path_history.append(current_cell)
                    print(f"  Backtracked to: {current_cell}")
                    self.logger.info(f"  Backtracked to: {current_cell}")
                else:
                    print(f"  Failed to backtrack to: {target_cell}")
                    self.logger.error(f"  Failed to backtrack to: {target_cell}")
                    break
            
            # Comprehensive logging at each step
            self._comprehensive_log(step_count, "DFS", current_cell, 
                                   visited=visited, 
                                   path_stack=path_stack, 
                                   path_history=path_history)
            
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
        
        # Final comprehensive log
        self._comprehensive_log(step_count, "DFS-FINAL", current_cell, 
                               visited=visited, 
                               path_stack=path_stack, 
                               path_history=path_history,
                               result=result)
        
        self.logger.info(f"DFS Algorithm Completed: Success={success}, Steps={step_count}, Visited={len(visited)}")
        self.visualizer.print_results(result, visited, path_stack)
        return success
    
    def run_bfs(self, start_cell=None, end_cell=(11, 11)):
        """
        Run Breadth-First Search algorithm.
        
        This implementation properly handles physical robot limitations:
        1. Explores maze systematically while building obstacle map
        2. Uses true BFS on discovered accessible areas
        3. Ensures all movements respect obstacles and adjacency
        
        Args:
            start_cell (tuple): Starting cell coordinates, None for current position
            end_cell (tuple): Target cell coordinates
            
        Returns:
            bool: True if target reached, False otherwise
        """
        print("=== Starting Breadth-First Search ===")
        print("Goal: Find shortest path using level-by-level exploration")
        
        # Initialize starting position
        if start_cell is None:
            start_cell = self.robot.get_current_cell()
        
        if start_cell is None:
            print("ERROR: Cannot get starting position!")
            return False
        
        print(f"Starting from {start_cell}, target: {end_cell}")
        self.logger.info(f"BFS Algorithm Starting: {start_cell} -> {end_cell}")
        
        # Data structures for exploration and pathfinding
        explored_cells = set()  # Cells we've physically visited
        accessible_from = {}    # cell -> {adjacent_cells_we_can_reach}
        parent_path = {}        # For backtracking during exploration
        
        # BFS exploration using actual robot movement
        from collections import deque
        exploration_queue = deque([start_cell])  # Just store the cell to explore
        explored_cells.add(start_cell)
        
        current_robot_pos = start_cell
        step_count = 0
        max_exploration_steps = 500
        
        print("\n=== Phase 1: BFS Exploration ===")
        self.logger.info("=== Phase 1: BFS Exploration ===")
        
        # Initial comprehensive log
        self._comprehensive_log(0, "BFS-EXPLORATION", current_robot_pos, 
                               explored_cells=explored_cells, 
                               accessible_from=accessible_from, 
                               parent_path=parent_path,
                               exploration_queue=list(exploration_queue))
        
        while exploration_queue and step_count < max_exploration_steps:
            step_count += 1
            target_cell = exploration_queue.popleft()
            
            print(f"\nStep {step_count}: Exploring {target_cell}")
            self.logger.info(f"BFS Exploration Step {step_count}: Exploring {target_cell}")
            
            # Move robot to target cell if not already there
            if current_robot_pos != target_cell:
                # Find path to target using already explored connections
                path_to_target = self._find_safe_path(current_robot_pos, target_cell, accessible_from)
                
                if path_to_target is None or len(path_to_target) < 2:
                    print(f"Could not find path from {current_robot_pos} to {target_cell}")
                    self.logger.warning(f"Could not find path from {current_robot_pos} to {target_cell}")
                    continue
                
                # Execute path step by step (skip first element as it's current position)
                for i in range(1, len(path_to_target)):
                    next_pos = path_to_target[i]
                    prev_pos = path_to_target[i-1]
                    
                    # Calculate direction to next position
                    dx = next_pos[0] - prev_pos[0]
                    dy = next_pos[1] - prev_pos[1]
                    
                    # Ensure it's an adjacent move
                    if abs(dx) + abs(dy) != 1:
                        print(f"ERROR: Invalid path step from {prev_pos} to {next_pos}")
                        self.logger.error(f"Invalid path step from {prev_pos} to {next_pos}")
                        break
                    
                    # Determine direction
                    if dx == 1:
                        direction = 'right'
                    elif dx == -1:
                        direction = 'left'
                    elif dy == 1:
                        direction = 'up'
                    elif dy == -1:
                        direction = 'down'
                    else:
                        print(f"ERROR: Invalid direction calculation")
                        self.logger.error(f"Invalid direction calculation")
                        break
                    
                    # Execute movement
                    print(f"  Moving {direction} from {prev_pos} to {next_pos}")
                    self.logger.debug(f"Moving {direction} from {prev_pos} to {next_pos}")
                    
                    if not self._execute_directional_move(direction):
                        print(f"  Failed to move {direction} - path blocked!")
                        self.logger.warning(f"Failed to move {direction} - path blocked!")
                        break
                    
                    current_robot_pos = self.robot.get_current_cell() or next_pos
                    
                    if current_robot_pos != next_pos:
                        print(f"  Robot ended up at {current_robot_pos} instead of {next_pos}")
                        self.logger.warning(f"Robot ended up at {current_robot_pos} instead of {next_pos}")
                        break
                
                # Verify we reached the target
                if current_robot_pos != target_cell:
                    print(f"Could not reach {target_cell}, robot at {current_robot_pos}")
                    self.logger.warning(f"Could not reach {target_cell}, robot at {current_robot_pos}")
                    continue
            
            # Initialize adjacency for this cell
            if target_cell not in accessible_from:
                accessible_from[target_cell] = set()
            
            # Check if target found
            if target_cell == end_cell:
                print(f"TARGET FOUND at step {step_count}!")
                self.logger.info(f"TARGET FOUND at step {step_count}!")
                break
            
            # Explore neighbors from current position
            sensors = self.robot.get_sensor_readings()
            print(f"  Sensor readings: {sensors}")
            self.logger.debug(f"Sensor readings: {sensors}")
            
            moves = [
                ('right', (1, 0), 'right'),
                ('up', (0, 1), 'front'),
                ('left', (-1, 0), 'left'),
                ('down', (0, -1), 'back')
            ]
            
            new_neighbors = 0
            
            for direction, (dx, dy), sensor_key in moves:
                neighbor = (target_cell[0] + dx, target_cell[1] + dy)
                
                # Check bounds
                if not self._is_valid_cell(neighbor):
                    continue
                
                # Check for wall
                if self._is_wall_detected(sensor_key, sensors):
                    print(f"    Wall detected {direction} to {neighbor}")
                    self.logger.debug(f"Wall detected {direction} to {neighbor}")
                    continue
                
                # Add to adjacency (bidirectional)
                accessible_from[target_cell].add(neighbor)
                if neighbor not in accessible_from:
                    accessible_from[neighbor] = set()
                accessible_from[neighbor].add(target_cell)
                
                # Add to exploration queue if not explored
                if neighbor not in explored_cells:
                    explored_cells.add(neighbor)
                    exploration_queue.append(neighbor)
                    new_neighbors += 1
                    print(f"    Added neighbor {direction}: {neighbor}")
                    self.logger.debug(f"Added neighbor {direction}: {neighbor}")
            
            print(f"  Added {new_neighbors} new neighbors. Queue size: {len(exploration_queue)}")
            self.logger.debug(f"Added {new_neighbors} new neighbors. Queue size: {len(exploration_queue)}")
            
            # Comprehensive logging at each step
            self._comprehensive_log(step_count, "BFS-EXPLORATION", current_robot_pos, 
                                   explored_cells=explored_cells, 
                                   accessible_from=accessible_from, 
                                   parent_path=parent_path,
                                   exploration_queue=list(exploration_queue))
        
        print(f"\nExploration complete. Explored {len(explored_cells)} cells.")
        self.logger.info(f"BFS Exploration complete. Explored {len(explored_cells)} cells.")
        
        # Check if target is reachable
        if end_cell not in explored_cells:
            print("ERROR: Target is not reachable!")
            result = {
                'algorithm': 'BFS',
                'success': False,
                'steps': step_count,
                'visited_count': len(explored_cells),
                'final_position': current_robot_pos
            }
            self.visualizer.print_results(result, explored_cells)
            return False
        
        # Phase 2: Find shortest path using BFS on explored graph
        print("\n=== Phase 2: BFS Shortest Path ===")
        
        path_queue = deque([(start_cell, [start_cell])])
        path_visited = set([start_cell])
        shortest_path = None
        
        while path_queue:
            current_cell, path = path_queue.popleft()
            
            if current_cell == end_cell:
                shortest_path = path
                print(f"Found shortest path with {len(path)} steps!")
                break
            
            # Explore accessible neighbors
            for neighbor in accessible_from.get(current_cell, set()):
                if neighbor not in path_visited:
                    path_visited.add(neighbor)
                    new_path = path + [neighbor]
                    path_queue.append((neighbor, new_path))
        
        if shortest_path is None:
            print("ERROR: No path found in explored area!")
            result = {
                'algorithm': 'BFS',
                'success': False,
                'steps': step_count,
                'visited_count': len(explored_cells),
                'final_position': current_robot_pos
            }
            self.visualizer.print_results(result, explored_cells)
            return False
        
        # Phase 3: Execute shortest path with obstacle-aware backtracking
        print(f"\n=== Phase 3: Executing Shortest Path ===")
        print(f"Shortest path: {shortest_path}")
        
        # First, navigate back to start using DFS backtracking (safe path)
        if current_robot_pos != start_cell:
            print(f"Navigating back to start from {current_robot_pos}")
            
            # Use DFS to find path back to start through explored cells
            back_path = self._find_safe_path(current_robot_pos, start_cell, accessible_from)
            
            if back_path is None:
                print("ERROR: Cannot find safe path back to start!")
                return False
            
            # Execute path back to start
            for i in range(1, len(back_path)):
                next_cell = back_path[i]
                prev_cell = back_path[i-1]
                
                dx = next_cell[0] - prev_cell[0]
                dy = next_cell[1] - prev_cell[1]
                
                if dx == 1:
                    direction = 'right'
                elif dx == -1:
                    direction = 'left'
                elif dy == 1:
                    direction = 'up'
                elif dy == -1:
                    direction = 'down'
                else:
                    print(f"ERROR: Invalid backtrack step from {prev_cell} to {next_cell}")
                    return False
                
                if not self._execute_directional_move(direction):
                    print(f"ERROR: Failed to backtrack {direction}")
                    return False
                
                current_robot_pos = self.robot.get_current_cell()
                print(f"  Backtracked to: {current_robot_pos}")
        
        # Now execute the shortest path
        execution_steps = 0
        final_cell = start_cell
        
        for i in range(1, len(shortest_path)):
            target_cell = shortest_path[i]
            prev_cell = shortest_path[i-1]
            
            # Calculate direction
            dx = target_cell[0] - prev_cell[0]
            dy = target_cell[1] - prev_cell[1]
            
            if dx == 1 and dy == 0:
                direction = 'right'
            elif dx == -1 and dy == 0:
                direction = 'left'
            elif dx == 0 and dy == 1:
                direction = 'up'
            elif dx == 0 and dy == -1:
                direction = 'down'
            else:
                print(f"ERROR: Invalid move from {prev_cell} to {target_cell}")
                break
            
            execution_steps += 1
            print(f"Step {execution_steps}: Moving {direction} to {target_cell}")
            
            # Execute the move
            success = self._execute_directional_move(direction)
            
            if success:
                final_cell = self.robot.get_current_cell() or target_cell
                print(f"Successfully moved to: {final_cell}")
                
                if final_cell == end_cell:
                    print("TARGET REACHED!")
                    break
            else:
                print(f"Failed to move {direction}")
                break
            
            time.sleep(0.1)  # Visualization delay
        
        # Results
        success = final_cell == end_cell
        result = {
            'algorithm': 'BFS',
            'success': success,
            'steps': execution_steps,
            'exploration_steps': step_count,
            'visited_count': len(explored_cells),
            'path_length': len(shortest_path),
            'shortest_path': shortest_path,
            'final_position': final_cell
        }
        
        # Final comprehensive log
        self._comprehensive_log(execution_steps, "BFS-FINAL", final_cell, 
                               explored_cells=explored_cells, 
                               accessible_from=accessible_from, 
                               shortest_path=shortest_path,
                               result=result)
        
        self.logger.info(f"BFS Algorithm Completed: Success={success}, Steps={execution_steps}, Explored={len(explored_cells)}")
        self.visualizer.print_results(result, explored_cells, shortest_path)
        return success
    
    def _find_safe_path(self, start, end, accessible_from):
        """Find a safe path between two cells using explored adjacency map."""
        if start == end:
            return [start]
        
        from collections import deque
        queue = deque([(start, [start])])
        visited = set([start])
        
        while queue:
            current, path = queue.popleft()
            
            for neighbor in accessible_from.get(current, set()):
                if neighbor == end:
                    return path + [neighbor]
                
                if neighbor not in visited:
                    visited.add(neighbor)
                    queue.append((neighbor, path + [neighbor]))
        
        return None  # No path found
    
    def run_astar(self, start_cell=None, end_cell=(11, 11)):
        """
        Run A* Search algorithm.
        
        This implementation uses Manhattan distance heuristic and handles
        physical robot limitations similar to BFS:
        1. Explores maze systematically while building obstacle map
        2. Uses A* on discovered accessible areas with heuristic guidance
        3. Ensures all movements respect obstacles and adjacency
        
        Args:
            start_cell (tuple): Starting cell coordinates, None for current position
            end_cell (tuple): Target cell coordinates
            
        Returns:
            bool: True if target reached, False otherwise
        """
        print("=== Starting A* Search ===")
        print("Goal: Find optimal path using heuristic-guided search")
        
        # Initialize starting position
        if start_cell is None:
            start_cell = self.robot.get_current_cell()
        
        if start_cell is None:
            print("ERROR: Cannot get starting position!")
            return False
        
        print(f"Starting from {start_cell}, target: {end_cell}")
        self.logger.info(f"A* Algorithm Starting: {start_cell} -> {end_cell}")
        
        # Data structures for exploration and pathfinding
        explored_cells = set()  # Cells we've physically visited
        accessible_from = {}    # cell -> {adjacent_cells_we_can_reach}
        
        # A* exploration using actual robot movement with heuristic priority
        
        def manhattan_distance(cell1, cell2):
            """Calculate Manhattan distance heuristic."""
            return abs(cell1[0] - cell2[0]) + abs(cell1[1] - cell2[1])
        
        # Priority queue: (f_score, step_count, cell)
        # f_score = g_score + h_score (actual cost + heuristic)
        exploration_queue = []
        heapq.heappush(exploration_queue, (manhattan_distance(start_cell, end_cell), 0, start_cell))
        explored_cells.add(start_cell)
        
        current_robot_pos = start_cell
        step_count = 0
        max_exploration_steps = 500
        
        print("\n=== Phase 1: A* Exploration ===")
        
        while exploration_queue and step_count < max_exploration_steps:
            step_count += 1
            
            # Get cell with lowest f_score
            f_score, g_score, target_cell = heapq.heappop(exploration_queue)
            
            print(f"\nStep {step_count}: Exploring {target_cell} (f={f_score:.1f}, g={g_score})")
            
            # Move robot to target cell if not already there
            if current_robot_pos != target_cell:
                # Find path to target using already explored connections
                path_to_target = self._find_safe_path(current_robot_pos, target_cell, accessible_from)
                
                if path_to_target is None or len(path_to_target) < 2:
                    print(f"Could not find path from {current_robot_pos} to {target_cell}")
                    continue
                
                # Execute path step by step (skip first element as it's current position)
                for i in range(1, len(path_to_target)):
                    next_pos = path_to_target[i]
                    prev_pos = path_to_target[i-1]
                    
                    # Calculate direction to next position
                    dx = next_pos[0] - prev_pos[0]
                    dy = next_pos[1] - prev_pos[1]
                    
                    # Ensure it's an adjacent move
                    if abs(dx) + abs(dy) != 1:
                        print(f"ERROR: Invalid path step from {prev_pos} to {next_pos}")
                        break
                    
                    # Determine direction
                    if dx == 1:
                        direction = 'right'
                    elif dx == -1:
                        direction = 'left'
                    elif dy == 1:
                        direction = 'up'
                    elif dy == -1:
                        direction = 'down'
                    else:
                        print(f"ERROR: Invalid direction calculation")
                        break
                    
                    # Execute movement
                    print(f"  Moving {direction} from {prev_pos} to {next_pos}")
                    if not self._execute_directional_move(direction):
                        print(f"  Failed to move {direction} - path blocked!")
                        break
                    
                    current_robot_pos = self.robot.get_current_cell() or next_pos
                    
                    if current_robot_pos != next_pos:
                        print(f"  Robot ended up at {current_robot_pos} instead of {next_pos}")
                        break
                
                # Verify we reached the target
                if current_robot_pos != target_cell:
                    print(f"Could not reach {target_cell}, robot at {current_robot_pos}")
                    continue
            
            # Initialize adjacency for this cell
            if target_cell not in accessible_from:
                accessible_from[target_cell] = set()
            
            # Check if target found
            if target_cell == end_cell:
                print(f"TARGET FOUND at step {step_count}!")
                break
            
            # Explore neighbors from current position
            sensors = self.robot.get_sensor_readings()
            print(f"  Sensor readings: {sensors}")
            
            moves = [
                ('right', (1, 0), 'right'),
                ('up', (0, 1), 'front'),
                ('left', (-1, 0), 'left'),
                ('down', (0, -1), 'back')
            ]
            
            new_neighbors = 0
            
            for direction, (dx, dy), sensor_key in moves:
                neighbor = (target_cell[0] + dx, target_cell[1] + dy)
                
                # Check bounds
                if not self._is_valid_cell(neighbor):
                    continue
                
                # Check for wall
                if self._is_wall_detected(sensor_key, sensors):
                    print(f"    Wall detected {direction} to {neighbor}")
                    continue
                
                # Add to adjacency (bidirectional)
                accessible_from[target_cell].add(neighbor)
                if neighbor not in accessible_from:
                    accessible_from[neighbor] = set()
                accessible_from[neighbor].add(target_cell)
                
                # Add to exploration queue if not explored
                if neighbor not in explored_cells:
                    explored_cells.add(neighbor)
                    neighbor_g_score = g_score + 1  # Distance from start
                    neighbor_h_score = manhattan_distance(neighbor, end_cell)  # Heuristic
                    neighbor_f_score = neighbor_g_score + neighbor_h_score
                    
                    heapq.heappush(exploration_queue, (neighbor_f_score, neighbor_g_score, neighbor))
                    new_neighbors += 1
                    print(f"    Added neighbor {direction}: {neighbor} (f={neighbor_f_score:.1f})")
            
            print(f"  Added {new_neighbors} new neighbors. Queue size: {len(exploration_queue)}")
        
        print(f"\nExploration complete. Explored {len(explored_cells)} cells.")
        
        # Check if target is reachable
        if end_cell not in explored_cells:
            print("ERROR: Target is not reachable!")
            result = {
                'algorithm': 'A*',
                'success': False,
                'steps': step_count,
                'visited_count': len(explored_cells),
                'final_position': current_robot_pos
            }
            self.visualizer.print_results(result, explored_cells)
            return False
        
        # Phase 2: Find optimal path using A* on explored graph
        print("\n=== Phase 2: A* Optimal Path ===")
        
        # A* pathfinding on discovered graph
        path_queue = []  # (f_score, g_score, cell, path)
        heapq.heappush(path_queue, (manhattan_distance(start_cell, end_cell), 0, start_cell, [start_cell]))
        path_visited = set([start_cell])
        optimal_path = None
        
        while path_queue:
            f_score, g_score, current_cell, path = heapq.heappop(path_queue)
            
            if current_cell == end_cell:
                optimal_path = path
                print(f"Found optimal path with {len(path)} steps!")
                break
            
            # Explore accessible neighbors
            for neighbor in accessible_from.get(current_cell, set()):
                if neighbor not in path_visited:
                    path_visited.add(neighbor)
                    new_g_score = g_score + 1
                    new_h_score = manhattan_distance(neighbor, end_cell)
                    new_f_score = new_g_score + new_h_score
                    new_path = path + [neighbor]
                    heapq.heappush(path_queue, (new_f_score, new_g_score, neighbor, new_path))
        
        if optimal_path is None:
            print("ERROR: No path found in explored area!")
            result = {
                'algorithm': 'A*',
                'success': False,
                'steps': step_count,
                'visited_count': len(explored_cells),
                'final_position': current_robot_pos
            }
            self.visualizer.print_results(result, explored_cells)
            return False
        
        # Phase 3: Execute optimal path with obstacle-aware backtracking
        print(f"\n=== Phase 3: Executing Optimal Path ===")
        print(f"Optimal path: {optimal_path}")
        
        # First, navigate back to start using safe path
        if current_robot_pos != start_cell:
            print(f"Navigating back to start from {current_robot_pos}")
            
            # Use pathfinding to find path back to start through explored cells
            back_path = self._find_safe_path(current_robot_pos, start_cell, accessible_from)
            
            if back_path is None:
                print("ERROR: Cannot find safe path back to start!")
                return False
            
            # Execute path back to start
            for i in range(1, len(back_path)):
                next_cell = back_path[i]
                prev_cell = back_path[i-1]
                
                dx = next_cell[0] - prev_cell[0]
                dy = next_cell[1] - prev_cell[1]
                
                if dx == 1:
                    direction = 'right'
                elif dx == -1:
                    direction = 'left'
                elif dy == 1:
                    direction = 'up'
                elif dy == -1:
                    direction = 'down'
                else:
                    print(f"ERROR: Invalid backtrack step from {prev_cell} to {next_cell}")
                    return False
                
                if not self._execute_directional_move(direction):
                    print(f"ERROR: Failed to backtrack {direction}")
                    return False
                
                current_robot_pos = self.robot.get_current_cell()
                print(f"  Backtracked to: {current_robot_pos}")
        
        # Now execute the optimal path
        execution_steps = 0
        final_cell = start_cell
        
        for i in range(1, len(optimal_path)):
            target_cell = optimal_path[i]
            prev_cell = optimal_path[i-1]
            
            # Calculate direction
            dx = target_cell[0] - prev_cell[0]
            dy = target_cell[1] - prev_cell[1]
            
            if dx == 1 and dy == 0:
                direction = 'right'
            elif dx == -1 and dy == 0:
                direction = 'left'
            elif dx == 0 and dy == 1:
                direction = 'up'
            elif dx == 0 and dy == -1:
                direction = 'down'
            else:
                print(f"ERROR: Invalid move from {prev_cell} to {target_cell}")
                break
            
            execution_steps += 1
            print(f"Step {execution_steps}: Moving {direction} to {target_cell}")
            
            # Execute the move
            success = self._execute_directional_move(direction)
            
            if success:
                final_cell = self.robot.get_current_cell() or target_cell
                print(f"Successfully moved to: {final_cell}")
                
                if final_cell == end_cell:
                    print("TARGET REACHED!")
                    break
            else:
                print(f"Failed to move {direction}")
                break
            
            time.sleep(0.1)  # Visualization delay
        
        # Results
        success = final_cell == end_cell
        result = {
            'algorithm': 'A*',
            'success': success,
            'steps': execution_steps,
            'exploration_steps': step_count,
            'visited_count': len(explored_cells),
            'path_length': len(optimal_path),
            'optimal_path': optimal_path,
            'final_position': final_cell
        }
        
        self.visualizer.print_results(result, explored_cells, optimal_path)
        return success
    
    def run_flood_fill(self, start_cell=None, end_cell=(11, 11)):
        """
        Run Flood Fill algorithm.
        
        This implementation uses distance-based exploration and handles
        physical robot limitations similar to BFS and A*:
        1. Explores maze systematically while building obstacle map
        2. Assigns distance values to each accessible cell from target
        3. Finds optimal path by following gradient descent back to start
        4. Ensures all movements respect obstacles and adjacency
        
        Args:
            start_cell (tuple): Starting cell coordinates, None for current position
            end_cell (tuple): Target cell coordinates
            
        Returns:
            bool: True if target reached, False otherwise
        """
        print("=== Starting Flood Fill Algorithm ===")
        print("Goal: Find optimal path using distance-based flooding")
        
        # Initialize starting position
        if start_cell is None:
            start_cell = self.robot.get_current_cell()
        
        if start_cell is None:
            print("ERROR: Cannot get starting position!")
            return False
        
        print(f"Starting from {start_cell}, target: {end_cell}")
        self.logger.info(f"Flood Fill Algorithm Starting: {start_cell} -> {end_cell}")
        
        # Data structures for exploration and pathfinding
        explored_cells = set()  # Cells we've physically visited
        accessible_from = {}    # cell -> {adjacent_cells_we_can_reach}
        distance_map = {}       # cell -> distance_from_target
        
        # Flood fill exploration using actual robot movement
        from collections import deque
        exploration_queue = deque([start_cell])
        explored_cells.add(start_cell)
        
        current_robot_pos = start_cell
        step_count = 0
        max_exploration_steps = 500
        
        print("\n=== Phase 1: Flood Fill Exploration ===")
        
        while exploration_queue and step_count < max_exploration_steps:
            step_count += 1
            target_cell = exploration_queue.popleft()
            
            print(f"\nStep {step_count}: Exploring {target_cell}")
            
            # Move robot to target cell if not already there
            if current_robot_pos != target_cell:
                # Find path to target using already explored connections
                path_to_target = self._find_safe_path(current_robot_pos, target_cell, accessible_from)
                
                if path_to_target is None or len(path_to_target) < 2:
                    print(f"Could not find path from {current_robot_pos} to {target_cell}")
                    continue
                
                # Execute path step by step (skip first element as it's current position)
                for i in range(1, len(path_to_target)):
                    next_pos = path_to_target[i]
                    prev_pos = path_to_target[i-1]
                    
                    # Calculate direction to next position
                    dx = next_pos[0] - prev_pos[0]
                    dy = next_pos[1] - prev_pos[1]
                    
                    # Ensure it's an adjacent move
                    if abs(dx) + abs(dy) != 1:
                        print(f"ERROR: Invalid path step from {prev_pos} to {next_pos}")
                        break
                    
                    # Determine direction
                    if dx == 1:
                        direction = 'right'
                    elif dx == -1:
                        direction = 'left'
                    elif dy == 1:
                        direction = 'up'
                    elif dy == -1:
                        direction = 'down'
                    else:
                        print(f"ERROR: Invalid direction calculation")
                        break
                    
                    # Execute movement
                    print(f"  Moving {direction} from {prev_pos} to {next_pos}")
                    if not self._execute_directional_move(direction):
                        print(f"  Failed to move {direction} - path blocked!")
                        break
                    
                    current_robot_pos = self.robot.get_current_cell() or next_pos
                    
                    if current_robot_pos != next_pos:
                        print(f"  Robot ended up at {current_robot_pos} instead of {next_pos}")
                        break
                
                # Verify we reached the target
                if current_robot_pos != target_cell:
                    print(f"Could not reach {target_cell}, robot at {current_robot_pos}")
                    continue
            
            # Initialize adjacency for this cell
            if target_cell not in accessible_from:
                accessible_from[target_cell] = set()
            
            # Check if target found
            if target_cell == end_cell:
                print(f"TARGET FOUND at step {step_count}!")
                break
            
            # Explore neighbors from current position
            sensors = self.robot.get_sensor_readings()
            print(f"  Sensor readings: {sensors}")
            
            moves = [
                ('right', (1, 0), 'right'),
                ('up', (0, 1), 'front'),
                ('left', (-1, 0), 'left'),
                ('down', (0, -1), 'back')
            ]
            
            new_neighbors = 0
            
            for direction, (dx, dy), sensor_key in moves:
                neighbor = (target_cell[0] + dx, target_cell[1] + dy)
                
                # Check bounds
                if not self._is_valid_cell(neighbor):
                    continue
                
                # Check for wall
                if self._is_wall_detected(sensor_key, sensors):
                    print(f"    Wall detected {direction} to {neighbor}")
                    continue
                
                # Add to adjacency (bidirectional)
                accessible_from[target_cell].add(neighbor)
                if neighbor not in accessible_from:
                    accessible_from[neighbor] = set()
                accessible_from[neighbor].add(target_cell)
                
                # Add to exploration queue if not explored
                if neighbor not in explored_cells:
                    explored_cells.add(neighbor)
                    exploration_queue.append(neighbor)
                    new_neighbors += 1
                    print(f"    Added neighbor {direction}: {neighbor}")
            
            print(f"  Added {new_neighbors} new neighbors. Queue size: {len(exploration_queue)}")
        
        print(f"\nExploration complete. Explored {len(explored_cells)} cells.")
        
        # Check if target is reachable
        if end_cell not in explored_cells:
            print("ERROR: Target is not reachable!")
            result = {
                'algorithm': 'Flood Fill',
                'success': False,
                'steps': step_count,
                'visited_count': len(explored_cells),
                'final_position': current_robot_pos
            }
            self.visualizer.print_results(result, explored_cells)
            return False
        
        # Phase 2: Build distance map using flood fill from target
        print("\n=== Phase 2: Building Distance Map ===")
        
        # Initialize distance map with target at distance 0
        distance_map[end_cell] = 0
        flood_queue = deque([end_cell])
        
        print(f"Starting flood fill from target {end_cell}")
        
        while flood_queue:
            current_cell = flood_queue.popleft()
            current_distance = distance_map[current_cell]
            
            # Flood to all accessible neighbors
            for neighbor in accessible_from.get(current_cell, set()):
                if neighbor not in distance_map:
                    distance_map[neighbor] = current_distance + 1
                    flood_queue.append(neighbor)
                    print(f"  Distance {current_distance + 1}: {neighbor}")
        
        print(f"Distance map built for {len(distance_map)} cells.")
        
        # Check if start is reachable from target
        if start_cell not in distance_map:
            print("ERROR: Start cell not reachable from target!")
            result = {
                'algorithm': 'Flood Fill',
                'success': False,
                'steps': step_count,
                'visited_count': len(explored_cells),
                'final_position': current_robot_pos
            }
            self.visualizer.print_results(result, explored_cells)
            return False
        
        # Phase 3: Find optimal path using gradient descent
        print("\n=== Phase 3: Finding Optimal Path ===")
        
        optimal_path = [start_cell]
        current_cell = start_cell
        
        print(f"Following gradient from start {start_cell} (distance: {distance_map[start_cell]})")
        
        while current_cell != end_cell:
            current_distance = distance_map[current_cell]
            best_neighbor = None
            best_distance = float('inf')
            
            # Find neighbor with smallest distance (steepest gradient)
            for neighbor in accessible_from.get(current_cell, set()):
                if neighbor in distance_map:
                    neighbor_distance = distance_map[neighbor]
                    if neighbor_distance < best_distance:
                        best_distance = neighbor_distance
                        best_neighbor = neighbor
            
            if best_neighbor is None:
                print(f"ERROR: No path forward from {current_cell}")
                break
            
            optimal_path.append(best_neighbor)
            current_cell = best_neighbor
            print(f"  Next step: {current_cell} (distance: {distance_map[current_cell]})")
        
        if current_cell != end_cell:
            print("ERROR: Could not find complete path to target!")
            result = {
                'algorithm': 'Flood Fill',
                'success': False,
                'steps': step_count,
                'visited_count': len(explored_cells),
                'final_position': current_robot_pos
            }
            self.visualizer.print_results(result, explored_cells)
            return False
        
        print(f"Optimal path found with {len(optimal_path)} steps!")
        print(f"Optimal path: {optimal_path}")
        
        # Phase 4: Execute optimal path with obstacle-aware backtracking
        print(f"\n=== Phase 4: Executing Optimal Path ===")
        
        # First, navigate back to start using safe path
        if current_robot_pos != start_cell:
            print(f"Navigating back to start from {current_robot_pos}")
            
            # Use pathfinding to find path back to start through explored cells
            back_path = self._find_safe_path(current_robot_pos, start_cell, accessible_from)
            
            if back_path is None:
                print("ERROR: Cannot find safe path back to start!")
                return False
            
            # Execute path back to start
            for i in range(1, len(back_path)):
                next_cell = back_path[i]
                prev_cell = back_path[i-1]
                
                dx = next_cell[0] - prev_cell[0]
                dy = next_cell[1] - prev_cell[1]
                
                if dx == 1:
                    direction = 'right'
                elif dx == -1:
                    direction = 'left'
                elif dy == 1:
                    direction = 'up'
                elif dy == -1:
                    direction = 'down'
                else:
                    print(f"ERROR: Invalid backtrack step from {prev_cell} to {next_cell}")
                    return False
                
                if not self._execute_directional_move(direction):
                    print(f"ERROR: Failed to backtrack {direction}")
                    return False
                
                current_robot_pos = self.robot.get_current_cell()
                print(f"  Backtracked to: {current_robot_pos}")
        
        # Now execute the optimal path
        execution_steps = 0
        final_cell = start_cell
        
        for i in range(1, len(optimal_path)):
            target_cell = optimal_path[i]
            prev_cell = optimal_path[i-1]
            
            # Calculate direction
            dx = target_cell[0] - prev_cell[0]
            dy = target_cell[1] - prev_cell[1]
            
            if dx == 1 and dy == 0:
                direction = 'right'
            elif dx == -1 and dy == 0:
                direction = 'left'
            elif dx == 0 and dy == 1:
                direction = 'up'
            elif dx == 0 and dy == -1:
                direction = 'down'
            else:
                print(f"ERROR: Invalid move from {prev_cell} to {target_cell}")
                break
            
            execution_steps += 1
            current_distance = distance_map.get(target_cell, '?')
            print(f"Step {execution_steps}: Moving {direction} to {target_cell} (distance: {current_distance})")
            
            # Execute the move
            success = self._execute_directional_move(direction)
            
            if success:
                final_cell = self.robot.get_current_cell() or target_cell
                print(f"Successfully moved to: {final_cell}")
                
                if final_cell == end_cell:
                    print("TARGET REACHED!")
                    break
            else:
                print(f"Failed to move {direction}")
                break
            
            time.sleep(0.1)  # Visualization delay
        
        # Results
        success = final_cell == end_cell
        result = {
            'algorithm': 'Flood Fill',
            'success': success,
            'steps': execution_steps,
            'exploration_steps': step_count,
            'visited_count': len(explored_cells),
            'path_length': len(optimal_path),
            'optimal_path': optimal_path,
            'distance_map_size': len(distance_map),
            'final_position': final_cell
        }
        
        self.visualizer.print_results(result, explored_cells, optimal_path)
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
        self.logger.info(f"Left Wall Following Algorithm Starting: {start_cell} -> {end_cell}")
        
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
        
        # Initial comprehensive log
        self._comprehensive_log(0, "LEFT_WALL_FOLLOWING", current_cell, 
                               visited=visited, 
                               current_direction=current_direction,
                               direction_name=directions[current_direction])
        
        while current_cell != end_cell and step_count < max_steps:
            step_count += 1
            visited.add(current_cell)
            
            if step_count % 20 == 0:
                self.visualizer.print_progress(visited, current_cell, step_count)
            
            print(f"\n--- Step {step_count} ---")
            print(f"Current cell: {current_cell}")
            print(f"Current direction: {directions[current_direction]}")
            self.logger.info(f"Left Wall Following Step {step_count}: At {current_cell}, facing {directions[current_direction]}")
            
            sensors = self.robot.get_sensor_readings()
            self.logger.debug(f"Sensor readings: {sensors}")
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
                self.logger.info("  Dead end: turning around 180 degrees")
                current_direction = (current_direction + 2) % 4
                print(f"  Now facing: {directions[current_direction]}")
                self.logger.info(f"  Now facing: {directions[current_direction]}")
                moved = True
            
            # Comprehensive logging at each step
            self._comprehensive_log(step_count, "LEFT_WALL_FOLLOWING", current_cell, 
                                   visited=visited, 
                                   current_direction=current_direction,
                                   direction_name=directions[current_direction])
            
            time.sleep(0.1)
        
        success = current_cell == end_cell
        result = {
            'algorithm': 'Left Wall Following',
            'success': success,
            'steps': step_count,
            'visited_count': len(visited),
            'final_position': current_cell
        }
        
        # Final comprehensive log
        self._comprehensive_log(step_count, "LEFT_WALL_FOLLOWING-FINAL", current_cell, 
                               visited=visited, 
                               current_direction=current_direction,
                               direction_name=directions[current_direction],
                               result=result)
        
        self.logger.info(f"Left Wall Following Algorithm Completed: Success={success}, Steps={step_count}, Visited={len(visited)}")
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
        self.logger.info(f"Smart Wall Following Algorithm Starting: {start_cell} -> {end_cell}")
        
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
        
        # Initial comprehensive log
        self._comprehensive_log(0, "SMART_WALL_FOLLOWING", current_cell, 
                               visited=visited, 
                               wall_map=wall_map,
                               path_taken=path_taken,
                               current_direction=current_direction,
                               direction_name=directions[current_direction])
        
        while current_cell != end_cell and step_count < max_steps:
            step_count += 1
            visited.add(current_cell)
            
            if step_count % 20 == 0:
                self.visualizer.print_progress(visited, current_cell, step_count)
            
            print(f"\n--- Step {step_count} ---")
            print(f"At {current_cell}, facing {directions[current_direction]}")
            self.logger.info(f"Smart Wall Following Step {step_count}: At {current_cell}, facing {directions[current_direction]}")
            
            # Update wall map with sensor readings
            self._update_wall_map(current_cell, wall_map)
            
            # Choose next direction using smart logic
            next_direction, next_cell = self._choose_smart_direction(
                current_cell, current_direction, wall_map, visited, directions, direction_vectors
            )
            
            if next_cell is None:
                print("No valid moves available!")
                self.logger.warning("No valid moves available!")
                break
            
            print(f"Choosing to go {directions[next_direction]} to {next_cell}")
            self.logger.info(f"Choosing to go {directions[next_direction]} to {next_cell}")
            
            # Execute the move
            if self._execute_directional_move(directions[next_direction]):
                current_direction = next_direction
                current_cell = self.robot.get_current_cell() or next_cell
                path_taken.append(current_cell)
                print(f"Successfully moved to: {current_cell}")
                self.logger.info(f"Successfully moved to: {current_cell}")
            else:
                print(f"Failed to move {directions[next_direction]}")
                self.logger.warning(f"Failed to move {directions[next_direction]}")
                wall_map[(current_cell, next_cell)] = True
            
            # Comprehensive logging at each step
            self._comprehensive_log(step_count, "SMART_WALL_FOLLOWING", current_cell, 
                                   visited=visited, 
                                   wall_map=wall_map,
                                   path_taken=path_taken,
                                   current_direction=current_direction,
                                   direction_name=directions[current_direction])
        
        success = current_cell == end_cell
        result = {
            'algorithm': 'Smart Wall Following',
            'success': success,
            'steps': step_count,
            'visited_count': len(visited),
            'path_efficiency': len(visited)/step_count*100 if step_count > 0 else 0,
            'final_position': current_cell
        }
        
        # Final comprehensive log
        self._comprehensive_log(step_count, "SMART_WALL_FOLLOWING-FINAL", current_cell, 
                               visited=visited, 
                               wall_map=wall_map,
                               path_taken=path_taken,
                               current_direction=current_direction,
                               direction_name=directions[current_direction],
                               result=result)
        
        self.logger.info(f"Smart Wall Following Algorithm Completed: Success={success}, Steps={step_count}, Visited={len(visited)}")
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
