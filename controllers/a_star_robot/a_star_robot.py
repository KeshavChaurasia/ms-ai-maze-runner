"""
Fixed A* Pathfinding Maze Explorer for Webots E-puck

Key fixes:
1. Proper orientation and movement control
2. Better obstacle detection and avoidance
3. Robust pathfinding with correct cell transitions
4. Improved fallback strategies
"""

from controller import Supervisor
import math
import heapq
from typing import List, Tuple, Set, Optional

# === CONSTANTS ===
MAX_SPEED = 6.28
PROX_THRESHOLD = 1000  # Lowered threshold for better obstacle detection
WHEEL_RADIUS = 0.0205
WHEEL_DISTANCE = 0.053

MAZE_SIZE = 12
ARENA_SIZE = 3.0
CELL_SIZE = ARENA_SIZE / MAZE_SIZE
STOP_RADIUS = 0.1

START_CELL = (0, 6)
END_CELL = (11, 6)

# Direction vectors for 4-connected grid
DIRECTIONS = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # N, S, W, E

# === POSITION CONVERSION UTILITIES ===
def cell_to_world(row: int, col: int) -> Tuple[float, float, float]:
    """Convert maze cell (row, col) to Webots world coordinates (x, y, z)."""
    x = -1.5 + (col + 0.5) * CELL_SIZE
    y = -1.5 + (row + 0.5) * CELL_SIZE
    z = 0.0
    return x, y, z

def world_to_cell(x: float, y: float) -> Tuple[int, int]:
    """Convert Webots world coordinates (x, y) to maze cell (row, col)."""
    col = int((x + 1.5) / CELL_SIZE)
    row = int((y + 1.5) / CELL_SIZE)
    # Clamp to valid range
    row = max(0, min(MAZE_SIZE - 1, row))
    col = max(0, min(MAZE_SIZE - 1, col))
    return row, col

# === A* ALGORITHM ===
def manhattan_distance(pos1: Tuple[int, int], pos2: Tuple[int, int]) -> int:
    """Calculate Manhattan distance between two positions."""
    return abs(pos1[0] - pos2[0]) + abs(pos1[1] - pos2[1])

def is_valid_cell(row: int, col: int) -> bool:
    """Check if a cell is within maze bounds."""
    return 0 <= row < MAZE_SIZE and 0 <= col < MAZE_SIZE

def get_neighbors(pos: Tuple[int, int], maze_map: List[List[str]]) -> List[Tuple[int, int]]:
    """Get valid neighboring cells that are not walls."""
    row, col = pos
    neighbors = []
    
    for dr, dc in DIRECTIONS:
        new_row, new_col = row + dr, col + dc
        if (is_valid_cell(new_row, new_col) and 
            maze_map[new_row][new_col] != '1'):  # Only walls ('1') are impassable
            neighbors.append((new_row, new_col))
    
    return neighbors

def a_star_pathfinding(start: Tuple[int, int], goal: Tuple[int, int], 
                      maze_map: List[List[str]]) -> Optional[List[Tuple[int, int]]]:
    """A* pathfinding algorithm that treats unknown cells as passable."""
    if not is_valid_cell(start[0], start[1]) or not is_valid_cell(goal[0], goal[1]):
        return None
    
    # Priority queue: (f_score, cell, g_score, path)
    open_set = [(0, start, 0, [start])]
    closed_set = set()
    g_scores = {start: 0}
    
    while open_set:
        f_score, current, g_score, path = heapq.heappop(open_set)
        
        if current in closed_set:
            continue
            
        if current == goal:
            return path
            
        closed_set.add(current)
        
        for neighbor in get_neighbors(current, maze_map):
            if neighbor in closed_set:
                continue
                
            new_g_score = g_score + 1
            
            if neighbor not in g_scores or new_g_score < g_scores[neighbor]:
                g_scores[neighbor] = new_g_score
                new_f_score = new_g_score + manhattan_distance(neighbor, goal)
                new_path = path + [neighbor]
                
                heapq.heappush(open_set, (new_f_score, neighbor, new_g_score, new_path))
    
    return None

# === IMPROVED ROBOT MOVEMENT ===
class RobotController:
    def __init__(self, robot, left_motor, right_motor, gps):
        self.robot = robot
        self.left_motor = left_motor
        self.right_motor = right_motor
        self.gps = gps
        self.target_angle = 0.0
        self.last_position = None
        self.current_bearing = 0.0  # Track bearing using movement
        
    def get_bearing(self) -> float:
        """Get current robot bearing using movement tracking."""
        current_pos = self.gps.getValues()
        
        if self.last_position is not None:
            dx = current_pos[0] - self.last_position[0]
            dy = current_pos[1] - self.last_position[1]
            
            # Only update bearing if robot has moved significantly
            if math.hypot(dx, dy) > 0.001:
                self.current_bearing = math.atan2(dy, dx)
        
        self.last_position = current_pos
        return self.current_bearing
        
    def normalize_angle(self, angle: float) -> float:
        """Normalize angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
        
    def turn_to_direction(self, target_cell: Tuple[int, int], current_cell: Tuple[int, int], 
                         ps_values: List[int], max_steps: int = 50) -> bool:
        """Turn robot to face target cell using sensor-based approach."""
        dr = target_cell[0] - current_cell[0]
        dc = target_cell[1] - current_cell[1]
        
        # Determine required turn based on direction
        if dr > 0:  # Need to go north
            # Check if front sensors are clear (robot facing north)
            if ps_values[0] < PROX_THRESHOLD and ps_values[7] < PROX_THRESHOLD:
                return True  # Already facing north
            else:
                # Turn to face north
                self.left_motor.setVelocity(-MAX_SPEED * 0.3)
                self.right_motor.setVelocity(MAX_SPEED * 0.3)
                return False
        elif dr < 0:  # Need to go south
            # Check if back sensor indicates we're facing south
            if ps_values[4] < PROX_THRESHOLD:
                return True  # Can move south
            else:
                # Turn around to face south
                self.left_motor.setVelocity(-MAX_SPEED * 0.3)
                self.right_motor.setVelocity(MAX_SPEED * 0.3)
                return False
        elif dc > 0:  # Need to go east
            # Check if right side is clear
            if ps_values[2] < PROX_THRESHOLD:
                return True  # Can move east
            else:
                # Turn to face east
                self.left_motor.setVelocity(MAX_SPEED * 0.3)
                self.right_motor.setVelocity(-MAX_SPEED * 0.3)
                return False
        elif dc < 0:  # Need to go west
            # Check if left side is clear
            if ps_values[5] < PROX_THRESHOLD:
                return True  # Can move west
            else:
                # Turn to face west
                self.left_motor.setVelocity(-MAX_SPEED * 0.3)
                self.right_motor.setVelocity(MAX_SPEED * 0.3)
                return False
        
        return True  # No movement needed
        
    def move_towards_target(self, target_cell: Tuple[int, int], current_cell: Tuple[int, int], 
                           ps_values: List[int]) -> bool:
        """Move towards target cell with obstacle avoidance."""
        dr = target_cell[0] - current_cell[0]
        dc = target_cell[1] - current_cell[1]
        
        if dr > 0:  # Move north
            if ps_values[0] > PROX_THRESHOLD or ps_values[7] > PROX_THRESHOLD:
                return False  # Obstacle ahead
            self.left_motor.setVelocity(MAX_SPEED * 0.8)
            self.right_motor.setVelocity(MAX_SPEED * 0.8)
            
        elif dr < 0:  # Move south
            if ps_values[4] > PROX_THRESHOLD:
                return False  # Obstacle behind
            # Move backward
            self.left_motor.setVelocity(-MAX_SPEED * 0.8)
            self.right_motor.setVelocity(-MAX_SPEED * 0.8)
            
        elif dc > 0:  # Move east
            if ps_values[2] > PROX_THRESHOLD:
                return False  # Obstacle on right
            # Turn right while moving forward
            self.left_motor.setVelocity(MAX_SPEED * 0.9)
            self.right_motor.setVelocity(MAX_SPEED * 0.6)
            
        elif dc < 0:  # Move west
            if ps_values[5] > PROX_THRESHOLD:
                return False  # Obstacle on left
            # Turn left while moving forward
            self.left_motor.setVelocity(MAX_SPEED * 0.6)
            self.right_motor.setVelocity(MAX_SPEED * 0.9)
            
        return True
        
    def stop(self) -> None:
        """Stop the robot."""
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)

def get_target_angle(current_cell: Tuple[int, int], target_cell: Tuple[int, int]) -> float:
    """Calculate target angle to move from current to target cell."""
    dr = target_cell[0] - current_cell[0]
    dc = target_cell[1] - current_cell[1]
    
    if dr > 0:  # Move north
        return math.pi / 2
    elif dr < 0:  # Move south
        return -math.pi / 2
    elif dc > 0:  # Move east
        return 0.0
    elif dc < 0:  # Move west
        return math.pi
    else:
        return 0.0  # No movement needed

# === SIMPLIFIED OBSTACLE DETECTION ===
def detect_obstacles_simple(ps_values: List[int], current_cell: Tuple[int, int], 
                           maze_map: List[List[str]]) -> Set[Tuple[int, int]]:
    """Simplified obstacle detection using proximity sensors."""
    detected_obstacles = set()
    row, col = current_cell
    
    # Front sensors (0, 7) - obstacle ahead (north)
    if ps_values[0] > PROX_THRESHOLD or ps_values[7] > PROX_THRESHOLD:
        north_cell = (row - 1, col)
        if is_valid_cell(north_cell[0], north_cell[1]):
            maze_map[north_cell[0]][north_cell[1]] = '1'
            detected_obstacles.add(north_cell)
    
    # Right sensor (2) - obstacle to the east
    if ps_values[2] > PROX_THRESHOLD:
        east_cell = (row, col + 1)
        if is_valid_cell(east_cell[0], east_cell[1]):
            maze_map[east_cell[0]][east_cell[1]] = '1'
            detected_obstacles.add(east_cell)
    
    # Back sensor (4) - obstacle behind (south)
    if ps_values[4] > PROX_THRESHOLD:
        south_cell = (row + 1, col)
        if is_valid_cell(south_cell[0], south_cell[1]):
            maze_map[south_cell[0]][south_cell[1]] = '1'
            detected_obstacles.add(south_cell)
    
    # Left sensor (5) - obstacle to the west
    if ps_values[5] > PROX_THRESHOLD:
        west_cell = (row, col - 1)
        if is_valid_cell(west_cell[0], west_cell[1]):
            maze_map[west_cell[0]][west_cell[1]] = '1'
            detected_obstacles.add(west_cell)
    
    return detected_obstacles

# === UTILITY FUNCTIONS ===
def drop_marker(robot, position: Tuple[float, float, float], 
                color: List[float] = [0, 1, 0], radius: float = 0.01):
    """Drop a colored marker at the given position."""
    try:
        children_field = robot.getRoot().getField('children')
        proto_str = (
            f'Transform {{ translation {position[0]} {position[1]} {position[2]} '
            'children [ '
            'Shape { '
            'appearance Appearance { material Material { diffuseColor ' + f'{color[0]} {color[1]} {color[2]}' + ' } } '
            f'geometry Sphere {{ radius {radius} }} '
            '} ] }'
        )
        children_field.importMFNodeFromString(-1, proto_str)
    except Exception as e:
        print(f"Warning: Could not drop marker: {e}")

def save_map(maze_map: List[List[str]], steps: int, visited: int, duration: float, path_length: int):
    """Save the maze map and metrics to a file."""
    filename = f"a_star_fixed_map_{steps}steps_{visited}cells_{duration:.1f}s.txt"
    
    try:
        with open(filename, 'w') as f:
            f.write(f"Fixed A* Algorithm Results\n")
            f.write(f"Steps taken: {steps}\n")
            f.write(f"Cells visited: {visited}\n")
            f.write(f"Duration: {duration:.2f} seconds\n")
            f.write(f"Path length: {path_length}\n")
            f.write(f"Start: {START_CELL}\n")
            f.write(f"End: {END_CELL}\n\n")
            f.write("Maze Map (0=free, 1=wall, ?=unknown, P=path):\n")
            
            for row in maze_map:
                f.write(' '.join(row) + '\n')
        print(f"Map saved to {filename}")
    except Exception as e:
        print(f"Warning: Could not save map: {e}")

# === MAIN CONTROLLER ===
def run_fixed_a_star_robot(robot):
    """Main control loop for fixed A* pathfinding navigation."""
    timestep = int(robot.getBasicTimeStep())
    
    # Initialize devices
    left_motor = robot.getDevice('left wheel motor')
    right_motor = robot.getDevice('right wheel motor')
    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)
    
    gps = robot.getDevice('gps')
    gps.enable(timestep)
    
    prox_sensors = [robot.getDevice(f'ps{i}') for i in range(8)]
    for sensor in prox_sensors:
        sensor.enable(timestep)
    
    # Initialize robot controller (without compass)
    controller = RobotController(robot, left_motor, right_motor, gps)
    
    # Initialize maze map and tracking
    maze_map = [['?' for _ in range(MAZE_SIZE)] for _ in range(MAZE_SIZE)]
    visited_cells = set()
    step_count = 0
    start_time = robot.getTime()
    
    print(f"Fixed A* Robot starting from {START_CELL} to {END_CELL}")
    
    # Initialize start and end cells as free
    maze_map[START_CELL[0]][START_CELL[1]] = '0'
    maze_map[END_CELL[0]][END_CELL[1]] = '0'
    
    # State machine variables
    current_path = []
    path_index = 0
    state = "PLANNING"  # States: PLANNING, TURNING, MOVING, STUCK
    turning_steps = 0
    replan_count = 0
    stuck_counter = 0
    last_position = None
    movement_timer = 0
    
    # Main control loop
    while robot.step(timestep) != -1:
        # Get current state
        x, y, z = gps.getValues()
        current_cell = world_to_cell(x, y)
        current_pos = (x, y)
        
        # Check if robot is stuck (not moving)
        if last_position is not None:
            distance_moved = math.hypot(current_pos[0] - last_position[0], 
                                      current_pos[1] - last_position[1])
            if distance_moved < 0.001:  # Very small movement threshold
                stuck_counter += 1
            else:
                stuck_counter = 0
        last_position = current_pos
        
        # Update visited cells and map
        if is_valid_cell(current_cell[0], current_cell[1]):
            maze_map[current_cell[0]][current_cell[1]] = '0'
            visited_cells.add(current_cell)
        
        # Get sensor readings and detect obstacles
        ps_values = [sensor.getValue() for sensor in prox_sensors]
        detected_obstacles = detect_obstacles_simple(ps_values, current_cell, maze_map)
        
        # Drop markers for visualization
        if step_count % 30 == 0:
            drop_marker(robot, (x, y, z + 0.01), color=[0, 1, 0])  # Green for path
        
        # Drop obstacle markers
        for obstacle in detected_obstacles:
            obs_x, obs_y, _ = cell_to_world(*obstacle)
            drop_marker(robot, (obs_x, obs_y, z + 0.02), color=[1, 0, 0], radius=0.015)
        
        step_count += 1
        
        # Check if we reached the goal
        goal_x, goal_y, _ = cell_to_world(*END_CELL)
        distance_to_goal = math.hypot(x - goal_x, y - goal_y)
        
        if distance_to_goal < STOP_RADIUS:
            controller.stop()
            print(f"Goal reached at ({x:.2f}, {y:.2f})!")
            break
        
        # State machine logic
        if state == "PLANNING" or detected_obstacles or stuck_counter > 100:
            # Need to plan or replan
            if stuck_counter > 100:
                print("Robot appears stuck, replanning...")
                stuck_counter = 0
                
            current_path = a_star_pathfinding(current_cell, END_CELL, maze_map)
            
            if current_path and len(current_path) > 1:
                path_index = 1  # Skip current cell
                state = "TURNING"
                turning_steps = 0
                replan_count += 1
                print(f"New path planned: {current_path[:5]}{'...' if len(current_path) > 5 else ''}")
            else:
                print("No path found, using fallback behavior")
                state = "STUCK"
                
        elif state == "TURNING":
            if path_index < len(current_path):
                target_cell = current_path[path_index]
                
                # Try to turn towards target for a limited time
                if controller.turn_to_direction(target_cell, current_cell, ps_values):
                    state = "MOVING"
                    movement_timer = 0
                    print(f"Ready to move to {target_cell}")
                else:
                    turning_steps += 1
                    if turning_steps > 30:  # Give up turning after 30 steps
                        state = "MOVING"
                        movement_timer = 0
                        print("Turning timeout, attempting movement anyway")
            else:
                state = "PLANNING"
                
        elif state == "MOVING":
            if path_index < len(current_path):
                target_cell = current_path[path_index]
                target_x, target_y, _ = cell_to_world(*target_cell)
                distance_to_target = math.hypot(x - target_x, y - target_y)
                
                if distance_to_target < CELL_SIZE * 0.4:
                    # Reached target cell
                    controller.stop()
                    path_index += 1
                    if path_index < len(current_path):
                        state = "TURNING"
                        turning_steps = 0
                        print(f"Reached {target_cell}, turning to next target")
                    else:
                        state = "PLANNING"
                        print("Path completed, planning next segment")
                else:
                    # Try to move towards target
                    if controller.move_towards_target(target_cell, current_cell, ps_values):
                        movement_timer += 1
                        # If moving for too long without progress, replan
                        if movement_timer > 100:
                            print("Movement timeout, replanning")
                            state = "PLANNING"
                            movement_timer = 0
                    else:
                        print("Movement blocked, replanning")
                        controller.stop()
                        state = "PLANNING"
            else:
                state = "PLANNING"
                
        elif state == "STUCK":
            # Fallback behavior: try to escape by turning
            print("Executing stuck recovery behavior")
            controller.left_motor.setVelocity(-MAX_SPEED * 0.3)
            controller.right_motor.setVelocity(MAX_SPEED * 0.3)
            
            # Turn for a short time then try planning again
            if step_count % 50 == 0:
                controller.stop()
                state = "PLANNING"
                stuck_counter = 0
        
        # Debug output
        if step_count % 100 == 0:
            print(f"Step {step_count}: State={state}, Cell={current_cell}, "
                  f"Goal distance={distance_to_goal:.2f}, Stuck counter={stuck_counter}")
    
    # Completion metrics
    end_time = robot.getTime()
    duration_seconds = end_time - start_time
    visited_count = len(visited_cells)
    path_length = len(current_path) if current_path else 0
    
    print(f"\nFixed A* run complete in {duration_seconds:.2f} seconds")
    print(f"Steps taken: {step_count}")
    print(f"Cells visited: {visited_count}")
    print(f"Final path length: {path_length}")
    print(f"Path replans: {replan_count}")
    
    # Save results
    save_map(maze_map, step_count, visited_count, duration_seconds, path_length)

# === ENTRY POINT ===
if __name__ == "__main__":
    robot = Supervisor()
    run_fixed_a_star_robot(robot)