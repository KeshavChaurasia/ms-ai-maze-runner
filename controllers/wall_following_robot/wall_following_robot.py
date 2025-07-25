# x = red
# y = green
# z = blue
from controller import Supervisor
import math

# Constants
MAX_SPEED = 6.28
PROX_THRESHOLD = 80  # Adjust as needed for your environment
WHEEL_RADIUS = 0.0205  # meters (e-puck standard)
WHEEL_DISTANCE = 0.053  # meters (distance between wheels)

# Maze and arena parameters
MAZE_SIZE = 12
ARENA_SIZE = 3.0  # meters
CELL_SIZE = ARENA_SIZE / MAZE_SIZE # 0.25

# Start and end cell (row, col)
START_CELL = (0, 6)  # bottom row, 7th column (0-indexed)
END_CELL = (11, 6)   # top row, 7th column (0-indexed)

# Corrected: x = -1.5 + (col + 0.5) * cell_size
#            y = -1.5 + (row + 0.5) * cell_size
#            z = 0.0

def cell_to_world(row, col):
    cell_size = ARENA_SIZE / MAZE_SIZE
    x = -1.5 + (col + 0.5) * cell_size
    y = -1.5 + (row + 0.5) * cell_size
    z = 0.0
    return x, y, z

# New function: world coordinates to maze cell (row, col)
def world_to_cell(x, y):
    cell_size = ARENA_SIZE / MAZE_SIZE
    col = int((x + 1.5) / cell_size)
    row = int((y + 1.5) / cell_size)
    return row, col

START_POS = cell_to_world(*START_CELL)
END_POS = cell_to_world(*END_CELL)
STOP_RADIUS = 0.1  # meters

print(f"START_CELL: {START_CELL} -> START_POS: {START_POS}")
print(f"END_CELL: {END_CELL} -> END_POS: {END_POS}")
print(f"START_POS as cell: {world_to_cell(START_POS[0], START_POS[1])}")
print(f"END_POS as cell: {world_to_cell(END_POS[0], END_POS[1])}")

# Wall following logic: left wall following
for i in range(12):
    for j in range(12):
        pos = cell_to_world(i, j)
        print(f"cell ({i},{j}) = {pos} -> cell: {world_to_cell(pos[0], pos[1])}")
        
# --- Add marker dropping logic ---
def drop_marker(supervisor, position, radius=0.01, color=[1,0,0]):
    """Drop a small colored sphere at the given position using Supervisor API."""
    children_field = supervisor.getRoot().getField('children')
    proto_str = (
        f'Transform {{ translation {position[0]} {position[1]} {position[2]} '
        'children [ '
        'Shape { '
        'appearance Appearance { material Material { diffuseColor ' + f'{color[0]} {color[1]} {color[2]}' + ' } } '
        f'geometry Sphere {{ radius {radius} }} '
        '} ] }'
    )
    children_field.importMFNodeFromString(-1, proto_str)

def run_wall_follower(robot):
    timestep = int(robot.getBasicTimeStep())

    # Motors
    left_motor = robot.getDevice('left wheel motor')
    right_motor = robot.getDevice('right wheel motor')
    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)

    # Proximity sensors
    prox_sensors = []
    for i in range(8):
        sensor = robot.getDevice(f'ps{i}')
        sensor.enable(timestep)
        prox_sensors.append(sensor)

    # Encoders
    left_encoder = robot.getDevice('left wheel sensor')
    right_encoder = robot.getDevice('right wheel sensor')
    left_encoder.enable(timestep)
    right_encoder.enable(timestep)

    # Try to get GPS device for ground truth (optional)
    gps = robot.getDevice('gps')
    gps.enable(timestep)

    print(f"Start position (odometry): x={START_POS[0]:.3f}, y={START_POS[1]:.3f}")
    print(f"End position: x={END_POS[0]:.3f}, y={END_POS[1]:.3f}")

    step_count = 0  # Add step counter for marker dropping
    while robot.step(timestep) != -1:
        # Get ground truth position from GPS if available
        gt_translation = gps.getValues()
        x, y, z = gt_translation[0], gt_translation[1], gt_translation[2]
        # Drop a marker every 10 steps
        if step_count % 10 == 0:
            drop_marker(robot, (x, y, z+0.01))  # Slightly above ground
        step_count += 1
        print(f"Ground truth (GPS): x={x:.3f}, y={y:.3f}, z={z:.3f}")

        # Read proximity sensors
        ps_values = [sensor.getValue() for sensor in prox_sensors]
        left_wall = ps_values[5] > PROX_THRESHOLD
        front_wall = ps_values[7] > PROX_THRESHOLD or ps_values[0] > PROX_THRESHOLD

        # Wall following logic
        if front_wall:
            left_speed = MAX_SPEED
            right_speed = -MAX_SPEED / 2
        elif left_wall:
            left_speed = MAX_SPEED
            right_speed = MAX_SPEED
        else:
            left_speed = MAX_SPEED / 4
            right_speed = MAX_SPEED

        print(f"Estimated Position (GPS): x={x:.3f}, y={y:.3f}")
        print(f"Estimated Position as cell: {world_to_cell(x, y)}")
        print(f"Target Position: x={END_POS[0]:.3f}, y={END_POS[1]:.3f}")
        print(f"Target Position as cell: {world_to_cell(END_POS[0], END_POS[1])}")
        dist_to_goal = math.hypot(x - END_POS[0], y - END_POS[1])
        print(f"Distance to goal: {dist_to_goal:.3f} (Stop radius: {STOP_RADIUS})")

        # Stop if within STOP_RADIUS of END_POS
        if dist_to_goal < STOP_RADIUS:
            left_motor.setVelocity(0.0)
            right_motor.setVelocity(0.0)
            print(f"Robot reached the end cell at x={x:.3f}, y={y:.3f}. Stopping.")
            break

        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)

if __name__ == "__main__":
    robot = Supervisor()
    run_wall_follower(robot)
