"""
Wall-Following Maze Explorer with Path Mapping and Metric Logging

This Python controller runs a left-hand wall-following algorithm for a Webots e-puck robot.
It uses 8 proximity sensors for obstacle detection and GPS to track position.

The robot explores a 12×12 maze starting from START_CELL and aims to reach END_CELL.
During navigation, it builds a 2D matrix map of the environment where:

    - '?' represents unknown cells (unexplored),
    - '0' represents visited cells.

Key metrics captured:
    - Total time to complete the maze (seconds).
    - Total control steps (iterations).
    - Total number of unique cells visited.

These metrics are useful for comparing efficiency and coverage between different algorithms
(e.g., DFS vs wall-following), assessing path optimality, and supporting report analysis.

A visual marker is dropped every 10 steps in the Webots environment to aid visual tracking.
"""

from controller import Supervisor
import math

# === CONSTANTS ===
MAX_SPEED = 6.28
PROX_THRESHOLD = 80
WHEEL_RADIUS = 0.0205
WHEEL_DISTANCE = 0.053

MAZE_SIZE = 12
ARENA_SIZE = 3.0
CELL_SIZE = ARENA_SIZE / MAZE_SIZE
STOP_RADIUS = 0.1

START_CELL = (0, 6)
END_CELL = (11, 6)

# === POSITION CONVERSION UTILITIES ===
def cellToWorld(row, col):
    """Convert maze cell (row, col) to Webots world coordinates (x, y, z)."""
    x = -1.5 + (col + 0.5) * CELL_SIZE
    y = -1.5 + (row + 0.5) * CELL_SIZE
    z = 0.0
    return x, y, z

def worldToCell(x, y):
    """Convert Webots world coordinates (x, y) to maze cell (row, col)."""
    col = int((x + 1.5) / CELL_SIZE)
    row = int((y + 1.5) / CELL_SIZE)
    return row, col

# === MARKER UTILITY ===
def dropMarker(supervisor, position, radius=0.01, color=[1, 0, 0]):
    """Drop a coloured marker sphere at the given position in Webots."""
    childrenField = supervisor.getRoot().getField('children')
    protoStr = (
        f'Transform {{ translation {position[0]} {position[1]} {position[2]} '
        'children [ '
        'Shape { '
        'appearance Appearance { material Material { diffuseColor ' +
        f'{color[0]} {color[1]} {color[2]}' +
        ' } } '
        f'geometry Sphere {{ radius {radius} }} '
        '} ] }'
    )
    childrenField.importMFNodeFromString(-1, protoStr)

# === MATRIX OUTPUT ===
def saveMap(matrix, stepCount, visitedCount, durationSeconds, filename="wall_map.txt"):
    """Write the final maze matrix and navigation metrics to a file."""
    with open(filename, "w") as f:
        f.write("# Wall follower run summary\n")
        f.write(f"Total steps: {stepCount}\n")
        f.write(f"Cells visited: {visitedCount}\n")
        f.write(f"Run time: {durationSeconds:.2f} seconds\n\n")
        f.write("Maze map:\n")
        for row in matrix:
            f.write(" ".join(row) + "\n")

# === MAIN CONTROLLER ===
def runWallFollower(robot):
    """Main control loop for wall-following navigation."""
    timestep = int(robot.getBasicTimeStep())

    # Devices
    leftMotor = robot.getDevice('left wheel motor')
    rightMotor = robot.getDevice('right wheel motor')
    leftMotor.setPosition(float('inf'))
    rightMotor.setPosition(float('inf'))
    leftMotor.setVelocity(0.0)
    rightMotor.setVelocity(0.0)

    gps = robot.getDevice('gps')
    gps.enable(timestep)

    proxSensors = [robot.getDevice(f'ps{i}') for i in range(8)]
    for sensor in proxSensors:
        sensor.enable(timestep)

    # Initialise map and tracking
    mazeMap = [['?' for _ in range(MAZE_SIZE)] for _ in range(MAZE_SIZE)]
    visitedCells = set()
    stepCount = 0
    startTime = robot.getTime()

    print(f"Start: {START_CELL} -> {cellToWorld(*START_CELL)}")
    print(f"End:   {END_CELL} -> {cellToWorld(*END_CELL)}")

    while robot.step(timestep) != -1:
        x, y, z = gps.getValues()
        row, col = worldToCell(x, y)

        if 0 <= row < MAZE_SIZE and 0 <= col < MAZE_SIZE:
            mazeMap[row][col] = '0'
            visitedCells.add((row, col))

        if stepCount % 10 == 0:
            dropMarker(robot, (x, y, z + 0.01), color=[1, 0, 0])  # Red marker

        stepCount += 1

        # Sensor readings
        psValues = [sensor.getValue() for sensor in proxSensors]
        frontWall = psValues[7] > PROX_THRESHOLD or psValues[0] > PROX_THRESHOLD
        leftWall = psValues[5] > PROX_THRESHOLD

        # Wall-following logic
        if frontWall:
            leftSpeed = MAX_SPEED
            rightSpeed = -MAX_SPEED / 2
        elif leftWall:
            leftSpeed = MAX_SPEED
            rightSpeed = MAX_SPEED
        else:
            leftSpeed = MAX_SPEED / 4
            rightSpeed = MAX_SPEED

        # Goal check
        goalX, goalY, _ = cellToWorld(*END_CELL)
        distanceToGoal = math.hypot(x - goalX, y - goalY)
        print(f"Step {stepCount}: Cell ({row},{col}) | Distance to goal: {distanceToGoal:.2f}")

        if distanceToGoal < STOP_RADIUS:
            leftMotor.setVelocity(0.0)
            rightMotor.setVelocity(0.0)
            print(f"Goal reached at ({x:.2f}, {y:.2f}).")
            break

        # Apply motor commands
        leftMotor.setVelocity(leftSpeed)
        rightMotor.setVelocity(rightSpeed)

    # Completion metrics
    endTime = robot.getTime()
    durationSeconds = endTime - startTime
    visitedCount = len(visitedCells)

    print(f"\nRun complete in {durationSeconds:.2f} seconds")
    print(f"Steps taken: {stepCount}")
    print(f"Cells visited: {visitedCount}\n")

    # Save map and metrics
    saveMap(mazeMap, stepCount, visitedCount, durationSeconds)

# === ENTRY POINT ===
if __name__ == "__main__":
    robot = Supervisor()
    runWallFollower(robot)
