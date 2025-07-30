"""
Bug 2 Maze Explorer with Map and Metrics

This controller implements the Bug 2 algorithm, a hybrid approach that combines
goal-directed motion with reactive wall-following.

The robot moves directly toward the goal until it encounters an obstacle,
at which point it follows the wall using a left-hand rule. When it regains
a clear path to the goal and has made progress, it resumes goal tracking.

Key metrics captured:
- Total number of control steps (iterations)
- Number of unique maze cells visited
- Total run time from START to END

Purpose:
Provides a balance between intelligent goal-seeking and local obstacle avoidance,
offering improved efficiency compared to random or pure wall-following strategies.
"""

from controller import Supervisor
import math

# === CONSTANTS ===
MAX_SPEED = 6.28
PROX_THRESHOLD = 80
WHEEL_RADIUS = 0.0205
MAZE_SIZE = 12
ARENA_SIZE = 3.0
CELL_SIZE = ARENA_SIZE / MAZE_SIZE
STOP_RADIUS = 0.1

START_CELL = (0, 6)
END_CELL = (11, 6)

# === POSITION UTILS ===
def cellToWorld(row, col):
    x = -1.5 + (col + 0.5) * CELL_SIZE
    y = -1.5 + (row + 0.5) * CELL_SIZE
    return x, y, 0.01

def worldToCell(x, y):
    col = int((x + 1.5) / CELL_SIZE)
    row = int((y + 1.5) / CELL_SIZE)
    return row, col

def distance(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])

# === MARKERS ===
def dropMarker(supervisor, pos, radius=0.01, color=[1, 0, 0]):
    children = supervisor.getRoot().getField('children')
    sphere = (
        f'Transform {{ translation {pos[0]} {pos[1]} {pos[2]} '
        'children [ Shape { appearance Appearance { material Material { diffuseColor ' +
        f'{color[0]} {color[1]} {color[2]}' +
        ' } } geometry Sphere { radius ' + f'{radius}' + ' } } ] }}'
    )
    children.importMFNodeFromString(-1, sphere)

# === OUTPUT ===
def saveMap(matrix, steps, visited, duration, file="bug2_map.txt"):
    with open(file, 'w') as f:
        f.write("# Bug 2 Maze Explorer Output\n")
        f.write(f"Total steps: {steps}\n")
        f.write(f"Cells visited: {visited}\n")
        f.write(f"Duration: {duration:.2f} seconds\n\n")
        for row in matrix:
            f.write(" ".join(row) + "\n")

# === VELOCITY CLAMP ===
def clamp_speed(speed):
    return max(-MAX_SPEED, min(MAX_SPEED, speed))

# === STARTUP MANOEUVRE ===
def startupTurnAndMove(robot, leftMotor, rightMotor, timestep):
    print("Commencing run...")

    # 1. Turn right (~90 degrees)
    turnDuration = int(1000 * 0.9)  # tweak as needed
    leftMotor.setVelocity(MAX_SPEED)
    rightMotor.setVelocity(-MAX_SPEED)
    for _ in range(turnDuration // timestep):
        robot.step(timestep)

    # 2. Move forward one cell
    moveDuration = int((CELL_SIZE / (WHEEL_RADIUS * MAX_SPEED)) * 1000 * 0.95)
    leftMotor.setVelocity(MAX_SPEED)
    rightMotor.setVelocity(MAX_SPEED)
    for _ in range(moveDuration // timestep):
        robot.step(timestep)

    # 3. Stop
    leftMotor.setVelocity(0)
    rightMotor.setVelocity(0)
    for _ in range(5):
        robot.step(timestep)

# === CONTROL LOGIC ===
def runBug2(robot):
    timestep = int(robot.getBasicTimeStep())

    leftMotor = robot.getDevice('left wheel motor')
    rightMotor = robot.getDevice('right wheel motor')
    leftMotor.setPosition(float('inf'))
    rightMotor.setPosition(float('inf'))

    gps = robot.getDevice('gps')
    gps.enable(timestep)

    prox = [robot.getDevice(f'ps{i}') for i in range(8)]
    for p in prox:
        p.enable(timestep)

    # Map setup
    mazeMap = [['?' for _ in range(MAZE_SIZE)] for _ in range(MAZE_SIZE)]
    visitedCells = set()
    steps = 0
    startTime = robot.getTime()

    # Initial reposition
    startupTurnAndMove(robot, leftMotor, rightMotor, timestep)

    # Wall-following state
    wallMode = False
    lastHitDistance = float('inf')

    while robot.step(timestep) != -1:
        x, y, _ = gps.getValues()
        row, col = worldToCell(x, y)

        if 0 <= row < MAZE_SIZE and 0 <= col < MAZE_SIZE:
            mazeMap[row][col] = '0'
            visitedCells.add((row, col))

        if steps % 10 == 0:
            dropMarker(robot, (x, y, 0.01))

        # Check if at goal
        gx, gy, _ = cellToWorld(*END_CELL)
        distToGoal = math.hypot(x - gx, y - gy)
        if distToGoal < STOP_RADIUS:
            leftMotor.setVelocity(0.0)
            rightMotor.setVelocity(0.0)
            print("Goal reached!")
            break

        # Read sensors
        vals = [p.getValue() for p in prox]
        front = vals[0] > PROX_THRESHOLD or vals[7] > PROX_THRESHOLD
        left = vals[5] > PROX_THRESHOLD
        right = vals[1] > PROX_THRESHOLD

        if not wallMode:
            # Try to go straight to goal
            angle = math.atan2(gy - y, gx - x)
            forwardSpeed = MAX_SPEED * 0.75
            turnSpeed = -angle * 2.0

            leftSpeed = clamp_speed(forwardSpeed + turnSpeed)
            rightSpeed = clamp_speed(forwardSpeed - turnSpeed)

            if front:
                wallMode = True
                lastHitDistance = distToGoal
            else:
                leftMotor.setVelocity(leftSpeed)
                rightMotor.setVelocity(rightSpeed)
        else:
            # Wall-following (left-hand)
            if not front and not left:
                leftMotor.setVelocity(clamp_speed(MAX_SPEED * 0.25))
                rightMotor.setVelocity(clamp_speed(MAX_SPEED))
            elif front:
                leftMotor.setVelocity(clamp_speed(MAX_SPEED))
                rightMotor.setVelocity(clamp_speed(-MAX_SPEED / 2))
            else:
                leftMotor.setVelocity(clamp_speed(MAX_SPEED))
                rightMotor.setVelocity(clamp_speed(MAX_SPEED))

            # Exit wall-following if progress made
            if not front and not left and distToGoal < lastHitDistance - 0.05:
                wallMode = False

        steps += 1

    duration = robot.getTime() - startTime
    saveMap(mazeMap, steps, len(visitedCells), duration)
    print(f"Run complete: {steps} steps, {len(visitedCells)} cells, {duration:.2f}s")

# === ENTRY POINT ===
if __name__ == "__main__":
    robot = Supervisor()
    runBug2(robot)
