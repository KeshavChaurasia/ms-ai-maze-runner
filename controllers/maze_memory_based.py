from controller import Robot as Bot, Camera as Cam, CameraRecognitionObject as RecObj, InertialUnit as IMU, DistanceSensor as DSensor, PositionSensor as PSensor
import math

bot = Bot()

time_step = int(bot.getBasicTimeStep())

# Distance sensors
dist_front = bot.getDevice('front_ds')
dist_left = bot.getDevice('left_ds')
dist_right = bot.getDevice('right_ds')
for sensor in [dist_front, dist_left, dist_right]:
    sensor.enable(time_step)

# Wheels
enc_left = bot.getDevice('left wheel sensor')
enc_right = bot.getDevice('right wheel sensor')
for encoder in [enc_left, enc_right]:
    encoder.enable(time_step)

vision = bot.getDevice('camera1')
vision.enable(time_step)
vision.recognitionEnable(time_step)

# Inertial measurement unit
inertial = bot.getDevice('inertial unit')
inertial.enable(time_step)

# Motors
motor_left = bot.getDevice('left wheel motor')
motor_right = bot.getDevice('right wheel motor')
for motor in [motor_left, motor_right]:
    motor.setPosition(float('inf'))
    motor.setVelocity(0)

# Robot physical constants
wheel_r = 0.8
circum = 2 * math.pi * wheel_r
enc_conv = circum / (2 * math.pi)
vel_max = 4
wheel_base = 2.28
half_base = wheel_base / 2

# Grid details
grid_map_index = [(r, c) for r in range(4) for c in range(4)]

# State
heading = "North"
current_pos = [15.0, -15.0]
encoder_last = [0, 0]
pose = [15.0, -15.0, 16, 180]
maze = [[0, 1, 1, 0, 0], [0, 0, 1, 0, 0], [0, 0, 1, 0, 0], [0, 0, 1, 1, 0],
        [0, 1, 0, 0, 0], [0, 0, 0, 0, 0], [0, 0, 0, 0, 0], [0, 0, 0, 1, 0],
        [0, 1, 0, 0, 0], [0, 0, 0, 0, 0], [0, 0, 0, 0, 0], [0, 0, 0, 1, 0],
        [0, 1, 0, 0, 1], [0, 0, 0, 0, 1], [0, 0, 0, 0, 1], [0, 0, 0, 1, 1]]

visited_map = ['.'] * 16
maze[pose[2]-1][0] = 1
visited_map[pose[2]-1] = 'X'

def convert_m_to_in(m):
    return m * 39.3701

def enc_to_in(val):
    return abs(val * wheel_r)

def get_encoder_readings():
    return enc_to_in(enc_left.getValue()), enc_to_in(enc_right.getValue())

def get_distance_readings():
    return convert_m_to_in(dist_left.getValue()), convert_m_to_in(dist_front.getValue()), convert_m_to_in(dist_right.getValue())

def time_to_travel(dist, speed):
    return dist / speed

def stop_all():
    motor_left.setVelocity(0)
    motor_right.setVelocity(0)

def compute_turn_speed(deg, duration):
    arc = half_base * 2 * math.pi * (deg / 360.0)
    lin_vel = arc / duration
    return lin_vel / wheel_r, -lin_vel / wheel_r

def update_orientation():
    global heading
    angle = math.degrees(inertial.getRollPitchYaw()[2])
    if -180 <= angle <= -135 or 135 <= angle <= 180:
        heading = "North"
    elif -135 < angle <= -45:
        heading = "West"
    elif 45 <= angle <= 135:
        heading = "East"
    else:
        heading = "South"

def update_pose(new_x, new_y, new_cell, new_theta):
    global pose
    pose = [new_x, new_y, new_cell, new_theta]

def display_pose():
    print(f"Pose: {pose}")

def display_heading():
    print(f"Direction: {heading}, Cell: {pose[2]}")

def mark_cell(cell_index):
    maze[cell_index][0] = 1
    visited_map[cell_index] = 'X'

def display_visited():
    print("Visited:")
    for i in range(0, 16, 4):
        print(' '.join(visited_map[i:i+4]))

def rotate_bot(degrees, duration):
    left_speed, right_speed = compute_turn_speed(degrees, duration)
    end_time = bot.getTime() + duration
    while bot.step(time_step) != -1 and bot.getTime() < end_time:
        motor_left.setVelocity(left_speed)
        motor_right.setVelocity(right_speed)
    stop_all()

def turn_left():
    rotate_bot(-90, 1.5)

def turn_right():
    rotate_bot(90, 1.5)

def move_forward(distance_in):
    duration = time_to_travel(distance_in, vel_max)
    end_time = bot.getTime() + duration
    speed = vel_max / wheel_r
    while bot.step(time_step) != -1 and bot.getTime() < end_time:
        motor_left.setVelocity(speed)
        motor_right.setVelocity(speed)
    stop_all()

def explore_maze():
    while bot.step(time_step) != -1:
        update_orientation()
        display_pose()
        display_heading()
        display_visited()
        move_forward(10)
        mark_cell(pose[2]-1)
        turn_right()
        break

def main_loop():
    while bot.step(time_step) != -1:
        update_orientation()
        display_pose()
        display_heading()
        display_visited()
        explore_maze()
        break

if __name__ == "__main__":
    main_loop()
