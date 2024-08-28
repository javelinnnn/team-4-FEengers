import ioty.monitor
import time 
import machine
import mpu6050
import stepper_wheels
from ioty import pin
import math
from machine import UART

i2c0 = None
mpu6050_device = None
mpu6050_device = mpu6050.MPU6050(i2c0, 104)
ez_timer_obj = None
sw_controller = sw_motor0 = sw_motor1 = sw_motor2 = sw_motor3 = None

def get_gyro():
    return -mpu6050_device.angle_z() + 90 #change to math convention instead of navigation convention

def steer(direction):
    pin.servo_write_deg(5, direction) #set servo at an angle to steer the robot
    
def drive(direction, speed): #allows robot to drive with front wheels at an angle
    steer(direction)
    sw_motor0.run(speed)
    
def drive_heading(direction, speed, gyro_angle): #allows robot to drive with correction
    error = direction - gyro_angle
    correction = -1 * error
    correction = max(-25, min(correction, 25))
    correction = correction + 90
    drive(correction, speed)

def limit_angle(angle): #ensures components are notoverexerted
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle

def get_front_laser(): #converts sensor values to actual distance
    x = pin.analog_read(32)
    if x == 0: #prevents zero dvision error if wall is out of range
        return 400
    return abs(((725406/pin.analog_read(32)) + 14.6127)*math.cos(get_gyro())) + 11.5 #equation of graph

def get_rear_laser(): #converts sensor values to actual distance
    x = pin.analog_read(35)
    if x == 0: #prevents zero dvision error if wall is out of range
        return 400
    return abs(((725406/pin.analog_read(35)) + 14.6127)*math.cos(get_gyro())) + 11.5 #equation of graph

def get_left_laser(): #converts sensor values to actual distance
    x = pin.analog_read(34)
    if x == 0: #prevents zero dvision error if wall is out of range
        return 400
    return abs(((481062/pin.analog_read(34)) + 0.30997)*math.cos(get_gyro())) + 6 #equation of graph

def get_right_laser(): #converts sensor values to actual distance
    x = pin.analog_read(33)
    if x == 0: #prevents zero dvision error if wall is out of range
        return 400
    return abs(((481062/pin.analog_read(33)) + 0.30997)*math.cos(get_gyro())) + 6 #equation of graph
    
def get_laser_line_eqn(gyro_angle, pos): #obtain the equation of laser line, allows us to determine if laser intercepts the 4 walls
    m = math.tan(math.radians(get_gyro()))
    x = pos[0]
    y = pos[1]
    c = y - m * x
    return m, c

def intercept_top(angle, m, c): #checks if laser intersects the top wall by solving simultaneous equations
    if -180 <= limit_angle(angle) <= 0: #if angle is out of this range, it will not be intercepting the top wall
        return False
    
    if m == 0: # Prevents a divide by zero error in the next line
        return False
    x = (300 - c) / m
    # Return true if intercept point is at least 5cm from edge
    if 5 < x < 295:
        return True
    return False

def intercept_bottom(angle, m, c): #checks if laser intersects the bottom wall
    if 0 <= limit_angle(angle) <= 180: #if angle is out of this range, it will not be intercepting the bottom wall
        return False
    
    if m == 0:
        return False
    x = (0 - c) / m

    if 5 < x < 295:
        return True
    return False

def intercept_left(angle, m, c):
    if -90 <= limit_angle(angle) <= 90: #if angle is out of this range, it will not be intercepting the left wall
        return
    
    y = c
    if 5 < y < 295:
        return True
    return False

def intercept_right(angle, m, c):
    if limit_angle(angle) >= 90 or limit_angle(angle) <= -90: #if angle is out of this range, it will not be intercepting the right wall
        return False

    y = m * 300 + c
    if 5 < y < 295:
        return True
    return False

prev_motor_degrees = 0
def get_distance_travelled(): #determines with stepper motor
    global prev_motor_degrees
    
    current_degrees = sw_motor0.steps()
    cm = (current_degrees - prev_motor_degrees) / 360 * 17.6 #converts degrees of stepper into centimetres
    prev_motor_degrees = current_degrees
    
    return cm

def get_position_estimate(prev_pos, avg_gyro_angle, distance_travelled): #calculates approx coordinates based on distance travelled
    dx = distance_travelled * math.cos(math.radians(avg_gyro_angle))
    dy = distance_travelled * math.sin(math.radians(avg_gyro_angle))

    x = prev_pos[0] + dx
    y = prev_pos[1] + dy
    return x, y

def calculate_xy(angle, est_pos, distance): #measures exact distance from wall
    m, c = get_laser_line_eqn(angle, est_pos)
    x = None
    y = None

    if intercept_top(angle, m, c):
        # Top wall is 300cm from origin, so our position is 300 - y_distance_to_wall
        y = 300 - distance * math.sin(math.radians(angle))
    elif intercept_bottom(angle, m, c):
        # Bottom wall is at origin, but measuring downwards will give us a negative
        # value, so we invert in with a "-" at the front.
        y = -distance * math.sin(math.radians(angle))
    elif intercept_left(angle, m, c):
        # Left wall is at origin, but measuring towards the left will give us a negative
        # value, so we invert in with a "-" at the front.
        x = -distance * math.cos(math.radians(angle))
    elif intercept_right(angle, m, c):
        # Right wall is 300cm from origin, so our position is 300 - x_distance_to_wall
        # value, so we invert in with a "-" at the front.
        x = 300 - distance * math.cos(math.radians(angle))
        
    return x, y

def calculate_position(gyro_angle, est_pos): # if the difference between measured and estimated position is large, take the estimated position
    # sensors may be blocked by traffic signs so it is important to determine a more accurate position instead of solely relying on sensor readings
    all_x = []
    all_y = []
    
    sensors = [
        [gyro_angle, get_front_laser()],
        [gyro_angle + 90, get_left_laser()],
        [gyro_angle + 180, get_rear_laser()],
        [gyro_angle + 270, get_right_laser()],
    ]

    for sensor in sensors:
        x, y = calculate_xy(sensor[0], est_pos, sensor[1])
        if x != None and abs(x - est_pos[0]) < 5:
            all_x.append(x)
        if y != None and abs(y - est_pos[1]) < 5:
            all_y.append(y)

    x_count = len(all_x)
    y_count = len(all_y)
    
    x = None
    y = None
    
    if x_count > 0:
        x = sum(all_x) / x_count
    if y_count > 0:
        y = sum(all_y) / y_count
        
    return x, y
    
def get_position(gyro_angle):
    global prev_pos, prev_gyro_angle

    distance_travelled = get_distance_travelled()
    avg_gyro_angle = (gyro_angle + prev_gyro_angle) / 2
    est_pos = get_position_estimate(prev_pos, avg_gyro_angle, distance_travelled)

    measured_pos = calculate_position(gyro_angle, est_pos)

    x, y = est_pos
    if measured_pos[0] != None and abs(measured_pos[0] - est_pos[0]) < 5:
        x = measured_pos[0]

    if measured_pos[1] != None and abs(measured_pos[1] - est_pos[1]) < 5:
        y = measured_pos[1]

    pos = [x, y]
    
    # Update prev values with the current values
    prev_pos = pos
    prev_gyro_angle = gyro_angle
    
    return pos


def dot(v1, v2): #dot product of 2 vectors
    return v1[0]*v2[0] + v1[1]*v2[1]

def drive_path(start, end): #determines path for robot to travel in based on specificed coordinates
    path_vec = [end[0]-start[0], end[1]-start[1]]
    path_length = math.sqrt(path_vec[0]**2 + path_vec[1]**2)
    path_direction = -(math.degrees(math.atan2(path_vec[1], path_vec[0]))-90)
    unit_path_vec = [path_vec[0]/path_length, path_vec[1]/path_length]
    unit_perpen_vec = [unit_path_vec[1], -unit_path_vec[0]]

    gyro_angle = get_gyro()
    while path_direction - gyro_angle > 180:
        path_direction -= 360
    while path_direction - gyro_angle < -180:
        path_direction += 360

    while True:
        gyro_angle = get_gyro()
        pos = get_position(gyro_angle)
        
        pos_vec = [pos[0] - start[0], pos[1] - start[1]]
        distance_travelled = dot(pos_vec, unit_path_vec)
        error = dot(pos_vec, unit_perpen_vec)
        if distance_travelled >= path_length:
            break
        correction = error * 4
        correction = min(max(correction, -40), 40)
        drive_heading(path_direction+correction, 40, gyro_angle)
        
prev_pos = [get_left_laser(), get_rear_laser()]
prev_gyro_angle = 90 

nodes = [ #coordinates robot will travel to 
    [51.3, 55.1],
    [30.6, 260.3],
    [259, 266],
    [104.4, 178],
    [256.7, 85.8],
    [164.3, 37.5],
]
path = 0
last_node = len(nodes) - 1

i2c0 = machine.I2C(0, freq=100000)
sw_controller = stepper_wheels.Controller(i2c0, 85)
sw_motor0 = sw_controller.get_motor(0)
sw_motor1 = sw_controller.get_motor(1)
sw_motor2 = sw_controller.get_motor(2)
sw_motor3 = sw_controller.get_motor(3)
mpu6050_device = mpu6050.MPU6050(i2c0, 104)
uart1 = machine.UART(1, baudrate=9600, tx=13, rx=4)

pin.digital_write(2, 1) #turns on LED when initialising is complete

while True: #receives information from camera on colour of traffic sign
    code = uart1.readline()
    if code != None:
        try:
            code = code.decode()
            code = int(code)
            print(code)
        except:
            print("err", code)

while True: #robot follows along path to get to different nodes
    print(prev_pos)
    start_node = nodes[path]
    # If the starting node is the last one, then the ending node must be the first
    if path == last_node:
        end_node = nodes[0]
    else:
        end_node = nodes[path+1]

    drive_path(start_node, end_node)
    
    # We've reached the end of the path, so move on to the next one.
    path += 1
    if path > last_node: # If at the last node, we'll start again at the first
        path = 0
