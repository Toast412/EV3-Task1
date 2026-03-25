#!/usr/bin/env pybricks-micropython
# Import necessary libraries
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port, Color
from pybricks.robotics import DriveBase
from pybricks.tools import wait
import math
# ---------- STANDARD SETUP CODE ---------- #

ev3 = EV3Brick()

# Initialize the motors.
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)

#Initialise the sensors.
light_sensor = ColorSensor(Port.S3)
distance_sensor = UltrasonicSensor(Port.S4)

# Initialize the drive base.
robot = DriveBase(left_motor, right_motor, wheel_diameter=56, axle_track=152)

# ---------- MAINLINE PROGRAM ---------- #

### Here is where your code starts ###
def linefollow(base_speed=90, kp=1, ki=0, kd=1.5, target=50):
    integral = 0
    last_error = 0
    max_turn = 500  # safe limit for turn rate
    sweep_amount = 0
    sweep_direction = 1
    r, g, b = light_sensor.rgb()
    color = light_sensor.color()
    #ev3.screen.print(color)
    
    # Check for color markers (break out if detected)
    if g > r and g > b and g > 100:
        return "green"
    elif r > g and r > b and r > 100:
        return "red"
    
    # Use brightness for line detection (can be based on RGB sum or specific channel)
    brightness = (r + g + b) // 3
    error = target - brightness
    integral += error
    error_change = error - last_error
    turn_rate = kp * error + ki * integral + kd * error_change
    turn_rate = int(max(-max_turn, min(max_turn, turn_rate)))
    
    # Add gentle sweep motion
    sweep_amount += sweep_direction * 2
    if sweep_amount > 15:
        sweep_direction = -1
    elif sweep_amount < -15:
        sweep_direction = 1
    
    turn_rate += sweep_amount
    
    speed = base_speed * math.exp(-0.07 * abs(error))
    robot.drive(speed, turn_rate)
    last_error = error

def colorsense():
    while True:
        color = light_sensor.color()
        if color == Color.GREEN:
            robot.drive(100, -45)
            ev3.screen.print("green")
                
        elif color == Color.RED or color == Color.YELLOW:
            robot.drive(150, -45)
            ev3.screen.print("red/yellow")
        elif color == Color.BLACK:
            ev3.screen.print("black")
            linefollow()

colorsense()