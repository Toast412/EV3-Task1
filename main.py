#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor
from pybricks.parameters import Port
from pybricks.robotics import DriveBase
import math

# Setup
ev3 = EV3Brick()
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)
robot = DriveBase(left_motor, right_motor, wheel_diameter=56, axle_track=152)
light_sensor = ColorSensor(Port.S3)

def getBrightness(sensor):
    r, g, b = sensor.rgb()
    return (r + g + b) / 3

def followLineSingleSensor(base_speed=100, kp=0.9, ki=0, kd=1.2, target=35):
    integral = 0
    last_error = 0
    max_turn = 1000  # safe limit for turn rate
    while True:
        brightness = light_sensor.reflection()
        # Invert error for left-hand side sensor
        error = target - brightness
        integral += error
        error_change = error - last_error
        turn_rate = kp * error + ki * integral + kd * error_change
        # Clamp turn_rate and cast to int
        turn_rate = int(max(-max_turn, min(max_turn, turn_rate)))
        # Slow down more on large errors
        speed = base_speed * math.exp(-0.07 * abs(error))
        robot.drive(speed, turn_rate)
        last_error = error

# Start line following
followLineSingleSensor()