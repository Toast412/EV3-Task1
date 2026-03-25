#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port, Color
from pybricks.robotics import DriveBase
from pybricks.tools import wait
import math

ev3 = EV3Brick()

left_motor = Motor(Port.B)
right_motor = Motor(Port.C)
light_sensor = ColorSensor(Port.S3)
#distance_sensor = UltrasonicSensor(Port.S4)

# Initialize the drive base.
robot = DriveBase(left_motor, right_motor, wheel_diameter=56, axle_track=152)

while True:
    print(light_sensor.color())
    if light_sensor.color() == Color.BLUE or light_sensor.color() == Color.GREEN: #green
        robot.drive(20, 50)
        wait(40)
    elif light_sensor.color() == Color.RED: #red
        robot.drive(20, -55)
        wait(40)

    elif light_sensor.reflection() < 50:
        robot.drive(40, 55)
    else:
        robot.drive(40, -45)
        wait(40)