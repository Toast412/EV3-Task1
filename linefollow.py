#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor
from pybricks.parameters import Port
from pybricks.robotics import DriveBase
from pybricks.tools import wait
import math

# Setup
simulator = 0
if simulator == 1:
    ev3 = EV3Brick()
    left_motor = Motor(Port.A)
    right_motor = Motor(Port.B)
    robot = DriveBase(left_motor, right_motor, wheel_diameter=56, axle_track=152)
    light_sensor = ColorSensor(Port.S1)
else: 
    ev3 = EV3Brick()
    left_motor = Motor(Port.B)
    right_motor = Motor(Port.C)
    robot = DriveBase(left_motor, right_motor, wheel_diameter=56, axle_track=152)
    light_sensor = ColorSensor(Port.S3)


def getBrightness(sensor):
    r, g, b = sensor.rgb()
    return (r + g + b) / 3

def followLineSingleSensor(
    base_speed=90,
    kp=1.1,
    ki=0,
    kd=1.8,
    target=35,
    right_angle_threshold=70,
    lost_cycles_limit=5,
    right_recovery_turn_rate=320,
    right_recovery_speed=15,
    reacquire_threshold=43,
    recovery_timeout_ms=900,
):
    integral = 0
    last_error = 0
    lost_cycles = 0
    max_turn = 220  # EV3 DriveBase turn-rate is in deg/s, keep this realistic

    while True:
        brightness = light_sensor.reflection()

        # If the sensor stays bright for a few cycles, we likely passed over a
        # right-angle corner and temporarily lost the line.
        if brightness > right_angle_threshold:
            lost_cycles += 1
        else:
            lost_cycles = 0

        if lost_cycles >= lost_cycles_limit:
            # Commit to a stronger right pivot until the line is seen again.
            elapsed = 0
            while elapsed < recovery_timeout_ms:
                robot.drive(right_recovery_speed, right_recovery_turn_rate)
                wait(10)
                elapsed += 10
                if light_sensor.reflection() <= reacquire_threshold:
                    break

            # Reset controller state after recovery to avoid derivative spikes.
            lost_cycles = 0
            integral = 0
            last_error = 0
            continue

        error = target - brightness
        integral += error
        error_change = error - last_error
        turn_rate = kp * error + ki * integral + kd * error_change

        # Clamp turn_rate and cast to int
        turn_rate = int(max(-max_turn, min(max_turn, turn_rate)))

        # Slow down on large errors, but keep a minimum speed so the robot still
        # reaches and rotates through sharp corners.
        speed = max(55, base_speed * math.exp(-0.045 * abs(error)))

        robot.drive(speed, turn_rate)
        last_error = error

# Start line following
followLineSingleSensor()