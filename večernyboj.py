#!/usr/bin/env python3

# Import the necessary libraries
import time
import math
from ev3dev2.motor import *
from ev3dev2.sound import Sound
from ev3dev2.button import Button
from ev3dev2.sensor import *
from ev3dev2.sensor.lego import *
from ev3dev2.sensor.virtual import *

# Create the sensors and motors objects
motorA = LargeMotor(OUTPUT_A)
motorB = LargeMotor(OUTPUT_B)
left_motor = motorA
right_motor = motorB
tank_drive = MoveTank(OUTPUT_A, OUTPUT_B)
steering_drive = MoveSteering(OUTPUT_A, OUTPUT_B)

spkr = Sound()
btn = Button()
radio = Radio()

color_sensor_in1 = ColorSensor(INPUT_1)
ultrasonic_sensor_in2 = UltrasonicSensor(INPUT_2)
ultrasonic_sensor_in3 = UltrasonicSensor(INPUT_3)
ultrasonic_sensor_in4 = UltrasonicSensor(INPUT_4)
gyro_sensor_in5 = GyroSensor(INPUT_5)
gps_sensor_in6 = GPSSensor(INPUT_6)

motorC = LargeMotor(OUTPUT_C) # Magnet

# Here is where your code starts

uhol = None
rychlost = None
otocenie = None

# Rotácia na mieste (viac menej)
def rotovat(uhol, rychlost):
    global otocenie
    gyro_sensor_in5.reset()
    left_motor = motorA
    right_motor = motorB
    tank_drive = MoveTank(OUTPUT_A, OUTPUT_B)
    steering_drive = MoveSteering(OUTPUT_A, OUTPUT_B)
    while not gyro_sensor_in5.angle == uhol:
        if gyro_sensor_in5.angle < uhol:
            tank_drive.on(rychlost, (rychlost * -1))
        else:
            tank_drive.on((rychlost * -1), rychlost)
    tank_drive.off(brake=False)

# Rotácia ale taká s oblúkom v podstade vyberiáme zatacku
def rotovat_drift(otocenie, rychlost):
    global uhol
    gyro_sensor_in5.reset()
    left_motor = motorA
    right_motor = motorB
    tank_drive = MoveTank(OUTPUT_A, OUTPUT_B)
    steering_drive = MoveSteering(OUTPUT_A, OUTPUT_B)
    while not gyro_sensor_in5.angle == otocenie:
        if gyro_sensor_in5.angle < otocenie:
            steering_drive.on(rychlost, (rychlost * -1))
        else:
            steering_drive.on((rychlost * -1), rychlost)
    tank_drive.off(brake=False)


motorC.on(100)
while ultrasonic_sensor_in2.distance_centimeters >= 95:
    tank_drive.on(30, 30)
rotovat_drift(-90, 20)
while ultrasonic_sensor_in2.distance_centimeters >= 20:
    tank_drive.on(30, 30)
motorA.off(brake=False)
rotovat(-90, 5)
tank_drive.on_for_rotations(20, 20, 2)
rotovat(180, 5)
while ultrasonic_sensor_in2.distance_centimeters >= 70:
    tank_drive.on(30, 30)
rotovat(90, 5)
while ultrasonic_sensor_in3.distance_centimeters >= 30:
    tank_drive.on(30, 30)
tank_drive.on_for_rotations(20, 20, 1)
tank_drive.off(brake=False)
rotovat(-80, 5)
while ultrasonic_sensor_in2.distance_centimeters >= 20:
    tank_drive.on(30, 30)
tank_drive.off(brake=True)
motorC.on(0)
while gps_sensor_in6.y >= 84:
    tank_drive.on((-20), (-20))
tank_drive.off(brake=False)
