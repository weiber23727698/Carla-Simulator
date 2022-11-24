from enum import Enum
from pickle import FALSE
from carla import Vector3D, CityObjectLabel
import carla
import csv
from math import *

WAYPOINTS = [] # x decrease, y increase (steer<0 y變大)
target = 0 # index of waypoint

stage = 1

def load_waypoints(path):
    global WAYPOINTS
    with open(path) as f:
        rows = csv.reader(f)
        WAYPOINTS = [[float(x) for x in row] for row in rows]

def step(my_car):
    
    global WAYPOINTS   # The waypoints are stored here
    global target 
    global stage

    car_orientation = my_car.get_transform().rotation.yaw   # Car orientation
    loc = (my_car.get_location().x, my_car.get_location().y)   # Car location
    print(loc)
    if(loc[1] > WAYPOINTS[24][1]):
        stage = 4
    steer_control = 1.0
    if(stage < 4):
        dist = sqrt((loc[0]-WAYPOINTS[target][0])**2 + (loc[1]-WAYPOINTS[target][1])**2)
        if(dist < 0.5):
            target += 1
        if(target>=4):
            stage = 2
        if(target >= 6):
            steer_control = 1.5
        if(target >= 12):
            steer_control = 2
        if(target >= 16):
            steer_control = 5
        if(target == 20):
            steer_control = 6 
        if(target == 21):
            steer_control = 3
        if(target >= 22):
            steer_control = 5
    if(target == 25):
        stage = 4

    # apply control to vehicle
    control = my_car.get_control()
    if(stage == 4):
        control.throttle = 0.0
        control.brake = 1.0
    else:
        control.throttle = 0.25
        control.brake = 0.0
        if(stage == 2):
            control.brake = 0.1
        if(target >= 21):
            control.brake = 0.08
        dx = WAYPOINTS[target][0] - loc[0]
        dy = WAYPOINTS[target][1] - loc[1]
        tan1 = dy/dx # direction should go
        tan2 = tan(car_orientation/180 * pi) # current direction
        diff = abs((tan1-tan2)/(1+tan1*tan2))

        if(stage == 1):
            control.steer = 0.0
        else:
            if(diff <= 0.2):
                control.steer = -0.001
            elif(diff <= 0.5):
                control.steer = -0.04
            elif(diff <= 0.8):
                control.steer = -0.11
            elif(diff <= 1.0):
                control.steer = -0.14
            elif(diff <= 1.3):
                control.steer = -0.16
            elif(diff <= 1.7):
                control.steer = -0.19
            elif(diff <= 2.0):
                control.steer = -0.22
            else:
                control.steer = -0.25
            control.steer *= steer_control
            if(tan1 < tan2 and target >= 21):
                control.steer *= -4
    my_car.apply_control(control)
