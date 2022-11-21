from enum import Enum
from pickle import FALSE
from carla import Vector3D, CityObjectLabel
import carla
import csv
from math import *

WAYPOINTS = []
target = 0 # index of next waypoint
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
    
    if(target == 25):
        stage = 4
        
    control = my_car.get_control()
    if(stage == 4):
        control.throttle = 0.0
        control.brake = 1.0
    else:
        control.throttle = 0.2
        control.brake = 0.0
        
        dx = WAYPOINTS[target][0] - loc[0]
        dy = WAYPOINTS[target][1] - loc[1]
        tan1 = dy/dx # direction should go
        tan2 = tan(car_orientation/180 * pi) # current direction
        diff = (tan1-tan2)/(1+tan1*tan2) # tan(a-b)
        
        if(abs(diff) <= 0.2):
            control.steer = 0
        elif(abs(diff) <= 0.4):
            control.steer = 0.1
        elif(abs(diff) <= 0.6):
            control.steer = 0.14
        elif(abs(diff) <= 0.8):
            control.steer = 0.18
        elif(abs(diff) <= 1.0):
            control.steer = 0.22
        elif(abs(diff) <= 1.4):
            control.steer = 0.26
        elif(abs(diff) <= 1.8):
            control.steer = 0.30
        elif(abs(diff) <= 2.3):
            control.steer = 0.35
        else:
            control.steer = 0.4
        if(diff > 0):
            control.steer *= -1
    my_car.apply_control(control)
