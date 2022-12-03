from carla import Actor, Image, VehicleControl, VehicleLightState
import numpy as np
import cv2 as cv
import colorsys
from enum import Enum

counter = 2020
threshold = 0
cast = None
reverse = True

class State(Enum):
    PENDING = -1
    FORWARDING = 0
    LEFT = 1
    RIGHT = 2
    FINISH = 3

def perspective_transform(img):
    xoffset = 0
    yoffset = 0
    img_size = (img.shape[1], img.shape[0])
    src = np.float32([(150, 400), (650, 400), (0, 599), (799, 599)])
    dst = np.float32([
        [xoffset, yoffset],
        [img_size[0]-xoffset, yoffset],
        [xoffset, img_size[1]-yoffset],
        [img_size[0]-xoffset, img_size[1]-yoffset]
    ])
    # Given src and dst points, calculate the perspective transform matrix
    M = cv.getPerspectiveTransform(src, dst)
    # Warp the image using OpenCV warpPerspective()
    warped = cv.warpPerspective(img, M, img_size)
    
    return warped

def bottom_points(img):
    imageHeight = img.shape[0]
    imageWidth = img.shape[1]

    left_histogram = np.sum(img[(imageHeight-30):,:(imageWidth//2)], axis=0)   
    right_histogram = np.sum(img[(imageHeight-30):,(imageWidth//2):], axis=0)

    # get local maxima
    starting_left_peak = np.argmax(left_histogram)
    starting_right_peak = np.argmax(right_histogram)

    return starting_left_peak, starting_right_peak

class StudentAgent:
    camera_image = None
    lidar_image = None
    state = None
    degree = None

    def __init__(self):
        ## TODO
        self.state = State.PENDING
        self.degree = 0

    def step(self, actor: Actor) -> VehicleControl:
        ## TODO
        actor.set_light_state(VehicleLightState.HighBeam)
        control = actor.get_control()

        ## To draw an image using OpenCV, please call imshow() in step().
        ## Do not imshow() in on_xxx_data(). It freezes the program!
        if self.camera_image is not None:
            cv.imshow("camera", self.camera_image)
            # img = cv.resize(perspective_transform(self.camera_image), (320, 240))
            # img = perspective_transform(self.camera_image)
            # cv.imshow("transform", img)

        if self.lidar_image is not None:
            im = cv.resize(self.lidar_image, (320, 240))
            cv.imshow("lidar", im)

        cv.waitKey(1)

        ## Decide control based on state & degree
        control.throttle = 0.24
        if(self.state == State.PENDING):
            control.throttle = 0.0
            control.steer = 0.0
        elif(self.state == State.FORWARDING):
            control.steer = 0.0
        elif(self.state == State.LEFT):
            control.steer = -1.0
        elif(self.state == State.RIGHT):
            control.steer = 1.0
        elif(self.state == State.FINISH):
            control.steer = 0.0
            control.throttle = 0.0
            control.brake = 0.8

        if(self.state==State.LEFT or self.state==State.RIGHT):
            if(self.degree == 1):
                control.throttle -= 0.02
                control.steer *= 0.05
            elif(self.degree == 2):
                control.throttle -= 0.04
                control.steer *= 0.08
            elif(self.degree == 3):
                control.throttle -= 0.07
                control.steer *= 0.14
            elif(self.degree == 4):
                control.throttle -= 0.1
                control.steer *= 0.2
            elif(self.degree == 5):
                control.throttle -= 0.13
                control.steer *= 0.4

        return control


    def on_lidar_data(self, points: np.ndarray):
        ## TODO
        ## 'points' is an Nx4 array with x, y, z, intensity columns

        lidar_range = 50.0
        ih = 600
        iw = 800

        points = points[:, :2].copy()
        points *= min(ih, iw) / (2.0 * lidar_range)
        points += (0.5 * ih, 0.5 * iw)
        points = np.fabs(points)  # pylint: disable=E1111
        points = points.astype(np.int32)
        points = np.reshape(points, (-1, 2))
        image = np.zeros((ih, iw, 3), dtype=np.uint8)
        image[tuple(points.T)] = (255, 255, 255)

        self.lidar_image = image # 600 * 800 * 3
        
    def on_camera_data(self, image: np.ndarray):
        ## TODO
        ## 'image' is an MxNx3 array with r, g, b columns

        ## HSV thresholding
        orange_min = np.array([28, 10, 10], np.uint8)
        orange_max = np.array([30, 80, 100], np.uint8)
        image = cv.cvtColor(image, cv.COLOR_BGR2HSV)
        image = cv.inRange(image, orange_min, orange_max)

        ## erosion
        erosion_size = 1
        kernel = cv.getStructuringElement(
            cv.MORPH_ELLIPSE,
            (2 * erosion_size + 1, 2 * erosion_size + 1),
            (erosion_size, erosion_size),
        )
        image = cv.dilate(image, kernel, iterations=2)
        
        ## remove noise
        image = cv.medianBlur(image, 5)
        # self.camera_image = cv.Laplacian(self.camera_image, ddepth=3)
        # self.camera_image = cv.convertScaleAbs(self.camera_image)
        ## perspective transform
        image = perspective_transform(image)
        
        self.camera_image = cv.resize(image, (320, 240))

        left_point, right_point = bottom_points(self.camera_image)
        ## Decide state
        global cast
        global reverse
        if(self.state == State.PENDING):
            if(left_point>=26 and left_point<=34):
                self.state = State.FORWARDING
        if(cast == None or cast <= 1000000):
            cast = np.sum(self.camera_image)
        elif(np.sum(self.camera_image) > cast*2):
            # self.state = State.FINISH
            reverse = True
        if(not reverse):           
            if(self.state != State.FINISH):
                if(left_point>=26 and left_point<=34):
                    self.state = State.FORWARDING
                elif(left_point > 34):
                    self.state = State.RIGHT
                elif(left_point < 26):
                    self.state = State.LEFT
                
            if(self.state==State.LEFT or self.state==State.RIGHT):
                gap = abs(left_point-30)
                a, b, c, d = 8, 13, 18, 23
                if(gap <= a):
                    self.degree = 1
                elif(gap>a and gap<=b):
                    self.degree = 2
                elif(gap>b and gap<=c):
                    self.degree = 3
                elif(gap>c and gap<=d):
                    self.degree = 4
                elif(gap>d):
                    self.degree = 5
            else:
                self.degree = 0
        else:
            if(self.state != State.FINISH):
                if(right_point>=136 and right_point<=144):
                    self.state = State.FORWARDING
                elif(right_point > 144):
                    self.state = State.RIGHT
                elif(right_point < 136):
                    self.state = State.LEFT
                
            if(self.state==State.LEFT or self.state==State.RIGHT):
                gap = abs(right_point-140)
                a, b, c, d = 8, 13, 18, 23
                if(gap <= a):
                    self.degree = 1
                elif(gap>a and gap<=b):
                    self.degree = 2
                elif(gap>b and gap<=c):
                    self.degree = 3
                elif(gap>c and gap<=d):
                    self.degree = 4
                elif(gap>d):
                    self.degree = 5
            else:
                self.degree = 0
