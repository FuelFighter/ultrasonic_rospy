#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Range
from geometry_msgs.msg import PointStamped
import math
import numpy as np
import threading

class CalculateVelRef:
    


    def __init__(self):

        self.VARIANCE_LIMIT = 200
        self.TIME_LIMIT = 5000

        self.CLOSEST_ALLOWED_CENTER_DISTANCE = 50
        self.CLOSEST_ALLOWED_SIDE_DISTANCE = 30

        self.MAX_SIDE_CONTROL_DISTANCE = 150

        self.UPPER_LIMIT_VELREF_X = 1
        self.ABS_LIMIT_VELREF_Y = 0.5

        self.MAX_CENTER_MEASUREMENT = 500

        rospy.init_node('velrefcalc', anonymous=True)

        
        self.velref_pub = rospy.Publisher("control/velref", PointStamped, queue_size=10)
        self.ultra_left_sub = rospy.Subscriber("ultrasound/left/filtered", Range, self.store_new_certain_measurement, callback_args="left")
        self.ultra_center_sub = rospy.Subscriber("ultrasound/center/filtered", Range, self.store_new_certain_measurement, callback_args="center")
        self.ultra_right_sub = rospy.Subscriber("ultrasound/right/filtered", Range, self.store_new_certain_measurement, callback_args="right")
        self.last_certain_measurement = {"left": Range(), "center": Range(), "right": Range()}
        self.velref = PointStamped()
        self.velref.header.stamp = rospy.Time.now()
        self.velref.header.frame_id = "/ultrasound/center"
        self.velref_calculate_counter = 0

        

        


    def store_new_certain_measurement(self, new_range_message, sensor):
        if new_range_message.field_of_view < self.VARIANCE_LIMIT:
            self.last_certain_measurement[sensor] = new_range_message
            self.velref_calculate_counter += 1
            if (self.velref_calculate_counter % 3) == 0:
                self.calculate_and_publish_velref()

    def calculate_new_velref(self):
        new_velref = self.velref
        if (self.last_certain_measurement["center"] < self.CLOSEST_ALLOWED_CENTER_DISTANCE or
        self.last_certain_measurement["right"] < self.CLOSEST_ALLOWED_SIDE_DISTANCE or
        self.last_certain_measurement["left"] < self.CLOSEST_ALLOWED_SIDE_DISTANCE):
            new_velref.point.x = 0
            new_velref.point.y = 0
        else:
            new_velref.point.x = self.last_certain_measurement["center"].range/self.MAX_CENTER_MEASUREMENT
            if ((self.last_certain_measurement["right"].range < self.MAX_SIDE_CONTROL_DISTANCE) or
            (self.last_certain_measurement["left"].range < self.MAX_SIDE_CONTROL_DISTANCE)):
                new_velref.point.y = (self.last_certain_measurement["left"].range-self.last_certain_measurement["right"].range)/50
                print(new_velref.point.y)
            else:
                new_velref.point.y = 0
        velref = self.set_within_bounds(new_velref)
        self.velref.header.stamp = rospy.Time.now()
        return velref

    def calculate_and_publish_velref(self):
        new_velref = self.calculate_new_velref()
        self.velref = new_velref
        self.velref_pub.publish(new_velref)
        return new_velref

    def set_within_bounds(self,velref):
        if velref.point.x > self.UPPER_LIMIT_VELREF_X:
            velref.point.x = self.UPPER_LIMIT_VELREF_X
        elif velref.point.x < 0:
            velref.point.x = 0
        
        if abs(velref.point.y) > self.ABS_LIMIT_VELREF_Y:
            velref.point.y = float(np.sign(velref.point.y))*self.ABS_LIMIT_VELREF_Y
        return velref



            

calculator = CalculateVelRef()
try:
    rospy.spin()
except KeyboardInterrupt:
    print("Shutting down")