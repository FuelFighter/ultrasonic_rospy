#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Range
from geometry_msgs.msg import Point
import math
import numpy as np

class UltraSonicCalculator:
    def __init__(self):


        rospy.init_node('listener', anonymous=True)

        
        self.last_measurements = {"left": [], "center": [], "right": []}
        self.left_measure_and_variance_pub = rospy.Publisher("ultrasound/left/filtered", Range, queue_size=10)
        self.center_measure_and_variance_pub = rospy.Publisher("ultrasound/center/filtered", Range, queue_size=10)
        self.right_measure_and_variance_pub = rospy.Publisher("ultrasound/right/filtered", Range, queue_size=10)
        self.measurement_ultra_left = rospy.Subscriber("ultrasound/left/raw", Range, self.add_range_measurement, callback_args="left")
        self.measurement_ultra_center = rospy.Subscriber("ultrasound/center/raw", Range, self.add_range_measurement, callback_args="center")
        self.measurement_ultra_right = rospy.Subscriber("ultrasound/right/raw", Range, self.add_range_measurement, callback_args="right")
        self.avg_measurement = {"left": 0, "center": 0, "right": 0}
        self.sum_of_squared_measurements = {"left": 0, "center": 0, "right": 0}
        self.variance = {"left": 0, "center": 0, "right": 0}


        


    def add_range_measurement(self, new_range_message, sensor):
        minimum_fov = 0.001

        self.last_measurements[sensor] = [new_range_message.range] + self.last_measurements[sensor]
        del self.last_measurements[sensor][10:]
        self.avg_measurement[sensor] = sum(self.last_measurements[sensor])/len(self.last_measurements[sensor])
        sum_of_squared_error = 0
        for measurement in self.last_measurements[sensor]:
            sum_of_squared_error += (measurement-self.avg_measurement[sensor])**2
        self.variance[sensor] = sum_of_squared_error/len(self.last_measurements[sensor])

        
        range_and_variance_message = Range()    
        range_and_variance_message.radiation_type = Range.ULTRASOUND
        range_and_variance_message.header.frame_id =  "/ultrasound/"+sensor
        range_and_variance_message.min_range = 25.0
        range_and_variance_message.max_range = 500.0
 
        range_and_variance_message.range = self.avg_measurement[sensor]
        
        range_and_variance_message.field_of_view = np.clip(self.variance[sensor]/1000, 0, 10) # Using FOV as variance

        
        if sensor == "left":
            self.left_measure_and_variance_pub.publish(range_and_variance_message)
        elif sensor == "center":
            self.center_measure_and_variance_pub.publish(range_and_variance_message)
        elif sensor == "right":
            self.right_measure_and_variance_pub.publish(range_and_variance_message)


            

ultrasonic_calculator = UltraSonicCalculator()
try:
    rospy.spin()
except KeyboardInterrupt:
    print("Shutting down")