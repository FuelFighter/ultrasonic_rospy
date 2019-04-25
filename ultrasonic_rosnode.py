#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Range
from geometry_msgs.msg import Point

class UltraSonicCalculator:
    def __init__(self):


        rospy.init_node('listener', anonymous=True)

        
        self.last_measurements = {"left": [], "center": [], "right": []}
        self.left_measure_and_variance_pub = rospy.Publisher("Ultrasound/left/variance_and_measurement", Point)
        self.center_measure_and_variance_pub = rospy.Publisher("Ultrasound/center/variance_and_measurement", Point)
        self.right_measure_and_variance_pub = rospy.Publisher("Ultrasound/right/variance_and_measurement", Point)
        self.measurement_ultra_left = rospy.Subscriber("ultrasoundL", Range, self.add_range_measurement, callback_args="left")
        self.measurement_ultra_center = rospy.Subscriber("ultrasoundC", Range, self.add_range_measurement, callback_args="center")
        self.measurement_ultra_right = rospy.Subscriber("ultrasoundR", Range, self.add_range_measurement, callback_args="right")
        self.avg_measurement = {"left": 0, "center": 0, "right": 0}
        self.sum_of_squared_measurements = {"left": 0, "center": 0, "right": 0}


        


    def add_range_measurement(self, new_range_message, sensor):
        self.last_measurements[sensor] = [new_range_message.range] + self.last_measurements[sensor]
        del self.last_measurements[sensor][10:]
        avg_measurement[sensor] = avg_measurement[sensor]*len(self.last_measurements[sensor])
        sum_of_squared_measurements = 0
        for measurement in self.last_measurements[sensor]:
            sum_of_squared_error += (measurement-avg_measurement)**2
        variance = sum_of_squared_error/len(self.last_measurements[sensor])

        range_and_variance_message = Point()
        range_and_variance_message.x = avg_measurement
        range_and_variance_message.y = variance

        if sensor == "left":
            self.left_measure_and_variance_pub.publish(range_and_variance_message)
        elif sensor == "center":
            self.center_measure_and_variance_pub.publish(range_and_variance_message)
        elif sensor == "center":
            self.right_measure_and_variance_pub.publish(range_and_variance_message)


            

ultrasonic_calculator = UltraSonicCalculator()
try:
    rospy.spin()
except KeyboardInterrupt:
    print("Shutting down")