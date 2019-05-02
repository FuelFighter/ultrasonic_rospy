# ultrasonic_rospy
This repository has two python scripts for processing the ultrasonic data gathered in three_sensor_ros.ino in [ultrasonic-test](https://github.com/FuelFighter/ultrasonic-test).

## ultrasonic_rosnode.py
This file creates a rosnode that subcribes to the raw ultrasonic sensordata topic `ultrasonic/[sensorplacement]/raw` and
calculates the mean and variance of the three ultrasonic sensors and
outputs this on the topic `ultrasonic/[sensorplacement]/filtered`.
In the filtered range message we use the range_msg.field_of_view to store the variance.

## calculate_velref.py
This file relies only on the filtered ultrasonic data to decide where we want the car to go.
This creates a rosnode that subscribes to `ultrasonic/[sensorplacement]/filtered` and
only uses the measurements that are below a certain variance level in the calculation of the velocity reference.

The velocity reference is outputed as a [PointStamped.msg](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/PointStamped.html)
where the x component is the desired speed in the forward direction and
the y component is how much we would like to turn. It published to the topic `control/velref` in the coordinate frame of the `ultrasonic/center`
