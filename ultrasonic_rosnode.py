import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo("I heard %s",data.data)
    
def listenerL():
    rospy.init_node('ultrasonic_rosnode')
    rospy.Subscriber("ultrasoundL", String, callback)
    rospy.spin()