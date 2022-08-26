import rospy
from std_msgs.msg import Int16MultiArray


pub = rospy.Publisher('cmd_angle', Int16MultiArray, queue_size=10)
rospy.init_node('ros_test', anonymous=True, log_level=rospy.DEBUG)

rate = rospy.Rate(10)
# while not rospy.is_shutdown():
rospy.loginfo('baal cholse')
msg = Int16MultiArray()
msg.data = [1,2,3]
pub.publish(msg)
rate.sleep()
