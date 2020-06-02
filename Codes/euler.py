import roslib 
import rospy
import tf

from sensor_msgs.msg import Imu                           
from std_msgs.msg import Float64
import math 
	   
def imu_callback(imu_data):
	D_Compass_declination = rospy.get_param('~declination',-7.462777777777778)* (math.pi/180.0)
	(r, p, ya) = tf.transformations.euler_from_quaternion([imu_data.orientation.x, imu_data.orientation.y, imu_data.orientation.z, imu_data.orientation.w])
	angle=ya+(math.pi/2.0)- D_Compass_declination
	pub.publish(angle*(180.0/math.pi))

# Main function.    
if __name__ == '__main__':
	global pub
	rospy.init_node('quat_to_euler')
	rospy.Subscriber("/kitti/oxts/imu", Imu, imu_callback)
	pub = rospy.Publisher("imu/HeadingTrue_degree", Float64)
	rospy.spin()
