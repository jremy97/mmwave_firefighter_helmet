#!/usr/bin/env python
import rospy
import tf
from tf.transformations import quaternion_from_euler
from rospy import Time
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Byte, Header
from math import pi

yaw_angle = None
frame_id = None
stamp = None

def deg2rad(angle):
    rads = (angle * pi)/180
    return rads

def get_angle(data):
    global yaw_angle
    servo_angle = data.data
    yaw_angle = deg2rad(servo_angle)
    #print(yaw_angle)

def get_PC_header(data_pc):
    global frame_id, stamp
    frame_id = data_pc.header.frame_id
    stamp = data_pc.header.stamp


def main():
	global yaw_angle, frame_id, stamp
	rospy.init_node('tf_broadcaster', anonymous=True)

	rate = rospy.Rate(5)

	broadcast = tf.TransformBroadcaster()
	pos_sub = rospy.Subscriber('/position', Byte, get_angle)
	pc_sub = rospy.Subscriber('/position', Byte, get_angle)

	translation = (0.0, -0.05, 0.795) 

	while not rospy.is_shutdown(): 
		if yaw_angle is not None: 
			broadcast.sendTransform(translation, quaternion_from_euler(0.0, 0.0, yaw_angle), Time.now(), frame_id, 'ti_mmwave_pcl')
			rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
