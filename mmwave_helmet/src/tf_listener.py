#!/usr/bin/env python
import rospy
import tf
import math
import geometry_msgs.msg 

def main(): 
	rospy.init_node('tf_listener') 

	listener = tf.TransformListener() 

	servo_vel = rospy.Publisher('servo_vel', geometry_msgs.msg.Twist, queue_size=1) 

	rate = rospy.Rate(10) 

	while not rospy.is_shutdown():
		try: 
			(trans,rot) = listener.lookupTransform('/ti_mmwave_0', '/ti_mmwave_pcl', rospy.Time(0)) 
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException): 
			continue 

		print(rot)
		#cmd = geometry_msgs.msg.Twist() 
		#cmd.angular.z = angular 
		#servo_vel.publish(cmd)

		rate.sleep()

if __name__=='__main__': 
	try: 
		main()
	except rospy.ROSInterruptException: 
		pass
	

		

