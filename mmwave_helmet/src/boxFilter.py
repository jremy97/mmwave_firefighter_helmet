#!/usr/bin/env python
import rospy
import ros_numpy
import numpy as np
import ctypes
import struct
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from ros_numpy import point_cloud2 as pc2


'''
To do

Create a function to convert PointCloud2 message to numpy array - completed

- Need to use pointcloud message from /ti_mmwave/radar_scan_pcl_0 topic - completed

- Need to keep track of frameID and timestamp - completed

Create a function to filter points based on given radius from radar - completed

Publish new Pointcloud to a new topic - Completed
'''

new_cloud_msg = None


# Function to filter out points that are close to the radar
def point_filter(pointcloud_array): 
	# Define Euclidean norm of each point from pointcloud to frame_id
	radius = np.linalg.norm(pointcloud_array, axis=1)
	# Get indexes of points that have a radius of over 0.5
	# from the frame_id
	filteredIdx = np.where(radius > 0.5)[0]
	# New pointcloud array with indexes
	# that satisfy the condition above
	new_points = pointcloud_array[filteredIdx]
	return new_points


# Callback function to access PointCloud2 message data
def pc_callback(data):
	global new_arr, new_cloud_msg, new_PC_array, timestamp, frame_id
	# global variables to access the data from the message
	# Convert Pointcloud message to numpy record array
	new_arr = pc2.pointcloud2_to_array(data, squeeze=False)
	# filter out unwanted points from PointCloud2 message
	new_PC_array = point_filter(new_arr)
	# get timestamp of original message
	timestamp = data.header.stamp
	# get frame id of original message
	frame_id = data.header.frame_id
	# Convert filtered pointcloud array to new PointCloud2 message
	new_cloud_msg = pc2.array_to_pointcloud2(new_PC_array, timestamp, frame_id)


def main():
	global new_cloud_msg
	# initialize ROS node
	rospy.init_node('boxFilter', anonymous=True)
	# Publish at 15hz
	rate = rospy.Rate(15)
	# Subscriber object
	sub = rospy.Subscriber('/ti_mmwave/radar_scan_pcl_0', PointCloud2, pc_callback)
	# Publisher Object -> creates a new topic called filteredPC
	pc_pub = rospy.Publisher('/filteredPC', PointCloud2, queue_size=20)
	while not rospy.is_shutdown():
		# Publish new pointcloud message
		if new_cloud_msg is not None:
			pc_pub.publish(new_cloud_msg)
			rate.sleep()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
