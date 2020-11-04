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

- Need to use pointcloud message from /ti_mmwave/radar_scan_pcl_0 topic 

- Need to keep track of frameID and timestamp - completed 

Create a function to filter points based on given radius from radar - in progress

'''  

# Function to filter out points that are close to the radar 
def point_filter(pointcloud_array): 
	# Define Euclidean norm of each point from pointcloud to frame_id 
	radius = np.linalg.norm(pointcloud_array, axis=0)

	filteredIdx = np.where(radius > 0.5)[0]

	new_points = pointcloud_array[filteredIdx] 

	return new_points 
 	

# Callback function to access PointCloud2 message data 
def pc_callback(data): 
	# global variables to access the data from the message 
	global new_arr, timestamp, frame_id  
	# Convert Pointcloud message to numpy record array 
	new_arr = pc2.pointcloud2_to_array(data, squeeze=True) 

	new_PC = point_filter(new_arr)
	
	# get timestamp of message 
	timestamp = data.header.stamp
 
	# get frame id 
	frame_id = data.header.frame_id  
	print(new_PC)

	

def main():
	# initialize ROS node 
	rospy.init_node('boxFilter', anonymous=True) 
	
	# Subscriber object 
	sub = rospy.Subscriber('/ti_mmwave/radar_scan_pcl_0', PointCloud2, pc_callback) 
	
	rospy.spin() 


if __name__ == '__main__': 
	main()

	 

	


