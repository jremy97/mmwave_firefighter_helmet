//
// Created by doug on 2021-03-20.
//

#include <iostream>

#include <algorithm>
#include <vector>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include "point_matcher.h"

#include <pcl/registration/icp.h>

//publisher for outputting point clouds used during testing
ros::Publisher test_pub;
//publisher for the final fine map point cloud
ros::Publisher output_pub;

//stores the number of points used for matching, only the x most intense points are used
const int NUM_POINTS = 30;

//stores the points from the previous scan
pcl::PointCloud<pcl::PointXYZ>::Ptr previousScanPoints;

//records the number of clouds already processed
int cloudsProcessed = 0;

//stores the helmet's current position
Eigen::Matrix4f position;

bool compareI(pcl::PointXYZI a, pcl::PointXYZI b) {
    return a.intensity > b.intensity;
}


void refreshMap(const sensor_msgs::PointCloud2::ConstPtr& msg) {

    //deserializes the point cloud
    pcl::PCLPointCloud2* incommingCloud = new pcl::PCLPointCloud2;
    pcl_conversions::toPCL(*msg, *incommingCloud);

    pcl::PointCloud<pcl::PointXYZI>::Ptr pointVector = pcl::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    pcl::fromPCLPointCloud2(*incommingCloud,*pointVector);

    std::sort (pointVector->begin(), pointVector->end(), compareI);
    if(pointVector->size()>NUM_POINTS) {
        pointVector->resize(NUM_POINTS);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredPoints = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    filteredPoints->resize(pointVector->size());
    for (size_t i = 0; i < filteredPoints->points.size(); i++) {
        filteredPoints->points[i].x = pointVector->points[i].x;
        filteredPoints->points[i].y = pointVector->points[i].y;
        filteredPoints->points[i].z = pointVector->points[i].z;
    }


    //the if is used to prevent a crash on the first run through
    if(previousScanPoints!= nullptr) {
        //calculates the transform for the current frame
        Eigen::Matrix4f frameTransform = maximumCorrespondenceTransform(previousScanPoints, filteredPoints);
        std::cout << "Transformation: " << std::endl;
        std::cout << frameTransform << std::endl;
        position = position * frameTransform;
        std::cout << "Position: " << std::endl;
        std::cout << position << std::endl;

        //clears the point cloud from the previous pass through
        previousScanPoints->clear();
    }
    previousScanPoints=filteredPoints;

    test_pub.publish(*pointVector);


    //clears incoming cloud from memory
    delete incommingCloud;

    //with the current bag file, this should have 469 points processed by the end of the bag file
    ROS_INFO("Processed %i clouds", ++cloudsProcessed);
}

int main(int argc, char** argv)
{

    std::cout << "Starting" << std::endl;

    //prevents PCL from logging to the console
    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

    //initializes the matrix to identity (1 to 1 transform)
    position.setIdentity();

    // Initialize the ROS Node
    ros::init(argc, argv, "fine_map");

    ROS_INFO_STREAM("Starting node " << ros::this_node::getName());

    // Instantiate the ROS Node Handler as nh
    ros::NodeHandle n;

    ros::Subscriber s= n.subscribe("preprocessed", 1000, refreshMap);

    test_pub = n.advertise<sensor_msgs::PointCloud2>("map_test", 5);
    output_pub = n.advertise<sensor_msgs::PointCloud2>("map", 5);
    ROS_INFO_STREAM("Subscribed " << ros::this_node::getName());
    ros::spin();

    ROS_INFO_STREAM("Closing node " << ros::this_node::getName());
    return 0;
}