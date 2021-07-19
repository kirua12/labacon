#ifndef STANLEY_H
#define STANLEY_H

#include<iostream>
#include"ros/ros.h"
#include<nav_msgs/Path.h>
#include<tf/transform_datatypes.h>
#include<tf/transform_listener.h>
#include<geometry_msgs/PoseStamped.h>
#include<geometry_msgs/Point.h>
#include<geometry_msgs/Twist.h>
#include<math.h>
#include<sensor_msgs/PointCloud2.h>
#include<std_msgs/String.h>
#include <vector>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/search/search.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

#include <geometry_msgs/PoseArray.h>
#include <pcl/features/moment_of_inertia_estimation.h>

#include <pcl/common/transformation_from_correspondences.h>
#include <tf/transform_listener.h>
#include <std_msgs/Float32.h>


class Stanley
{
public:
    Stanley(ros::NodeHandle nh,ros::NodeHandle pnh);
    //double Stanley::Yaw(const geometry_msgs::Pose& carPose);
    ~Stanley();
private:
    double L;
    double min_dist, error_x, error_y, error, error_cte;
    float  first_point, next_point;
    double cyaw, cx, cy, steering, velocity,velocity_gain;
    double min_index = 0;
    double k = 0.5;
    void Publish(void);
    void PathPointCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void CurrentPoseCallback(const geometry_msgs::PoseStamped::ConstPtr msg);
    void stateCB(const std_msgs::String stateMsg);
    double steering_cte();
    double velocity_();
    double velocity_mini();


    ros::NodeHandle nh;
    ros::NodeHandle pnh;

    std_msgs::String state;
    sensor_msgs::PointCloud2 path_point;
    pcl::PointCloud<pcl::PointXYZI> raw_data;
    pcl::PointXYZI path_pose,next_pose;
    geometry_msgs::PoseStamped current_pose;
    geometry_msgs::Twist cmd_vel;
    sensor_msgs::PointCloud2 waypoint;

    ackermann_msgs::AckermannDriveStamped cmd_vel_;

    ros::Publisher twist_pub, ackermann_pub;
    ros::Subscriber path_sub, current_pose_,state_sub, data_sub;

};

#endif // STANLEY_H
