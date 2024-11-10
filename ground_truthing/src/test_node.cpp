#include "utils/ros_eigen_conversions.h"
#include <iostream>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/Marker.h>
#include <sbg_driver/SbgImuData.h>
#include <sbg_driver/SbgGpsHdt.h>
#include <sbg_driver/SbgGpsPos.h>

int main(int argc, char *argv[]){
    std::cout << "something" << std::endl;
    nav_msgs::Odometry odom;
    const Eigen::Vector3d vels = EigenVelFromOdom(odom);
    std::cout << "something else" << std::endl;

    return 0;
}
