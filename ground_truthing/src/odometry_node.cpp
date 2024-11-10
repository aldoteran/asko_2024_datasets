#include <ros/console.h>
#include <ros/ros.h>

#include <utils/odometer.h>


int main(int argc, char **argv) {
  ros::init(argc, argv, "odometry_node");
  ros::NodeHandle nh;

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);

  // Instantiate Odometer.
  Odometer odometer(nh);

  ros::Subscriber ins_sub =
      nh.subscribe("/lolo/core/ins", /*queue_size=*/50,
                   &Odometer::InsCallback, &odometer);

  ros::Subscriber imu_sub =
      nh.subscribe("/lolo/core/imu", /*queue_size=*/50,
                   &Odometer::ImuCallback, &odometer);

  ros::Rate rate(100.0);
  while (ros::ok()) {
    if (odometer.is_imu_init && odometer.is_ins_init) {
      try {
        geometry_msgs::TransformStamped map_tfm_lolo;
        map_tfm_lolo =
            tf_buffer.lookupTransform("map", "lolo/base_link", ros::Time(0));

        nav_msgs::Odometry odom_map_msg =
            odometer.ComposeOdometryMsg(map_tfm_lolo);
        nav_msgs::Odometry odom_rel_msg =
            odometer.ComputeRelative(odom_map_msg);

        odometer.map_odom_pub.publish(odom_map_msg);
        odometer.odom_pub.publish(odom_rel_msg);
      } catch (tf2::TransformException &ex) {
        ROS_WARN("We're fucking up: %s", ex.what());
        continue;
      }
    }

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
