#include "robot_driver/estimators/realsense_estimator.h"

RealsenseEstimator::RealsenseEstimator() {}

void RealsenseEstimator::init(ros::NodeHandle& nh) {
  ROS_INFO("Realsense estimator initialized");
  nh_ = nh;
}

bool RealsenseEstimator::updateOnce(quad_msgs::RobotState& last_robot_state_msg) {
  ros::Time state_timestamp = ros::Time::now();

  // TODO
  //
  // Transform the coordinate frame
  // The camera is fixed to the body facing upwards
  // last_robot_state_msg.body.pose.position.z = last_odom.pose.pose.position.x;
  // last_robot_state_msg.body.pose.position.x = -last_odom.pose.pose.position.z;
  // last_robot_state_msg.body.pose.position.y = last_odom.pose.pose.position.y;

  // tf2::Quaternion quat;
  // quat.setW(last_odom.pose.pose.orientation.w);
  // quat.setX(last_odom.pose.pose.orientation.x);
  // quat.setY(last_odom.pose.pose.orientation.y);
  // quat.setZ(last_odom.pose.pose.orientation.z);

  // Assume that the realsense odometry data is already in global ROS coordinate frame
  last_robot_state_msg.body.pose.position = last_odom.pose.pose.position;

  tf2::Quaternion quat, rot;

  tf2::fromMsg(last_odom.pose.pose.orientation, quat);

  rot.setRPY(0, KDL::PI / 2, 0);

  last_robot_state_msg.body.pose.orientation = tf2::toMsg(quat * rot);

  last_robot_state_msg.body.twist.angular = last_odom.twist.twist.angular;
  last_robot_state_msg.body.twist.linear = last_odom.twist.twist.linear;

  last_robot_state_msg.joints = last_joint_state_msg_;

  last_joint_state_msg_.header.stamp = state_timestamp;

  // Fill in the rest of the state message (foot state and headers)
  quad_utils::fkRobotState(*quadKD_, last_robot_state_msg);
  quad_utils::updateStateHeaders(last_robot_state_msg, state_timestamp, "map",
                                 0);

  return true;
}

void RealsenseEstimator::updateOdomMsg(nav_msgs::Odometry &odom) {
  last_odom = odom;
  last_odom_upd_time = ros::Time::now();
}
