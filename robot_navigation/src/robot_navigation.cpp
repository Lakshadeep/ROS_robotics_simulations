#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "robot_navigation/desired_heading_velocity.h"

double desired_heading, desired_velocity_x, desired_velocity_y;
bool desired_heading_velocity_callback(robot_navigation::desired_heading_velocity::Request &req, robot_navigation::desired_heading_velocity::Response &res)
{
  desired_heading = req.desired_heading ;
  desired_velocity_x = req.desired_velocity_x;
  desired_velocity_y = req.desired_velocity_y;
  ROS_INFO("Desired heading %f | Desired velocity x %f y %f", desired_heading, desired_velocity_x, desired_velocity_y);
  return true;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_publisher");

  ros::NodeHandle n;
  ros::ServiceServer desired_heading_velocity = n.advertiseService("desired_heading_velocity", desired_heading_velocity_callback);
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  tf::TransformBroadcaster odom_broadcaster;

  double x = 0.0;
  double y = 0.0;
  double th = 0.0;

  double vx = 0.0;
  double vy = 0.0;
  double vth = 0.0;

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate r(1.0);
  while(n.ok()){

    ros::spinOnce();               // check for incoming messages
    
    current_time = ros::Time::now();
    vx = desired_velocity_x;
    vy = desired_velocity_y;
    th = desired_heading;

    //compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();
    // double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    // double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    double delta_x = (vx * dt);
    double delta_y = (vy * dt);
    double delta_th = vth * dt;

    x += delta_x;
    y += delta_y;
    th = desired_heading ;

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th * 3.1457/180.0);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;
    // odom.pose.pose.orientation.w = th * 3.1457/180.0;
    // odom.pose.pose.orientation.z = th * 3.1457/180.0;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = 0;

    //publish the message
    odom_pub.publish(odom);

    last_time = current_time;
    r.sleep();
  }
}