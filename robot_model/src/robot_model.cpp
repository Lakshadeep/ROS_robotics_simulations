#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <robot_model/motor_voltage.h>

double left_motor, right_motor;
bool motor_voltage_callback(robot_model::motor_voltage::Request &req, robot_model::motor_voltage::Response &res)
{
  left_motor = req.left_motor ;
  right_motor = req.right_motor;
  ROS_INFO("Left motor voltage %f V | Right motor voltage %f V", left_motor, right_motor);
  return true;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_model");

  ros::NodeHandle n;
  ros::ServiceServer desired_heading_velocity = n.advertiseService("set_motor_voltage", motor_voltage_callback);
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  tf::TransformBroadcaster odom_broadcaster;

  double x = 0.0;
  double y = 0.0;
  double th = 0.0;

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  double calculated_angle;
  double calculated_velocity;
  double diff_voltage;

  ros::Rate r(10.0);
  while(n.ok()){

    //compute odometry in a typical way given the velocities of the robot
    ROS_WARN("Theta (Yaw) %f", th * 180/3.1457);
    current_time = ros::Time::now();
    double dt = (current_time - last_time).toSec();
    // double delta_x = (calculated_velocity * cos(th) - calculated_velocity * sin(th)) * dt;
    double delta_x = (calculated_velocity * cos(th)) * dt;
    
    // double delta_y = (calculated_velocity * sin(th) + calculated_velocity * cos(th)) * dt;
    double delta_y = (calculated_velocity * sin(th)) * dt;

    x += delta_x;
    y += delta_y;

    ros::spinOnce();               // check for incoming messages
    
    if(left_motor > right_motor)
    {
      diff_voltage = left_motor - right_motor;
      calculated_angle = (diff_voltage / 5) * 90;
      calculated_velocity = 0.01;

      if(calculated_angle > 18)
      {
        th = fmod((((th * 180/3.1457) - 18.0) + 360),360);
        th = (th * 3.1457/180);
      }
      else
      {
        th = fmod((((th * 180/3.1457) + calculated_angle) + 360),360);
        th = (th * 3.1457/180);
      }
    }
    else if(right_motor > left_motor)
    {
      diff_voltage = left_motor - right_motor;
      calculated_angle = (diff_voltage / 5) * 90;
      calculated_velocity = 0.01;

      if(calculated_angle < -18)
      {
        th = fmod((((th * 180/3.1457) + 18.0) + 360),360);
        th = (th * 3.1457/180);
      }
      else
      {
        th = fmod((((th * 180/3.1457) + calculated_angle) + 360),360);
        th = (th * 3.1457/180);
      }
    }
    else if(right_motor != 0 && left_motor != 0) 
    {
      calculated_angle = 0;         // for both equal case
      calculated_velocity = 1.0/100.0;
    }
    else{
      calculated_angle = 0;         // for both equal case
      calculated_velocity = 0;
    }

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

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
    odom.twist.twist.linear.x = calculated_velocity * cos(th);
    odom.twist.twist.linear.y = calculated_velocity * sin(th) ;
    odom.twist.twist.angular.z = 0;

    //publish the message
    odom_pub.publish(odom);

    last_time = current_time;
    r.sleep();
  }
}