#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <robot_model/motor_voltage.h>
#include <robot_model/velocity.h>

double left_motor, right_motor;

double v_err, w_err, gamma_err; 

float alpha1 = 0.001;
float alpha2 = 0.001;
float alpha3 = 0.001;
float alpha4 = 0.001;
float alpha5 = 0.001;
float alpha6 = 0.001;


bool motor_voltage_callback(robot_model::motor_voltage::Request &req, robot_model::motor_voltage::Response &res)
{
  left_motor = req.left_motor ;
  right_motor = req.right_motor;
  ROS_INFO("Left motor voltage %f V | Right motor voltage %f V", left_motor, right_motor);
  return true;
}

bool velocity_callback(robot_model::velocity::Request &req, robot_model::velocity::Response &res)
{
  v_err = req.velocity_error ;
  w_err = req.omega_error;
  gamma_err = req.gamma_error;
  ROS_INFO("Velocity err %f Omega err %f  Gamma  %f", v_err, w_err, gamma_err);
  return true;
}

double randInRange(double min, double max)
{
  return min + (double) (rand() / (double) (RAND_MAX) * (max - min + 1));
}


double gaussion_sampling(double deviation)
{
  double sum = 0.0;
  for(int i = 1; i <= 12; i++ )
  {
     sum =+ (deviation / 6) * randInRange(-1.0, 1.0);
  }
  return sum;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_model");

  ros::NodeHandle n;
  // ros::ServiceServer desired_heading_velocity = n.advertiseService("set_motor_voltage", motor_voltage_callback);
  ros::ServiceServer desired_heading_velocity = n.advertiseService("set_velocity", velocity_callback);
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  tf::TransformBroadcaster odom_broadcaster;

  double x = 0.0;
  double y = 0.0;
  double th = 0.0;

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  double velocity;
  double omega;
  double gamma;

  ros::Rate r(10.0);
  while(n.ok()){

    ros::spinOnce(); 

    current_time = ros::Time::now();
    double dt = (current_time - last_time).toSec();

    velocity = velocity - v_err/100.0;
    omega = omega - w_err;
    gamma = gamma_err;

    //modelling noise
    velocity = velocity + gaussion_sampling(alpha1 * velocity + alpha2 * omega);
    omega = omega + gaussion_sampling(alpha3 * velocity + alpha4 * omega);
    gamma = gamma + gaussion_sampling(alpha5 * velocity + alpha6 * omega);


    if(velocity > 0.01){
      velocity  = 0.01;
    }

    //signs have been reversed here
    x = x + ( velocity * sin(th) / omega) - (velocity / omega) * sin( th + (omega * dt));
    y = y - ( velocity * cos(th) / omega) + (velocity / omega) * cos( th + (omega * dt));

    th = th + (omega * dt) + (gamma * dt);


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
    odom.twist.twist.linear.x = velocity * cos(th);
    odom.twist.twist.linear.y = velocity * sin(th) ;
    odom.twist.twist.angular.z = omega;

    //publish the message
    odom_pub.publish(odom);

    last_time = current_time;
    r.sleep();
  }
}