#include <vector>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <robot_model/motor_voltage.h>
#include <robot_model/velocity.h>
#include "nav_msgs/OccupancyGrid.h"
#include "tf/transform_datatypes.h"

static std::vector< std::vector<float> > path_pts(0);
int path_pts_length;
int is_path_received;
double roll, pitch, yaw;
double robot_pose_x , robot_pose_y, robot_yaw;
double robot_velocity_linear_x, robot_velocity_linear_y, robot_velocity_angular_z;
double radial_velocity;

float alpha1 = 0.1;
float alpha2 = 0.1;
float alpha3 = 0.1;
float alpha4 = 0.1;
float alpha5 = 0.1;
float alpha6 = 0.1;

void posecallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  robot_pose_x = msg->pose.pose.position.x;
  robot_pose_y = msg->pose.pose.position.y;
  robot_velocity_linear_x = msg->twist.twist.linear.x;
  robot_velocity_linear_y = msg->twist.twist.linear.y;
  robot_velocity_angular_z = msg->twist.twist.angular.z;
  ROS_INFO("Angular z %f", robot_velocity_angular_z);

  tf::Quaternion quat(msg-> pose.pose.orientation.x, msg-> pose.pose.orientation.y, msg-> pose.pose.orientation.z, msg-> pose.pose.orientation.w);

  tf::Matrix3x3 m(quat);
  m.getRPY(roll, pitch, yaw);

  ROS_INFO("Roll %f Pitch %f Yaw %f", roll, pitch, yaw);
}


void pathcallback (const nav_msgs::Path::ConstPtr& path)
{
  path_pts.clear();
  int i,j;
  
  for(i = 0; i < path->poses.size(); i++ ){
    float temp[2];
    temp[0] = path->poses[i].pose.position.x;
    temp[1] = path->poses[i].pose.position.y;
    std::vector<float> start_vect(temp, temp + sizeof(temp) / sizeof(float) );
    path_pts.push_back(start_vect);  
  }
  path_pts_length = path->poses.size();
  is_path_received = 1;
  ROS_INFO("Received path from planner");
}

double gaussion_probability(double value, double deviation)
{
  double temp5 = 0.0;
  if(deviation < 0.0001){
    ROS_WARN("inside deviation");
    deviation = 0.0001;
  }
  ROS_INFO("Value %f deviation %f", value, deviation);
  double temp1 = 1.0 / (sqrt(2 * 3.1457 * deviation));
  double temp3 = -0.5 * value * value / deviation;
  double temp2 = exp(temp3);
  ROS_INFO("temp1 %f temp2 %f temp3 %f", temp1, temp2, temp3);

  ROS_INFO("value %f", temp1 * temp2);
  return temp1 * temp2;
}


int main(int argc, char** argv){
  ros::init(argc, argv, "compute_node");
  ros::NodeHandle n;
  ros::Subscriber sub1 = n.subscribe("path_planner", 10, pathcallback);
  ros::Subscriber sub2 = n.subscribe("odom", 10, posecallback);

  double x = robot_pose_x;
  double y = robot_pose_y;
  double th = yaw;

  double desired_heading;
  double desired_velocity;

  double left_motor_voltage, right_motor_voltage;

  double target_x, target_y;

  //velocity motion model
  double u;
  double xcenter, ycenter;
  double radius;
  double delta_q;
  double v_estm, w_estm, gamma_estm;
  double dt = 0.1;


  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();
  
  int i = 0;
  ros::Rate r(10.0);
  while(n.ok()){

    ros::spinOnce();               // check for incoming messages

    if(is_path_received)
    {
      target_x = path_pts[i][0];
      target_y = path_pts[i][1];
      double dist = sqrt(pow(target_x - robot_pose_x,2) + pow(target_y - robot_pose_y, 2));
      if(dist < 0.01){
        i = i + 1;

        if(i == path_pts_length){
          ROS_INFO("Target Acheived......stopping the process");
          break;
        }else{
          target_x = path_pts[i][0];
          target_y = path_pts[i][1];
        }
      }

      ROS_INFO("Current target Point %d - (%f,%f)", i, target_x, target_y);


      desired_heading = atan2(target_y - robot_pose_y, target_x - robot_pose_x) ;
      ROS_INFO("Desired heading %f", desired_heading * 180/3.1457);

      u = 0.5 * ((robot_pose_x - target_x) *  cos(yaw) + (robot_pose_y - target_y) * sin(yaw)) / 
          ((robot_pose_y - target_y) * cos(yaw) - (robot_pose_x - target_x) * sin(yaw));

      xcenter = (target_x + robot_pose_x)/2 + u * (robot_pose_y - target_y); 
      ycenter = (target_y + robot_pose_y)/2 + u * (target_x - robot_pose_x);

      // ROS_INFO("xcenter %f ycenter %f", xcenter, ycenter);

      radius = sqrt(pow((robot_pose_x - xcenter), 2) + pow((robot_pose_y - ycenter), 2));
      // ROS_INFO("radius %f", radius);

      delta_q = atan2(target_y - ycenter, target_x - xcenter) - atan2(robot_pose_y - ycenter, robot_pose_x - xcenter);


      // ROS_WARN("Yaw in degrees %f ", yaw * 180/3.1457);
      // ROS_WARN("Robot pose in degrees %f ", atan2(robot_pose_y - ycenter, robot_pose_x - xcenter) * 180/3.1457);

      float yaw_shifted;
      //holy crap ...quaternion calculates heading in other direction
      // yaw_shifted = 1.5707 - (yaw * -1);
      yaw_shifted = yaw + 3.1457;

      // ROS_WARN("Delta in degrees %f", delta_q * 180/3.1457);
      v_estm = delta_q * radius / dt;
      w_estm = delta_q / dt;
      gamma_estm =  ((desired_heading - yaw_shifted)/ dt) - w_estm;

      radial_velocity = sqrt(pow(robot_velocity_linear_x,2) + pow(robot_velocity_linear_y,2));

      // ROS_WARN("radial velocity %f ", radial_velocity);
      ros::ServiceClient velocity_client;
      robot_model::velocity temp;

      temp.request.velocity_error = radial_velocity - v_estm;
      temp.request.omega_error = robot_velocity_angular_z - w_estm;
      temp.request.gamma_error = gamma_estm; 

      if (ros::service::call("set_velocity", temp))
      {
        double prob_v , prob_w, prob_g;
        prob_v = gaussion_probability(radial_velocity - v_estm, alpha1 * radial_velocity + alpha2 * robot_velocity_angular_z);
        prob_w = gaussion_probability(robot_velocity_angular_z - w_estm, alpha3 * radial_velocity + alpha4 * robot_velocity_angular_z);
        prob_g = gaussion_probability(gamma_estm, alpha5 * radial_velocity + alpha6 * robot_velocity_angular_z);
        ROS_INFO("Gaussion probabilities v:%f w:%f g:%f", prob_v, prob_w, prob_g);     
      }
      else
      {
        ROS_ERROR("Failed to set desired velocities");
      }

    }
    
    
    r.sleep();
  }
}


