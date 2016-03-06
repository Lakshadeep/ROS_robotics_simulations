#include <vector>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <robot_model/motor_voltage.h>
#include "nav_msgs/OccupancyGrid.h"
#include "tf/transform_datatypes.h"

static std::vector< std::vector<float> > path_pts(0);
int path_pts_length;
int is_path_received;
double roll, pitch, yaw;
double robot_pose_x , robot_pose_y, robot_yaw;

void posecallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  robot_pose_x = msg->pose.pose.position.x;
  robot_pose_y = msg->pose.pose.position.y;
  tf::Quaternion quat(msg-> pose.pose.orientation.x, msg-> pose.pose.orientation.y, msg-> pose.pose.orientation.z, msg-> pose.pose.orientation.w);

  tf::Matrix3x3 m(quat);
  m.getRPY(roll, pitch, yaw);

/*  roll = fmod(roll + 6.28, 6.28);
  pitch = fmod(pitch + 6.28, 6.28);
  yaw = fmod(yaw + 6.28, 6.28);
*/
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
        }
      }

      ROS_INFO("Current target Point %d - (%f,%f)", i, target_x, target_y);

      desired_heading = atan2(target_y - robot_pose_y, target_x - robot_pose_x) * 180/3.1457;
      ROS_INFO("Desired heading %f", desired_heading);

      desired_velocity = sqrt(pow(target_x - robot_pose_x, 2) + pow(target_y - robot_pose_y, 2));
      ROS_INFO("Desired velocity  %f ", desired_velocity);

      yaw = yaw * 180/3.1457;

      if(desired_heading > yaw){
        ROS_WARN("Desired heading  - yaw =  %f", desired_heading - yaw);
        right_motor_voltage = ((desired_heading - yaw)/90) * 5;
        left_motor_voltage = 0;
      }
      else if(yaw > desired_heading){
        ROS_WARN("Yaw - desired heading =  %f", yaw - desired_heading);
        left_motor_voltage = ((yaw - desired_heading)/90) * 5;
        right_motor_voltage = 0;
      }

      ros::ServiceClient motor_voltage_client;
      robot_model::motor_voltage temp;
      temp.request.left_motor = left_motor_voltage;
      temp.request.right_motor = right_motor_voltage;
      if (ros::service::call("set_motor_voltage", temp))
      {
        ROS_INFO("Setting desired motor voltages %f %f", left_motor_voltage, right_motor_voltage);     
      }
      else
      {
        ROS_ERROR("Failed to set desired motor voltages");
      }

    }
    
    
    r.sleep();
  }
}


