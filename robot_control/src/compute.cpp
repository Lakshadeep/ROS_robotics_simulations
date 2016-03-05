#include <vector>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include "nav_msgs/OccupancyGrid.h"
#include "robot_navigation/desired_heading_velocity.h"

static std::vector< std::vector<float> > path_pts(0);
int path_pts_length;
int is_path_received;


void pathcallback (const nav_msgs::Path::ConstPtr& path)
{
  // ROS_WARN("Path pts size %d" , path_pts.size());
  // ROS_WARN("Size %d", path->poses.size());
  path_pts.clear();
  // ROS_WARN("Path pts size %d" , path_pts.size());
  int i,j;
  
  for(i = 0; i < path->poses.size(); i++ ){
    float temp[2];
    temp[0] = path->poses[i].pose.position.x;
    temp[1] = path->poses[i].pose.position.y;
    // ROS_INFO("Sample data %f %f ", temp[0], temp[1]);
    std::vector<float> start_vect(temp, temp + sizeof(temp) / sizeof(float) );
    path_pts.push_back(start_vect);
    // ROS_INFO("Sample data from path_pts %f %f ", path_pts.at(i).at(0), path_pts.at(i).at(1));
  
  }
  path_pts_length = path->poses.size();
  is_path_received = 1;
  ROS_INFO("Received path from planner");
}


int main(int argc, char** argv){
  ros::init(argc, argv, "compute_node");
  ros::NodeHandle n;
  ros::Subscriber sub1 = n.subscribe("path_planner", 10, pathcallback);
  // ros::Publisher odom_pub = n.advertise<nav_msgs::Path>("path_planner", 50);

  double x = 0.0;
  double y = 0.0;
  double th = 0.0;

  double desired_heading;
  double desired_velocity_x;
  double desired_velocity_y;

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();
  
  int i = 0;
  ros::Rate r(1);
  while(n.ok()){

    ros::spinOnce();               // check for incoming messages

    if(is_path_received)
    {
      ROS_INFO("Path point %d %f %f", i, path_pts[i][1], path_pts[i][0]);
      if(i == 0)
        desired_heading = atan2(path_pts[i][1] - 0, path_pts[i][0] - 0) * 180/3.1457;
      else
        desired_heading = atan2(path_pts[i][1] - path_pts[i - 1][1], path_pts[i][0] - path_pts[i - 1][0]) * 180/3.1457;

      ROS_INFO("Desired heading %f", desired_heading);
      if(i== 0){
        desired_velocity_x = path_pts[i][0];
        desired_velocity_y = path_pts[i][1];
      }else{
        desired_velocity_x = path_pts[i][0] - path_pts[i - 1][0];
        desired_velocity_y = path_pts[i][1] - path_pts[i - 1][1];
      }

      ROS_INFO("Desired velocity  x %f y %f", desired_velocity_x, desired_velocity_y);

      ros::ServiceClient desired_heading_velocity_client;
      robot_navigation::desired_heading_velocity temp;
      temp.request.desired_velocity_x = desired_velocity_x;
      temp.request.desired_velocity_y = desired_velocity_y;
      temp.request.desired_heading = desired_heading;
      // cooperative_data_client = n.serviceClient<auv::cooperative_data_srv>("cooperative_data");
      if (ros::service::call("desired_heading_velocity", temp))
      {
        ROS_INFO("Setting desired heading and velocity");
        i++;     
      }
      else
      {
        ROS_ERROR("Failed to st desired heading and velocity");
      }

    }
    
    
    r.sleep();
  }
}


