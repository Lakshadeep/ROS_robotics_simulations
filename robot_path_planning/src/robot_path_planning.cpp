#include <vector>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include "nav_msgs/OccupancyGrid.h"

static std::vector< std::vector<float> > path_pts(0);

float map[332][441] = {0};
int actions[332][441] = {-1};
int expand[332][441] = {-1};

int closed[332][441];

static std::vector< std::vector<int> > open(0);

int start[] = {10,10,0};
int goal[] = {300,300,0};

int delta[4][2] = { {-1, 0}, {0, -1}, {1, 0}, {0, 1}};
int map_ready = 0;

void mapcallback (const nav_msgs::OccupancyGrid::ConstPtr& map_in)
{
  int i,j;
  
  for(i = 0; i < 441; i++ ){
    for(j = 0; j < 332; j++ ){
      map[i][j] = map_in->data[332 * i + j];
      // ROS_INFO("Map in callback %f", map[i][j]);
    }
  }
  map_ready = 1;
  ROS_INFO("Map ready");
}

void path_planner()
{
  ROS_WARN("Planning path for %d %d to %d %d", start[0], start[1], goal[0], goal[1]);
  bool path_success = false;
  bool path_failure = false;

  int step = 0;

  std::vector<int> open_temp(start, start + sizeof(start) / sizeof(int) );

  open.push_back(open_temp);

  closed[start[0]][start[1]] = 1;
  expand[start[0]][start[1]] = step;

  int open_size;
  std::vector<int> next;

  while(!path_success || !path_failure)
  {
    if(open.size() == 0){
      path_failure = true;
      ROS_ERROR("No path to destination");
      break;
    }
    else
    {
  
      open_size = open.size();
      int i, j, flag = 1;    // set flag to 1 to start first pass

      for(i = 1; (i <= open_size) && flag; i++)
      {
        flag = 0;
        for (j = 0; j < (open_size -1); j++)
        {
          if(open[j+1][2] > open[j][2] )      // ascending order simply changes to <
          { 
            int temp[3];
            std::vector<int> temp_vector(temp, temp + sizeof(temp) / sizeof(int) );
            temp_vector = open.at(j);
            open.at(j) = open.at(j+1);
            open.at(j+1) = temp_vector;
            flag = 1;    
          }
        }
      }
      

      

      next = open.at(open_size - 1);  //last element (least step)
      // ROS_WARN("Next %d %d %d",next.at(0),next.at(1),next.at(2) );

      /*ROS_INFO("Elements in open loop b4 pop");
      for(int i = 0; i< open.size(); i++ ){
        ROS_INFO("%d %d %d", open[i][0], open[i][1], open[i][2]);
      }*/

      open.pop_back();
      expand[next[0]][next[1]] = step;
      step = step + 1;


      /*ROS_INFO("Elements in open loop");
      for(int i = 0; i< open.size(); i++ ){
        ROS_INFO("%d %d %d", open[i][0], open[i][1], open[i][2]);
      }
      */

      if(next[0] == goal[0] && next[1] == goal[1])
      {
        path_success = true;
        ROS_INFO("Path successfully planned");
        break;
      }
      else
      {
      
        if(step == 10000000){
          break;
        }
        for (int i = 0; i < 4; i++)
        {
          int temp[3];
          temp[0] = next[0] + delta[i][0];
          temp[1] = next[1] + delta[i][1];
          temp[2] = step;
          
          // ROS_INFO("Map %f Closed %d", map[temp[0]][temp[1]], closed[temp[0]][temp[1]]);
          
          if(temp[0] < 332 && temp[0] >= 0 && temp[1] < 441 && temp[1] >= 0 && map[temp[1]][temp[0]] < 99 && closed[temp[0]][temp[1]] == 0)
          {
            std::vector<int> temp_vector(temp, temp + sizeof(temp) / sizeof(int) ); 
            open.push_back(temp_vector);   
            closed[temp[0]][temp[1]] = 1;
            actions[temp[0]][temp[1]] = i;

            // ROS_INFO("Open list new item %d %d %d", temp[0], temp[1], step);
          }

        }
      }


    }
  }
  /*ROS_INFO("Actions matrix");
  for(int i = 0; i <  332; i++ ){
    printf("[");
    for(int j = 0; j<  441; j++ ){
      printf("%d ", actions[i][j]);
    }
    printf("]\n");
  }*/

  /*ROS_INFO("Expand matrix");
  for(int i = 0; i <  332; i++ ){
    printf("[");
    for(int j = 0; j<  441; j++ ){
      printf("%d ", expand[i][j]);
    }
    printf("]\n");
  }*/


  int target[3];
  target[0] = next[0];
  target[1] = next[1];
  target[2] = next[2];

  float target_f[3];
  target_f[0] = target[0]/100.0;
  target_f[1] = target[1]/100.0;
  target_f[2] = target[2];

  std::vector<float> target_vect(target_f, target_f + sizeof(target_f) / sizeof(float) );
  path_pts.push_back(target_vect);
  
  int current[3];

  while(target[0] != start[0] && target[1] != start[1])
  {
    ROS_INFO("Delta %d %d",delta[actions[target[0]][target[1]]][0],delta[actions[target[0]][target[1]]][1]);
    current[0] = target[0] - delta[actions[target[0]][target[1]]][0]; 
    current[1] = target[1] - delta[actions[target[0]][target[1]]][1];
    current[2] = 0;
    target[0] = current[0];
    target[1] = current[1];
    target[2] = current[2];

    float current_f[3];
    current_f[0] = current[0]/100.0;
    current_f[1] = current[1]/100.0;
    current_f[2] = current[2];

    ROS_WARN("Adding point to path x: %f y: %f", current_f[0], current_f[1]);

    std::vector<float> current_vect(current_f, current_f + sizeof(current_f) / sizeof(int) );
    path_pts.push_back(current_vect);

  }

  float start_f[3];
  start_f[0] = start[0]/100.0;
  start_f[1] = start[1]/100.0;
  start_f[2] = start[2];

  std::vector<float> start_vect(start_f, start_f + sizeof(start_f) / sizeof(float) );
  path_pts.push_back(start_vect);

  ROS_INFO("Total path length %d", path_pts.size());


}

int main(int argc, char** argv){
  ros::init(argc, argv, "path_planner");
  ros::NodeHandle n;
  ros::Subscriber sub1 = n.subscribe("map", 10, mapcallback);
  ros::Publisher odom_pub = n.advertise<nav_msgs::Path>("path_planner", 50);

  double x = 0.0;
  double y = 0.0;
  double th = 0.0;

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();
  ROS_WARN("Debug 1");
  
  ros::Rate r(0.1);
  while(n.ok()){

    ros::spinOnce();               // check for incoming messages
    
    path_pts.clear();
    open.clear();
    
    for(int i = 0; i < 332 ; i++){
      for(int j = 0; j < 441 ; j++){
        actions[i][j] = -1;
        expand[i][j] = -1;
        closed[i][j] = 0;
      }
    }
    if(map_ready == 1)
      path_planner();
    current_time = ros::Time::now();

    //since all odometry is 441DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    nav_msgs::Path path;
    path.header.stamp = current_time;
    path.header.frame_id = "odom";

    path.poses.resize(path_pts.size());

    geometry_msgs::Pose pos;

    ROS_INFO("Debug 1");

    for(int i = path_pts.size() - 1; i >= 0; i--)
    {
      pos.position.x = path_pts[i][0];
      pos.position.y = path_pts[i][1];
      pos.position.z = 0;
      pos.orientation = odom_quat;

      geometry_msgs::PoseStamped posestamp;
      posestamp.pose = pos;

      path.poses[i] = posestamp;
    }

    ROS_INFO("Debug 2");


    //publish the message
    odom_pub.publish(path);

    ROS_INFO("Path published");

    last_time = current_time;
    r.sleep();
  }
}


