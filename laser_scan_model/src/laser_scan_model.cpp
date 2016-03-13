#include "ros/ros.h"
#include <sstream>
#include "geometry_msgs/Pose2D.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"
#include "tf/transform_datatypes.h"
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include "sensor_msgs/PointCloud.h"
#include <tf/transform_broadcaster.h>
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "laser_geometry/laser_geometry.h"

#define degtorad 3.1457/180

float map[758][984];

void mapcallback (const nav_msgs::OccupancyGrid::ConstPtr& map_in)
{
  // int size = 758*984;
  int i,j;
  
  for(i = 0; i < 441; i++ ){
    for(j = 0; j < 332; j++ ){
      map[i][j] = map_in->data[332 * i + j];
      // ROS_INFO("Map in callback %f", map[i][j]);
    }
  }
}


double roll, pitch, yaw;
double robot_pose_x , robot_pose_y, robot_yaw;
double robot_velocity_linear_x, robot_velocity_linear_y, robot_velocity_angular_z;
double radial_velocity;

void posecallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  robot_pose_x = msg->pose.pose.position.x;
  robot_pose_y = msg->pose.pose.position.y;
  robot_velocity_linear_x = msg->twist.twist.linear.x;
  robot_velocity_linear_y = msg->twist.twist.linear.y;
  robot_velocity_angular_z = msg->twist.twist.angular.z;

  tf::Quaternion quat(msg-> pose.pose.orientation.x, msg-> pose.pose.orientation.y, msg-> pose.pose.orientation.z, msg-> pose.pose.orientation.w);

  tf::Matrix3x3 m(quat);
  m.getRPY(roll, pitch, yaw);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "laser_scan_publisher");

  ros::NodeHandle n;
  ros::Publisher scan_pub = n.advertise<sensor_msgs::LaserScan>("scan", 50);
  ros::Subscriber sub1 = n.subscribe("map", 10, mapcallback);
  ros::Subscriber sub2 = n.subscribe("odom", 10, posecallback);

  // laser_geometry::LaserProjection projector;
  // tf::TransformListener listener;



  unsigned int num_readings = 144 ;
  double laser_frequency = 40;
  double ranges[num_readings];
  double intensities[num_readings];

  int count = 0;
  ros::Rate r(1);
  while(n.ok()){
    //generate some fake data for our laser scan
    for(unsigned int i = 0; i < num_readings; ++i){
      ranges[i] = i * 0.1;
      intensities[i] = 100 + count;
    }
    ros::Time scan_time = ros::Time::now();

    //populate the LaserScan message
    sensor_msgs::LaserScan scan;
    scan.header.stamp = scan_time;
    scan.header.frame_id = "odom";
    scan.angle_min = 0.785;
    scan.angle_max = 7.065;
    scan.angle_increment = 6.28 / num_readings;
    scan.time_increment = (1 / laser_frequency) / (num_readings);
    scan.range_min = 0.2;
    scan.range_max = 1.5;

    scan.ranges.resize(num_readings);
    scan.intensities.resize(num_readings);
    
    float th;
    float a = 100.0;
    for(unsigned int i = 0; i < num_readings; i++){
      int dist;
      float x = robot_pose_x * 100,y = robot_pose_y* 100;
      th = (i * 6.28/ num_readings);

      // ROS_WARN("Theta %f", th * 180/3.14);
      // ROS_WARN("x %f y %f", x, y);

      
      while(1){
        // ROS_INFO("Map %d", map[abs(x)][abs(y)]);
        x = x + (0.01 * cos(th) - 0.01 * sin(th));
        y = y + (0.01 * sin(th) + 0.01 * cos(th));
        // ROS_INFO("Co-ordinates %d %d", abs(x),abs(y));

        if(x > 332 || x <= 0) break;
        if(y > 431 || y <= 0) break;

        if(x < 332 && x > 0 && y < 441 && y > 0)
        {
          if(map[abs(y)][abs(x)] > 99){
            // ROS_WARN("obstacles detected");
            scan.ranges[i] = sqrt(pow(x - 0,2) + pow(y - 0,2))/100;
            break;
          }
          else{
            scan.ranges[i] = 2;
          }
        }
      }
      // sensor_msgs::PointCloud cloud;
      // projector.transformLaserScanToPointCloud("odom",scan,  cloud,listener);

      scan.intensities[i] = 0;
    }

    scan_pub.publish(scan);
    ++count;
    ros::spinOnce();
    r.sleep();
  }
}