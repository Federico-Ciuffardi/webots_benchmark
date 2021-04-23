/*
 * To run gmapping you should start gmapping:
 * rosrun gmapping slam_gmapping scan:=lidar_topic _xmax:=30 _xmin:=-30 _ymax:=30 _ymin:=-30
 * _delta:=0.2
 */

#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PointStamped.h>
#include <signal.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>
#include "ros/ros.h"
#include <ros/console.h>

#include <utility>
#include <webots_ros/set_float.h>
#include <webots_ros/set_int.h>

#include <math.h>
#include <stdexcept>      // std::out_of_range
#include <vector>         // std::vector

#define TIME_STEP 32
#define MAX_SPEED 6.4
#define OBSTACLE_THRESHOLD 0.1
#define DECREASE_FACTOR 0.9
#define BACK_SLOWDOWN 0.9
#define NMOTORS 2

using namespace std;

static const char *motorNames[NMOTORS] = {"right_wheel", "left_wheel"};

ros::NodeHandle *n;

static std::vector<float> lidarValues;

ros::ServiceClient timeStepClient;
webots_ros::set_int timeStepSrv;

enum State { FORWARD , BACKWARD };
int state = FORWARD;
static const float targets[2] = {180, 0};

// motors
void updateSpeed(float linearSpeed) {
  float rotationalSpeed = linearSpeed/0.0975 ;// since the speed value set on the wheel corresponds to the rotational speed in radians per second
                                              // and the radius of a wheel is 0.0975m then the speed is ajusted acordingly
  for (int i = 0; i < NMOTORS; ++i) {
    ros::ServiceClient set_velocity_client;
    webots_ros::set_float set_velocity_srv;
    set_velocity_client = n->serviceClient<webots_ros::set_float>(std::string(motorNames[i]) +
                                                                  std::string("/set_velocity"));
    set_velocity_srv.request.value = rotationalSpeed;
    if (!set_velocity_client.call(set_velocity_srv) || !set_velocity_srv.response.success){
      ROS_ERROR("Failed to call service set_velocity on motor %s.", motorNames[i]);
    }    
  }
}

// lidar
void lidarCallback(const sensor_msgs::LaserScan::ConstPtr &scan) {
  float angleIncrement = scan->angle_increment;
  float angleMin = scan->angle_min;

  int target_laser_index;

  target_laser_index = scan->ranges.size()*(targets[state]/360.0);

  /* ROS_INFO_STREAM_ONCE("angleIncrement: " << angleIncrement ); */
  /* ROS_INFO_STREAM_ONCE("angleMin: " << angleMin ); */

  /* int target_laser_index = (target - angleMin) / angleIncrement; */

  /* ROS_INFO_STREAM("target_laser_index: " << target_laser_index ); */
  /* ROS_INFO_STREAM("target_laser_index angle: " << (target_laser_index*angleIncrement) + angleMin ); */

  float distance;
  try {
    distance = scan->ranges.at(target_laser_index);
    /* ROS_INFO_STREAM("distance to obstacle ahead: " << distance); */
  } catch (const out_of_range& oor){
    ROS_ERROR_STREAM("Out of Range error: " << oor.what());
    ros::shutdown();
    exit(1);
  }

  bool tooClose = scan->ranges[target_laser_index] < 1;

  state = (state + tooClose ) % 2;

  float speed = min(scan->ranges[scan->ranges.size()*(targets[BACKWARD]/360.0)], scan->ranges[scan->ranges.size()*(targets[FORWARD]/360.0)])/5;
  if ( state == BACKWARD ){
    speed = -speed;
  }
  updateSpeed(speed);
  
}

// at SIGINT quit
void quit(int sig) {
  ROS_INFO("User stopped the node.");
  timeStepSrv.request.value = 0;
  timeStepClient.call(timeStepSrv);
  ros::shutdown();
  exit(0);
}

int main(int argc, char **argv) {
  std::string controllerName;
  ros::init(argc, argv, "benchmark_controller");
  n = new ros::NodeHandle();
  ros::Duration(5).sleep(); // first ROS_INFO_STREAM did not show on rqt_console and
                            // also webots services failed otherwise 

  string ns = ros::this_node::getNamespace();
  ROS_INFO_STREAM("ns = " << ns);

  timeStepClient = n->serviceClient<webots_ros::set_int>("robot/time_step");
  timeStepSrv.request.value = TIME_STEP;

  signal(SIGINT, quit);
  // init motors
  for (int i = 0; i < NMOTORS; ++i) {
    // position
    ros::ServiceClient set_position_client;
    webots_ros::set_float set_position_srv;
    set_position_client = n->serviceClient<webots_ros::set_float>(std::string(motorNames[i]) +
                                                                  std::string("/set_position"));

    set_position_srv.request.value = INFINITY; // set endless motion controlled to be controlled with the velocity 
    if (set_position_client.call(set_position_srv) && set_position_srv.response.success)
      ROS_INFO("Position set to INFINITY for motor %s.", motorNames[i]);
    else
      ROS_ERROR("Failed to call service set_position on motor %s.", motorNames[i]);

    // speed
    ros::ServiceClient set_velocity_client;
    webots_ros::set_float set_velocity_srv;
    set_velocity_client = n->serviceClient<webots_ros::set_float>(std::string(motorNames[i]) +
                                                                  std::string("/set_velocity"));

    set_velocity_srv.request.value = 1.0;
    if (set_velocity_client.call(set_velocity_srv) && set_velocity_srv.response.success == 1)
      ROS_INFO("Velocity set to 0.0 for motor %s.", motorNames[i]);
    else
      ROS_ERROR("Failed to call service set_velocity on motor %s.", motorNames[i]);
  }

  // enable lidar
  ros::ServiceClient set_lidar_client;
  webots_ros::set_int lidar_srv;
  ros::Subscriber sub_lidar_scan;

  set_lidar_client = n->serviceClient<webots_ros::set_int>("laser/enable");
  lidar_srv.request.value = TIME_STEP;
  if (set_lidar_client.call(lidar_srv) && lidar_srv.response.success) {
    ROS_INFO("Lidar enabled.");
    sub_lidar_scan = n->subscribe("laser/laser_scan/layer0", 10, lidarCallback);
    ROS_INFO("Topic for lidar initialized.");
    while (sub_lidar_scan.getNumPublishers() == 0) {
    }
    ROS_INFO("Topic for lidar scan connected.");
  } else {
    if (!lidar_srv.response.success)
      ROS_ERROR("Sampling period is not valid.");
    ROS_ERROR("Failed to enable lidar.");
    return 1;
  }

  // main loop
  ROS_INFO("Initialized!");
  while (ros::ok()) {
    if (!timeStepClient.call(timeStepSrv) || !timeStepSrv.response.success) {
      ROS_ERROR("FAILED to call service time_step for next step");
      break;
    }
    ros::spinOnce();
  }
  timeStepSrv.request.value = 0;
  timeStepClient.call(timeStepSrv);

  ros::spin();
  ros::shutdown();
  return 0;
}
