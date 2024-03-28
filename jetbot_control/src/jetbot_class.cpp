#include "jetbot_control/jetbot_class.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int16.h"
#include "unistd.h"
#include <limits> // for numeric_limits
#include <csignal> // for signal handling
#include <ros/ros.h>

#include <list>
#include <string>

using namespace std;

// JetbotClass constructor
JetbotClass::JetbotClass() {
  n = ros::NodeHandle("~");
  laser_topic = "/scan";
  laser_sub = n.subscribe(laser_topic, 10, &JetbotClass::laser_callback, this);
  vel_topic = "/cmd_vel";
  vel_pub = n.advertise<geometry_msgs::Twist>(n.resolveName(vel_topic), 1);
  odom_topic = "/odom";
  odom_sub = n.subscribe(odom_topic, 10, &JetbotClass::odom_callback, this);
  ROS_INFO("Initializing node .................................");
  usleep(2000000);
  ros::spinOnce();
}

void JetbotClass::laser_callback(
    const sensor_msgs::LaserScan::ConstPtr &laser_msg) {
  laser_range = laser_msg->ranges;
  //ROS_INFO_STREAM(laser_range.size());
}
void JetbotClass::odom_callback(const nav_msgs::Odometry::ConstPtr &odom_msg) {
  x_pos = odom_msg->pose.pose.position.x;
  y_pos = odom_msg->pose.pose.position.y;
  // ROS_INFO_STREAM("Odometry: x=" << x_pos << " y=" << y_pos);
}

void JetbotClass::move_forward(int time) {
  // Rate of publishing
  ros::Rate rate(10);

  ros::Time start_time = ros::Time::now();
  ros::Duration timeout(time);
  while (ros::Time::now() - start_time < timeout) {
    ROS_INFO_STREAM("Moving forward ........... ");
    ros::spinOnce();
    vel_msg.linear.x = 0.5;
    vel_msg.angular.z = 0.0;
    vel_pub.publish(vel_msg);
    rate.sleep();
  }
  vel_msg.linear.x = 0.0;
  vel_msg.angular.z = 0.0;
  vel_pub.publish(vel_msg);
}

void JetbotClass::move_backwards(int time) {
  // Rate of publishing
  ros::Rate rate(10);

  ros::Time start_time = ros::Time::now();
  ros::Duration timeout(time);
  while (ros::Time::now() - start_time < timeout) {
    ROS_INFO_STREAM("Moving backwards ........... ");
    ros::spinOnce();
    vel_msg.linear.x = -0.5;
    vel_msg.angular.z = 0.0;
    vel_pub.publish(vel_msg);
    rate.sleep();
  }
  vel_msg.linear.x = 0.0;
  vel_msg.angular.z = 0.0;
  vel_pub.publish(vel_msg);
}

void JetbotClass::turn(string clock, int n_secs) {
  ros::Rate rate(10);
  ros::Time start_time = ros::Time::now();
  ros::Duration timeout(n_secs);

  double WZ = 0.0;
  if (clock == "clockwise") {
    ROS_INFO_STREAM("Turning clockwise ........... ");
    WZ = -0.4;
  } else if (clock == "counterclockwise") {
    ROS_INFO_STREAM("Turning counterclockwise ........... ");
    WZ = 0.4;
  }

  while (ros::Time::now() - start_time < timeout) {
    ros::spinOnce();
    vel_msg.linear.x = 0.5;
    vel_msg.angular.z = WZ;
    vel_pub.publish(vel_msg);
    rate.sleep();
  }
  vel_msg.linear.x = 0.0;
  vel_msg.angular.z = 0.0;
  vel_pub.publish(vel_msg);
}

void JetbotClass::rotate(string clock, int n_secs){
  ros::Rate rate(10);
  ros::Time start_time = ros::Time::now();
  ros::Duration timeout(n_secs);

  double WZ = 0.0;
  if (clock == "clockwise"){
    ROS_INFO_STREAM("Rotating clockwise");
    WZ = -0.125;
  } else if (clock == "counterclockwise"){
    ROS_INFO_STREAM("Rotating counterclockwise");
    WZ = 0.125;
  }

  while (ros::Time::now() - start_time < timeout){
    ros::spinOnce();
    vel_msg.linear.x = 0.0;
    vel_msg.angular.z = WZ;
    vel_pub.publish(vel_msg);
    rate.sleep();
  }
  vel_msg.linear.x = 0.0;
  vel_msg.angular.z = 0.0;
  vel_pub.publish(vel_msg);
}

void JetbotClass::stop_moving() {
  ROS_INFO_STREAM("Stopping the robot ........... ");
  vel_msg.linear.x = 0.0;
  vel_msg.angular.z = 0.0;
  vel_pub.publish(vel_msg);
}

float JetbotClass::get_position(int param) {
  if (param == 1) {
    return this->x_pos;
  } else if (param == 2) {
    return this->y_pos;
  } 
  return 0;
}

list<float> JetbotClass::get_position_full() {
  list<float> coordinates({this->x_pos, this->y_pos});
  return coordinates;
}

float JetbotClass::get_laser(int index) { 
  return laser_range[index]; 
}

float *JetbotClass::get_laser_full() {
  float *laser_range_pointer = laser_range.data();
  return laser_range_pointer;
}

// Global variable to indicate whether Ctrl+C is pressed
bool g_stop_requested = false;

// Signal handler for Ctrl+C
void sigintHandler(int sig) {
    g_stop_requested = true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "jetbot_class_node");

  JetbotClass jetbot;

  // ~35 cm translation
  //jetbot.move_forward(1);
  //jetbot.move_backwards(1);

  // ~90 degree turn
  //jetbot.turn("clockwise",2);

  // ~90 degree rotation
  //jetbot.rotate("clockwise",1);

  // The size of laser_range is 1147
  // index 0 and 1146 point forward
  // index 286 points left
  // index 573 points backward
  // index 859 points right
  //float x = jetbot.get_laser(0);

  return 0;
}
