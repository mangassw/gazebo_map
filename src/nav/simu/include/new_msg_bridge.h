

#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf2/utils.h>
#include <geometry_msgs/TwistStamped.h>


// #include "xj_robot_simu/fusion_analysis.h"

constexpr static const char* NODE_NAME = "xj_robot_bridge";
constexpr static const char* SubSpeedTopic = "/cmd_vel";
constexpr static const char* PubSpeedTopic = "/cmd_vel_gazebo";
constexpr static const char* PubOdomTopic = "/state_estimation";

constexpr static const char* FeedbackTopic = "/odom";
constexpr static const char* FusionTopic = "/fusion_analysis";
constexpr static const char* OdomTransform = "/odom_transform";
constexpr static const double WheelSeparation = 0.35;
constexpr static const double WheelRadius = 0.07;
constexpr static const double TimerDuration = 0.2;

float vehicleX = 0;
float vehicleY = 0;
float vehicleZ = 0;
float vehicleRoll = 0;
float vehiclePitch = 0;
float vehicleYaw = 0;
ros::Time odomTime;