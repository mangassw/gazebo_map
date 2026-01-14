/**
 * *********************************************************
 *
 * @file: rpp_controller.h
 * @brief: Contains the regulated_pure_pursuit (RPP) local controller class
 * @author: Yang Haodong
 * @date: 2024-01-08
 * @version: 1.0
 *
 * Copyright (c) 2024, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#ifndef RPP_PKG_RPP_CONTROLLER_H_
#define RPP_PKG_RPP_CONTROLLER_H_


#include <tf2/utils.h>
#include <nav_core/base_local_planner.h>
#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <base_local_planner/odometry_helper_ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include "rpp_pkg/controller.h"
#include <deque>


//==================================================================================
namespace rpp_pkg
{
namespace controller
{
class RPPController : public nav_core::BaseLocalPlanner, public Controller
{
public:
  RPPController();
  RPPController(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);
  ~RPPController();

  void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);
  bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);
  bool isGoalReached();
  bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

protected:
  double _dphi(geometry_msgs::PointStamped lookahead_pt, geometry_msgs::PoseStamped robot_pose_global);

private:
  double _applyCurvatureConstraint(const double raw_linear_vel, const double curvature);
  double _applyObstacleConstraint(const double raw_linear_vel);
  double _applyApproachConstraint(const double raw_linear_vel, geometry_msgs::PoseStamped robot_pose_global,
                                  const std::vector<geometry_msgs::PoseStamped>& prune_plan);

private:

//===========================1维卡尔曼滤波==================================
    struct KalmanFilter {
        double x;      // 状态估计（速度）
        double P;      // 估计误差协方差
        double Q;      // 过程噪声
        double R;      // 测量噪声
        
        void init(double initial_vel = 0.0) {
            x = initial_vel;
            P = 1.0;
            Q = 0.001;  // 速度预测噪声（可调）0.01 （更相信预测）
            R = 1.0;   // 速度测量噪声（可调）0.5   （更相信测量）
        }
        
        double update(double measurement) {
            double x_pred = x;
            double P_pred = P + Q;
            double K = P_pred / (P_pred + R);
            x = x_pred + K * (measurement - x_pred);
            P = (1 - K) * P_pred;
            return x;
        }
    };
//===========================2维度卡尔曼滤波器================================
     // 二维坐标卡尔曼滤波器结构
    struct KalmanFilter2D {
        // 状态估计值 (x, y)
        double x, y;
        // 状态估计误差协方差
        double P_x, P_y;
        // 过程噪声协方差
        double Q_x, Q_y;
        // 测量噪声协方差
        double R_x, R_y;

        // 初始化
        void init(double initial_x = 0.0, double initial_y = 0.0) {
            x = initial_x; y = initial_y;
            P_x = 1.0; P_y = 1.0;
            Q_x = 0.01; Q_y = 0.01;    // 过程噪声（可调）
            R_x = 1.0; R_y = 1.0;    // 测量噪声（可调）
        }

        // 更新滤波
        void update(double measurement_x, double measurement_y) {
            // X 坐标滤波
            double x_pred = x;
            double P_pred_x = P_x + Q_x;
            double K_x = P_pred_x / (P_pred_x + R_x);
            x = x_pred + K_x * (measurement_x - x_pred);
            P_x = (1 - K_x) * P_pred_x;

            // Y 坐标滤波
            double y_pred = y;
            double P_pred_y = P_y + Q_y;
            double K_y = P_pred_y / (P_pred_y + R_y);
            y = y_pred + K_y * (measurement_y - y_pred);
            P_y = (1 - K_y) * P_pred_y;
        }
    };

    // 为 lookahead_pt 和 slowdown_pt 创建独立的滤波器
    KalmanFilter2D lookahead_filter_;
    KalmanFilter2D slowdown_filter_;
    KalmanFilter k_vel_filter_;
    // MovingAverageFilter a_vel_filter_;
    bool filter_initialized_ = false;  // 滤波器初始化标志

//===========================================================================
  bool initialized_;     // initialized flag
  bool goal_reached_;    // goal reached flag
  tf2_ros::Buffer* tf_;  // transform buffer

  double d_t_;                          // control time interval
  double regulated_min_radius_;         // the threshold to apply curvature constraint
  double inflation_cost_factor_;        // the heuristical factor to calculate minimum distance to obstacles
  double scaling_dist_, scaling_gain_;  // the threshold to apply obstacle constraint
  double approach_dist_;                // the threshold to apply approaching goal constraint
  double approach_min_v_;               // minimum approaching velocity

  ros::Publisher target_pt_pub_, current_pose_pub_,slowdown_pt_pub_;

  // goal parameters
  double goal_x_, goal_y_, goal_theta_;




};



}  // namespace rpp_planner
}  // namespace rpp_pkg
#endif