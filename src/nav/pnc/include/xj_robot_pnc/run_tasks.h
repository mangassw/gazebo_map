#pragma once

#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>
#include <vector>
#include <std_msgs/Int32.h>
#include "sensor_msgs/Joy.h"

#include "costmap_2d/costmap_2d.h"
#include "costmap_2d/costmap_2d_ros.h"
#include "mbf_msgs/MoveBaseAction.h"
#include "mbf_msgs/ExePathAction.h"
#include "xj_robot_pnc/exe_path.h"
#include "xj_robot_pnc/record_start.h"
#include "xj_robot_pnc/record_stop.h"
#include <iostream>
#include <fstream>
#include <ctime>
#include <iomanip>

namespace xj_robot {
constexpr static const char* DEFAULT_DIR = "/home/art/xj_robot/PATH";

constexpr static const char* NODE_NAME = "xj_robot_tasks";
constexpr static const char* RECORD_START_SRV = "xj_robot_record_start";
constexpr static const char* RECORD_STOP_SRV = "xj_robot_record_stop";
constexpr static const char* EXE_PATH_SRV = "xj_robot_task";
constexpr static const char* COSTMAP = "/move_base_flex/global_costmap/costmap";
constexpr static const char* COSTMAP_UPDATE = "/move_base_flex/global_costmap/costmap_updates";


constexpr static const double DEG2RAD = M_PI / 180;
constexpr static const double RECORD_PATH_LEN_DENS = 0.05;
constexpr static const double RECORD_PATH_AGU_DENS = 10 * DEG2RAD;
constexpr static const int PATH_SAFE_DIS_NUM = 1.3 / RECORD_PATH_LEN_DENS; // 1.3m
constexpr static const int WAIT_COUNT = 50;


using GotoCtrl = actionlib::SimpleActionClient<mbf_msgs::MoveBaseAction>;
using ExeCtrl = actionlib::SimpleActionClient<mbf_msgs::ExePathAction>;
using goalState = actionlib::SimpleClientGoalState;

enum class StateValue : uint8_t {
  Idle = 0,
  RecordStart,
  RecordStop,
  Pause,
  Navigate,
  Wait,
  Charge,
  Sleep

};

enum class NavState : uint8_t {
  Exe = 0,
  Goto,
  Wait
};

class Task {
public:
  Task();

  ~Task();

  void init();

  void run();

  void stop();

  std::string FILE_PATH = "/home/wu/xj_robot/PATH/path_1";

private:
  auto getRobotPose() -> std::optional<geometry_msgs::Pose>;
  void recordPath();
  auto writePathFile(std::string& path_file) -> bool;
  auto readPathFile(std::string& path_file) -> bool;
  void pubZeroVel();
  void keyboardInputCB(const std_msgs::Int32::ConstPtr& input);
  void visualizePath(std::vector<geometry_msgs::Pose> const& record_path,ros::Publisher const& path_pub);
  auto poseDistance(geometry_msgs::Pose const& cur_pose, geometry_msgs::Pose const& last_pose) -> std::pair<double, double>;
  void navigate();
  bool sendExeCtrl(size_t const& index);
  void exeDone(const actionlib::SimpleClientGoalState& state,const mbf_msgs::ExePathResultConstPtr& result);
  void cancelNav();
  void reset();
  auto isFree(const geometry_msgs::PoseStamped& pose) const -> bool ;
  auto isDanger(const geometry_msgs::PoseStamped& pose) const -> bool ;
  auto calcCost(const geometry_msgs::PoseStamped& pose) const -> uint8_t;
  void costmapCB(nav_msgs::OccupancyGrid::ConstPtr const& msg);
  void costmapUpdateCB(map_msgs::OccupancyGridUpdate::ConstPtr const& msg);
  std::pair<size_t, double> nearestIndex(nav_msgs::Path const& path, size_t const& index, int lb, int rb);
  auto sendGoto(size_t const& index) -> bool ;
  void gotoDone(const actionlib::SimpleClientGoalState& state,
                             const mbf_msgs::MoveBaseResultConstPtr& result);
  void joyCtrlCB(const sensor_msgs::Joy::ConstPtr& joy);

private:
  StateValue cur_state_;
  NavState nav_state_;
  std::unique_ptr<tf2_ros::Buffer> tf_;
  std::unique_ptr<tf2_ros::TransformListener> tfl_;
  std::vector<geometry_msgs::Pose> record_path_;
  nav_msgs::Path cur_route_;
  
  std::unique_ptr<GotoCtrl> goto_ctrl_;
  std::unique_ptr<ExeCtrl> exe_ctrl_;
  size_t cur_index_;
  std::shared_ptr<costmap_2d::Costmap2D> costmap_;
  int wait_cnt = 0;
  int obs_index_ = 0;
  bool read_once_ = true;

  bool key_point_record = false;

  std::mutex map_update_mutex_;
  // 
  static uint8_t* cost_translation_;

  float REC_PATH_LEN_DENS = 0.1;
  float REC_PATH_ANG_DENS = 10 * DEG2RAD;

  ros::Publisher vel_pub_;
  ros::Publisher visual_path_pub_;

  ros::Subscriber keyboard_sub_;
  ros::Subscriber costmap_sub_;
  ros::Subscriber costmap_update_sub_;
  ros::Subscriber joy_sub_;
  

};
}
