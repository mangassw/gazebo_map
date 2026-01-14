/**
 * *********************************************************
 *
 * @file: rpp_controller.cpp
 * @brief: RPP 局部规划器
 * @author: flamingo
 * @date: 2025-7-16
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#include <pluginlib/class_list_macros.h>
#include "rpp_pkg/rpp_controller.h"
#include "rpp_pkg/rpp_math.h"
#include "rpp_pkg/rpp_vec2d.h"
#include "rpp_pkg/controller.h"



PLUGINLIB_EXPORT_CLASS(rpp_pkg::controller::RPPController, nav_core::BaseLocalPlanner)

namespace rpp_pkg
{
namespace controller
{
RPPController::RPPController() 
  : Controller(),  // 调用基类的构造函数
  initialized_(false), tf_(nullptr), goal_reached_(false) {
  odom_helper_ = std::make_shared<base_local_planner::OdometryHelperRos>(getOdomFrame()); // 假设基类有getOdomFrame方法
}

RPPController::RPPController(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
  : RPPController()
{
  initialize(name, tf, costmap_ros);
}

RPPController::~RPPController()
{
}

/**
 * @brief 1.局部规划器初始化
 * @param name        实例化名称
 * @param tf          坐标变换监听器的指针
 * @param costmap_ros 代价地图
 */
void RPPController::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
{
  if (!initialized_)
  {
    initialized_ = true;
    tf_ = tf;
    costmap_ros_ = costmap_ros;
    setlocale(LC_ALL, "");

    ros::NodeHandle nh = ros::NodeHandle("~/" + name);

    // base-基础参数
    nh.param("goal_dist_tolerance", goal_dist_tol_, 0.2);  //位置容忍
    nh.param("rotate_tolerance", rotate_tol_, 0.5);        //角度容忍
    nh.param("base_frame", base_frame_, base_frame_);      //机器人坐标系名称
    nh.param("map_frame", map_frame_, map_frame_);         //地图坐标系名称

    // lookahead-前瞻性参数
    nh.param("lookahead_time", lookahead_time_, 0.5);         //前瞻时间1.5
    nh.param("min_lookahead_dist", min_lookahead_dist_, 0.5); // 最小前瞻距离（单位：米）：无论速度多慢，前瞻距离不小于该值
    nh.param("max_lookahead_dist", max_lookahead_dist_, 1.0); // 最大前瞻距离

    // linear velocity-线速度
    nh.param("max_v", max_v_, 0.5);     
    nh.param("min_v", min_v_, 0.0);
    nh.param("max_v_inc", max_v_inc_, 0.5);

    // angular velocity -角速度
    nh.param("max_w", max_w_, 1.57);
    nh.param("min_w", min_w_, 0.0);
    nh.param("max_w_inc", max_w_inc_, 1.57);

    // constriants -约束参数
    nh.param("regulated_min_radius", regulated_min_radius_, 0.5);   // 最小转弯半径0.9
    nh.param("inflation_cost_factor", inflation_cost_factor_, 3.0); // 代价地图膨胀系数
    nh.param("scaling_dist", scaling_dist_, 2.0);                   // 障碍距离阈值（单位：米）：小于该距离时开始减速避障0.6
    nh.param("scaling_gain", scaling_gain_, 1.0);                // 障碍减速系数：控制避障时的减速幅度
    nh.param("approach_dist", approach_dist_, 0.8);              // 目标接近距离（单位：米）：小于该距离时开始减速靠近目标
    nh.param("approach_min_v", approach_min_v_, 0.1);            // 目标接近阶段的最小速度（单位：米/秒）：避免速度过低导致停滞

    double controller_freqency;
    nh.param("/move_base/controller_frequency", controller_freqency, 10.0);
    d_t_ = 1 / controller_freqency;               //控制周期

    // 发布跟踪目标点（前瞻点），话题名为 `/target_point`
    target_pt_pub_    = nh.advertise<geometry_msgs::PointStamped>("/target_point",    10); 
    slowdown_pt_pub_  = nh.advertise<geometry_msgs::PointStamped>("/slowdown_point",  10); 
    current_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped> ("/current_pose",    10);

    ROS_INFO("RPP Controller initialized!");
  }
  else
    ROS_WARN("RPP Controller has already been initialized.");
}

/**
 * @brief  2.设置控制器目标点（临时）
 * @param orig_global_plan 传递给控制器的路径
 * @return  若路径更新成功则返回 true，否则返回 false
 */
bool RPPController::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
{
  if (!initialized_)
  {
    ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
    return false;
  }

  ROS_INFO("找到新路径");

  // 清空旧路径，存储新路径
  global_plan_.clear();
  global_plan_ = orig_global_plan;

  // 检查新路径的终点，重置目标点
  if (goal_x_ != global_plan_.back().pose.position.x || goal_y_ != global_plan_.back().pose.position.y)
  {
    goal_x_ = global_plan_.back().pose.position.x;
    goal_y_ = global_plan_.back().pose.position.y;
    goal_theta_ = getYawAngle(global_plan_.back());
    goal_reached_ = false;
  }

  return true;
}

/**
 * @brief  3.检查是否到达最终目标
 * @return 若路径更新成功则返回 true，否则返回 false
 */
bool RPPController::isGoalReached()
{
  if (!initialized_)
  {
    ROS_ERROR("RPP Controller has not been initialized");
    return false;
  }

  if (goal_reached_)
  {
    ROS_INFO("GOAL Reached!");
    return true;
  }
  return false;
}

/**
 * @brief 4.计算速度指令（主要函数）
 * @param cmd_vel 速度
 * @return  若找到有效轨迹则返回 true，否则返回 false
 */
bool RPPController::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
  if (!initialized_){ROS_ERROR("RPP控制器没有初始化!");return false;}

  geometry_msgs::PoseStamped robot_pose_map;
  const double obstacle_threshold = costmap_2d::INSCRIBED_INFLATED_OBSTACLE;  //253

  // 获取机器人在里程计坐标系下的速度
  nav_msgs::Odometry base_odom;
  odom_helper_->getOdom(base_odom);
  double current_linear_vel = std::hypot(base_odom.twist.twist.linear.x, base_odom.twist.twist.linear.y); 


  // 获取机器人在地图坐标系下的位姿
  geometry_msgs::PoseStamped robot_pose_odom;
  costmap_ros_->getRobotPose(robot_pose_odom);
  transformPose(tf_, map_frame_, robot_pose_odom, robot_pose_map);

  // 裁剪全局路径，仅保留机器人前方的点
  std::vector<geometry_msgs::PoseStamped> prune_plan = prune(robot_pose_map);

  // 计算前瞻点（L）
  double vt = std::hypot(base_odom.twist.twist.linear.x, base_odom.twist.twist.linear.y);
  double L = getLookAheadDistance(vt);

  //定义减速距离限制
  const double min_slow_distance = 0.4;   // 最小减速距离
  const double max_slow_distance = 2.0;   // 最大减速距离
  const double slow_vel_factor = 1.5;     // 速度系数（减速距离 = 速度 × 系数）

  //基于当前车速，动态计算减速距离
  double slow_distance = current_linear_vel * slow_vel_factor;  
  slow_distance = std::max(slow_distance, min_slow_distance);  
  slow_distance = std::min(slow_distance, max_slow_distance);  

  // 获取1倍前瞻点（L）和减速点
  geometry_msgs::PointStamped lookahead_pt, slowdown_pt;
  double theta_L, kappa_L, theta_S, kappa_S;
  getLookAheadPoint(L,  robot_pose_map, prune_plan, lookahead_pt,  theta_L, kappa_L);  // 1L点
  getLookAheadPoint(slow_distance, robot_pose_map, prune_plan, slowdown_pt, theta_S, kappa_S); // 减速点

  // //初始化滤波器（首次调用时）
  // if (!filter_initialized_) {
  //   lookahead_filter_.init(lookahead_pt.point.x, lookahead_pt.point.y);
  //   slowdown_filter_.init(slowdown_pt.point.x, slowdown_pt.point.y);
  //   filter_initialized_ = true;
  // }

  // // 应用卡尔曼滤波
  // lookahead_filter_.update(lookahead_pt.point.x, lookahead_pt.point.y);
  // slowdown_filter_.update(slowdown_pt.point.x, slowdown_pt.point.y);

  // // 使用滤波后的值替换原始值
  // lookahead_pt.point.x = lookahead_filter_.x;
  // lookahead_pt.point.y = lookahead_filter_.y;
  // slowdown_pt.point.x = slowdown_filter_.x;
  // slowdown_pt.point.y = slowdown_filter_.y;

  // 计算跟踪曲率（Pure Pursuit核心）
  double lookahead_k = 2 * sin(_dphi(lookahead_pt, robot_pose_map)) / L;



  // ================================  减速逻辑 + 停止逻辑  =========================================
  
  const double max_cost  = 253.0;                     // 最大代价（对应障碍物）
  const double safe_cost = 80.0;                     // 安全代价（低于此值不减速）

  //2. 检测1L点是否有障碍物（用于停止）
  bool stop = false;
  unsigned int mx_L, my_L;
  if (costmap_ros_->getCostmap()->worldToMap(
        lookahead_pt.point.x, lookahead_pt.point.y, mx_L, my_L)) {
    double cost_L = costmap_ros_->getCostmap()->getCost(mx_L, my_L);
    if (cost_L >= obstacle_threshold) {  // 1L点有障碍物
      stop = true;
      ROS_WARN_THROTTLE(1, "前瞻点有障碍物，停车");
    }
  }

  // 3. 检测动态减速点代价（用于减速）
  double speed_factor = 1.0;  // 速度因子（0~1，1=不减速）
  unsigned int mx_S, my_S;
  if (costmap_ros_->getCostmap()->worldToMap(
        slowdown_pt.point.x, slowdown_pt.point.y, mx_S, my_S)) {
    double cost_S = costmap_ros_->getCostmap()->getCost(mx_S, my_S);
    
    if (cost_S > safe_cost && cost_S < obstacle_threshold) {
      // 减速点代价越高，速度因子越小（线性衰减）
      speed_factor = 1.0 - (cost_S - safe_cost) / (max_cost - safe_cost);
      speed_factor = std::max(speed_factor, 0.1);  // 最低保留10%速度，避免停滞
      ROS_DEBUG("减速点代价: %.1f, 减速比例: %.2f, 当前车速: %.2f m/s", 
                cost_S, speed_factor, current_linear_vel);
    }
  }

  // 4. 若前瞻点有障碍物，直接停车
  if (stop) {
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
    return true;
  }
  // ===============================================================================


  // 判断是否已接近目标位置，需要旋转到目标朝向
  if (shouldRotateToGoal(robot_pose_map, global_plan_.back()))
  {
    // 计算角度误差并正则化到[-π, π]
    double e_theta = regularizeAngle(goal_theta_ - tf2::getYaw(robot_pose_map.pose.orientation));

    // 角度误差小于阈值，认为已到达目标
    if (!shouldRotateToPath(std::fabs(e_theta)))
    {
      cmd_vel.linear.x = 0.0;
      cmd_vel.angular.z = 0.0;
      goal_reached_ = true;
    }
    // 角度未达标，仅进行旋转
    else
    {
      cmd_vel.linear.x = 0.0;
      cmd_vel.angular.z = angularRegularization(base_odom, e_theta / d_t_);
    }
  }
  else
  {
    // 计算机器人朝向与前瞻点的角度误差
    double e_theta = regularizeAngle(_dphi(lookahead_pt, robot_pose_map));

    // 大角度误差，先转向再前进
    if (shouldRotateToPath(std::fabs(e_theta), M_PI / 4))
    {
        // 角度误差大于 M_PI / 4，仅进行转向
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = angularRegularization(base_odom, e_theta / d_t_);
    }
    else if (shouldRotateToPath(std::fabs(e_theta), M_PI / 16))
    {
        // 角度误差在 M_PI / 16 到 M_PI / 4 之间，边前进边转向
        // 避免速度突变
        double forward_vel = linearRegularization(base_odom, 0.1); // 使用线性正则化函数调整前进速度
        cmd_vel.linear.x = forward_vel;
        cmd_vel.angular.z = angularRegularization(base_odom, e_theta / d_t_);
    }

    // 正常路径跟踪
    else
    {
      //约束三种速度
      double curv_vel = _applyCurvatureConstraint(max_v_, lookahead_k); // 曲率约束
      double cost_vel = _applyObstacleConstraint(max_v_);               // 障碍物约束
      double v_d = std::min(curv_vel, cost_vel);                        // 取最小速度
      v_d = _applyApproachConstraint(v_d, robot_pose_map, prune_plan);  // 目标接近约束
      v_d *= speed_factor;        //减速速度约束
      
      //卡尔曼滤波
      v_d = k_vel_filter_.update(v_d);

      cmd_vel.linear.x = linearRegularization(base_odom, v_d);
      cmd_vel.angular.z = angularRegularization(base_odom, v_d * lookahead_k);
    }
  }

  // 发布前瞻点和机器人当前位姿
  target_pt_pub_.publish(lookahead_pt);
  slowdown_pt_pub_.publish(slowdown_pt);    //发布减速点，便于可视化
  current_pose_pub_.publish(robot_pose_map);
  
  return true;
}

/**
 * @brief 4.1计算机器人朝向与前瞻点方向之间的夹角 
 * @param lookahead_pt      the lookahead pose [global] 全局坐标系下的前瞻点（路径上距离机器人 L 的点）
 * @param robot_pose_global the robot's pose  [global]全局坐标系下的机器人位姿
 * @return dphi             the lookahead angle - robot's yaw 机器人朝向与前瞻点方向的夹角（单位：弧度），范围为 [-π, π]
 */
double RPPController::_dphi(geometry_msgs::PointStamped lookahead_pt, geometry_msgs::PoseStamped robot_pose_global)
{
  return atan2(lookahead_pt.point.y - robot_pose_global.pose.position.y,
               lookahead_pt.point.x - robot_pose_global.pose.position.x) -
         tf2::getYaw(robot_pose_global.pose.orientation);               //偏航角
}

/**
 * @brief 4.2曲率约束机制，在急转弯时降低机器人的线速度
 * @param raw_linear_vel    当前速度
 * @param curvature         跟踪曲率
 * @return reg_vel          修正速度
 */
double RPPController::_applyCurvatureConstraint(const double raw_linear_vel, const double curvature)
{
  const double radius = std::fabs(1.0 / curvature); //计算转弯半径  fabs-->取绝对值
  if (radius < regulated_min_radius_)               //如果小于最小转弯半径
    return raw_linear_vel * (radius / regulated_min_radius_);   //减速
  else
    return raw_linear_vel;
}

/**
 * @brief 4.3障碍物约束机制，接近障碍物时降低线速度，避免碰撞
 * @param raw_linear_vel    当前速度
 * @return reg_vel          修正速度
 */
double RPPController::_applyObstacleConstraint(const double raw_linear_vel)
{
  //获取机器人中心在代价地图中的坐标
  int size_x = costmap_ros_->getCostmap()->getSizeInCellsX() / 2;
  int size_y = costmap_ros_->getCostmap()->getSizeInCellsY() / 2;

  //读取机器人中心位置的代价
  double robot_cost = static_cast<double>(costmap_ros_->getCostmap()->getCost(size_x, size_y));

  //如果机器人不处于自由空间、且不是未知区域
  if (robot_cost != static_cast<double>(costmap_2d::FREE_SPACE) &&
      robot_cost != static_cast<double>(costmap_2d::NO_INFORMATION))
  {
    const double& inscribed_radius = costmap_ros_->getLayeredCostmap()->getInscribedRadius();//机器人的内切圆半径

    // 估算障碍物距离
    const double obs_dist =
        inscribed_radius -
        (log(robot_cost) - log(static_cast<double>(costmap_2d::INSCRIBED_INFLATED_OBSTACLE))) / inflation_cost_factor_;
    //小于阈值，开始减速
    if (obs_dist < scaling_dist_)
      return raw_linear_vel * scaling_gain_ * obs_dist / 2*(scaling_dist_);
  }
  return raw_linear_vel;
}

/**
 * @brief 4.4目标接近约束机制，在机器人接近最终目标时平滑降低线速度，避免因速度过快而冲过目标点
 * @param raw_linear_vel    当前速度
 * @param robot_pose_global 全局坐标系下的机器人位姿
 * @param prune_plan        裁减后的前瞻点（修剪后的全局路径）
 * @return reg_vel          修正速度
 */
double RPPController::_applyApproachConstraint(const double raw_linear_vel,
                                               geometry_msgs::PoseStamped robot_pose_global,
                                               const std::vector<geometry_msgs::PoseStamped>& prune_plan)
{
  //用于计算两个位姿点（PoseStamped）之间的直线距离    hypot --> 计算直角三角形斜边
  auto dist = [](const geometry_msgs::PoseStamped& ps_1, const geometry_msgs::PoseStamped& ps_2) {
    return std::hypot(ps_1.pose.position.x - ps_2.pose.position.x, ps_1.pose.position.y - ps_2.pose.position.y);
  };

  //计算剩余路径长度
  double remain_dist = 0.0;
  for (size_t i = 0; i < prune_plan.size() - 1; i++)
    remain_dist += dist(prune_plan[i], prune_plan[i + 1]);

  //计算速度缩放因子
  double s = remain_dist < approach_dist_ ? dist(prune_plan.back(), robot_pose_global) / approach_dist_ : 1.0;

  return std::min(raw_linear_vel, std::max(approach_min_v_, raw_linear_vel * s));
}



}  // namespace controller
}  // namespace rpp_pkg