/**
 * @file controller.h
 * @brief 控制器基类与RPP控制器的声明
 */
#ifndef RPP_PKG_CONTROLLER_H_
#define RPP_PKG_CONTROLLER_H_

#include <ros/ros.h>
#include <nav_core/base_local_planner.h>
#include <base_local_planner/odometry_helper_ros.h>

#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>


namespace rpp_pkg {
namespace controller {

class Controller
{
public:
  Controller();
  ~Controller();

  void setFactor(double factor);
  void setBaseFrame(std::string base_frame);
  void setMapFrame(std::string map_frame);
  double regularizeAngle(double angle);
  double getYawAngle(geometry_msgs::PoseStamped& ps);
  bool shouldRotateToGoal(const geometry_msgs::PoseStamped& cur, const geometry_msgs::PoseStamped& goal);
  bool shouldRotateToPath(double angle_to_path, double tolerance = 0.0);
  double linearRegularization(nav_msgs::Odometry& base_odometry, double v_d);
  double angularRegularization(nav_msgs::Odometry& base_odometry, double w_d);
  void transformPose(tf2_ros::Buffer* tf, const std::string out_frame, const geometry_msgs::PoseStamped& in_pose,
                     geometry_msgs::PoseStamped& out_pose) const;
  bool worldToMap(double wx, double wy, int& mx, int& my);
  std::vector<geometry_msgs::PoseStamped> prune(const geometry_msgs::PoseStamped robot_pose_global);
  double getLookAheadDistance(double vt);
  void getLookAheadPoint(double lookahead_dist, geometry_msgs::PoseStamped robot_pose_global,
                         const std::vector<geometry_msgs::PoseStamped>& prune_plan, geometry_msgs::PointStamped& pt,
                         double& theta, double& kappa);
  std::string getOdomFrame() const {return odom_frame_;}

protected:
  double factor_;  // obstacle factor(greater means obstacles)
  double max_v_, min_v_, max_v_inc_;  // linear velocity
  double max_w_, min_w_, max_w_inc_;  // angular velocity
  double goal_dist_tol_;
  double rotate_tol_;
  std::string base_frame_, map_frame_, odom_frame_;
  std::shared_ptr<base_local_planner::OdometryHelperRos> odom_helper_;  // odometry helper
  costmap_2d::Costmap2DROS* costmap_ros_;                               // costmap(ROS wrapper)
  std::vector<geometry_msgs::PoseStamped> global_plan_;
  double lookahead_time_;      // lookahead time gain
  double min_lookahead_dist_;  // minimum lookahead distance
  double max_lookahead_dist_;  // maximum lookahead distance
};


}  // namespace controller
}  // namespace rpp_pkg

#endif  // RPP_PKG_CONTROLLER_H_