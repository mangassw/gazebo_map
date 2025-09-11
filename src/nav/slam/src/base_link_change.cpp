#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <hdl_localization/ScanMatchingStatus.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <deque>
#include <cmath>

// 自定义消息类型结构
struct PositionAccuracy {
    std_msgs::Bool within_threshold;
    std_msgs::Float64 distance;
};

class LidarRTKValidator {
private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    
    // 订阅者
    ros::Subscriber lidar_status_sub_;
    ros::Subscriber gps_sub_;
    
    // 发布者
    ros::Publisher initial_pose_pub_;
    ros::Publisher accuracy_pub_;
    
    // TF相关
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    
    // 参数
    double rtk_covariance_threshold_;
    double position_diff_threshold_;
    double matching_error_threshold_;
    bool enable_area_check_;
    int history_window_;
    // 数据存储
    hdl_localization::ScanMatchingStatusConstPtr latest_lidar_status_;
    sensor_msgs::NavSatFixConstPtr latest_gps_;
    std::deque<double> position_diff_history_;
    
    // 定时器
    ros::Timer timer_;
    
public:
    double lidar_del_x_;
    double lidar_del_y_;
    LidarRTKValidator() : tf_listener_(tf_buffer_), private_nh_("~") {
        // 获取参数
        private_nh_.param("rtk_covariance_threshold", rtk_covariance_threshold_, 0.1);
        private_nh_.param("position_diff_threshold", position_diff_threshold_, 1.0);
        private_nh_.param("matching_error_threshold", matching_error_threshold_, 0.01);
        private_nh_.param("enable_area_check", enable_area_check_, false);
        private_nh_.param("history_window", history_window_, 10);
        private_nh_.param("lidar_del_x", lidar_del_x_, 0.1);
        private_nh_.param("lidar_del_y", lidar_del_y_, 0.1);


        
        // 订阅话题
        lidar_status_sub_ = nh_.subscribe("/status", 10, &LidarRTKValidator::lidarStatusCallback, this);
        gps_sub_ = nh_.subscribe("/gps/fix", 10, &LidarRTKValidator::gpsCallback, this);
        
        // 发布话题
        initial_pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 10);
        // accuracy_pub_ = nh_.advertise<PositionAccuracy>("/position_accuracy", 10);
        
        // 定时器
        timer_ = nh_.createTimer(ros::Duration(0.1), &LidarRTKValidator::timerCallback, this);
        
        ROS_INFO("Lidar RTK Validator initialized");
        ROS_INFO("Parameters: RTK cov threshold=%.3f, position diff threshold=%.3f, matching error threshold=%.6f", 
                 rtk_covariance_threshold_, position_diff_threshold_, matching_error_threshold_);
    }
    
private:
    void lidarStatusCallback(const hdl_localization::ScanMatchingStatus::ConstPtr& msg) {
        latest_lidar_status_ = msg;
    }
    
    void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
        latest_gps_ = msg;
    }
    
    geometry_msgs::TransformStamped::Ptr getTransform(const std::string& target_frame, 
                                                     const std::string& source_frame,
                                                     const ros::Time& time = ros::Time(0)) {
        try {
            geometry_msgs::TransformStamped transform = tf_buffer_.lookupTransform(
                target_frame, source_frame, time, ros::Duration(0.1));
            geometry_msgs::TransformStamped::Ptr result(new geometry_msgs::TransformStamped(transform));
            return result;
        } catch (tf2::TransformException& ex) {
            ROS_WARN_THROTTLE(5.0, "Could not get transform from %s to %s: %s", 
                             source_frame.c_str(), target_frame.c_str(), ex.what());
            return geometry_msgs::TransformStamped::Ptr();
        }
    }
    
    double calculatePositionDistance(const geometry_msgs::Vector3& pos1, 
                                   const geometry_msgs::Vector3& pos2) {
        double dx = pos1.x - pos2.x;
        double dy = pos1.y - pos2.y;
        double dz = pos1.z - pos2.z;
        return std::sqrt(dx*dx + dy*dy + dz*dz);
    }
    
    bool isGPSValid() {
        if (!latest_gps_) {
            return false;
        }
        
        // 检查GPS状态
        if (latest_gps_->status.status < 0) {
            return false;
        }
        
        // 检查协方差
            // 计算协方差迹
        double covariance_trace = latest_gps_->position_covariance[0] + 
                                latest_gps_->position_covariance[4] + 
                                latest_gps_->position_covariance[8];
        return covariance_trace < rtk_covariance_threshold_;
    }
    
    bool isLidarMatchingValid() {
        if (!latest_lidar_status_) {
            return false;
        }
        
        return latest_lidar_status_->has_converged && 
               latest_lidar_status_->matching_error < matching_error_threshold_;
    }
    
    bool isPositionStable(double current_distance) {
        // 添加当前距离到历史记录
        position_diff_history_.push_back(current_distance);
        if (position_diff_history_.size() > static_cast<size_t>(history_window_)) {
            position_diff_history_.pop_front();
        }
        
        // 检查历史窗口期内是否都小于阈值
        if (position_diff_history_.size() < 5) {
            return false;
        }
        
        for (const auto& dist : position_diff_history_) {
            if (dist >= position_diff_threshold_) {
                return false;
            }
        }
        
        return true;
    }
    
    void publishInitialPose(const geometry_msgs::TransformStamped::Ptr& gps_transform) {
        if (!gps_transform) return;
        
        geometry_msgs::PoseWithCovarianceStamped pose_msg;
        pose_msg.header.stamp = ros::Time::now();
        pose_msg.header.frame_id = "map";
        
        pose_msg.pose.pose.position.x = gps_transform->transform.translation.x;
        pose_msg.pose.pose.position.y = gps_transform->transform.translation.y;
        pose_msg.pose.pose.position.z = gps_transform->transform.translation.z;
        
        pose_msg.pose.pose.orientation = gps_transform->transform.rotation;
        
        // 设置协方差矩阵
        for (int i = 0; i < 36; ++i) {
            pose_msg.pose.covariance[i] = 0.0;
        }
        // 设置对角线元素
        for (int i = 0; i < 6; ++i) {
            pose_msg.pose.covariance[i*6 + i] = 0.1;
        }
        
        initial_pose_pub_.publish(pose_msg);
        ROS_INFO("Published initial pose from GPS");
    }
    
    void publishBaseLinkTF(bool lidar_valid, 
                          const geometry_msgs::TransformStamped::Ptr& lidar_transform,
                          const geometry_msgs::TransformStamped::Ptr& gps_transform) {
        geometry_msgs::TransformStamped transform;
        transform.header.stamp = ros::Time::now();
        transform.header.frame_id = "map";
        transform.child_frame_id = "base_link";
        
        if (lidar_valid && lidar_transform) {
            transform.transform = lidar_transform->transform;
            ROS_INFO("Using lidar position for base_link");
        } else if (gps_transform) {
            transform.transform = gps_transform->transform;

            ROS_INFO("Using GPS position for base_link");
        } else {
            return;
        }
        
        tf_broadcaster_.sendTransform(transform);
    }
    
    void timerCallback(const ros::TimerEvent& event) {
        // 获取变换
        auto lidar_transform = getTransform("map", "lidar2car");
        auto gps_transform = getTransform("map", "gps_base_link");
        
        if (!lidar_transform || !gps_transform) {
            return;
        }
        
        // 计算位置差值
        double distance = calculatePositionDistance(
            lidar_transform->transform.translation,
            gps_transform->transform.translation
        );
        
        // 检查各种条件
        bool gps_valid = isGPSValid();
        bool lidar_matching_valid = isLidarMatchingValid();
        bool position_stable = isPositionStable(distance);
        // 综合判断激光雷达是否有效
        bool lidar_valid = !gps_valid && lidar_matching_valid && position_stable;
        ROS_INFO("gps_valid: %s, lidar_matching_valid: %s, position_stable: %s, lidar_valid: %s", 
            gps_valid ? "true" : "false",
            lidar_matching_valid ? "true" : "false",
            position_stable ? "true" : "false",
            lidar_valid ? "true" : "false");        
        // 如果位置差值超过阈值，发布初始位置
        if (distance > position_diff_threshold_&&gps_valid) {
            publishInitialPose(gps_transform);
            ROS_WARN("Large position difference detected: %.2f m, published initial pose", distance);
        }
        
        // 发布base_link TF
        publishBaseLinkTF(lidar_valid, lidar_transform, gps_transform);
        
        // 发布准确度信息
        // PositionAccuracy accuracy_msg;
        // accuracy_msg.within_threshold.data = (distance < position_diff_threshold_);
        // accuracy_msg.distance.data = distance;
        // accuracy_pub_.publish(accuracy_msg);
        
        // 日志输出
        if (lidar_valid) {
            ROS_DEBUG("Lidar positioning valid. Distance: %.3f m", distance);
        } else {
            ROS_DEBUG("Lidar positioning invalid. Distance: %.3f m, GPS valid: %s, Matching valid: %s", 
                     distance, gps_valid ? "true" : "false", 
                     lidar_matching_valid ? "true" : "false");
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "lidar_rtk_validator");
    
    try {
        LidarRTKValidator validator;
        ros::spin();
    } catch (const std::exception& e) {
        ROS_ERROR("Exception in Lidar RTK Validator: %s", e.what());
        return -1;
    }
    
    return 0;
}