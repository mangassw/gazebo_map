#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>

class RTKTFPublisher {
private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    
    // 订阅者
    ros::Subscriber gps_sub_;
    ros::Subscriber imu_sub_;
    
    // TF广播器
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    
    // 地图原点坐标
    double map_origin_lat_;
    double map_origin_lon_;
    double map_origin_alt_;
    
    // Local Cartesian转换器
    GeographicLib::LocalCartesian local_cartesian_;
    
    // 最新的IMU数据
    sensor_msgs::Imu latest_imu_;
    bool imu_received_;
    
public:
    RTKTFPublisher() : private_nh_("~"), imu_received_(false) {
        // 获取参数
        private_nh_.param("map_origin_lat", map_origin_lat_, 46.999999999979096);
        private_nh_.param("map_origin_lon", map_origin_lon_, 8.00000000000818);
        private_nh_.param("map_origin_alt", map_origin_alt_, 0.0);
        
        // 初始化Local Cartesian转换器
        local_cartesian_ = GeographicLib::LocalCartesian(map_origin_lat_, map_origin_lon_, map_origin_alt_);
        
        // 订阅话题
        gps_sub_ = nh_.subscribe("/gps/fix", 10, &RTKTFPublisher::gpsCallback, this);
        imu_sub_ = nh_.subscribe("/imu/data", 10, &RTKTFPublisher::imuCallback, this);
        
        ROS_INFO("RTK TF Publisher initialized with origin: lat=%.6f, lon=%.6f, alt=%.1f", 
                 map_origin_lat_, map_origin_lon_, map_origin_alt_);
    }
    
private:
    void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
        if (msg->status.status < 0) {
            ROS_WARN_THROTTLE(5.0, "GPS fix not available");
            return;
        }
        
        try {
            // 使用GeographicLib进行精确坐标转换
            double x, y, z;
            local_cartesian_.Forward(msg->latitude, msg->longitude, msg->altitude, x, y, z);
            
            // 创建TF变换
            geometry_msgs::TransformStamped transform;
            transform.header.stamp = msg->header.stamp;
            transform.header.frame_id = "gps_link";
            transform.child_frame_id = "gps_base_link";
            
            // 设置平移
            transform.transform.translation.x = x;
            transform.transform.translation.y = y;
            transform.transform.translation.z = z;
            
            // 设置旋转
            if (imu_received_) {
                transform.transform.rotation = latest_imu_.orientation;
            } else {
                // 默认单位四元数
                transform.transform.rotation.x = 0.0;
                transform.transform.rotation.y = 0.0;
                transform.transform.rotation.z = 0.0;
                transform.transform.rotation.w = 1.0;
            }
            
            // 广播TF
            tf_broadcaster_.sendTransform(transform);
            
        } catch (const std::exception& e) {
            ROS_ERROR("Coordinate transformation error: %s", e.what());
        }
    }
    
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
        latest_imu_ = *msg;
        imu_received_ = true;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "rtk_tf_publisher");
    
    try {
        RTKTFPublisher publisher;
        ros::spin();
    } catch (const std::exception& e) {
        ROS_ERROR("Exception in RTK TF Publisher: %s", e.what());
        return -1;
    }
    
    return 0;
}