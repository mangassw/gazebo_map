#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <hdl_localization/ScanMatchingStatus.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <mutex>
#include <atomic>
#include <cmath>
#include <Eigen/Dense>
class EnuConverter
{
public:
    EnuConverter() = default;

    // 设置原点（WGS84 经纬度，度）
    void setOrigin(double lat0_deg, double lon0_deg, double h0 = 0.0)
    {
        lat0_ = deg2rad(lat0_deg);
        lon0_ = deg2rad(lon0_deg);
        h0_   = h0;

        // 计算原点 ECEF
        ecef0_ = llh2ecef(lat0_, lon0_, h0_);

        // ENU 旋转矩阵
        double sina = std::sin(lat0_);
        double cosa = std::cos(lat0_);
        double sinb = std::sin(lon0_);
        double cosb = std::cos(lon0_);

        R_ << -sinb,            cosb,            0.0,
              -sina * cosb,    -sina * sinb,    cosa,
               cosa * cosb,     cosa * sinb,    sina;
    }

    // 输入：WGS84 (deg,deg,m)  输出：ENU (m,m)
    void convert(double lat_deg, double lon_deg, double h,
                 double &x, double &y) const
    {
        Eigen::Vector3d ecef = llh2ecef(deg2rad(lat_deg), deg2rad(lon_deg), h);
        Eigen::Vector3d enu  = R_ * (ecef - ecef0_);
        x = enu(0);
        y = enu(1);
    }

private:
    static constexpr double kA   = 6378137.0;            // 地球半长轴
    static constexpr double kE2  = 6.69437999014e-3;     // e²

    inline double deg2rad(double d) const { return d * M_PI / 180.0; }

    Eigen::Vector3d llh2ecef(double lat, double lon, double h) const
    {
        double sina = std::sin(lat);
        double cosa = std::cos(lat);
        double sinb = std::sin(lon);
        double cosb = std::cos(lon);

        double N = kA / std::sqrt(1.0 - kE2 * sina * sina);
        Eigen::Vector3d ecef;
        ecef(0) = (N + h) * cosa * cosb;
        ecef(1) = (N + h) * cosa * sinb;
        ecef(2) = (N * (1.0 - kE2) + h) * sina;
        return ecef;
    }

    double lat0_ = 0.0;
    double lon0_ = 0.0;
    double h0_   = 0.0;

    Eigen::Vector3d ecef0_;
    Eigen::Matrix3d R_;
};

/* ---------- 数据源枚举 ---------- */
enum class DataStatus { NONE, REC, USED };

template<typename T>
struct DataSlot
{
    typename T::ConstPtr msg;
    double err = 1e6;
    std::atomic<DataStatus> status{DataStatus::NONE};
    std::mutex mtx;
};

/* ---------- 节点选择策略 ---------- */
enum class Source { GPS, HDL };

class GpsImuSelectTfNode
{
public:
    explicit GpsImuSelectTfNode(ros::NodeHandle &nh) : nh_(nh), rate_(10)
    {
        loadParams();
        converter_.setOrigin(origin_lat_, origin_lon_, 0.0);
        gps_sub_   = nh_.subscribe("/gps/fix_with_covariance", 10,
                                   &GpsImuSelectTfNode::gpsCb, this);
        hdl_sub_   = nh_.subscribe("/status", 10,
                                   &GpsImuSelectTfNode::hdlCb, this);
        imu_sub_   = nh_.subscribe("/imu/data", 10,
                                   &GpsImuSelectTfNode::imuCb, this);
        hdl_origin_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(
                                    "/initialpose", 1, true);
    }
    void publishInitialPose(double x, double y)
    {
        geometry_msgs::PoseWithCovarianceStamped pose_msg;
        pose_msg.header.stamp    = ros::Time::now();
        pose_msg.header.frame_id = "map";          // 通常固定为 map
        pose_msg.pose.pose.position.x = x;
        pose_msg.pose.pose.position.y = y;
        pose_msg.pose.pose.position.z = 0;

        // 使用当前 IMU 朝向
        {
            std::lock_guard<std::mutex> lk(imu_mtx_);
            pose_msg.pose.pose.orientation = imu_->orientation;
        }

        // 简单地把协方差设为 0；可根据需要填写
        pose_msg.pose.covariance.fill(0.0);
        hdl_origin_pub_.publish(pose_msg);
        ROS_INFO_ONCE("Published initialpose from GPS");
    }
    void run()
    {
        while (ros::ok())
    {
        ros::spinOnce();

        /* 等待 IMU */
        if (!imu_) { rate_.sleep(); continue; }

        /* 等待两条新数据 */
        if (gps_.status.load() != DataStatus::REC ||
            hdl_.status.load() != DataStatus::REC)
        {
            rate_.sleep(); continue;
        }

        /* 选择来源（仅决定 /base_link 的父坐标系） */
        Source src = selectSource();
        gps_.status.store(DataStatus::USED);
        hdl_.status.store(DataStatus::USED);

        /* ---------- 1. 始终发布 GPS 的 tf ---------- */
        geometry_msgs::TransformStamped gps_tf;
        {
            std::lock_guard<std::mutex> lk(gps_.mtx);
            double x, y;
            converter_.convert(gps_.msg->latitude,
                               gps_.msg->longitude,
                               0.0,
                               x, y);
            gps_tf.transform.translation.x = x;
            gps_tf.transform.translation.y = y;
            gps_tf.transform.translation.z = 0;
            gps_tf.header.frame_id = "gps_link";
        }
        {
            std::lock_guard<std::mutex> lk_imu(imu_mtx_);
            gps_tf.transform.rotation = imu_->orientation;
        }
        gps_tf.child_frame_id = "gps_base_link";
        gps_tf.header.stamp   = ros::Time::now();
        tf_br_.sendTransform(gps_tf);

        /* ---------- 2. 按策略发布主 tf ---------- */
        geometry_msgs::TransformStamped tf_msg;
        tf_msg.header.stamp = ros::Time::now();

        if (src == Source::GPS)
        {
            /* 把 GPS 数据直接给 base_link（沿用旧逻辑） */
            std::lock_guard<std::mutex> lk(gps_.mtx);
            double x, y;
            converter_.convert(gps_.msg->latitude,
                gps_.msg->longitude,
                0.0,
                x, y);
            tf_msg.transform.translation.x = x;
            tf_msg.transform.translation.y = y;
            tf_msg.transform.translation.z = 0;
            tf_msg.header.frame_id = "gps_link";
            {
                std::lock_guard<std::mutex> lk_imu(imu_mtx_);
                tf_msg.transform.rotation = imu_->orientation;
            }
            tf_msg.child_frame_id = "base_link";

            /* 下面判断是否发 initialpose 的逻辑保持不变…… */
            bool should_pub = false;
            double dx = 0, dy = 0;
            if (gps_.err * 10 <= err_thresh_)
            {
                double hdl_x = 0, hdl_y = 0;
                {
                    std::lock_guard<std::mutex> lk(hdl_.mtx);
                    if (hdl_.msg)
                    {
                        hdl_x = hdl_.msg->relative_pose.translation.x;
                        hdl_y = hdl_.msg->relative_pose.translation.y;
                    }
                }
                dx = hdl_x - x;
                dy = hdl_y - y;
                if (std::sqrt(dx * dx + dy * dy) > 1) should_pub = true;
            }
            if (should_pub) publishInitialPose(x, y);
        }
        else   // HDL
        {
            /* 沿用旧逻辑：laser_livox → base_link 的静态平移 */
            std::lock_guard<std::mutex> lk(hdl_.mtx);
            tf_msg.transform.translation.x = -0.45;
            tf_msg.transform.translation.y = 0;
            tf_msg.transform.translation.z = 0;
            tf_msg.transform.rotation.w = 1.0;
            tf_msg.header.frame_id = "laser_livox";
            tf_msg.child_frame_id = "base_link";
        }

        tf_br_.sendTransform(tf_msg);
        ROS_INFO("[%s] publish tf", __FUNCTION__);
        rate_.sleep();
    }
    }

private:
    void loadParams()
    {
        nh_.param("origin_lat",  origin_lat_, 46.999999999979096);
        nh_.param("origin_lon",  origin_lon_, 8.00000000000818);
        nh_.param("utm_zone",    utm_zone_, 32);
        nh_.param("error_threshold", err_thresh_, 0.1);
    }

    Source selectSource()
    {
        double gps_err, hdl_err;
        {
            std::lock_guard<std::mutex> lk(gps_.mtx);
            gps_err = gps_.err;
        }
        {
            std::lock_guard<std::mutex> lk(hdl_.mtx);
            hdl_err = hdl_.err;
        }

        bool gps_ok = gps_err <= err_thresh_;
        bool hdl_ok = hdl_err <= err_thresh_;

        if (!gps_ok && !hdl_ok)
        {
            handleBothError(gps_err, hdl_err);
            return Source::GPS; // fallback
        }
        if (gps_ok && !hdl_ok) return Source::GPS;
        if (!gps_ok && hdl_ok) return Source::HDL;
        return (gps_err < hdl_err) ? Source::GPS : Source::HDL;
    }

    void handleBothError(double gps_err, double hdl_err)
    {
        ROS_WARN_THROTTLE(1.0,
            "Both GPS(%.3f) & HDL(%.3f) exceed threshold, use GPS fallback",
            gps_err, hdl_err);
    }

    /* ---------- 回调 ---------- */
    void gpsCb(const sensor_msgs::NavSatFixConstPtr &msg)
    {
        std::lock_guard<std::mutex> lk(gps_.mtx);
        gps_.msg  = msg;
        gps_.err  = msg->position_covariance[0];
        gps_.status.store(DataStatus::REC);
    }

    void hdlCb(const hdl_localization::ScanMatchingStatusConstPtr &msg)
    {
        std::lock_guard<std::mutex> lk(hdl_.mtx);
        hdl_.msg  = msg;
        hdl_.err  = msg->matching_error;
        hdl_.status.store(DataStatus::REC);
    }

    void imuCb(const sensor_msgs::ImuConstPtr &msg)
    {
        std::lock_guard<std::mutex> lk(imu_mtx_);
        imu_ = msg;
    }

    /* ---------- 成员 ---------- */
    ros::NodeHandle nh_;
    ros::Subscriber gps_sub_, hdl_sub_, imu_sub_;
    ros::Publisher hdl_origin_pub_;

    tf::TransformBroadcaster tf_br_;

    EnuConverter converter_;

    DataSlot<sensor_msgs::NavSatFix>              gps_;
    DataSlot<hdl_localization::ScanMatchingStatus> hdl_;
    sensor_msgs::ImuConstPtr imu_;
    std::mutex imu_mtx_;

    double origin_lat_, origin_lon_;
    int    utm_zone_;
    double err_thresh_;
    ros::Rate rate_;
};

/* ---------- main ---------- */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "gps_imu_select_tf_node");
    ros::NodeHandle nh("~");
    GpsImuSelectTfNode node(nh);
    node.run();
    return 0;
}