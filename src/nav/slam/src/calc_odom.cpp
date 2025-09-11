#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64.h>
#include "can_msgs/Frame.h"
#include <sensor_msgs/Imu.h>

#include "dynamic_reconfigure/server.h"

#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/system/error_code.hpp>
#include <boost/system/system_error.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>

#include <string>
#include <vector>

class CalcOdom  {
private:
    void recv_vel(const can_msgs::Frame::ConstPtr& msg);
    void update_bot_odom();
    double convertRpmToMs(float inch,float rpm);
    void nav_start_cb(const std_msgs::String::ConstPtr& nav_start_msg);
    void nav_offset_yaw_cb(const std_msgs::Float64::ConstPtr& offset_yaw_msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
public:
    CalcOdom();

    // ~CalcOdom();

    void init();
    void run();

private:
    ros::Subscriber sub_wheel_vel_;
    // ros::Publisher odom_pub_,odom_simu_imu_pub_,real_rpm_feedback_,real_vel_feedback_;
    ros::Subscriber nav_start_sub_, nav_offset_yaw_sub_;
    ros::Subscriber odom_sub;
    ros::Publisher combined_odom_pub;


    bool start_flag_;
    float left_rpm_fd = 0,right_rpm_fd = 0;
    double wheel_distance_ = 0.562;
    double delta_time_;
    double accumulation_x_ = 0, accumulation_y_ = 0, accumulation_th_ = 0;
    int cur_left_, cur_right_, rev_left_, rev_right_, delta_left_, delta_right_;
    int control_rate_ = 50;


    tf2_ros::TransformBroadcaster br_;
    geometry_msgs::TransformStamped transformStamped_;
    ros::Time last_time_, now_;
    nav_msgs::Odometry odom_;
    nav_msgs::Odometry received_odom;

    double offset_yaw_;

    std::string odom_frame_ = "odom";
    std::string base_frame_ = "base_link";
};

CalcOdom::CalcOdom() :start_flag_(true){}

void CalcOdom::init() {
    ros::NodeHandle nh;
    sub_wheel_vel_ = nh.subscribe<can_msgs::Frame>("/can_msg_fb",10,&CalcOdom::recv_vel,this);
    // odom_pub_ = nh.advertise<nav_msgs::Odometry>("/odom", 10);
    // odom_simu_imu_pub_ = nh.advertise<sensor_msgs::Imu>("/odom_to_imu_yaw", 10);
    // real_rpm_feedback_ = nh.advertise<std_msgs::Float32MultiArray>("/rpm_feedback", 10);
    // real_vel_feedback_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel_feedback", 10);
    nav_start_sub_ = nh.subscribe<std_msgs::String>("workstatus",1,&CalcOdom::nav_start_cb,this);
    nav_offset_yaw_sub_ = nh.subscribe<std_msgs::Float64>("/offset_yaw", 10,&CalcOdom::nav_offset_yaw_cb,this);
    ros::param::set("/rosconsole/default_level", "debug");


    // 创建订阅者，订阅odom话题
    odom_sub = nh.subscribe<nav_msgs::Odometry>("odom", 10, &CalcOdom::odomCallback,this);

    // 创建发布者，发布新的odom消息
    combined_odom_pub = nh.advertise<nav_msgs::Odometry>("combined_odom", 10);
}

void CalcOdom::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    received_odom = *msg;
}

void CalcOdom::nav_offset_yaw_cb(const std_msgs::Float64::ConstPtr& offset_yaw_msg)
{
    offset_yaw_ = offset_yaw_msg->data;
}

void CalcOdom::nav_start_cb(const std_msgs::String::ConstPtr& nav_start_msg)
{
    // if(nav_start_msg->data == "start")
    // {
    //     start_flag_ = true;
    //     ROS_WARN("start_flag is true ! clear x,y,th !");
    // }

}

void CalcOdom::recv_vel(const can_msgs::Frame::ConstPtr& can_msg) {
    if((can_msg->id & 0x180) == 0x180)
    {
        uint16_t left_v,right_v;
        left_v  = (can_msg->data[1]<<8) | can_msg->data[0];
        right_v = (can_msg->data[3]<<8) | can_msg->data[2];
        if(left_v > 32768)  
        {
            left_v = -left_v;
            left_rpm_fd = -static_cast<float>(left_v)/10;  //收到的转速是0.1rpm为单位的
            // std::cout <<"---> [Feedback] left_rpm: " << left_rpm_fd ;
        }else {
            left_rpm_fd = static_cast<float>(left_v)/10;
            // std::cout <<"---> [Feedback] left_rpm: " << left_rpm_fd;
        }
        if(right_v > 32768) 
        {
            right_v = -right_v;
            right_rpm_fd = -static_cast<float>(right_v)/10;
            // std::cout <<" --- right_rpm: " << right_rpm_fd << std::endl;
        }else {
            right_rpm_fd = static_cast<float>(right_v)/10;
            // std::cout <<" --- right_rpm: " << right_rpm_fd << std::endl;
        }
        std_msgs::Float32MultiArray rpm_fd_msg;
        rpm_fd_msg.data.clear();
        rpm_fd_msg.data.push_back(left_rpm_fd);
        rpm_fd_msg.data.push_back(right_rpm_fd);
        // real_rpm_feedback_.publish(rpm_fd_msg);
    }
}

// 如果是前进的话，左右两轮应均为正速度
void CalcOdom::update_bot_odom()
{
    double linear_speed, angular_speed;
    double left_speed, right_speed;
    left_speed = convertRpmToMs(8.0,left_rpm_fd);
    right_speed = convertRpmToMs(8.0,right_rpm_fd);
    // ROS_INFO_STREAM_THROTTLE(0.5,"l,s: "<<left_speed<<"  "<< right_speed);
    right_speed = -right_speed;
    now_ = ros::Time::now();
    if (start_flag_) {
        accumulation_x_ = accumulation_y_ = accumulation_th_ = 0.0;
        last_time_ = now_;
        start_flag_ = false;
        // ROS_WARN("start_flag is true ! clear x,y,th !  --- 2");
        return;
    }
    delta_time_ = (now_ - last_time_).toSec();
    if (delta_time_ >= (0.5 / control_rate_)) {
        linear_speed  = (right_speed + left_speed) / 2.0;   // 计算线速度
        angular_speed = (right_speed - left_speed) / wheel_distance_;   // 计算角速度
        double delta_theta = (delta_time_ * angular_speed);
        double v_theta = delta_theta / delta_time_;
        
        double delta_dis = delta_time_ * linear_speed;
        double v_dis = delta_dis / delta_time_;
        double delta_x, delta_y;
        if (delta_theta == 0) {
            delta_x = delta_dis;
            delta_y = 0.0;
        } else {
            // delta_x = delta_dis * (sin(delta_theta) / delta_theta);
            // delta_y = delta_dis * ((1 - cos(delta_theta)) / delta_theta);
            delta_x = delta_dis * cos(delta_theta);
            delta_y = delta_dis * sin(delta_theta);
        }
        // accumulation_x_ += (cos(accumulation_th_) * delta_x - sin(accumulation_th_) * delta_y);
        // accumulation_y_ += (sin(accumulation_th_) * delta_x + cos(accumulation_th_) * delta_y);
        accumulation_x_ += (cos(accumulation_th_) * delta_x - sin(accumulation_th_) * delta_y);
        accumulation_y_ += (sin(accumulation_th_) * delta_x + cos(accumulation_th_) * delta_y);
        accumulation_th_ += delta_theta;
        if (accumulation_th_ <= -M_PI)    accumulation_th_ += 2 * M_PI;
        else if(accumulation_th_ >= M_PI)  accumulation_th_ -= 2 * M_PI;

        // std::cout <<"Calc Odom:";
        // std::cout <<"  [L_V]: " <<std::fixed <<std::setprecision(2)<< left_speed*100 ;
        // std::cout <<"  [L_RPM]: " <<std::fixed <<std::setprecision(2)<< left_rpm_fd*100 ;
        // std::cout <<"  [R_V]: " <<std::fixed <<std::setprecision(2)<< right_speed*100 ;
        // std::cout <<"  [R_RPM]: " <<std::fixed <<std::setprecision(2)<< right_rpm_fd*100 << std::endl ;
        geometry_msgs::Twist cmd_vel_fb;
        cmd_vel_fb.angular.z = angular_speed;
        cmd_vel_fb.linear.x = linear_speed;
        // real_vel_feedback_.publish(cmd_vel_fb);
          
        tf2::Quaternion q;
        q.setRPY(0, 0, accumulation_th_);

        if (true){
            transformStamped_.header.stamp = ros::Time::now();
            transformStamped_.header.frame_id = odom_frame_;
            transformStamped_.child_frame_id = base_frame_;
            transformStamped_.transform.translation.x = accumulation_x_;
            transformStamped_.transform.translation.y = accumulation_y_;
            transformStamped_.transform.translation.z = 0.0;

            transformStamped_.transform.rotation.x = q.x();
            transformStamped_.transform.rotation.y = q.y();
            transformStamped_.transform.rotation.z = q.z();
            transformStamped_.transform.rotation.w = q.w();

            // br_.sendTransform(transformStamped_);
        }

        // odom_.header.frame_id = odom_frame_;
        // odom_.child_frame_id = base_frame_;
        // odom_.header.stamp = now_;
        // odom_.pose.pose.position.x = accumulation_x_;
        // odom_.pose.pose.position.y = accumulation_y_;
        // odom_.pose.pose.position.z = 0;
        // odom_.pose.pose.orientation.x = q.getX();
        // odom_.pose.pose.orientation.y = q.getY();
        // odom_.pose.pose.orientation.z = q.getZ();
        // odom_.pose.pose.orientation.w = q.getW();
        // odom_.twist.twist.linear.x = v_dis;
        // odom_.twist.twist.linear.y = 0;
        // odom_.twist.twist.angular.z = v_theta;
        nav_msgs::Odometry odom_;
        // 设置header信息
        odom_.header.stamp = ros::Time::now();
        odom_.header.frame_id = received_odom.header.frame_id;
        odom_.pose = received_odom.pose;
        odom_.twist.twist.linear.x = v_dis;
        odom_.twist.twist.linear.y = 0;
        odom_.twist.twist.angular.z = v_theta;
        odom_.child_frame_id = received_odom.child_frame_id;

        // 发布新的odom消息
        combined_odom_pub.publish(odom_);

        // odom_pub_.publish(odom_);

        // 1. 使用里程计的yaw信息模拟imu的yaw
        // 2. 加上融合后的补偿值
        // 3. 即为真实坐标系下的yaw值，用于数字孪生
        tf2::Quaternion q_imu;
        double real_yaw_ = accumulation_th_+ offset_yaw_;
        if (real_yaw_ <= -M_PI)    real_yaw_ += 2 * M_PI;
        else if(real_yaw_ >= M_PI)  real_yaw_ -= 2 * M_PI;
        q_imu.setRPY(0, 0, real_yaw_);
        sensor_msgs::Imu imu_msg;
        imu_msg.orientation.x = q_imu.getX();
        imu_msg.orientation.y = q_imu.getY();
        imu_msg.orientation.z = q_imu.getZ();
        imu_msg.orientation.w = q_imu.getW();
        
        // odom_simu_imu_pub_.publish(imu_msg);
        
        last_time_ = now_;
        ROS_INFO_STREAM_THROTTLE(0.5,"[x]:" <<std::fixed <<std::setprecision(2)<< accumulation_x_*100 
                                  <<" [y]:" <<std::fixed <<std::setprecision(2)<< accumulation_y_*100  
                                  <<" [th]:" <<std::fixed <<std::setprecision(2)<< accumulation_th_*57.3
                                  <<" [r_th]:" <<std::fixed <<std::setprecision(2)<< real_yaw_*57.3);
    }
}

double CalcOdom::convertRpmToMs(float inch,float rpm) {
    double circumferenceInch = inch * 3.14159; // 计算轮胎的周长，单位为英寸
    double circumferenceMeter = circumferenceInch * 0.0254; // 将周长转换为米 0.638371088
    double metersPerMinute = circumferenceMeter * rpm; // 计算每分钟走过的米数 10rpm：6.38
    double metersPerSecond = metersPerMinute / 60; // 将每分钟转换为每秒钟 0.1m/s
    metersPerSecond = metersPerSecond * 100 / 105; // 实际补偿值
    return metersPerSecond;
}

void CalcOdom::run(){
    update_bot_odom();
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "calc_odom");
    CalcOdom calc_odom;
    calc_odom.init();
    ros::Rate loop_rate(100);
    while (ros::ok()) {
        calc_odom.run();
        ros::spinOnce();
        loop_rate.sleep();
    };
    return 0;
}