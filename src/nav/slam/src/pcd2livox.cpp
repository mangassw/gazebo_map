#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <livox_ros_driver2/CustomMsg.h> // Livox 自定义消息头文件
 
 
void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& pc2_msg, ros::Publisher& publisher) {
    livox_ros_driver2::CustomMsg livox_msg;
    float frame_duration = 100000.0; // 1秒的帧时间，单位微秒
    int num_points = pc2_msg->width * pc2_msg->height;
    float point_interval = frame_duration / num_points;
 
    // 设置消息头
    livox_msg.header = pc2_msg->header; // 直接复制 ROS 消息头
    
    // 计算点云的数量
    livox_msg.point_num = pc2_msg->width * pc2_msg->height; 
    livox_msg.timebase = pc2_msg->header.stamp.toNSec(); 
    // 假设只使用一个 Lidar ID（可以根据需要更改）
    livox_msg.lidar_id = 1; // 设置 Lidar ID
    memset(livox_msg.rsvd.data(), 0, sizeof(livox_msg.rsvd)); // 清空保留字段
 
    // 分配内存以存储点云数据
    livox_msg.points.resize(livox_msg.point_num);
    
    int x_offset = -1, y_offset = -1, z_offset = -1, intensity_offset = -1;
    int tag_offset = -1, line_offset = -1;
 
    for (const auto& field : pc2_msg->fields) {
        if (field.name == "x") {
            x_offset = field.offset;
        } else if (field.name == "y") {
            y_offset = field.offset;;
        } else if (field.name == "z") {
            z_offset = field.offset;;
        } else if (field.name == "intensity") {
            intensity_offset = field.offset;;
        }
        else if (field.name == "tag") {
            tag_offset = field.offset;;
        }
        else if (field.name == "line") {
            line_offset = field.offset;;
        }
    }
 
    if (x_offset == -1 || y_offset == -1 || z_offset == -1 || intensity_offset == -1 || tag_offset == -1 || line_offset == -1) {
        ROS_ERROR("PointCloud2 message missing required fields.");
        return;
    }
 
 
    // 获取 PointCloud2 数据指针
    const float *x_data = nullptr;
    const float *y_data = nullptr;
    const float *z_data = nullptr;
    const float *intensity_data = nullptr;
    const uint8_t *tag_data = nullptr;
    const uint8_t *line_data = nullptr;
 
    // 填充 Livox 消息点云数据
    for (size_t i = 0; i < livox_msg.point_num; ++i) {
        size_t offset = i * pc2_msg->point_step;
        x_data = reinterpret_cast<const float*>(&pc2_msg->data[offset + x_offset]);
        y_data = reinterpret_cast<const float*>(&pc2_msg->data[offset + y_offset]);
        z_data = reinterpret_cast<const float*>(&pc2_msg->data[offset + z_offset]);
        intensity_data = reinterpret_cast<const float*>(&pc2_msg->data[offset + intensity_offset]);
        tag_data = reinterpret_cast<const uint8_t*>(&pc2_msg->data[offset + tag_offset]);
        line_data = reinterpret_cast<const uint8_t*>(&pc2_msg->data[offset + line_offset]);
        livox_msg.points[i].x = x_data[0];
        livox_msg.points[i].y = y_data[0];
        livox_msg.points[i].z = z_data[0];
        float intensity = intensity_data[0];  // 假设 intensity 数据已经提取
        uint8_t reflectivity = static_cast<uint8_t>(std::min(std::max(intensity * 255.0f, 0.0f), 255.0f));
        livox_msg.points[i].reflectivity = reflectivity;  // 将 reflectivity 设置为 uint8_t 类型的值
        livox_msg.points[i].tag = tag_data[0]; // 根据需要设置
        livox_msg.points[i].line = line_data[0]; // 根据需要设置
        livox_msg.points[i].offset_time = i * point_interval; // 根据需要设置
    }
 
    // 发布 Livox 消息
    publisher.publish(livox_msg);
}
 
int main(int argc, char** argv) {
    ros::init(argc, argv, "pointcloud_to_livox");
    ROS_INFO("pcd to livox ==> /livox/lidar_raw to /livox/lidar");
    ros::NodeHandle nh;
 
    // 创建发布器
    ros::Publisher livox_publisher = nh.advertise<livox_ros_driver2::CustomMsg>("/livox/lidar_custom", 10);
    
    // 创建订阅器
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/livox/lidar", 1,
        boost::bind(pointCloudCallback, _1, livox_publisher));
    
    ros::spin(); // 进入循环，处理回调
    return 0;
}