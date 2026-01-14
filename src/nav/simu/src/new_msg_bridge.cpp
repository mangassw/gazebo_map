
#include "new_msg_bridge.h"

ros::Publisher *pubSpeedPointer = NULL;

void speedHandler(const geometry_msgs::TwistStamped::ConstPtr& speedIn)
{
  float lwheel_control, rwheel_control;
  lwheel_control = (speedIn->twist.linear.x - 0.5 * speedIn->twist.angular.z * WheelSeparation) / WheelRadius;
  rwheel_control = (speedIn->twist.linear.x + 0.5 * speedIn->twist.angular.z * WheelSeparation) / WheelRadius;
  geometry_msgs::Twist speedOut;
  speedOut.linear.x = speedIn->twist.linear.x;
  speedOut.angular.z = speedIn->twist.angular.z;
  pubSpeedPointer->publish(speedOut);
}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "new_msg_bridge");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate = ros::NodeHandle("~");

  ros::Subscriber subSpeed = nh.subscribe<geometry_msgs::TwistStamped>(SubSpeedTopic, 5, speedHandler);
  ros::Publisher pubSpeed = nh.advertise<geometry_msgs::Twist>(PubSpeedTopic, 5);
  pubSpeedPointer = &pubSpeed;


  ros::Rate rate(200);
  while (ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
