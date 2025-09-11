#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include "mbf_msgs/MoveBaseAction.h"


namespace xj_robot
{
	class MbfBridge
	{
		public:
			MbfBridge()
			{
				//we'll provide a mechanism for some people to send goals as PoseStamped messages over a topic
				//they won't get any useful information back about its status, but this is useful for tools
				//like nav_view and rviz
				ros::NodeHandle simple_nh("move_base_simple");
				goal_sub_ = simple_nh.subscribe<geometry_msgs::PoseStamped>("goal", 1, [this](auto& goal){ goalCB(goal); });
				// goal_sub_ = simple_nh.subscribe<geometry_msgs::PoseStamped>("goal", 1, boost::bind(&MbfBridge::goalCB, this, _1));

				ros::NodeHandle action_nh("move_base_flex/move_base");
				action_goal_pub_ = action_nh.advertise<mbf_msgs::MoveBaseActionGoal>("goal", 1);
			}
			~MbfBridge() = default;


			private:
				ros::Subscriber goal_sub_;
				ros::Publisher action_goal_pub_;

				void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal);
		};
		

		void MbfBridge::goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal){
			ROS_DEBUG_NAMED("move_base","In ROS goal callback, wrapping the PoseStamped in the action message and re-sending to the server.");
			mbf_msgs::MoveBaseActionGoal action_goal;
			action_goal.header.stamp = ros::Time::now();
			action_goal.goal.target_pose = *goal;

			action_goal_pub_.publish(action_goal);
		}
		
} // namespace xj_robot

int main(int argc, char** argv) {
	ros::init(argc, argv, "xj_robot_mbf_bridge");

	xj_robot::MbfBridge mbf_bridge_;
	ros::spin();

	return 0;
}