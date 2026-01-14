
#include <csignal>
#include "xj_robot_pnc/nav_to_goal.h"

std::shared_ptr<nav_to_goal::Task> task_ptr;

void sigintHandler(int sig) {
  if (task_ptr) {
    task_ptr->stop();
    // task_ptr.reset();
  }

  ROS_INFO("xj_robot task_ptr shutting down!");
  ros::shutdown();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, nav_to_goal::NODE_NAME);
  task_ptr = std::make_shared<nav_to_goal::Task>();
  signal(SIGINT, sigintHandler);
  task_ptr->init();
  task_ptr->run();
  return 0;
}
