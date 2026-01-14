//
// Created by tony on 2022/9/22.
//

#include "msg_bridge.h"

int main (int argc, char** argv) {
  ros::init(argc, argv, jz::simu::NODE_NAME);
  jz::simu::JzBridge _bridge_node;
  _bridge_node.init();

  ros::spin();
  return 0;
}
