#include "xj_robot_pnc/run_tasks.h"


namespace xj_robot {

uint8_t* Task::cost_translation_ = nullptr;

Task::Task():
    tf_(new tf2_ros::Buffer()),
    tfl_(new tf2_ros::TransformListener(*tf_)),
    goto_ctrl_(new GotoCtrl("move_base_flex/move_base")),
    exe_ctrl_(new ExeCtrl("move_base_flex/exe_path")),
    cur_state_(StateValue::Idle),
    cur_index_(0),
    nav_state_(NavState::Goto)
{
  if (!cost_translation_) {
    cost_translation_ = new uint8_t[101];
    for (int i = 0; i < 101; ++i) {
      cost_translation_[i] = static_cast<uint8_t>(i * 254 / 100);
    }
  }
}
Task::~Task()
{

}

void Task::init()
{
  ros::NodeHandle nh;
  vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  visual_path_pub_ = nh.advertise<nav_msgs::Path>("record_path_visual",1);
  keyboard_sub_ = nh.subscribe<std_msgs::Int32>("keyboard_input", 1, &Task::keyboardInputCB,this);
  joy_sub_ = nh.subscribe<sensor_msgs::Joy>("/joy", 1, &Task::joyCtrlCB,this);

  costmap_sub_ = nh.subscribe(COSTMAP, 5, &Task::costmapCB, this);
  costmap_update_sub_ = nh.subscribe(COSTMAP_UPDATE, 5, &Task::costmapUpdateCB, this);

  nh.param<std::string>("file_path", FILE_PATH, "/home/wu/xj_robot/PATH/path_1");


  ROS_ERROR_COND(!goto_ctrl_->waitForServer(ros::Duration(5.0)), "move base action not online!");
  ROS_ERROR_COND(!exe_ctrl_->waitForServer(ros::Duration(5.0)), "exe path action not online!");
  ROS_INFO("Task Manager Initialized!");
}

void Task::run()
{
  ros::Rate r(ros::Duration(0.05));
  while (ros::ok())
  {
    ros::spinOnce();
    switch (cur_state_)
    {
      case StateValue::Idle:
        ROS_INFO_THROTTLE(1.0,"Idle");
        read_once_ = true;
        break;
      case StateValue::RecordStart:
        ROS_INFO_ONCE("record start...");
        recordPath();
        break;
      case StateValue::RecordStop:
        ROS_INFO_ONCE("record stop.");
        break;
      case StateValue::Pause:
        break;
      case StateValue::Navigate:
        ROS_INFO_THROTTLE(1.0,"Navigate.");
        if(read_once_){
          read_once_ = false;
          if (!readPathFile(FILE_PATH)) {
            ROS_ERROR("Read path in file failed.");
            return ;
          }
        }
        navigate();
        
        break;
      case StateValue::Wait:
        break;  
      case StateValue::Charge:
        break;
      case StateValue::Sleep:
        break;
      default:
        break;
    }
    r.sleep();
  }
}

void Task::joyCtrlCB(const sensor_msgs::Joy::ConstPtr& joy)
{
  
  // float left_rocker   = joy->axes[1];  // 0-axis[1]-->左摇杆上下移动-->控制轮毂电机前进后退
	// float right_rocker  = joy->axes[2];  // 1-axis[2]-->右摇杆左右移动-->控制轮毂电机左右转
	float A_button = joy->buttons[0];       // 3-buttons[0]-->A键-->循迹以及一级爬升到位
	// float B_button = joy->buttons[3];       // 4-buttons[1]-->B键-->一级爬升到位至二级爬升到位
	float X_button = joy->buttons[3];       // 5-buttons[3]-->X键-->任意时刻退出至原点
	// float Y_button = joy->buttons[5];       // 6-buttons[4]-->Y键-->任意时刻暂停后恢复运动，且若已经二级爬升到位则一键撤离至原点
	// float left_bumper = joy->axes[6];    // 7-axis[5]-->左上扳机键-->按住将控制后电机作为驱动轮，默认为前电机驱动
	// float M_button1 = joy->buttons[7];      // 8-buttons[5]-->中间，从左往右第一个View按钮-->测试用
	// float M_button3 = joy->buttons[8];      // 10-buttons[7]-->中间，从左往右第三个菜单按钮-->测试用
	float L_button = joy->buttons[6];      // 11-buttons[8]-->左上的按键-->测试用
	float R_button = joy->buttons[7];      // 12-buttons[9]-->右上的按键-->测试用
	// float dir_down = joy->axes[11];      // 12-方向键下键 ---> 用于齿轮电机的归零
	// float right_bumper = joy->axes[12];  // 13-axis[4]-->右上扳机键
	// float left_roc_mid  = joy->buttons[13]; // 14-button[13]-->左摇杆中键
	// float right_roc_mid = joy->buttons[14]; // 15-button[14]-->右摇杆中键
	// ROS_INFO_STREAM("joy_ctrl_cb: "<< left_rocker << " " << right_rocker << " " << A_button << " " << B_button << " " << X_button << " " << Y_button << " " << left_bumper << " " << M_button1 << " " << M_button3 << " " << L_button << " " << R_button << " " << dir_down << " " << right_bumper << " " << left_roc_mid << " " << right_roc_mid);	
  // 避免按键弹起带来影响
	static float last_button[4];
	if(last_button[0] != 0 || last_button[1] != 0 || last_button[2] != 0 || last_button[3] != 0) { 
		last_button[0] = 0;  last_button[1] = 0;  last_button[2] = 0;  last_button[3] = 0;
		return ; 
	}  
	// last_button[0] = A_button*1000+B_button*100+X_button*10+Y_button;
	// last_button[1] = M_button1*10+M_button3;
	last_button[2] = L_button*10+R_button;
	// last_button[2] = L_button*10;
	// last_button[3] = left_roc_mid*10+right_roc_mid;
  // ----------OVER------------------
  std_msgs::Int32 input;
  if(A_button || X_button || L_button || R_button ){
	if(L_button) {ROS_INFO("Received L_button");  input.data = 1; }
	if(R_button) {ROS_INFO("Received R_button");  input.data = 2; }
	if(A_button) {ROS_INFO("Received A_button");  input.data = 0; }
	if(X_button) {ROS_INFO("Received X_button");  input.data = 3; }
  if((cur_state_ == StateValue::RecordStop)&&(input.data == 1))
  {
    key_point_record = true;
    return;
  }

  if(cur_state_ == StateValue::RecordStop)
  {
    if (input.data == 2) {
      auto t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
      std::stringstream ss;
      ss << "path_" << std::put_time(std::localtime(&t), "%F %T");
      std::string dir(DEFAULT_DIR);
      std::string write_path = dir + "/" + ss.str();
      if (!writePathFile(write_path)) {
        ROS_ERROR("Write file failed.");
      } else {
        ROS_INFO("Keep teach path, save successful.");
      }
    } else {
      ROS_WARN("Do not keep teach path, discard recorded data.");
    }
    record_path_.clear();
    cur_state_ = StateValue::Idle;
    return;
  }
  switch (input.data)
  {
    case 0:
      cur_state_ = StateValue::Idle;
      break;
    case 1:
      cur_state_ = StateValue::RecordStart;
      break;
    case 2:
      cur_state_ = StateValue::RecordStop;
      break;
    case 3:
      cur_state_ = StateValue::Navigate;
      break;
  }


  }


}

auto Task::getRobotPose() -> std::optional<geometry_msgs::Pose>
{
  geometry_msgs::Pose ret_pose;
  geometry_msgs::TransformStamped transformStamped;
  try
  {
    transformStamped = tf_->lookupTransform("map", "base_link", ros::Time(0), ros::Duration(0.1));
  } catch (tf2::TransformException &ex)
  {
    ROS_WARN("%s", ex.what());
    return std::nullopt;
  }

  ret_pose.position.x = transformStamped.transform.translation.x;
  ret_pose.position.y = transformStamped.transform.translation.y;
  ret_pose.orientation = transformStamped.transform.rotation;
  ROS_DEBUG_STREAM("getRobotPose: " << ret_pose);
  return std::make_optional(ret_pose);

}


void Task::recordPath()
{
  auto rec_pose = getRobotPose();
  if(!rec_pose) return;

  if(record_path_.empty()) {
    record_path_.push_back(rec_pose.value());
    return;
  }

  auto len = std::hypot(record_path_.back().position.x - rec_pose->position.x,
                record_path_.back().position.y - rec_pose->position.y);
  auto ang = std::abs(tf2::getYaw(record_path_.back().orientation) - tf2::getYaw(rec_pose->orientation));
  if(len < REC_PATH_LEN_DENS && ang < REC_PATH_ANG_DENS) return;  
  record_path_.push_back(rec_pose.value());

  if(key_point_record)  {
    // record_path_.push_back(9999999999);
    key_point_record = false;
  }
}

auto Task::writePathFile(std::string& path_file) -> bool
{
  std::ofstream ofs(path_file.c_str());
  
  if (!ofs.is_open()) {
    ROS_ERROR("Open file %s failed!", path_file.c_str());
    return false;
  }
  int lines = 0;
  for (auto const& p : record_path_) {
    ++lines;
    ofs << std::to_string(p.position.x) << " " << std::to_string(p.position.y) << " " << std::to_string(tf2::getYaw(p.orientation)) << "\n";
  }

  ROS_INFO("Write path success. (%d poses)", lines);
  return true;
}

// 打开路径文件并读取其中的路径点
auto Task::readPathFile(std::string& path_file) -> bool
{
  std::ifstream ifs(path_file);
  if (!ifs.is_open()) {
    ROS_ERROR("Failed to open file for reading: %s", path_file.c_str());
    return false;
  }

  cur_route_.header.frame_id = "map";
  cur_route_.header.stamp = ros::Time::now();
  cur_route_.poses.clear();

  int lines = 0;
  std::string contend, temp;
  std::vector<std::string> temps;
  while (getline(ifs, contend)) {
    ++lines;
    temps.clear();
    temp.clear();

    for (auto const& c : contend) {
      if (c != ' ') {
        temp += c;
      } else {
        temps.emplace_back(temp);
        temp.clear();
      }
    }

    if (!temp.empty()) temps.emplace_back(temp);
    if (temps.size() != 3) {
      ROS_ERROR("%d line in %s file not correct!", lines, path_file.c_str());
      continue;
    }

    geometry_msgs::PoseStamped p;
    p.header.frame_id = "map";
    p.header.stamp = ros::Time::now();
    p.pose.position.x = std::stod(temps[0]);
    p.pose.position.y = std::stod(temps[1]);
    p.pose.orientation.z = std::sin(std::stod(temps[2]) / 2.0);
    p.pose.orientation.w = std::cos(std::stod(temps[2]) / 2.0);
    cur_route_.poses.emplace_back(p);
  }

  ROS_INFO("Read path success. (%d poses)", lines);
  ifs.close();
  return !cur_route_.poses.empty();
}


void Task::pubZeroVel()
{
  geometry_msgs::Twist zero_vel;
  zero_vel.angular.x = 0;
  zero_vel.angular.y = 0;
  zero_vel.angular.z = 0;
  zero_vel.linear.x = 0;
  zero_vel.linear.y = 0;
  zero_vel.linear.z = 0;
  vel_pub_.publish(zero_vel);
}

void Task::keyboardInputCB(const std_msgs::Int32::ConstPtr& input)
{
  // 双击2 保存路径
  if(cur_state_ == StateValue::RecordStop)
  {
    if (input->data == 2) {
      auto t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
      std::stringstream ss;
      ss << "path_" << std::put_time(std::localtime(&t), "%F %T");
      std::string dir(DEFAULT_DIR);
      std::string write_path = dir + "/" + ss.str();
      if (!writePathFile(write_path)) {
        ROS_ERROR("Write file failed.");
      } else {
        ROS_INFO("Keep teach path, save successful.");
      }
    } else {
      ROS_WARN("Do not keep teach path, discard recorded data.");
    }
    record_path_.clear();
    cur_state_ = StateValue::Idle;
    return;
  }
  switch (input->data)
  {
    case 0:
      cur_state_ = StateValue::Idle;
      break;
    case 1:
      cur_state_ = StateValue::RecordStart;
      break;
    case 2:
      cur_state_ = StateValue::RecordStop;
      break;
    case 3:
      cur_state_ = StateValue::Navigate;
      break;
  }
}

void Task::visualizePath(std::vector<geometry_msgs::Pose> const& record_path,ros::Publisher const& path_pub)
{
  nav_msgs::Path path;
  path.header.stamp = ros::Time::now();
  path.header.frame_id = "map";
  for (auto const& p : record_path) {
    geometry_msgs::PoseStamped ps;
    ps.header.frame_id = "map";
    ps.pose = p;
    path.poses.emplace_back(ps);
  }

  path_pub.publish(path);
}

void Task::stop()
{
  cancelNav();
  // pub zero velocity for 1s
  for (auto i = 0; i < 10; ++i) {
    pubZeroVel();
    ros::Duration(0.1).sleep();
  }
}
void Task::cancelNav() {
  if (!goto_ctrl_->getState().isDone()) {
    ROS_INFO("Cancel current goto goal");
    goto_ctrl_->cancelGoal();
  }

  if (!exe_ctrl_->getState().isDone()) {
    ROS_INFO("Cancel current exe path goal");
    exe_ctrl_->cancelGoal();
  }
}

// 导航
void Task::navigate()
{
  ROS_INFO_THROTTLE(1.0,"nav_state_: %d",static_cast<int>(nav_state_));
  switch (nav_state_)
  {
    case NavState::Exe:{
      if(exe_ctrl_->getState() == goalState::SUCCEEDED){
        ROS_INFO("Task success!");
        reset();
        return ;
      }
      if(exe_ctrl_->getState().isDone()){
        ROS_WARN("Exe failed, try to send goto");
        for(auto i = cur_index_; i < cur_route_.poses.size(); i++){
          if(isFree(cur_route_.poses.at(i))) {
            cur_index_ = i;
            break;
          }
        }
        // 索引安全
        cur_index_ == std::min(cur_index_,cur_route_.poses.size()-1); 
        nav_state_ = NavState::Goto;
        ROS_INFO_STREAM("Send goto target:"<<cur_index_);
        sendGoto(cur_index_);
      }
      cur_index_ = nearestIndex(cur_route_, cur_index_, 0, 20).first;
      sendExeCtrl(cur_index_);

      
      for (auto i = 0; i < PATH_SAFE_DIS_NUM; ++i) {
        if (cur_index_ + i >= cur_route_.poses.size()) break;
        if (isDanger(cur_route_.poses.at(cur_index_ + i))) {
          obs_index_ = cur_index_ + i;
          ROS_WARN("Obs found at %.3f away",
                   poseDistance(cur_route_.poses[cur_index_].pose, cur_route_.poses[obs_index_].pose).first);
          stop();
          wait_cnt = 0;
          nav_state_ = NavState::Wait;
        }
      }
      break;
    }
    case NavState::Wait:{
      bool is_danger = false;
      for(auto i=0;i<10;i++){
        if (obs_index_ + i >= cur_route_.poses.size()) break;
        if (isDanger(cur_route_.poses.at(obs_index_ + i))) {
          is_danger = true;
          break;
        }
      }
      if(!is_danger) {
        ROS_INFO("Obs clear, keep moving");
        nav_state_ = NavState::Exe;
        sendExeCtrl(cur_index_);
        break;
      }
      if(++wait_cnt >= WAIT_COUNT){
        wait_cnt = 0;
        // 前向搜索
        for(auto i = cur_index_; i<cur_route_.poses.size(); i++){
          if(isFree(cur_route_.poses[i])){
            cur_index_ = i;
            break;
          }
        }
        cur_index_ = std::min(cur_index_,cur_route_.poses.size()-1);
        ROS_INFO("Wait overtime, start to avoid.");
        nav_state_ = NavState::Goto;
        sendGoto(cur_index_);
      }
      break;
    }
    case NavState::Goto:{
      if(goto_ctrl_->getState() == goalState::SUCCEEDED){
        ROS_INFO("Goto success, send exe.");
        nav_state_ = NavState::Exe;
        sendExeCtrl(cur_index_);
        break;
      }
      if(goto_ctrl_->getState().isDone() || !isFree(cur_route_.poses.at(cur_index_))){
        for(auto i = cur_index_ + 10; i < cur_route_.poses.size(); ++i){
          if(isFree(cur_route_.poses.at(i))){
            cur_index_ = i;
            break;
          }
        }
        if (cur_index_ >= cur_route_.poses.size()) {
          ROS_ERROR("Task failed!");
          reset();
          return;
        }
        ROS_WARN("Target no safe, update %lu", cur_index_);
        sendGoto(cur_index_);
      }
      break;
    }
    default:
        ROS_ERROR("Error RunStateValue!");
        break;
  }
  
}


void Task::reset()
{
  nav_state_ = NavState::Goto;
  cur_state_ = StateValue::Idle;
  cur_index_ = 0;
  cur_route_.poses.clear();
  stop();
}

auto Task::isFree(const geometry_msgs::PoseStamped& pose) const -> bool {
  auto cost = calcCost(pose);
  return cost < 66;
}

auto Task::isDanger(const geometry_msgs::PoseStamped& pose) const -> bool {
  auto cost = calcCost(pose);
  return cost >= 253;
}

auto Task::calcCost(const geometry_msgs::PoseStamped& pose) const -> uint8_t {
  if (!costmap_) {
    ROS_WARN("No costmap yet");
    return true;
  }

  uint32_t mx, my;
  if (!costmap_->worldToMap(pose.pose.position.x, pose.pose.position.y, mx, my)) {
    ROS_WARN("Can't find point mx, my in cost map");
    return 0;
  }

  // TODO 实时输出当前的costmap值
  return costmap_->getCost(mx, my);
}

// 距离计算
auto Task::poseDistance(geometry_msgs::Pose const& cur_pose, geometry_msgs::Pose const& last_pose) -> std::pair<double, double>{
  std::pair<double, double> ret;
  ret.first = std::hypot(cur_pose.position.x - last_pose.position.x, cur_pose.position.y - last_pose.position.y);
  ret.second = std::abs(tf2::getYaw(cur_pose.orientation) - tf2::getYaw(last_pose.orientation));

  return ret;
}

// 搜寻当前位置的最近索引点
std::pair<size_t, double> Task::nearestIndex(nav_msgs::Path const& path, size_t const& index, int lb, int rb)
{
  size_t idx;
  double dis = std::numeric_limits<double>::max();
  auto cur_pose = getRobotPose();
  if( !cur_pose ) return std::make_pair(0, 0);
  // 确定区间
  size_t lbb = std::max(static_cast<int>(index) - lb,0);
  size_t rbb = std::min(index + rb,path.poses.size());
  for(auto i= lbb; i < rbb; ++i ){
    auto err = poseDistance(cur_pose.value(), path.poses[i].pose);
    if(err.first < dis && err.second < 30 * DEG2RAD){
      dis = err.first;
      idx = i;
    }
  }
  return std::make_pair(idx, dis);
}

// 点到点（远）
auto Task::sendGoto(size_t const& index) -> bool {
  if (index == std::numeric_limits<size_t>::max() || index > cur_route_.poses.size()) {
    ROS_ERROR_STREAM("send_goto index error " << index << " / " << cur_route_.poses.size());
    return false;
  }

  mbf_msgs::MoveBaseGoal goal{};
  goal.target_pose = cur_route_.poses.at(index);
  ROS_INFO("send_goto goal");
  goto_ctrl_->sendGoal(goal,
                       boost::bind(&Task::gotoDone, this, _1, _2),
                       GotoCtrl::SimpleActiveCallback(),
                       GotoCtrl::SimpleFeedbackCallback());
  return true;
}
// 点到点结果检查
void Task::gotoDone(const actionlib::SimpleClientGoalState& state,
                             const mbf_msgs::MoveBaseResultConstPtr& result) {
  ROS_INFO("MoveBase got state [%s]", state.toString().c_str());

  if (!result) return;
  ROS_INFO("MoveBase got result [%d]", result->outcome);
//  send_exe(current_index_);
}


// 路径跟踪（近）
bool Task::sendExeCtrl(size_t const& index)
{
  // index校验
  if (index == std::numeric_limits<size_t>::max() || index > cur_route_.poses.size()) {
    ROS_ERROR_STREAM("send_exe index error " << index << " / " << cur_route_.poses.size());
    return false;
  }
  // path 
  mbf_msgs::ExePathGoal goal{};
  goal.path.header = cur_route_.header;
  goal.path.poses.assign(cur_route_.poses.begin()+index, cur_route_.poses.end());
  // send
  ROS_INFO("send_exe goal");
  exe_ctrl_->sendGoal(goal,
                      boost::bind(&Task::exeDone, this, _1, _2),
                      ExeCtrl::SimpleActiveCallback(),
                      ExeCtrl::SimpleFeedbackCallback());
  return true;
}
// 路径跟踪结果检查
void Task::exeDone(const actionlib::SimpleClientGoalState& state,const mbf_msgs::ExePathResultConstPtr& result) {
  ROS_INFO("ExePath got state [%s]", state.toString().c_str());

  if (!result) return;
  ROS_INFO("ExePath got result [%d]", result->outcome);
}

void Task::costmapCB(nav_msgs::OccupancyGrid::ConstPtr const& msg) {
  std::lock_guard<std::mutex> lock(map_update_mutex_);
  if (!costmap_) {
    ROS_WARN("Initiate new costmap");
    costmap_ = std::make_shared<costmap_2d::Costmap2D>(msg->info.width, msg->info.height, msg->info.resolution,
                                                       msg->info.origin.position.x, msg->info.origin.position.y);
  } else {
    ROS_WARN("Update costmap!");
    costmap_->resizeMap(msg->info.width, msg->info.height, msg->info.resolution,
                        msg->info.origin.position.x, msg->info.origin.position.y);
  }

  uint32_t x;
  for (uint32_t y = 0; y < msg->info.height; y++) {
    for (x = 0; x < msg->info.width; x++) {
      if (msg->data[y * msg->info.width + x] < 0) {
        costmap_->setCost(x, y, costmap_2d::NO_INFORMATION);
        continue;
      }
      costmap_->setCost(x, y, cost_translation_[msg->data[y * msg->info.width + x]]);
    }
  }
}

void Task::costmapUpdateCB(map_msgs::OccupancyGridUpdate::ConstPtr const& msg) {
  std::lock_guard<std::mutex> lock(map_update_mutex_);
  if (!costmap_) {
    ROS_WARN("Costmap not initiate yet");
    return;
  }

  if (msg->width * msg->height != msg->data.size()
      || msg->x + msg->width > costmap_->getSizeInCellsX()
      || msg->y + msg->height > costmap_->getSizeInCellsY()) {
    ROS_ERROR("Costmap update got invalid data set");
    return;
  }

  size_t index = 0;
  int x;
  for (auto y = msg->y; y < msg->y + msg->height; y++) {
    for (x = msg->x; x < msg->x + msg->width; x++) {
      if (msg->data[index] < 0) {
        costmap_->setCost(x, y, costmap_2d::NO_INFORMATION);
        index++;
        continue;
      }
      costmap_->setCost(x, y, cost_translation_[msg->data[index++]]);
    }
  }
}




}










