#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32.h>
#include <fstream>
#include <vector>
#include <queue>

// 全局变量
ros::Publisher motor_pub_r;
ros::Publisher motor_pub_l;
std::queue<std::vector<float>> data_queue;  // 用于存储从CSV文件中读取的数据
bool is_playing = false;  // 标记是否正在播放数据

void loadCSV(const std::string& file_path) {
  std::ifstream file(file_path);
  if (!file.is_open()) {
    ROS_ERROR("Failed to open CSV file: %s", file_path.c_str());
    return;
  }

  std::string line;
  bool is_first_line = true;  // Skip the first line (header)
  while (std::getline(file, line)) {
    if (is_first_line) {
      is_first_line = false;  // Skip the header
      continue;
    }

    std::vector<float> row;
    std::stringstream ss(line);
    std::string cell;

    // Skip the first column (time)
    std::getline(ss, cell, ',');

    // Read the remaining 16 columns
    while (std::getline(ss, cell, ',')) {
      row.push_back(std::stof(cell));
    }

    // Store the row in the queue
    data_queue.push(row);
  }

  file.close();
  ROS_INFO("CSV file loaded with %lu rows.", data_queue.size());
}

// 回调函数，用于处理 /motor_sim_action 的消息
void actionCallback(const std_msgs::Int32::ConstPtr& msg) {
  if (msg->data == 1) {
    loadCSV("/root/csv/a.csv");
  }
  else if (msg->data == 11) {
    loadCSV("/root/csv/salute.csv");
  }
  else if (msg->data == 12) {
    loadCSV("/root/csv/pointTo.csv");
  }
  else if (msg->data == 13) {
    loadCSV("/root/csv/waveHand.csv");
  }
  else if (msg->data == 14) {
    loadCSV("/root/csv/shakeHand.csv");
  }  
   else if (msg->data == 2) {
    // 开始播放数据
    is_playing = true;
    ROS_INFO("Start playing data.");
  }
}

// 定时器回调函数，用于发布数据
void publishData(const ros::TimerEvent&) {
  if (!is_playing || data_queue.empty()) return;

  // 从队列中取出一个元素
  std::vector<float> data = data_queue.front();
  data_queue.pop();

  // 拆分数据为左臂和右臂
  std::vector<float> left_data(data.begin(), data.begin() + 7);
  std::vector<float> right_data(data.begin() + 7, data.end());

  // 创建消息并发布
  std_msgs::Float32MultiArray msg_l;
  std_msgs::Float32MultiArray msg_r;
  msg_l.data = left_data;
  msg_r.data = right_data;

  motor_pub_l.publish(msg_l);
  motor_pub_r.publish(msg_r);

  // 如果队列为空，停止播放
  if (data_queue.empty()) {
    is_playing = false;
    ROS_INFO("All data published, stopping playback.");
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "upper_limb_sim");
  ros::NodeHandle nh;

  // 初始化发布者和订阅者
  motor_pub_r = nh.advertise<std_msgs::Float32MultiArray>("/motor_command_r", 1);
  motor_pub_l = nh.advertise<std_msgs::Float32MultiArray>("/motor_command_l", 1);
  ros::Subscriber action_sub = nh.subscribe("/motor_sim_action", 1, actionCallback);

  // 创建定时器，周期性调用 publishData，每次间隔 0.008 秒
  ros::Timer timer = nh.createTimer(ros::Duration(0.008), publishData);

  // 进入 ROS 循环，等待回调
  ros::spin();

  ROS_INFO("Program exiting.");
  return 0;
}