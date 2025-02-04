#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <vector>

std::vector<float> arr_r(9, 0.0);  // 右侧电机的位置数据
std::vector<float> arr_l(9, 0.0);  // 左侧电机的位置数据



ros::Publisher motor_pub_r;
ros::Publisher motor_pub_l;

ros::Subscriber motor_sub_r;
ros::Subscriber motor_sub_l;

static std_msgs::Float32MultiArray msg_l;
static std_msgs::Float32MultiArray msg_r;
int cnt = 0;
// 回调函数，用于处理来自左侧电机的消息
void motorStatusCallbackL(const std_msgs::Float32MultiArray::ConstPtr& msg) {
    if(cnt>=1500)return;
  // 提取位置数据（数组前9个元素）
  for (int i = 0; i < 9; i++) {
    arr_l[i] = msg->data[i * 3];  // 将前9个位置数据存储到 arr_l 中
    msg_l.data[i] = arr_l[i];
    // printf("%f,%f \n", msg->data[i * 3], arr_l[i]);
  }

}

// 回调函数，用于处理来自右侧电机的消息
void motorStatusCallbackR(const std_msgs::Float32MultiArray::ConstPtr& msg) {
     if(cnt>=1500)return;
  // 提取位置数据（数组前9个元素）
  for (int i = 0; i < 9; i++) {
    arr_r[i] = msg->data[i * 3];  // 将前9个位置数据存储到 arr_r 中
    msg_r.data[i] = arr_r[i];
  }
}

// 定时器回调函数，用于发布数据
void publishData(const ros::TimerEvent&) {
  cnt++;
  if (cnt < 1500) return;
  if (cnt > 8500) return;

  // 更新数据
  static float increment = 250.0;
  for (int j = 0; j < 9; j++) {
    msg_l.data[j] -= increment;
    msg_r.data[j] += increment;
  }

  // 发布数据
  motor_pub_r.publish(msg_r);
  motor_pub_l.publish(msg_l);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "upper_limb_sim");
  ros::NodeHandle nh;
  motor_pub_r = nh.advertise<std_msgs::Float32MultiArray>("/motor_command_r", 1);
  motor_pub_l = nh.advertise<std_msgs::Float32MultiArray>("/motor_command_l", 1);
  motor_sub_r = nh.subscribe("/motor_feedback_r", 1, motorStatusCallbackR);
  motor_sub_l = nh.subscribe("/motor_feedback_l", 1, motorStatusCallbackL);
  msg_l.data.resize(9, 0.0);  // 初始化数据为长度9的全0数组
  msg_r.data.resize(9, 0.0);
  ros::Rate loop_rate(125);

  // 创建定时器，周期性调用 publishData，每次间隔 0.008 秒
  ros::Timer timer = nh.createTimer(ros::Duration(0.008), publishData);

  // 进入 ROS 循环，等待回调
  ros::spin();

  ROS_INFO("Publishing complete, program exiting.");
  return 0;
}
